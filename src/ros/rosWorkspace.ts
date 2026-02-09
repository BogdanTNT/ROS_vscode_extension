import * as vscode from 'vscode';
import * as cp from 'child_process';
import * as path from 'path';
import * as fs from 'fs';
import { BuildStampManager, StampStorage } from './buildStampManager';
import { DependencyResolver } from './dependencyResolver';
import {
    BuildPolicy,
    BuildEvaluation,
} from './buildPolicy';

export interface RosEnvironmentInfo {
    distro: string;   // e.g. "humble", "noetic"
    version: number;   // 1 or 2
    workspacePath: string;
}

export interface RosPackageInfo {
    name: string;
    path: string;
    buildType: string; // ament_cmake | ament_python | catkin
    dependencies: string[];
}

export interface RosWorkspacePackageDetails {
    name: string;
    packagePath: string;
    launchFiles: string[];
    nodes: RosRunnableNodeInfo[];
    isPython: boolean;
}

export interface RosRunnableNodeInfo {
    name: string;
    sourcePath?: string;
}

export interface LaunchArgOption {
    name: string;
    defaultValue?: string;
}

export interface RosGraphEntityInfo {
    name: string;
    type: string;
}

export interface RosParameterInfo {
    name: string;
    node?: string;
}

export interface RosTopicPublishTemplateResult {
    success: boolean;
    topicName: string;
    topicType?: string;
    template?: string;
    error?: string;
}

export interface RosTopicPublishResult {
    success: boolean;
    topicName: string;
    topicType?: string;
    error?: string;
}

export interface RosNodeGraphInfo {
    publishers: string[];
    subscribers: string[];
    serviceServers: string[];
    serviceClients: string[];
    actionServers: string[];
    actionClients: string[];
}

/** Result from preLaunchBuildCheck. */
export interface PreLaunchBuildResult {
    /** What the launch flow should do next. */
    action: 'launch' | 'build-and-launch' | 'cancel';
    /** Packages that need building (only when action is 'build-and-launch'). */
    stalePackages?: string[];
}

export interface TrackedTerminalInfo {
    id: string;
    kind: 'integrated' | 'external';
    name: string;
    status: 'running' | 'closed';
    cmd: string;
    pid?: number;
    launchLabel?: string;
    launchPath?: string;
    createdAt: number;
    lastUsed: number;
    isPreferred?: boolean;
}

interface TrackedTerminal extends TrackedTerminalInfo {
    terminal?: vscode.Terminal;
    childProcess?: cp.ChildProcess;
    pidFile?: string;
    innerPid?: number;
    /** Set after a Ctrl+C has been sent so the next kill force-closes. */
    ctrlCSent?: boolean;
}

/**
 * Central helper that interacts with the ROS CLI tools installed on the host.
 * All shell calls go through here so views stay decoupled from the system.
 */
export class RosWorkspace {
    private _env: RosEnvironmentInfo | undefined;
    private _terminal: vscode.Terminal | undefined;
    private _trackedTerminals: Map<string, TrackedTerminal> = new Map();
    private _terminalSeq = 1;
    private _preferredTerminalId: string | undefined;
    private _terminalsEmitter = new vscode.EventEmitter<TrackedTerminalInfo[]>();
    public readonly onTerminalsChanged = this._terminalsEmitter.event;

    // ── Smart-build subsystem (initialised lazily via initSmartBuild) ──
    private _stampManager: BuildStampManager | undefined;
    private _depResolver: DependencyResolver | undefined;
    private _buildPolicy: BuildPolicy | undefined;
    private _fileWatcher: vscode.FileSystemWatcher | undefined;
    private _buildStatusEmitter = new vscode.EventEmitter<BuildEvaluation | undefined>();
    public readonly onBuildStatusChanged = this._buildStatusEmitter.event;
    private _cachedEvaluation: BuildEvaluation | undefined;
    private _evaluationDirty = true;

    constructor() {
        vscode.window.onDidCloseTerminal((terminal) => {
            for (const [id, tracked] of this._trackedTerminals.entries()) {
                if (tracked.kind === 'integrated' && tracked.terminal === terminal) {
                    this._trackedTerminals.delete(id);
                    this._emitTerminalsChanged();
                    break;
                }
            }
        });

        // Shell integration: detect when a command finishes in one of
        // our tracked terminals and flip its status to 'closed'.
        vscode.window.onDidEndTerminalShellExecution?.((event) => {
            for (const [, tracked] of this._trackedTerminals.entries()) {
                if (tracked.kind === 'integrated' && tracked.terminal === event.terminal && tracked.status === 'running') {
                    tracked.status = 'closed';
                    this._emitTerminalsChanged();
                    break;
                }
            }
        });
    }

    /**
     * Initialise the smart-build subsystem.  Must be called once after
     * construction, passing the extension context for persistent storage.
     */
    initSmartBuild(context: vscode.ExtensionContext): void {
        const storage: StampStorage = {
            get: (key) => context.workspaceState.get(key),
            update: (key, value) => { context.workspaceState.update(key, value); },
        };

        this._stampManager = new BuildStampManager(storage);
        this._depResolver = new DependencyResolver();
        this._buildPolicy = new BuildPolicy(this._stampManager, this._depResolver);

        // Watch src/ for changes to invalidate cached evaluation
        const srcDir = this.getSrcDir();
        if (srcDir) {
            const pattern = new vscode.RelativePattern(srcDir, '**/*');
            this._fileWatcher = vscode.workspace.createFileSystemWatcher(pattern);

            const invalidate = () => {
                this._evaluationDirty = true;
                this._buildStatusEmitter.fire(undefined);
            };

            this._fileWatcher.onDidChange(invalidate);
            this._fileWatcher.onDidCreate(invalidate);
            this._fileWatcher.onDidDelete(invalidate);
        }
    }

    disposeSmartBuild(): void {
        this._fileWatcher?.dispose();
        this._buildStatusEmitter.dispose();
    }

    // ── Smart-build public API ─────────────────────────────────

    /**
     * Evaluate which packages need rebuilding.
     * Uses a cache that is invalidated by the file watcher.
     */
    evaluateBuildNeeds(targetPackages?: string[]): BuildEvaluation | undefined {
        if (!this._buildPolicy) {
            return undefined;
        }

        const srcDir = this.getSrcDir();
        if (!srcDir) {
            return undefined;
        }

        // If no specific targets, evaluate all workspace packages
        if (!targetPackages || targetPackages.length === 0) {
            if (!this._evaluationDirty && this._cachedEvaluation) {
                return this._cachedEvaluation;
            }

            const graph = this._depResolver!.buildGraph(srcDir);
            targetPackages = Array.from(graph.keys());
        }

        const evaluation = this._buildPolicy.evaluateBuildNeeds(srcDir, targetPackages);

        this._cachedEvaluation = evaluation;
        this._evaluationDirty = false;

        return evaluation;
    }

    /**
     * Mark packages as successfully built, updating their stamps.
     */
    markPackagesBuilt(packageNames: string[]): void {
        if (!this._stampManager || !this._depResolver) {
            return;
        }

        const srcDir = this.getSrcDir();
        if (!srcDir) {
            return;
        }

        const graph = this._depResolver.buildGraph(srcDir);
        const now = Date.now();

        for (const name of packageNames) {
            const pkg = graph.get(name);
            const buildType = pkg?.buildType ?? 'unknown';
            this._stampManager.markBuilt(name, buildType, now);
        }

        this._evaluationDirty = true;
    }

    /**
     * Invalidate all build stamps (e.g. after a clean).
     */
    invalidateAllStamps(): void {
        this._stampManager?.invalidateAll();
        this._evaluationDirty = true;
    }

    /**
     * Build only packages that need building.
     * Returns the list of packages that were actually sent to build.
     */
    smartBuildPackages(packages: string[]): string[] {
        const evaluation = this.evaluateBuildNeeds(packages);
        if (!evaluation) {
            this.buildPackages(packages);
            return packages;
        }

        const toBuild = evaluation.packagesNeedingBuild;
        if (toBuild.length === 0) {
            return [];
        }

        this.buildPackages(toBuild);
        return toBuild;
    }

    // ── Environment detection ──────────────────────────────────
    async detectEnvironment(): Promise<RosEnvironmentInfo | undefined> {
        try {
            const distro = await this.exec('echo $ROS_DISTRO');
            const versionStr = await this.exec('echo $ROS_VERSION');
            if (!distro.trim()) {
                return undefined;
            }
            const wsPath = this.getWorkspacePath();
            this._env = {
                distro: distro.trim(),
                version: parseInt(versionStr.trim(), 10) || 2,
                workspacePath: wsPath,
            };
            return this._env;
        } catch {
            return undefined;
        }
    }

    get env(): RosEnvironmentInfo | undefined {
        return this._env;
    }

    // ── Package helpers ────────────────────────────────────────
    async listPackages(): Promise<string[]> {
        try {
            const raw = this.isRos2()
                ? await this.exec('ros2 pkg list')
                : await this.exec('rospack list-names');
            return raw.trim().split('\n').filter(Boolean);
        } catch {
            return [];
        }
    }

    async listWorkspacePackages(): Promise<string[]> {
        const srcDir = this.getSrcDir();
        if (!srcDir) {
            return [];
        }

        const packageXmlFiles = this.findPackageXmlFiles(srcDir, 4);
        const names = new Set<string>();

        for (const file of packageXmlFiles) {
            try {
                const xml = fs.readFileSync(file, 'utf8');
                const match = xml.match(/<name>\s*([^<\s]+)\s*<\/name>/);
                if (match?.[1]) {
                    names.add(match[1]);
                }
            } catch {
                // ignore malformed package.xml
            }
        }

        return Array.from(names).sort();
    }

    async listWorkspacePackageDetails(): Promise<RosWorkspacePackageDetails[]> {
        const srcDir = this.getSrcDir();
        if (!srcDir) {
            return [];
        }

        const packageXmlFiles = this.findPackageXmlFiles(srcDir, 4);
        const packages: RosWorkspacePackageDetails[] = [];
        const seen = new Set<string>();

        for (const file of packageXmlFiles) {
            try {
                const xml = fs.readFileSync(file, 'utf8');
                const match = xml.match(/<name>\s*([^<\s]+)\s*<\/name>/);
                if (!match?.[1]) {
                    continue;
                }
                const name = match[1];
                if (seen.has(name)) {
                    continue;
                }
                const packagePath = path.dirname(file);
                const launchFiles = this.findLaunchFiles(packagePath, 3);
                const nodes = this.findRunnableNodes(packagePath);

                const isPython = fs.existsSync(path.join(packagePath, 'setup.py'));
                packages.push({
                    name,
                    packagePath,
                    launchFiles,
                    nodes,
                    isPython,
                });
                seen.add(name);
            } catch {
                // ignore malformed package.xml
            }
        }

        return packages.sort((a, b) => a.name.localeCompare(b.name));
    }

    getLaunchArgOptions(filePath: string): LaunchArgOption[] {
        if (!filePath || !fs.existsSync(filePath)) {
            return [];
        }

        const ext = path.extname(filePath).toLowerCase();
        try {
            const content = fs.readFileSync(filePath, 'utf8');
            if (ext === '.py') {
                return this.parsePythonLaunchArgs(content);
            }
            if (ext === '.xml' || ext === '.launch') {
                return this.parseXmlLaunchArgs(content);
            }
        } catch {
            return [];
        }

        return [];
    }

    async createPackage(name: string, buildType: string, deps: string[]): Promise<boolean> {
        const srcDir = this.getSrcDir();
        if (!srcDir) {
            vscode.window.showErrorMessage('Could not determine workspace src directory.');
            return false;
        }

        const normalizedDeps = deps
            .map(dep => dep.trim())
            .filter(Boolean);

        const defaultDeps = this.isRos2()
            ? this.getDefaultCreatePackageDependencies(buildType)
            : [];

        const mergedDeps = [...defaultDeps, ...normalizedDeps].filter((dep, idx, arr) => arr.indexOf(dep) === idx);
        const depString = mergedDeps.length > 0 ? ` --dependencies ${mergedDeps.join(' ')}` : '';

        const cmd = this.isRos2()
            ? `cd "${srcDir}" && ros2 pkg create ${name} --build-type ${buildType}${depString}`
            : `cd "${srcDir}" && catkin_create_pkg ${name} ${mergedDeps.join(' ')}`;

        this.runInTerminal(cmd);
        return true;
    }

    /**
     * Add a new Python node to an existing ament_python package.
     * Creates the .py source file and registers it in setup.py console_scripts.
     */
    async addNodeToPackage(packageName: string, nodeName: string): Promise<boolean> {
        const srcDir = this.getSrcDir();
        if (!srcDir) {
            vscode.window.showErrorMessage('Could not determine workspace src directory.');
            return false;
        }

        const pkgDir = path.join(srcDir, packageName);
        const pyDir = path.join(pkgDir, packageName);
        const setupPyPath = path.join(pkgDir, 'setup.py');
        const nodeFilePath = path.join(pyDir, `${nodeName}.py`);

        if (!fs.existsSync(pyDir)) {
            vscode.window.showErrorMessage(`Package folder not found: ${pyDir}`);
            return false;
        }

        if (!fs.existsSync(setupPyPath)) {
            vscode.window.showErrorMessage(`setup.py not found for package "${packageName}".`);
            return false;
        }

        // 1. Create the node .py file if it doesn't exist
        if (!fs.existsSync(nodeFilePath)) {
            const template =
`#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class ${this.toPascalCase(nodeName)}Node(Node):
    def __init__(self):
        super().__init__('${nodeName}')
        self.get_logger().info('${nodeName} node started')


def main(args=None):
    rclpy.init(args=args)
    node = ${this.toPascalCase(nodeName)}Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
`;
            try {
                fs.writeFileSync(nodeFilePath, template, { mode: 0o755 });
            } catch (err) {
                vscode.window.showErrorMessage(`Failed to create node file: ${err}`);
                return false;
            }
        }

        // 2. Register the entry point in setup.py console_scripts
        try {
            let setupPy = fs.readFileSync(setupPyPath, 'utf8');
            const entry = `'${nodeName} = ${packageName}.${nodeName}:main'`;

            if (setupPy.includes(entry)) {
                // Already registered, just open the file
                const uri = vscode.Uri.file(nodeFilePath);
                await vscode.window.showTextDocument(uri, { preview: false });
                return true;
            }

            // Insert after 'console_scripts': [
            const consoleScriptsPattern = /(['"]console_scripts['"]\s*:\s*\[)/;
            if (consoleScriptsPattern.test(setupPy)) {
                setupPy = setupPy.replace(
                    consoleScriptsPattern,
                    `$1\n            ${entry},`,
                );
            } else {
                // If there's no console_scripts block, try to add one in entry_points
                const entryPointsPattern = /(entry_points\s*=\s*\{)/;
                if (entryPointsPattern.test(setupPy)) {
                    setupPy = setupPy.replace(
                        entryPointsPattern,
                        `$1\n        'console_scripts': [\n            ${entry},\n        ],`,
                    );
                } else {
                    vscode.window.showWarningMessage(
                        'Could not find console_scripts or entry_points in setup.py. ' +
                        'Node file was created but you need to register it manually.',
                    );
                }
            }

            fs.writeFileSync(setupPyPath, setupPy);
        } catch (err) {
            vscode.window.showErrorMessage(`Failed to update setup.py: ${err}`);
            return false;
        }

        // Open the new node file in the editor
        const uri = vscode.Uri.file(nodeFilePath);
        await vscode.window.showTextDocument(uri, { preview: false });

        return true;
    }

    private toPascalCase(name: string): string {
        return name
            .split(/[_-]/)
            .map((part) => part.charAt(0).toUpperCase() + part.slice(1))
            .join('');
    }

    private getDefaultCreatePackageDependencies(buildType: string): string[] {
        switch (buildType) {
            case 'ament_python':
                return ['rclpy', 'std_msgs'];
            case 'ament_cmake':
                return ['rclcpp', 'std_msgs'];
            default:
                return [];
        }
    }

    // ── Build ──────────────────────────────────────────────────
    buildPackages(packages: string[]): void {
        const wsPath = this.getWorkspacePath();
        const pkgArg = packages.length > 0 ? ` --packages-select ${packages.join(' ')}` : '';

        if (!this.isRos2()) {
            const cmd = `cd "${wsPath}" && catkin_make && source "${wsPath}/devel/setup.bash"`;
            this.runInTerminal(cmd);
            return;
        }

        const useSymlink = vscode.workspace
            .getConfiguration('rosDevToolkit')
            .get<boolean>('symlinkInstall', true);
        const symlinkFlag = useSymlink ? ' --symlink-install' : '';

        // Clean stale build & install artifacts per-package before building.
        //   • install/<pkg>/ — prevents EEXIST when switching to --symlink-install
        //   • build/<pkg>/   — removes stale CMakeCache.txt when the workspace
        //     was moved / copied from a different path
        let cleanCmd = '';
        if (packages.length > 0) {
            const rmTargets = packages
                .flatMap(p => [`"${wsPath}/build/${p}"`, `"${wsPath}/install/${p}"`])
                .join(' ');
            cleanCmd = `rm -rf ${rmTargets} && `;
        } else {
            cleanCmd = `rm -rf "${wsPath}/build" "${wsPath}/install" && `;
        }

        // Only source the ROS base — NOT the workspace overlay.
        // We just deleted install dirs so the overlay would reference missing
        // paths and produce colcon AMENT_PREFIX_PATH warnings.
        // The overlay is re-sourced at the end after the build recreates it.
        const parts: string[] = [
            this.getRosSourceCommand(),
            `cd "${wsPath}"`,
            `${cleanCmd}colcon build${symlinkFlag}${pkgArg}`,
            `source "${wsPath}/install/setup.bash"`,
        ];

        this.sendRawCommand(parts.join(' && '));

        // Update build stamps so the next evaluateBuildNeeds() knows these
        // packages were sent to build.  The actual build runs asynchronously
        // in the terminal, but we stamp now so a subsequent launch check sees
        // them as up-to-date instead of re-triggering a build every time.
        if (packages.length > 0) {
            this.markPackagesBuilt(packages);
        }
    }

    // ── Run node ───────────────────────────────────────────────
    async runNode(
        pkg: string,
        executable: string,
        args: string = '',
        nodePath?: string,
        argsLabel?: string,
    ): Promise<void> {
        const argString = args?.trim() ? ` ${args.trim()}` : '';
        const runCmd = this.isRos2()
            ? `ros2 run ${pkg} ${executable}${argString}`
            : `rosrun ${pkg} ${executable}${argString}`;
        const normalizedArgsLabel = argsLabel?.trim();
        const normalizedNodePath = nodePath?.trim() ? nodePath : undefined;
        const labelSuffix = normalizedArgsLabel ? ` [${normalizedArgsLabel}]` : '';
        const nodeLabel = `${pkg} / ${executable}${labelSuffix}`;

        // Smart-build check before running
        const result = await this.preLaunchBuildCheck(pkg);

        if (result.action === 'cancel') {
            return;
        }

        if (result.action === 'build-and-launch') {
            this.buildThenRun(result.stalePackages!, runCmd, nodeLabel, normalizedNodePath);
            return;
        }

        // No build needed — run directly
        this.runInConfiguredLaunchTerminal(runCmd, nodeLabel, normalizedNodePath);
    }

    /**
     * Build stale packages, then run a command (node, launch, etc.) in the
     * configured terminal.  The command is chained with `&&` so it only
     * starts after a successful build.
     */
    buildThenRun(
        stalePackages: string[],
        runCmd: string,
        runLabel?: string,
        runPath?: string,
    ): void {
        const wsPath = this.getWorkspacePath();
        const pkgArg = stalePackages.length > 0
            ? ` --packages-select ${stalePackages.join(' ')}`
            : '';

        const useSymlink = vscode.workspace
            .getConfiguration('rosDevToolkit')
            .get<boolean>('symlinkInstall', true);
        const symlinkFlag = useSymlink ? ' --symlink-install' : '';

        let cleanCmd = '';
        if (stalePackages.length > 0) {
            const rmTargets = stalePackages
                .flatMap(p => [`"${wsPath}/build/${p}"`, `"${wsPath}/install/${p}"`])
                .join(' ');
            cleanCmd = `rm -rf ${rmTargets} && `;
        } else {
            cleanCmd = `rm -rf "${wsPath}/build" "${wsPath}/install" && `;
        }

        const parts: string[] = [
            this.getRosSourceCommand(),
            `cd "${wsPath}"`,
            `${cleanCmd}colcon build${symlinkFlag}${pkgArg}`,
            `source "${wsPath}/install/setup.bash"`,
            runCmd,
        ];

        const fullCmd = parts.join(' && ');
        this.runInConfiguredLaunchTerminal(fullCmd, runLabel, runPath, true);

        // Update stamps so a subsequent run sees them as up-to-date.
        if (stalePackages.length > 0) {
            this.markPackagesBuilt(stalePackages);
        }
    }

    // ── Launch file ────────────────────────────────────────────
    async launchFile(
        pkg: string,
        launchFile: string,
        args: string = '',
        launchPath?: string,
        argsLabel?: string,
    ): Promise<void> {
        const argString = args?.trim() ? ` ${args.trim()}` : '';
        const launchCmd = this.isRos2()
            ? `ros2 launch ${pkg} ${launchFile}${argString}`
            : `roslaunch ${pkg} ${launchFile}${argString}`;

        const labelSuffix = argsLabel ? ` [${argsLabel}]` : '';
        const launchLabel = `${pkg} / ${launchFile}${labelSuffix}`;

        // Smart-build check before launching
        const result = await this.preLaunchBuildCheck(pkg);

        if (result.action === 'cancel') {
            return;
        }

        if (result.action === 'build-and-launch') {
            this.buildThenRun(result.stalePackages!, launchCmd, launchLabel, launchPath);
            return;
        }

        // No build needed — launch directly
        this.runInConfiguredLaunchTerminal(launchCmd, launchLabel, launchPath);
    }

    /**
     * Check if packages need building before a launch.
     * Controlled by the `rosDevToolkit.preLaunchBuildCheck` toggle:
     *  - enabled (default) → auto-build stale packages, then launch
     *  - disabled → skip check, launch directly
     *
     * Returns a structured result so the caller can chain build + launch.
     */
    async preLaunchBuildCheck(pkg: string): Promise<PreLaunchBuildResult> {
        const enabled = vscode.workspace
            .getConfiguration('rosDevToolkit')
            .get<boolean>('preLaunchBuildCheck', true);
        if (!enabled) {
            return { action: 'launch' };
        }

        const evaluation = this.evaluateBuildNeeds([pkg]);
        if (!evaluation || evaluation.packagesNeedingBuild.length === 0) {
            return { action: 'launch' };
        }

        return { action: 'build-and-launch', stalePackages: evaluation.packagesNeedingBuild };
    }

    /** @deprecated Use {@link buildThenRun} instead. */
    buildThenLaunch(
        stalePackages: string[],
        launchCmd: string,
        launchLabel?: string,
        launchPath?: string,
    ): void {
        this.buildThenRun(stalePackages, launchCmd, launchLabel, launchPath);
    }

    /** @deprecated Use {@link buildThenRun} instead. */
    buildThenRunNode(
        stalePackages: string[],
        runCmd: string,
        nodeLabel?: string,
        nodePath?: string,
    ): void {
        this.buildThenRun(stalePackages, runCmd, nodeLabel, nodePath);
    }

    private runInConfiguredLaunchTerminal(
        cmd: string,
        launchLabel?: string,
        launchPath?: string,
        raw?: boolean,
    ): void {
        const useExternal = vscode.workspace
            .getConfiguration('rosDevToolkit')
            .get<boolean>('launchInExternalTerminal', true);

        if (useExternal) {
            if (raw === undefined) {
                this.runInExternalTerminal(cmd, launchLabel, launchPath);
            } else {
                this.runInExternalTerminal(cmd, launchLabel, launchPath, raw);
            }
        } else {
            if (raw === undefined) {
                this.runInLaunchTerminal(cmd, launchLabel, launchPath);
            } else {
                this.runInLaunchTerminal(cmd, launchLabel, launchPath, raw);
            }
        }
    }

    /**
     * Run a command in a native external terminal window.
     * This ensures GUI apps (RViz, Gazebo, rqt) can display their UI.
     *
     * VS Code installed via snap pollutes the environment with GTK / GIO
     * paths that point inside the snap sandbox.  Those paths make
     * gnome-terminal (and other GTK apps) crash with a symbol-lookup
     * error.  We strip them before spawning the external terminal.
     */
    runInExternalTerminal(cmd: string, launchLabel?: string, launchPath?: string, raw?: boolean): void {
        const fullCmd = raw ? cmd : this.buildSourcedCommand(cmd);
        const bashPath = this.resolveBashPath() || '/bin/bash';

        // Build a clean environment: remove snap-injected GTK / GIO vars
        const cleanEnv = this.buildCleanEnv();

        const terminalBin = this.resolveExternalTerminal();
        if (!terminalBin) {
            vscode.window.showWarningMessage(
                'No external terminal found (tried gnome-terminal, konsole, xfce4-terminal, mate-terminal, xterm). Falling back to integrated terminal.'
            );
            this.runInTerminal(cmd);
            return;
        }

        // PID file so we can discover the real bash PID inside gnome-terminal
        const pidFile = `/tmp/ros-devtool-${Date.now()}-${this._terminalSeq}.pid`;

        // Inner bash writes its own PID, runs the command, then stays open
        const wrappedCmd = `echo $$ > "${pidFile}"; ${fullCmd}; exec ${bashPath}`;
        const args = this.buildExternalTerminalArgs(terminalBin, bashPath, wrappedCmd);

        try {
            const child = cp.spawn(terminalBin, args, {
                detached: true,
                stdio: 'ignore',
                env: cleanEnv,
            });
            this.trackExternalTerminal(child, terminalBin, cmd, launchLabel, launchPath, pidFile);
            child.on('error', () => {
                vscode.window.showWarningMessage(
                    'Failed to open external terminal. Falling back to integrated terminal.'
                );
                this.runInTerminal(cmd);
            });
            child.unref();
        } catch {
            vscode.window.showWarningMessage(
                'Failed to open external terminal. Falling back to integrated terminal.'
            );
            this.runInTerminal(cmd);
        }
    }

    runInLaunchTerminal(cmd: string, launchLabel?: string, launchPath?: string, raw?: boolean): void {
        const fullCmd = raw ? cmd : this.buildSourcedCommand(cmd);
        let tracked = this.pickLaunchTerminal();

        if (!tracked || !tracked.terminal || tracked.terminal.exitStatus !== undefined) {
            if (tracked) {
                this._trackedTerminals.delete(tracked.id);
            }
            tracked = this.createLaunchTerminal();
        }

        tracked.cmd = cmd;
        tracked.lastUsed = Date.now();
        tracked.launchLabel = launchLabel;
        tracked.launchPath = launchPath;
        tracked.ctrlCSent = false;
        tracked.status = 'running';

        if (!tracked.terminal) {
            return;
        }

        tracked.terminal.show();
        tracked.terminal.sendText(fullCmd);
        this._emitTerminalsChanged();
    }

    getTrackedTerminals(): TrackedTerminalInfo[] {
        return Array.from(this._trackedTerminals.values()).map((t) => ({
            id: t.id,
            kind: t.kind,
            name: t.name,
            status: t.status,
            cmd: t.cmd,
            pid: t.pid,
            launchLabel: t.launchLabel,
            launchPath: t.launchPath,
            createdAt: t.createdAt,
            lastUsed: t.lastUsed,
            isPreferred: t.id === this._preferredTerminalId,
        }));
    }

    getPreferredTerminalId(): string | undefined {
        return this._preferredTerminalId;
    }

    setPreferredTerminal(id?: string): void {
        if (id && this._trackedTerminals.has(id)) {
            this._preferredTerminalId = id;
        } else {
            this._preferredTerminalId = undefined;
        }
        this._emitTerminalsChanged();
    }

    focusTrackedTerminal(id: string): void {
        const tracked = this._trackedTerminals.get(id);
        if (tracked?.kind === 'integrated' && tracked.terminal) {
            tracked.terminal.show();
        }
    }

    async killTrackedTerminal(id: string): Promise<void> {
        const tracked = this._trackedTerminals.get(id);
        if (!tracked) {
            return;
        }

        if (tracked.kind === 'integrated') {
            if (tracked.terminal) {
                if (
                    tracked.terminal.exitStatus !== undefined ||
                    tracked.status === 'closed' ||
                    tracked.ctrlCSent
                ) {
                    // The process already finished, or we previously sent
                    // Ctrl+C — force-close the terminal.
                    tracked.terminal.dispose();
                    this._trackedTerminals.delete(id);
                    if (this._preferredTerminalId === id) {
                        this._preferredTerminalId = undefined;
                    }
                } else {
                    // Process still running — send Ctrl+C.  The appended
                    // "; exit" in the original command will make the shell
                    // exit once the process terminates.
                    tracked.terminal.sendText('\x03', false);
                    tracked.ctrlCSent = true;
                    tracked.lastUsed = Date.now();
                }
            }
        } else {
            // Prefer the real inner bash PID so SIGINT behaves like Ctrl+C.
            const innerPid = this.readInnerPid(tracked, false);
            if (innerPid) {
                this.interruptProcess(innerPid, true);
            } else {
                // PID file may not be ready yet right after spawn; retry briefly.
                this.scheduleDeferredInterrupt(id);
            }
            tracked.lastUsed = Date.now();
        }

        this._emitTerminalsChanged();
    }

    private async isIntegratedShellIdle(terminal: vscode.Terminal): Promise<boolean> {
        try {
            const shellPid = await terminal.processId;
            if (!shellPid || shellPid <= 0) {
                // If VS Code cannot provide a PID, prefer close semantics for Kill.
                return true;
            }

            const hasActiveChildren = (): boolean => {
                const descendants = this.getDescendantPids(shellPid);
                if (!descendants.length) {
                    return false;
                }

                for (const pid of descendants) {
                    const processName = this.getProcessName(pid);
                    if (!processName) {
                        // Process disappeared between pgrep and ps
                        // (e.g. zombie reaped); treat as gone.
                        continue;
                    }
                    if (!this.isShellLikeProcess(processName)) {
                        return true;
                    }
                }
                return false;
            };

            if (!hasActiveChildren()) {
                return true;
            }

            // Prompt helpers (git/starship/etc.) can be brief; re-check once.
            await this.delay(140);
            return !hasActiveChildren();
        } catch {
            return false;
        }
    }

    private scheduleIntegratedCloseWhenIdle(id: string, killTimestamp: number, attempt: number = 0): void {
        if (attempt >= 8) {
            return;
        }

        setTimeout(async () => {
            const tracked = this._trackedTerminals.get(id);
            if (!tracked || tracked.kind !== 'integrated' || !tracked.terminal) {
                return;
            }

            // Terminal process already exited — close immediately.
            if (tracked.terminal.exitStatus !== undefined) {
                tracked.terminal.dispose();
                this._trackedTerminals.delete(id);
                if (this._preferredTerminalId === id) {
                    this._preferredTerminalId = undefined;
                }
                this._emitTerminalsChanged();
                return;
            }

            // Terminal was reused for a newer command; don't auto-close it.
            if (tracked.lastUsed !== killTimestamp) {
                return;
            }

            const shellIdle = await this.isIntegratedShellIdle(tracked.terminal);
            if (shellIdle) {
                tracked.terminal.dispose();
                this._trackedTerminals.delete(id);
                if (this._preferredTerminalId === id) {
                    this._preferredTerminalId = undefined;
                }
                this._emitTerminalsChanged();
                return;
            }

            this.scheduleIntegratedCloseWhenIdle(id, killTimestamp, attempt + 1);
        }, 180);
    }

    private getProcessName(pid: number): string | undefined {
        try {
            const out = cp
                .execSync(`ps -p ${pid} -o comm= 2>/dev/null`, { encoding: 'utf8' })
                .trim()
                .toLowerCase();
            return out || undefined;
        } catch {
            return undefined;
        }
    }

    private isShellLikeProcess(name: string): boolean {
        const shellNames = new Set([
            'bash',
            'sh',
            'zsh',
            'fish',
            'dash',
            'ksh',
            'tmux',
        ]);
        return shellNames.has(name);
    }

    private delay(ms: number): Promise<void> {
        return new Promise((resolve) => setTimeout(resolve, ms));
    }

    private pickLaunchTerminal(): TrackedTerminal | undefined {
        if (this._preferredTerminalId) {
            const preferred = this._trackedTerminals.get(this._preferredTerminalId);
            if (preferred?.kind === 'integrated' && preferred.status === 'running') {
                return preferred;
            }
        }

        const active = Array.from(this._trackedTerminals.values())
            .filter((t) => t.kind === 'integrated' && t.status === 'running');

        if (!active.length) {
            return undefined;
        }

        active.sort((a, b) => b.lastUsed - a.lastUsed);
        return active[0];
    }

    private createLaunchTerminal(): TrackedTerminal {
        const bashPath = this.resolveBashPath();
        const env = this.getTerminalEnv();
        const num = this._terminalSeq++;
        const name = `ROS Launch ${num}`;
        const terminal = bashPath
            ? vscode.window.createTerminal({ name, shellPath: bashPath, env })
            : vscode.window.createTerminal({ name, env });

        const tracked: TrackedTerminal = {
            id: `launch-${num}`,
            kind: 'integrated',
            name,
            status: 'running',
            cmd: '',
            createdAt: Date.now(),
            lastUsed: Date.now(),
            terminal,
        };

        this._trackedTerminals.set(tracked.id, tracked);
        this._emitTerminalsChanged();
        return tracked;
    }

    private trackExternalTerminal(
        child: cp.ChildProcess,
        terminalBin: string,
        cmd: string,
        launchLabel?: string,
        launchPath?: string,
        pidFile?: string,
    ): void {
        const num = this._terminalSeq++;
        const name = `External (${path.basename(terminalBin)}) ${num}`;
        const tracked: TrackedTerminal = {
            id: `external-${num}`,
            kind: 'external',
            name,
            status: 'running',
            cmd,
            pid: child.pid ?? undefined,
            launchLabel,
            launchPath,
            createdAt: Date.now(),
            lastUsed: Date.now(),
            childProcess: child,
            pidFile,
        };

        this._trackedTerminals.set(tracked.id, tracked);
        this._emitTerminalsChanged();

        // Poll for the PID file so we cache the inner bash PID early
        if (pidFile) {
            let attempts = 0;
            const poll = setInterval(() => {
                attempts++;
                try {
                    if (fs.existsSync(pidFile)) {
                        const content = fs.readFileSync(pidFile, 'utf8').trim();
                        const pid = parseInt(content, 10);
                        if (pid > 0) {
                            tracked.innerPid = pid;
                            clearInterval(poll);
                        }
                    }
                } catch { /* ignore */ }
                if (attempts > 20) { clearInterval(poll); }
            }, 250);
        }

        child.on('exit', () => {
            this.cleanupPidFile(tracked);
            this._trackedTerminals.delete(tracked.id);
            if (this._preferredTerminalId === tracked.id) {
                this._preferredTerminalId = undefined;
            }
            this._emitTerminalsChanged();
        });
    }

    /**
     * Read the real inner bash PID from the PID file.
     * Falls back to the cached innerPid or the child PID.
     */
    private readInnerPid(tracked: TrackedTerminal, allowChildFallback: boolean = true): number | undefined {
        // Try cached first
        if (tracked.innerPid) {
            return tracked.innerPid;
        }

        // Try reading from PID file
        if (tracked.pidFile) {
            try {
                const content = fs.readFileSync(tracked.pidFile, 'utf8').trim();
                const pid = parseInt(content, 10);
                if (pid > 0) {
                    tracked.innerPid = pid;
                    return pid;
                }
            } catch { /* file not ready yet */ }
        }

        // Last resort: use the spawned child PID (won't work for gnome-terminal)
        return allowChildFallback ? tracked.pid : undefined;
    }

    private scheduleDeferredInterrupt(trackedId: string, attempt: number = 0): void {
        if (attempt >= 10) {
            return;
        }

        setTimeout(() => {
            const tracked = this._trackedTerminals.get(trackedId);
            if (!tracked || tracked.kind !== 'external') {
                return;
            }

            const innerPid = this.readInnerPid(tracked, false);
            if (innerPid) {
                this.interruptProcess(innerPid, true);
                return;
            }

            this.scheduleDeferredInterrupt(trackedId, attempt + 1);
        }, 150);
    }

    /**
     * Kill the inner bash process and its entire child tree.
     * Sends SIGTERM, then SIGKILL after 500ms.
     */
    private killProcessTree(pid: number): void {
        const descendants = this.getDescendantPids(pid);

        // SIGTERM descendants leaf-first, then the parent
        for (const desc of descendants) {
            try { process.kill(desc, 'SIGTERM'); } catch { /* ignore */ }
        }
        try { process.kill(pid, 'SIGTERM'); } catch { /* ignore */ }

        // Force kill after delay
        setTimeout(() => {
            for (const desc of descendants) {
                try { process.kill(desc, 'SIGKILL'); } catch { /* ignore */ }
            }
            try { process.kill(pid, 'SIGKILL'); } catch { /* ignore */ }
        }, 500);
    }

    /**
     * Send SIGINT (Ctrl+C) to the inner bash's child processes.
     * This interrupts ros2 launch without closing the terminal.
     *
     * For `ros2 run` the actual node is a grandchild of the bash, so we
     * must walk the full descendant tree — `pkill -P` only hits direct
     * children and would leave the node process orphaned.
     */
    private interruptProcess(pid: number, includeProcessGroup: boolean = false): void {
        // Collect ALL descendant PIDs (recursive), leaf-first, so children
        // are signalled before parents.
        const descendants = this.getDescendantPids(pid);

        if (includeProcessGroup) {
            // Best-effort: signal the shell's process group to mimic Ctrl+C.
            try {
                process.kill(-pid, 'SIGINT');
            } catch { /* ignore */ }
        }

        // Signal every descendant individually (leaf → root order).
        for (const desc of descendants) {
            try {
                process.kill(desc, 'SIGINT');
            } catch { /* ignore */ }
        }

        // Also send to the bash itself in case it's the foreground
        try {
            process.kill(pid, 'SIGINT');
        } catch { /* ignore */ }
    }

    /**
     * Recursively collect all descendant PIDs of `parentPid`,
     * returned in leaf-first (deepest-first) order.
     */
    private getDescendantPids(parentPid: number): number[] {
        const result: number[] = [];
        try {
            // `pgrep -P <pid>` lists direct children.
            const raw = cp.execSync(`pgrep -P ${parentPid} 2>/dev/null`, { encoding: 'utf8' }).trim();
            if (!raw) {
                return result;
            }
            const children = raw.split('\n').map(s => parseInt(s, 10)).filter(n => n > 0);
            // Recurse into each child first (depth-first) …
            for (const child of children) {
                result.push(...this.getDescendantPids(child));
            }
            // … then add the direct children.
            result.push(...children);
        } catch { /* pgrep returns non-zero when there are no children */ }
        return result;
    }

    /**
     * Remove the temporary PID file.
     */
    private cleanupPidFile(tracked: TrackedTerminal): void {
        if (tracked.pidFile) {
            try {
                fs.unlinkSync(tracked.pidFile);
            } catch { /* ignore */ }
        }
    }

    private _emitTerminalsChanged(): void {
        this._terminalsEmitter.fire(this.getTrackedTerminals());
    }

    /**
     * Return a copy of `process.env` with snap-injected GTK / GIO /
     * locale variables removed.  These cause gnome-terminal (and other
     * native GTK apps) to crash when VS Code is installed as a snap.
     */
    private buildCleanEnv(): Record<string, string | undefined> {
        const env = { ...process.env };

        // Variables the VS Code snap injects that point inside /snap/code/…
        const snapVars = [
            'GTK_PATH',
            'GTK_EXE_PREFIX',
            'GTK_IM_MODULE_FILE',
            'GIO_MODULE_DIR',
            'LOCPATH',
            'GSETTINGS_SCHEMA_DIR',
        ];

        for (const key of snapVars) {
            // Restore the original value if the snap saved one
            const origKey = `${key}_VSCODE_SNAP_ORIG`;
            const origValue = env[origKey];
            if (origValue) {
                env[key] = origValue;
            } else {
                delete env[key];
            }
            delete env[origKey];
        }

        // Also restore XDG dirs to their pre-snap values
        for (const xdg of ['XDG_DATA_DIRS', 'XDG_CONFIG_DIRS', 'XDG_DATA_HOME']) {
            const origKey = `${xdg}_VSCODE_SNAP_ORIG`;
            const origValue = env[origKey];
            if (origValue) {
                env[xdg] = origValue;
            }
            delete env[origKey];
        }

        // Make sure DISPLAY is set (needed for X11 / XWayland)
        if (!env.DISPLAY) {
            env.DISPLAY = ':0';
        }

        return env;
    }

    private resolveExternalTerminal(): string | undefined {
        const candidates = [
            'gnome-terminal',
            'x-terminal-emulator',
            'konsole',
            'xfce4-terminal',
            'mate-terminal',
            'xterm',
        ];

        for (const name of candidates) {
            try {
                const bin = cp.execSync(`which ${name} 2>/dev/null`, { encoding: 'utf8' }).trim();
                if (bin) { return bin; }
            } catch { /* not found */ }
        }

        return undefined;
    }

    private buildExternalTerminalArgs(
        terminalBin: string,
        bashPath: string,
        wrappedCmd: string,
    ): string[] {
        const base = path.basename(terminalBin);

        if (base.startsWith('gnome-terminal')) {
            return ['--wait', '--', bashPath, '-ic', wrappedCmd];
        }
        if (base === 'konsole') {
            return ['-e', bashPath, '-ic', wrappedCmd];
        }
        if (base === 'xfce4-terminal' || base === 'mate-terminal') {
            return ['-e', `${bashPath} -ic '${wrappedCmd.replace(/'/g, "'\\''")}'`];
        }
        // xterm and others
        return ['-e', bashPath, '-ic', wrappedCmd];
    }

    // ── Graph data (nodes / topics / services / actions / parameters) ──────
    async getNodeList(): Promise<string[]> {
        try {
            const raw = this.isRos2()
                ? await this.exec('ros2 node list')
                : await this.exec('rosnode list');
            return raw.trim().split('\n').filter(Boolean);
        } catch {
            return [];
        }
    }

    async getTopicList(): Promise<RosGraphEntityInfo[]> {
        try {
            const raw = this.isRos2()
                ? await this.exec('ros2 topic list -t')
                : await this.exec('rostopic list');
            return this.parseEntityList(raw);
        } catch {
            return [];
        }
    }

    async getServiceList(): Promise<RosGraphEntityInfo[]> {
        try {
            const raw = this.isRos2()
                ? await this.exec('ros2 service list -t')
                : await this.exec('rosservice list');
            return this.parseEntityList(raw);
        } catch {
            return [];
        }
    }

    async getActionList(): Promise<RosGraphEntityInfo[]> {
        if (!this.isRos2()) {
            return [];
        }
        try {
            const raw = await this.exec('ros2 action list -t');
            return this.parseEntityList(raw);
        } catch {
            return [];
        }
    }

    async getParameterList(): Promise<RosParameterInfo[]> {
        try {
            const raw = this.isRos2()
                ? await this.exec('ros2 param list')
                : await this.exec('rosparam list');
            return this.isRos2()
                ? this.parseRos2ParameterList(raw)
                : this.parseRos1ParameterList(raw);
        } catch {
            return [];
        }
    }

    async getLatestTopicMessage(topicName: string, timeoutSeconds = 2): Promise<string | undefined> {
        if (!this.isValidRosResourceName(topicName)) {
            return undefined;
        }

        try {
            const safeTimeoutSeconds = Number.isFinite(timeoutSeconds) && timeoutSeconds > 0
                ? timeoutSeconds
                : 2;
            const timeoutToken = `${safeTimeoutSeconds}s`;
            const command = this.isRos2()
                // Read from the normal stream output and stop after the first
                // message block separator ("---") so the UI receives a sample
                // exactly as users see it in `ros2 topic echo`.
                ? `timeout ${timeoutToken} ros2 topic echo ${topicName} 2>/dev/null | sed -n '/^---$/q;p'`
                : `timeout ${timeoutToken} rostopic echo -n 1 ${topicName} 2>/dev/null || true`;
            const raw = await this.exec(command);
            const trimmed = raw.trim();
            if (!trimmed) {
                return undefined;
            }
            const maxChars = 6000;
            if (trimmed.length > maxChars) {
                return `${trimmed.slice(0, maxChars)}\n... (truncated)`;
            }
            return trimmed;
        } catch {
            return undefined;
        }
    }

    async getTopicPublishTemplate(
        topicName: string,
        topicTypeHint?: string,
    ): Promise<RosTopicPublishTemplateResult> {
        if (!this.isValidRosResourceName(topicName)) {
            return {
                success: false,
                topicName,
                error: 'Invalid topic name.',
            };
        }

        const topicType = await this.resolveTopicType(topicName, topicTypeHint);
        if (!topicType) {
            return {
                success: false,
                topicName,
                error: 'Could not resolve topic type for this topic.',
            };
        }

        try {
            const command = this.isRos2()
                ? `ros2 interface show ${topicType}`
                : `rosmsg show ${topicType}`;
            const rawDefinition = await this.exec(command);
            const defaults = this.buildMessageDefaultsFromInterface(rawDefinition);
            const template = JSON.stringify(defaults, null, 2);
            return {
                success: true,
                topicName,
                topicType,
                template,
            };
        } catch (err) {
            return {
                success: false,
                topicName,
                topicType,
                error: this.normalizeCliError(err, 'Failed to fetch topic interface.'),
            };
        }
    }

    async publishTopicMessage(
        topicName: string,
        payload: string,
        topicTypeHint?: string,
    ): Promise<RosTopicPublishResult> {
        if (!this.isValidRosResourceName(topicName)) {
            return {
                success: false,
                topicName,
                error: 'Invalid topic name.',
            };
        }

        const payloadText = String(payload ?? '').trim();
        if (!payloadText) {
            return {
                success: false,
                topicName,
                error: 'Message payload is empty.',
            };
        }

        const topicType = await this.resolveTopicType(topicName, topicTypeHint);
        if (!topicType) {
            return {
                success: false,
                topicName,
                error: 'Could not resolve topic type for this topic.',
            };
        }

        try {
            // Keep payload exactly as authored by the user. JSON is accepted as
            // YAML input by ROS CLIs, and users may also provide hand-written YAML.
            const command = this.isRos2()
                ? `timeout 6s ros2 topic pub --once ${topicName} ${topicType} '${payloadText}'`
                : `timeout 6s rostopic pub -1 ${topicName} ${topicType} '${payloadText}'`;
            await this.exec(command);
            return {
                success: true,
                topicName,
                topicType,
            };
        } catch (err) {
            return {
                success: false,
                topicName,
                topicType,
                error: this.normalizeCliError(err, 'Failed to publish message.'),
            };
        }
    }

    async getNodeGraphInfo(nodeName: string): Promise<RosNodeGraphInfo> {
        try {
            const raw = this.isRos2()
                ? await this.exec(`ros2 node info ${nodeName}`)
                : await this.exec(`rosnode info ${nodeName}`);
            return this.parseNodeGraphInfo(raw);
        } catch {
            return {
                publishers: [],
                subscribers: [],
                serviceServers: [],
                serviceClients: [],
                actionServers: [],
                actionClients: [],
            };
        }
    }

    async getNodeInfo(nodeName: string): Promise<{ publishers: string[]; subscribers: string[] }> {
        const info = await this.getNodeGraphInfo(nodeName);
        return { publishers: info.publishers, subscribers: info.subscribers };
    }

    private parseEntityList(raw: string): RosGraphEntityInfo[] {
        // ROS 2 uses "<name> [<type>]" while ROS 1 usually returns just "<name>".
        // This parser accepts both formats and normalizes duplicates by name.
        const lines = raw.split('\n').map((line) => line.trim()).filter(Boolean);
        const entities: RosGraphEntityInfo[] = [];
        const seen = new Set<string>();

        for (const line of lines) {
            const typedMatch = line.match(/^(\S+)\s+\[(.+)\]$/);
            const name = typedMatch?.[1] ?? line;
            const type = typedMatch?.[2] ?? 'unknown';

            if (!name.startsWith('/')) {
                continue;
            }
            if (seen.has(name)) {
                continue;
            }
            seen.add(name);
            entities.push({ name, type });
        }

        return entities;
    }

    private parseRos2ParameterList(raw: string): RosParameterInfo[] {
        const params: RosParameterInfo[] = [];
        const seen = new Set<string>();
        let currentNode = '';

        for (const line of raw.split('\n')) {
            const trimmed = line.trim();
            if (!trimmed) {
                continue;
            }

            if (trimmed.startsWith('/') && trimmed.endsWith(':')) {
                const nodeName = trimmed.slice(0, -1).trim();
                currentNode = this.isValidRosResourceName(nodeName) ? nodeName : '';
                continue;
            }

            if (!currentNode) {
                continue;
            }

            const paramName = trimmed.replace(/^-+\s*/, '').trim();
            if (!this.isValidRosParameterName(paramName)) {
                continue;
            }

            const key = `${currentNode}:${paramName}`;
            if (seen.has(key)) {
                continue;
            }
            seen.add(key);
            params.push({ name: paramName, node: currentNode });
        }

        return params.sort(
            (a, b) => (a.node ?? '').localeCompare(b.node ?? '') || a.name.localeCompare(b.name),
        );
    }

    private parseRos1ParameterList(raw: string): RosParameterInfo[] {
        const params: RosParameterInfo[] = [];
        const seen = new Set<string>();

        for (const line of raw.split('\n')) {
            const trimmed = line.trim();
            if (!trimmed || !this.isValidRosResourceName(trimmed) || seen.has(trimmed)) {
                continue;
            }
            seen.add(trimmed);
            params.push({ name: trimmed });
        }

        return params.sort((a, b) => a.name.localeCompare(b.name));
    }

    private parseNodeGraphInfo(raw: string): RosNodeGraphInfo {
        // `ros2 node info` and `rosnode info` are section-based text outputs.
        // We track the active section while scanning and route each discovered
        // endpoint to its matching relation bucket.
        const publishers: string[] = [];
        const subscribers: string[] = [];
        const serviceServers: string[] = [];
        const serviceClients: string[] = [];
        const actionServers: string[] = [];
        const actionClients: string[] = [];

        let section:
            | 'pub'
            | 'sub'
            | 'srvServer'
            | 'srvClient'
            | 'actionServer'
            | 'actionClient'
            | null = null;

        for (const line of raw.split('\n')) {
            const trimmed = line.trim();
            if (!trimmed) {
                continue;
            }

            const lower = trimmed.toLowerCase();
            if (
                lower === 'publishers:' ||
                lower === 'publications:' ||
                lower.startsWith('publishers:')
            ) {
                section = 'pub';
                continue;
            }
            if (
                lower === 'subscribers:' ||
                lower === 'subscriptions:' ||
                lower.startsWith('subscribers:')
            ) {
                section = 'sub';
                continue;
            }
            if (lower === 'service servers:' || lower.startsWith('service servers:')) {
                section = 'srvServer';
                continue;
            }
            if (lower === 'service clients:' || lower.startsWith('service clients:')) {
                section = 'srvClient';
                continue;
            }
            if (lower === 'services:' || lower.startsWith('services:')) {
                section = 'srvServer';
                continue;
            }
            if (lower === 'action servers:' || lower.startsWith('action servers:')) {
                section = 'actionServer';
                continue;
            }
            if (lower === 'action clients:' || lower.startsWith('action clients:')) {
                section = 'actionClient';
                continue;
            }

            const endpoint = this.parseNodeInfoEndpoint(trimmed);
            if (!endpoint || !section) {
                continue;
            }

            const normalizedEndpoint = this.normalizeGraphEndpoint(section, endpoint);

            if (section === 'pub') {
                publishers.push(normalizedEndpoint);
                continue;
            }
            if (section === 'sub') {
                subscribers.push(normalizedEndpoint);
                continue;
            }
            if (section === 'srvServer') {
                serviceServers.push(normalizedEndpoint);
                continue;
            }
            if (section === 'srvClient') {
                serviceClients.push(normalizedEndpoint);
                continue;
            }
            if (section === 'actionServer') {
                actionServers.push(normalizedEndpoint);
                continue;
            }
            actionClients.push(normalizedEndpoint);
        }

        return {
            publishers: Array.from(new Set(publishers)),
            subscribers: Array.from(new Set(subscribers)),
            serviceServers: Array.from(new Set(serviceServers)),
            serviceClients: Array.from(new Set(serviceClients)),
            actionServers: Array.from(new Set(actionServers)),
            actionClients: Array.from(new Set(actionClients)),
        };
    }

    private parseNodeInfoEndpoint(trimmedLine: string): string | undefined {
        // Node info lines typically include one absolute ROS name first
        // (topic/service/action path). We extract that first path token.
        const pathMatch = trimmedLine.match(/(\/[^\s:\[]+)/);
        if (!pathMatch?.[1]) {
            return undefined;
        }
        return pathMatch[1];
    }

    private isValidRosResourceName(name: string): boolean {
        // Reject whitespace and shell metacharacters because names are used
        // in CLI commands. ROS names we care about are absolute paths.
        return /^\/[A-Za-z0-9_./~-]+$/.test(name);
    }

    private isValidRosParameterName(name: string): boolean {
        // ROS parameter names are identifiers with namespace separators, used
        // in CLI calls and therefore must not contain shell metacharacters.
        return /^[A-Za-z0-9_./~-]+$/.test(name);
    }

    private isValidRosTypeName(typeName: string): boolean {
        // ROS type names are slash-delimited package/type tokens.
        return /^[A-Za-z][A-Za-z0-9_]*(?:\/[A-Za-z][A-Za-z0-9_]*)+$/.test(typeName);
    }

    private async resolveTopicType(topicName: string, topicTypeHint?: string): Promise<string | undefined> {
        const hinted = String(topicTypeHint ?? '').trim();
        if (hinted && hinted !== 'unknown' && this.isValidRosTypeName(hinted)) {
            return hinted;
        }

        try {
            const command = this.isRos2()
                ? `ros2 topic type ${topicName}`
                : `rostopic type ${topicName}`;
            const raw = await this.exec(command);
            const resolved = raw
                .split('\n')
                .map((line) => line.trim())
                .find(Boolean);

            if (!resolved || !this.isValidRosTypeName(resolved)) {
                return undefined;
            }
            return resolved;
        } catch {
            return undefined;
        }
    }

    private normalizeCliError(err: unknown, fallback: string): string {
        const text = String(err ?? '').trim();
        if (!text) {
            return fallback;
        }
        const maxChars = 400;
        return text.length > maxChars
            ? `${text.slice(0, maxChars)}...`
            : text;
    }

    private buildMessageDefaultsFromInterface(rawInterface: string): Record<string, unknown> {
        type ParseContext = {
            indent: number;
            target: Record<string, unknown>;
        };

        const root: Record<string, unknown> = {};
        const stack: ParseContext[] = [{ indent: -1, target: root }];

        // `ros2 interface show` and `rosmsg show` present nested message fields
        // as indentation-based trees. We reconstruct that tree and assign simple
        // scalar defaults so users get an editable publish-ready payload.
        for (const line of rawInterface.split('\n')) {
            const withoutComment = line.split('#')[0].trimEnd();
            if (!withoutComment.trim()) {
                continue;
            }
            if (withoutComment.trim() === '---') {
                // Service request/response separator (not expected for topics),
                // but ignored defensively.
                continue;
            }

            const indent = withoutComment.length - withoutComment.trimStart().length;
            const trimmed = withoutComment.trim();
            const parts = trimmed.split(/\s+/);
            if (parts.length < 2) {
                continue;
            }

            const typeToken = parts[0];
            const fieldToken = parts[1];
            if (!fieldToken || fieldToken.includes('=')) {
                // Constant line (e.g. uint8 FOO=1), not a message field.
                continue;
            }
            if (!/^[A-Za-z][A-Za-z0-9_]*$/.test(fieldToken)) {
                continue;
            }

            while (stack.length > 1 && indent <= stack[stack.length - 1].indent) {
                stack.pop();
            }
            const parent = stack[stack.length - 1].target;

            const descriptor = this.parseInterfaceTypeDescriptor(typeToken);
            const primitiveDefault = this.getPrimitiveDefaultValue(descriptor.baseType);

            if (!descriptor.isArray && primitiveDefault !== undefined) {
                parent[fieldToken] = primitiveDefault;
                continue;
            }

            if (descriptor.isArray && primitiveDefault !== undefined) {
                if (descriptor.fixedLength && descriptor.fixedLength > 0) {
                    parent[fieldToken] = Array.from({ length: descriptor.fixedLength }, () => primitiveDefault);
                } else {
                    parent[fieldToken] = [];
                }
                continue;
            }

            // Complex message field. For arrays we pre-populate one sample item
            // so nested structure is visible/editable in the modal by default.
            const nestedTemplate: Record<string, unknown> = {};
            if (descriptor.isArray) {
                if (descriptor.fixedLength && descriptor.fixedLength > 0) {
                    parent[fieldToken] = Array.from({ length: descriptor.fixedLength }, () => nestedTemplate);
                } else {
                    parent[fieldToken] = [nestedTemplate];
                }
            } else {
                parent[fieldToken] = nestedTemplate;
            }

            stack.push({ indent, target: nestedTemplate });
        }

        return root;
    }

    private parseInterfaceTypeDescriptor(typeToken: string): {
        baseType: string;
        isArray: boolean;
        fixedLength?: number;
    } {
        const arrayMatch = typeToken.match(/^(.+)\[([^\]]*)\]$/);
        if (!arrayMatch) {
            return { baseType: typeToken, isArray: false };
        }

        const baseType = arrayMatch[1].trim();
        const lengthToken = arrayMatch[2].trim();
        const fixedLength = /^[0-9]+$/.test(lengthToken)
            ? parseInt(lengthToken, 10)
            : undefined;

        return { baseType, isArray: true, fixedLength };
    }

    private getPrimitiveDefaultValue(baseType: string): string | number | boolean | undefined {
        const normalized = baseType.trim().toLowerCase();

        if (
            normalized === 'string'
            || normalized === 'wstring'
            || normalized.startsWith('string<=')
            || normalized.startsWith('wstring<=')
        ) {
            return '';
        }
        if (normalized === 'bool') {
            return false;
        }
        if (
            normalized === 'byte'
            || normalized === 'char'
            || normalized === 'float'
            || normalized === 'double'
            || /^u?int(8|16|32|64)$/.test(normalized)
            || /^float(32|64)$/.test(normalized)
        ) {
            return 0;
        }
        return undefined;
    }

    private normalizeGraphEndpoint(
        section: 'pub' | 'sub' | 'srvServer' | 'srvClient' | 'actionServer' | 'actionClient',
        endpoint: string,
    ): string {
        if (section !== 'actionServer' && section !== 'actionClient') {
            return endpoint;
        }

        // Some ROS 2 node-info outputs expose internal action transport topics
        // (e.g. /fibonacci/_action/feedback). Normalize those back to /fibonacci
        // so they match `ros2 action list -t` entries used by the UI.
        const actionMatch = endpoint.match(/^(\/.+?)\/_action(?:\/.*)?$/);
        if (actionMatch?.[1]) {
            return actionMatch[1];
        }

        return endpoint;
    }

    // ── Utilities ──────────────────────────────────────────────
    isRos2(): boolean {
        return (this._env?.version ?? 2) === 2;
    }

    private findRunnableNodes(packagePath: string): RosRunnableNodeInfo[] {
        const nodeMap = new Map<string, RosRunnableNodeInfo>();

        const addNode = (name: string, sourcePath?: string) => {
            const normalizedName = name.trim();
            if (!normalizedName) {
                return;
            }
            const normalizedPath = sourcePath && fs.existsSync(sourcePath)
                ? sourcePath
                : undefined;
            const existing = nodeMap.get(normalizedName);
            if (!existing) {
                nodeMap.set(normalizedName, { name: normalizedName, sourcePath: normalizedPath });
                return;
            }
            if (!existing.sourcePath && normalizedPath) {
                nodeMap.set(normalizedName, { ...existing, sourcePath: normalizedPath });
            }
        };

        const parseConsoleScriptLine = (line: string): { execName: string; moduleName: string } | undefined => {
            const match = line.match(/^\s*([A-Za-z0-9_-]+)\s*=\s*([A-Za-z0-9_.]+)\s*:/);
            if (!match) {
                return undefined;
            }
            return { execName: match[1], moduleName: match[2] };
        };

        const setupPyPath = path.join(packagePath, 'setup.py');
        if (fs.existsSync(setupPyPath)) {
            try {
                const setupPy = fs.readFileSync(setupPyPath, 'utf8');
                const consoleScriptsBlock = setupPy.match(/['"]console_scripts['"]\s*:\s*\[([\s\S]*?)\]/m);
                if (consoleScriptsBlock?.[1]) {
                    const entryRegex = /['"]\s*([A-Za-z0-9_-]+)\s*=\s*([A-Za-z0-9_.]+)\s*:[^'"]+['"]/g;
                    let match: RegExpExecArray | null;
                    while ((match = entryRegex.exec(consoleScriptsBlock[1])) !== null) {
                        const execName = match[1];
                        const moduleName = match[2];
                        const sourcePath = path.join(packagePath, ...moduleName.split('.')) + '.py';
                        addNode(execName, sourcePath);
                    }
                }
            } catch {
                // ignore parse errors
            }
        }

        const setupCfgPath = path.join(packagePath, 'setup.cfg');
        if (fs.existsSync(setupCfgPath)) {
            try {
                const setupCfg = fs.readFileSync(setupCfgPath, 'utf8');
                const sectionMatch = setupCfg.match(/\[options\.entry_points\][\s\S]*?console_scripts\s*=\s*([\s\S]*?)(?:\n\s*\[|$)/m);
                if (sectionMatch?.[1]) {
                    for (const line of sectionMatch[1].split('\n')) {
                        const parsed = parseConsoleScriptLine(line);
                        if (!parsed) {
                            continue;
                        }
                        const sourcePath = path.join(packagePath, ...parsed.moduleName.split('.')) + '.py';
                        addNode(parsed.execName, sourcePath);
                    }
                }
            } catch {
                // ignore parse errors
            }
        }

        const cmakePath = path.join(packagePath, 'CMakeLists.txt');
        if (fs.existsSync(cmakePath)) {
            try {
                const cmake = fs.readFileSync(cmakePath, 'utf8');
                const addExecutableRegex = /add_executable\s*\(\s*([^\s\)]+)\s+([\s\S]*?)\)/gm;
                let match: RegExpExecArray | null;
                while ((match = addExecutableRegex.exec(cmake)) !== null) {
                    const executableName = match[1].trim();
                    const sourcesBlock = match[2];
                    const sourceMatch = sourcesBlock.match(/["']?([^\s"'$)]+?\.(?:cxx|cpp|cc|c))["']?/i);
                    const sourcePath = sourceMatch
                        ? path.join(packagePath, sourceMatch[1])
                        : undefined;
                    addNode(executableName, sourcePath);
                }
            } catch {
                // ignore parse errors
            }
        }

        return Array.from(nodeMap.values()).sort((a, b) => a.name.localeCompare(b.name));
    }

    getWorkspacePath(): string {
        const folders = vscode.workspace.workspaceFolders;
        return folders?.[0]?.uri.fsPath ?? process.cwd();
    }

    getSrcDir(): string | undefined {
        const wsPath = this.getWorkspacePath();
        const srcPath = path.join(wsPath, 'src');
        if (fs.existsSync(srcPath)) {
            return srcPath;
        }
        // Fallback – create the src directory
        try {
            fs.mkdirSync(srcPath, { recursive: true });
            return srcPath;
        } catch {
            return undefined;
        }
    }

    private findPackageXmlFiles(dir: string, depth: number): string[] {
        if (depth < 0) {
            return [];
        }

        const results: string[] = [];
        let entries: fs.Dirent[] = [];

        try {
            entries = fs.readdirSync(dir, { withFileTypes: true });
        } catch {
            return results;
        }

        for (const entry of entries) {
            const fullPath = path.join(dir, entry.name);

            if (entry.isDirectory()) {
                if (entry.name.startsWith('.') || ['build', 'install', 'log', 'node_modules'].includes(entry.name)) {
                    continue;
                }
                results.push(...this.findPackageXmlFiles(fullPath, depth - 1));
            } else if (entry.isFile() && entry.name === 'package.xml') {
                results.push(fullPath);
            }
        }

        return results;
    }

    private findLaunchFiles(packagePath: string, depth: number): string[] {
        const launchDir = path.join(packagePath, 'launch');
        if (!fs.existsSync(launchDir)) {
            return [];
        }

        const results: string[] = [];
        const allowedExt = new Set(['.py', '.xml', '.launch', '.yaml', '.yml']);

        const walk = (dir: string, d: number) => {
            if (d < 0) {
                return;
            }
            let entries: fs.Dirent[] = [];
            try {
                entries = fs.readdirSync(dir, { withFileTypes: true });
            } catch {
                return;
            }

            for (const entry of entries) {
                const fullPath = path.join(dir, entry.name);
                if (entry.isDirectory()) {
                    if (entry.name.startsWith('.')) {
                        continue;
                    }
                    walk(fullPath, d - 1);
                } else if (entry.isFile()) {
                    const ext = path.extname(entry.name);
                    if (allowedExt.has(ext)) {
                        results.push(fullPath);
                    }
                }
            }
        };

        walk(launchDir, depth);
        return results.sort();
    }

    private parsePythonLaunchArgs(content: string): LaunchArgOption[] {
        const results: LaunchArgOption[] = [];
        const regex = /DeclareLaunchArgument\(\s*['"]([^'"]+)['"][^\)]*\)/g;

        let match: RegExpExecArray | null;
        while ((match = regex.exec(content)) !== null) {
            const block = match[0];
            const name = match[1];

            const defaultMatch = block.match(/default_value\s*=\s*([^,\)]+)/);
            const defaultValue = defaultMatch
                ? defaultMatch[1].trim().replace(/^['"]|['"]$/g, '')
                : undefined;

            results.push({ name, defaultValue });
        }

        return this.deduplicateArgs(results);
    }

    private parseXmlLaunchArgs(content: string): LaunchArgOption[] {
        const results: LaunchArgOption[] = [];
        const regex = /<arg\s+[^>]*name=["']([^"']+)["'][^>]*>/g;

        let match: RegExpExecArray | null;
        while ((match = regex.exec(content)) !== null) {
            const tag = match[0];
            const name = match[1];
            const defaultMatch = tag.match(/default=["']([^"']+)["']/);
            const defaultValue = defaultMatch ? defaultMatch[1] : undefined;

            results.push({ name, defaultValue });
        }

        return this.deduplicateArgs(results);
    }

    private deduplicateArgs(args: LaunchArgOption[]): LaunchArgOption[] {
        const seen = new Set<string>();
        const results: LaunchArgOption[] = [];
        for (const arg of args) {
            if (seen.has(arg.name)) {
                continue;
            }
            seen.add(arg.name);
            results.push(arg);
        }
        return results;
    }

    runInTerminal(cmd: string): void {
        const terminal = this.ensureTerminal();
        terminal.show();
        if (cmd.trim().length > 0) {
            const fullCmd = this.buildSourcedCommand(cmd);
            terminal.sendText(fullCmd);
        }
    }

    /**
     * Send a fully-formed command to the terminal WITHOUT prepending
     * the automatic source / overlay prefix.  Use this when the caller
     * has already embedded its own sourcing logic (e.g. buildPackages).
     */
    sendRawCommand(cmd: string): void {
        const terminal = this.ensureTerminal();
        terminal.show();
        if (cmd.trim().length > 0) {
            terminal.sendText(cmd);
        }
    }

    openSourcedTerminal(): void {
        const terminal = this.ensureTerminal();
        terminal.show();
    }

    private ensureTerminal(): vscode.Terminal {
        if (!this._terminal || this._terminal.exitStatus !== undefined) {
            const bashPath = this.resolveBashPath();
            const env = this.getTerminalEnv();
            this._terminal = bashPath
                ? vscode.window.createTerminal({
                    name: 'ROS Dev Toolkit',
                    shellPath: bashPath,
                    env,
                })
                : vscode.window.createTerminal({ name: 'ROS Dev Toolkit', env });
        }
        return this._terminal;
    }

    private getTerminalEnv(): Record<string, string> {
        return {};
    }

    private resolveBashPath(): string | undefined {
        const candidates = ['/bin/bash', '/usr/bin/bash'];
        for (const candidate of candidates) {
            if (fs.existsSync(candidate)) {
                return candidate;
            }
        }
        return undefined;
    }

    /**
     * Returns the `source …/setup.bash` string to prepend to every command.
     * Resolution order:
     *   1. User setting  rosDevToolkit.rosSetupPath
     *   2. Detected distro  /opt/ros/<distro>/setup.bash
     *   3. $ROS_DISTRO env var
     *   4. First distro found under /opt/ros
     */
    /**
     * Returns the `source …/setup.bash` string for the base ROS install.
     * Protected so that specialised command builders (e.g. buildPackages)
     * can reference it without going through the full overlay pipeline.
     */
    protected getRosSourceCommand(): string {
        const setupPath = this.resolveRosSetupPath();
        return `source "${setupPath}"`;
    }

    private getWorkspaceOverlayCommand(): string | undefined {
        const wsPath = this.getWorkspacePath();
        const ros2Overlay = path.join(wsPath, 'install', 'setup.bash');
        const ros1Overlay = path.join(wsPath, 'devel', 'setup.bash');

        if (fs.existsSync(ros2Overlay)) {
            return `source "${ros2Overlay}"`;
        }
        if (fs.existsSync(ros1Overlay)) {
            return `source "${ros1Overlay}"`;
        }
        return undefined;
    }

    private buildSourcedCommand(cmd: string): string {
        const parts: string[] = [this.getRosSourceCommand()];
        const overlay = this.getWorkspaceOverlayCommand();
        if (overlay) {
            parts.push(overlay);
        }
        parts.push(cmd);
        return parts.join(' && ');
    }

    private resolveRosSetupPath(): string {
        // 1. Explicit user setting
        const config = vscode.workspace.getConfiguration('rosDevToolkit');
        const configured = config.get<string>('rosSetupPath', '').trim();
        if (configured && fs.existsSync(configured)) {
            return configured;
        }

        // 2. Detected distro
        if (this._env?.distro) {
            const p = `/opt/ros/${this._env.distro}/setup.bash`;
            if (fs.existsSync(p)) { return p; }
        }

        // 3. $ROS_DISTRO env var
        const envDistro = process.env.ROS_DISTRO;
        if (envDistro) {
            const p = `/opt/ros/${envDistro}/setup.bash`;
            if (fs.existsSync(p)) { return p; }
        }

        // 4. First distro folder found under /opt/ros
        const rosRoot = '/opt/ros';
        if (fs.existsSync(rosRoot)) {
            try {
                const dirs = fs.readdirSync(rosRoot, { withFileTypes: true })
                    .filter(e => e.isDirectory())
                    .map(e => e.name)
                    .sort();
                for (const d of dirs) {
                    const p = path.join(rosRoot, d, 'setup.bash');
                    if (fs.existsSync(p)) { return p; }
                }
            } catch { /* ignore */ }
        }

        // Last resort – assume jazzy
        return '/opt/ros/jazzy/setup.bash';
    }

    private exec(cmd: string): Promise<string> {
        return new Promise((resolve, reject) => {
            const setupPath = this.resolveRosSetupPath();
            const sourced = this.buildSourcedCommand(cmd.replace(/'/g, "'\\''"));
            const finalCmd = `bash -c '${sourced}'`;

            cp.exec(finalCmd, { env: process.env }, (err, stdout, stderr) => {
                if (err) { reject(stderr || err.message); }
                else { resolve(stdout); }
            });
        });
    }
}
