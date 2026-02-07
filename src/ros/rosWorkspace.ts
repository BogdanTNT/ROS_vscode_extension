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

    killTrackedTerminal(id: string): void {
        const tracked = this._trackedTerminals.get(id);
        if (!tracked) {
            return;
        }

        if (tracked.kind === 'integrated') {
            if (tracked.terminal) {
                // Send Ctrl+C without an appended newline so it behaves
                // exactly like a manual keypress in the terminal.
                tracked.terminal.sendText('\x03', false);
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
        }

        tracked.lastUsed = Date.now();
        this._emitTerminalsChanged();
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

    // ── Graph data (nodes / topics) ────────────────────────────
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

    async getTopicList(): Promise<{ name: string; type: string }[]> {
        try {
            const raw = this.isRos2()
                ? await this.exec('ros2 topic list -t')
                : await this.exec('rostopic list -v');
            const lines = raw.trim().split('\n').filter(Boolean);
            return lines.map((line) => {
                // ROS 2 format: "/topic [type]"
                const match = line.match(/^(\S+)\s+\[(.+)\]$/);
                if (match) {
                    return { name: match[1], type: match[2] };
                }
                return { name: line.trim(), type: 'unknown' };
            });
        } catch {
            return [];
        }
    }

    async getNodeInfo(nodeName: string): Promise<{ publishers: string[]; subscribers: string[] }> {
        try {
            const raw = this.isRos2()
                ? await this.exec(`ros2 node info ${nodeName}`)
                : await this.exec(`rosnode info ${nodeName}`);

            const publishers: string[] = [];
            const subscribers: string[] = [];
            let section: 'pub' | 'sub' | null = null;

            for (const line of raw.split('\n')) {
                if (line.includes('Publishers:') || line.includes('Publish')) {
                    section = 'pub';
                } else if (line.includes('Subscribers:') || line.includes('Subscribe')) {
                    section = 'sub';
                } else if (line.includes('Service')) {
                    section = null;
                } else if (section && line.trim().startsWith('/')) {
                    const topic = line.trim().split(':')[0].trim();
                    if (section === 'pub') { publishers.push(topic); }
                    else { subscribers.push(topic); }
                }
            }
            return { publishers, subscribers };
        } catch {
            return { publishers: [], subscribers: [] };
        }
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
