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
import { WslPersistentGraphRunner } from './runtime/wslPersistentGraphRunner';

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
    isCmake: boolean;
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

export interface RosGraphSnapshotScope {
    nodes: boolean;
    topics: boolean;
    services: boolean;
    actions: boolean;
    parameters: boolean;
}

export interface RosGraphSnapshotSection<T> {
    ok: boolean;
    data: T;
    error?: string;
    warnings?: string[];
}

export interface RosGraphSnapshotResult {
    nodes: RosGraphSnapshotSection<string[]>;
    topics: RosGraphSnapshotSection<RosGraphEntityInfo[]>;
    services: RosGraphSnapshotSection<RosGraphEntityInfo[]>;
    actions: RosGraphSnapshotSection<RosGraphEntityInfo[]>;
    parameters: RosGraphSnapshotSection<RosParameterInfo[]>;
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

export interface RosActionGoalTemplateResult {
    success: boolean;
    actionName: string;
    actionType?: string;
    template?: string;
    error?: string;
}

export interface RosActionGoalResult {
    success: boolean;
    actionName: string;
    actionType?: string;
    error?: string;
}

export interface RosTopicMessageSubscription extends vscode.Disposable {
    topicName: string;
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
    canRelaunch?: boolean;
}

type TrackedTerminalReplay =
    | {
        kind: 'launch';
        pkg: string;
        launchFile: string;
        launchPath?: string;
        args: string;
        argsLabel?: string;
        runTarget: string;
    }
    | {
        kind: 'node';
        pkg: string;
        executable: string;
        nodePath?: string;
        args: string;
        argsLabel?: string;
        runTarget: string;
    }
    | {
        kind: 'sourcedTerminal';
        runTarget: string;
    };

export interface RunTerminalTargetOption {
    id: string;
    label: string;
    description: string;
}

interface TrackedTerminal extends TrackedTerminalInfo {
    terminal?: vscode.Terminal;
    childProcess?: cp.ChildProcess;
    pidFile?: string;
    innerPid?: number;
    wslIntegrated?: boolean;
    wslDistro?: string;
    wslPidFile?: string;
    /** Set after a Ctrl+C has been sent so the next kill force-closes. */
    ctrlCSent?: boolean;
    /** Long-lived interactive shell that should remain "running" after bootstrap commands finish. */
    persistentShell?: boolean;
    replay?: TrackedTerminalReplay;
}

interface OsReleaseInfo {
    name?: string;
    version?: string;
    versionId?: string;
    prettyName?: string;
}

interface WslDistroSummary {
    name: string;
    state: string;
    version: string;
    isDefault: boolean;
}

interface ExecFileResult {
    ok: boolean;
    stdout: string;
    stderr: string;
    error?: string;
}

interface CommandExecutionContext {
    useWsl: boolean;
    runTarget: string;
    wslDistro?: string;
}

type RosNodeTemplateKind =
    | 'none'
    | 'publisher'
    | 'subscriber'
    | 'service'
    | 'client'
    | 'timer';

type GraphSnapshotKey = keyof RosGraphSnapshotScope;

interface GraphSnapshotParsedSection {
    status: number;
    output: string;
}

const DEFAULT_ROS_GRAPH_LIST_TIMEOUT_SECONDS = 6;
const DEFAULT_ROS_NODE_INFO_TIMEOUT_SECONDS = 3;
const DEFAULT_ROS_TOPIC_INFO_TIMEOUT_SECONDS = 3;
const ROS_GRAPH_LIST_TIMEOUT_SETTING = 'graphListTimeoutSeconds';
const ROS_NODE_INFO_TIMEOUT_SETTING = 'nodeInfoTimeoutSeconds';
const ROS_TOPIC_INFO_TIMEOUT_SETTING = 'topicInfoTimeoutSeconds';
const ROS_NODE_VISUALIZER_PERSISTENT_WSL_SHELL_SETTING = 'nodeVisualizerUsePersistentWslShell';
const DEFAULT_MAINTAINER_NAME_SETTING = 'defaultMaintainerName';
const DEFAULT_MAINTAINER_EMAIL_SETTING = 'defaultMaintainerEmail';
const DEFAULT_CREATE_PACKAGE_LICENSE = 'GPL-3.0';
const DEFAULT_CREATE_PACKAGE_DESCRIPTION = 'TO DO: A very good package description';

/**
 * Central helper that interacts with the ROS CLI tools installed on the host.
 * All shell calls go through here so views stay decoupled from the system.
 */
export class RosWorkspace {
    private _env: RosEnvironmentInfo | undefined;
    private _terminal: vscode.Terminal | undefined;
    private _terminalRunTarget: string | undefined;
    private _trackedTerminals: Map<string, TrackedTerminal> = new Map();
    private _terminalSeq = 1;
    private _preferredTerminalId: string | undefined;
    private _runTerminalTarget = 'auto';
    private _pendingTrackedTerminalReplay: TrackedTerminalReplay | undefined;
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
    private _wslGraphRunner?: WslPersistentGraphRunner;
    private _wslGraphRunnerDistro?: string;

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
            for (const [id, tracked] of this._trackedTerminals.entries()) {
                if (tracked.kind === 'integrated' && tracked.terminal === event.terminal && tracked.status === 'running') {
                    if (tracked.persistentShell) {
                        this._emitTerminalsChanged();
                        break;
                    }
                    tracked.status = 'closed';
                    if (tracked.ctrlCSent && tracked.terminal) {
                        tracked.terminal.dispose();
                        this._trackedTerminals.delete(id);
                        if (this._preferredTerminalId === id) {
                            this._preferredTerminalId = undefined;
                        }
                    }
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

    async getEnvironmentInfoReport(): Promise<string> {
        const platform = process.platform;
        const remoteName = vscode.env.remoteName || 'none';
        const wsPath = this.getWorkspacePath();
        const lines: string[] = [
            `Runtime platform: ${platform}`,
            `VS Code remote: ${remoteName}`,
            `Workspace path: ${wsPath}`,
            `Runtime mode: ${this.describeRuntimeMode(platform, vscode.env.remoteName)}`,
        ];

        if (platform === 'linux') {
            const osInfo = this.readOsReleaseFile('/etc/os-release');
            lines.push(`Linux distro: ${osInfo ? this.formatLinuxDistro(osInfo) : 'unavailable'}`);
            lines.push(`ROS environment vars: ${this.formatRosEnvironment(process.env.ROS_DISTRO, process.env.ROS_VERSION)}`);
            const rosInstalls = this.listRosInstallations('/opt/ros');
            lines.push(`ROS installs in /opt/ros: ${rosInstalls.length > 0 ? rosInstalls.join(', ') : 'none detected'}`);

            if (vscode.env.remoteName?.startsWith('wsl+')) {
                lines.push(`WSL distro (remote): ${vscode.env.remoteName.slice(4)}`);
            } else {
                lines.push(`WSL kernel detected: ${this.isWslKernel() ? 'yes' : 'no'}`);
            }

            return lines.join('\n');
        }

        if (platform === 'win32') {
            lines.push(`Windows ROS environment vars: ${this.formatRosEnvironment(process.env.ROS_DISTRO, process.env.ROS_VERSION)}`);

            const listResult = await this.execFileSafe('wsl.exe', ['-l', '-v'], 5000);
            if (!listResult.ok) {
                const reason = listResult.stderr.trim() || listResult.stdout.trim() || listResult.error || 'wsl.exe is not available';
                lines.push(`WSL probe: unavailable (${reason})`);
                return lines.join('\n');
            }

            const distros = this.parseWslList(listResult.stdout);
            if (distros.length === 0) {
                lines.push('WSL distros: none detected');
                const stderr = listResult.stderr.trim();
                if (stderr) {
                    lines.push(`WSL detail: ${stderr}`);
                }
                return lines.join('\n');
            }

            lines.push(`WSL distros detected: ${distros.length}`);
            for (const distro of distros) {
                const prefix = distro.isDefault ? '* ' : '- ';
                lines.push(`${prefix}${distro.name} [state: ${distro.state}, version: WSL${distro.version}]`);

                const probe = await this.probeWslDistro(distro.name);
                if (probe.error) {
                    lines.push(`  Probe error: ${probe.error}`);
                    continue;
                }

                lines.push(`  Linux: ${probe.linuxDistro || 'unavailable'}`);
                lines.push(`  ROS environment vars: ${this.formatRosEnvironment(probe.rosDistro, probe.rosVersion)}`);
                lines.push(`  /opt/ros distros: ${probe.rosInstalls.length > 0 ? probe.rosInstalls.join(', ') : 'none detected'}`);
            }

            return lines.join('\n');
        }

        lines.push(`ROS environment vars: ${this.formatRosEnvironment(process.env.ROS_DISTRO, process.env.ROS_VERSION)}`);
        return lines.join('\n');
    }

    async listRunTerminalTargets(): Promise<RunTerminalTargetOption[]> {
        const options: RunTerminalTargetOption[] = [
            {
                id: 'auto',
                label: 'Auto (use extension setting)',
                description: 'Use rosDevToolkit.launchInExternalTerminal to choose integrated vs external terminal.',
            },
        ];

        if (process.platform !== 'win32') {
            options.push({
                id: 'integrated',
                label: 'VS Code integrated terminal',
                description: 'Always run launch/node actions inside a VS Code integrated terminal tab.',
            });
            options.push({
                id: 'external',
                label: 'External terminal window',
                description: 'Always run launch/node actions in a native external terminal window.',
            });
            return options;
        }

        const wslList = await this.execFileSafe('wsl.exe', ['-l', '-v'], 5000);
        if (!wslList.ok) {
            options.push({
                id: 'integrated',
                label: 'VS Code integrated terminal',
                description: 'Run launch/node actions in the current VS Code terminal shell.',
            });
            options.push({
                id: 'external',
                label: 'External terminal window',
                description: 'Run launch/node actions in a native external terminal window.',
            });
            return options;
        }

        const distros = this.parseWslList(wslList.stdout);
        if (!distros.length) {
            options.push({
                id: 'integrated',
                label: 'VS Code integrated terminal',
                description: 'Run launch/node actions in the current VS Code terminal shell.',
            });
            options.push({
                id: 'external',
                label: 'External terminal window',
                description: 'Run launch/node actions in a native external terminal window.',
            });
            return options;
        }

        options.push({
            id: 'wsl-integrated:default',
            label: 'WSL (default distro) in VS Code terminal',
            description: 'Run commands in the default WSL distro inside a VS Code terminal tab.',
        });
        options.push({
            id: 'wsl-external:default',
            label: 'WSL (default distro) in external terminal',
            description: 'Open a native terminal window in the default WSL distro and run commands with bash.',
        });
        for (const distro of distros) {
            const distroName = this.sanitizeWslDistroName(distro.name);
            if (!distroName) {
                continue;
            }
            options.push({
                id: `wsl-integrated:${distroName}`,
                label: `WSL (${distroName}) in VS Code terminal`,
                description: `Run commands in ${distroName} inside a VS Code terminal tab.`,
            });
            options.push({
                id: `wsl-external:${distroName}`,
                label: `WSL (${distroName}) in external terminal`,
                description: `Open a native terminal window in ${distroName} and run commands with bash.`,
            });
        }

        return options;
    }

    setRunTerminalTarget(target: string): void {
        this._runTerminalTarget = this.normalizeRunTerminalTarget(target);
    }

    getRunTerminalTarget(): string {
        return this._runTerminalTarget;
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
        const workspacePath = this.getWorkspacePath();
        if (!workspacePath || !fs.existsSync(workspacePath)) {
            return [];
        }

        const packageXmlFiles = this.findPackageXmlFiles(workspacePath, 6);
        const names = new Set<string>();

        for (const file of packageXmlFiles) {
            const packageName = this.readPackageNameFromPackageXml(file);
            if (packageName) {
                names.add(packageName);
            }
        }

        return Array.from(names).sort();
    }

    async listWorkspacePackageDetails(): Promise<RosWorkspacePackageDetails[]> {
        const workspacePath = this.getWorkspacePath();
        if (!workspacePath || !fs.existsSync(workspacePath)) {
            return [];
        }

        const packageXmlFiles = this.findPackageXmlFiles(workspacePath, 6);
        const packages: RosWorkspacePackageDetails[] = [];
        const seen = new Set<string>();

        for (const file of packageXmlFiles) {
            const name = this.readPackageNameFromPackageXml(file);
            if (!name || seen.has(name)) {
                continue;
            }
            const packagePath = path.dirname(file);
            const launchFiles = this.findLaunchFiles(packagePath, 3);
            const nodes = this.findRunnableNodes(packagePath);
            const isPython = fs.existsSync(path.join(packagePath, 'setup.py'));
            const isCmake = fs.existsSync(path.join(packagePath, 'CMakeLists.txt'));
            packages.push({
                name,
                packagePath,
                launchFiles,
                nodes,
                isPython,
                isCmake,
            });
            seen.add(name);
        }

        return packages.sort((a, b) => a.name.localeCompare(b.name));
    }

    async listPackageDetails(packageNames: string[]): Promise<RosWorkspacePackageDetails[]> {
        const normalizedNames = Array.from(new Set(
            packageNames
                .map((name) => this.sanitizeRosPackageName(name))
                .filter((name): name is string => Boolean(name)),
        )).sort((a, b) => a.localeCompare(b));

        return Promise.all(normalizedNames.map(async (packageName) => {
            const packagePath = await this.resolvePackagePath(packageName);
            const launchFiles = packagePath ? this.findLaunchFiles(packagePath, 3) : [];
            const localNodes = packagePath ? this.findRunnableNodes(packagePath) : [];
            const rosCliNodes = await this.listRosCliRunnableNodes(packageName);
            const mergedNodes = new Map<string, RosRunnableNodeInfo>();

            for (const node of [...localNodes, ...rosCliNodes]) {
                if (!node?.name) {
                    continue;
                }
                const existing = mergedNodes.get(node.name);
                if (!existing) {
                    mergedNodes.set(node.name, node);
                    continue;
                }
                if (!existing.sourcePath && node.sourcePath) {
                    mergedNodes.set(node.name, { ...existing, sourcePath: node.sourcePath });
                }
            }

            const nodes = Array.from(mergedNodes.values()).sort((a, b) => a.name.localeCompare(b.name));
            const isPython = packagePath ? fs.existsSync(path.join(packagePath, 'setup.py')) : false;
            const isCmake = packagePath ? fs.existsSync(path.join(packagePath, 'CMakeLists.txt')) : false;

            return {
                name: packageName,
                packagePath: packagePath || '',
                launchFiles,
                nodes,
                isPython,
                isCmake,
            };
        }));
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

    async createPackage(
        name: string,
        buildType: string,
        deps: string[],
        license?: string,
        description?: string,
    ): Promise<boolean> {
        const srcDir = this.getSrcDir();
        if (!srcDir) {
            vscode.window.showErrorMessage('Could not determine workspace src directory.');
            return false;
        }
        const context = this.resolveCommandExecutionContext();
        const targetSrcDir = context.useWsl
            ? `${this.getWorkspacePathForRunTarget(context.runTarget)}/src`
            : srcDir;

        // Keep package names ROS-friendly even if callers pass whitespace.
        const normalizedName = String(name || '').trim().replace(/\s+/g, '_');
        if (!normalizedName) {
            vscode.window.showErrorMessage('Package name cannot be empty.');
            return false;
        }

        const maintainer = await this.resolveCreatePackageMaintainerIdentity();
        if (!maintainer) {
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
        const maintainerNameArg = this.escapeShellArg(maintainer.name);
        const maintainerEmailArg = this.escapeShellArg(maintainer.email);
        const ros1MaintainerArg = this.escapeShellArg(`${maintainer.name} <${maintainer.email}>`);
        const normalizedLicense = String(license || '').trim() || DEFAULT_CREATE_PACKAGE_LICENSE;
        const normalizedDescription = String(description || '').trim() || DEFAULT_CREATE_PACKAGE_DESCRIPTION;
        const licenseArg = this.escapeShellArg(normalizedLicense);
        const descriptionArg = this.escapeShellArg(normalizedDescription);
        const ros1DepsArg = mergedDeps.join(' ');

        const cmd = this.isRos2()
            ? `cd "${targetSrcDir}" && ros2 pkg create ${normalizedName} --build-type ${buildType}${depString} --maintainer-name ${maintainerNameArg} --maintainer-email ${maintainerEmailArg} --license ${licenseArg} --description ${descriptionArg}`
            : `cd "${targetSrcDir}" && catkin_create_pkg -m ${ros1MaintainerArg} --license ${licenseArg} --description ${descriptionArg} ${normalizedName}${ros1DepsArg ? ` ${ros1DepsArg}` : ''}`;

        this.runInTerminal(cmd);
        return true;
    }

    private async resolveCreatePackageMaintainerIdentity(): Promise<{ name: string; email: string } | undefined> {
        const config = vscode.workspace.getConfiguration('rosDevToolkit');
        const configuredName = config.get<string>(DEFAULT_MAINTAINER_NAME_SETTING, '').trim();
        const configuredEmail = config.get<string>(DEFAULT_MAINTAINER_EMAIL_SETTING, '').trim();

        let maintainerName = configuredName || this.getGitConfigValue('user.name');
        let maintainerEmail = configuredEmail || this.getGitConfigValue('user.email');
        let prompted = false;

        if (!maintainerName) {
            const enteredName = await vscode.window.showInputBox({
                title: 'Default ROS Maintainer',
                prompt: 'Enter maintainer name for new packages',
                placeHolder: 'Jane Doe',
                ignoreFocusOut: true,
                value: '',
                validateInput: (value) => value.trim() ? undefined : 'Maintainer name is required.',
            });
            if (enteredName === undefined) {
                return undefined;
            }
            maintainerName = enteredName.trim();
            prompted = true;
        }

        if (!maintainerEmail) {
            const enteredEmail = await vscode.window.showInputBox({
                title: 'Default ROS Maintainer',
                prompt: 'Enter maintainer email for new packages',
                placeHolder: 'jane@example.com',
                ignoreFocusOut: true,
                value: '',
                validateInput: (value) => {
                    const trimmed = value.trim();
                    if (!trimmed) {
                        return 'Maintainer email is required.';
                    }
                    if (!/^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(trimmed)) {
                        return 'Enter a valid email address.';
                    }
                    return undefined;
                },
            });
            if (enteredEmail === undefined) {
                return undefined;
            }
            maintainerEmail = enteredEmail.trim();
            prompted = true;
        }

        if (prompted) {
            await Promise.all([
                config.update(DEFAULT_MAINTAINER_NAME_SETTING, maintainerName, vscode.ConfigurationTarget.Global),
                config.update(DEFAULT_MAINTAINER_EMAIL_SETTING, maintainerEmail, vscode.ConfigurationTarget.Global),
            ]);
        }

        return { name: maintainerName, email: maintainerEmail };
    }

    private getGitConfigValue(key: string): string {
        const workspacePath = this.getWorkspacePath();
        const read = (args: string[]): string => {
            try {
                return cp.execFileSync('git', args, {
                    encoding: 'utf8',
                    stdio: ['ignore', 'pipe', 'ignore'],
                }).trim();
            } catch {
                return '';
            }
        };

        if (workspacePath) {
            const localValue = read(['-C', workspacePath, 'config', '--get', key]);
            if (localValue) {
                return localValue;
            }
        }

        return read(['config', '--global', '--get', key]);
    }

    /**
     * Create a launch file in an existing workspace package and ensure it can
     * be installed by the package build system.
     */
    async createLaunchFileInPackage(
        packageName: string,
        launchName: string,
        packagePathHint?: string,
    ): Promise<boolean> {
        const safePackageName = this.sanitizeRosPackageName(packageName);
        if (!safePackageName) {
            vscode.window.showErrorMessage(
                'Invalid package name. Use only letters, numbers, and underscores, and start with a letter.',
            );
            return false;
        }

        const safeLaunchFileName = this.normalizeLaunchFileName(launchName);
        if (!safeLaunchFileName) {
            vscode.window.showErrorMessage(
                'Invalid launch file name. Use letters, numbers, underscores, dashes, and dots only.',
            );
            return false;
        }

        const pkgDir = this.resolveWorkspacePackagePath(safePackageName, packagePathHint);
        if (!pkgDir) {
            vscode.window.showErrorMessage(`Package folder not found for "${safePackageName}" in this workspace.`);
            return false;
        }

        const setupPyPath = path.join(pkgDir, 'setup.py');
        const setupCfgPath = path.join(pkgDir, 'setup.cfg');
        const cmakePath = path.join(pkgDir, 'CMakeLists.txt');
        const hasPythonBuildFiles = fs.existsSync(setupPyPath) || fs.existsSync(setupCfgPath);
        const hasCmakeBuildFile = fs.existsSync(cmakePath);

        if (!hasPythonBuildFiles && !hasCmakeBuildFile) {
            vscode.window.showErrorMessage(
                `Package "${safePackageName}" is not supported for create-launch (missing setup.py/setup.cfg and CMakeLists.txt).`,
            );
            return false;
        }

        const launchDir = path.join(pkgDir, 'launch');
        const launchFilePath = path.resolve(path.join(launchDir, safeLaunchFileName));
        if (!this.isPathInsideDirectory(launchFilePath, pkgDir)) {
            vscode.window.showErrorMessage('Launch file path must stay inside the selected package.');
            return false;
        }

        try {
            fs.mkdirSync(launchDir, { recursive: true });
        } catch (err) {
            vscode.window.showErrorMessage(`Failed to create launch directory: ${err}`);
            return false;
        }

        if (!fs.existsSync(launchFilePath)) {
            const template = this.buildLaunchFileTemplate(safePackageName, safeLaunchFileName);
            try {
                fs.writeFileSync(launchFilePath, template, 'utf8');
            } catch (err) {
                vscode.window.showErrorMessage(`Failed to create launch file: ${err}`);
                return false;
            }
        }

        if (hasPythonBuildFiles) {
            if (fs.existsSync(setupPyPath)) {
                try {
                    const setupPy = fs.readFileSync(setupPyPath, 'utf8');
                    const updated = this.ensurePythonLaunchInstallInSetupPy(setupPy, safePackageName);
                    if (updated !== setupPy) {
                        fs.writeFileSync(setupPyPath, updated, 'utf8');
                    }
                } catch (err) {
                    vscode.window.showErrorMessage(`Failed to update setup.py: ${err}`);
                    return false;
                }
            } else if (fs.existsSync(setupCfgPath)) {
                try {
                    const setupCfg = fs.readFileSync(setupCfgPath, 'utf8');
                    const updated = this.ensurePythonLaunchInstallInSetupCfg(setupCfg, safePackageName);
                    if (updated !== setupCfg) {
                        fs.writeFileSync(setupCfgPath, updated, 'utf8');
                    }
                } catch (err) {
                    vscode.window.showErrorMessage(`Failed to update setup.cfg: ${err}`);
                    return false;
                }
            }
        }

        if (hasCmakeBuildFile) {
            try {
                const cmake = fs.readFileSync(cmakePath, 'utf8');
                const updated = this.ensureLaunchDirectoryInstalledInCmake(cmake);
                if (updated !== cmake) {
                    fs.writeFileSync(cmakePath, updated, 'utf8');
                }
            } catch (err) {
                vscode.window.showErrorMessage(`Failed to update CMakeLists.txt: ${err}`);
                return false;
            }
        }

        const uri = vscode.Uri.file(launchFilePath);
        await vscode.window.showTextDocument(uri, { preview: false });
        return true;
    }

    async removeLaunchFileFromPackage(
        packageName: string,
        launchName: string,
        packagePathHint?: string,
        launchPathHint?: string,
    ): Promise<boolean> {
        const safePackageName = this.sanitizeRosPackageName(packageName);
        if (!safePackageName) {
            vscode.window.showErrorMessage(
                'Invalid package name. Use only letters, numbers, and underscores, and start with a letter.',
            );
            return false;
        }

        const safeLaunchFileName = this.sanitizeExistingLaunchFileName(launchName);
        if (!safeLaunchFileName) {
            vscode.window.showErrorMessage(
                'Invalid launch file name. Use a supported launch file with a valid extension.',
            );
            return false;
        }

        const pkgDir = this.resolveWorkspacePackagePath(safePackageName, packagePathHint);
        if (!pkgDir) {
            vscode.window.showErrorMessage(`Package folder not found for "${safePackageName}" in this workspace.`);
            return false;
        }

        const setupPyPath = path.join(pkgDir, 'setup.py');
        const setupCfgPath = path.join(pkgDir, 'setup.cfg');
        const cmakePath = path.join(pkgDir, 'CMakeLists.txt');
        const hasPythonBuildFiles = fs.existsSync(setupPyPath) || fs.existsSync(setupCfgPath);
        const hasCmakeBuildFile = fs.existsSync(cmakePath);

        if (!hasPythonBuildFiles && !hasCmakeBuildFile) {
            vscode.window.showErrorMessage(
                `Package "${safePackageName}" is not supported for remove-launch (missing setup.py/setup.cfg and CMakeLists.txt).`,
            );
            return false;
        }

        const launchCandidates = this.resolveSafeLaunchFileCandidates(
            pkgDir,
            launchPathHint,
            [safeLaunchFileName],
        );
        for (const launchCandidate of launchCandidates) {
            if (this.tryDeleteSourceFile(launchCandidate)) {
                return true;
            }
        }

        vscode.window.showWarningMessage(
            `Launch file "${safeLaunchFileName}" was not found in package "${safePackageName}".`,
        );
        return false;
    }

    private normalizeLaunchFileName(name: string): string | undefined {
        const normalized = String(name || '').trim().replace(/\s+/g, '_');
        if (!normalized) {
            return undefined;
        }
        if (normalized.includes('/') || normalized.includes('\\')) {
            return undefined;
        }
        if (!/^[A-Za-z0-9][A-Za-z0-9_.-]*$/.test(normalized)) {
            return undefined;
        }

        const normalizedLower = normalized.toLowerCase();
        const knownSuffixes = ['.launch.py', '.launch.xml', '.launch.yaml', '.launch.yml', '.py', '.xml', '.launch', '.yaml', '.yml'];
        if (knownSuffixes.some((suffix) => normalizedLower.endsWith(suffix))) {
            const ext = path.extname(normalizedLower);
            const allowedExt = new Set(['.py', '.xml', '.launch', '.yaml', '.yml']);
            return allowedExt.has(ext) ? normalized : undefined;
        }

        if (path.extname(normalized)) {
            return undefined;
        }
        return `${normalized}.launch.py`;
    }

    private sanitizeExistingLaunchFileName(name: string): string | undefined {
        const normalized = String(name || '').trim().replace(/\s+/g, '_');
        if (!normalized) {
            return undefined;
        }
        if (normalized.includes('/') || normalized.includes('\\')) {
            return undefined;
        }
        if (!/^[A-Za-z0-9][A-Za-z0-9_.-]*$/.test(normalized)) {
            return undefined;
        }
        const allowedExt = new Set(['.py', '.xml', '.launch', '.yaml', '.yml']);
        const ext = path.extname(normalized).toLowerCase();
        if (!allowedExt.has(ext)) {
            return undefined;
        }
        return normalized;
    }

    private buildLaunchFileTemplate(packageName: string, launchFileName: string): string {
        const lower = launchFileName.toLowerCase();
        if (lower.endsWith('.xml') || lower.endsWith('.launch')) {
            return `<launch>
  <!-- Add launch actions for package "${packageName}" here. -->
</launch>
`;
        }

        if (lower.endsWith('.yaml') || lower.endsWith('.yml')) {
            return `launch:
  # Add launch configuration for package "${packageName}" here.
`;
        }

        return `from launch import LaunchDescription


def generate_launch_description():
    # Add actions for package "${packageName}" here.
    return LaunchDescription([])
`;
    }

    private ensurePythonLaunchInstallInSetupPy(setupPy: string, packageName: string): string {
        let updated = setupPy;
        const hasFromGlobImport = /^\s*from\s+glob\s+import\s+glob\b/m.test(updated);
        if (!hasFromGlobImport) {
            updated = `from glob import glob\n${updated}`;
        }

        const escapedPkgName = this.escapeRegex(packageName);
        const hasLaunchDataFilesEntry = new RegExp(`['"]share/${escapedPkgName}/launch['"]`).test(updated);
        if (hasLaunchDataFilesEntry) {
            return updated;
        }

        const launchEntry = `        ('share/${packageName}/launch', glob('launch/*')),`;
        const dataFilesBlockPattern = /(data_files\s*=\s*\[)([\s\S]*?)(\]\s*,)/m;
        if (dataFilesBlockPattern.test(updated)) {
            return updated.replace(
                dataFilesBlockPattern,
                (_full, prefix: string, body: string, suffix: string) => {
                    const trimmedBody = body.replace(/\s*$/, '');
                    const nextBody = trimmedBody
                        ? `${trimmedBody}\n${launchEntry}\n`
                        : `\n${launchEntry}\n`;
                    return `${prefix}${nextBody}${suffix}`;
                },
            );
        }

        const setupCallPattern = /setup\s*\(\s*/m;
        if (setupCallPattern.test(updated)) {
            return updated.replace(
                setupCallPattern,
                (match) => `${match}data_files=[\n${launchEntry}\n    ],\n    `,
            );
        }
        return updated;
    }

    private ensurePythonLaunchInstallInSetupCfg(setupCfg: string, packageName: string): string {
        const lineEnding = setupCfg.includes('\r\n') ? '\r\n' : '\n';
        const launchKey = `share/${packageName}/launch`;
        const launchKeyPattern = new RegExp(`^\\s*${this.escapeRegex(launchKey)}\\s*=`, 'm');
        if (launchKeyPattern.test(setupCfg)) {
            return setupCfg;
        }

        const lines = setupCfg.split(/\r?\n/);
        const dataFilesSectionIndex = lines.findIndex((line) => /^\s*\[options\.data_files\]\s*$/i.test(line));
        if (dataFilesSectionIndex < 0) {
            const trimmed = setupCfg.trimEnd();
            const separator = trimmed ? `${lineEnding}${lineEnding}` : '';
            return `${trimmed}${separator}[options.data_files]${lineEnding}${launchKey} =${lineEnding}    launch/*${lineEnding}`;
        }

        let insertIndex = lines.length;
        for (let idx = dataFilesSectionIndex + 1; idx < lines.length; idx += 1) {
            if (/^\s*\[.*\]\s*$/.test(lines[idx])) {
                insertIndex = idx;
                break;
            }
        }

        lines.splice(
            insertIndex,
            0,
            `${launchKey} =`,
            '    launch/*',
        );
        return lines.join(lineEnding);
    }

    private ensureLaunchDirectoryInstalledInCmake(cmake: string): string {
        const hasLaunchInstall = /install\s*\(\s*DIRECTORY\s+launch\b[\s\S]*?DESTINATION\s+share\s*\/\s*\$\{PROJECT_NAME\}[\s\S]*?\)/im.test(cmake);
        if (hasLaunchInstall) {
            return cmake;
        }
        const installBlock = [
            'install(',
            '  DIRECTORY launch',
            '  DESTINATION share/${PROJECT_NAME}',
            ')',
        ].join('\n');
        return this.insertBeforeAmentPackage(cmake, installBlock);
    }

    private resolveSafeLaunchFileCandidates(
        packageDir: string,
        launchPathHint: string | undefined,
        launchFileNames: string[],
    ): string[] {
        const launchDir = path.join(packageDir, 'launch');
        const candidates: string[] = [];
        const addCandidate = (candidatePath: string) => {
            if (!candidatePath) {
                return;
            }
            const resolved = path.resolve(candidatePath);
            if (!this.isPathInsideDirectory(resolved, launchDir)) {
                return;
            }
            if (!candidates.includes(resolved)) {
                candidates.push(resolved);
            }
        };

        const trimmedHint = String(launchPathHint || '').trim();
        if (trimmedHint) {
            addCandidate(trimmedHint);
        }
        launchFileNames.forEach((fileName) => addCandidate(path.join(launchDir, fileName)));
        return candidates;
    }

    /**
     * Add a new node to an existing workspace package.
     * Supports both ament_python (setup.py) and ament_cmake (CMakeLists.txt).
     */
    async addNodeToPackage(
        packageName: string,
        nodeName: string,
        packagePathHint?: string,
        nodeTemplate?: string,
        nodeTopic?: string,
    ): Promise<boolean> {
        const safePackageName = this.sanitizeRosPackageName(packageName);
        if (!safePackageName) {
            vscode.window.showErrorMessage(
                'Invalid package name. Use only letters, numbers, and underscores, and start with a letter.',
            );
            return false;
        }

        const safeNodeName = this.sanitizeRosNodeName(nodeName);
        if (!safeNodeName) {
            vscode.window.showErrorMessage(
                'Invalid node name. Use only letters, numbers, and underscores, and start with a letter.',
            );
            return false;
        }
        const templateKind = this.normalizeNodeTemplateKind(nodeTemplate);
        const normalizedTemplateTopic = this.normalizeNodeTemplateTopic(nodeTopic);

        const pkgDir = this.resolveWorkspacePackagePath(safePackageName, packagePathHint);
        if (!pkgDir) {
            vscode.window.showErrorMessage(`Package folder not found for "${safePackageName}" in this workspace.`);
            return false;
        }

        const setupPyPath = path.join(pkgDir, 'setup.py');
        if (fs.existsSync(setupPyPath)) {
            return this.addPythonNodeToPackage(
                pkgDir,
                safePackageName,
                safeNodeName,
                setupPyPath,
                templateKind,
                normalizedTemplateTopic,
            );
        }

        const cmakePath = path.join(pkgDir, 'CMakeLists.txt');
        if (fs.existsSync(cmakePath)) {
            return this.addCppNodeToPackage(
                pkgDir,
                safeNodeName,
                cmakePath,
                templateKind,
                normalizedTemplateTopic,
            );
        }

        vscode.window.showErrorMessage(
            `Package "${safePackageName}" is not supported for add-node (missing setup.py or CMakeLists.txt).`,
        );
        return false;
    }

    /**
     * Remove a node from an existing workspace package.
     * Supports both ament_python (setup.py/setup.cfg) and ament_cmake (CMakeLists.txt).
     */
    async removeNodeFromPackage(
        packageName: string,
        nodeName: string,
        packagePathHint?: string,
        nodeSourcePathHint?: string,
    ): Promise<boolean> {
        const safePackageName = this.sanitizeRosPackageName(packageName);
        if (!safePackageName) {
            vscode.window.showErrorMessage(
                'Invalid package name. Use only letters, numbers, and underscores, and start with a letter.',
            );
            return false;
        }

        const safeNodeName = this.sanitizeRosNodeName(nodeName);
        if (!safeNodeName) {
            vscode.window.showErrorMessage(
                'Invalid node name. Use only letters, numbers, and underscores, and start with a letter.',
            );
            return false;
        }

        const pkgDir = this.resolveWorkspacePackagePath(safePackageName, packagePathHint);
        if (!pkgDir) {
            vscode.window.showErrorMessage(`Package folder not found for "${safePackageName}" in this workspace.`);
            return false;
        }

        const setupPyPath = path.join(pkgDir, 'setup.py');
        const setupCfgPath = path.join(pkgDir, 'setup.cfg');
        const cmakePath = path.join(pkgDir, 'CMakeLists.txt');
        const hasPythonBuildFiles = fs.existsSync(setupPyPath) || fs.existsSync(setupCfgPath);
        const hasCmakeBuildFile = fs.existsSync(cmakePath);

        // Source hint can disambiguate mixed packages; fall back to trying Python first,
        // then CMake if nothing was changed.
        const normalizedHintExt = path.extname(String(nodeSourcePathHint || '').trim()).toLowerCase();
        const preferCmake = normalizedHintExt === '.cpp'
            || normalizedHintExt === '.cc'
            || normalizedHintExt === '.cxx'
            || normalizedHintExt === '.c';
        const preferPython = normalizedHintExt === '.py';

        if (preferCmake && hasCmakeBuildFile) {
            if (await this.removeCppNodeFromPackage(pkgDir, safeNodeName, cmakePath, nodeSourcePathHint)) {
                return true;
            }
            if (hasPythonBuildFiles) {
                if (await this.removePythonNodeFromPackage(
                    pkgDir,
                    safePackageName,
                    safeNodeName,
                    setupPyPath,
                    setupCfgPath,
                    nodeSourcePathHint,
                )) {
                    return true;
                }
            }
        } else if (preferPython && hasPythonBuildFiles) {
            if (await this.removePythonNodeFromPackage(
                pkgDir,
                safePackageName,
                safeNodeName,
                setupPyPath,
                setupCfgPath,
                nodeSourcePathHint,
            )) {
                return true;
            }
            if (hasCmakeBuildFile) {
                if (await this.removeCppNodeFromPackage(pkgDir, safeNodeName, cmakePath, nodeSourcePathHint)) {
                    return true;
                }
            }
        } else {
            if (hasPythonBuildFiles) {
                if (await this.removePythonNodeFromPackage(
                    pkgDir,
                    safePackageName,
                    safeNodeName,
                    setupPyPath,
                    setupCfgPath,
                    nodeSourcePathHint,
                )) {
                    return true;
                }
            }
            if (hasCmakeBuildFile) {
                if (await this.removeCppNodeFromPackage(pkgDir, safeNodeName, cmakePath, nodeSourcePathHint)) {
                    return true;
                }
            }
        }

        if (!hasPythonBuildFiles && !hasCmakeBuildFile) {
            vscode.window.showErrorMessage(
                `Package "${safePackageName}" is not supported for remove-node (missing setup.py/setup.cfg and CMakeLists.txt).`,
            );
            return false;
        }

        vscode.window.showWarningMessage(
            `No removable node artifacts found for "${safeNodeName}" in package "${safePackageName}".`,
        );
        return false;
    }

    private async removePythonNodeFromPackage(
        pkgDir: string,
        safePackageName: string,
        safeNodeName: string,
        setupPyPath: string,
        setupCfgPath: string,
        nodeSourcePathHint?: string,
    ): Promise<boolean> {
        let changed = false;

        if (fs.existsSync(setupPyPath)) {
            try {
                const setupPy = fs.readFileSync(setupPyPath, 'utf8');
                const updated = this.removeNodeFromSetupPyConsoleScripts(setupPy, safeNodeName);
                if (updated !== setupPy) {
                    fs.writeFileSync(setupPyPath, updated, 'utf8');
                    changed = true;
                }
            } catch (err) {
                vscode.window.showErrorMessage(`Failed to update setup.py: ${err}`);
                return false;
            }
        }

        if (fs.existsSync(setupCfgPath)) {
            try {
                const setupCfg = fs.readFileSync(setupCfgPath, 'utf8');
                const updated = this.removeNodeFromSetupCfgConsoleScripts(setupCfg, safeNodeName);
                if (updated !== setupCfg) {
                    fs.writeFileSync(setupCfgPath, updated, 'utf8');
                    changed = true;
                }
            } catch (err) {
                vscode.window.showErrorMessage(`Failed to update setup.cfg: ${err}`);
                return false;
            }
        }

        const sourceCandidates = this.resolveSafeNodeSourceCandidates(
            pkgDir,
            nodeSourcePathHint,
            [
                path.join(safePackageName, `${safeNodeName}.py`),
                `${safeNodeName}.py`,
            ],
        );
        for (const sourceCandidate of sourceCandidates) {
            if (this.tryDeleteSourceFile(sourceCandidate)) {
                changed = true;
                break;
            }
        }

        return changed;
    }

    private async removeCppNodeFromPackage(
        pkgDir: string,
        safeNodeName: string,
        cmakePath: string,
        nodeSourcePathHint?: string,
    ): Promise<boolean> {
        let changed = false;

        try {
            const cmake = fs.readFileSync(cmakePath, 'utf8');
            const updated = this.removeTargetFromCmake(cmake, safeNodeName);
            if (updated !== cmake) {
                fs.writeFileSync(cmakePath, updated, 'utf8');
                changed = true;
            }
        } catch (err) {
            vscode.window.showErrorMessage(`Failed to update CMakeLists.txt: ${err}`);
            return false;
        }

        const sourceCandidates = this.resolveSafeNodeSourceCandidates(
            pkgDir,
            nodeSourcePathHint,
            [
                path.join('src', `${safeNodeName}.cpp`),
                path.join('src', `${safeNodeName}.cc`),
                path.join('src', `${safeNodeName}.cxx`),
                path.join('src', `${safeNodeName}.c`),
            ],
        );
        for (const sourceCandidate of sourceCandidates) {
            if (this.tryDeleteSourceFile(sourceCandidate)) {
                changed = true;
                break;
            }
        }

        return changed;
    }

    private removeNodeFromSetupPyConsoleScripts(setupPy: string, nodeName: string): string {
        const escapedNode = this.escapeRegex(nodeName);
        const consoleScriptsBlockPattern = /(['"]console_scripts['"]\s*:\s*\[)([\s\S]*?)(\])/gm;
        let changed = false;

        const updated = setupPy.replace(
            consoleScriptsBlockPattern,
            (full, prefix: string, body: string, suffix: string) => {
                const updatedBody = this.removeNodeFromPythonConsoleScriptBody(body, escapedNode);
                if (updatedBody !== body) {
                    changed = true;
                }
                return `${prefix}${updatedBody}${suffix}`;
            },
        );

        if (!changed) {
            return setupPy;
        }
        return updated.replace(/\n{3,}/g, '\n\n');
    }

    private removeNodeFromPythonConsoleScriptBody(body: string, escapedNodeName: string): string {
        let updatedBody = body;

        // Primary path: remove normal list entries on their own lines.
        const linePattern = new RegExp(
            `^[ \\t]*['"]\\s*${escapedNodeName}\\s*=\\s*[^'"]+['"]\\s*,?\\s*(?:#.*)?\\r?\\n?`,
            'gm',
        );
        updatedBody = updatedBody.replace(linePattern, '');

        // Fallback for one-line lists.
        if (!updatedBody.includes('\n') && !updatedBody.includes('\r')) {
            const inlinePattern = new RegExp(
                `(^|\\s*,\\s*)['"]\\s*${escapedNodeName}\\s*=\\s*[^'"]+['"]\\s*(?=\\s*,|\\s*$)`,
                'g',
            );
            updatedBody = updatedBody
                .replace(inlinePattern, '')
                .replace(/^\s*,\s*/, '')
                .replace(/\s*,\s*$/, '');
        }

        return updatedBody;
    }

    private removeNodeFromSetupCfgConsoleScripts(setupCfg: string, nodeName: string): string {
        const escapedNode = this.escapeRegex(nodeName);
        const lineEnding = setupCfg.includes('\r\n') ? '\r\n' : '\n';
        const lines = setupCfg.split(/\r?\n/);
        const nextLines: string[] = [];
        let inEntryPointsSection = false;
        const entryPattern = new RegExp(`^\\s*${escapedNode}\\s*=`);

        for (const line of lines) {
            if (/^\s*\[options\.entry_points\]\s*$/i.test(line)) {
                inEntryPointsSection = true;
                nextLines.push(line);
                continue;
            }
            if (inEntryPointsSection && /^\s*\[.*\]\s*$/.test(line)) {
                inEntryPointsSection = false;
            }
            if (inEntryPointsSection && entryPattern.test(line)) {
                continue;
            }
            nextLines.push(line);
        }

        return nextLines.join(lineEnding).replace(/\n{3,}/g, '\n\n');
    }

    private removeTargetFromCmake(cmake: string, targetName: string): string {
        let updated = cmake;
        const targetAwareCommands = [
            'add_executable',
            'ament_target_dependencies',
            'target_link_libraries',
            'target_include_directories',
            'target_compile_definitions',
            'target_compile_options',
            'target_compile_features',
            'set_target_properties',
        ];

        targetAwareCommands.forEach((commandName) => {
            updated = this.removeCmakeCommandBlockForTarget(updated, commandName, targetName);
        });
        updated = this.removeTargetFromInstallTargetsBlocks(updated, targetName);
        updated = updated.replace(/\n{3,}/g, '\n\n');

        return updated;
    }

    private removeCmakeCommandBlockForTarget(cmake: string, commandName: string, targetName: string): string {
        const escapedCommand = this.escapeRegex(commandName);
        const escapedTarget = this.escapeRegex(targetName);
        const commandPattern = new RegExp(
            `^[ \\t]*${escapedCommand}\\s*\\(\\s*${escapedTarget}\\b[\\s\\S]*?\\)\\s*\\n?`,
            'gmi',
        );
        return cmake.replace(commandPattern, '');
    }

    private removeTargetFromInstallTargetsBlocks(cmake: string, targetName: string): string {
        const installTargetsPattern = /install\s*\(\s*TARGETS[\s\S]*?\)/gim;
        return cmake.replace(
            installTargetsPattern,
            (block: string) => this.removeTargetFromSingleInstallTargetsBlock(block, targetName),
        );
    }

    private removeTargetFromSingleInstallTargetsBlock(block: string, targetName: string): string {
        const openParen = block.indexOf('(');
        const closeParen = block.lastIndexOf(')');
        if (openParen < 0 || closeParen <= openParen) {
            return block;
        }

        const inner = block.slice(openParen + 1, closeParen).trim();
        if (!inner) {
            return block;
        }
        const tokens = inner.split(/\s+/).filter(Boolean);
        if (tokens.length < 2 || tokens[0].toUpperCase() !== 'TARGETS') {
            return block;
        }

        const installKeywords = new Set([
            'ARCHIVE',
            'BUNDLE',
            'COMPONENT',
            'CONFIGURATIONS',
            'DESTINATION',
            'EXCLUDE_FROM_ALL',
            'EXPORT',
            'FRAMEWORK',
            'INCLUDES',
            'LIBRARY',
            'NAMELINK_COMPONENT',
            'NAMELINK_ONLY',
            'NAMELINK_SKIP',
            'OPTIONAL',
            'PERMISSIONS',
            'PRIVATE_HEADER',
            'PUBLIC_HEADER',
            'RESOURCE',
            'RUNTIME',
        ]);

        let targetEnd = tokens.length;
        for (let idx = 1; idx < tokens.length; idx += 1) {
            if (installKeywords.has(tokens[idx].toUpperCase())) {
                targetEnd = idx;
                break;
            }
        }

        const declaredTargets = tokens.slice(1, targetEnd);
        const remainingTargets = declaredTargets.filter((token) => token !== targetName);
        if (remainingTargets.length === declaredTargets.length) {
            return block;
        }
        if (remainingTargets.length === 0) {
            return '';
        }

        const trailingTokens = tokens.slice(targetEnd);
        const rebuiltInner = ['TARGETS', ...remainingTargets, ...trailingTokens].join(' ');
        return `${block.slice(0, openParen + 1)}${rebuiltInner}${block.slice(closeParen)}`;
    }

    private resolveSafeNodeSourceCandidates(
        packageDir: string,
        sourcePathHint: string | undefined,
        relativeCandidates: string[],
    ): string[] {
        const candidates: string[] = [];
        const addCandidate = (candidatePath: string) => {
            if (!candidatePath) {
                return;
            }
            const resolved = path.resolve(candidatePath);
            if (!this.isPathInsideDirectory(resolved, packageDir)) {
                return;
            }
            if (!candidates.includes(resolved)) {
                candidates.push(resolved);
            }
        };

        const trimmedHint = String(sourcePathHint || '').trim();
        if (trimmedHint) {
            addCandidate(trimmedHint);
        }
        relativeCandidates.forEach((relativePath) => addCandidate(path.join(packageDir, relativePath)));

        return candidates;
    }

    private tryDeleteSourceFile(filePath: string): boolean {
        try {
            if (!fs.existsSync(filePath)) {
                return false;
            }
            const stat = fs.statSync(filePath);
            if (!stat.isFile()) {
                return false;
            }
            fs.unlinkSync(filePath);
            return true;
        } catch (err) {
            vscode.window.showWarningMessage(`Failed to remove source file "${path.basename(filePath)}": ${err}`);
            return false;
        }
    }

    private isPathInsideDirectory(candidatePath: string, directoryPath: string): boolean {
        const resolvedCandidate = path.resolve(candidatePath);
        const resolvedDirectory = path.resolve(directoryPath);
        const relative = path.relative(resolvedDirectory, resolvedCandidate);
        return relative === '' || (!relative.startsWith('..') && !path.isAbsolute(relative));
    }

    private async addPythonNodeToPackage(
        pkgDir: string,
        safePackageName: string,
        safeNodeName: string,
        setupPyPath: string,
        templateKind: RosNodeTemplateKind,
        templateTopic: string,
    ): Promise<boolean> {
        const pyDir = path.join(pkgDir, safePackageName);
        const nodeFilePath = path.join(pyDir, `${safeNodeName}.py`);

        if (!fs.existsSync(pyDir)) {
            vscode.window.showErrorMessage(`Package folder not found: ${pyDir}`);
            return false;
        }

        // 1. Create the node .py file if it doesn't exist
        if (!fs.existsSync(nodeFilePath)) {
            const template = this.buildPythonNodeTemplate(safeNodeName, templateKind, templateTopic);
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
            const entry = `'${safeNodeName} = ${safePackageName}.${safeNodeName}:main'`;

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

    private normalizeNodeTemplateKind(rawKind?: string): RosNodeTemplateKind {
        const normalized = String(rawKind || 'none').trim().toLowerCase();
        switch (normalized) {
            case 'publisher':
            case 'subscriber':
            case 'service':
            case 'client':
            case 'timer':
                return normalized;
            default:
                return 'none';
        }
    }

    private normalizeNodeTemplateTopic(rawTopic?: string): string {
        const trimmed = String(rawTopic || '').trim();
        if (!trimmed) {
            return 'chatter';
        }
        return trimmed.replace(/\s+/g, '_');
    }

    private escapePythonSingleQuotedString(value: string): string {
        return String(value || '').replace(/\\/g, '\\\\').replace(/'/g, '\\\'');
    }

    private escapeCppDoubleQuotedString(value: string): string {
        return String(value || '').replace(/\\/g, '\\\\').replace(/"/g, '\\"');
    }

    private buildPythonNodeTemplate(
        safeNodeName: string,
        templateKind: RosNodeTemplateKind,
        templateTopic: string,
    ): string {
        const className = `${this.toPascalCase(safeNodeName)}Node`;
        const escapedTopicName = this.escapePythonSingleQuotedString(templateTopic);

        if (templateKind === 'publisher') {
            return `#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ${className}(Node):
    def __init__(self):
        # Give this node a unique runtime name.
        super().__init__('${safeNodeName}')
        # 1) Publisher: sends String messages to this topic.
        self.publisher_ = self.create_publisher(String, '${escapedTopicName}', 10)
        # 2) Timer: calls timer_callback every 0.5s.
        self.timer_ = self.create_timer(0.5, self.timer_callback)
        self.counter_ = 0
        self.get_logger().info('${safeNodeName} publisher node started on topic ${escapedTopicName}')

    def timer_callback(self):
        # This function runs repeatedly from the timer.
        msg = String()
        msg.data = f'Hello from ${safeNodeName}: {self.counter_}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.counter_ += 1


def main(args=None):
    # rclpy setup.
    rclpy.init(args=args)
    node = ${className}()
    # Keep node alive so callbacks continue running.
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
`;
        }

        if (templateKind === 'subscriber') {
            return `#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ${className}(Node):
    def __init__(self):
        # Give this node a unique runtime name.
        super().__init__('${safeNodeName}')
        # Create subscriber for String messages on this topic.
        self.subscription = self.create_subscription(
            String,
            '${escapedTopicName}',
            self.listener_callback,
            10,
        )
        self.get_logger().info('${safeNodeName} subscriber node listening on ${escapedTopicName}')

    def listener_callback(self, msg):
        # Called each time a new message is received.
        self.get_logger().info(f'Received: {msg.data}')


def main(args=None):
    # rclpy setup.
    rclpy.init(args=args)
    node = ${className}()
    # Keep node alive so callbacks continue running.
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
`;
        }

        if (templateKind === 'service') {
            return `#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class ${className}(Node):
    def __init__(self):
        # Give this node a unique runtime name.
        super().__init__('${safeNodeName}')
        # Create a very simple service server.
        # Rename "add_two_ints" later if you want.
        self.service_ = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service ready: add_two_ints')

    def add_two_ints_callback(self, request, response):
        # request has "a" and "b"; fill response.sum.
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}')
        return response


def main(args=None):
    # rclpy setup.
    rclpy.init(args=args)
    node = ${className}()
    # Keep node alive so it can answer service calls.
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
`;
        }

        if (templateKind === 'client') {
            return `#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class ${className}(Node):
    def __init__(self):
        # Give this node a unique runtime name.
        super().__init__('${safeNodeName}')
        # Create a client for the "add_two_ints" service.
        self.client_ = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.request_ = AddTwoInts.Request()
        self.get_logger().info('Client ready for service: add_two_ints')

    def send_request(self, a, b):
        # Fill request and send it asynchronously.
        self.request_.a = a
        self.request_.b = b
        return self.client_.call_async(self.request_)


def main(args=None):
    # rclpy setup.
    rclpy.init(args=args)
    node = ${className}()
    # Change these numbers to test different requests.
    future = node.send_request(2, 3)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(f'Result: {future.result().sum}')
    else:
        node.get_logger().error('Service call failed')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
`;
        }

        if (templateKind === 'timer') {
            return `#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class ${className}(Node):
    def __init__(self):
        super().__init__('${safeNodeName}')
        self.counter_ = 0
        self.timer_ = self.create_timer(1.0, self.on_timer)
        self.get_logger().info('${safeNodeName} timer node started')

    def on_timer(self):
        self.counter_ += 1
        self.get_logger().info(f'Tick {self.counter_}')


def main(args=None):
    rclpy.init(args=args)
    node = ${className}()
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
        }

        return `#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class ${className}(Node):
    def __init__(self):
        super().__init__('${safeNodeName}')
        self.get_logger().info('${safeNodeName} node started')


def main(args=None):
    rclpy.init(args=args)
    node = ${className}()
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
    }

    private async addCppNodeToPackage(
        pkgDir: string,
        safeNodeName: string,
        cmakePath: string,
        templateKind: RosNodeTemplateKind,
        templateTopic: string,
    ): Promise<boolean> {
        const cppSrcDir = path.join(pkgDir, 'src');
        const nodeFilePath = path.join(cppSrcDir, `${safeNodeName}.cpp`);
        const relativeSourcePath = path.posix.join('src', `${safeNodeName}.cpp`);
        const template = this.buildCppNodeTemplate(safeNodeName, templateKind, templateTopic);
        const templateDependencies = this.getCppTemplateDependencies(templateKind);

        try {
            fs.mkdirSync(cppSrcDir, { recursive: true });
        } catch (err) {
            vscode.window.showErrorMessage(`Failed to create C++ source directory: ${err}`);
            return false;
        }

        if (!fs.existsSync(nodeFilePath)) {
            try {
                fs.writeFileSync(nodeFilePath, template, { encoding: 'utf8' });
            } catch (err) {
                vscode.window.showErrorMessage(`Failed to create C++ node source file: ${err}`);
                return false;
            }
        }

        try {
            const cmake = fs.readFileSync(cmakePath, 'utf8');
            const updated = this.ensureCppNodeRegisteredInCmake(
                cmake,
                safeNodeName,
                relativeSourcePath,
                templateDependencies,
            );
            if (updated !== cmake) {
                fs.writeFileSync(cmakePath, updated, 'utf8');
            }
        } catch (err) {
            vscode.window.showErrorMessage(`Failed to update CMakeLists.txt: ${err}`);
            return false;
        }

        const uri = vscode.Uri.file(nodeFilePath);
        await vscode.window.showTextDocument(uri, { preview: false });
        return true;
    }

    private ensureCppNodeRegisteredInCmake(
        cmake: string,
        nodeName: string,
        relativeSourcePath: string,
        extraDependencies: string[] = [],
    ): string {
        let updated = cmake;
        const dependencies = Array.from(new Set(['rclcpp', ...extraDependencies]));
        dependencies.forEach((dependency) => {
            updated = this.ensureFindPackageDependency(updated, dependency);
        });

        const escapedNode = this.escapeRegex(nodeName);
        const hasExecutable = new RegExp(
            `add_executable\\s*\\(\\s*${escapedNode}\\b`,
            'm',
        ).test(updated);
        if (!hasExecutable) {
            const block = [
                `add_executable(${nodeName} ${relativeSourcePath})`,
                `ament_target_dependencies(${nodeName} ${dependencies.join(' ')})`,
                'install(TARGETS',
                `  ${nodeName}`,
                '  DESTINATION lib/${PROJECT_NAME}',
                ')',
            ].join('\n');
            updated = this.insertBeforeAmentPackage(updated, block);
        }

        dependencies.forEach((dependency) => {
            updated = this.ensureTargetDependency(updated, nodeName, dependency);
        });
        updated = this.ensureInstallTarget(updated, nodeName);
        return updated;
    }

    private getCppTemplateDependencies(templateKind: RosNodeTemplateKind): string[] {
        if (templateKind === 'publisher' || templateKind === 'subscriber') {
            return ['std_msgs'];
        }
        if (templateKind === 'service' || templateKind === 'client') {
            return ['example_interfaces'];
        }
        return [];
    }

    private buildCppNodeTemplate(
        safeNodeName: string,
        templateKind: RosNodeTemplateKind,
        templateTopic: string,
    ): string {
        const className = `${this.toPascalCase(safeNodeName)}Node`;
        const escapedNodeName = this.escapeCppDoubleQuotedString(safeNodeName);
        const escapedTopicName = this.escapeCppDoubleQuotedString(templateTopic);

        if (templateKind === 'publisher') {
            return `#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ${className} : public rclcpp::Node {
public:
  ${className}()
  : Node("${escapedNodeName}"), published_(false) {
    // 1) Publisher: sends String messages to this topic.
    publisher_ = this->create_publisher<std_msgs::msg::String>("${escapedTopicName}", 10);
    // 2) Timer: calls publish_once every 0.5s.
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&${className}::publish_once, this)
    );
    RCLCPP_INFO(this->get_logger(), "${escapedNodeName} publisher node started on topic ${escapedTopicName}");
  }

private:
  void publish_once() {
    // This function runs repeatedly from the timer.
    if (published_) {
      return;
    }
    auto msg = std_msgs::msg::String();
    msg.data = "hello world my name is ${escapedNodeName}";
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing: %s", msg.data.c_str());
    published_ = true;
    timer_->cancel();
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool published_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<${className}>());
  rclcpp::shutdown();
  return 0;
}
`;
        }

        if (templateKind === 'subscriber') {
            return `#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ${className} : public rclcpp::Node {
public:
  ${className}()
  : Node("${escapedNodeName}") {
    // Create subscriber for String messages on this topic.
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "${escapedTopicName}",
      10,
      std::bind(&${className}::on_message, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "${escapedNodeName} subscriber node listening on ${escapedTopicName}");
  }

private:
  void on_message(const std_msgs::msg::String::SharedPtr msg) const {
    // Called each time a new message is received.
    RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<${className}>());
  rclcpp::shutdown();
  return 0;
}
`;
        }

        if (templateKind === 'service') {
            return `#include <functional>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class ${className} : public rclcpp::Node {
public:
  ${className}()
  : Node("${escapedNodeName}") {
    // Create a very simple service server.
    // Rename "add_two_ints" later if you want.
    service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
      "add_two_ints",
      std::bind(&${className}::add_two_ints, this, _1, _2)
    );
    RCLCPP_INFO(this->get_logger(), "Service ready: add_two_ints");
  }

private:
  void add_two_ints(
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response
  ) {
    // request has "a" and "b"; fill response->sum.
    response->sum = request->a + request->b;
    RCLCPP_INFO(
      this->get_logger(),
      "Incoming request: a=%ld, b=%ld",
      static_cast<long>(request->a),
      static_cast<long>(request->b)
    );
  }

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<${className}>());
  rclcpp::shutdown();
  return 0;
}
`;
        }

        if (templateKind === 'client') {
            return `#include <chrono>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("${escapedNodeName}");

  // Create a client for the "add_two_ints" service.
  auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
  while (!client->wait_for_service(1s)) {
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting...");
  }

  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = 2;
  request->b = 3;

  // Send request and wait for result.
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(
      node->get_logger(),
      "Result: %ld",
      static_cast<long>(future.get()->sum)
    );
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
  }

  rclcpp::shutdown();
  return 0;
}
`;
        }

        return `#include <memory>

#include "rclcpp/rclcpp.hpp"

class ${className} : public rclcpp::Node {
public:
  ${className}()
  : Node("${escapedNodeName}") {
    RCLCPP_INFO(this->get_logger(), "${escapedNodeName} node started");
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<${className}>());
  rclcpp::shutdown();
  return 0;
}
`;
    }

    private ensureFindPackageDependency(cmake: string, dependency: string): string {
        const escapedDep = this.escapeRegex(dependency);
        if (new RegExp(`find_package\\s*\\(\\s*${escapedDep}\\b`, 'i').test(cmake)) {
            return cmake;
        }

        const findAmentPattern = /find_package\s*\(\s*ament_cmake\s+REQUIRED\s*\)\s*\n?/i;
        if (findAmentPattern.test(cmake)) {
            return cmake.replace(
                findAmentPattern,
                (match) => `${match}find_package(${dependency} REQUIRED)\n`,
            );
        }

        const projectPattern = /project\s*\([^\)]*\)\s*\n?/i;
        if (projectPattern.test(cmake)) {
            return cmake.replace(
                projectPattern,
                (match) => `${match}find_package(${dependency} REQUIRED)\n`,
            );
        }

        return `find_package(${dependency} REQUIRED)\n${cmake}`;
    }

    private ensureTargetDependency(cmake: string, targetName: string, dependency: string): string {
        const escapedTarget = this.escapeRegex(targetName);
        const targetDepPattern = new RegExp(
            `ament_target_dependencies\\s*\\(\\s*${escapedTarget}[\\s\\S]*?\\)`,
            'm',
        );
        const existingCall = cmake.match(targetDepPattern)?.[0];
        if (existingCall) {
            const hasDependency = new RegExp(`\\b${this.escapeRegex(dependency)}\\b`, 'm').test(existingCall);
            if (hasDependency) {
                return cmake;
            }
            return cmake.replace(
                targetDepPattern,
                existingCall.replace(/\)\s*$/, ` ${dependency})`),
            );
        }

        const executablePattern = new RegExp(
            `add_executable\\s*\\(\\s*${escapedTarget}[\\s\\S]*?\\)`,
            'm',
        );
        if (executablePattern.test(cmake)) {
            return cmake.replace(
                executablePattern,
                (match) => `${match}\nament_target_dependencies(${targetName} ${dependency})`,
            );
        }

        return this.insertBeforeAmentPackage(
            cmake,
            `ament_target_dependencies(${targetName} ${dependency})`,
        );
    }

    private ensureInstallTarget(cmake: string, targetName: string): string {
        const escapedTarget = this.escapeRegex(targetName);
        const installTargetPattern = new RegExp(
            `install\\s*\\(\\s*TARGETS[\\s\\S]*?\\b${escapedTarget}\\b[\\s\\S]*?\\)`,
            'm',
        );
        if (installTargetPattern.test(cmake)) {
            return cmake;
        }

        const installBlock = [
            'install(TARGETS',
            `  ${targetName}`,
            '  DESTINATION lib/${PROJECT_NAME}',
            ')',
        ].join('\n');
        return this.insertBeforeAmentPackage(cmake, installBlock);
    }

    private insertBeforeAmentPackage(cmake: string, block: string): string {
        const normalizedBlock = block.trim();
        const amentPackagePattern = /^\s*ament_package\s*\(\s*\)\s*$/m;
        if (amentPackagePattern.test(cmake)) {
            return cmake.replace(
                amentPackagePattern,
                `${normalizedBlock}\n\nament_package()`,
            );
        }

        const trailingNewline = cmake.endsWith('\n') ? '' : '\n';
        return `${cmake}${trailingNewline}\n${normalizedBlock}\n`;
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
        const context = this.resolveCommandExecutionContext();
        const wsPath = context.useWsl
            ? this.getWorkspacePathForRunTarget(context.runTarget)
            : this.getWorkspacePath();
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
            this.getRosSourceCommand(context.runTarget),
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
        runTargetOverride?: string,
    ): Promise<void> {
        const normalizedExecutableLabel = path.basename(String(executable || '').replace(/\\/g, '/')) || executable;
        const argString = args?.trim() ? ` ${args.trim()}` : '';
        const runTarget = this.normalizeRunTerminalTarget(runTargetOverride || this.getRunTerminalTarget());
        const baseRunCmd = this.isRos2()
            ? `ros2 run ${pkg} ${executable}${argString}`
            : `rosrun ${pkg} ${executable}${argString}`;
        const runCmd = this.buildRunCommandWithWslPackageFallback(pkg, baseRunCmd, runTarget);
        const normalizedArgsLabel = argsLabel?.trim();
        const normalizedNodePath = nodePath?.trim() ? nodePath : undefined;
        const normalizedArgs = String(args || '').trim();
        const labelSuffix = normalizedArgsLabel ? ` [${normalizedArgsLabel}]` : '';
        const nodeLabel = `${pkg} / ${normalizedExecutableLabel}${labelSuffix}`;
        const replay: TrackedTerminalReplay = {
            kind: 'node',
            pkg,
            executable,
            nodePath: normalizedNodePath,
            args: normalizedArgs,
            argsLabel: normalizedArgsLabel,
            runTarget,
        };

        // Smart-build check before running
        const result = await this.preLaunchBuildCheck(pkg);

        if (result.action === 'cancel') {
            return;
        }

        if (result.action === 'build-and-launch') {
            this.withPendingTrackedTerminalReplay(replay, () => {
                this.buildThenRun(result.stalePackages!, runCmd, nodeLabel, normalizedNodePath, runTarget);
            });
            return;
        }

        // No build needed — run directly
        this.withPendingTrackedTerminalReplay(replay, () => {
            this.runInConfiguredLaunchTerminal(runCmd, nodeLabel, normalizedNodePath, undefined, runTarget);
        });
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
        runTargetOverride?: string,
    ): void {
        const runTarget = this.normalizeRunTerminalTarget(runTargetOverride || this.getRunTerminalTarget());
        const wsPath = this.getWorkspacePathForRunTarget(runTarget);
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
            this.getRosSourceCommand(runTarget),
            `cd "${wsPath}"`,
            `${cleanCmd}colcon build${symlinkFlag}${pkgArg}`,
            `source "${wsPath}/install/setup.bash"`,
            runCmd,
        ];

        const fullCmd = parts.join(' && ');
        this.runInConfiguredLaunchTerminal(fullCmd, runLabel, runPath, true, runTarget);

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
        runTargetOverride?: string,
    ): Promise<void> {
        const normalizedLaunchFile = path.basename(String(launchFile || '').replace(/\\/g, '/'));
        if (!normalizedLaunchFile) {
            return;
        }
        const argString = args?.trim() ? ` ${args.trim()}` : '';
        const runTarget = this.normalizeRunTerminalTarget(runTargetOverride || this.getRunTerminalTarget());
        const baseLaunchCmd = this.isRos2()
            ? `ros2 launch ${pkg} ${normalizedLaunchFile}${argString}`
            : `roslaunch ${pkg} ${normalizedLaunchFile}${argString}`;
        const launchCmd = this.buildRunCommandWithWslPackageFallback(pkg, baseLaunchCmd, runTarget);
        const normalizedArgs = String(args || '').trim();
        const normalizedLaunchPath = launchPath?.trim() ? launchPath : undefined;
        const normalizedArgsLabel = argsLabel?.trim() || undefined;
        const labelSuffix = normalizedArgsLabel ? ` [${normalizedArgsLabel}]` : '';
        const launchLabel = `${pkg} / ${normalizedLaunchFile}${labelSuffix}`;
        const replay: TrackedTerminalReplay = {
            kind: 'launch',
            pkg,
            launchFile: normalizedLaunchFile,
            launchPath: normalizedLaunchPath,
            args: normalizedArgs,
            argsLabel: normalizedArgsLabel,
            runTarget,
        };

        // Smart-build check before launching
        const result = await this.preLaunchBuildCheck(pkg);

        if (result.action === 'cancel') {
            return;
        }

        if (result.action === 'build-and-launch') {
            this.withPendingTrackedTerminalReplay(replay, () => {
                this.buildThenRun(
                    result.stalePackages!,
                    launchCmd,
                    launchLabel,
                    normalizedLaunchPath,
                    runTarget,
                );
            });
            return;
        }

        // No build needed — launch directly
        this.withPendingTrackedTerminalReplay(replay, () => {
            this.runInConfiguredLaunchTerminal(
                launchCmd,
                launchLabel,
                normalizedLaunchPath,
                undefined,
                runTarget,
            );
        });
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
        runTargetOverride?: string,
    ): void {
        const runTarget = this.normalizeRunTerminalTarget(runTargetOverride || this.getRunTerminalTarget());
        if (this.isWindowsWslTarget(runTarget)) {
            if (this.isWindowsWslIntegratedTarget(runTarget)) {
                this.runInWslIntegratedLaunchTerminal(cmd, runTarget, launchLabel, launchPath, raw);
            } else {
                this.runInWslLaunchTerminal(cmd, runTarget, launchLabel, launchPath, raw);
            }
            return;
        }

        let useExternal = vscode.workspace
            .getConfiguration('rosDevToolkit')
            .get<boolean>('launchInExternalTerminal', true);
        if (runTarget === 'integrated') {
            useExternal = false;
        } else if (runTarget === 'external') {
            useExternal = true;
        }

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

    private runInWslLaunchTerminal(
        cmd: string,
        runTarget: string,
        launchLabel?: string,
        launchPath?: string,
        raw?: boolean,
    ): void {
        const fullCmd = raw
            ? cmd
            : this.buildSourcedCommandForRunTarget(cmd, runTarget);
        const distro = this.resolveInstalledWslDistro(this.getWslDistroFromTarget(runTarget));
        const wslPidFile = `/tmp/ros-devtool-wsl-${Date.now()}-${this._terminalSeq}.pid`;
        const bashCmd = `echo $$ > "${wslPidFile}"; ${fullCmd}; exec bash`;
        const wslArgs: string[] = [];
        if (distro) {
            wslArgs.push('-d', distro);
        }
        wslArgs.push('--exec', 'bash', '-lc', bashCmd);

        try {
            const child = cp.spawn('wsl.exe', wslArgs, {
                detached: true,
                stdio: 'ignore',
                windowsHide: false,
            });

            if (!child.pid) {
                this.runInWslIntegratedTerminalFallback(cmd, runTarget, launchLabel, launchPath, raw);
                return;
            }

            this.trackExternalTerminal(child, 'wsl.exe', cmd, launchLabel, launchPath, undefined, distro, wslPidFile);
            child.unref();
        } catch {
            this.runInWslIntegratedTerminalFallback(cmd, runTarget, launchLabel, launchPath, raw);
        }
    }

    private runInWslIntegratedLaunchTerminal(
        cmd: string,
        runTarget: string,
        launchLabel?: string,
        launchPath?: string,
        raw?: boolean,
    ): void {
        const fullCmd = raw
            ? cmd
            : this.buildSourcedCommandForRunTarget(cmd, runTarget);
        const distro = this.resolveInstalledWslDistro(this.getWslDistroFromTarget(runTarget));
        this.runPreparedInLaunchTerminal(
            fullCmd,
            cmd,
            launchLabel,
            launchPath,
            {
                wslIntegrated: true,
                wslDistro: distro,
            },
        );
    }

    private runInWslIntegratedTerminalFallback(
        cmd: string,
        runTarget: string,
        launchLabel?: string,
        launchPath?: string,
        raw?: boolean,
    ): void {
        vscode.window.showWarningMessage(
            'Failed to open a native WSL terminal window. Falling back to a VS Code WSL terminal for this run.',
        );
        this.runInWslIntegratedLaunchTerminal(cmd, runTarget, launchLabel, launchPath, raw);
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
            this.runInLaunchTerminal(cmd, launchLabel, launchPath, raw);
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
                this.runInLaunchTerminal(cmd, launchLabel, launchPath, raw);
            });
            child.unref();
        } catch {
            vscode.window.showWarningMessage(
                'Failed to open external terminal. Falling back to integrated terminal.'
            );
            this.runInLaunchTerminal(cmd, launchLabel, launchPath, raw);
        }
    }

    runInLaunchTerminal(cmd: string, launchLabel?: string, launchPath?: string, raw?: boolean): void {
        const fullCmd = raw ? cmd : this.buildSourcedCommand(cmd);
        this.runPreparedInLaunchTerminal(fullCmd, cmd, launchLabel, launchPath);
    }

    private runPreparedInLaunchTerminal(
        fullCmd: string,
        trackedCmd: string,
        launchLabel?: string,
        launchPath?: string,
        profile?: { wslIntegrated?: boolean; wslDistro?: string },
    ): void {
        const tracked = this.createLaunchTerminal(profile, launchLabel);

        tracked.cmd = trackedCmd;
        tracked.lastUsed = Date.now();
        tracked.launchLabel = launchLabel;
        tracked.launchPath = launchPath;
        tracked.ctrlCSent = false;
        tracked.status = 'running';
        tracked.replay = this.getResolvedPendingTrackedTerminalReplay(tracked);

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
            canRelaunch: Boolean(t.replay),
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

    async relaunchTrackedTerminal(id: string): Promise<void> {
        const replay = this.cloneTrackedTerminalReplay(this._trackedTerminals.get(id)?.replay);
        if (!replay) {
            return;
        }

        await this.terminateTrackedTerminalForRelaunch(id);
        await this.runTrackedTerminalReplay(replay);
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
                    const killTimestamp = Date.now();
                    tracked.lastUsed = killTimestamp;
                    this.scheduleIntegratedCloseWhenIdle(id, killTimestamp);
                }
            }
        } else {
            if (process.platform === 'win32' && tracked.wslPidFile) {
                const interrupted = this.tryInterruptWslTrackedTerminal(tracked);
                if (!interrupted || tracked.ctrlCSent) {
                    this.forceCloseTrackedExternalTerminal(tracked);
                    this._trackedTerminals.delete(id);
                    if (this._preferredTerminalId === id) {
                        this._preferredTerminalId = undefined;
                    }
                } else {
                    tracked.ctrlCSent = true;
                    const killTimestamp = Date.now();
                    tracked.lastUsed = killTimestamp;
                    this.scheduleWslCloseWhenIdle(id, killTimestamp);
                }
                this._emitTerminalsChanged();
                return;
            }

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
        const maxAttempts = 120;
        if (attempt >= maxAttempts) {
            const tracked = this._trackedTerminals.get(id);
            if (tracked?.kind === 'integrated' && tracked.terminal) {
                tracked.terminal.dispose();
                this._trackedTerminals.delete(id);
                if (this._preferredTerminalId === id) {
                    this._preferredTerminalId = undefined;
                }
                this._emitTerminalsChanged();
            }
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
        }, 250);
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

    private pickLaunchTerminal(profile?: { wslIntegrated?: boolean; wslDistro?: string }): TrackedTerminal | undefined {
        const wantsWslIntegrated = profile?.wslIntegrated === true;
        const wantedDistro = this.sanitizeWslDistroName(profile?.wslDistro);

        if (this._preferredTerminalId) {
            const preferred = this._trackedTerminals.get(this._preferredTerminalId);
            if (
                preferred?.kind === 'integrated'
                && preferred.status === 'running'
                && this.matchesIntegratedLaunchTerminalProfile(preferred, wantsWslIntegrated, wantedDistro)
            ) {
                return preferred;
            }
        }

        const active = Array.from(this._trackedTerminals.values())
            .filter((t) => (
                t.kind === 'integrated'
                && t.status === 'running'
                && this.matchesIntegratedLaunchTerminalProfile(t, wantsWslIntegrated, wantedDistro)
            ));

        if (!active.length) {
            return undefined;
        }

        active.sort((a, b) => b.lastUsed - a.lastUsed);
        return active[0];
    }

    private matchesIntegratedLaunchTerminalProfile(
        tracked: TrackedTerminal,
        wantsWslIntegrated: boolean,
        wantedDistro: string,
    ): boolean {
        if (wantsWslIntegrated) {
            if (!tracked.wslIntegrated) {
                return false;
            }
            const trackedDistro = this.sanitizeWslDistroName(tracked.wslDistro);
            if (!wantedDistro) {
                return !trackedDistro;
            }
            return trackedDistro.toLowerCase() === wantedDistro.toLowerCase();
        }

        return tracked.wslIntegrated !== true;
    }

    private createLaunchTerminal(
        profile?: { wslIntegrated?: boolean; wslDistro?: string },
        launchLabel?: string,
    ): TrackedTerminal {
        const bashPath = this.resolveBashPath();
        const env = this.getTerminalEnv();
        const wantsWslIntegrated = process.platform === 'win32' && profile?.wslIntegrated === true;
        const requestedDistro = this.sanitizeWslDistroName(profile?.wslDistro);
        const num = this._terminalSeq++;
        const name = this.buildLaunchTerminalName(launchLabel, {
            wslIntegrated: wantsWslIntegrated,
            wslDistro: requestedDistro,
            sequence: num,
        });
        let terminal: vscode.Terminal;

        if (wantsWslIntegrated) {
            const shellArgs = requestedDistro ? ['-d', requestedDistro] : [];
            terminal = vscode.window.createTerminal({
                name,
                shellPath: 'wsl.exe',
                shellArgs,
            });
        } else {
            terminal = bashPath
                ? vscode.window.createTerminal({ name, shellPath: bashPath, env })
                : vscode.window.createTerminal({ name, env });
        }

        const tracked: TrackedTerminal = {
            id: `launch-${num}`,
            kind: 'integrated',
            name,
            status: 'running',
            cmd: '',
            createdAt: Date.now(),
            lastUsed: Date.now(),
            terminal,
            wslIntegrated: wantsWslIntegrated,
            wslDistro: wantsWslIntegrated ? (requestedDistro || undefined) : undefined,
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
        wslDistro?: string,
        wslPidFile?: string,
    ): void {
        const num = this._terminalSeq++;
        const normalizedLabel = this.normalizeLaunchTerminalLabel(launchLabel);
        const transportLabel = wslDistro
            ? `wsl:${wslDistro}`
            : path.basename(terminalBin);
        const name = normalizedLabel
            ? `${normalizedLabel} (${transportLabel})`
            : (wslDistro
                ? `External (wsl:${wslDistro}) ${num}`
                : `External (${path.basename(terminalBin)}) ${num}`);
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
            wslDistro,
            wslPidFile,
            ctrlCSent: false,
        };
        tracked.replay = this.getResolvedPendingTrackedTerminalReplay(tracked);

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

    private tryInterruptWslTrackedTerminal(tracked: TrackedTerminal): boolean {
        if (process.platform !== 'win32' || !tracked.wslPidFile) {
            return false;
        }

        const wslArgs: string[] = [];
        if (tracked.wslDistro) {
            wslArgs.push('-d', tracked.wslDistro);
        }

        const pidFile = this.escapeShellArg(tracked.wslPidFile);
        const interruptScript = [
            `pid_file=${pidFile}`,
            '[ -f "$pid_file" ] || exit 0',
            'pid="$(cat "$pid_file" 2>/dev/null || true)"',
            '[ -n "$pid" ] || exit 0',
            'kill -INT -"$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true',
            'pkill -INT -P "$pid" 2>/dev/null || true',
            'true',
        ].join('; ');

        wslArgs.push('--exec', 'bash', '-lc', interruptScript);

        try {
            cp.execFileSync('wsl.exe', wslArgs, {
                windowsHide: true,
                stdio: 'ignore',
                maxBuffer: 1024 * 1024,
            });
            return true;
        } catch {
            return false;
        }
    }

    private scheduleWslCloseWhenIdle(
        trackedId: string,
        killTimestamp: number,
        attempt: number = 0,
    ): void {
        const maxAttempts = 120;
        if (attempt >= maxAttempts) {
            const tracked = this._trackedTerminals.get(trackedId);
            if (tracked?.kind === 'external') {
                this.forceCloseTrackedExternalTerminal(tracked);
                this._trackedTerminals.delete(trackedId);
                if (this._preferredTerminalId === trackedId) {
                    this._preferredTerminalId = undefined;
                }
                this._emitTerminalsChanged();
            }
            return;
        }

        setTimeout(() => {
            const tracked = this._trackedTerminals.get(trackedId);
            if (!tracked || tracked.kind !== 'external' || !tracked.wslPidFile) {
                return;
            }

            // Terminal was reused/updated after this kill request.
            if (tracked.lastUsed !== killTimestamp) {
                return;
            }

            if (this.isWslTrackedTerminalIdle(tracked)) {
                this.forceCloseTrackedExternalTerminal(tracked);
                this._trackedTerminals.delete(trackedId);
                if (this._preferredTerminalId === trackedId) {
                    this._preferredTerminalId = undefined;
                }
                this._emitTerminalsChanged();
                return;
            }

            this.scheduleWslCloseWhenIdle(trackedId, killTimestamp, attempt + 1);
        }, 250);
    }

    private isWslTrackedTerminalIdle(tracked: TrackedTerminal): boolean {
        if (process.platform !== 'win32' || !tracked.wslPidFile) {
            return false;
        }

        const wslArgs: string[] = [];
        if (tracked.wslDistro) {
            wslArgs.push('-d', tracked.wslDistro);
        }

        const pidFile = this.escapeShellArg(tracked.wslPidFile);
        const idleScript = [
            `pid_file=${pidFile}`,
            '[ -f "$pid_file" ] || exit 1',
            'pid="$(cat "$pid_file" 2>/dev/null || true)"',
            '[ -n "$pid" ] || exit 1',
            // If the shell already died, nothing is running and we can close.
            'kill -0 "$pid" 2>/dev/null || exit 0',
            'children="$(pgrep -P "$pid" 2>/dev/null || true)"',
            '[ -z "$children" ]',
        ].join('; ');

        wslArgs.push('--exec', 'bash', '-lc', idleScript);

        try {
            cp.execFileSync('wsl.exe', wslArgs, {
                windowsHide: true,
                stdio: 'ignore',
                maxBuffer: 1024 * 1024,
            });
            return true;
        } catch {
            return false;
        }
    }

    private forceCloseTrackedExternalTerminal(tracked: TrackedTerminal): void {
        const pid = tracked.pid;
        if (!pid || pid <= 0) {
            return;
        }

        if (process.platform === 'win32') {
            try {
                cp.execFileSync('taskkill.exe', ['/PID', String(pid), '/T', '/F'], {
                    windowsHide: true,
                    stdio: 'ignore',
                    maxBuffer: 1024 * 1024,
                });
            } catch {
                // ignore force-close failures
            }
            return;
        }

        try {
            process.kill(pid, 'SIGTERM');
        } catch {
            // ignore force-close failures
        }
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
    async getGraphSnapshot(scope?: Partial<RosGraphSnapshotScope>): Promise<RosGraphSnapshotResult> {
        const effectiveScope: RosGraphSnapshotScope = {
            nodes: scope?.nodes === true,
            topics: scope?.topics === true,
            services: scope?.services === true,
            actions: scope?.actions === true,
            parameters: scope?.parameters === true,
        };
        const defaults = this.createGraphSnapshotDefaults();
        const requestedKeys = this.getRequestedGraphSnapshotKeys(effectiveScope);
        if (requestedKeys.length === 0) {
            return defaults;
        }

        const timeoutSeconds = this.getGraphListTimeoutSeconds();
        const markerPrefix = `__RDT_GRAPH_SNAPSHOT__${Date.now()}_${Math.floor(Math.random() * 1_000_000)}__`;
        const commandByKey: Record<GraphSnapshotKey, string | undefined> = {
            nodes: this.isRos2()
                ? `timeout ${timeoutSeconds}s ros2 node list`
                : `timeout ${timeoutSeconds}s rosnode list`,
            topics: this.isRos2()
                ? `timeout ${timeoutSeconds}s ros2 topic list -t`
                : `timeout ${timeoutSeconds}s rostopic list`,
            services: this.isRos2()
                ? `timeout ${timeoutSeconds}s ros2 service list -t`
                : `timeout ${timeoutSeconds}s rosservice list`,
            actions: this.isRos2()
                ? `timeout ${timeoutSeconds}s ros2 action list -t`
                : undefined,
            parameters: this.isRos2()
                ? `timeout ${timeoutSeconds}s ros2 param list`
                : `timeout ${timeoutSeconds}s rosparam list`,
        };

        const sectionsToRun = requestedKeys
            .filter((key) => !!commandByKey[key])
            .map((key) => ({
                key,
                command: String(commandByKey[key] || ''),
            }));

        if (sectionsToRun.length === 0) {
            return defaults;
        }

        const compositeCommand = this.buildGraphSnapshotParallelCommand(markerPrefix, sectionsToRun);
        const context = this.resolveCommandExecutionContext();
        let raw = '';
        try {
            raw = await this.execGraphSnapshotCompositeCommand(
                compositeCommand,
                context,
                sectionsToRun.length,
            );
        } catch (err) {
            const errorText = this.normalizeCliError(err, 'Failed to execute graph snapshot command.');
            for (const key of requestedKeys) {
                if (!effectiveScope[key]) {
                    continue;
                }
                (defaults[key] as RosGraphSnapshotSection<unknown>).ok = false;
                (defaults[key] as RosGraphSnapshotSection<unknown>).error = errorText;
            }
            return defaults;
        }

        const parsedSections = this.parseGraphSnapshotSections(
            raw,
            markerPrefix,
            sectionsToRun.map((entry) => entry.key),
        );
        for (const key of requestedKeys) {
            if (!effectiveScope[key]) {
                continue;
            }
            if (key === 'actions' && !this.isRos2()) {
                defaults.actions = { ok: true, data: [] };
                continue;
            }

            const parsed = parsedSections[key];
            if (!parsed) {
                (defaults[key] as RosGraphSnapshotSection<unknown>).ok = false;
                (defaults[key] as RosGraphSnapshotSection<unknown>).error = 'Snapshot output was missing this section.';
                continue;
            }

            if (parsed.status !== 0) {
                (defaults[key] as RosGraphSnapshotSection<unknown>).ok = false;
                (defaults[key] as RosGraphSnapshotSection<unknown>).error = this.buildGraphSnapshotSectionError(
                    key,
                    parsed.status,
                    parsed.output,
                );
                continue;
            }

            try {
                if (key === 'nodes') {
                    const parsedNodes = this.parseNodeList(parsed.output);
                    defaults.nodes = {
                        ok: true,
                        data: parsedNodes.nodes,
                        ...(parsedNodes.warnings.length > 0 ? { warnings: parsedNodes.warnings } : {}),
                    };
                    continue;
                }
                if (key === 'topics') {
                    defaults.topics = {
                        ok: true,
                        data: this.parseEntityList(parsed.output),
                    };
                    continue;
                }
                if (key === 'services') {
                    defaults.services = {
                        ok: true,
                        data: this.parseEntityList(parsed.output),
                    };
                    continue;
                }
                if (key === 'actions') {
                    defaults.actions = {
                        ok: true,
                        data: this.parseEntityList(parsed.output),
                    };
                    continue;
                }
                defaults.parameters = {
                    ok: true,
                    data: this.isRos2()
                        ? this.parseRos2ParameterList(parsed.output)
                        : this.parseRos1ParameterList(parsed.output),
                };
            } catch (err) {
                (defaults[key] as RosGraphSnapshotSection<unknown>).ok = false;
                (defaults[key] as RosGraphSnapshotSection<unknown>).error = this.normalizeCliError(
                    err,
                    `Failed to parse ${key} section from snapshot output.`,
                );
            }
        }

        return defaults;
    }

    private buildLaunchTerminalName(
        launchLabel: string | undefined,
        options: { wslIntegrated: boolean; wslDistro: string; sequence: number },
    ): string {
        const normalizedLabel = this.normalizeLaunchTerminalLabel(launchLabel);
        if (!normalizedLabel) {
            return options.wslIntegrated
                ? `ROS WSL ${options.wslDistro || 'default'} ${options.sequence}`
                : `ROS Launch ${options.sequence}`;
        }

        if (!options.wslIntegrated) {
            return normalizedLabel;
        }
        const wslSuffix = options.wslDistro ? ` (WSL:${options.wslDistro})` : ' (WSL)';
        return `${normalizedLabel}${wslSuffix}`;
    }

    private normalizeLaunchTerminalLabel(launchLabel?: string): string {
        const normalized = String(launchLabel || '')
            .replace(/\s+/g, ' ')
            .trim();
        if (!normalized) {
            return '';
        }

        const maxLength = 72;
        if (normalized.length <= maxLength) {
            return normalized;
        }
        return `${normalized.slice(0, maxLength - 3)}...`;
    }

    async getNodeList(): Promise<string[]> {
        try {
            const timeoutSeconds = this.getGraphListTimeoutSeconds();
            const raw = this.isRos2()
                ? await this.exec(`timeout ${timeoutSeconds}s ros2 node list`)
                : await this.exec(`timeout ${timeoutSeconds}s rosnode list`);
            return this.parseNodeList(raw).nodes;
        } catch {
            return [];
        }
    }

    async getTopicList(): Promise<RosGraphEntityInfo[]> {
        try {
            const timeoutSeconds = this.getGraphListTimeoutSeconds();
            const raw = this.isRos2()
                ? await this.exec(`timeout ${timeoutSeconds}s ros2 topic list -t`)
                : await this.exec(`timeout ${timeoutSeconds}s rostopic list`);
            return this.parseEntityList(raw);
        } catch {
            return [];
        }
    }

    async getServiceList(): Promise<RosGraphEntityInfo[]> {
        try {
            const timeoutSeconds = this.getGraphListTimeoutSeconds();
            const raw = this.isRos2()
                ? await this.exec(`timeout ${timeoutSeconds}s ros2 service list -t`)
                : await this.exec(`timeout ${timeoutSeconds}s rosservice list`);
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
            const timeoutSeconds = this.getGraphListTimeoutSeconds();
            const raw = await this.exec(`timeout ${timeoutSeconds}s ros2 action list -t`);
            return this.parseEntityList(raw);
        } catch {
            return [];
        }
    }

    async getParameterList(): Promise<RosParameterInfo[]> {
        try {
            const timeoutSeconds = this.getGraphListTimeoutSeconds();
            const raw = this.isRos2()
                ? await this.exec(`timeout ${timeoutSeconds}s ros2 param list`)
                : await this.exec(`timeout ${timeoutSeconds}s rosparam list`);
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
                ? `timeout ${timeoutToken} ros2 topic echo --once ${topicName} 2>/dev/null || true`
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

    /**
     * Start a long-running `topic echo` process and stream every full message.
     * The caller owns the returned disposable and must dispose it to stop.
     */
    subscribeToTopicMessages(
        topicName: string,
        onMessage: (message: string) => void,
        onClosed?: (info: { code: number | null; signal: NodeJS.Signals | null; errorOutput?: string }) => void,
    ): RosTopicMessageSubscription | undefined {
        if (!this.isValidRosResourceName(topicName)) {
            return undefined;
        }

        const echoCmd = this.isRos2()
            ? `PYTHONUNBUFFERED=1 ros2 topic echo ${topicName}`
            : `PYTHONUNBUFFERED=1 rostopic echo ${topicName}`;
        const context = this.resolveCommandExecutionContext();
        const sourcedCommand = this.buildSourcedCommandForContext(echoCmd, context);
        const spawnCommand = process.platform === 'win32' && context.useWsl ? 'wsl.exe' : 'bash';
        const spawnArgs = process.platform === 'win32' && context.useWsl
            ? [
                ...(context.wslDistro ? ['-d', context.wslDistro] : []),
                '--exec',
                'bash',
                '-lc',
                sourcedCommand,
            ]
            : ['-lc', sourcedCommand];
        const child = cp.spawn(spawnCommand, spawnArgs, {
            env: process.env,
            stdio: ['ignore', 'pipe', 'pipe'],
            windowsHide: process.platform === 'win32',
        });

        let disposed = false;
        let stdoutRemainder = '';
        let stderrBuffer = '';
        let pendingMessageLines: string[] = [];

        const truncateMessage = (message: string): string => {
            const maxChars = 6000;
            if (message.length <= maxChars) {
                return message;
            }
            return `${message.slice(0, maxChars)}\n... (truncated)`;
        };

        const emitPendingMessage = () => {
            const normalized = pendingMessageLines.join('\n').trim();
            pendingMessageLines = [];
            if (!normalized) {
                return;
            }
            onMessage(truncateMessage(normalized));
        };

        const processStdoutLine = (rawLine: string) => {
            const line = rawLine.replace(/\r$/, '');
            const trimmed = line.trim();

            // ROS topic echo separates individual messages with `---`.
            if (trimmed === '---') {
                emitPendingMessage();
                return;
            }

            if (!trimmed && pendingMessageLines.length === 0) {
                return;
            }

            pendingMessageLines.push(line);
        };

        child.stdout?.on('data', (chunk: Buffer) => {
            stdoutRemainder += chunk.toString();
            const lines = stdoutRemainder.split(/\r?\n/);
            stdoutRemainder = lines.pop() ?? '';
            for (const line of lines) {
                processStdoutLine(line);
            }
        });

        child.stderr?.on('data', (chunk: Buffer) => {
            stderrBuffer += chunk.toString();
        });

        child.on('close', (code, signal) => {
            if (stdoutRemainder.trim()) {
                processStdoutLine(stdoutRemainder);
            }
            emitPendingMessage();

            if (!disposed) {
                const normalizedStderr = stderrBuffer.trim();
                onClosed?.({
                    code,
                    signal,
                    errorOutput: normalizedStderr ? this.normalizeCliError(normalizedStderr, '') : undefined,
                });
            }
        });

        child.on('error', (err) => {
            if (disposed) {
                return;
            }
            onClosed?.({
                code: null,
                signal: null,
                errorOutput: this.normalizeCliError(err, 'Failed to start topic echo process.'),
            });
        });

        const dispose = () => {
            if (disposed) {
                return;
            }
            disposed = true;
            child.stdout?.removeAllListeners();
            child.stderr?.removeAllListeners();
            child.removeAllListeners();

            if (child.killed) {
                return;
            }

            child.kill('SIGTERM');
            const killTimer = setTimeout(() => {
                if (!child.killed) {
                    child.kill('SIGKILL');
                }
            }, 1000);
            killTimer.unref();
        };

        return {
            topicName,
            dispose,
        };
    }

    async getTopicRoles(topicName: string): Promise<{ publishers: string[]; subscribers: string[] }> {
        if (!this.isValidRosResourceName(topicName)) {
            return { publishers: [], subscribers: [] };
        }

        try {
            const timeoutSeconds = this.getTopicInfoTimeoutSeconds();
            const raw = this.isRos2()
                ? await this.exec(`timeout ${timeoutSeconds}s ros2 topic info ${topicName} --verbose`)
                : await this.exec(`timeout ${timeoutSeconds}s rostopic info ${topicName}`);
            return this.isRos2()
                ? this.parseRos2TopicRoles(raw)
                : this.parseRos1TopicRoles(raw);
        } catch {
            return { publishers: [], subscribers: [] };
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

    async getActionGoalTemplate(
        actionName: string,
        actionTypeHint?: string,
    ): Promise<RosActionGoalTemplateResult> {
        if (!this.isRos2()) {
            return {
                success: false,
                actionName,
                error: 'Action goals are only available in ROS 2.',
            };
        }

        if (!this.isValidRosResourceName(actionName)) {
            return {
                success: false,
                actionName,
                error: 'Invalid action name.',
            };
        }

        const actionType = await this.resolveActionType(actionName, actionTypeHint);
        if (!actionType) {
            return {
                success: false,
                actionName,
                error: 'Could not resolve action type for this action.',
            };
        }

        try {
            const rawDefinition = await this.exec(`ros2 interface show ${actionType}`);
            const goalDefinition = this.extractFirstInterfaceSection(rawDefinition);
            const defaults = this.buildMessageDefaultsFromInterface(goalDefinition);
            const template = JSON.stringify(defaults, null, 2);
            return {
                success: true,
                actionName,
                actionType,
                template,
            };
        } catch (err) {
            return {
                success: false,
                actionName,
                actionType,
                error: this.normalizeCliError(err, 'Failed to fetch action interface.'),
            };
        }
    }

    async sendActionGoal(
        actionName: string,
        payload: string,
        actionTypeHint?: string,
    ): Promise<RosActionGoalResult> {
        if (!this.isRos2()) {
            return {
                success: false,
                actionName,
                error: 'Action goals are only available in ROS 2.',
            };
        }

        if (!this.isValidRosResourceName(actionName)) {
            return {
                success: false,
                actionName,
                error: 'Invalid action name.',
            };
        }

        const payloadText = String(payload ?? '').trim();
        if (!payloadText) {
            return {
                success: false,
                actionName,
                error: 'Goal payload is empty.',
            };
        }

        const actionType = await this.resolveActionType(actionName, actionTypeHint);
        if (!actionType) {
            return {
                success: false,
                actionName,
                error: 'Could not resolve action type for this action.',
            };
        }

        try {
            // `ros2 action send_goal` may wait for action completion; enforce
            // a bounded runtime so the webview stays responsive.
            const command = `timeout 12s ros2 action send_goal ${actionName} ${actionType} '${payloadText}'`;
            await this.exec(command);
            return {
                success: true,
                actionName,
                actionType,
            };
        } catch (err) {
            return {
                success: false,
                actionName,
                actionType,
                error: this.normalizeCliError(err, 'Failed to send action goal.'),
            };
        }
    }

    async getNodeGraphInfo(nodeName: string): Promise<RosNodeGraphInfo> {
        try {
            const timeoutSeconds = this.getNodeInfoTimeoutSeconds();
            const raw = this.isRos2()
                ? await this.exec(`timeout ${timeoutSeconds}s ros2 node info ${nodeName}`)
                : await this.exec(`timeout ${timeoutSeconds}s rosnode info ${nodeName}`);
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

    private getGraphListTimeoutSeconds(): number {
        return this.getNumericRosSetting(
            ROS_GRAPH_LIST_TIMEOUT_SETTING,
            DEFAULT_ROS_GRAPH_LIST_TIMEOUT_SECONDS,
            1,
            120,
        );
    }

    private getNodeInfoTimeoutSeconds(): number {
        return this.getNumericRosSetting(
            ROS_NODE_INFO_TIMEOUT_SETTING,
            DEFAULT_ROS_NODE_INFO_TIMEOUT_SECONDS,
            1,
            120,
        );
    }

    private getTopicInfoTimeoutSeconds(): number {
        return this.getNumericRosSetting(
            ROS_TOPIC_INFO_TIMEOUT_SETTING,
            DEFAULT_ROS_TOPIC_INFO_TIMEOUT_SECONDS,
            1,
            120,
        );
    }

    private createGraphSnapshotDefaults(): RosGraphSnapshotResult {
        return {
            nodes: { ok: true, data: [] },
            topics: { ok: true, data: [] },
            services: { ok: true, data: [] },
            actions: { ok: true, data: [] },
            parameters: { ok: true, data: [] },
        };
    }

    private getRequestedGraphSnapshotKeys(scope: RosGraphSnapshotScope): GraphSnapshotKey[] {
        const keys: GraphSnapshotKey[] = [];
        if (scope.nodes) {
            keys.push('nodes');
        }
        if (scope.topics) {
            keys.push('topics');
        }
        if (scope.services) {
            keys.push('services');
        }
        if (scope.actions) {
            keys.push('actions');
        }
        if (scope.parameters) {
            keys.push('parameters');
        }
        return keys;
    }

    private buildGraphSnapshotParallelCommand(
        markerPrefix: string,
        sections: Array<{ key: GraphSnapshotKey; command: string }>,
    ): string {
        const lines: string[] = [
            '__rdt_graph_tmp_root="${TMPDIR:-/tmp}"',
            '__rdt_graph_tmp_dir="$(mktemp -d "${__rdt_graph_tmp_root%/}/rdt_graph_XXXXXX")"',
            'if [ -z "$__rdt_graph_tmp_dir" ] || [ ! -d "$__rdt_graph_tmp_dir" ]; then echo "Failed to create graph snapshot temp dir."; exit 1; fi',
            'cleanup_rdt_graph_tmp() { rm -rf "$__rdt_graph_tmp_dir" 2>/dev/null || true; }',
            'trap cleanup_rdt_graph_tmp EXIT',
        ];

        for (const section of sections) {
            lines.push(
                `( ( ${section.command} ) > "$__rdt_graph_tmp_dir/${section.key}.out" 2>&1; ` +
                `echo "$?" > "$__rdt_graph_tmp_dir/${section.key}.status" ) &`,
            );
        }

        lines.push('wait');

        for (const section of sections) {
            const beginMarker = this.getGraphSnapshotBeginMarker(markerPrefix, section.key);
            const statusPrefix = this.getGraphSnapshotStatusPrefix(markerPrefix, section.key);
            const endMarker = this.getGraphSnapshotEndMarker(markerPrefix, section.key);
            lines.push(`echo '${beginMarker}'`);
            lines.push(`if [ -f "$__rdt_graph_tmp_dir/${section.key}.out" ]; then cat "$__rdt_graph_tmp_dir/${section.key}.out"; fi`);
            lines.push('__rdt_graph_status_value=1');
            lines.push(
                `if [ -f "$__rdt_graph_tmp_dir/${section.key}.status" ]; then ` +
                `__rdt_graph_status_value="$(cat "$__rdt_graph_tmp_dir/${section.key}.status" 2>/dev/null || echo 1)"; fi`,
            );
            lines.push(`echo '${statusPrefix}'"$__rdt_graph_status_value"`);
            lines.push(`echo '${endMarker}'`);
        }

        lines.push('cleanup_rdt_graph_tmp');
        lines.push('trap - EXIT');
        lines.push('true');
        return lines.join('\n');
    }

    private getGraphSnapshotBeginMarker(prefix: string, key: GraphSnapshotKey): string {
        return `${prefix}BEGIN:${key}`;
    }

    private getGraphSnapshotStatusPrefix(prefix: string, key: GraphSnapshotKey): string {
        return `${prefix}STATUS:${key}:`;
    }

    private getGraphSnapshotEndMarker(prefix: string, key: GraphSnapshotKey): string {
        return `${prefix}END:${key}`;
    }

    private parseGraphSnapshotSections(
        raw: string,
        markerPrefix: string,
        expectedKeys: GraphSnapshotKey[],
    ): Partial<Record<GraphSnapshotKey, GraphSnapshotParsedSection>> {
        const sectionOutput = new Map<GraphSnapshotKey, string[]>();
        const sectionStatus = new Map<GraphSnapshotKey, number>();
        const seenBegin = new Set<GraphSnapshotKey>();
        const seenEnd = new Set<GraphSnapshotKey>();

        let currentSection: GraphSnapshotKey | undefined;
        for (const rawLine of raw.split(/\r?\n/)) {
            const line = rawLine.replace(/\r$/, '');

            const beginKey = expectedKeys.find((key) => line === this.getGraphSnapshotBeginMarker(markerPrefix, key));
            if (beginKey) {
                currentSection = beginKey;
                sectionOutput.set(beginKey, []);
                seenBegin.add(beginKey);
                continue;
            }

            let matchedStatus = false;
            for (const key of expectedKeys) {
                const statusPrefix = this.getGraphSnapshotStatusPrefix(markerPrefix, key);
                if (!line.startsWith(statusPrefix)) {
                    continue;
                }
                const statusText = line.slice(statusPrefix.length).trim();
                const status = Number.parseInt(statusText, 10);
                sectionStatus.set(key, Number.isFinite(status) ? status : 1);
                matchedStatus = true;
                break;
            }
            if (matchedStatus) {
                continue;
            }

            const endKey = expectedKeys.find((key) => line === this.getGraphSnapshotEndMarker(markerPrefix, key));
            if (endKey) {
                seenEnd.add(endKey);
                if (currentSection === endKey) {
                    currentSection = undefined;
                }
                continue;
            }

            if (currentSection) {
                const lines = sectionOutput.get(currentSection);
                if (lines) {
                    lines.push(line);
                }
            }
        }

        const parsed: Partial<Record<GraphSnapshotKey, GraphSnapshotParsedSection>> = {};
        for (const key of expectedKeys) {
            if (!seenBegin.has(key) || !seenEnd.has(key)) {
                continue;
            }

            parsed[key] = {
                status: sectionStatus.get(key) ?? 1,
                output: (sectionOutput.get(key) || []).join('\n'),
            };
        }
        return parsed;
    }

    private buildGraphSnapshotSectionError(
        key: GraphSnapshotKey,
        status: number,
        output: string,
    ): string {
        const normalizedKey = key.charAt(0).toUpperCase() + key.slice(1);
        const detail = output.trim().replace(/\s+/g, ' ');
        if (!detail) {
            return `${normalizedKey} graph query failed (exit ${status}).`;
        }
        const maxChars = 220;
        const trimmed = detail.length > maxChars
            ? `${detail.slice(0, maxChars)}...`
            : detail;
        return `${normalizedKey} graph query failed (exit ${status}): ${trimmed}`;
    }

    private async execGraphSnapshotCompositeCommand(
        compositeCommand: string,
        context: CommandExecutionContext,
        sectionCount: number,
    ): Promise<string> {
        if (!this.shouldUsePersistentWslForGraph(context)) {
            this.disposePersistentWslGraphRunner();
            return this.exec(compositeCommand);
        }

        try {
            const runner = this.getPersistentWslGraphRunner(context);
            if (!runner) {
                return this.exec(compositeCommand);
            }

            const sourced = this.buildSourcedCommandForContext(compositeCommand, context);
            return await runner.exec(
                sourced,
                this.getGraphSnapshotRunnerTimeoutMs(sectionCount),
            );
        } catch (err) {
            console.warn('[RosWorkspace] Persistent WSL graph runner failed; falling back to one-shot execution.', err);
            return this.exec(compositeCommand);
        }
    }

    private disposePersistentWslGraphRunner() {
        if (!this._wslGraphRunner) {
            return;
        }
        this._wslGraphRunner.dispose();
        this._wslGraphRunner = undefined;
        this._wslGraphRunnerDistro = undefined;
    }

    private getGraphSnapshotRunnerTimeoutMs(sectionCount: number): number {
        const boundedSections = Math.max(1, Math.floor(sectionCount));
        const timeoutSeconds = this.getGraphListTimeoutSeconds();
        const estimate = (timeoutSeconds * boundedSections * 1000) + 4000;
        return Math.min(180000, Math.max(5000, estimate));
    }

    private shouldUsePersistentWslForGraph(context: CommandExecutionContext): boolean {
        return process.platform === 'win32'
            && context.useWsl
            && this.getNodeVisualizerUsePersistentWslShellEnabled();
    }

    private getNodeVisualizerUsePersistentWslShellEnabled(): boolean {
        return vscode.workspace
            .getConfiguration('rosDevToolkit')
            .get<boolean>(ROS_NODE_VISUALIZER_PERSISTENT_WSL_SHELL_SETTING, true) === true;
    }

    private getPersistentWslGraphRunner(context: CommandExecutionContext): WslPersistentGraphRunner | undefined {
        if (process.platform !== 'win32' || !context.useWsl) {
            return undefined;
        }

        const normalizedDistro = this.sanitizeWslDistroName(context.wslDistro) || undefined;
        if (this._wslGraphRunner && this._wslGraphRunnerDistro !== normalizedDistro) {
            this._wslGraphRunner.dispose();
            this._wslGraphRunner = undefined;
            this._wslGraphRunnerDistro = undefined;
        }

        if (!this._wslGraphRunner) {
            this._wslGraphRunner = new WslPersistentGraphRunner({
                distro: normalizedDistro,
                idleTimeoutMs: 90000,
            });
            this._wslGraphRunnerDistro = normalizedDistro;
        }

        return this._wslGraphRunner;
    }

    private getNumericRosSetting(
        key: string,
        defaultValue: number,
        min: number,
        max: number,
    ): number {
        const raw = vscode.workspace
            .getConfiguration('rosDevToolkit')
            .get<number>(key, defaultValue);
        const numeric = Number(raw);
        if (!Number.isFinite(numeric)) {
            return defaultValue;
        }
        return Math.min(max, Math.max(min, Math.floor(numeric)));
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

    private parseNodeList(raw: string): { nodes: string[]; warnings: string[] } {
        const nodes: string[] = [];
        const warnings: string[] = [];

        for (const line of raw.split('\n')) {
            const trimmed = line.trim();
            if (!trimmed) {
                continue;
            }
            if (trimmed.startsWith('/')) {
                nodes.push(trimmed);
                continue;
            }
            if (/^warning\b/i.test(trimmed)) {
                warnings.push(trimmed);
                continue;
            }
            warnings.push(trimmed);
        }

        return { nodes, warnings };
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

    private parseRos2TopicRoles(raw: string): { publishers: string[]; subscribers: string[] } {
        const publishers: string[] = [];
        const subscribers: string[] = [];
        let currentNodeName = '';

        for (const line of raw.split('\n')) {
            const trimmed = line.trim();
            if (!trimmed) {
                continue;
            }

            const nodePrefix = 'Node name:';
            if (trimmed.startsWith(nodePrefix)) {
                currentNodeName = trimmed.slice(nodePrefix.length).trim();
                continue;
            }

            const endpointPrefix = 'Endpoint type:';
            if (!trimmed.startsWith(endpointPrefix)) {
                continue;
            }

            const endpointType = trimmed.slice(endpointPrefix.length).trim().toUpperCase();
            const normalizedNodeName = this.normalizeTopicRoleNodeName(currentNodeName);
            if (!normalizedNodeName) {
                continue;
            }

            if (endpointType.includes('PUBLISHER')) {
                publishers.push(normalizedNodeName);
                continue;
            }
            if (endpointType.includes('SUBSCRIPTION')) {
                subscribers.push(normalizedNodeName);
            }
        }

        return {
            publishers: Array.from(new Set(publishers)),
            subscribers: Array.from(new Set(subscribers)),
        };
    }

    private parseRos1TopicRoles(raw: string): { publishers: string[]; subscribers: string[] } {
        const publishers: string[] = [];
        const subscribers: string[] = [];
        let section: 'publishers' | 'subscribers' | null = null;

        for (const line of raw.split('\n')) {
            const trimmed = line.trim();
            if (!trimmed) {
                continue;
            }

            const lower = trimmed.toLowerCase();
            if (lower.startsWith('publishers:')) {
                section = 'publishers';
                continue;
            }
            if (lower.startsWith('subscribers:')) {
                section = 'subscribers';
                continue;
            }

            const nodeName = this.parseTopicRoleNodeEntry(trimmed);
            if (!nodeName || !section) {
                continue;
            }

            if (section === 'publishers') {
                publishers.push(nodeName);
                continue;
            }
            subscribers.push(nodeName);
        }

        return {
            publishers: Array.from(new Set(publishers)),
            subscribers: Array.from(new Set(subscribers)),
        };
    }

    private parseTopicRoleNodeEntry(trimmedLine: string): string | undefined {
        if (!(trimmedLine.startsWith('*') || trimmedLine.startsWith('-'))) {
            return undefined;
        }

        const withoutBullet = trimmedLine.replace(/^[-*]\s*/, '').trim();
        if (!withoutBullet) {
            return undefined;
        }

        const token = withoutBullet.split(/\s+/)[0] ?? '';
        return this.normalizeTopicRoleNodeName(token);
    }

    private normalizeTopicRoleNodeName(rawName: string): string | undefined {
        const trimmed = String(rawName || '').trim();
        if (!trimmed) {
            return undefined;
        }

        if (this.isValidRosResourceName(trimmed)) {
            return trimmed;
        }

        // ROS 2 topic info can print relative node names (without leading '/').
        if (/^[A-Za-z0-9_./~-]+$/.test(trimmed)) {
            const normalized = '/' + trimmed.replace(/^\/+/, '');
            return this.isValidRosResourceName(normalized) ? normalized : undefined;
        }

        return undefined;
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

    private async resolveActionType(actionName: string, actionTypeHint?: string): Promise<string | undefined> {
        if (!this.isRos2()) {
            return undefined;
        }

        const hinted = String(actionTypeHint ?? '').trim();
        if (hinted && hinted !== 'unknown' && this.isValidRosTypeName(hinted)) {
            return hinted;
        }

        try {
            const raw = await this.exec(`ros2 action type ${actionName}`);
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

    private extractFirstInterfaceSection(rawInterface: string): string {
        const lines: string[] = [];
        for (const line of String(rawInterface || '').split('\n')) {
            if (line.trim() === '---') {
                break;
            }
            lines.push(line);
        }
        return lines.join('\n');
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

    private async listRosCliRunnableNodes(packageName: string): Promise<RosRunnableNodeInfo[]> {
        if (!this.isRos2()) {
            return [];
        }

        const safePackageName = this.sanitizeRosPackageName(packageName);
        if (!safePackageName) {
            return [];
        }

        try {
            const raw = await this.exec(`ros2 pkg executables ${safePackageName}`);
            const nodeNames = new Set<string>();

            for (const line of raw.split('\n')) {
                const executable = this.parseRos2ExecutableName(line, safePackageName);
                if (!executable) {
                    continue;
                }
                nodeNames.add(executable);
            }

            return Array.from(nodeNames)
                .sort((a, b) => a.localeCompare(b))
                .map((name) => ({ name }));
        } catch {
            return [];
        }
    }

    private parseRos2ExecutableName(line: string, packageName: string): string | undefined {
        const trimmed = line.trim();
        if (!trimmed) {
            return undefined;
        }

        const pkgPrefix = new RegExp(`^${this.escapeRegex(packageName)}\\s+([^\\s]+)`);
        const prefixedMatch = trimmed.match(pkgPrefix);
        const candidate = prefixedMatch?.[1]
            || (trimmed.includes(' ') ? '' : trimmed);
        if (!candidate) {
            return undefined;
        }
        if (!/^[A-Za-z0-9_.-]+$/.test(candidate)) {
            return undefined;
        }
        return candidate;
    }

    private async resolvePackagePath(packageName: string): Promise<string | undefined> {
        const safePackageName = this.sanitizeRosPackageName(packageName);
        if (!safePackageName) {
            return undefined;
        }

        if (this.isRos2()) {
            try {
                const shareRaw = await this.exec(`ros2 pkg prefix --share ${safePackageName}`);
                const sharePath = this.extractFirstExistingPath(shareRaw);
                if (sharePath) {
                    return sharePath;
                }
            } catch {
                // Ignore and fall through to prefix fallback.
            }

            try {
                const prefixRaw = await this.exec(`ros2 pkg prefix ${safePackageName}`);
                const prefixPath = this.extractFirstExistingPath(prefixRaw);
                if (prefixPath) {
                    const shareCandidate = path.join(prefixPath, 'share', safePackageName);
                    if (fs.existsSync(shareCandidate)) {
                        return shareCandidate;
                    }
                    return prefixPath;
                }
            } catch {
                return undefined;
            }

            return undefined;
        }

        try {
            const raw = await this.exec(`rospack find ${safePackageName}`);
            return this.extractFirstExistingPath(raw);
        } catch {
            return undefined;
        }
    }

    private extractFirstExistingPath(raw: string): string | undefined {
        const context = this.resolveCommandExecutionContext();
        const acceptWslLinuxPaths = process.platform === 'win32' && context.useWsl;
        for (const line of raw.split('\n')) {
            const candidate = line.trim();
            if (!candidate) {
                continue;
            }
            if (acceptWslLinuxPaths || fs.existsSync(candidate)) {
                return candidate;
            }
        }
        return undefined;
    }

    private readPackageNameFromPackageXml(packageXmlPath: string): string | undefined {
        try {
            const xml = fs.readFileSync(packageXmlPath, 'utf8');
            const match = xml.match(/<name>\s*([^<\s]+)\s*<\/name>/);
            return match?.[1];
        } catch {
            return undefined;
        }
    }

    private resolveWorkspacePackagePath(packageName: string, packagePathHint?: string): string | undefined {
        const hintedPath = String(packagePathHint || '').trim();
        if (hintedPath) {
            const resolvedHint = path.resolve(hintedPath);
            const hintedPackageXml = path.join(resolvedHint, 'package.xml');
            if (fs.existsSync(hintedPackageXml)) {
                const hintedName = this.readPackageNameFromPackageXml(hintedPackageXml);
                if (hintedName === packageName) {
                    return resolvedHint;
                }
            }
        }

        const workspacePath = this.getWorkspacePath();
        if (!workspacePath || !fs.existsSync(workspacePath)) {
            return undefined;
        }

        const packageXmlFiles = this.findPackageXmlFiles(workspacePath, 6);
        for (const packageXmlFile of packageXmlFiles) {
            const candidateName = this.readPackageNameFromPackageXml(packageXmlFile);
            if (candidateName === packageName) {
                return path.dirname(packageXmlFile);
            }
        }

        return undefined;
    }

    private sanitizeRosPackageName(name: string): string | undefined {
        const trimmed = name.trim();
        if (!trimmed) {
            return undefined;
        }
        if (!/^[A-Za-z][A-Za-z0-9_]*$/.test(trimmed)) {
            return undefined;
        }
        return trimmed;
    }

    private sanitizeRosNodeName(name: string): string | undefined {
        const trimmed = String(name || '').trim();
        if (!trimmed) {
            return undefined;
        }
        if (!/^[A-Za-z][A-Za-z0-9_]*$/.test(trimmed)) {
            return undefined;
        }
        return trimmed;
    }

    private escapeRegex(value: string): string {
        return value.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
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
        const context = this.resolveCommandExecutionContext();
        const terminal = this.ensureTerminal(context);
        terminal.show();
        if (cmd.trim().length > 0) {
            const fullCmd = this.buildSourcedCommandForContext(cmd, context);
            terminal.sendText(fullCmd);
        }
    }

    /**
     * Send a fully-formed command to the terminal WITHOUT prepending
     * the automatic source / overlay prefix.  Use this when the caller
     * has already embedded its own sourcing logic (e.g. buildPackages).
     */
    sendRawCommand(cmd: string): void {
        const context = this.resolveCommandExecutionContext();
        const terminal = this.ensureTerminal(context);
        terminal.show();
        if (cmd.trim().length > 0) {
            terminal.sendText(cmd);
        }
    }

    openSourcedTerminal(runTargetOverride?: string): void {
        const context = this.resolveCommandExecutionContext(runTargetOverride);
        const tracked = this.createLaunchTerminal(this.getTrackedTerminalProfileForContext(context), 'ROS Terminal');
        const bootstrapCommand = this.buildSourcedShellBootstrapCommand(context);

        tracked.cmd = 'ROS shell';
        tracked.lastUsed = Date.now();
        tracked.launchLabel = 'ROS Terminal';
        tracked.launchPath = undefined;
        tracked.ctrlCSent = false;
        tracked.status = 'running';
        tracked.persistentShell = true;
        tracked.replay = {
            kind: 'sourcedTerminal',
            runTarget: this.resolveTrackedTerminalRunTarget(tracked, context.runTarget),
        };

        if (!tracked.terminal) {
            return;
        }

        tracked.terminal.show();
        tracked.terminal.sendText(bootstrapCommand);
        this._emitTerminalsChanged();
    }

    private ensureTerminal(context: CommandExecutionContext): vscode.Terminal {
        const currentTarget = this.normalizeRunTerminalTarget(context.runTarget);
        if (
            this._terminal
            && this._terminal.exitStatus === undefined
            && this._terminalRunTarget
            && this._terminalRunTarget !== currentTarget
        ) {
            this._terminal.dispose();
            this._terminal = undefined;
            this._terminalRunTarget = undefined;
        }

        if (!this._terminal || this._terminal.exitStatus !== undefined) {
            const bashPath = this.resolveBashPath();
            const env = this.getTerminalEnv();
            if (process.platform === 'win32' && context.useWsl) {
                const shellArgs = context.wslDistro ? ['-d', context.wslDistro] : [];
                this._terminal = vscode.window.createTerminal({
                    name: `ROS Dev Toolkit (WSL${context.wslDistro ? `:${context.wslDistro}` : ''})`,
                    shellPath: 'wsl.exe',
                    shellArgs,
                });
            } else {
                this._terminal = bashPath
                    ? vscode.window.createTerminal({
                        name: 'ROS Dev Toolkit',
                        shellPath: bashPath,
                        env,
                    })
                    : vscode.window.createTerminal({ name: 'ROS Dev Toolkit', env });
            }
            this._terminalRunTarget = currentTarget;
        }
        return this._terminal;
    }

    private withPendingTrackedTerminalReplay<T>(
        replay: TrackedTerminalReplay | undefined,
        action: () => T,
    ): T {
        const previousReplay = this._pendingTrackedTerminalReplay;
        this._pendingTrackedTerminalReplay = replay
            ? this.cloneTrackedTerminalReplay(replay)
            : undefined;
        try {
            return action();
        } finally {
            this._pendingTrackedTerminalReplay = previousReplay;
        }
    }

    private getResolvedPendingTrackedTerminalReplay(tracked: TrackedTerminal): TrackedTerminalReplay | undefined {
        if (!this._pendingTrackedTerminalReplay) {
            return undefined;
        }

        const replay = this.cloneTrackedTerminalReplay(this._pendingTrackedTerminalReplay);
        if (!replay) {
            return undefined;
        }
        replay.runTarget = this.resolveTrackedTerminalRunTarget(tracked, replay.runTarget);
        return replay;
    }

    private cloneTrackedTerminalReplay(
        replay: TrackedTerminalReplay | undefined,
    ): TrackedTerminalReplay | undefined {
        if (!replay) {
            return undefined;
        }

        switch (replay.kind) {
            case 'launch':
                return {
                    kind: 'launch',
                    pkg: replay.pkg,
                    launchFile: replay.launchFile,
                    launchPath: replay.launchPath,
                    args: replay.args,
                    argsLabel: replay.argsLabel,
                    runTarget: replay.runTarget,
                };
            case 'node':
                return {
                    kind: 'node',
                    pkg: replay.pkg,
                    executable: replay.executable,
                    nodePath: replay.nodePath,
                    args: replay.args,
                    argsLabel: replay.argsLabel,
                    runTarget: replay.runTarget,
                };
            case 'sourcedTerminal':
                return {
                    kind: 'sourcedTerminal',
                    runTarget: replay.runTarget,
                };
        }
    }

    private resolveTrackedTerminalRunTarget(tracked: TrackedTerminal, fallbackRunTarget: string): string {
        if (tracked.kind === 'external') {
            if (tracked.wslPidFile !== undefined || tracked.wslDistro !== undefined) {
                return `wsl-external:${tracked.wslDistro || 'default'}`;
            }
            return 'external';
        }

        if (tracked.wslIntegrated) {
            return `wsl-integrated:${tracked.wslDistro || 'default'}`;
        }

        if (tracked.kind === 'integrated') {
            return 'integrated';
        }

        return this.normalizeRunTerminalTarget(fallbackRunTarget);
    }

    private async runTrackedTerminalReplay(replay: TrackedTerminalReplay): Promise<void> {
        switch (replay.kind) {
            case 'launch':
                await this.launchFile(
                    replay.pkg,
                    replay.launchFile,
                    replay.args,
                    replay.launchPath,
                    replay.argsLabel,
                    replay.runTarget,
                );
                return;
            case 'node':
                await this.runNode(
                    replay.pkg,
                    replay.executable,
                    replay.args,
                    replay.nodePath,
                    replay.argsLabel,
                    replay.runTarget,
                );
                return;
            case 'sourcedTerminal':
                this.openSourcedTerminal(replay.runTarget);
                return;
        }
    }

    private async terminateTrackedTerminalForRelaunch(id: string): Promise<void> {
        const tracked = this._trackedTerminals.get(id);
        if (!tracked) {
            return;
        }

        if (tracked.kind === 'integrated') {
            if (tracked.terminal) {
                if (tracked.status === 'running' && tracked.terminal.exitStatus === undefined) {
                    tracked.terminal.sendText('\x03', false);
                }
                tracked.terminal.dispose();
            }
            this._trackedTerminals.delete(id);
            if (this._preferredTerminalId === id) {
                this._preferredTerminalId = undefined;
            }
            this._emitTerminalsChanged();
            return;
        }

        if (process.platform === 'win32' && tracked.wslPidFile) {
            this.tryInterruptWslTrackedTerminal(tracked);
        } else {
            const innerPid = this.readInnerPid(tracked, false);
            if (innerPid) {
                this.interruptProcess(innerPid, true);
            }
        }

        this.forceCloseTrackedExternalTerminal(tracked);
        this.cleanupPidFile(tracked);
        this._trackedTerminals.delete(id);
        if (this._preferredTerminalId === id) {
            this._preferredTerminalId = undefined;
        }
        this._emitTerminalsChanged();
    }

    private getTrackedTerminalProfileForContext(
        context: CommandExecutionContext,
    ): { wslIntegrated?: boolean; wslDistro?: string } | undefined {
        if (process.platform === 'win32' && context.useWsl) {
            return {
                wslIntegrated: true,
                wslDistro: context.wslDistro,
            };
        }
        return undefined;
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
    protected getRosSourceCommand(runTarget?: string): string {
        const setupPath = this.resolveRosSetupPath(runTarget);
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

    private buildSourcedCommandForContext(cmd: string, context: CommandExecutionContext): string {
        if (context.useWsl) {
            return this.buildSourcedCommandForRunTarget(cmd, context.runTarget);
        }
        return this.buildSourcedCommand(cmd);
    }

    private buildSourcedShellBootstrapCommand(context: CommandExecutionContext): string {
        return this.buildSourcedCommandForContext('printf "ROS environment ready\\n"', context);
    }

    private buildSourcedCommandForRunTarget(cmd: string, runTarget: string): string {
        if (!this.isWindowsWslTarget(runTarget)) {
            return this.buildSourcedCommand(cmd);
        }

        const wsPath = this.getWorkspacePathForRunTarget(runTarget);
        const overlayParts = [
            `[ -f "${wsPath}/install/setup.bash" ] && source "${wsPath}/install/setup.bash" || true`,
            `[ -f "${wsPath}/devel/setup.bash" ] && source "${wsPath}/devel/setup.bash" || true`,
        ];
        return [this.getRosSourceCommand(runTarget), `cd "${wsPath}"`, ...overlayParts, cmd].join(' && ');
    }

    private buildRunCommandWithWslPackageFallback(
        pkg: string,
        baseCmd: string,
        runTarget: string,
    ): string {
        if (!this.isWindowsWslTarget(runTarget)) {
            return baseCmd;
        }

        const normalizedPkg = String(pkg || '').trim();
        if (!normalizedPkg) {
            return baseCmd;
        }

        const wsPath = this.getWorkspacePathForRunTarget(runTarget);
        const pkgArg = this.escapeShellArg(normalizedPkg);

        if (this.isRos2()) {
            const useSymlink = vscode.workspace
                .getConfiguration('rosDevToolkit')
                .get<boolean>('symlinkInstall', true);
            const symlinkFlag = useSymlink ? ' --symlink-install' : '';
            const fallbackBuild = [
                `echo "[ROS Dev Toolkit] Package ${normalizedPkg} not found in sourced environment; building it and local dependencies now..."`,
                `cd "${wsPath}"`,
                // `--packages-up-to` prevents first-run failures when local deps were never built.
                `colcon build${symlinkFlag} --packages-up-to ${pkgArg}`,
                `source "${wsPath}/install/setup.bash"`,
            ].join(' && ');
            return `if ! ros2 pkg prefix ${pkgArg} >/dev/null 2>&1; then ${fallbackBuild}; fi && ${baseCmd}`;
        }

        const fallbackBuild = [
            `echo "[ROS Dev Toolkit] Package ${normalizedPkg} not found in sourced environment; building workspace now..."`,
            `cd "${wsPath}"`,
            'catkin_make',
            `source "${wsPath}/devel/setup.bash"`,
        ].join(' && ');
        return `if ! rospack find ${pkgArg} >/dev/null 2>&1; then ${fallbackBuild}; fi && ${baseCmd}`;
    }

    private normalizeRunTerminalTarget(target?: string): string {
        const normalized = String(target || '').trim();
        if (!normalized) {
            return 'auto';
        }
        if (normalized === 'auto' || normalized === 'integrated' || normalized === 'external') {
            return normalized;
        }
        if (normalized === 'wsl:default') {
            return 'wsl-external:default';
        }
        if (normalized.startsWith('wsl:')) {
            return `wsl-external:${normalized.slice('wsl:'.length)}`;
        }
        if (
            normalized === 'wsl-integrated:default'
            || normalized.startsWith('wsl-integrated:')
            || normalized === 'wsl-external:default'
            || normalized.startsWith('wsl-external:')
        ) {
            return normalized;
        }
        return 'auto';
    }

    private isWindowsWslTarget(target: string): boolean {
        if (process.platform !== 'win32') {
            return false;
        }
        const normalized = this.normalizeRunTerminalTarget(target);
        return normalized.startsWith('wsl-integrated:') || normalized.startsWith('wsl-external:');
    }

    private isWindowsWslIntegratedTarget(target: string): boolean {
        return process.platform === 'win32'
            && this.normalizeRunTerminalTarget(target).startsWith('wsl-integrated:');
    }

    private getWslDistroFromTarget(target: string): string | undefined {
        if (!this.isWindowsWslTarget(target)) {
            return undefined;
        }

        const normalized = this.normalizeRunTerminalTarget(target);
        const isIntegrated = normalized.startsWith('wsl-integrated:');
        const prefix = isIntegrated ? 'wsl-integrated:' : 'wsl-external:';
        const rawDistro = normalized.slice(prefix.length);
        if (!rawDistro || rawDistro === 'default') {
            return undefined;
        }

        const distro = this.sanitizeWslDistroName(rawDistro);
        return distro || undefined;
    }

    private resolveCommandExecutionContext(runTargetOverride?: string): CommandExecutionContext {
        const selectedTarget = this.normalizeRunTerminalTarget(runTargetOverride || this.getRunTerminalTarget());
        if (process.platform !== 'win32') {
            return {
                useWsl: false,
                runTarget: selectedTarget,
            };
        }

        if (this.isWindowsWslTarget(selectedTarget)) {
            const requestedDistro = this.getWslDistroFromTarget(selectedTarget);
            const resolvedDistro = this.resolveInstalledWslDistroQuiet(requestedDistro) || requestedDistro;
            return {
                useWsl: true,
                runTarget: `wsl-integrated:${resolvedDistro || 'default'}`,
                wslDistro: resolvedDistro,
            };
        }

        const inferredDistro = this.getWslDistroFromWorkspacePath();
        if (inferredDistro) {
            const resolvedDistro = this.resolveInstalledWslDistroQuiet(inferredDistro) || inferredDistro;
            return {
                useWsl: true,
                runTarget: `wsl-integrated:${resolvedDistro || 'default'}`,
                wslDistro: resolvedDistro,
            };
        }

        return {
            useWsl: false,
            runTarget: selectedTarget,
        };
    }

    private getWslDistroFromWorkspacePath(): string | undefined {
        if (process.platform !== 'win32') {
            return undefined;
        }

        const wsPath = this.getWorkspacePath().replace(/\\/g, '/');
        const wslShareMatch = wsPath.match(/^\/\/(?:wsl(?:\.localhost)?|wsl\$)\/([^/]+)(?:\/|$)/i);
        const rawDistro = wslShareMatch?.[1];
        if (!rawDistro) {
            return undefined;
        }
        const distro = this.sanitizeWslDistroName(rawDistro);
        return distro || undefined;
    }

    private getWorkspacePathForRunTarget(runTarget: string): string {
        const wsPath = this.getWorkspacePath();
        if (!this.isWindowsWslTarget(runTarget)) {
            return wsPath;
        }

        const normalized = wsPath.replace(/\\/g, '/');
        const wslShareMatch = normalized.match(/^\/\/(?:wsl(?:\.localhost)?|wsl\$)\/([^/]+)(\/.*)$/i);
        if (wslShareMatch) {
            const linuxPath = wslShareMatch[2];
            return linuxPath || '/';
        }

        const driveMatch = normalized.match(/^([A-Za-z]):\/(.*)$/);
        if (driveMatch) {
            const drive = driveMatch[1].toLowerCase();
            const rest = driveMatch[2];
            return `/mnt/${drive}/${rest}`;
        }

        return normalized;
    }

    private sanitizeWslDistroName(value?: string): string {
        return String(value || '')
            .replace(/\u0000/g, '')
            .replace(/\uFEFF/g, '')
            .replace(/[\u0001-\u001F\u007F-\u009F\u200B-\u200F\u202A-\u202E\u2060-\u206F]/g, '')
            .replace(/^"+|"+$/g, '')
            .trim();
    }

    private resolveInstalledWslDistro(requested?: string): string | undefined {
        const desired = this.sanitizeWslDistroName(requested);
        if (!desired) {
            return undefined;
        }

        const installed = this.getInstalledWslDistrosSync();
        if (!installed.length) {
            vscode.window.showWarningMessage(
                'Could not verify installed WSL distros. Using the default WSL distro for this run.',
            );
            return undefined;
        }

        const exact = installed.find((name) => name === desired);
        if (exact) {
            return exact;
        }

        const ci = installed.find((name) => name.toLowerCase() === desired.toLowerCase());
        if (ci) {
            return ci;
        }

        vscode.window.showWarningMessage(
            `WSL distro "${desired}" was not found. Falling back to default WSL distro for this run.`,
        );
        return undefined;
    }

    private resolveInstalledWslDistroQuiet(requested?: string): string | undefined {
        const desired = this.sanitizeWslDistroName(requested);
        if (!desired) {
            return undefined;
        }

        const installed = this.getInstalledWslDistrosSync();
        if (!installed.length) {
            return undefined;
        }

        const exact = installed.find((name) => name === desired);
        if (exact) {
            return exact;
        }

        return installed.find((name) => name.toLowerCase() === desired.toLowerCase());
    }

    private getInstalledWslDistrosSync(): string[] {
        if (process.platform !== 'win32') {
            return [];
        }

        try {
            const rawBuffer = cp.execFileSync('wsl.exe', ['-l', '-v'], {
                windowsHide: true,
                maxBuffer: 1024 * 1024,
            });
            const raw = this.decodeWslCliOutput(rawBuffer);
            return this.parseWslList(raw)
                .map((distro) => this.sanitizeWslDistroName(distro.name))
                .filter(Boolean);
        } catch {
            return [];
        }
    }

    private decodeWslCliOutput(raw: Buffer | string): string {
        if (typeof raw === 'string') {
            return raw;
        }
        if (!raw || raw.length === 0) {
            return '';
        }

        const hasUtf16Bom = raw.length >= 2 && raw[0] === 0xFF && raw[1] === 0xFE;
        const looksUtf16Le = raw.length >= 4 && raw[1] === 0x00 && raw[3] === 0x00;
        if (hasUtf16Bom || looksUtf16Le) {
            return raw.toString('utf16le');
        }
        return raw.toString('utf8');
    }

    private formatWslDistroArg(distro: string): string {
        const normalized = this.sanitizeWslDistroName(distro);
        if (!normalized) {
            return '';
        }
        if (/^[A-Za-z0-9._-]+$/.test(normalized)) {
            return normalized;
        }
        return `"${normalized.replace(/"/g, '\\"')}"`;
    }

    private escapeShellArg(value: string): string {
        const normalized = String(value ?? '');
        return `'${normalized.replace(/'/g, `'\\''`)}'`;
    }

    private resolveRosSetupPath(runTarget?: string): string {
        const normalizedTarget = this.normalizeRunTerminalTarget(runTarget);
        const runningInWslContext = process.platform === 'win32' && this.isWindowsWslTarget(normalizedTarget);

        // 1. Explicit user setting
        const config = vscode.workspace.getConfiguration('rosDevToolkit');
        const configured = config.get<string>('rosSetupPath', '').trim();
        if (configured) {
            if (runningInWslContext) {
                // In WSL context only Linux-style absolute paths are valid.
                if (configured.startsWith('/')) {
                    return configured;
                }
            } else if (fs.existsSync(configured)) {
                return configured;
            }
        }

        // 2. Detected distro
        if (this._env?.distro) {
            const p = `/opt/ros/${this._env.distro}/setup.bash`;
            if (runningInWslContext || fs.existsSync(p)) { return p; }
        }

        // 3. $ROS_DISTRO env var
        const envDistro = process.env.ROS_DISTRO;
        if (envDistro) {
            const p = `/opt/ros/${envDistro}/setup.bash`;
            if (runningInWslContext || fs.existsSync(p)) { return p; }
        }

        // 4. First distro folder found under /opt/ros
        const rosRoot = '/opt/ros';
        if (runningInWslContext) {
            return '/opt/ros/jazzy/setup.bash';
        }
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

    private describeRuntimeMode(platform: string, remoteName?: string): string {
        if (remoteName?.startsWith('wsl+')) {
            const distro = remoteName.slice(4);
            return distro ? `WSL remote (${distro})` : 'WSL remote';
        }
        if (platform === 'linux') {
            return this.isWslKernel() ? 'Linux (WSL kernel)' : 'Linux native';
        }
        if (platform === 'win32') {
            return 'Windows local';
        }
        return platform;
    }

    private isWslKernel(): boolean {
        if (process.platform !== 'linux') {
            return false;
        }

        const candidates = ['/proc/sys/kernel/osrelease', '/proc/version'];
        for (const candidate of candidates) {
            if (!fs.existsSync(candidate)) {
                continue;
            }
            try {
                const content = fs.readFileSync(candidate, 'utf8');
                if (/microsoft/i.test(content)) {
                    return true;
                }
            } catch {
                // ignore read failures and continue fallback probing
            }
        }

        return false;
    }

    private readOsReleaseFile(filePath: string): OsReleaseInfo | undefined {
        if (!fs.existsSync(filePath)) {
            return undefined;
        }
        try {
            const content = fs.readFileSync(filePath, 'utf8');
            return this.parseOsReleaseInfo(content);
        } catch {
            return undefined;
        }
    }

    private parseOsReleaseInfo(content: string): OsReleaseInfo | undefined {
        if (!content.trim()) {
            return undefined;
        }

        const info: OsReleaseInfo = {};
        for (const line of content.split(/\r?\n/)) {
            const trimmed = line.trim();
            if (!trimmed || trimmed.startsWith('#')) {
                continue;
            }

            const eqIdx = trimmed.indexOf('=');
            if (eqIdx <= 0) {
                continue;
            }

            const key = trimmed.slice(0, eqIdx).trim();
            let value = trimmed.slice(eqIdx + 1).trim();
            if (
                (value.startsWith('"') && value.endsWith('"'))
                || (value.startsWith('\'') && value.endsWith('\''))
            ) {
                value = value.slice(1, -1);
            }

            if (key === 'NAME') {
                info.name = value;
            } else if (key === 'VERSION') {
                info.version = value;
            } else if (key === 'VERSION_ID') {
                info.versionId = value;
            } else if (key === 'PRETTY_NAME') {
                info.prettyName = value;
            }
        }

        if (!info.prettyName && !info.name && !info.version && !info.versionId) {
            return undefined;
        }
        return info;
    }

    private formatLinuxDistro(info: OsReleaseInfo): string {
        if (info.prettyName?.trim()) {
            return info.prettyName.trim();
        }

        const name = info.name?.trim() || '';
        const version = info.version?.trim() || info.versionId?.trim() || '';
        const joined = [name, version].filter(Boolean).join(' ').trim();
        return joined || 'unknown';
    }

    private formatRosEnvironment(rosDistro?: string, rosVersion?: string): string {
        const distro = String(rosDistro || '').trim();
        const version = String(rosVersion || '').trim();

        if (!distro && !version) {
            return 'not set';
        }
        if (distro && version) {
            return `${distro} (ROS ${version})`;
        }
        if (distro) {
            return `${distro} (ROS version unknown)`;
        }
        return `unknown distro (ROS ${version})`;
    }

    private listRosInstallations(rosRoot: string): string[] {
        if (!fs.existsSync(rosRoot)) {
            return [];
        }

        try {
            return fs.readdirSync(rosRoot, { withFileTypes: true })
                .filter((entry) => entry.isDirectory())
                .map((entry) => entry.name)
                .filter((distro) => (
                    fs.existsSync(path.join(rosRoot, distro, 'setup.bash'))
                    || fs.existsSync(path.join(rosRoot, distro, 'setup.sh'))
                ))
                .sort((a, b) => a.localeCompare(b));
        } catch {
            return [];
        }
    }

    private parseWslList(raw: string): WslDistroSummary[] {
        const distros: WslDistroSummary[] = [];
        const cleanedRaw = raw.replace(/\u0000/g, '').replace(/\uFEFF/g, '');

        for (const line of cleanedRaw.split(/\r?\n/)) {
            const trimmed = line.trim();
            if (!trimmed || /^name\s+state\s+version$/i.test(trimmed)) {
                continue;
            }

            let normalized = trimmed;
            let isDefault = false;
            if (normalized.startsWith('*')) {
                isDefault = true;
                normalized = normalized.slice(1).trim();
            }

            const parts = normalized.split(/\s{2,}/).map((part) => part.trim()).filter(Boolean);
            if (parts.length < 3) {
                continue;
            }

            const name = this.sanitizeWslDistroName(parts[0]);
            if (!name) {
                continue;
            }

            distros.push({
                name,
                state: parts[1] || 'unknown',
                version: parts[2] || '?',
                isDefault,
            });
        }

        return distros;
    }

    private async probeWslDistro(name: string): Promise<{
        linuxDistro?: string;
        rosDistro?: string;
        rosVersion?: string;
        rosInstalls: string[];
        error?: string;
    }> {
        const probeSeparator = '__ROS_DEV_TOOLKIT_SPLIT__';
        const probeScript = [
            'if [ -f /etc/os-release ]; then cat /etc/os-release; fi',
            `echo ${probeSeparator}`,
            'printf "ROS_DISTRO=%s\\n" "$ROS_DISTRO"',
            'printf "ROS_VERSION=%s\\n" "$ROS_VERSION"',
            `echo ${probeSeparator}`,
            'ls -1 /opt/ros 2>/dev/null || true',
        ].join('; ');
        const probe = await this.execFileSafe(
            'wsl.exe',
            ['-d', name, '--exec', 'sh', '-lc', probeScript],
            7000,
        );
        if (!probe.ok) {
            return {
                rosInstalls: [],
                error: probe.error || probe.stderr.trim() || 'WSL probe failed',
            };
        }

        const probeOutput = probe.stdout.replace(/\u0000/g, '').replace(/\uFEFF/g, '');
        const parts = probeOutput.split(probeSeparator);
        const osInfo = this.parseOsReleaseInfo(parts[0] || '');
        const envChunk = parts[1] || '';
        const rosDist = this.extractEnvVar(envChunk, 'ROS_DISTRO');
        const rosVersion = this.extractEnvVar(envChunk, 'ROS_VERSION');
        const rosInstalls = (parts[2] || '')
            .split(/\r?\n/)
            .map((line) => line.trim())
            .filter(Boolean)
            .sort((a, b) => a.localeCompare(b));

        return {
            linuxDistro: osInfo ? this.formatLinuxDistro(osInfo) : undefined,
            rosDistro: rosDist,
            rosVersion,
            rosInstalls,
        };
    }

    private extractEnvVar(content: string, key: string): string | undefined {
        const escapedKey = key.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
        const match = content.match(new RegExp(`^${escapedKey}=(.*)$`, 'm'));
        const value = match?.[1]?.trim();
        return value || undefined;
    }

    private execFileSafe(
        command: string,
        args: string[],
        timeoutMs: number = 4000,
    ): Promise<ExecFileResult> {
        return new Promise((resolve) => {
            cp.execFile(
                command,
                args,
                {
                    env: process.env,
                    timeout: timeoutMs,
                    windowsHide: true,
                    maxBuffer: 1024 * 1024,
                },
                (error, stdout, stderr) => {
                    if (error) {
                        resolve({
                            ok: false,
                            stdout: stdout || '',
                            stderr: stderr || '',
                            error: error.message,
                        });
                        return;
                    }

                    resolve({
                        ok: true,
                        stdout: stdout || '',
                        stderr: stderr || '',
                    });
                },
            );
        });
    }

    private exec(cmd: string): Promise<string> {
        const context = this.resolveCommandExecutionContext();
        const sourced = this.buildSourcedCommandForContext(cmd, context);

        return new Promise((resolve, reject) => {
            if (process.platform === 'win32' && context.useWsl) {
                const args: string[] = [];
                if (context.wslDistro) {
                    args.push('-d', context.wslDistro);
                }
                args.push('--exec', 'bash', '-lc', sourced);
                cp.execFile(
                    'wsl.exe',
                    args,
                    {
                        env: process.env,
                        windowsHide: true,
                        maxBuffer: 1024 * 1024,
                    },
                    (err, stdout, stderr) => {
                        if (err) {
                            reject(stderr || err.message);
                            return;
                        }
                        resolve(stdout || '');
                    },
                );
                return;
            }

            cp.execFile(
                'bash',
                ['-lc', sourced],
                {
                    env: process.env,
                    maxBuffer: 1024 * 1024,
                },
                (err, stdout, stderr) => {
                    if (err) {
                        reject(stderr || err.message);
                        return;
                    }
                    resolve(stdout || '');
                },
            );
        });
    }
}
