import * as vscode from 'vscode';
import * as cp from 'child_process';
import * as path from 'path';
import * as fs from 'fs';

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
}

export interface LaunchArgOption {
    name: string;
    defaultValue?: string;
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

type LaunchBuildMode = 'smart' | 'always' | 'never';

interface WorkspacePackageMeta {
    name: string;
    packagePath: string;
    buildType: string;
    dependencies: string[];
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

                packages.push({
                    name,
                    packagePath,
                    launchFiles,
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

        const depString = deps.length > 0 ? ` --dependencies ${deps.join(' ')}` : '';

        const cmd = this.isRos2()
            ? `cd "${srcDir}" && ros2 pkg create ${name} --build-type ${buildType}${depString}`
            : `cd "${srcDir}" && catkin_create_pkg ${name} ${deps.join(' ')}`;

        this.runInTerminal(cmd);
        return true;
    }

    // ── Build ──────────────────────────────────────────────────
    buildPackages(packages: string[]): void {
        const cmd = this.buildWorkspaceBuildCommand(packages);
        if (cmd) {
            this.runInTerminal(cmd);
        }
    }

    // ── Run node ───────────────────────────────────────────────
    runNode(pkg: string, executable: string, args: string = ''): void {
        const cmd = this.isRos2()
            ? `ros2 run ${pkg} ${executable} ${args}`
            : `rosrun ${pkg} ${executable} ${args}`;
        this.runInTerminal(cmd);
    }

    // ── Launch file ────────────────────────────────────────────
    launchFile(pkg: string, launchFile: string, args: string = '', launchPath?: string, argsLabel?: string): void {
        const argString = args?.trim() ? ` ${args.trim()}` : '';
        const launchCmd = this.isRos2()
            ? `ros2 launch ${pkg} ${launchFile}${argString}`
            : `roslaunch ${pkg} ${launchFile}${argString}`;
        const buildPlan = this.getLaunchBuildPlan(pkg, launchPath);
        const buildCmd = buildPlan.shouldBuild
            ? this.buildWorkspaceBuildCommand(buildPlan.packages)
            : undefined;
        const cmd = buildCmd ? `${buildCmd} && ${launchCmd}` : launchCmd;

        const useExternal = vscode.workspace
            .getConfiguration('rosDevToolkit')
            .get<boolean>('launchInExternalTerminal', true);

        const labelSuffix = argsLabel ? ` [${argsLabel}]` : '';
        const launchLabel = `${pkg} / ${launchFile}${labelSuffix}`;

        if (buildPlan.shouldBuild) {
            const mode = this.getLaunchBuildMode();
            const buildTargetLabel = buildPlan.packages.length > 0
                ? buildPlan.packages.join(', ')
                : 'workspace';
            const modeLabel = mode === 'always' ? 'always' : 'smart';
            vscode.window.setStatusBarMessage(
                `ROS Dev Toolkit: pre-launch build (${modeLabel}) ${buildTargetLabel}`,
                3500,
            );
        }

        if (useExternal) {
            this.runInExternalTerminal(cmd, launchLabel, launchPath);
        } else {
            this.runInLaunchTerminal(cmd, launchLabel, launchPath);
        }
    }

    private getLaunchBuildMode(): LaunchBuildMode {
        const mode = vscode.workspace
            .getConfiguration('rosDevToolkit')
            .get<string>('launchBuildMode', 'smart')
            .toLowerCase();
        if (mode === 'always' || mode === 'never') {
            return mode;
        }
        return 'smart';
    }

    private getLaunchBuildPlan(pkg: string, launchPath?: string): { shouldBuild: boolean; packages: string[] } {
        const mode = this.getLaunchBuildMode();
        if (mode === 'never') {
            return { shouldBuild: false, packages: [] };
        }

        const packageMap = this.getWorkspacePackageMetaMap();
        const launchWorkspacePkg = this.resolveLaunchWorkspacePackageName(pkg, launchPath, packageMap);
        if (!launchWorkspacePkg) {
            return { shouldBuild: false, packages: [] };
        }

        const closure = this.getWorkspaceDependencyClosure(launchWorkspacePkg, packageMap);
        const packagesToBuild = this.filterValidPackageNames(closure);
        if (!packagesToBuild.length) {
            return { shouldBuild: false, packages: [] };
        }

        if (mode === 'always') {
            if (this.isRos2()) {
                return { shouldBuild: true, packages: packagesToBuild };
            }
            return { shouldBuild: true, packages: [] };
        }

        const needsBuild = packagesToBuild.some((name) => {
            const meta = packageMap.get(name);
            if (!meta) {
                return true;
            }
            return this.packageNeedsBuild(meta);
        });

        if (!needsBuild) {
            return { shouldBuild: false, packages: [] };
        }

        if (this.isRos2()) {
            return { shouldBuild: true, packages: packagesToBuild };
        }
        return { shouldBuild: true, packages: [] };
    }

    private buildWorkspaceBuildCommand(packages: string[]): string {
        const wsPath = this.getWorkspacePath();

        if (!this.isRos2()) {
            return `cd "${wsPath}" && catkin_make && source "${wsPath}/devel/setup.bash"`;
        }

        const safePackages = this.filterValidPackageNames(packages);
        const pkgArg = safePackages.length > 0
            ? ` --packages-select ${safePackages.join(' ')}`
            : '';
        return `cd "${wsPath}" && colcon build${pkgArg} && source "${wsPath}/install/setup.bash"`;
    }

    private filterValidPackageNames(packages: string[]): string[] {
        const seen = new Set<string>();
        const valid: string[] = [];
        for (const pkg of packages) {
            if (!/^[A-Za-z0-9_]+$/.test(pkg)) {
                continue;
            }
            if (seen.has(pkg)) {
                continue;
            }
            seen.add(pkg);
            valid.push(pkg);
        }
        return valid;
    }

    private getWorkspacePackageMetaMap(): Map<string, WorkspacePackageMeta> {
        const map = new Map<string, WorkspacePackageMeta>();
        const srcDir = this.getSrcDir();
        if (!srcDir) {
            return map;
        }

        const defaultBuildType = this.isRos2() ? 'ament_cmake' : 'catkin';
        const packageXmlFiles = this.findPackageXmlFiles(srcDir, 4);
        for (const file of packageXmlFiles) {
            try {
                const xml = fs.readFileSync(file, 'utf8');
                const parsed = this.parsePackageManifest(xml);
                if (!parsed.name || map.has(parsed.name)) {
                    continue;
                }
                map.set(parsed.name, {
                    name: parsed.name,
                    packagePath: path.dirname(file),
                    buildType: parsed.buildType || defaultBuildType,
                    dependencies: parsed.dependencies,
                });
            } catch {
                // ignore malformed package.xml
            }
        }

        return map;
    }

    private parsePackageManifest(xml: string): { name?: string; buildType?: string; dependencies: string[] } {
        const nameMatch = xml.match(/<name>\s*([^<\s]+)\s*<\/name>/);
        const buildTypeMatch = xml.match(/<build_type>\s*([^<\s]+)\s*<\/build_type>/);
        const depTags = ['depend', 'build_depend', 'exec_depend', 'run_depend', 'build_export_depend'];
        const deps = new Set<string>();

        for (const tag of depTags) {
            const regex = new RegExp(`<${tag}>\\s*([^<\\s]+)\\s*<\\/${tag}>`, 'g');
            let match: RegExpExecArray | null;
            while ((match = regex.exec(xml)) !== null) {
                if (match[1]) {
                    deps.add(match[1]);
                }
            }
        }

        return {
            name: nameMatch?.[1],
            buildType: buildTypeMatch?.[1],
            dependencies: Array.from(deps),
        };
    }

    private resolveLaunchWorkspacePackageName(
        pkg: string,
        launchPath: string | undefined,
        packageMap: Map<string, WorkspacePackageMeta>,
    ): string | undefined {
        if (launchPath) {
            const normalizedLaunch = path.resolve(launchPath);
            for (const meta of packageMap.values()) {
                const packageRoot = path.resolve(meta.packagePath);
                if (normalizedLaunch === packageRoot || normalizedLaunch.startsWith(packageRoot + path.sep)) {
                    return meta.name;
                }
            }
        }

        if (packageMap.has(pkg)) {
            return pkg;
        }

        return undefined;
    }

    private getWorkspaceDependencyClosure(
        rootPackage: string,
        packageMap: Map<string, WorkspacePackageMeta>,
    ): string[] {
        const queue: string[] = [rootPackage];
        const seen = new Set<string>();

        while (queue.length > 0) {
            const current = queue.shift();
            if (!current || seen.has(current)) {
                continue;
            }
            seen.add(current);

            const meta = packageMap.get(current);
            if (!meta) {
                continue;
            }

            for (const dep of meta.dependencies) {
                if (packageMap.has(dep) && !seen.has(dep)) {
                    queue.push(dep);
                }
            }
        }

        return Array.from(seen);
    }

    private packageNeedsBuild(meta: WorkspacePackageMeta): boolean {
        const buildStamp = this.getPackageBuildTimestamp(meta.name);
        if (buildStamp <= 0) {
            return true;
        }

        const changedFiles = this.listFilesChangedAfter(meta.packagePath, buildStamp);
        if (changedFiles.length === 0) {
            return false;
        }

        if (changedFiles.every((file) => this.isRuntimeOnlyChange(meta.packagePath, file))) {
            return false;
        }

        if (
            this.isRos2() &&
            meta.buildType === 'ament_python' &&
            this.isPackageSymlinkInstalled(meta.name) &&
            changedFiles.every((file) => this.isSymlinkSafePythonChange(meta.packagePath, file))
        ) {
            return false;
        }

        return true;
    }

    private getPackageBuildTimestamp(packageName: string): number {
        const wsPath = this.getWorkspacePath();
        const candidates = this.isRos2()
            ? [
                path.join(wsPath, 'install', packageName, 'share', packageName, 'package.xml'),
                path.join(wsPath, 'install', packageName),
                path.join(wsPath, 'build', packageName),
            ]
            : [
                path.join(wsPath, 'devel', 'lib', packageName),
                path.join(wsPath, 'devel', 'share', packageName),
                path.join(wsPath, 'build', packageName),
            ];

        let latest = 0;
        for (const candidate of candidates) {
            latest = Math.max(latest, this.getPathTimestamp(candidate));
        }
        return latest;
    }

    private getPathTimestamp(filePath: string): number {
        try {
            return fs.statSync(filePath).mtimeMs;
        } catch {
            return 0;
        }
    }

    private listFilesChangedAfter(rootDir: string, sinceMs: number): string[] {
        const changed: string[] = [];
        const excludedDirs = new Set([
            '.git',
            '.svn',
            '.hg',
            'build',
            'install',
            'log',
            'node_modules',
            '__pycache__',
            '.vscode',
            '.idea',
        ]);

        const walk = (dir: string) => {
            let entries: fs.Dirent[] = [];
            try {
                entries = fs.readdirSync(dir, { withFileTypes: true });
            } catch {
                return;
            }

            for (const entry of entries) {
                const fullPath = path.join(dir, entry.name);
                if (entry.isDirectory()) {
                    if (excludedDirs.has(entry.name) || entry.name.startsWith('.')) {
                        continue;
                    }
                    walk(fullPath);
                    continue;
                }
                if (!entry.isFile()) {
                    continue;
                }
                try {
                    const stat = fs.statSync(fullPath);
                    if (stat.mtimeMs > sinceMs) {
                        changed.push(fullPath);
                    }
                } catch {
                    // ignore files that vanished while scanning
                }
            }
        };

        walk(rootDir);
        return changed;
    }

    private toRelativePackagePath(packagePath: string, filePath: string): string {
        return path.relative(packagePath, filePath).replace(/\\/g, '/');
    }

    private isRuntimeOnlyChange(packagePath: string, filePath: string): boolean {
        const rel = this.toRelativePackagePath(packagePath, filePath);
        if (!rel || rel.startsWith('..')) {
            return false;
        }

        if (rel.startsWith('launch/')) {
            return true;
        }
        if (rel.startsWith('config/') && /\.(yaml|yml|json|rviz|urdf|xacro)$/i.test(rel)) {
            return true;
        }
        if (rel.startsWith('params/') && /\.(yaml|yml|json)$/i.test(rel)) {
            return true;
        }

        return /\.(launch|launch\.py|launch\.xml|rviz|urdf|xacro)$/i.test(rel);
    }

    private isSymlinkSafePythonChange(packagePath: string, filePath: string): boolean {
        if (this.isRuntimeOnlyChange(packagePath, filePath)) {
            return true;
        }

        const rel = this.toRelativePackagePath(packagePath, filePath);
        if (!rel || rel.startsWith('..')) {
            return false;
        }

        const base = path.basename(rel).toLowerCase();
        if (
            base === 'package.xml' ||
            base === 'setup.py' ||
            base === 'setup.cfg' ||
            base === 'pyproject.toml' ||
            base === 'colcon.pkg' ||
            base === 'cmakelists.txt'
        ) {
            return false;
        }

        if (
            rel.startsWith('msg/') ||
            rel.startsWith('srv/') ||
            rel.startsWith('action/') ||
            rel.startsWith('include/') ||
            rel.startsWith('src/')
        ) {
            return false;
        }

        if (rel.endsWith('.py')) {
            return true;
        }

        if (rel.startsWith('resource/')) {
            return true;
        }

        return /\.(yaml|yml|json|txt|md)$/i.test(rel);
    }

    private isPackageSymlinkInstalled(packageName: string): boolean {
        if (!this.isRos2()) {
            return false;
        }

        const wsPath = this.getWorkspacePath();
        const candidates = [
            path.join(wsPath, 'install', packageName, 'share', packageName, 'package.xml'),
            path.join(wsPath, 'install', packageName, 'share', packageName),
        ];

        for (const candidate of candidates) {
            try {
                if (fs.lstatSync(candidate).isSymbolicLink()) {
                    return true;
                }
            } catch {
                // ignore missing paths
            }
        }

        return false;
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
    runInExternalTerminal(cmd: string, launchLabel?: string, launchPath?: string): void {
        const fullCmd = this.buildSourcedCommand(cmd);
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

    runInLaunchTerminal(cmd: string, launchLabel?: string, launchPath?: string): void {
        const fullCmd = this.buildSourcedCommand(cmd);
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
                tracked.terminal.sendText('\x03');
            }
        } else {
            // Read the real inner bash PID and send SIGINT (Ctrl+C)
            const innerPid = this.readInnerPid(tracked);
            if (innerPid) {
                this.interruptProcess(innerPid);
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
    private readInnerPid(tracked: TrackedTerminal): number | undefined {
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
        return tracked.pid;
    }

    /**
     * Kill the inner bash process and its entire child tree.
     * Sends SIGTERM, then SIGKILL after 500ms.
     */
    private killProcessTree(pid: number): void {
        // Kill all children first, then the parent
        try {
            cp.execSync(`pkill -TERM -P ${pid} 2>/dev/null`);
        } catch { /* ignore */ }
        try {
            process.kill(pid, 'SIGTERM');
        } catch { /* ignore */ }

        // Force kill after delay
        setTimeout(() => {
            try {
                cp.execSync(`pkill -KILL -P ${pid} 2>/dev/null`);
            } catch { /* ignore */ }
            try {
                process.kill(pid, 'SIGKILL');
            } catch { /* ignore */ }
        }, 500);
    }

    /**
     * Send SIGINT (Ctrl+C) to the inner bash's child processes.
     * This interrupts ros2 launch without closing the terminal.
     */
    private interruptProcess(pid: number): void {
        // Send SIGINT to children of the bash (ros2 launch, etc)
        try {
            cp.execSync(`pkill -INT -P ${pid} 2>/dev/null`);
        } catch { /* ignore */ }
        // Also send to the bash itself in case it's the foreground
        try {
            process.kill(pid, 'SIGINT');
        } catch { /* ignore */ }
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

    getWorkspacePath(): string {
        const folders = vscode.workspace.workspaceFolders;
        return folders?.[0]?.uri.fsPath ?? process.cwd();
    }

    private getSrcDir(): string | undefined {
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
    private getRosSourceCommand(): string {
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
