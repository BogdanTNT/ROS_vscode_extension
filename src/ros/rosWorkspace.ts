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

/**
 * Central helper that interacts with the ROS CLI tools installed on the host.
 * All shell calls go through here so views stay decoupled from the system.
 */
export class RosWorkspace {
    private _env: RosEnvironmentInfo | undefined;
    private _terminal: vscode.Terminal | undefined;

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
        const wsPath = this.getWorkspacePath();
        const pkgArg = packages.length > 0 ? ` --packages-select ${packages.join(' ')}` : '';

        const cmd = this.isRos2()
            ? `cd "${wsPath}" && colcon build${pkgArg} && source "${wsPath}/install/setup.bash"`
            : `cd "${wsPath}" && catkin_make && source "${wsPath}/devel/setup.bash"`;

        this.runInTerminal(cmd);
    }

    // ── Run node ───────────────────────────────────────────────
    runNode(pkg: string, executable: string, args: string = ''): void {
        const cmd = this.isRos2()
            ? `ros2 run ${pkg} ${executable} ${args}`
            : `rosrun ${pkg} ${executable} ${args}`;
        this.runInTerminal(cmd);
    }

    // ── Launch file ────────────────────────────────────────────
    launchFile(pkg: string, launchFile: string, args: string = ''): void {
        const argString = args?.trim() ? ` ${args.trim()}` : '';
        const cmd = this.isRos2()
            ? `ros2 launch ${pkg} ${launchFile}${argString}`
            : `roslaunch ${pkg} ${launchFile}${argString}`;

        const useExternal = vscode.workspace
            .getConfiguration('rosDevToolkit')
            .get<boolean>('launchInExternalTerminal', false);

        if (useExternal) {
            this.runInExternalTerminal(cmd);
        } else {
            this.runInTerminal(cmd);
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
    runInExternalTerminal(cmd: string): void {
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

        // Keep the terminal alive after the command finishes
        const wrappedCmd = `${fullCmd}; exec ${bashPath}`;
        const args = this.buildExternalTerminalArgs(terminalBin, bashPath, wrappedCmd);

        try {
            const child = cp.spawn(terminalBin, args, {
                detached: true,
                stdio: 'ignore',
                env: cleanEnv,
            });
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
            return ['--', bashPath, '-ic', wrappedCmd];
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
