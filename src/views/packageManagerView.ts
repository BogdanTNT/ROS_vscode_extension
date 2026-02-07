import * as vscode from 'vscode';
import { RosWorkspace } from '../ros/rosWorkspace';
import { getWebviewHtml } from './webviewHelper';
import { PMToHostCommand, PMToWebviewCommand } from './packageManagerMessages';

type LaunchArgConfig = { id: string; name: string; args: string };
type LaunchArgConfigMap = Record<string, { configs: LaunchArgConfig[] }>;

const PINNED_LAUNCH_FILES_KEY = 'pinnedLaunchFiles';
const LAUNCH_ARG_CONFIGS_KEY = 'launchArgConfigs';
const LEGACY_LAUNCH_ARGS_KEY = 'launchArgs';
// TODO(remove-by: v0.3.0 / 2026-06-30): Remove migration from legacy launch args.

/**
 * Sidebar webview that lets the user create new ROS packages.
 */
export class PackageManagerViewProvider implements vscode.WebviewViewProvider {
    private _view?: vscode.WebviewView;
    private _pinnedLaunchFiles: string[] = [];
    private _launchArgConfigs: LaunchArgConfigMap = {};

    constructor(
        private readonly _extensionUri: vscode.Uri,
        private readonly _ros: RosWorkspace,
        private readonly _context: vscode.ExtensionContext,
    ) {}

    // ── WebviewViewProvider ────────────────────────────────────
    resolveWebviewView(
        webviewView: vscode.WebviewView,
        _context: vscode.WebviewViewResolveContext,
        _token: vscode.CancellationToken,
    ): void {
        this._view = webviewView;

        webviewView.webview.options = {
            enableScripts: true,
            localResourceRoots: [this._extensionUri],
        };

        webviewView.webview.html = this._getHtml(webviewView.webview);

        // Handle messages from the webview
        webviewView.webview.onDidReceiveMessage(async (msg) => {
            switch (msg.command) {
                case PMToHostCommand.CREATE_PACKAGE:
                    await this._handleCreate(msg);
                    break;
                case PMToHostCommand.REFRESH_PACKAGES:
                    await this._sendPackageList();
                    break;
                case PMToHostCommand.OPEN_LAUNCH:
                    await this._openLaunchFile(msg.path);
                    break;
                case PMToHostCommand.OPEN_NODE:
                    await this._openNodeFile(msg.path);
                    break;
                case PMToHostCommand.LAUNCH_FILE:
                    await this._launchFile(msg.pkg, msg.file, msg.path, msg.args, msg.argsName);
                    break;
                case PMToHostCommand.RUN_NODE:
                    await this._runNode(msg.pkg, msg.executable, msg.args, msg.argsName, msg.path);
                    break;
                case PMToHostCommand.TOGGLE_PIN:
                    await this._togglePin(msg.path);
                    break;
                case PMToHostCommand.SET_LAUNCH_ARG_CONFIGS:
                    await this._setLaunchArgConfigs(msg.path, msg.configs);
                    break;
                case PMToHostCommand.REQUEST_LAUNCH_ARGS:
                    await this._sendLaunchArgOptions(msg.argsKey ?? msg.path, msg.sourcePath ?? msg.path);
                    break;
                case PMToHostCommand.KILL_TERMINAL:
                    await this._killTerminal(msg.id);
                    break;
                case PMToHostCommand.FOCUS_TERMINAL:
                    await this._focusTerminal(msg.id);
                    break;
                case PMToHostCommand.SET_PREFERRED_TERMINAL:
                    await this._setPreferredTerminal(msg.id);
                    break;
                case PMToHostCommand.TOGGLE_BUILD_CHECK:
                    await this._toggleBuildCheck(msg.enabled);
                    break;
                case PMToHostCommand.ADD_NODE:
                    await this._handleAddNode(msg.pkg, msg.nodeName);
                    break;
            }
        });

        this._ros.onTerminalsChanged((terminals) => {
            this._view?.webview.postMessage({
                command: PMToWebviewCommand.TERMINAL_LIST,
                terminals,
            });
        });

        // Send the initial package list once the view is visible
        this._sendPackageList();
        this._sendBuildCheckState();
    }

    /** Called from the "ROS: Create Package" command. */
    focusCreateForm(): void {
        this._view?.show(true);
        this._view?.webview.postMessage({ command: PMToWebviewCommand.FOCUS_CREATE });
    }

    // ── Private ────────────────────────────────────────────────
    private _sendBuildCheckState() {
        const enabled = vscode.workspace
            .getConfiguration('rosDevToolkit')
            .get<boolean>('preLaunchBuildCheck', true);
        this._view?.webview.postMessage({
            command: PMToWebviewCommand.BUILD_CHECK_STATE,
            enabled,
        });
    }

    private async _handleAddNode(pkg: string, nodeName: string) {
        if (!pkg || !nodeName) {
            return;
        }
        const ok = await this._ros.addNodeToPackage(pkg, nodeName);
        if (ok) {
            vscode.window.showInformationMessage(
                `Node "${nodeName}" added to package "${pkg}".`,
            );
        }
        this._view?.webview.postMessage({
            command: PMToWebviewCommand.ADD_NODE_DONE,
            success: ok,
        });
        if (ok) {
            await this._sendPackageList();
        }
    }

    private async _toggleBuildCheck(enabled: boolean) {
        await vscode.workspace
            .getConfiguration('rosDevToolkit')
            .update('preLaunchBuildCheck', enabled, vscode.ConfigurationTarget.Global);
    }

    private async _handleCreate(msg: { name: string; buildType: string; deps: string }) {
        const deps = msg.deps
            .split(/[\s,]+/)
            .map((d: string) => d.trim())
            .filter(Boolean);
        const ok = await this._ros.createPackage(msg.name, msg.buildType, deps);
        if (ok) {
            vscode.window.showInformationMessage(`Package "${msg.name}" created.`);
        }
        this._view?.webview.postMessage({ command: PMToWebviewCommand.CREATE_DONE, success: ok });
    }

    private async _sendPackageList() {
        this._pinnedLaunchFiles = this._context.globalState.get<string[]>(PINNED_LAUNCH_FILES_KEY, []);
        this._launchArgConfigs = await this._loadLaunchArgConfigsWithLegacyMigration();
        const packages = await this._ros.listWorkspacePackageDetails();
        this._view?.webview.postMessage({
            command: PMToWebviewCommand.PACKAGE_LIST,
            packages,
            pinned: this._pinnedLaunchFiles,
            launchArgConfigs: this._launchArgConfigs,
            terminals: this._ros.getTrackedTerminals(),
            preferredTerminalId: this._ros.getPreferredTerminalId(),
        });
    }

    private async _loadLaunchArgConfigsWithLegacyMigration(): Promise<LaunchArgConfigMap> {
        const launchArgConfigs = this._context.globalState.get<LaunchArgConfigMap>(LAUNCH_ARG_CONFIGS_KEY, {});
        const legacyArgs = this._context.globalState.get<Record<string, string>>(LEGACY_LAUNCH_ARGS_KEY, {});

        let migrated = false;
        for (const [path, args] of Object.entries(legacyArgs)) {
            if (!launchArgConfigs[path]) {
                launchArgConfigs[path] = {
                    configs: [{ id: 'default', name: 'default', args }],
                };
                migrated = true;
            }
        }

        if (migrated) {
            await this._context.globalState.update(LAUNCH_ARG_CONFIGS_KEY, launchArgConfigs);
        }

        return launchArgConfigs;
    }

    private async _openLaunchFile(filePath: string) {
        await this._openWorkspaceFile(filePath, 'Launch file');
    }

    private async _openNodeFile(filePath?: string) {
        if (!filePath) {
            vscode.window.showWarningMessage('No source file detected for this node.');
            return;
        }
        await this._openWorkspaceFile(filePath, 'Node source');
    }

    private async _openWorkspaceFile(filePath: string, label: string) {
        if (!filePath) {
            return;
        }

        const wsPath = this._ros.getWorkspacePath();
        const normalized = filePath.replace(/\\/g, '/');
        const normalizedWs = wsPath.replace(/\\/g, '/');
        if (!normalized.startsWith(normalizedWs)) {
            vscode.window.showErrorMessage(`${label} is outside the workspace.`);
            return;
        }

        const uri = vscode.Uri.file(filePath);
        await vscode.window.showTextDocument(uri, { preview: false });
    }

    private async _launchFile(
        pkg: string,
        file: string,
        path: string,
        argsOverride?: string,
        argsNameOverride?: string,
    ) {
        if (!pkg || !file) {
            return;
        }
        const args = argsOverride !== undefined
            ? argsOverride
            : undefined;

        const argsLabel = argsNameOverride ?? undefined;

        this._ros.launchFile(pkg, file, args, path, argsLabel);
    }

    private async _runNode(
        pkg: string,
        executable: string,
        argsOverride?: string,
        argsNameOverride?: string,
        sourcePath?: string,
    ) {
        if (!pkg || !executable) {
            return;
        }
        const args = argsOverride !== undefined
            ? argsOverride
            : '';
        const argsLabel = argsNameOverride?.trim() ? argsNameOverride : undefined;
        const nodePath = sourcePath?.trim() ? sourcePath : undefined;
        this._ros.runNode(pkg, executable, args, nodePath, argsLabel);
    }

    private async _killTerminal(id: string) {
        if (!id) {
            return;
        }
        this._ros.killTrackedTerminal(id);
    }

    private async _focusTerminal(id: string) {
        if (!id) {
            return;
        }
        this._ros.focusTrackedTerminal(id);
    }

    private async _setPreferredTerminal(id: string) {
        this._ros.setPreferredTerminal(id);
    }

    private async _togglePin(path: string) {
        if (!path) {
            return;
        }

        const pinned = this._context.globalState.get<string[]>(PINNED_LAUNCH_FILES_KEY, []);
        const next = pinned.includes(path)
            ? pinned.filter((p) => p !== path)
            : [...pinned, path];

        await this._context.globalState.update(PINNED_LAUNCH_FILES_KEY, next);
        await this._sendPackageList();
    }

    private async _setLaunchArgConfigs(
        path: string,
        configs: LaunchArgConfig[],
    ) {
        if (!path) {
            return;
        }

        const current = this._context.globalState.get<LaunchArgConfigMap>(LAUNCH_ARG_CONFIGS_KEY, {});
        current[path] = {
            configs: configs.map((c) => ({
                id: c.id,
                name: c.name?.trim() || 'config',
                args: c.args?.trim() || '',
            })),
        };

        await this._context.globalState.update(LAUNCH_ARG_CONFIGS_KEY, current);
        await this._sendPackageList();
    }


    private async _sendLaunchArgOptions(argsKey: string, sourcePath?: string) {
        if (!argsKey) {
            return;
        }

        const options = sourcePath
            ? this._ros.getLaunchArgOptions(sourcePath)
            : [];
        this._view?.webview.postMessage({
            command: PMToWebviewCommand.LAUNCH_ARGS_OPTIONS,
            argsKey,
            options,
        });
    }

    private _getHtml(webview: vscode.Webview): string {
        const body = /* html */ `
<h2>📦 Package Manager</h2>

<div class="section">
    <div class="toolbar">
        <button id="btnOpenCreate">＋ New Package</button>
        <button class="secondary small" id="btnRefresh">↻ Refresh</button>
    </div>

    <div class="toggle-row">
        <label for="toggleBuildCheck">⚡ Auto build check before launch</label>
        <span class="toggle-switch">
            <input type="checkbox" id="toggleBuildCheck" checked />
            <span class="toggle-slider"></span>
        </span>
    </div>

    <div class="search-row">
        <input type="text" id="pkgFilter" placeholder="Filter packages, launch files, or nodes…" />
        <span id="pkgCount" class="badge info">0</span>
    </div>

    <div id="pkgStatus" class="text-muted text-sm mb"></div>

    <div class="subsection">
        <h3>Active Launch Terminals</h3>
        <ul class="item-list" id="terminalList">
            <li class="text-muted">No active launch terminals</li>
        </ul>
    </div>

    <div class="subsection">
        <h3>Pinned Items</h3>
        <ul class="item-list" id="pinnedList">
            <li class="text-muted">No pinned items</li>
        </ul>
    </div>

    <div class="subsection">
        <h3>Workspace Packages</h3>
        <ul class="item-list" id="pkgList">
            <li class="text-muted">Loading…</li>
        </ul>
    </div>
</div>

<div class="modal hidden" id="createModal" role="dialog" aria-modal="true">
    <div class="modal-backdrop" id="createBackdrop"></div>
    <div class="modal-card">
        <div class="modal-header">
            <h3>Create New Package</h3>
            <button class="secondary small" id="btnCloseCreate">✕</button>
        </div>
        <div class="modal-body">
            <label for="pkgName">Package name</label>
            <input type="text" id="pkgName" placeholder="my_robot_pkg" />

            <label for="buildType">Build type</label>
            <select id="buildType">
                <option value="ament_cmake">ament_cmake (C++)</option>
                <option value="ament_python">ament_python (Python)</option>
            </select>

            <label for="deps">Dependencies <span class="text-muted text-sm">(space / comma separated)</span></label>
            <input type="text" id="deps" placeholder="auto: rclcpp std_msgs / rclpy std_msgs" />
        </div>
        <div class="modal-footer">
            <button class="secondary" id="btnCancelCreate">Cancel</button>
            <button id="btnCreate">Create Package</button>
        </div>
        <div id="createStatus" class="mt hidden"></div>
    </div>
</div>

<div class="modal hidden" id="argsModal" role="dialog" aria-modal="true">
    <div class="modal-backdrop" id="argsBackdrop"></div>
    <div class="modal-card">
        <div class="modal-header">
            <h3>Launch file arguments</h3>
            <button class="secondary small" id="btnCloseArgs">✕</button>
        </div>
        <div class="modal-body">
                    <div class="config-row">
                        <label>Configurations</label>
                        <div class="config-actions">
                            <button class="secondary small" id="btnAddConfig">＋ Add</button>
                            <button class="secondary small" id="btnRemoveConfig">－ Remove</button>
                        </div>
                    </div>
                    <div class="config-list" id="configList"></div>

                    <label for="configName">Config name</label>
                    <input type="text" id="configName" placeholder="default" />

            <label for="argsInput">Arguments</label>
            <input type="text" id="argsInput" placeholder="use_sim_time:=true namespace:=robot1" />

            <label>Available arguments</label>
            <ul class="arg-list" id="argsList">
                <li class="text-muted">No arguments detected</li>
            </ul>
            <div class="btn-row mt">
                <button class="secondary small" id="btnInsertAll">Insert all defaults</button>
            </div>
        </div>
        <div class="modal-footer">
            <button class="secondary" id="btnCancelArgs">Cancel</button>
            <button id="btnSaveArgs">Save</button>
        </div>
    </div>
</div>

<div class="modal hidden" id="addNodeModal" role="dialog" aria-modal="true">
    <div class="modal-backdrop" id="addNodeBackdrop"></div>
    <div class="modal-card">
        <div class="modal-header">
            <h3>Add Node to Package</h3>
            <button class="secondary small" id="btnCloseAddNode">✕</button>
        </div>
        <div class="modal-body">
            <label for="addNodePkg">Package</label>
            <input type="text" id="addNodePkg" readonly />

            <label for="addNodeName">Node name</label>
            <input type="text" id="addNodeName" placeholder="my_node" />
        </div>
        <div class="modal-footer">
            <button class="secondary" id="btnCancelAddNode">Cancel</button>
            <button id="btnAddNode">Add Node</button>
        </div>
        <div id="addNodeStatus" class="mt hidden"></div>
    </div>
</div>
`;
        const scriptUris = [
            vscode.Uri.joinPath(this._extensionUri, 'media', 'packageManager', 'state.js'),
            vscode.Uri.joinPath(this._extensionUri, 'media', 'packageManager', 'dom.js'),
            vscode.Uri.joinPath(this._extensionUri, 'media', 'packageManager', 'messages.js'),
            vscode.Uri.joinPath(this._extensionUri, 'media', 'packageManager', 'actions.js'),
            vscode.Uri.joinPath(this._extensionUri, 'media', 'packageManager', 'render.js'),
            vscode.Uri.joinPath(this._extensionUri, 'media', 'packageManager', 'events.js'),
            vscode.Uri.joinPath(this._extensionUri, 'media', 'packageManager', 'index.js'),
        ];
        return getWebviewHtml(webview, this._extensionUri, body, '', undefined, scriptUris);
    }
}
