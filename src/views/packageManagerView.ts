import * as vscode from 'vscode';
import { RosWorkspace } from '../ros/rosWorkspace';
import { getWebviewHtml } from './webviewHelper';

/**
 * Sidebar webview that lets the user create new ROS packages.
 */
export class PackageManagerViewProvider implements vscode.WebviewViewProvider {
    private _view?: vscode.WebviewView;
    private _pinnedLaunchFiles: string[] = [];
    private _launchArgConfigs: Record<string, { selectedId: string; configs: Array<{ id: string; name: string; args: string }> }> = {};

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
                case 'createPackage':
                    await this._handleCreate(msg);
                    break;
                case 'refreshPackages':
                    await this._sendPackageList();
                    break;
                case 'openLaunch':
                    await this._openLaunchFile(msg.path);
                    break;
                case 'launchFile':
                    await this._launchFile(msg.pkg, msg.file, msg.path, msg.args, msg.argsName);
                    break;
                case 'togglePin':
                    await this._togglePin(msg.path);
                    break;
                case 'setLaunchArgs':
                    await this._setLaunchArgs(msg.path, msg.args);
                    break;
                case 'setLaunchArgConfigs':
                    await this._setLaunchArgConfigs(msg.path, msg.selectedId, msg.configs);
                    break;
                case 'selectLaunchArgConfig':
                    await this._selectLaunchArgConfig(msg.path, msg.id);
                    break;
                case 'requestLaunchArgs':
                    await this._sendLaunchArgOptions(msg.path);
                    break;
                case 'killTerminal':
                    await this._killTerminal(msg.id);
                    break;
                case 'focusTerminal':
                    await this._focusTerminal(msg.id);
                    break;
                case 'setPreferredTerminal':
                    await this._setPreferredTerminal(msg.id);
                    break;
            }
        });

        this._ros.onTerminalsChanged((terminals) => {
            this._view?.webview.postMessage({
                command: 'terminalList',
                terminals,
            });
        });

        // Send the initial package list once the view is visible
        this._sendPackageList();
    }

    /** Called from the "ROS: Create Package" command. */
    focusCreateForm(): void {
        this._view?.show(true);
        this._view?.webview.postMessage({ command: 'focusCreate' });
    }

    // ── Private ────────────────────────────────────────────────
    private async _handleCreate(msg: { name: string; buildType: string; deps: string }) {
        const deps = msg.deps
            .split(/[\s,]+/)
            .map((d: string) => d.trim())
            .filter(Boolean);
        const ok = await this._ros.createPackage(msg.name, msg.buildType, deps);
        if (ok) {
            vscode.window.showInformationMessage(`Package "${msg.name}" created.`);
        }
        this._view?.webview.postMessage({ command: 'createDone', success: ok });
    }

    private async _sendPackageList() {
        this._pinnedLaunchFiles = this._context.globalState.get<string[]>('pinnedLaunchFiles', []);
        const legacyArgs = this._context.globalState.get<Record<string, string>>('launchArgs', {});
        this._launchArgConfigs = this._context.globalState.get<Record<string, { selectedId: string; configs: Array<{ id: string; name: string; args: string }> }>>('launchArgConfigs', {});

        // Migrate legacy single-args to config list
        let migrated = false;
        for (const [path, args] of Object.entries(legacyArgs)) {
            if (!this._launchArgConfigs[path]) {
                this._launchArgConfigs[path] = {
                    selectedId: 'default',
                    configs: [{ id: 'default', name: 'default', args }],
                };
                migrated = true;
            }
        }
        if (migrated) {
            await this._context.globalState.update('launchArgConfigs', this._launchArgConfigs);
        }
        const packages = await this._ros.listWorkspacePackageDetails();
        this._view?.webview.postMessage({
            command: 'packageList',
            packages,
            pinned: this._pinnedLaunchFiles,
            launchArgConfigs: this._launchArgConfigs,
            terminals: this._ros.getTrackedTerminals(),
            preferredTerminalId: this._ros.getPreferredTerminalId(),
        });
    }

    private async _openLaunchFile(filePath: string) {
        if (!filePath) {
            return;
        }

        const wsPath = this._ros.getWorkspacePath();
        const normalized = filePath.replace(/\\/g, '/');
        const normalizedWs = wsPath.replace(/\\/g, '/');
        if (!normalized.startsWith(normalizedWs)) {
            vscode.window.showErrorMessage('Launch file is outside the workspace.');
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
        const config = path ? this._launchArgConfigs[path] : undefined;
        const selectedId = config?.selectedId;
        const selectedCfg = selectedId
            ? config?.configs.find((c) => c.id === selectedId)
            : undefined;

        const args = argsOverride !== undefined
            ? argsOverride
            : selectedCfg?.args;

        const argsLabel = argsNameOverride ?? selectedCfg?.name;

        this._ros.launchFile(pkg, file, args, path, argsLabel);
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

        const pinned = this._context.globalState.get<string[]>('pinnedLaunchFiles', []);
        const next = pinned.includes(path)
            ? pinned.filter((p) => p !== path)
            : [...pinned, path];

        await this._context.globalState.update('pinnedLaunchFiles', next);
        await this._sendPackageList();
    }

    private async _setLaunchArgs(path: string, args: string) {
        if (!path) {
            return;
        }

        const current = this._context.globalState.get<Record<string, { selectedId: string; configs: Array<{ id: string; name: string; args: string }> }>>('launchArgConfigs', {});
        const trimmed = args?.trim() ?? '';

        if (!current[path]) {
            current[path] = {
                selectedId: 'default',
                configs: [{ id: 'default', name: 'default', args: trimmed }],
            };
        } else {
            const cfg = current[path];
            const active = cfg.configs.find((c) => c.id === cfg.selectedId);
            if (active) {
                active.args = trimmed;
            }
        }

        await this._context.globalState.update('launchArgConfigs', current);
        await this._sendPackageList();
    }

    private async _setLaunchArgConfigs(
        path: string,
        selectedId: string,
        configs: Array<{ id: string; name: string; args: string }>,
    ) {
        if (!path) {
            return;
        }

        const current = this._context.globalState.get<Record<string, { selectedId: string; configs: Array<{ id: string; name: string; args: string }> }>>('launchArgConfigs', {});
        current[path] = {
            selectedId,
            configs: configs.map((c) => ({
                id: c.id,
                name: c.name?.trim() || 'config',
                args: c.args?.trim() || '',
            })),
        };

        await this._context.globalState.update('launchArgConfigs', current);
        await this._sendPackageList();
    }

    private async _selectLaunchArgConfig(path: string, id: string) {
        if (!path || !id) {
            return;
        }
        const current = this._context.globalState.get<Record<string, { selectedId: string; configs: Array<{ id: string; name: string; args: string }> }>>('launchArgConfigs', {});
        if (!current[path]) {
            return;
        }
        current[path].selectedId = id;
        await this._context.globalState.update('launchArgConfigs', current);
        await this._sendPackageList();
    }

    private async _sendLaunchArgOptions(path: string) {
        if (!path) {
            return;
        }

        const options = this._ros.getLaunchArgOptions(path);
        this._view?.webview.postMessage({
            command: 'launchArgsOptions',
            path,
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

    <div class="search-row">
        <input type="text" id="pkgFilter" placeholder="Filter packages or launch files…" />
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
        <h3>Pinned Launch Files</h3>
        <ul class="item-list" id="pinnedList">
            <li class="text-muted">No pinned launch files</li>
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
            <input type="text" id="deps" placeholder="rclpy std_msgs geometry_msgs" />
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
`;

        const script = /* js */ `
const btnCreate      = document.getElementById('btnCreate');
const btnRefresh     = document.getElementById('btnRefresh');
const btnOpenCreate  = document.getElementById('btnOpenCreate');
const btnCloseCreate = document.getElementById('btnCloseCreate');
const btnCancelCreate = document.getElementById('btnCancelCreate');
const createModal    = document.getElementById('createModal');
const createBackdrop = document.getElementById('createBackdrop');
const statusEl       = document.getElementById('createStatus');
const pkgStatusEl    = document.getElementById('pkgStatus');
const filterInput    = document.getElementById('pkgFilter');
const pkgCount       = document.getElementById('pkgCount');
const pinnedList     = document.getElementById('pinnedList');
const terminalList   = document.getElementById('terminalList');
const argsModal      = document.getElementById('argsModal');
const argsBackdrop   = document.getElementById('argsBackdrop');
const argsInput      = document.getElementById('argsInput');
const configName     = document.getElementById('configName');
const configList     = document.getElementById('configList');
const btnAddConfig   = document.getElementById('btnAddConfig');
const btnRemoveConfig = document.getElementById('btnRemoveConfig');
const argsList       = document.getElementById('argsList');
const btnInsertAll   = document.getElementById('btnInsertAll');
const btnSaveArgs    = document.getElementById('btnSaveArgs');
const btnCancelArgs  = document.getElementById('btnCancelArgs');
const btnCloseArgs   = document.getElementById('btnCloseArgs');

let allPackages = [];
let pinnedPaths = [];
let launchArgConfigs = {};
let argsOptions = [];
let currentArgsPath = '';
let terminals = [];
let preferredTerminalId = '';
let currentConfigId = '';

const openCreate = () => {
    createModal.classList.remove('hidden');
    statusEl.className = 'mt hidden';
    statusEl.textContent = '';
    document.getElementById('pkgName').focus();
};

const closeCreate = () => {
    createModal.classList.add('hidden');
};

const openArgsModal = (path) => {
    currentArgsPath = path;
    const cfg = launchArgConfigs[path] || { selectedId: 'default', configs: [{ id: 'default', name: 'default', args: '' }] };
    launchArgConfigs[path] = cfg;
    currentConfigId = cfg.selectedId || (cfg.configs[0]?.id || 'default');
    const currentCfg = cfg.configs.find(c => c.id === currentConfigId) || cfg.configs[0];
    argsInput.value = currentCfg?.args || '';
    configName.value = currentCfg?.name || '';
    argsOptions = [];
    renderArgsOptions();
    renderConfigTabs();
    argsModal.classList.remove('hidden');
    argsInput.focus();
    if (currentArgsPath) {
        vscode.postMessage({ command: 'requestLaunchArgs', path: currentArgsPath });
    }
};

const closeArgsModal = () => {
    argsModal.classList.add('hidden');
};

const renderArgsOptions = () => {
    if (!argsOptions.length) {
        argsList.innerHTML = '<li class="text-muted">No arguments detected</li>';
        return;
    }

    argsList.innerHTML = argsOptions.map(opt => {
        const value = opt.defaultValue ? opt.name + ':=' + opt.defaultValue : opt.name + ':=';
        const label = opt.defaultValue ? opt.name + ' (default ' + opt.defaultValue + ')' : opt.name + ' (no default)';
        return (
            '<li class="arg-item" data-value="' + value + '">' +
                '<span>' + label + '</span>' +
                '<button class="secondary small">Add</button>' +
            '</li>'
        );
    }).join('');

    argsList.querySelectorAll('.arg-item').forEach(el => {
        const btn = el.querySelector('button');
        btn.addEventListener('click', () => {
            const val = el.dataset.value;
            if (!val) { return; }
            const current = argsInput.value.trim();
            argsInput.value = current ? current + ' ' + val : val;
        });
    });
};

const renderConfigTabs = () => {
    if (!configList) { return; }
    const cfg = launchArgConfigs[currentArgsPath];
    if (!cfg || !cfg.configs?.length) {
        configList.innerHTML = '<span class="text-muted text-sm">No configs</span>';
        return;
    }

    configList.innerHTML = cfg.configs.map(c => {
        const active = c.id === currentConfigId ? ' active' : '';
        return '<button class="config-pill' + active + '" data-id="' + c.id + '">' + c.name + '</button>';
    }).join('');

    configList.querySelectorAll('.config-pill').forEach(el => {
        el.addEventListener('click', () => {
            const id = el.dataset.id;
            if (!id) { return; }
            currentConfigId = id;
            const cfgItem = cfg.configs.find(c => c.id === currentConfigId);
            argsInput.value = cfgItem?.args || '';
            configName.value = cfgItem?.name || '';
            renderConfigTabs();
        });
    });
};

const getRunningLaunchPaths = () => {
    const running = new Set();
    (terminals || []).forEach(t => {
        if (t.launchPath) { running.add(t.launchPath); }
    });
    return running;
};

const renderTerminals = () => {
    if (!terminalList) { return; }

    if (!terminals.length) {
        terminalList.innerHTML = '<li class="text-muted">No active launch terminals</li>';
        return;
    }

    terminalList.innerHTML = terminals.map(t => {
        const kindLabel = t.kind === 'external' ? 'External' : 'VS Code';
        const kindBadge = '<span class="badge info">' + kindLabel + '</span>';
        const statusBadge = t.status === 'running'
            ? '<span class="badge success">running</span>'
            : '<span class="badge error">closed</span>';
        const launchLabel = t.launchLabel ? t.launchLabel : 'Idle';
        const preferred = t.isPreferred ? '<span class="badge info">active</span>' : '';
        const useBtn = t.kind === 'integrated'
            ? '<button class="secondary small term-use">Use</button>'
            : '';
        const focusBtn = t.kind === 'integrated'
            ? '<button class="secondary small term-focus">Focus</button>'
            : '';

        return (
            '<li class="terminal-item" data-id="' + t.id + '">' +
                '<div class="terminal-main">' +
                    '<div class="terminal-title">' +
                        '<span class="terminal-name">' + t.name + '</span>' +
                        kindBadge +
                        statusBadge +
                        preferred +
                    '</div>' +
                    '<div class="terminal-sub text-muted text-sm">' + launchLabel + '</div>' +
                '</div>' +
                '<div class="terminal-actions">' +
                    useBtn +
                    focusBtn +
                    '<button class="danger small term-kill">Kill</button>' +
                '</div>' +
            '</li>'
        );
    }).join('');

    terminalList.querySelectorAll('.terminal-item').forEach(el => {
        const id = el.dataset.id;
        const useBtn = el.querySelector('.term-use');
        const focusBtn = el.querySelector('.term-focus');
        const killBtn = el.querySelector('.term-kill');

        if (useBtn) {
            useBtn.addEventListener('click', () => setPreferredTerminal(id));
        }
        if (focusBtn) {
            focusBtn.addEventListener('click', () => focusTerminal(id));
        }
        if (killBtn) {
            killBtn.addEventListener('click', () => killTerminal(id));
        }
    });
};

const renderPackages = () => {
    const list = document.getElementById('pkgList');
    const filter = filterInput.value.trim().toLowerCase();
    const running = getRunningLaunchPaths();
    const filtered = allPackages.filter(p => {
        if (!filter) { return true; }
        const nameMatch = p.name.toLowerCase().includes(filter);
        const launchMatch = (p.launchFiles || []).some(f => f.toLowerCase().includes(filter));
        return nameMatch || launchMatch;
    });
    pkgCount.textContent = String(filtered.length);

    if (filtered.length === 0) {
        list.innerHTML = '<li class="text-muted">No packages found</li>';
        return;
    }

    list.innerHTML = filtered.map(p => {
        const launchFiles = (p.launchFiles || []).map(f => {
            const fileName = f.split('/').pop();
            const isPinned = pinnedPaths.includes(f);
            const cfg = launchArgConfigs[f];
            const configButtons = cfg?.configs?.length
                ? cfg.configs.map(c => {
                    const active = c.id === cfg.selectedId ? ' active' : '';
                    return '<button class="config-pill small' + active + '" data-path="' + f + '" data-id="' + c.id + '">' + c.name + '</button>';
                }).join('')
                : '';
            const runningBadge = running.has(f) ? '<span class="badge success">running</span>' : '';
            const runningClass = running.has(f) ? ' running' : '';
            return (
                '<li class="launch-item' + runningClass + '" data-path="' + f + '" data-pkg="' + p.name + '" data-file="' + fileName + '">' +
                    '<span class="launch-action launch-run" tabindex="0">Run</span>' +
                    '<span class="launch-action launch-open" tabindex="0">' + fileName + '</span>' +
                    '<button class="args-btn" title="Edit args">⚙</button>' +
                    '<span class="config-pill-group">' + configButtons + '</span>' +
                    runningBadge +
                    '<button class="pin-btn ' + (isPinned ? 'pinned' : '') + '" title="Pin">★</button>' +
                '</li>'
            );
        }).join('');

        const launchSection = launchFiles
            ? '<ul class="launch-list">' + launchFiles + '</ul>'
            : '<div class="text-muted text-sm">No launch files</div>';

        return (
            '<li class="pkg-row">' +
                '<div class="pkg-header">' +
                    '<span class="pkg-name" tabindex="0" data-name="' + p.name + '">' + p.name + '</span>' +
                    '<span class="text-muted text-sm">' + (p.launchFiles?.length || 0) + ' launch</span>' +
                '</div>' +
                '<div class="pkg-launches">' + launchSection + '</div>' +
            '</li>'
        );
    }).join('');

    list.querySelectorAll('.pkg-name').forEach(el => {
        el.addEventListener('click', () => copyPackage(el.dataset.name));
        el.addEventListener('keydown', (e) => {
            if (e.key === 'Enter') { copyPackage(el.dataset.name); }
        });
    });

    list.querySelectorAll('.launch-item').forEach(el => {
        const runEl = el.querySelector('.launch-run');
        const openEl = el.querySelector('.launch-open');
        const pinEl = el.querySelector('.pin-btn');
        const argsEl = el.querySelector('.args-btn');
        const configEls = el.querySelectorAll('.config-pill');

        runEl.addEventListener('click', () => launchFile(el.dataset.pkg, el.dataset.file, el.dataset.path, ''));
        runEl.addEventListener('keydown', (e) => { if (e.key === 'Enter') { launchFile(el.dataset.pkg, el.dataset.file, el.dataset.path, ''); } });

        openEl.addEventListener('click', () => openLaunch(el.dataset.path));
        openEl.addEventListener('keydown', (e) => { if (e.key === 'Enter') { openLaunch(el.dataset.path); } });

        argsEl.addEventListener('click', () => editLaunchArgs(el.dataset.path));

        configEls.forEach(btn => {
            btn.addEventListener('click', (e) => {
                e.stopPropagation();
                const id = btn.dataset.id;
                const path = btn.dataset.path;
                if (id && path) {
                    selectLaunchArgConfig(path, id);
                    const cfg = launchArgConfigs[path];
                    const match = cfg?.configs?.find(c => c.id === id);
                    const args = match?.args || '';
                    const argsName = match?.name || '';
                    launchFile(el.dataset.pkg, el.dataset.file, path, args, argsName);
                }
            });
        });

        pinEl.addEventListener('click', () => togglePin(el.dataset.path));
    });
};

const renderPinned = () => {
    if (!pinnedPaths.length) {
        pinnedList.innerHTML = '<li class="text-muted">No pinned launch files</li>';
        return;
    }

    const running = getRunningLaunchPaths();
    const pinnedItems = [];
    for (const pkg of allPackages) {
        for (const file of (pkg.launchFiles || [])) {
            if (!pinnedPaths.includes(file)) { continue; }
            const fileName = file.split('/').pop();
            const cfg = launchArgConfigs[file];
            const configButtons = cfg?.configs?.length
                ? cfg.configs.map(c => {
                    const active = c.id === cfg.selectedId ? ' active' : '';
                    return '<button class="config-pill small' + active + '" data-path="' + file + '" data-id="' + c.id + '">' + c.name + '</button>';
                }).join('')
                : '';
            const runningBadge = running.has(file) ? '<span class="badge success">running</span>' : '';
            const runningClass = running.has(file) ? ' running' : '';
            pinnedItems.push(
                '<li class="launch-item' + runningClass + '" data-path="' + file + '" data-pkg="' + pkg.name + '" data-file="' + fileName + '">' +
                    '<span class="launch-action launch-run" tabindex="0">Run</span>' +
                    '<span class="launch-action launch-open" tabindex="0">' + pkg.name + ' / ' + fileName + '</span>' +
                    '<button class="args-btn" title="Edit args">⚙</button>' +
                    '<span class="config-pill-group">' + configButtons + '</span>' +
                    runningBadge +
                    '<button class="pin-btn pinned" title="Unpin">★</button>' +
                '</li>'
            );
        }
    }

    pinnedList.innerHTML = pinnedItems.length
        ? pinnedItems.join('')
        : '<li class="text-muted">No pinned launch files</li>';

    pinnedList.querySelectorAll('.launch-item').forEach(el => {
        const runEl = el.querySelector('.launch-run');
        const openEl = el.querySelector('.launch-open');
        const pinEl = el.querySelector('.pin-btn');
        const argsEl = el.querySelector('.args-btn');
        const configEls = el.querySelectorAll('.config-pill');

        runEl.addEventListener('click', () => launchFile(el.dataset.pkg, el.dataset.file, el.dataset.path, ''));
        runEl.addEventListener('keydown', (e) => { if (e.key === 'Enter') { launchFile(el.dataset.pkg, el.dataset.file, el.dataset.path, ''); } });

        openEl.addEventListener('click', () => openLaunch(el.dataset.path));
        openEl.addEventListener('keydown', (e) => { if (e.key === 'Enter') { openLaunch(el.dataset.path); } });

        argsEl.addEventListener('click', () => editLaunchArgs(el.dataset.path));

        configEls.forEach(btn => {
            btn.addEventListener('click', (e) => {
                e.stopPropagation();
                const id = btn.dataset.id;
                const path = btn.dataset.path;
                if (id && path) {
                    selectLaunchArgConfig(path, id);
                    const cfg = launchArgConfigs[path];
                    const match = cfg?.configs?.find(c => c.id === id);
                    const args = match?.args || '';
                    const argsName = match?.name || '';
                    launchFile(el.dataset.pkg, el.dataset.file, path, args, argsName);
                }
            });
        });

        pinEl.addEventListener('click', () => togglePin(el.dataset.path));
    });
};

const copyPackage = async (name) => {
    try {
        await navigator.clipboard.writeText(name);
        pkgStatusEl.textContent = 'Copied "' + name + '"';
        setTimeout(() => { pkgStatusEl.textContent = ''; }, 1200);
    } catch {
        pkgStatusEl.textContent = 'Failed to copy "' + name + '"';
    }
};

const openLaunch = (path) => {
    vscode.postMessage({ command: 'openLaunch', path });
};

const launchFile = (pkg, file, path, args, argsName) => {
    vscode.postMessage({ command: 'launchFile', pkg, file, path, args, argsName });
};

const togglePin = (path) => {
    vscode.postMessage({ command: 'togglePin', path });
};

const editLaunchArgs = (path) => {
    openArgsModal(path);
};

const killTerminal = (id) => {
    if (!id) { return; }
    vscode.postMessage({ command: 'killTerminal', id });
};

const focusTerminal = (id) => {
    if (!id) { return; }
    vscode.postMessage({ command: 'focusTerminal', id });
};

const setPreferredTerminal = (id) => {
    if (!id) { return; }
    preferredTerminalId = id;
    vscode.postMessage({ command: 'setPreferredTerminal', id });
};

const selectLaunchArgConfig = (path, id) => {
    if (!path || !id) { return; }
    vscode.postMessage({ command: 'selectLaunchArgConfig', path, id });
};

btnCreate.addEventListener('click', () => {
    const name      = document.getElementById('pkgName').value.trim();
    const buildType = document.getElementById('buildType').value;
    const deps      = document.getElementById('deps').value.trim();
    if (!name) { return; }
    btnCreate.disabled = true;
    statusEl.className = 'mt';
    statusEl.innerHTML = '<span class="spinner"></span> Creating…';
    vscode.postMessage({ command: 'createPackage', name, buildType, deps });
});

btnOpenCreate.addEventListener('click', openCreate);
btnCloseCreate.addEventListener('click', closeCreate);
btnCancelCreate.addEventListener('click', closeCreate);
createBackdrop.addEventListener('click', closeCreate);

btnRefresh.addEventListener('click', () => {
    vscode.postMessage({ command: 'refreshPackages' });
});

filterInput.addEventListener('input', renderPackages);

btnInsertAll.addEventListener('click', () => {
    if (!argsOptions.length) { return; }
    const values = argsOptions.map(opt => opt.defaultValue ? opt.name + ':=' + opt.defaultValue : opt.name + ':=').join(' ');
    const current = argsInput.value.trim();
    argsInput.value = current ? current + ' ' + values : values;
});

btnSaveArgs.addEventListener('click', () => {
    const cfg = launchArgConfigs[currentArgsPath] || { selectedId: 'default', configs: [] };
    const list = cfg.configs.length ? cfg.configs : [{ id: 'default', name: 'default', args: '' }];
    const idx = list.findIndex(c => c.id === currentConfigId);
    if (idx >= 0) {
        list[idx].args = argsInput.value;
        list[idx].name = configName.value || list[idx].name || 'config';
    }
    cfg.configs = list;
    cfg.selectedId = currentConfigId || list[0].id;
    launchArgConfigs[currentArgsPath] = cfg;
    vscode.postMessage({ command: 'setLaunchArgConfigs', path: currentArgsPath, selectedId: cfg.selectedId, configs: cfg.configs });
    closeArgsModal();
});

btnAddConfig.addEventListener('click', () => {
    const cfg = launchArgConfigs[currentArgsPath] || { selectedId: 'default', configs: [] };
    const id = 'cfg-' + Date.now();
    const name = 'config ' + (cfg.configs.length + 1);
    cfg.configs.push({ id, name, args: '' });
    cfg.selectedId = id;
    launchArgConfigs[currentArgsPath] = cfg;
    currentConfigId = id;
    configName.value = name;
    argsInput.value = '';
    renderConfigTabs();
});

btnRemoveConfig.addEventListener('click', () => {
    const cfg = launchArgConfigs[currentArgsPath];
    if (!cfg || cfg.configs.length <= 1) { return; }
    cfg.configs = cfg.configs.filter(c => c.id !== currentConfigId);
    cfg.selectedId = cfg.configs[0].id;
    currentConfigId = cfg.selectedId;
    const currentCfg = cfg.configs[0];
    configName.value = currentCfg?.name || '';
    argsInput.value = currentCfg?.args || '';
    launchArgConfigs[currentArgsPath] = cfg;
    renderConfigTabs();
});

btnCancelArgs.addEventListener('click', closeArgsModal);
btnCloseArgs.addEventListener('click', closeArgsModal);
argsBackdrop.addEventListener('click', closeArgsModal);

window.addEventListener('message', (event) => {
    const msg = event.data;
    switch (msg.command) {
        case 'packageList': {
            allPackages = msg.packages.slice().sort((a, b) => a.name.localeCompare(b.name));
            pinnedPaths = msg.pinned || [];
            launchArgConfigs = msg.launchArgConfigs || {};
            terminals = msg.terminals || [];
            preferredTerminalId = msg.preferredTerminalId || '';
            renderPackages();
            renderPinned();
            renderTerminals();
            break;
        }
        case 'createDone': {
            btnCreate.disabled = false;
            statusEl.innerHTML = msg.success
                ? '<span class="badge success">✓ Created</span>'
                : '<span class="badge error">✗ Failed</span>';
            if (msg.success) {
                vscode.postMessage({ command: 'refreshPackages' });
                closeCreate();
            }
            break;
        }
        case 'focusCreate': {
            openCreate();
            break;
        }
        case 'launchArgsOptions': {
            if (msg.path !== currentArgsPath) { break; }
            argsOptions = msg.options || [];
            renderArgsOptions();
            break;
        }
        case 'terminalList': {
            terminals = msg.terminals || [];
            renderPackages();
            renderPinned();
            renderTerminals();
            break;
        }
    }
});
`;

        return getWebviewHtml(webview, this._extensionUri, body, script);
    }
}
