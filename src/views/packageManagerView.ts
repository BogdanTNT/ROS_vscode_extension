import * as vscode from 'vscode';
import { RosWorkspace } from '../ros/rosWorkspace';
import { getWebviewHtml } from './webviewHelper';

/**
 * Sidebar webview that lets the user create new ROS packages.
 */
export class PackageManagerViewProvider implements vscode.WebviewViewProvider {
    private _view?: vscode.WebviewView;
    private _pinnedLaunchFiles: string[] = [];
    private _launchArgs: Record<string, string> = {};

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
                    await this._launchFile(msg.pkg, msg.file, msg.path);
                    break;
                case 'togglePin':
                    await this._togglePin(msg.path);
                    break;
                case 'setLaunchArgs':
                    await this._setLaunchArgs(msg.path, msg.args);
                    break;
                case 'requestLaunchArgs':
                    await this._sendLaunchArgOptions(msg.path);
                    break;
            }
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
        this._launchArgs = this._context.globalState.get<Record<string, string>>('launchArgs', {});
        const packages = await this._ros.listWorkspacePackageDetails();
        this._view?.webview.postMessage({
            command: 'packageList',
            packages,
            pinned: this._pinnedLaunchFiles,
            launchArgs: this._launchArgs,
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

    private async _launchFile(pkg: string, file: string, path: string) {
        if (!pkg || !file) {
            return;
        }
        const args = path ? this._launchArgs[path] : undefined;
        this._ros.launchFile(pkg, file, args);
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

        const current = this._context.globalState.get<Record<string, string>>('launchArgs', {});

        if (!args || args.trim().length === 0) {
            delete current[path];
        } else {
            current[path] = args.trim();
        }

        await this._context.globalState.update('launchArgs', current);
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
const argsModal      = document.getElementById('argsModal');
const argsBackdrop   = document.getElementById('argsBackdrop');
const argsInput      = document.getElementById('argsInput');
const argsList       = document.getElementById('argsList');
const btnInsertAll   = document.getElementById('btnInsertAll');
const btnSaveArgs    = document.getElementById('btnSaveArgs');
const btnCancelArgs  = document.getElementById('btnCancelArgs');
const btnCloseArgs   = document.getElementById('btnCloseArgs');

let allPackages = [];
let pinnedPaths = [];
let launchArgs = {};
let argsOptions = [];
let currentArgsPath = '';

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
    argsInput.value = launchArgs[path] || '';
    argsOptions = [];
    renderArgsOptions();
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

const renderPackages = () => {
    const list = document.getElementById('pkgList');
    const filter = filterInput.value.trim().toLowerCase();
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
            const argLabel = launchArgs[f] ? '<span class="badge info">args</span>' : '';
            return (
                '<li class="launch-item" data-path="' + f + '" data-pkg="' + p.name + '" data-file="' + fileName + '">' +
                    '<span class="launch-action launch-run" tabindex="0">Run</span>' +
                    '<span class="launch-action launch-open" tabindex="0">' + fileName + '</span>' +
                    '<button class="args-btn" title="Edit args">⚙</button>' +
                    argLabel +
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

        runEl.addEventListener('click', () => launchFile(el.dataset.pkg, el.dataset.file, el.dataset.path));
        runEl.addEventListener('keydown', (e) => { if (e.key === 'Enter') { launchFile(el.dataset.pkg, el.dataset.file, el.dataset.path); } });

        openEl.addEventListener('click', () => openLaunch(el.dataset.path));
        openEl.addEventListener('keydown', (e) => { if (e.key === 'Enter') { openLaunch(el.dataset.path); } });

        argsEl.addEventListener('click', () => editLaunchArgs(el.dataset.path));

        pinEl.addEventListener('click', () => togglePin(el.dataset.path));
    });
};

const renderPinned = () => {
    if (!pinnedPaths.length) {
        pinnedList.innerHTML = '<li class="text-muted">No pinned launch files</li>';
        return;
    }

    const pinnedItems = [];
    for (const pkg of allPackages) {
        for (const file of (pkg.launchFiles || [])) {
            if (!pinnedPaths.includes(file)) { continue; }
            const fileName = file.split('/').pop();
            const argLabel = launchArgs[file] ? '<span class="badge info">args</span>' : '';
            pinnedItems.push(
                '<li class="launch-item" data-path="' + file + '" data-pkg="' + pkg.name + '" data-file="' + fileName + '">' +
                    '<span class="launch-action launch-run" tabindex="0">Run</span>' +
                    '<span class="launch-action launch-open" tabindex="0">' + pkg.name + ' / ' + fileName + '</span>' +
                    '<button class="args-btn" title="Edit args">⚙</button>' +
                    argLabel +
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

        runEl.addEventListener('click', () => launchFile(el.dataset.pkg, el.dataset.file, el.dataset.path));
        runEl.addEventListener('keydown', (e) => { if (e.key === 'Enter') { launchFile(el.dataset.pkg, el.dataset.file, el.dataset.path); } });

        openEl.addEventListener('click', () => openLaunch(el.dataset.path));
        openEl.addEventListener('keydown', (e) => { if (e.key === 'Enter') { openLaunch(el.dataset.path); } });

        argsEl.addEventListener('click', () => editLaunchArgs(el.dataset.path));

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

const launchFile = (pkg, file, path) => {
    vscode.postMessage({ command: 'launchFile', pkg, file, path });
};

const togglePin = (path) => {
    vscode.postMessage({ command: 'togglePin', path });
};

const editLaunchArgs = (path) => {
    openArgsModal(path);
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
    vscode.postMessage({ command: 'setLaunchArgs', path: currentArgsPath, args: argsInput.value });
    closeArgsModal();
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
            launchArgs = msg.launchArgs || {};
            renderPackages();
            renderPinned();
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
    }
});
`;

        return getWebviewHtml(webview, this._extensionUri, body, script);
    }
}
