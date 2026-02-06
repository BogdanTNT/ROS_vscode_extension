import * as vscode from 'vscode';
import { RosWorkspace } from '../ros/rosWorkspace';
import { getWebviewHtml } from './webviewHelper';

/**
 * Sidebar webview for building packages and running nodes / launch files.
 */
export class BuildRunViewProvider implements vscode.WebviewViewProvider {
    private _view?: vscode.WebviewView;

    constructor(
        private readonly _extensionUri: vscode.Uri,
        private readonly _ros: RosWorkspace,
    ) {}

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

        webviewView.webview.onDidReceiveMessage(async (msg) => {
            switch (msg.command) {
                case 'build':
                    this._ros.buildPackages(msg.packages ?? []);
                    break;
                case 'run':
                    this._ros.runNode(msg.pkg, msg.executable, msg.args ?? '');
                    break;
                case 'launch':
                    this._ros.launchFile(msg.pkg, msg.launchFile);
                    break;
                case 'refreshPackages':
                    await this._sendPackageList();
                    break;
                case 'sourceWorkspace':
                    this._sourceWorkspace();
                    break;
                case 'cleanBuild':
                    this._cleanBuild();
                    break;
            }
        });

        this._sendPackageList();
    }

    /** Trigger a build from the command palette. */
    triggerBuild(): void {
        this._view?.show(true);
        this._view?.webview.postMessage({ command: 'focusBuild' });
    }

    /** Trigger a run from the command palette. */
    triggerRun(): void {
        this._view?.show(true);
        this._view?.webview.postMessage({ command: 'focusRun' });
    }

    // ── Private ────────────────────────────────────────────────
    private async _sendPackageList() {
        const packages = await this._ros.listPackages();
        this._view?.webview.postMessage({ command: 'packageList', packages });
    }

    private _sourceWorkspace(): void {
        const wsPath = this._ros.getWorkspacePath();
        const cmd = this._ros.isRos2()
            ? `source "${wsPath}/install/setup.bash"`
            : `source "${wsPath}/devel/setup.bash"`;
        this._ros.runInTerminal(cmd);
    }

    private _cleanBuild(): void {
        const wsPath = this._ros.getWorkspacePath();
        const cmd = this._ros.isRos2()
            ? `cd "${wsPath}" && rm -rf build install log && colcon build`
            : `cd "${wsPath}" && catkin_make clean && catkin_make`;
        this._ros.runInTerminal(cmd);
    }

    private _getHtml(webview: vscode.Webview): string {
        const body = /* html */ `
<h2>🔨 Build &amp; Run</h2>

<!-- ── Build Section ──────────────────────────────────────── -->
<div class="section">
    <h3>Build</h3>
    <div class="card">
        <label for="buildPkgs">Packages to build <span class="text-muted text-sm">(blank = all)</span></label>
        <input type="text" id="buildPkgs" placeholder="my_robot_pkg controller_pkg" />

        <div class="btn-row">
            <button id="btnBuild">▶ Build</button>
            <button class="secondary" id="btnClean">🗑 Clean Build</button>
            <button class="secondary" id="btnSource">⟳ Source WS</button>
        </div>
    </div>
</div>

<!-- ── Run Node Section ───────────────────────────────────── -->
<div class="section">
    <h3>Run Node</h3>
    <div class="card">
        <label for="runPkg">Package</label>
        <input type="text" id="runPkg" placeholder="my_robot_pkg" list="pkgSuggestions" />

        <label for="runExec">Executable / Node name</label>
        <input type="text" id="runExec" placeholder="talker" />

        <label for="runArgs">Extra arguments</label>
        <input type="text" id="runArgs" placeholder="--ros-args -p param:=value" />

        <div class="btn-row">
            <button id="btnRun">▶ Run</button>
        </div>
    </div>
</div>

<!-- ── Launch File Section ────────────────────────────────── -->
<div class="section">
    <h3>Launch File</h3>
    <div class="card">
        <label for="launchPkg">Package</label>
        <input type="text" id="launchPkg" placeholder="my_robot_pkg" list="pkgSuggestions" />

        <label for="launchFile">Launch file</label>
        <input type="text" id="launchFile" placeholder="robot.launch.py" />

        <div class="btn-row">
            <button id="btnLaunch">🚀 Launch</button>
        </div>
    </div>
</div>

<datalist id="pkgSuggestions"></datalist>
`;
        const scriptUris = [
            vscode.Uri.joinPath(this._extensionUri, 'media', 'buildRun', 'index.js'),
        ];
        return getWebviewHtml(webview, this._extensionUri, body, '', undefined, scriptUris);
    }
}
