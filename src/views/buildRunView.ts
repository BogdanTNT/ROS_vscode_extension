import * as vscode from 'vscode';
import { RosWorkspace } from '../ros/rosWorkspace';
import { getWebviewHtml } from './webviewHelper';
import { BuildEvaluation } from '../ros/buildPolicy';

/**
 * Sidebar webview for building packages and running nodes / launch files.
 */
export class BuildRunViewProvider implements vscode.WebviewViewProvider {
    private _view?: vscode.WebviewView;
    private _statusListener?: vscode.Disposable;

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
                    this._ros.buildPackages(this._normalizePackageNames(msg.packages));
                    break;
                case 'smartBuild': {
                    const packages = this._normalizePackageNames(msg.packages);
                    const built = this._ros.smartBuildPackages(packages);
                    if (built.length === 0) {
                        this._view?.webview.postMessage({
                            command: 'buildStatus',
                            text: 'All selected packages are up to date — no build needed.',
                        });
                    }
                    break;
                }
                case 'markBuilt': {
                    const packages = this._normalizePackageNames(msg.packages);
                    this._ros.markPackagesBuilt(packages);
                    await this._sendBuildStatus();
                    break;
                }
                case 'refreshPackages':
                    await this._sendPackageList();
                    await this._sendBuildStatus();
                    break;
                case 'requestBuildStatus':
                    await this._sendBuildStatus();
                    break;
            }
        });

        // Listen for file-watcher invalidations to refresh the UI
        this._statusListener = this._ros.onBuildStatusChanged(() => {
            this._sendBuildStatus();
        });

        webviewView.onDidDispose(() => {
            this._statusListener?.dispose();
        });

        this._sendPackageList();
        this._sendBuildStatus();
    }

    /** Trigger a build from the command palette. */
    triggerBuild(): void {
        this._view?.show(true);
        this._view?.webview.postMessage({ command: 'focusBuild' });
    }

    /** Trigger a run from the command palette. */
    triggerRun(): void {
        this._view?.show(true);
        this._view?.webview.postMessage({ command: 'focusBuild' });
    }

    // ── Private ────────────────────────────────────────────────
    private async _sendPackageList() {
        const packages = await this._ros.listWorkspacePackages();
        this._view?.webview.postMessage({ command: 'packageList', packages });
    }

    private async _sendBuildStatus() {
        const evaluation = this._ros.evaluateBuildNeeds();
        if (!evaluation) {
            return;
        }

        const statusMap: Record<string, { needsBuild: boolean; reason: string; reasonCode: string }> = {};
        for (const [name, detail] of evaluation.details) {
            statusMap[name] = {
                needsBuild: detail.needsBuild,
                reason: detail.reason,
                reasonCode: detail.reasonCode,
            };
        }

        this._view?.webview.postMessage({
            command: 'buildStatusMap',
            statusMap,
            packagesNeedingBuild: evaluation.packagesNeedingBuild,
            upToDate: evaluation.upToDate,
        });
    }

    private _normalizePackageNames(rawPackages: unknown): string[] {
        if (!Array.isArray(rawPackages)) {
            return [];
        }

        const seen = new Set<string>();
        const result: string[] = [];

        for (const value of rawPackages) {
            if (typeof value !== 'string') {
                continue;
            }
            const name = value.trim();
            if (!name || seen.has(name)) {
                continue;
            }
            seen.add(name);
            result.push(name);
        }

        return result;
    }

    private _getHtml(webview: vscode.Webview): string {
        const body = /* html */ `
<h2>🔨 Build Packages</h2>
<div class="section">
    <div class="card">
        <div class="toolbar">
            <button class="secondary small" id="btnRefreshPackages">↻ Refresh</button>
        </div>

        <label for="btnPackageDropdown">Workspace packages</label>
        <div class="build-dropdown" id="buildDropdown">
            <button class="secondary" id="btnPackageDropdown" aria-expanded="false" title="Open package selector (Alt+click: select/clear all packages)">
                Select packages
            </button>
            <div class="build-dropdown-menu hidden" id="packageDropdownMenu">
                <div class="build-dropdown-actions">
                    <button class="secondary small" id="btnSelectAllPackages">Select all</button>
                    <button class="secondary small" id="btnClearPackages">Clear</button>
                </div>
                <ul class="build-package-list" id="buildPackageList">
                    <li class="text-muted">Loading workspace packages…</li>
                </ul>
            </div>
        </div>

        <div id="buildSelectionSummary" class="text-muted text-sm mt">
            No packages selected.
        </div>

        <div id="buildStatusSummary" class="text-muted text-sm mt"></div>

        <div class="btn-row mt">
            <button id="btnBuildSelected" disabled>▶ Build Selected</button>
            <button id="btnSmartBuild" class="secondary" disabled>⚡ Smart Build</button>
        </div>
        <div id="buildStatus" class="text-muted text-sm mt"></div>
    </div>
</div>
`;
        const scriptUris = [
            vscode.Uri.joinPath(this._extensionUri, 'media', 'shared', 'interactions.js'),
            vscode.Uri.joinPath(this._extensionUri, 'media', 'buildRun', 'index.js'),
        ];
        return getWebviewHtml(webview, this._extensionUri, body, '', undefined, scriptUris);
    }
}
