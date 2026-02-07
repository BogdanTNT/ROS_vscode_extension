import * as vscode from 'vscode';
import { RosNodeGraphInfo, RosWorkspace } from '../ros/rosWorkspace';
import { getWebviewHtml } from './webviewHelper';

const AUTO_REFRESH_INTERVAL_MS = 3000;

/**
 * Sidebar webview for runtime ROS graph exploration.
 */
export class NodeVisualizerViewProvider implements vscode.WebviewViewProvider {
    private _view?: vscode.WebviewView;
    private _autoRefreshEnabled = false;
    private _autoRefreshTimer?: NodeJS.Timeout;
    private _refreshInFlight = false;
    private _refreshQueued = false;
    private _queuedShowLoading = false;

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
                case 'refresh':
                    await this._sendGraphData({ showLoading: true });
                    break;
                case 'setAutoRefresh':
                    this._autoRefreshEnabled = !!msg.enabled;
                    this._syncAutoRefresh();
                    break;
                case 'getTopicLatestMessage':
                    await this._sendTopicLatestMessage(msg.topicName);
                    break;
            }
        });

        webviewView.onDidChangeVisibility(() => {
            this._syncAutoRefresh();
        });
        webviewView.onDidDispose(() => {
            this._stopAutoRefresh();
            this._view = undefined;
        });

        this._syncAutoRefresh();
        this._sendGraphData({ showLoading: true });
    }

    /** Externally triggered refresh (e.g. from command palette). */
    refreshGraph(): void {
        this._view?.show(true);
        this._syncAutoRefresh();
        this._sendGraphData({ showLoading: true });
    }

    // ── Private ────────────────────────────────────────────────
    private _syncAutoRefresh() {
        if (this._view?.visible && this._autoRefreshEnabled) {
            const wasRunning = !!this._autoRefreshTimer;
            this._startAutoRefresh();
            if (!wasRunning) {
                void this._sendGraphData();
            }
            return;
        }
        this._stopAutoRefresh();
    }

    private _startAutoRefresh() {
        if (this._autoRefreshTimer) {
            return;
        }

        this._autoRefreshTimer = setInterval(() => {
            void this._sendGraphData();
        }, AUTO_REFRESH_INTERVAL_MS);
    }

    private _stopAutoRefresh() {
        if (!this._autoRefreshTimer) {
            return;
        }
        clearInterval(this._autoRefreshTimer);
        this._autoRefreshTimer = undefined;
    }

    private async _sendGraphData(options?: { showLoading?: boolean }) {
        const view = this._view;
        if (!view) {
            return;
        }

        if (this._refreshInFlight) {
            // Coalesce bursts of refresh requests (manual + timer) into one
            // follow-up fetch to avoid overlapping ROS CLI calls.
            this._refreshQueued = true;
            this._queuedShowLoading = this._queuedShowLoading || !!options?.showLoading;
            return;
        }

        this._refreshInFlight = true;
        try {
            if (options?.showLoading) {
                view.webview.postMessage({ command: 'loading' });
            }

            const [nodes, topics, services, actions] = await Promise.all([
                this._ros.getNodeList(),
                this._ros.getTopicList(),
                this._ros.getServiceList(),
                this._ros.getActionList(),
            ]);

            const nodeEntries = await Promise.all(
                nodes.map(async (node) => {
                    const info = await this._ros.getNodeGraphInfo(node);
                    return [node, info] as const;
                }),
            );
            const connections: Record<string, RosNodeGraphInfo> = Object.fromEntries(nodeEntries);

            if (this._view === view) {
                view.webview.postMessage({
                    command: 'graphData',
                    nodes,
                    topics,
                    services,
                    actions,
                    connections,
                    rosVersion: this._ros.isRos2() ? 2 : 1,
                });
            }
        } finally {
            this._refreshInFlight = false;
            if (this._refreshQueued) {
                const queuedShowLoading = this._queuedShowLoading;
                this._refreshQueued = false;
                this._queuedShowLoading = false;
                await this._sendGraphData({ showLoading: queuedShowLoading });
            }
        }
    }

    private async _sendTopicLatestMessage(topicName: string) {
        const view = this._view;
        if (!view || typeof topicName !== 'string' || !topicName) {
            return;
        }

        const message = await this._ros.getLatestTopicMessage(topicName);
        if (this._view === view) {
            view.webview.postMessage({
                command: 'topicLatestMessage',
                topicName,
                message: message ?? '',
            });
        }
    }

    private _getHtml(webview: vscode.Webview): string {
        const body = /* html */ `
<h2>🔗 Node Visualizer</h2>

<div class="nv-toolbar mb">
    <button id="btnRefresh">↻ Refresh</button>
    <label class="nv-layer">
        <input type="checkbox" id="toggleAutoRefresh" />
        <span>Auto refresh</span>
    </label>
</div>

<div id="status" class="mb text-muted text-sm">Connecting to ROS…</div>
<div id="summary" class="nv-summary mb"></div>

<div class="section mb">
    <h3>Details</h3>
    <div class="card nv-details-card" id="detailsCard">
        <span class="text-muted">Select an item to see details.</span>
    </div>
</div>

<div class="nv-tabs mb" role="tablist" aria-label="Node visualizer views">
    <button class="secondary nv-tab active" data-view="overview">Overview</button>
    <button class="secondary nv-tab" data-view="nodes">Nodes</button>
    <button class="secondary nv-tab" data-view="topics">Topics</button>
    <button class="secondary nv-tab" data-view="services">Services</button>
    <button class="secondary nv-tab" data-view="actions">Actions</button>
</div>

<div id="overviewView" class="nv-view">
    <div class="nv-layer-row mb">
        <label class="nv-layer">
            <input type="checkbox" id="toggleNodes" checked />
            <span>Nodes</span>
        </label>
        <label class="nv-layer">
            <input type="checkbox" id="toggleTopics" checked />
            <span>Topics</span>
        </label>
        <label class="nv-layer">
            <input type="checkbox" id="toggleServices" checked />
            <span>Services</span>
        </label>
        <label class="nv-layer">
            <input type="checkbox" id="toggleActions" checked />
            <span>Actions</span>
        </label>
    </div>

    <div id="overviewSections" class="nv-overview-sections"></div>
</div>

<div id="listView" class="nv-view hidden">
    <div class="search-row">
        <input type="text" id="entityFilter" placeholder="Filter..." />
        <span id="entityCount" class="badge info">0</span>
    </div>

    <ul class="item-list nv-entity-list" id="entityList">
        <li class="text-muted">No data loaded.</li>
    </ul>
</div>
`;
        const scriptUris = [
            vscode.Uri.joinPath(this._extensionUri, 'media', 'nodeVisualizer', 'index.js'),
        ];
        return getWebviewHtml(webview, this._extensionUri, body, '', undefined, scriptUris);
    }
}
