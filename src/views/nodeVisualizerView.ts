import * as vscode from 'vscode';
import {
    RosNodeGraphInfo,
    RosTopicPublishResult,
    RosTopicPublishTemplateResult,
    RosWorkspace,
} from '../ros/rosWorkspace';
import { getWebviewHtml } from './webviewHelper';

const AUTO_REFRESH_INTERVAL_MS = 3000;
const TRACKED_TOPIC_MESSAGE_REFRESH_CONCURRENCY = 3;
const TRACKED_TOPIC_ECHO_TIMEOUT_SECONDS = 3;
const NODE_VISUALIZER_PREFS_KEY = 'rosDevToolkit.nodeVisualizerPrefs';

interface NodeVisualizerPrefs {
    autoRefreshEnabled: boolean;
    showNodes: boolean;
    showTopics: boolean;
    showServices: boolean;
    showActions: boolean;
    showParameters: boolean;
}

const DEFAULT_NODE_VISUALIZER_PREFS: NodeVisualizerPrefs = {
    autoRefreshEnabled: false,
    showNodes: true,
    showTopics: true,
    showServices: true,
    showActions: true,
    showParameters: true,
};

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
    private _topicLatestMessages: Record<string, string> = {};
    private _trackedTopicMessageRefreshInFlight = false;
    private _queuedTrackedTopicNames = new Set<string>();
    private _prefs: NodeVisualizerPrefs;

    constructor(
        private readonly _extensionUri: vscode.Uri,
        private readonly _ros: RosWorkspace,
        private readonly _context: vscode.ExtensionContext,
    ) {
        this._prefs = this._loadPrefs();
        this._autoRefreshEnabled = this._prefs.autoRefreshEnabled;
    }

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
                    this._prefs.autoRefreshEnabled = this._autoRefreshEnabled;
                    await this._savePrefs();
                    this._syncAutoRefresh();
                    break;
                case 'setOverviewToggles':
                    this._prefs.showNodes = msg.showNodes !== false;
                    this._prefs.showTopics = msg.showTopics !== false;
                    this._prefs.showServices = msg.showServices !== false;
                    this._prefs.showActions = msg.showActions !== false;
                    this._prefs.showParameters = msg.showParameters !== false;
                    await this._savePrefs();
                    break;
                case 'refreshTrackedTopicMessages':
                    await this._refreshTrackedTopicMessages(msg.topicNames, webviewView);
                    break;
                case 'getTopicPublishTemplate':
                    await this._sendTopicPublishTemplate(msg.topicName, msg.topicTypeHint);
                    break;
                case 'publishTopicMessage':
                    await this._publishTopicMessage(msg.topicName, msg.payload, msg.topicTypeHint);
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

            const [nodes, topics, services, actions, parameters] = await Promise.all([
                this._ros.getNodeList(),
                this._ros.getTopicList(),
                this._ros.getServiceList(),
                this._ros.getActionList(),
                this._ros.getParameterList(),
            ]);
            const topicNames = topics.map((topic) => topic.name);
            this._pruneTopicMessageCache(topicNames);

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
                    parameters,
                    connections,
                    topicLatestMessages: this._buildTopicMessageSnapshot(topicNames),
                    rosVersion: this._ros.isRos2() ? 2 : 1,
                    prefs: this._prefs,
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

    private _pruneTopicMessageCache(topicNames: string[]) {
        const validTopics = new Set(topicNames);
        for (const topicName of Object.keys(this._topicLatestMessages)) {
            if (!validTopics.has(topicName)) {
                delete this._topicLatestMessages[topicName];
            }
        }
    }

    private _buildTopicMessageSnapshot(topicNames: string[]): Record<string, string> {
        const snapshot: Record<string, string> = {};
        for (const topicName of topicNames) {
            const cached = this._topicLatestMessages[topicName];
            if (cached) {
                snapshot[topicName] = cached;
            }
        }
        return snapshot;
    }

    private async _refreshTrackedTopicMessages(topicNamesRaw: unknown, view: vscode.WebviewView) {
        if (this._view !== view) {
            return;
        }

        const topicNames = Array.isArray(topicNamesRaw)
            ? Array.from(new Set(topicNamesRaw.map((value) => String(value || '').trim()).filter(Boolean)))
            : [];
        if (!topicNames.length) {
            return;
        }

        topicNames.forEach((topicName) => this._queuedTrackedTopicNames.add(topicName));
        if (this._trackedTopicMessageRefreshInFlight) {
            return;
        }

        this._trackedTopicMessageRefreshInFlight = true;
        try {
            while (this._queuedTrackedTopicNames.size && this._view === view) {
                const batch = Array.from(this._queuedTrackedTopicNames);
                this._queuedTrackedTopicNames.clear();

                let cursor = 0;
                const worker = async () => {
                    while (cursor < batch.length) {
                        const topicName = batch[cursor++];
                        const message = await this._ros.getLatestTopicMessage(
                            topicName,
                            TRACKED_TOPIC_ECHO_TIMEOUT_SECONDS,
                        );
                        if (this._view !== view) {
                            return;
                        }

                        const normalized = String(message ?? '').trim();
                        if (!normalized) {
                            continue;
                        }

                        if (this._topicLatestMessages[topicName] === normalized) {
                            continue;
                        }

                        this._topicLatestMessages[topicName] = normalized;
                        view.webview.postMessage({
                            command: 'topicLatestMessage',
                            topicName,
                            message: normalized,
                        });
                    }
                };

                const workerCount = Math.min(TRACKED_TOPIC_MESSAGE_REFRESH_CONCURRENCY, batch.length);
                await Promise.all(Array.from({ length: workerCount }, () => worker()));
            }
        } finally {
            this._trackedTopicMessageRefreshInFlight = false;
        }
    }

    private async _sendTopicPublishTemplate(topicName: string, topicTypeHint?: string) {
        const view = this._view;
        if (!view || typeof topicName !== 'string' || !topicName) {
            return;
        }

        const result: RosTopicPublishTemplateResult = await this._ros.getTopicPublishTemplate(
            topicName,
            typeof topicTypeHint === 'string' ? topicTypeHint : undefined,
        );

        if (this._view === view) {
            view.webview.postMessage({
                command: 'topicPublishTemplate',
                ...result,
            });
        }
    }

    private async _publishTopicMessage(topicName: string, payload: string, topicTypeHint?: string) {
        const view = this._view;
        if (!view || typeof topicName !== 'string' || !topicName) {
            return;
        }

        const result: RosTopicPublishResult = await this._ros.publishTopicMessage(
            topicName,
            String(payload ?? ''),
            typeof topicTypeHint === 'string' ? topicTypeHint : undefined,
        );

        if (this._view === view) {
            view.webview.postMessage({
                command: 'topicPublishResult',
                ...result,
            });
        }
    }

    private _loadPrefs(): NodeVisualizerPrefs {
        const stored = this._context.globalState.get<Partial<NodeVisualizerPrefs>>(NODE_VISUALIZER_PREFS_KEY, {});
        return {
            autoRefreshEnabled: stored.autoRefreshEnabled ?? DEFAULT_NODE_VISUALIZER_PREFS.autoRefreshEnabled,
            showNodes: stored.showNodes ?? DEFAULT_NODE_VISUALIZER_PREFS.showNodes,
            showTopics: stored.showTopics ?? DEFAULT_NODE_VISUALIZER_PREFS.showTopics,
            showServices: stored.showServices ?? DEFAULT_NODE_VISUALIZER_PREFS.showServices,
            showActions: stored.showActions ?? DEFAULT_NODE_VISUALIZER_PREFS.showActions,
            showParameters: stored.showParameters ?? DEFAULT_NODE_VISUALIZER_PREFS.showParameters,
        };
    }

    private async _savePrefs() {
        await this._context.globalState.update(NODE_VISUALIZER_PREFS_KEY, this._prefs);
    }

    private _getHtml(webview: vscode.Webview): string {
        const body = /* html */ `
<div id="status" class="mb text-muted text-sm">Waiting for data…</div>

<div class="nv-tabs mb" role="tablist" aria-label="Node visualizer views">
    <button class="secondary nv-tab active" data-view="overview">
        <span>Overview</span>
    </button>
    <button class="secondary nv-tab" data-view="nodes">
        <span>Nodes</span>
        <span class="badge info nv-tab-count" id="tabNodesCount">0</span>
    </button>
    <button class="secondary nv-tab" data-view="topics">
        <span>Topics</span>
        <span class="badge info nv-tab-count" id="tabTopicsCount">0</span>
    </button>
    <button class="secondary nv-tab" data-view="services">
        <span>Services</span>
        <span class="badge info nv-tab-count" id="tabServicesCount">0</span>
    </button>
    <button class="secondary nv-tab" data-view="actions">
        <span>Actions</span>
        <span class="badge info nv-tab-count" id="tabActionsCount">0</span>
    </button>
    <button class="secondary nv-tab" data-view="parameters">
        <span>Parameters</span>
        <span class="badge info nv-tab-count" id="tabParametersCount">0</span>
    </button>
</div>

<div id="overviewToggleBar" class="nv-layer-row mb">
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
    <label class="nv-layer">
        <input type="checkbox" id="toggleParameters" checked />
        <span>Parameters</span>
    </label>
</div>

<div class="nv-details-header mb">
    <h3>Details</h3>
    <div class="nv-toolbar">
        <button id="btnRefresh">↻ Refresh</button>
        <button class="secondary hidden" id="btnPublishTopic" title="Publish a debug message to a topic">
            ✉ Publish
        </button>
        <label class="nv-layer">
            <input type="checkbox" id="toggleAutoRefresh" />
            <span>Auto refresh</span>
        </label>
    </div>
</div>

<div class="card nv-details-card mb" id="detailsCard">
    <span class="text-muted">Select an item to see details.</span>
</div>

<div class="card nv-pinned-card mb">
    <div class="nv-pinned-card-head">
        <h3>Pinned Topics</h3>
        <button
            class="pin-btn nv-topic-freeze-btn"
            id="btnToggleAllPinnedPause"
            type="button"
            title="Pause all pinned topic reading"
            aria-label="Pause all pinned topic reading"
        >
            ⏸
        </button>
    </div>
    <div id="pinnedTopicList" class="nv-pinned-list">
        <div class="text-muted">No pinned topics yet.</div>
    </div>
</div>

<div id="overviewView" class="nv-view">
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

<div class="modal hidden" id="publishTopicModal" role="dialog" aria-modal="true">
    <div class="modal-backdrop" id="publishTopicBackdrop"></div>
    <div class="modal-card nv-publish-modal-card">
        <div class="modal-header">
            <h3>Publish Topic Message</h3>
            <button class="secondary small" id="btnClosePublishTopic">✕</button>
        </div>
        <div class="modal-body">
            <label for="publishTopicSelect">Topic</label>
            <select id="publishTopicSelect"></select>

            <label for="publishTopicType">Message type</label>
            <input type="text" id="publishTopicType" readonly />

            <label for="publishTopicPayload">Payload (JSON / YAML)</label>
            <textarea id="publishTopicPayload" class="nv-publish-payload" spellcheck="false"></textarea>

            <div class="text-muted text-sm">
                Auto-generated defaults are loaded from the topic message interface.
            </div>
            <div id="publishTopicStatus" class="mt text-sm hidden"></div>
        </div>
        <div class="modal-footer">
            <button class="secondary" id="btnCancelPublishTopic">Cancel</button>
            <button id="btnConfirmPublishTopic">Publish once</button>
        </div>
    </div>
</div>
`;
        const scriptUris = [
            vscode.Uri.joinPath(this._extensionUri, 'media', 'shared', 'interactions.js'),
            vscode.Uri.joinPath(this._extensionUri, 'media', 'nodeVisualizer', 'index.js'),
        ];
        return getWebviewHtml(webview, this._extensionUri, body, '', undefined, scriptUris);
    }
}
