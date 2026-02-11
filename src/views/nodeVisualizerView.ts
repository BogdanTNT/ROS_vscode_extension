import * as vscode from 'vscode';
import {
    RosNodeGraphInfo,
    RosTopicMessageSubscription,
    RosTopicPublishResult,
    RosTopicPublishTemplateResult,
    RosWorkspace,
} from '../ros/rosWorkspace';
import { getWebviewHtml } from './webviewHelper';

const DEFAULT_NODE_VISUALIZER_AUTO_REFRESH_INTERVAL_MS = 3000;
const TOPIC_ECHO_RESTART_DELAY_MS = 1200;
const NODE_VISUALIZER_PREFS_KEY = 'rosDevToolkit.nodeVisualizerPrefs';
const NODE_VISUALIZER_AUTO_REFRESH_INTERVAL_SETTING = 'nodeVisualizerAutoRefreshIntervalMs';

/** Which categories to actually fetch during a refresh. */
interface RefreshScope {
    nodes: boolean;
    topics: boolean;
    services: boolean;
    actions: boolean;
    parameters: boolean;
}

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
    private _cachedNodes: string[] = [];
    private _cachedTopics: Array<{ name: string; type: string }> = [];
    private _cachedServices: Array<{ name: string; type: string }> = [];
    private _cachedActions: Array<{ name: string; type: string }> = [];
    private _cachedParameters: Array<{ name: string; node?: string }> = [];
    private _topicLatestMessages: Record<string, string> = {};
    private _topicLatestMessageTimes: Record<string, number> = {};
    private _cachedConnections: Record<string, RosNodeGraphInfo> = {};
    private _cachedGraphPayload?: Record<string, unknown>;
    private _trackedTopicNames = new Set<string>();
    private _topicSubscriptions = new Map<string, RosTopicMessageSubscription>();
    private _topicSubscriptionTokens = new Map<string, number>();
    private _topicSubscriptionRestartTimers = new Map<string, NodeJS.Timeout>();
    private _topicSubscriptionSeq = 0;
    private _connectionsRefreshInFlight = false;
    private _queuedConnectionsRefresh = false;
    private _prefs: NodeVisualizerPrefs;
    private _lastScope: RefreshScope = { nodes: true, topics: true, services: true, actions: true, parameters: true };

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
                case 'refresh': {
                    this._cancelScheduledRefresh();
                    const parsedScope = this._parseScopeMessage(msg as Record<string, unknown>);
                    const hasScope = parsedScope.nodes
                        || parsedScope.topics
                        || parsedScope.services
                        || parsedScope.actions
                        || parsedScope.parameters;
                    if (hasScope) {
                        this._lastScope = parsedScope;
                    }
                    await this._sendGraphData({
                        showLoading: msg.showLoading !== false,
                        scope: hasScope ? parsedScope : undefined,
                    });
                    break;
                }
                case 'setAutoRefresh':
                    this._autoRefreshEnabled = !!msg.enabled;
                    this._prefs.autoRefreshEnabled = this._autoRefreshEnabled;
                    await this._savePrefs();
                    this._syncAutoRefresh();
                    break;
                case 'viewScope':
                    this._lastScope = this._parseScopeMessage(msg);
                    break;
                case 'fetchNodeInfo':
                    await this._fetchAndSendNodeInfo(String(msg.nodeName || ''), webviewView);
                    break;
                case 'refreshConnections':
                    await this._refreshAndSendConnections(webviewView);
                    break;
                case 'getTopicRoles':
                    await this._sendTopicRoles(String(msg.topicName || ''), webviewView);
                    break;
                case 'setOverviewToggles':
                    this._prefs.showNodes = msg.showNodes !== false;
                    this._prefs.showTopics = msg.showTopics !== false;
                    this._prefs.showServices = msg.showServices !== false;
                    this._prefs.showActions = msg.showActions !== false;
                    this._prefs.showParameters = msg.showParameters !== false;
                    await this._savePrefs();
                    break;
                case 'setTrackedTopics':
                    this._setTrackedTopics(msg.topicNames, webviewView);
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
            this._cancelScheduledRefresh();
            this._disposeAllTopicSubscriptions();
            this._view = undefined;
        });

        // The webview will send a 'refresh' message once its JS has loaded
        // and its message listener is ready.  That triggers the first fetch
        // and bootstraps the auto-refresh cycle via _sendGraphData's finally block.
    }

    /** Externally triggered refresh (e.g. from command palette). */
    refreshGraph(): void {
        this._view?.show(true);
        this._cancelScheduledRefresh();
        this._sendGraphData({ showLoading: true });
    }

    // ── Private ────────────────────────────────────────────────
    private _syncAutoRefresh() {
        if (this._view?.visible && this._autoRefreshEnabled) {
            // Auto-refresh should be active.  If nothing is pending or
            // in-flight, kick off an immediate refresh.  The finally block
            // of _sendGraphData will schedule the next one.
            if (!this._autoRefreshTimer && !this._refreshInFlight) {
                void this._sendGraphData();
            }
        } else {
            this._cancelScheduledRefresh();
        }
    }

    /**
     * Schedule the next auto-refresh after the configured interval.
     * Uses setTimeout (not setInterval) so the next tick only fires
     * after the previous refresh has fully completed, preventing
     * overlapping ROS CLI calls and runaway queue build-up.
     */
    private _scheduleNextAutoRefresh() {
        this._cancelScheduledRefresh();
        if (this._view?.visible && this._autoRefreshEnabled) {
            const refreshIntervalMs = this._getAutoRefreshIntervalMs();
            this._autoRefreshTimer = setTimeout(() => {
                this._autoRefreshTimer = undefined;
                void this._sendGraphData({ scope: this._lastScope });
            }, refreshIntervalMs);
        }
    }

    private _cancelScheduledRefresh() {
        if (this._autoRefreshTimer) {
            clearTimeout(this._autoRefreshTimer);
            this._autoRefreshTimer = undefined;
        }
    }

    private async _sendGraphData(options?: { showLoading?: boolean; scope?: RefreshScope }) {
        const view = this._view;
        if (!view) {
            return;
        }

        if (this._refreshInFlight) {
            this._refreshQueued = true;
            this._queuedShowLoading = this._queuedShowLoading || !!options?.showLoading;
            if (options?.showLoading) {
                view.webview.postMessage({ command: 'loading' });
            }
            return;
        }

        const scope: RefreshScope = options?.scope
            ?? { nodes: true, topics: true, services: true, actions: true, parameters: true };

        this._refreshInFlight = true;
        try {
            if (options?.showLoading) {
                if (this._cachedGraphPayload) {
                    view.webview.postMessage(this._cachedGraphPayload);
                }
                view.webview.postMessage({ command: 'loading' });
            }

            // Fetch only the list commands that are in scope.
            // Node info (connections) is never fetched here — it is
            // loaded lazily when the user clicks a node for details.
            const [nodes, topics, services, actions, parameters] = await Promise.all([
                scope.nodes
                    ? this._ros.getNodeList()
                    : Promise.resolve(this._cachedNodes),
                scope.topics
                    ? this._ros.getTopicList()
                    : Promise.resolve(this._cachedTopics),
                scope.services
                    ? this._ros.getServiceList()
                    : Promise.resolve(this._cachedServices),
                scope.actions
                    ? this._ros.getActionList()
                    : Promise.resolve(this._cachedActions),
                scope.parameters
                    ? this._ros.getParameterList()
                    : Promise.resolve(this._cachedParameters),
            ]);

            // Update per-category caches.
            this._cachedNodes = nodes as string[];
            this._cachedTopics = topics as Array<{ name: string; type: string }>;
            this._cachedServices = services as Array<{ name: string; type: string }>;
            this._cachedActions = actions as Array<{ name: string; type: string }>;
            this._cachedParameters = parameters as Array<{ name: string; node?: string }>;

            const topicNames = this._cachedTopics.map((topic) => topic.name);
            this._pruneTopicMessageCache(topicNames);
            this._syncTrackedTopicSubscriptions();

            // Prune cached connections for nodes that disappeared.
            if (scope.nodes) {
                const currentNodeSet = new Set(this._cachedNodes);
                for (const key of Object.keys(this._cachedConnections)) {
                    if (!currentNodeSet.has(key)) {
                        delete this._cachedConnections[key];
                    }
                }
            }

            if (this._view === view) {
                const graphPayload = {
                    command: 'graphData',
                    nodes: this._cachedNodes,
                    topics: this._cachedTopics,
                    services: this._cachedServices,
                    actions: this._cachedActions,
                    parameters: this._cachedParameters,
                    connections: { ...this._cachedConnections },
                    topicLatestMessages: this._buildTopicMessageSnapshot(topicNames),
                    topicLatestMessageTimes: this._buildTopicMessageTimeSnapshot(topicNames),
                    rosVersion: this._ros.isRos2() ? 2 : 1,
                    prefs: this._prefs,
                };
                this._cachedGraphPayload = graphPayload;
                view.webview.postMessage(graphPayload);
            }
        } catch (err) {
            // Keep the panel responsive even if one ROS CLI call fails
            // unexpectedly; the next refresh can recover.
            console.error('[NodeVisualizer] Failed to refresh graph data', err);

            if (this._view === view) {
                view.webview.postMessage({
                    command: 'graphData',
                    nodes: [],
                    topics: [],
                    services: [],
                    actions: [],
                    parameters: [],
                    connections: {},
                    topicLatestMessages: {},
                    topicLatestMessageTimes: {},
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
            } else {
                // Only schedule the next tick when there is nothing left
                // in the queue – this keeps the refresh cycle sequential.
                this._scheduleNextAutoRefresh();
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
        for (const topicName of Object.keys(this._topicLatestMessageTimes)) {
            if (!validTopics.has(topicName)) {
                delete this._topicLatestMessageTimes[topicName];
            }
        }
        for (const topicName of Array.from(this._trackedTopicNames)) {
            if (!validTopics.has(topicName)) {
                this._trackedTopicNames.delete(topicName);
            }
        }
        for (const topicName of Array.from(this._topicSubscriptions.keys())) {
            if (!validTopics.has(topicName)) {
                this._stopTopicSubscription(topicName);
            }
        }
    }

    private _isValidTrackedTopicName(topicName: string): boolean {
        return /^\/[A-Za-z0-9_./~-]+$/.test(topicName);
    }

    /**
     * Fetch `ros2 node info` for a single node on-demand (when the user
     * clicks a node in the detail card).  The result is cached so
     * subsequent clicks are instant and reverse-lookups for
     * topics/services/actions improve over time.
     */
    private async _fetchAndSendNodeInfo(nodeName: string, view: vscode.WebviewView) {
        if (!nodeName || this._view !== view) {
            return;
        }

        try {
            const info = await this._ros.getNodeGraphInfo(nodeName);
            this._cachedConnections[nodeName] = info;

            if (this._view === view) {
                view.webview.postMessage({
                    command: 'nodeInfo',
                    nodeName,
                    info,
                });
            }
        } catch {
            // Silently ignore — the detail card will show cached
            // (possibly empty) data; next click can retry.
        }
    }

    private async _refreshAndSendConnections(view: vscode.WebviewView) {
        if (this._view !== view) {
            return;
        }

        if (this._connectionsRefreshInFlight) {
            this._queuedConnectionsRefresh = true;
            return;
        }

        this._connectionsRefreshInFlight = true;
        try {
            do {
                this._queuedConnectionsRefresh = false;

                const nodes = this._cachedNodes.length
                    ? this._cachedNodes.slice()
                    : await this._ros.getNodeList();
                this._cachedNodes = nodes;

                const refreshedConnections: Record<string, RosNodeGraphInfo> = {};
                let cursor = 0;
                const worker = async () => {
                    while (cursor < nodes.length) {
                        const nodeName = nodes[cursor++];
                        refreshedConnections[nodeName] = await this._ros.getNodeGraphInfo(nodeName);
                    }
                };

                const workerCount = Math.min(4, nodes.length);
                await Promise.all(Array.from({ length: workerCount }, () => worker()));

                if (this._view !== view) {
                    return;
                }

                this._cachedConnections = refreshedConnections;
                view.webview.postMessage({
                    command: 'connectionData',
                    connections: { ...this._cachedConnections },
                });
            } while (this._queuedConnectionsRefresh && this._view === view);
        } finally {
            this._connectionsRefreshInFlight = false;
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

    private _buildTopicMessageTimeSnapshot(topicNames: string[]): Record<string, number> {
        const snapshot: Record<string, number> = {};
        for (const topicName of topicNames) {
            const cached = this._topicLatestMessageTimes[topicName];
            if (cached) {
                snapshot[topicName] = cached;
            }
        }
        return snapshot;
    }

    private _setTrackedTopics(topicNamesRaw: unknown, view: vscode.WebviewView) {
        if (this._view !== view) {
            return;
        }

        const topicNames = Array.isArray(topicNamesRaw)
            ? Array.from(
                new Set(
                    topicNamesRaw
                        .map((value) => String(value || '').trim())
                        .filter((name) => this._isValidTrackedTopicName(name)),
                ),
            )
            : [];

        this._trackedTopicNames = new Set(topicNames);
        this._syncTrackedTopicSubscriptions();
        this._sendTrackedTopicSnapshot(topicNames, view);
    }

    private _sendTrackedTopicSnapshot(topicNames: string[], view: vscode.WebviewView) {
        for (const topicName of topicNames) {
            const message = this._topicLatestMessages[topicName];
            if (!message) {
                continue;
            }

            view.webview.postMessage({
                command: 'topicLatestMessage',
                topicName,
                message,
                receivedAt: this._topicLatestMessageTimes[topicName] ?? Date.now(),
            });
        }
    }

    private _syncTrackedTopicSubscriptions() {
        const nextTracked = Array.from(this._trackedTopicNames);
        const nextTrackedSet = new Set(nextTracked);

        for (const topicName of Array.from(this._topicSubscriptions.keys())) {
            if (!nextTrackedSet.has(topicName)) {
                this._stopTopicSubscription(topicName);
            }
        }

        for (const topicName of nextTracked) {
            if (!this._topicSubscriptions.has(topicName)) {
                this._startTopicSubscription(topicName);
            }
        }
    }

    private _startTopicSubscription(topicName: string) {
        if (this._topicSubscriptions.has(topicName)) {
            return;
        }

        const token = ++this._topicSubscriptionSeq;
        this._topicSubscriptionTokens.set(topicName, token);

        const subscription = this._ros.subscribeToTopicMessages(
            topicName,
            (message) => this._handleTrackedTopicMessage(topicName, message),
            () => this._handleTopicSubscriptionClosed(topicName, token),
        );
        if (!subscription) {
            this._topicSubscriptionTokens.delete(topicName);
            return;
        }

        this._topicSubscriptions.set(topicName, subscription);
    }

    private _handleTrackedTopicMessage(topicName: string, rawMessage: string) {
        const normalized = String(rawMessage || '').trim();
        if (!normalized) {
            return;
        }

        const receivedAt = Date.now();
        this._topicLatestMessages[topicName] = normalized;
        this._topicLatestMessageTimes[topicName] = receivedAt;
        if (this._cachedGraphPayload) {
            const payload = this._cachedGraphPayload as {
                topicLatestMessages?: Record<string, string>;
                topicLatestMessageTimes?: Record<string, number>;
            };
            payload.topicLatestMessages = payload.topicLatestMessages ?? {};
            payload.topicLatestMessageTimes = payload.topicLatestMessageTimes ?? {};
            payload.topicLatestMessages[topicName] = normalized;
            payload.topicLatestMessageTimes[topicName] = receivedAt;
        }

        if (!this._view) {
            return;
        }

        this._view.webview.postMessage({
            command: 'topicLatestMessage',
            topicName,
            message: normalized,
            receivedAt,
        });
    }

    private _handleTopicSubscriptionClosed(topicName: string, token: number) {
        const activeToken = this._topicSubscriptionTokens.get(topicName);
        if (activeToken !== token) {
            return;
        }

        this._topicSubscriptions.delete(topicName);

        if (!this._trackedTopicNames.has(topicName) || !this._view) {
            this._topicSubscriptionTokens.delete(topicName);
            return;
        }

        if (this._topicSubscriptionRestartTimers.has(topicName)) {
            return;
        }

        const restartTimer = setTimeout(() => {
            this._topicSubscriptionRestartTimers.delete(topicName);
            if (!this._trackedTopicNames.has(topicName) || !this._view) {
                this._topicSubscriptionTokens.delete(topicName);
                return;
            }

            this._startTopicSubscription(topicName);
        }, TOPIC_ECHO_RESTART_DELAY_MS);
        restartTimer.unref();
        this._topicSubscriptionRestartTimers.set(topicName, restartTimer);
    }

    private _stopTopicSubscription(topicName: string) {
        const restartTimer = this._topicSubscriptionRestartTimers.get(topicName);
        if (restartTimer) {
            clearTimeout(restartTimer);
            this._topicSubscriptionRestartTimers.delete(topicName);
        }

        this._topicSubscriptionTokens.delete(topicName);
        const active = this._topicSubscriptions.get(topicName);
        if (!active) {
            return;
        }

        this._topicSubscriptions.delete(topicName);
        active.dispose();
    }

    private _disposeAllTopicSubscriptions() {
        for (const timer of this._topicSubscriptionRestartTimers.values()) {
            clearTimeout(timer);
        }
        this._topicSubscriptionRestartTimers.clear();
        this._topicSubscriptionTokens.clear();
        this._trackedTopicNames.clear();

        for (const subscription of this._topicSubscriptions.values()) {
            subscription.dispose();
        }
        this._topicSubscriptions.clear();
    }

    private async _sendTopicRoles(topicName: string, view: vscode.WebviewView) {
        if (!topicName || this._view !== view) {
            return;
        }

        const roles = await this._ros.getTopicRoles(topicName);
        if (this._view !== view) {
            return;
        }

        view.webview.postMessage({
            command: 'topicRoles',
            topicName,
            publishers: roles.publishers || [],
            subscribers: roles.subscribers || [],
        });
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

    private _parseScopeMessage(msg: Record<string, unknown>): RefreshScope {
        return {
            nodes: msg.nodes === true,
            topics: msg.topics === true,
            services: msg.services === true,
            actions: msg.actions === true,
            parameters: msg.parameters === true,
        };
    }

    private _getAutoRefreshIntervalMs(): number {
        const raw = vscode.workspace
            .getConfiguration('rosDevToolkit')
            .get<number>(
                NODE_VISUALIZER_AUTO_REFRESH_INTERVAL_SETTING,
                DEFAULT_NODE_VISUALIZER_AUTO_REFRESH_INTERVAL_MS,
            );
        const numeric = Number(raw);
        if (!Number.isFinite(numeric)) {
            return DEFAULT_NODE_VISUALIZER_AUTO_REFRESH_INTERVAL_MS;
        }
        return Math.min(120000, Math.max(250, Math.floor(numeric)));
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
    <button class="secondary hidden" id="btnRefetchSelected" title="Refetch selected item details" disabled>
        ↻ Refetch Selected
    </button>
    <div class="nv-toolbar">
        <button class="secondary hidden" id="btnPublishTopic" title="Publish a debug message to a topic">
            ✉ Publish
        </button>
        <button id="btnRefresh" title="Refresh all graph data">↻ Refresh All</button>
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
