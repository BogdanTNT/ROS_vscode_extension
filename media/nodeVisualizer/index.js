/* Node Visualizer Webview Script */
(function () {
    const vscode = acquireVsCodeApi();

    const commands = Object.freeze({
        toHost: Object.freeze({
            REFRESH: 'refresh',
            SET_AUTO_REFRESH: 'setAutoRefresh',
            SET_OVERVIEW_TOGGLES: 'setOverviewToggles',
            VIEW_SCOPE: 'viewScope',
            FETCH_NODE_INFO: 'fetchNodeInfo',
            REFRESH_CONNECTIONS: 'refreshConnections',
            GET_TOPIC_ROLES: 'getTopicRoles',
            SET_TRACKED_TOPICS: 'setTrackedTopics',
            GET_TOPIC_PUBLISH_TEMPLATE: 'getTopicPublishTemplate',
            PUBLISH_TOPIC_MESSAGE: 'publishTopicMessage',
        }),
        toWebview: Object.freeze({
            LOADING: 'loading',
            GRAPH_DATA: 'graphData',
            CONNECTION_DATA: 'connectionData',
            NODE_INFO: 'nodeInfo',
            TOPIC_ROLES: 'topicRoles',
            TOPIC_LATEST_MESSAGE: 'topicLatestMessage',
            TOPIC_PUBLISH_TEMPLATE: 'topicPublishTemplate',
            TOPIC_PUBLISH_RESULT: 'topicPublishResult',
        }),
    });

    const viewKinds = Object.freeze({
        OVERVIEW: 'overview',
        NODES: 'nodes',
        TOPICS: 'topics',
        SERVICES: 'services',
        ACTIONS: 'actions',
        PARAMETERS: 'parameters',
    });

    const kindLabels = Object.freeze({
        [viewKinds.NODES]: 'Nodes',
        [viewKinds.TOPICS]: 'Topics',
        [viewKinds.SERVICES]: 'Services',
        [viewKinds.ACTIONS]: 'Actions',
        [viewKinds.PARAMETERS]: 'Parameters',
    });
    const TOPIC_ROLES_REFRESH_MIN_INTERVAL_MS = 2000;
    const TOPIC_MESSAGE_STALE_AFTER_MS = 15000;

    const dom = {
        btnRefresh: document.getElementById('btnRefresh'),
        btnPublishTopic: document.getElementById('btnPublishTopic'),
        btnRefetchSelected: document.getElementById('btnRefetchSelected'),
        toggleAutoRefresh: document.getElementById('toggleAutoRefresh'),
        status: document.getElementById('status'),
        tabs: Array.from(document.querySelectorAll('.nv-tab')),
        tabNodesCount: document.getElementById('tabNodesCount'),
        tabTopicsCount: document.getElementById('tabTopicsCount'),
        tabServicesCount: document.getElementById('tabServicesCount'),
        tabActionsCount: document.getElementById('tabActionsCount'),
        tabParametersCount: document.getElementById('tabParametersCount'),
        overviewToggleBar: document.getElementById('overviewToggleBar'),
        overviewView: document.getElementById('overviewView'),
        listView: document.getElementById('listView'),
        overviewSections: document.getElementById('overviewSections'),
        detailsCard: document.getElementById('detailsCard'),
        filterInput: document.getElementById('entityFilter'),
        entityCount: document.getElementById('entityCount'),
        entityList: document.getElementById('entityList'),
        pinnedTopicList: document.getElementById('pinnedTopicList'),
        btnToggleAllPinnedPause: document.getElementById('btnToggleAllPinnedPause'),
        toggleNodes: document.getElementById('toggleNodes'),
        toggleTopics: document.getElementById('toggleTopics'),
        toggleServices: document.getElementById('toggleServices'),
        toggleActions: document.getElementById('toggleActions'),
        toggleParameters: document.getElementById('toggleParameters'),
        publishTopicModal: document.getElementById('publishTopicModal'),
        publishTopicBackdrop: document.getElementById('publishTopicBackdrop'),
        btnClosePublishTopic: document.getElementById('btnClosePublishTopic'),
        btnCancelPublishTopic: document.getElementById('btnCancelPublishTopic'),
        btnConfirmPublishTopic: document.getElementById('btnConfirmPublishTopic'),
        publishTopicSelect: document.getElementById('publishTopicSelect'),
        publishTopicType: document.getElementById('publishTopicType'),
        publishTopicPayload: document.getElementById('publishTopicPayload'),
        publishTopicStatus: document.getElementById('publishTopicStatus'),
    };

    const emptyNodeInfo = Object.freeze({
        publishers: [],
        subscribers: [],
        serviceServers: [],
        serviceClients: [],
        actionServers: [],
        actionClients: [],
    });

    const state = {
        activeView: viewKinds.OVERVIEW,
        filter: '',
        selectedKey: '',
        viewLoadingByKind: {
            [viewKinds.OVERVIEW]: false,
            [viewKinds.NODES]: false,
            [viewKinds.TOPICS]: false,
            [viewKinds.SERVICES]: false,
            [viewKinds.ACTIONS]: false,
            [viewKinds.PARAMETERS]: false,
        },
        topicMessageCache: {},
        topicMessageReceivedAt: {},
        lastTrackedTopicSignature: '',
        nodeInfoLoadingByName: {},
        topicRolesByName: {},
        topicRolesLoadingByName: {},
        topicRolesLastFetchedAt: {},
        selectedRefetchPendingByKind: {
            [viewKinds.NODES]: false,
            [viewKinds.TOPICS]: false,
            [viewKinds.SERVICES]: false,
            [viewKinds.ACTIONS]: false,
            [viewKinds.PARAMETERS]: false,
        },
        pinnedTopics: [],
        frozenPinnedTopics: [],
        frozenTopicMessages: {},
        allPinnedTopicsPaused: false,
        allPinnedTopicMessages: {},
        publishTemplateByTopic: {},
        graphData: {
            rosVersion: 2,
            nodes: [],
            topics: [],
            services: [],
            actions: [],
            parameters: [],
            connections: {},
        },
    };

    const persistedState = vscode.getState();
    if (Array.isArray(persistedState?.pinnedTopics)) {
        state.pinnedTopics = Array.from(
            new Set(
                persistedState.pinnedTopics
                    .map((topicName) => String(topicName || '').trim())
                    .filter(Boolean),
            ),
        );
    }
    if (Array.isArray(persistedState?.frozenPinnedTopics)) {
        state.frozenPinnedTopics = Array.from(
            new Set(
                persistedState.frozenPinnedTopics
                    .map((topicName) => String(topicName || '').trim())
                    .filter(Boolean),
            ),
        );
    }
    if (persistedState?.allPinnedTopicsPaused === true) {
        state.allPinnedTopicsPaused = true;
    }

    const escapeHtml = (value) => String(value ?? '').replace(/[&<>"']/g, (ch) => {
        if (ch === '&') {
            return '&amp;';
        }
        if (ch === '<') {
            return '&lt;';
        }
        if (ch === '>') {
            return '&gt;';
        }
        if (ch === '"') {
            return '&quot;';
        }
        return '&#39;';
    });

    const renderNameList = (values) => {
        if (!values || values.length === 0) {
            return '—';
        }
        return values.map(escapeHtml).join(', ');
    };

    const renderDetailLine = (label, values) =>
        '<div><span class="text-muted">' + escapeHtml(label) + ':</span> ' + renderNameList(values) + '</div>';

    dom.btnRefresh.addEventListener('click', () => {
        requestGraphRefresh({
            showGlobalLoading: true,
            showLocalLoading: true,
            localView: state.activeView,
        });
    });

    dom.btnPublishTopic.addEventListener('click', () => {
        openPublishTopicModal();
    });
    dom.btnRefetchSelected?.addEventListener('click', () => {
        refetchSelectedDetails();
    });

    dom.toggleAutoRefresh.addEventListener('change', () => {
        vscode.postMessage({
            command: commands.toHost.SET_AUTO_REFRESH,
            enabled: dom.toggleAutoRefresh.checked,
        });
        sendViewScope();
    });

    dom.btnClosePublishTopic.addEventListener('click', () => {
        closePublishTopicModal();
    });
    dom.btnCancelPublishTopic.addEventListener('click', () => {
        closePublishTopicModal();
    });
    dom.publishTopicBackdrop.addEventListener('click', () => {
        closePublishTopicModal();
    });
    dom.publishTopicSelect.addEventListener('change', () => {
        requestTopicPublishTemplate(dom.publishTopicSelect.value);
    });
    dom.btnConfirmPublishTopic.addEventListener('click', () => {
        publishTopicMessage();
    });
    document.addEventListener('keydown', (event) => {
        if (event.key === 'Escape' && !dom.publishTopicModal.classList.contains('hidden')) {
            closePublishTopicModal();
        }
    });
    dom.detailsCard?.addEventListener('click', (event) => {
        const target = event.target;
        if (!(target instanceof Element)) {
            return;
        }

        const pinBtn = target.closest('.nv-topic-pin-btn');
        if (!pinBtn) {
            return;
        }
        togglePinnedTopic(pinBtn.getAttribute('data-topic-name') || '');
    });
    dom.pinnedTopicList?.addEventListener('click', (event) => {
        const target = event.target;
        if (!(target instanceof Element)) {
            return;
        }

        const pinBtn = target.closest('.nv-topic-pin-btn');
        if (pinBtn) {
            togglePinnedTopic(pinBtn.getAttribute('data-topic-name') || '');
            return;
        }

        const freezeBtn = target.closest('.nv-topic-freeze-btn');
        if (freezeBtn) {
            togglePinnedTopicFreeze(freezeBtn.getAttribute('data-topic-name') || '');
        }
    });
    dom.btnToggleAllPinnedPause?.addEventListener('click', () => {
        toggleAllPinnedTopicsPause();
    });

    dom.filterInput.addEventListener('input', () => {
        state.filter = dom.filterInput.value.trim().toLowerCase();
        if (state.activeView !== viewKinds.OVERVIEW) {
            renderListView();
        }
    });

    [dom.toggleNodes, dom.toggleTopics, dom.toggleServices, dom.toggleActions, dom.toggleParameters].forEach((toggle) => {
        toggle.addEventListener('change', () => {
            persistOverviewToggleState();
            sendViewScope();
            if (state.activeView === viewKinds.OVERVIEW) {
                renderOverview();
            }
        });
    });

    dom.tabs.forEach((btn) => {
        btn.addEventListener('click', () => {
            const view = btn.dataset.view;
            if (!view) {
                return;
            }
            switchView(view);
        });
    });

    // Host -> webview bridge. Data updates are centralized here so every
    // render path consumes the same state shape.
    window.addEventListener('message', (event) => {
        const msg = event.data;
        if (msg.command === commands.toWebview.LOADING) {
            dom.status.innerHTML = '<span class="spinner"></span> Fetching runtime data...';
            return;
        }
        if (msg.command === commands.toWebview.GRAPH_DATA) {
            state.graphData = {
                rosVersion: msg.rosVersion || 2,
                nodes: msg.nodes || [],
                topics: msg.topics || [],
                services: msg.services || [],
                actions: msg.actions || [],
                parameters: msg.parameters || [],
                connections: msg.connections || {},
            };
            Object.keys(state.viewLoadingByKind).forEach((view) => {
                state.viewLoadingByKind[view] = false;
            });
            applyPreferences(msg.prefs || {});
            mergeTopicMessageCache(msg.topicLatestMessages || {}, msg.topicLatestMessageTimes || {});
            pruneTopicMessageCacheByGraph();
            pruneTopicRoleCacheByGraph();
            prunePinnedTopicsByGraph();
            state.selectedRefetchPendingByKind[viewKinds.PARAMETERS] = false;
            state.publishTemplateByTopic = {};
            renderStatus();
            renderTabCounts();
            updateActionAvailability();
            renderPublishButtonState();
            renderPinnedTopics();
            syncTrackedTopicSubscriptions({ force: true });
            renderCurrentView();
            return;
        }

        if (msg.command === commands.toWebview.CONNECTION_DATA) {
            state.graphData.connections = msg.connections || {};
            state.selectedRefetchPendingByKind[viewKinds.SERVICES] = false;
            state.selectedRefetchPendingByKind[viewKinds.ACTIONS] = false;
            renderCurrentView();
            return;
        }

        if (msg.command === commands.toWebview.NODE_INFO) {
            const nodeName = String(msg.nodeName || '');
            if (nodeName && msg.info) {
                state.graphData.connections[nodeName] = msg.info;
                state.nodeInfoLoadingByName[nodeName] = false;
                if (state.selectedKey === (viewKinds.NODES + ':' + nodeName)) {
                    state.selectedRefetchPendingByKind[viewKinds.NODES] = false;
                }
                // Re-render the detail card if this node (or a
                // topic/service that depends on it) is selected.
                renderCurrentView();
                if (state.selectedKey === (viewKinds.NODES + ':' + nodeName)) {
                    showNodeDetails(nodeName, dom.detailsCard);
                } else if (state.selectedKey.startsWith(viewKinds.TOPICS + ':') ||
                           state.selectedKey.startsWith(viewKinds.SERVICES + ':') ||
                           state.selectedKey.startsWith(viewKinds.ACTIONS + ':')) {
                    // Reverse-lookup data may have changed — refresh detail card.
                    const selectedRow = findSelectedRow();
                    if (selectedRow) {
                        selectedRow.showDetails(dom.detailsCard);
                    }
                }
            }
            return;
        }

        if (msg.command === commands.toWebview.TOPIC_LATEST_MESSAGE) {
            const topicName = String(msg.topicName || '');
            if (!topicName) {
                return;
            }
            const message = String(msg.message || '').trim();
            if (!message) {
                return;
            }
            state.topicMessageCache[topicName] = message;
            const receivedAt = Number(msg.receivedAt || Date.now());
            if (Number.isFinite(receivedAt) && receivedAt > 0) {
                state.topicMessageReceivedAt[topicName] = receivedAt;
            } else {
                state.topicMessageReceivedAt[topicName] = Date.now();
            }
            renderPinnedTopics();

            if (state.selectedKey === (viewKinds.TOPICS + ':' + topicName)) {
                showTopicDetails(topicName, dom.detailsCard);
            }
            return;
        }

        if (msg.command === commands.toWebview.TOPIC_ROLES) {
            const topicName = String(msg.topicName || '').trim();
            if (!topicName) {
                return;
            }

            const normalizeRoleList = (values) => Array.from(new Set(
                (Array.isArray(values) ? values : [])
                    .map((value) => String(value || '').trim())
                    .filter(Boolean),
            ));

            state.topicRolesByName[topicName] = {
                publishers: normalizeRoleList(msg.publishers),
                subscribers: normalizeRoleList(msg.subscribers),
            };
            state.topicRolesLoadingByName[topicName] = false;
            state.topicRolesLastFetchedAt[topicName] = Date.now();
            if (state.selectedKey === (viewKinds.TOPICS + ':' + topicName)) {
                state.selectedRefetchPendingByKind[viewKinds.TOPICS] = false;
            }

            if (state.selectedKey === (viewKinds.TOPICS + ':' + topicName)) {
                showTopicDetails(topicName, dom.detailsCard);
            }
            return;
        }

        if (msg.command === commands.toWebview.TOPIC_PUBLISH_TEMPLATE) {
            handleTopicPublishTemplateMessage(msg);
            return;
        }

        if (msg.command === commands.toWebview.TOPIC_PUBLISH_RESULT) {
            handleTopicPublishResultMessage(msg);
        }
    });

    function renderStatus() {
        const data = state.graphData;
        const now = new Date();
        const stamp = now.toLocaleTimeString();
        let text = 'Last updated: ' + stamp;

        if (data.rosVersion !== 2) {
            text += ' · ROS 1 mode (actions unavailable)';
        }

        dom.status.textContent = text;
    }

    function renderTabCounts() {
        const data = state.graphData;
        dom.tabNodesCount.textContent = String(data.nodes.length);
        dom.tabTopicsCount.textContent = String(data.topics.length);
        dom.tabServicesCount.textContent = String(data.services.length);
        dom.tabActionsCount.textContent = data.rosVersion === 2 ? String(data.actions.length) : '0';
        dom.tabParametersCount.textContent = String(data.parameters.length);
    }

    function updateActionAvailability() {
        const actionsTab = dom.tabs.find((tab) => tab.dataset.view === viewKinds.ACTIONS);
        if (!actionsTab) {
            return;
        }

        const ros2 = state.graphData.rosVersion === 2;
        actionsTab.disabled = !ros2;
        actionsTab.title = ros2 ? '' : 'Actions are available in ROS 2 only';
        dom.toggleActions.disabled = !ros2;

        if (!ros2) {
            dom.toggleActions.checked = false;
        }

        if (!ros2 && state.activeView === viewKinds.ACTIONS) {
            switchView(viewKinds.OVERVIEW);
        }
    }

    function applyPreferences(prefs) {
        dom.toggleAutoRefresh.checked = prefs.autoRefreshEnabled === true;
        dom.toggleNodes.checked = prefs.showNodes !== false;
        dom.toggleTopics.checked = prefs.showTopics !== false;
        dom.toggleServices.checked = prefs.showServices !== false;
        dom.toggleActions.checked = prefs.showActions !== false;
        dom.toggleParameters.checked = prefs.showParameters !== false;
    }

    function persistOverviewToggleState() {
        vscode.postMessage({
            command: commands.toHost.SET_OVERVIEW_TOGGLES,
            showNodes: dom.toggleNodes.checked,
            showTopics: dom.toggleTopics.checked,
            showServices: dom.toggleServices.checked,
            showActions: dom.toggleActions.checked,
            showParameters: dom.toggleParameters.checked,
        });
    }

    function mergeTopicMessageCache(rawCache, rawTimes) {
        if (!rawCache || typeof rawCache !== 'object') {
            return;
        }
        for (const [topicName, rawMessage] of Object.entries(rawCache)) {
            const normalizedTopicName = String(topicName || '').trim();
            const normalizedMessage = String(rawMessage || '').trim();
            if (!normalizedTopicName || !normalizedMessage) {
                continue;
            }
            state.topicMessageCache[normalizedTopicName] = normalizedMessage;

            const receivedAt = Number(rawTimes?.[normalizedTopicName] || 0);
            if (Number.isFinite(receivedAt) && receivedAt > 0) {
                state.topicMessageReceivedAt[normalizedTopicName] = receivedAt;
            }
        }
    }

    function pruneTopicMessageCacheByGraph() {
        const topicNames = new Set((state.graphData.topics || []).map((topic) => topic.name));
        for (const topicName of Object.keys(state.topicMessageCache)) {
            if (!topicNames.has(topicName)) {
                delete state.topicMessageCache[topicName];
            }
        }
        for (const topicName of Object.keys(state.topicMessageReceivedAt)) {
            if (!topicNames.has(topicName)) {
                delete state.topicMessageReceivedAt[topicName];
            }
        }
    }

    function pruneTopicRoleCacheByGraph() {
        const topicNames = new Set((state.graphData.topics || []).map((topic) => topic.name));
        for (const topicName of Object.keys(state.topicRolesByName)) {
            if (!topicNames.has(topicName)) {
                delete state.topicRolesByName[topicName];
            }
        }
        for (const topicName of Object.keys(state.topicRolesLoadingByName)) {
            if (!topicNames.has(topicName)) {
                delete state.topicRolesLoadingByName[topicName];
            }
        }
        for (const topicName of Object.keys(state.topicRolesLastFetchedAt)) {
            if (!topicNames.has(topicName)) {
                delete state.topicRolesLastFetchedAt[topicName];
            }
        }
    }

    function getTopicCachedMessage(topicName) {
        return String(state.topicMessageCache[topicName] || '').trim();
    }

    function normalizeTopicName(topicName) {
        return String(topicName || '').trim();
    }

    function requestTopicRoles(topicName, options = {}) {
        const normalizedTopicName = normalizeTopicName(topicName);
        if (!normalizedTopicName) {
            return;
        }
        if (state.topicRolesLoadingByName[normalizedTopicName]) {
            return;
        }

        const now = Date.now();
        const minInterval = options.force ? 0 : TOPIC_ROLES_REFRESH_MIN_INTERVAL_MS;
        const lastFetchedAt = Number(state.topicRolesLastFetchedAt[normalizedTopicName] || 0);
        if (now - lastFetchedAt < minInterval) {
            return;
        }

        state.topicRolesLoadingByName[normalizedTopicName] = true;
        if (state.selectedKey === (viewKinds.TOPICS + ':' + normalizedTopicName)) {
            showTopicDetails(normalizedTopicName, dom.detailsCard);
        }
        vscode.postMessage({
            command: commands.toHost.GET_TOPIC_ROLES,
            topicName: normalizedTopicName,
        });
    }

    function requestNodeInfo(nodeName, options = {}) {
        const normalizedNodeName = String(nodeName || '').trim();
        if (!normalizedNodeName) {
            return;
        }
        if (state.nodeInfoLoadingByName[normalizedNodeName] && options.force !== true) {
            return;
        }

        state.nodeInfoLoadingByName[normalizedNodeName] = true;
        vscode.postMessage({
            command: commands.toHost.FETCH_NODE_INFO,
            nodeName: normalizedNodeName,
        });
    }

    function getSelectedEntityRef() {
        if (!state.selectedKey) {
            return undefined;
        }
        const separator = state.selectedKey.indexOf(':');
        if (separator < 0) {
            return undefined;
        }

        const kind = state.selectedKey.slice(0, separator);
        const name = state.selectedKey.slice(separator + 1);
        if (!kind || !name) {
            return undefined;
        }
        return { kind, name };
    }

    function getRefetchLabelForKind(kind) {
        if (kind === viewKinds.NODES) {
            return 'Node';
        }
        if (kind === viewKinds.TOPICS) {
            return 'Topic';
        }
        if (kind === viewKinds.SERVICES) {
            return 'Service';
        }
        if (kind === viewKinds.ACTIONS) {
            return 'Action';
        }
        if (kind === viewKinds.PARAMETERS) {
            return 'Parameter';
        }
        return 'Selected';
    }

    function updateRefetchSelectedButtonState() {
        const button = dom.btnRefetchSelected;
        if (!button) {
            return;
        }

        const selected = getSelectedEntityRef();
        if (!selected) {
            button.classList.add('hidden');
            button.disabled = true;
            button.textContent = '↻ Refetch Selected';
            button.title = 'Select an item to refetch details.';
            button.setAttribute('aria-label', 'Select an item to refetch details.');
            return;
        }

        button.classList.remove('hidden');
        const label = getRefetchLabelForKind(selected.kind);
        const pending = state.selectedRefetchPendingByKind[selected.kind] === true;
        button.disabled = false;
        button.textContent = pending ? '↻ Refreshing ' + label + '...' : '↻ Refetch ' + label;
        button.title = pending
            ? 'Refreshing ' + label.toLowerCase() + ' details...'
            : 'Refetch ' + label.toLowerCase() + ' details';
        button.setAttribute(
            'aria-label',
            pending
                ? 'Refreshing ' + label.toLowerCase() + ' details'
                : 'Refetch ' + label.toLowerCase() + ' details',
        );
    }

    function refetchSelectedDetails() {
        const selected = getSelectedEntityRef();
        if (!selected) {
            return;
        }

        if (selected.kind === viewKinds.NODES) {
            state.selectedRefetchPendingByKind[viewKinds.NODES] = true;
            requestNodeInfo(selected.name, { force: true });
            showNodeDetails(selected.name, dom.detailsCard);
            updateRefetchSelectedButtonState();
            return;
        }

        if (selected.kind === viewKinds.TOPICS) {
            state.selectedRefetchPendingByKind[viewKinds.TOPICS] = true;
            requestTopicRoles(selected.name, { force: true });
            syncTrackedTopicSubscriptions();
            showTopicDetails(selected.name, dom.detailsCard);
            updateRefetchSelectedButtonState();
            return;
        }

        if (selected.kind === viewKinds.SERVICES || selected.kind === viewKinds.ACTIONS) {
            state.selectedRefetchPendingByKind[selected.kind] = true;
            vscode.postMessage({ command: commands.toHost.REFRESH_CONNECTIONS });
            renderCurrentView();
            return;
        }

        if (selected.kind === viewKinds.PARAMETERS) {
            state.selectedRefetchPendingByKind[viewKinds.PARAMETERS] = true;
            vscode.postMessage({ command: commands.toHost.REFRESH });
            renderCurrentView();
        }
    }

    function persistTopicMonitorState() {
        // Keep pin/freeze state in webview-local storage so it survives reloads.
        const persisted = vscode.getState() || {};
        vscode.setState({
            ...persisted,
            pinnedTopics: state.pinnedTopics.slice(),
            frozenPinnedTopics: state.frozenPinnedTopics.slice(),
            allPinnedTopicsPaused: state.allPinnedTopicsPaused,
        });
    }

    function prunePinnedTopicsByGraph() {
        // Pinned/frozen topics should only reference currently discovered graph topics.
        const topicNames = new Set((state.graphData.topics || []).map((topic) => topic.name));
        const nextPinnedTopics = state.pinnedTopics.filter((topicName) => topicNames.has(topicName));
        const nextFrozenTopics = state.frozenPinnedTopics.filter((topicName) => nextPinnedTopics.includes(topicName));
        let changed = false;
        if (nextPinnedTopics.length !== state.pinnedTopics.length) {
            changed = true;
        }
        if (nextFrozenTopics.length !== state.frozenPinnedTopics.length) {
            changed = true;
        }
        state.frozenPinnedTopics = nextFrozenTopics;
        state.pinnedTopics = nextPinnedTopics;
        for (const topicName of Object.keys(state.frozenTopicMessages)) {
            if (!state.frozenPinnedTopics.includes(topicName)) {
                delete state.frozenTopicMessages[topicName];
            }
        }
        for (const topicName of Object.keys(state.allPinnedTopicMessages)) {
            if (!state.pinnedTopics.includes(topicName)) {
                delete state.allPinnedTopicMessages[topicName];
            }
        }
        if (changed) {
            persistTopicMonitorState();
        }
    }

    function isTopicPinned(topicName) {
        return state.pinnedTopics.includes(normalizeTopicName(topicName));
    }

    function isPinnedTopicFrozen(topicName) {
        return state.frozenPinnedTopics.includes(normalizeTopicName(topicName));
    }

    function getPinnedTopicDisplayMessage(topicName) {
        const normalizedTopicName = normalizeTopicName(topicName);
        if (state.allPinnedTopicsPaused) {
            const pausedSnapshot = String(state.allPinnedTopicMessages[normalizedTopicName] || '').trim();
            if (pausedSnapshot) {
                return pausedSnapshot;
            }
        }
        if (!isPinnedTopicFrozen(normalizedTopicName)) {
            return getTopicCachedMessage(normalizedTopicName);
        }

        return String(state.frozenTopicMessages[normalizedTopicName] || '').trim();
    }

    function togglePinnedTopic(topicName) {
        const normalizedTopicName = normalizeTopicName(topicName);
        if (!normalizedTopicName) {
            return;
        }

        const wasPinned = isTopicPinned(normalizedTopicName);
        if (wasPinned) {
            state.pinnedTopics = state.pinnedTopics.filter((name) => name !== normalizedTopicName);
            state.frozenPinnedTopics = state.frozenPinnedTopics.filter((name) => name !== normalizedTopicName);
            delete state.frozenTopicMessages[normalizedTopicName];
            delete state.allPinnedTopicMessages[normalizedTopicName];
        } else if (getTopicByName(normalizedTopicName)) {
            state.pinnedTopics = [...state.pinnedTopics, normalizedTopicName];
            if (state.allPinnedTopicsPaused) {
                const snapshot = getTopicCachedMessage(normalizedTopicName);
                if (snapshot) {
                    state.allPinnedTopicMessages[normalizedTopicName] = snapshot;
                }
            }
        } else {
            return;
        }

        persistTopicMonitorState();
        renderPinnedTopics();
        syncTrackedTopicSubscriptions();

        if (!wasPinned && !state.allPinnedTopicsPaused) {
            // Topic stream subscription sync is enough for immediate updates.
            syncTrackedTopicSubscriptions();
        }

        renderCurrentView();

        if (state.selectedKey === (viewKinds.TOPICS + ':' + normalizedTopicName)) {
            showTopicDetails(normalizedTopicName, dom.detailsCard);
        }
    }

    function togglePinnedTopicFreeze(topicName) {
        const normalizedTopicName = normalizeTopicName(topicName);
        if (!normalizedTopicName || !isTopicPinned(normalizedTopicName)) {
            return;
        }

        if (isPinnedTopicFrozen(normalizedTopicName)) {
            state.frozenPinnedTopics = state.frozenPinnedTopics.filter((name) => name !== normalizedTopicName);
            delete state.frozenTopicMessages[normalizedTopicName];
            persistTopicMonitorState();
            syncTrackedTopicSubscriptions();
            renderPinnedTopics();
            return;
        }

        const snapshot = getTopicCachedMessage(normalizedTopicName);
        if (snapshot) {
            state.frozenTopicMessages[normalizedTopicName] = snapshot;
        } else {
            delete state.frozenTopicMessages[normalizedTopicName];
        }
        state.frozenPinnedTopics = [...state.frozenPinnedTopics, normalizedTopicName];
        persistTopicMonitorState();
        syncTrackedTopicSubscriptions();
        renderPinnedTopics();
    }

    function toggleAllPinnedTopicsPause() {
        const nextPaused = !state.allPinnedTopicsPaused;
        state.allPinnedTopicsPaused = nextPaused;

        if (nextPaused) {
            state.allPinnedTopicMessages = {};
            // Snapshot currently visible values so every pinned card freezes
            // with stable content while global pause is enabled.
            state.pinnedTopics.forEach((topicName) => {
                const cached = getTopicCachedMessage(topicName);
                if (cached) {
                    state.allPinnedTopicMessages[topicName] = cached;
                }
            });
        } else {
            state.allPinnedTopicMessages = {};
        }

        persistTopicMonitorState();
        syncTrackedTopicSubscriptions();
        renderPinnedTopics();
    }

    function isPinnedTopicEffectivelyPaused(topicName) {
        const normalizedTopicName = normalizeTopicName(topicName);
        if (!isTopicPinned(normalizedTopicName)) {
            return false;
        }
        return state.allPinnedTopicsPaused || isPinnedTopicFrozen(normalizedTopicName);
    }

    function renderPinnedTopicsHeaderControls() {
        const pauseAllBtn = dom.btnToggleAllPinnedPause;
        if (!pauseAllBtn) {
            return;
        }

        const paused = state.allPinnedTopicsPaused;
        pauseAllBtn.disabled = state.pinnedTopics.length === 0;
        pauseAllBtn.textContent = paused ? '▶' : '⏸';
        pauseAllBtn.classList.toggle('frozen', paused);
        const label = paused ? 'Resume all pinned topic reading' : 'Pause all pinned topic reading';
        pauseAllBtn.title = label;
        pauseAllBtn.setAttribute('aria-label', label);
    }

    function getTrackedTopicNames() {
        const trackedTopicNames = new Set();
        if (!state.allPinnedTopicsPaused) {
            state.pinnedTopics
                .filter((topicName) => !isPinnedTopicFrozen(topicName))
                .forEach((topicName) => trackedTopicNames.add(topicName));
        }

        const selectedTopicName = getSelectedTopicName();
        if (selectedTopicName) {
            if (!isPinnedTopicEffectivelyPaused(selectedTopicName)) {
                trackedTopicNames.add(selectedTopicName);
            }
        }
        return Array.from(trackedTopicNames);
    }

    function sendTrackedTopicsToHost(topicNames, options = {}) {
        const normalizedTopicNames = Array.from(new Set(
            (Array.isArray(topicNames) ? topicNames : [])
                .map((name) => normalizeTopicName(name))
                .filter(Boolean),
        ));
        const signature = normalizedTopicNames.slice().sort().join('|');
        if (!options.force && signature === state.lastTrackedTopicSignature) {
            return;
        }
        state.lastTrackedTopicSignature = signature;
        vscode.postMessage({
            command: commands.toHost.SET_TRACKED_TOPICS,
            topicNames: normalizedTopicNames,
        });
    }

    function syncTrackedTopicSubscriptions(options = {}) {
        sendTrackedTopicsToHost(getTrackedTopicNames(), options);
    }

    function getTopicMessageReceivedAt(topicName) {
        const timestamp = Number(state.topicMessageReceivedAt[topicName] || 0);
        if (!Number.isFinite(timestamp) || timestamp <= 0) {
            return 0;
        }
        return timestamp;
    }

    function formatTopicMessageAge(topicName) {
        const receivedAt = getTopicMessageReceivedAt(topicName);
        if (!receivedAt) {
            return '';
        }

        const ageMs = Math.max(0, Date.now() - receivedAt);
        const ageSeconds = Math.floor(ageMs / 1000);
        const stamp = new Date(receivedAt).toLocaleTimeString();
        const staleSuffix = ageMs > TOPIC_MESSAGE_STALE_AFTER_MS ? ' · stale' : '';
        return 'Updated ' + ageSeconds + 's ago (' + stamp + ')' + staleSuffix;
    }

    function renderPinnedTopics() {
        if (!dom.pinnedTopicList) {
            return;
        }
        renderPinnedTopicsHeaderControls();
        dom.pinnedTopicList.innerHTML = '';

        if (!state.pinnedTopics.length) {
            dom.pinnedTopicList.innerHTML = '<div class="text-muted">No pinned topics yet.</div>';
            return;
        }

        // Render a compact live-monitor card for each pinned topic.
        state.pinnedTopics.forEach((topicName) => {
            const topic = getTopicByName(topicName);
            if (!topic) {
                return;
            }

            const item = document.createElement('article');
            item.className = 'nv-pinned-item';
            const topicType = topic.type || 'unknown';
            const frozen = isPinnedTopicFrozen(topic.name);
            const freezeLabel = frozen ? 'Resume' : 'Pause';
            const freezeIcon = frozen ? '▶' : '⏸';
            const effectivelyPaused = isPinnedTopicEffectivelyPaused(topic.name);
            const displayedMessage = getPinnedTopicDisplayMessage(topic.name);
            const emptyMessage = state.allPinnedTopicsPaused
                ? 'Reading paused for all pinned topics.'
                : (frozen ? 'Reading paused.' : 'No cached message yet.');

            item.innerHTML =
                '<div class="nv-pinned-head">' +
                '<button class="pin-btn nv-topic-pin-btn nv-pinned-pin pinned" type="button" data-topic-name="' +
                escapeHtml(topic.name) +
                '" title="Unpin topic" aria-label="Unpin topic">' +
                '★' +
                '</button>' +
                '<span class="nv-pinned-topic-name" title="' + escapeHtml(topic.name) + '">' +
                escapeHtml(topic.name) +
                '</span>' +
                '<button class="pin-btn nv-topic-freeze-btn' + (frozen ? ' frozen' : '') + '" type="button" ' +
                'data-topic-name="' + escapeHtml(topic.name) + '" ' +
                'title="' + escapeHtml(freezeLabel + ' reading') + '" aria-label="' +
                escapeHtml(freezeLabel + ' reading') + '">' +
                freezeIcon +
                '</button>' +
                '<span class="nv-pinned-topic-type-inline" title="' + escapeHtml(topicType) + '">' +
                escapeHtml(topicType) +
                '</span>' +
                '</div>' +
                renderTopicMessageBlock(topic.name, {
                    compact: true,
                    messageOverride: displayedMessage,
                    emptyMessage,
                });

            if (effectivelyPaused) {
                item.classList.add('is-paused');
            }

            dom.pinnedTopicList.appendChild(item);
        });

        if (!dom.pinnedTopicList.children.length) {
            dom.pinnedTopicList.innerHTML = '<div class="text-muted">No pinned topics yet.</div>';
        }
    }

    function getSelectedTopicName() {
        const prefix = viewKinds.TOPICS + ':';
        if (!state.selectedKey.startsWith(prefix)) {
            return '';
        }
        return state.selectedKey.slice(prefix.length);
    }

    function getTopicByName(topicName) {
        return state.graphData.topics.find((topic) => topic.name === topicName);
    }

    function renderPublishButtonState() {
        const isTopicsView = state.activeView === viewKinds.TOPICS;
        dom.btnPublishTopic.classList.toggle('hidden', !isTopicsView);
        dom.btnPublishTopic.disabled = !isTopicsView || state.graphData.topics.length === 0;
    }

    function setPublishTopicStatus(text, options = {}) {
        const message = String(text || '').trim();
        dom.publishTopicStatus.className = 'mt text-sm';
        dom.publishTopicStatus.style.color = options.isError ? 'var(--danger)' : '';

        if (!message) {
            dom.publishTopicStatus.textContent = '';
            dom.publishTopicStatus.classList.add('hidden');
            return;
        }

        if (options.showSpinner) {
            dom.publishTopicStatus.innerHTML = '<span class="spinner"></span> ' + escapeHtml(message);
        } else {
            dom.publishTopicStatus.textContent = message;
        }
        dom.publishTopicStatus.classList.remove('hidden');
    }

    function populatePublishTopicOptions() {
        const topics = state.graphData.topics.slice().sort((a, b) => a.name.localeCompare(b.name));
        dom.publishTopicSelect.innerHTML = '';

        topics.forEach((topic) => {
            const option = document.createElement('option');
            option.value = topic.name;
            option.textContent = topic.name;
            dom.publishTopicSelect.appendChild(option);
        });
    }

    function openPublishTopicModal() {
        if (state.activeView !== viewKinds.TOPICS) {
            return;
        }

        populatePublishTopicOptions();
        if (!dom.publishTopicSelect.options.length) {
            return;
        }

        const selectedTopicName = getSelectedTopicName();
        const hasSelectedTopic = selectedTopicName
            && Array.from(dom.publishTopicSelect.options).some((option) => option.value === selectedTopicName);
        dom.publishTopicSelect.value = hasSelectedTopic
            ? selectedTopicName
            : dom.publishTopicSelect.options[0].value;

        dom.publishTopicPayload.value = '';
        dom.publishTopicType.value = '';
        dom.btnConfirmPublishTopic.disabled = true;
        setPublishTopicStatus('', {});
        dom.publishTopicModal.classList.remove('hidden');

        requestTopicPublishTemplate(dom.publishTopicSelect.value);
        dom.publishTopicPayload.focus();
    }

    function closePublishTopicModal() {
        dom.publishTopicModal.classList.add('hidden');
        dom.btnConfirmPublishTopic.disabled = false;
        setPublishTopicStatus('', {});
    }

    function requestTopicPublishTemplate(topicName) {
        const normalizedTopicName = String(topicName || '').trim();
        if (!normalizedTopicName) {
            return;
        }

        const topic = getTopicByName(normalizedTopicName);
        const topicTypeHint = topic?.type || '';
        dom.publishTopicType.value = topicTypeHint && topicTypeHint !== 'unknown' ? topicTypeHint : 'Resolving...';
        dom.publishTopicPayload.value = '';

        const cached = state.publishTemplateByTopic[normalizedTopicName];
        if (cached?.template) {
            dom.publishTopicType.value = cached.topicType || topicTypeHint || '';
            dom.publishTopicPayload.value = cached.template;
            dom.btnConfirmPublishTopic.disabled = false;
            setPublishTopicStatus('', {});
            return;
        }

        dom.btnConfirmPublishTopic.disabled = true;
        setPublishTopicStatus('Loading default payload...', { showSpinner: true });
        vscode.postMessage({
            command: commands.toHost.GET_TOPIC_PUBLISH_TEMPLATE,
            topicName: normalizedTopicName,
            topicTypeHint: topicTypeHint || '',
        });
    }

    function handleTopicPublishTemplateMessage(msg) {
        const topicName = String(msg.topicName || '');
        if (!topicName) {
            return;
        }

        if (msg.success) {
            const topicType = String(msg.topicType || '');
            const template = String(msg.template || '{}');
            state.publishTemplateByTopic[topicName] = {
                topicType,
                template,
            };

            if (!dom.publishTopicModal.classList.contains('hidden') && dom.publishTopicSelect.value === topicName) {
                dom.publishTopicType.value = topicType || '';
                dom.publishTopicPayload.value = template;
                dom.btnConfirmPublishTopic.disabled = false;
                setPublishTopicStatus('', {});
            }
            return;
        }

        if (!dom.publishTopicModal.classList.contains('hidden') && dom.publishTopicSelect.value === topicName) {
            dom.btnConfirmPublishTopic.disabled = false;
            dom.publishTopicType.value = String(msg.topicType || dom.publishTopicType.value || '');
            setPublishTopicStatus(String(msg.error || 'Failed to build topic payload template.'), { isError: true });
        }
    }

    function publishTopicMessage() {
        const topicName = String(dom.publishTopicSelect.value || '').trim();
        const payload = String(dom.publishTopicPayload.value || '').trim();
        const topicTypeHint = String(dom.publishTopicType.value || '').trim();

        if (!topicName) {
            setPublishTopicStatus('Select a topic first.', { isError: true });
            return;
        }
        if (!payload) {
            setPublishTopicStatus('Payload cannot be empty.', { isError: true });
            return;
        }

        dom.btnConfirmPublishTopic.disabled = true;
        setPublishTopicStatus('Publishing message...', { showSpinner: true });
        vscode.postMessage({
            command: commands.toHost.PUBLISH_TOPIC_MESSAGE,
            topicName,
            payload,
            topicTypeHint,
        });
    }

    function handleTopicPublishResultMessage(msg) {
        const topicName = String(msg.topicName || '');
        const success = msg.success === true;

        if (dom.publishTopicModal.classList.contains('hidden')) {
            return;
        }

        dom.btnConfirmPublishTopic.disabled = false;
        if (!success) {
            setPublishTopicStatus(String(msg.error || 'Failed to publish message.'), { isError: true });
            return;
        }

        setPublishTopicStatus('Message published on ' + topicName + '.', {});

        if (topicName) {
            delete state.topicMessageCache[topicName];
            delete state.topicMessageReceivedAt[topicName];
            if (state.selectedKey === (viewKinds.TOPICS + ':' + topicName)) {
                showTopicDetails(topicName, dom.detailsCard);
            }
        }

        // Trigger a fresh graph + topic-message poll so the latest
        // message appears quickly in details and pinned topic cards.
        requestGraphRefresh({
            showGlobalLoading: false,
            showLocalLoading: true,
            localView: state.activeView,
        });
    }

    function computeCurrentScope() {
        const view = state.activeView;
        if (view === viewKinds.OVERVIEW) {
            return {
                nodes: dom.toggleNodes.checked,
                topics: dom.toggleTopics.checked,
                services: dom.toggleServices.checked,
                actions: dom.toggleActions.checked && state.graphData.rosVersion === 2,
                parameters: dom.toggleParameters.checked,
            };
        }

        return {
            nodes: view === viewKinds.NODES,
            topics: view === viewKinds.TOPICS,
            services: view === viewKinds.SERVICES,
            actions: view === viewKinds.ACTIONS,
            parameters: view === viewKinds.PARAMETERS,
        };
    }

    /**
     * Tell the backend which graph categories are currently visible so
     * auto-refresh can skip unnecessary CLI calls.
     *
     * - Single-kind tab (nodes / topics / …) → only that category.
     * - Overview → whichever overview toggles are checked.
     */
    function sendViewScope() {
        const scope = computeCurrentScope();
        vscode.postMessage({
            command: commands.toHost.VIEW_SCOPE,
            ...scope,
        });
        return scope;
    }

    function setViewLoading(view, loading) {
        if (!Object.prototype.hasOwnProperty.call(state.viewLoadingByKind, view)) {
            return;
        }
        state.viewLoadingByKind[view] = Boolean(loading);
    }

    function isViewLoading(view) {
        return state.viewLoadingByKind[view] === true;
    }

    function requestGraphRefresh(options = {}) {
        const scope = options.scope || computeCurrentScope();
        const localView = options.localView || state.activeView;
        const showLocalLoading = options.showLocalLoading !== false;
        const showGlobalLoading = options.showGlobalLoading === true;

        if (showLocalLoading) {
            setViewLoading(localView, true);
            renderCurrentView();
        }

        vscode.postMessage({
            command: commands.toHost.REFRESH,
            showLoading: showGlobalLoading,
            ...scope,
        });
    }

    function switchView(view) {
        state.activeView = view;
        state.selectedKey = '';
        updateRefetchSelectedButtonState();
        dom.tabs.forEach((tab) => tab.classList.toggle('active', tab.dataset.view === view));
        dom.overviewToggleBar?.classList.toggle('hidden', view !== viewKinds.OVERVIEW);
        renderPublishButtonState();
        const scope = sendViewScope();

        if (view === viewKinds.OVERVIEW) {
            closePublishTopicModal();
            dom.overviewView.classList.remove('hidden');
            dom.listView.classList.add('hidden');
            requestGraphRefresh({
                scope,
                localView: viewKinds.OVERVIEW,
                showLocalLoading: true,
                showGlobalLoading: false,
            });
            return;
        }

        if (view !== viewKinds.TOPICS) {
            closePublishTopicModal();
        }

        dom.overviewView.classList.add('hidden');
        dom.listView.classList.remove('hidden');
        dom.filterInput.placeholder = 'Filter ' + view + '...';
        requestGraphRefresh({
            scope,
            localView: view,
            showLocalLoading: true,
            showGlobalLoading: false,
        });
    }

    function renderCurrentView() {
        if (state.activeView === viewKinds.OVERVIEW) {
            renderOverview();
        } else {
            renderListView();
        }
        updateRefetchSelectedButtonState();
    }

    function renderOverview() {
        const selectedKinds = getSelectedOverviewKinds();
        const allVisibleRows = [];

        dom.overviewSections.innerHTML = '';

        if (isViewLoading(viewKinds.OVERVIEW)) {
            dom.overviewSections.innerHTML =
                '<div class="text-muted"><span class="spinner"></span> Fetching overview data...</div>';
            setDetailsPlaceholder();
            syncTrackedTopicSubscriptions();
            return;
        }

        if (selectedKinds.length === 0) {
            dom.overviewSections.innerHTML = '<div class="text-muted">Select at least one list above.</div>';
            setDetailsPlaceholder();
            state.selectedKey = '';
            syncTrackedTopicSubscriptions();
            return;
        }

        selectedKinds.forEach((kind) => {
            const rows = getRowsForKind(kind);
            allVisibleRows.push(...rows);
            const sectionEl = renderOverviewSection(kind, rows);
            dom.overviewSections.appendChild(sectionEl);
        });

        const selected = allVisibleRows.find((row) => row.key === state.selectedKey);
        if (selected) {
            selected.showDetails(dom.detailsCard);
            syncTrackedTopicSubscriptions();
            return;
        }

        state.selectedKey = '';
        setDetailsPlaceholder();
        syncTrackedTopicSubscriptions();
    }

    function getSelectedOverviewKinds() {
        const selected = [];
        if (dom.toggleNodes.checked) {
            selected.push(viewKinds.NODES);
        }
        if (dom.toggleTopics.checked) {
            selected.push(viewKinds.TOPICS);
        }
        if (dom.toggleServices.checked) {
            selected.push(viewKinds.SERVICES);
        }
        if (dom.toggleActions.checked && state.graphData.rosVersion === 2) {
            selected.push(viewKinds.ACTIONS);
        }
        if (dom.toggleParameters.checked) {
            selected.push(viewKinds.PARAMETERS);
        }
        return selected;
    }

    function renderOverviewSection(kind, rows) {
        const section = document.createElement('section');
        section.className = 'nv-overview-section';

        const heading = document.createElement('div');
        heading.className = 'nv-overview-header';
        heading.innerHTML =
            '<h3>' + escapeHtml(kindLabels[kind] || kind) + '</h3>' +
            '<span class="badge info">' + rows.length + '</span>';
        section.appendChild(heading);

        const list = document.createElement('ul');
        list.className = 'item-list nv-entity-list nv-overview-list';
        renderRowsToList(list, rows, (row) => {
            if (canToggleTopicSelectionOff(row)) {
                state.selectedKey = '';
            } else {
                state.selectedKey = row.key;
                if (row.key.startsWith(viewKinds.TOPICS + ':')) {
                    requestTopicRoles(row.name, { force: true });
                }
            }
            syncTrackedTopicSubscriptions();
            renderOverview();
        });

        section.appendChild(list);
        return section;
    }

    function renderListView() {
        if (isViewLoading(state.activeView)) {
            dom.entityCount.textContent = '...';
            dom.entityList.innerHTML =
                '<li class="text-muted"><span class="spinner"></span> Fetching ' +
                escapeHtml(kindLabels[state.activeView] || 'data') +
                '...</li>';
            setDetailsPlaceholder();
            syncTrackedTopicSubscriptions();
            return;
        }

        const rows = getRowsForKind(state.activeView)
            .filter((row) => row.name.toLowerCase().includes(state.filter));

        dom.entityCount.textContent = String(rows.length);
        renderRowsToList(dom.entityList, rows, (row) => {
            if (canToggleTopicSelectionOff(row)) {
                state.selectedKey = '';
            } else {
                state.selectedKey = row.key;
                if (row.key.startsWith(viewKinds.TOPICS + ':')) {
                    requestTopicRoles(row.name, { force: true });
                }
            }
            syncTrackedTopicSubscriptions();
            renderListView();
        });

        const selected = rows.find((row) => row.key === state.selectedKey);
        if (selected) {
            selected.showDetails(dom.detailsCard);
            syncTrackedTopicSubscriptions();
            return;
        }
        setDetailsPlaceholder();
        syncTrackedTopicSubscriptions();
    }

    function renderRowsToList(listEl, rows, onSelect) {
        listEl.innerHTML = '';

        if (rows.length === 0) {
            listEl.innerHTML = '<li class="text-muted">No items.</li>';
            return;
        }

        rows.forEach((row) => {
            const selected = state.selectedKey === row.key;
            const item = document.createElement('li');
            item.className = 'nv-entity-item' + (selected ? ' active' : '');
            const metaClass = row.metaVariant === 'path'
                ? 'nv-entity-meta nv-entity-meta-path'
                : 'nv-entity-meta';
            const nameClass = row.pinTopicName
                ? 'nv-entity-name nv-topic-name'
                : 'nv-entity-name';
            const effectiveMetaClass = row.pinTopicName
                ? (metaClass + ' nv-topic-type-ref')
                : metaClass;

            if (row.pinTopicName) {
                item.classList.add('nv-topic-row-item');

                const pinButton = document.createElement('button');
                pinButton.type = 'button';
                pinButton.className = isTopicPinned(row.pinTopicName)
                    ? 'pin-btn nv-topic-row-pin pinned'
                    : 'pin-btn nv-topic-row-pin';
                pinButton.textContent = '★';
                const pinLabel = isTopicPinned(row.pinTopicName) ? 'Unpin topic' : 'Pin topic';
                pinButton.title = pinLabel;
                pinButton.setAttribute('aria-label', pinLabel);
                pinButton.addEventListener('click', (event) => {
                    event.preventDefault();
                    event.stopPropagation();
                    togglePinnedTopic(row.pinTopicName);
                });
                item.appendChild(pinButton);
            }

            const selectButton = document.createElement('button');
            selectButton.type = 'button';
            selectButton.className = 'nv-entity-btn';
            selectButton.innerHTML =
                '<span class="' + nameClass + '">' + escapeHtml(row.name) + '</span>' +
                '<span class="' + effectiveMetaClass + '" title="' + escapeHtml(row.meta) + '">' +
                escapeHtml(row.meta) +
                '</span>';
            selectButton.addEventListener('click', () => onSelect(row));

            item.appendChild(selectButton);
            listEl.appendChild(item);
        });
    }

    function canToggleTopicSelectionOff(row) {
        if (!row || !row.key || !state.selectedKey) {
            return false;
        }
        return row.key === state.selectedKey
            && row.key.startsWith(viewKinds.TOPICS + ':');
    }

    function findSelectedRow() {
        if (!state.selectedKey) {
            return undefined;
        }
        const colonIndex = state.selectedKey.indexOf(':');
        if (colonIndex < 0) {
            return undefined;
        }
        const kind = state.selectedKey.slice(0, colonIndex);
        return getRowsForKind(kind).find((row) => row.key === state.selectedKey);
    }

    function getRowsForKind(kind) {
        const data = state.graphData;

        if (kind === viewKinds.NODES) {
            return data.nodes.map((name) => {
                const info = data.connections[name];
                let meta;
                if (info) {
                    const relationCount =
                        (info.publishers || []).length +
                        (info.subscribers || []).length +
                        (info.serviceServers || []).length +
                        (info.serviceClients || []).length +
                        (info.actionServers || []).length +
                        (info.actionClients || []).length;
                    meta = relationCount + ' links';
                } else {
                    meta = 'click for details';
                }
                return {
                    key: viewKinds.NODES + ':' + name,
                    name,
                    meta,
                    showDetails: (target) => showNodeDetails(name, target),
                };
            });
        }

        if (kind === viewKinds.TOPICS) {
            return data.topics.map((topic) => ({
                key: viewKinds.TOPICS + ':' + topic.name,
                name: topic.name,
                pinTopicName: topic.name,
                meta: topic.type || 'unknown',
                metaVariant: 'path',
                showDetails: (target) => showEntityDetails(viewKinds.TOPICS, topic.name, target),
            }));
        }

        if (kind === viewKinds.SERVICES) {
            return data.services.map((service) => ({
                key: viewKinds.SERVICES + ':' + service.name,
                name: service.name,
                meta: service.type || 'unknown',
                metaVariant: 'path',
                showDetails: (target) => showEntityDetails(viewKinds.SERVICES, service.name, target),
            }));
        }

        if (kind === viewKinds.ACTIONS) {
            return data.actions.map((action) => ({
                key: viewKinds.ACTIONS + ':' + action.name,
                name: action.name,
                meta: action.type || 'unknown',
                metaVariant: 'path',
                showDetails: (target) => showEntityDetails(viewKinds.ACTIONS, action.name, target),
            }));
        }

        if (kind === viewKinds.PARAMETERS) {
            return data.parameters.map((parameter) => ({
                key: viewKinds.PARAMETERS + ':' + getParameterKey(parameter),
                name: parameter.name || '',
                meta: parameter.node || 'global',
                metaVariant: 'path',
                showDetails: (target) => showParameterDetails(parameter, target),
            }));
        }

        return [];
    }

    function showNodeDetails(name, targetEl) {
        const info = state.graphData.connections[name] || null;
        const nodeParameters = state.graphData.parameters
            .filter((parameter) => parameter.node === name)
            .map((parameter) => parameter.name);
        const isRefreshing = state.nodeInfoLoadingByName[name] === true;

        if (!info) {
            // Info not yet fetched — request it lazily.
            requestNodeInfo(name);
            targetEl.innerHTML =
                '<strong>' + escapeHtml(name) + '</strong> <span class="text-muted">(node)</span>' +
                '<div class="text-muted"><span class="spinner"></span> Loading node info…</div>';
            return;
        }

        targetEl.innerHTML =
            '<strong>' + escapeHtml(name) + '</strong> <span class="text-muted">(node)</span>' +
            renderDetailLine('Publishes', info.publishers || []) +
            renderDetailLine('Subscribes', info.subscribers || []) +
            renderDetailLine('Service servers', info.serviceServers || []) +
            renderDetailLine('Service clients', info.serviceClients || []) +
            renderDetailLine('Action servers', info.actionServers || []) +
            renderDetailLine('Action clients', info.actionClients || []) +
            renderDetailLine('Parameters', nodeParameters) +
            (isRefreshing
                ? '<div class="text-muted text-sm"><span class="spinner"></span> Refreshing node info...</div>'
                : '');
    }

    function showParameterDetails(parameter, targetEl) {
        const nodeLabel = parameter.node || 'global';
        const isRefreshing = state.selectedRefetchPendingByKind[viewKinds.PARAMETERS] === true;
        targetEl.innerHTML =
            '<strong>' + escapeHtml(parameter.name || '') + '</strong> <span class="text-muted">(parameter)</span>' +
            renderDetailLine('Node', [nodeLabel]) +
            renderDetailLine('Name', [parameter.name || '']) +
            (isRefreshing
                ? '<div class="text-muted text-sm"><span class="spinner"></span> Refreshing parameter list...</div>'
                : '');
    }

    function showEntityDetails(kind, name, targetEl) {
        if (kind === viewKinds.TOPICS) {
            showTopicDetails(name, targetEl);
            return;
        }

        const entity = getEntity(kind, name);
        const roles = collectEntityRoles(kind, name);

        const primaryLabel = kind === viewKinds.TOPICS ? 'Publishers' : 'Servers';
        const secondaryLabel = kind === viewKinds.TOPICS ? 'Subscribers' : 'Clients';
        const isRefreshing = kind === viewKinds.SERVICES
            ? state.selectedRefetchPendingByKind[viewKinds.SERVICES] === true
            : (kind === viewKinds.ACTIONS && state.selectedRefetchPendingByKind[viewKinds.ACTIONS] === true);

        targetEl.innerHTML =
            '<strong>' + escapeHtml(name) + '</strong> <span class="text-muted">(' + escapeHtml(kind.slice(0, -1)) + ')</span>' +
            renderDetailLine('Type', [entity ? entity.type : 'unknown']) +
            renderDetailLine(primaryLabel, roles.primary) +
            renderDetailLine(secondaryLabel, roles.secondary) +
            (isRefreshing
                ? '<div class="text-muted text-sm"><span class="spinner"></span> Refreshing relation data...</div>'
                : '');
    }

    function showTopicDetails(name, targetEl) {
        const entity = getEntity(viewKinds.TOPICS, name);
        const cachedRoles = state.topicRolesByName[name];
        const fallbackRoles = collectEntityRoles(viewKinds.TOPICS, name);
        const roles = cachedRoles
            ? { primary: cachedRoles.publishers, secondary: cachedRoles.subscribers }
            : fallbackRoles;
        const loadingRoles = state.topicRolesLoadingByName[name] === true;
        if (!cachedRoles && !loadingRoles) {
            requestTopicRoles(name);
        }
        const isRefreshing = state.selectedRefetchPendingByKind[viewKinds.TOPICS] === true;
        const pinned = isTopicPinned(name);
        const messageBlock = renderTopicMessageBlock(name);
        const pinButtonLabel = pinned ? 'Unpin topic' : 'Pin topic';
        const pinButtonClass = pinned
            ? 'pin-btn nv-topic-pin-btn nv-topic-pin-icon pinned'
            : 'pin-btn nv-topic-pin-btn nv-topic-pin-icon';

        targetEl.innerHTML =
            '<div class="nv-topic-heading">' +
            '<button class="' + pinButtonClass + '" type="button" data-topic-name="' + escapeHtml(name) + '"' +
            ' title="' + escapeHtml(pinButtonLabel) + '" aria-label="' + escapeHtml(pinButtonLabel) + '">' +
            '★' +
            '</button>' +
            '<div><strong>' + escapeHtml(name) + '</strong> <span class="text-muted">(topic)</span></div>' +
            '</div>' +
            renderDetailLine('Type', [entity ? entity.type : 'unknown']) +
            renderDetailLine('Publishers', roles.primary) +
            renderDetailLine('Subscribers', roles.secondary) +
            (loadingRoles || isRefreshing
                ? '<div class="text-muted text-sm"><span class="spinner"></span> Refreshing topic details...</div>'
                : '') +
            messageBlock;
    }

    function renderTopicMessageBlock(topicName, options = {}) {
        const compact = options.compact === true;
        const label = compact ? 'Latest message' : 'Latest cached message';
        const rowClass = compact ? 'nv-topic-msg-row nv-topic-msg-row-compact' : 'nv-topic-msg-row';
        const contentClass = compact ? 'nv-topic-msg nv-topic-msg-compact' : 'nv-topic-msg';
        const cached = typeof options.messageOverride === 'string'
            ? String(options.messageOverride).trim()
            : getTopicCachedMessage(topicName);
        const emptyMessage = String(options.emptyMessage || 'No cached message yet.');
        if (!cached) {
            return (
                '<div class="' + rowClass + '">' +
                '<span class="text-muted">' + escapeHtml(label) + ':</span> <em>' + escapeHtml(emptyMessage) + '</em>' +
                '</div>'
            );
        }

        const ageText = formatTopicMessageAge(topicName);
        const ageLine = ageText
            ? '<div class="text-muted text-sm">' + escapeHtml(ageText) + '</div>'
            : '';

        return (
            '<div class="' + rowClass + '"><span class="text-muted">' + escapeHtml(label) + ':</span></div>' +
            '<pre class="' + contentClass + '">' + escapeHtml(cached) + '</pre>' +
            ageLine
        );
    }

    function getParameterKey(parameter) {
        const name = String(parameter.name || '');
        const node = String(parameter.node || '');
        return node ? node + ':' + name : name;
    }

    function getEntity(kind, name) {
        if (kind === viewKinds.TOPICS) {
            return state.graphData.topics.find((item) => item.name === name);
        }
        if (kind === viewKinds.SERVICES) {
            return state.graphData.services.find((item) => item.name === name);
        }
        if (kind === viewKinds.ACTIONS) {
            return state.graphData.actions.find((item) => item.name === name);
        }
        return undefined;
    }

    function collectEntityRoles(kind, name) {
        const primary = [];
        const secondary = [];

        // Compute reverse roles from node-centric edges to keep state normalized.
        for (const [nodeName, info] of Object.entries(state.graphData.connections || {})) {
            if (kind === viewKinds.TOPICS) {
                if ((info.publishers || []).includes(name)) {
                    primary.push(nodeName);
                }
                if ((info.subscribers || []).includes(name)) {
                    secondary.push(nodeName);
                }
                continue;
            }

            if (kind === viewKinds.SERVICES) {
                if ((info.serviceServers || []).includes(name)) {
                    primary.push(nodeName);
                }
                if ((info.serviceClients || []).includes(name)) {
                    secondary.push(nodeName);
                }
                continue;
            }

            if (kind === viewKinds.ACTIONS) {
                if ((info.actionServers || []).includes(name)) {
                    primary.push(nodeName);
                }
                if ((info.actionClients || []).includes(name)) {
                    secondary.push(nodeName);
                }
            }
        }

        return { primary, secondary };
    }

    function setDetailsPlaceholder() {
        dom.detailsCard.innerHTML = '<span class="text-muted">Select an item to see details.</span>';
    }

    switchView(viewKinds.OVERVIEW);
})();
