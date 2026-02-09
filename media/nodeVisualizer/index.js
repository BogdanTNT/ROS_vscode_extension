/* Node Visualizer Webview Script */
(function () {
    const vscode = acquireVsCodeApi();

    const commands = Object.freeze({
        toHost: Object.freeze({
            REFRESH: 'refresh',
            SET_AUTO_REFRESH: 'setAutoRefresh',
            SET_OVERVIEW_TOGGLES: 'setOverviewToggles',
            GET_TOPIC_LATEST_MESSAGE: 'getTopicLatestMessage',
            GET_TOPIC_PUBLISH_TEMPLATE: 'getTopicPublishTemplate',
            PUBLISH_TOPIC_MESSAGE: 'publishTopicMessage',
        }),
        toWebview: Object.freeze({
            LOADING: 'loading',
            GRAPH_DATA: 'graphData',
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

    const dom = {
        btnRefresh: document.getElementById('btnRefresh'),
        btnPublishTopic: document.getElementById('btnPublishTopic'),
        toggleAutoRefresh: document.getElementById('toggleAutoRefresh'),
        status: document.getElementById('status'),
        tabs: Array.from(document.querySelectorAll('.nv-tab')),
        tabNodesCount: document.getElementById('tabNodesCount'),
        tabTopicsCount: document.getElementById('tabTopicsCount'),
        tabServicesCount: document.getElementById('tabServicesCount'),
        tabActionsCount: document.getElementById('tabActionsCount'),
        tabParametersCount: document.getElementById('tabParametersCount'),
        overviewView: document.getElementById('overviewView'),
        listView: document.getElementById('listView'),
        overviewSections: document.getElementById('overviewSections'),
        detailsCard: document.getElementById('detailsCard'),
        filterInput: document.getElementById('entityFilter'),
        entityCount: document.getElementById('entityCount'),
        entityList: document.getElementById('entityList'),
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
        topicMessageCache: {},
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
        vscode.postMessage({ command: commands.toHost.REFRESH });
    });

    dom.btnPublishTopic.addEventListener('click', () => {
        openPublishTopicModal();
    });

    dom.toggleAutoRefresh.addEventListener('change', () => {
        vscode.postMessage({
            command: commands.toHost.SET_AUTO_REFRESH,
            enabled: dom.toggleAutoRefresh.checked,
        });
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

    dom.filterInput.addEventListener('input', () => {
        state.filter = dom.filterInput.value.trim().toLowerCase();
        if (state.activeView !== viewKinds.OVERVIEW) {
            renderListView();
        }
    });

    [dom.toggleNodes, dom.toggleTopics, dom.toggleServices, dom.toggleActions, dom.toggleParameters].forEach((toggle) => {
        toggle.addEventListener('change', () => {
            persistOverviewToggleState();
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
            applyPreferences(msg.prefs || {});
            state.topicMessageCache = {};
            state.publishTemplateByTopic = {};
            renderStatus();
            renderTabCounts();
            updateActionAvailability();
            renderPublishButtonState();
            renderCurrentView();
            return;
        }

        if (msg.command === commands.toWebview.TOPIC_LATEST_MESSAGE) {
            const topicName = String(msg.topicName || '');
            if (!topicName) {
                return;
            }
            const message = String(msg.message || '').trim();
            state.topicMessageCache[topicName] = message
                ? { status: 'ready', value: message }
                : { status: 'empty', value: '' };

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
            state.topicMessageCache[topicName] = { status: 'loading', value: '' };
            if (state.selectedKey === (viewKinds.TOPICS + ':' + topicName)) {
                showTopicDetails(topicName, dom.detailsCard);
            }
        }
    }

    function switchView(view) {
        state.activeView = view;
        state.selectedKey = '';
        dom.tabs.forEach((tab) => tab.classList.toggle('active', tab.dataset.view === view));
        renderPublishButtonState();

        if (view === viewKinds.OVERVIEW) {
            closePublishTopicModal();
            dom.overviewView.classList.remove('hidden');
            dom.listView.classList.add('hidden');
            renderOverview();
            return;
        }

        if (view !== viewKinds.TOPICS) {
            closePublishTopicModal();
        }

        dom.overviewView.classList.add('hidden');
        dom.listView.classList.remove('hidden');
        dom.filterInput.placeholder = 'Filter ' + view + '...';
        renderListView();
    }

    function renderCurrentView() {
        if (state.activeView === viewKinds.OVERVIEW) {
            renderOverview();
            return;
        }
        renderListView();
    }

    function renderOverview() {
        const selectedKinds = getSelectedOverviewKinds();
        const allVisibleRows = [];

        dom.overviewSections.innerHTML = '';

        if (selectedKinds.length === 0) {
            dom.overviewSections.innerHTML = '<div class="text-muted">Select at least one list above.</div>';
            setDetailsPlaceholder();
            state.selectedKey = '';
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
            return;
        }

        state.selectedKey = '';
        setDetailsPlaceholder();
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
            state.selectedKey = row.key;
            renderOverview();
        });

        section.appendChild(list);
        return section;
    }

    function renderListView() {
        const rows = getRowsForKind(state.activeView)
            .filter((row) => row.name.toLowerCase().includes(state.filter));

        dom.entityCount.textContent = String(rows.length);
        renderRowsToList(dom.entityList, rows, (row) => {
            state.selectedKey = row.key;
            renderListView();
        });

        const selected = rows.find((row) => row.key === state.selectedKey);
        if (selected) {
            selected.showDetails(dom.detailsCard);
            return;
        }
        setDetailsPlaceholder();
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

            const button = document.createElement('button');
            button.type = 'button';
            button.className = 'nv-entity-btn';
            button.innerHTML =
                '<span class="nv-entity-name">' + escapeHtml(row.name) + '</span>' +
                '<span class="' + metaClass + '" title="' + escapeHtml(row.meta) + '">' +
                escapeHtml(row.meta) +
                '</span>';
            button.addEventListener('click', () => onSelect(row));

            item.appendChild(button);
            listEl.appendChild(item);
        });
    }

    function getRowsForKind(kind) {
        const data = state.graphData;

        if (kind === viewKinds.NODES) {
            return data.nodes.map((name) => {
                const info = data.connections[name] || emptyNodeInfo;
                const relationCount =
                    (info.publishers || []).length +
                    (info.subscribers || []).length +
                    (info.serviceServers || []).length +
                    (info.serviceClients || []).length +
                    (info.actionServers || []).length +
                    (info.actionClients || []).length;
                return {
                    key: viewKinds.NODES + ':' + name,
                    name,
                    meta: relationCount + ' links',
                    showDetails: (target) => showNodeDetails(name, target),
                };
            });
        }

        if (kind === viewKinds.TOPICS) {
            return data.topics.map((topic) => ({
                key: viewKinds.TOPICS + ':' + topic.name,
                name: topic.name,
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
        const info = state.graphData.connections[name] || emptyNodeInfo;
        const nodeParameters = state.graphData.parameters
            .filter((parameter) => parameter.node === name)
            .map((parameter) => parameter.name);

        targetEl.innerHTML =
            '<strong>' + escapeHtml(name) + '</strong> <span class="text-muted">(node)</span>' +
            renderDetailLine('Publishes', info.publishers || []) +
            renderDetailLine('Subscribes', info.subscribers || []) +
            renderDetailLine('Service servers', info.serviceServers || []) +
            renderDetailLine('Service clients', info.serviceClients || []) +
            renderDetailLine('Action servers', info.actionServers || []) +
            renderDetailLine('Action clients', info.actionClients || []) +
            renderDetailLine('Parameters', nodeParameters);
    }

    function showParameterDetails(parameter, targetEl) {
        const nodeLabel = parameter.node || 'global';
        targetEl.innerHTML =
            '<strong>' + escapeHtml(parameter.name || '') + '</strong> <span class="text-muted">(parameter)</span>' +
            renderDetailLine('Node', [nodeLabel]) +
            renderDetailLine('Name', [parameter.name || '']);
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

        targetEl.innerHTML =
            '<strong>' + escapeHtml(name) + '</strong> <span class="text-muted">(' + escapeHtml(kind.slice(0, -1)) + ')</span>' +
            renderDetailLine('Type', [entity ? entity.type : 'unknown']) +
            renderDetailLine(primaryLabel, roles.primary) +
            renderDetailLine(secondaryLabel, roles.secondary);
    }

    function showTopicDetails(name, targetEl) {
        const entity = getEntity(viewKinds.TOPICS, name);
        const roles = collectEntityRoles(viewKinds.TOPICS, name);
        const messageBlock = renderTopicMessageBlock(name);

        targetEl.innerHTML =
            '<strong>' + escapeHtml(name) + '</strong> <span class="text-muted">(topic)</span>' +
            renderDetailLine('Type', [entity ? entity.type : 'unknown']) +
            renderDetailLine('Publishers', roles.primary) +
            renderDetailLine('Subscribers', roles.secondary) +
            messageBlock;
    }

    function renderTopicMessageBlock(topicName) {
        const cached = state.topicMessageCache[topicName];
        if (!cached) {
            state.topicMessageCache[topicName] = { status: 'loading', value: '' };
            requestTopicLatestMessage(topicName);
            return '<div class="nv-topic-msg-row"><span class="text-muted">Latest message:</span> <em>Loading...</em></div>';
        }

        if (cached.status === 'loading') {
            return '<div class="nv-topic-msg-row"><span class="text-muted">Latest message:</span> <em>Loading...</em></div>';
        }

        if (cached.status === 'empty') {
            return '<div class="nv-topic-msg-row"><span class="text-muted">Latest message:</span> <em>No recent message received.</em></div>';
        }

        return (
            '<div class="nv-topic-msg-row"><span class="text-muted">Latest message:</span></div>' +
            '<pre class="nv-topic-msg">' + escapeHtml(cached.value || '') + '</pre>'
        );
    }

    function requestTopicLatestMessage(topicName) {
        vscode.postMessage({
            command: commands.toHost.GET_TOPIC_LATEST_MESSAGE,
            topicName,
        });
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
    vscode.postMessage({ command: commands.toHost.REFRESH });
})();
