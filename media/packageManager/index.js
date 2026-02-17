/* Package Manager Webview Index */
(function () {
    window.PM = window.PM || {};

    const { state, render, actions } = window.PM;
    const { toWebview } = window.PM.messages;
    let lastAutoRefreshAt = 0;
    const AUTO_REFRESH_COOLDOWN_MS = 500;
    const normalizePackageName = (value) => {
        if (typeof value !== 'string') {
            return '';
        }
        return value.trim();
    };
    const normalizePackageDetails = (value) => {
        if (typeof value === 'string') {
            return {
                name: value,
                packagePath: '',
                launchFiles: [],
                nodes: [],
                isPython: false,
                isCmake: false,
                detailsLoaded: false,
                detailsLoading: false,
            };
        }
        if (!value || typeof value !== 'object') {
            return null;
        }
        const name = typeof value.name === 'string' ? value.name.trim() : '';
        if (!name) {
            return null;
        }
        const launchFiles = Array.isArray(value.launchFiles)
            ? value.launchFiles.filter((item) => typeof item === 'string')
            : [];
        const nodes = Array.isArray(value.nodes)
            ? value.nodes
                .filter((node) => node && typeof node === 'object' && typeof node.name === 'string')
                .map((node) => ({
                    name: node.name,
                    sourcePath: typeof node.sourcePath === 'string' ? node.sourcePath : '',
                }))
            : [];

        return {
            name,
            packagePath: typeof value.packagePath === 'string' ? value.packagePath : '',
            launchFiles,
            nodes,
            isPython: value.isPython === true,
            isCmake: value.isCmake === true,
            detailsLoaded: true,
            detailsLoading: false,
        };
    };
    const normalizeOtherPackageNames = (packages) => {
        if (!Array.isArray(packages)) {
            return [];
        }
        const names = new Set();
        packages.forEach((item) => {
            if (typeof item === 'string') {
                const name = normalizePackageName(item);
                if (name) {
                    names.add(name);
                }
                return;
            }
            if (item && typeof item === 'object') {
                const name = normalizePackageName(item.name);
                if (name) {
                    names.add(name);
                }
            }
        });
        return Array.from(names);
    };
    const buildOtherPackageModel = (name, existing) => ({
        name,
        packagePath: existing?.packagePath || '',
        launchFiles: Array.isArray(existing?.launchFiles) ? existing.launchFiles : [],
        nodes: Array.isArray(existing?.nodes) ? existing.nodes : [],
        isPython: existing?.isPython === true,
        isCmake: existing?.isCmake === true,
        detailsLoaded: existing?.detailsLoaded === true,
        detailsLoading: false,
    });

    const requestAutoRefresh = () => {
        const now = Date.now();
        if (now - lastAutoRefreshAt < AUTO_REFRESH_COOLDOWN_MS) {
            return;
        }
        lastAutoRefreshAt = now;
        actions.refreshPackages();
    };

    window.addEventListener('message', (event) => {
        const msg = event.data;
        switch (msg.command) {
            case toWebview.PACKAGE_LIST: {
                state.allPackages = msg.packages.slice().sort((a, b) => a.name.localeCompare(b.name));
                state.pinnedPaths = msg.pinned || [];
                state.launchArgConfigs = msg.launchArgConfigs || {};
                state.terminals = msg.terminals || [];
                state.preferredTerminalId = msg.preferredTerminalId || '';
                render.renderPackages();
                render.renderOtherPackages();
                render.renderPinned();
                render.renderTerminals();
                break;
            }
            case toWebview.OTHER_PACKAGE_LIST: {
                const names = normalizeOtherPackageNames(msg.packages || []);
                const existingByName = new Map(
                    (state.otherPackages || []).map((pkg) => [pkg.name, pkg])
                );
                state.otherPackages = names
                    .map((name) => buildOtherPackageModel(name, existingByName.get(name)))
                    .sort((a, b) => a.name.localeCompare(b.name));
                state.otherPackagesLoaded = true;
                state.otherPackagesLoading = false;
                if (window.PM.dom?.btnLoadOtherPackages) {
                    window.PM.dom.btnLoadOtherPackages.disabled = false;
                    window.PM.dom.btnLoadOtherPackages.textContent = 'Reload';
                }
                render.renderOtherPackages();
                render.renderPinned();
                break;
            }
            case toWebview.OTHER_PACKAGE_DETAILS: {
                const detail = normalizePackageDetails(msg.package || msg.details);
                if (!detail?.name) {
                    break;
                }

                const current = state.otherPackages || [];
                const existingIdx = current.findIndex((pkg) => pkg.name === detail.name);
                if (existingIdx >= 0) {
                    current[existingIdx] = {
                        ...current[existingIdx],
                        ...detail,
                        detailsLoaded: true,
                        detailsLoading: false,
                    };
                } else {
                    current.push({
                        ...detail,
                        detailsLoaded: true,
                        detailsLoading: false,
                    });
                }
                state.otherPackages = current.sort((a, b) => a.name.localeCompare(b.name));
                render.renderOtherPackages();
                render.renderPinned();
                break;
            }
            case toWebview.CREATE_DONE: {
                const { dom } = window.PM;
                dom.btnCreate.disabled = false;
                dom.statusEl.innerHTML = msg.success
                    ? '<span class="badge success">✓ Created</span>'
                    : '<span class="badge error">✗ Failed</span>';
                if (msg.success) {
                    actions.refreshPackages();
                    window.PM.handlers.closeCreate();
                }
                break;
            }
            case toWebview.FOCUS_CREATE: {
                window.PM.handlers.openCreate();
                break;
            }
            case toWebview.LAUNCH_ARGS_OPTIONS: {
                const argsKey = msg.argsKey || msg.path;
                if (argsKey !== state.currentArgsKey) {
                    break;
                }
                state.argsOptions = msg.options || [];
                render.renderArgsOptions();
                break;
            }
            case toWebview.TERMINAL_LIST: {
                state.terminals = msg.terminals || [];
                render.renderPackages();
                render.renderOtherPackages();
                render.renderPinned();
                render.renderTerminals();
                break;
            }
            case toWebview.BUILD_CHECK_STATE: {
                const { dom: d } = window.PM;
                if (d.toggleBuildCheck) {
                    d.toggleBuildCheck.checked = Boolean(msg.enabled);
                }
                break;
            }
            case toWebview.ADD_NODE_DONE: {
                const { dom: d2 } = window.PM;
                d2.btnAddNode.disabled = false;
                d2.addNodeStatus.innerHTML = msg.success
                    ? '<span class="badge success">✓ Node added</span>'
                    : '<span class="badge error">✗ Failed</span>';
                if (msg.success) {
                    const currentTopic = String(d2.addNodeTopic?.value || '')
                        .trim()
                        .replace(/\s+/g, '_');
                    if (currentTopic) {
                        if (d2.addNodeTopic) {
                            d2.addNodeTopic.value = currentTopic;
                        }
                        if (typeof window.PM.setLastNodeTopic === 'function') {
                            window.PM.setLastNodeTopic(currentTopic);
                        } else {
                            state.lastNodeTopic = currentTopic;
                        }
                    }
                    actions.refreshPackages();
                    window.PM.handlers.closeAddNodeModal();
                }
                break;
            }
            case toWebview.REMOVE_NODE_DONE: {
                const { dom: d3 } = window.PM;
                d3.btnRemoveNode.disabled = false;
                d3.removeNodeStatus.innerHTML = msg.success
                    ? '<span class="badge success">✓ Node removed</span>'
                    : '<span class="badge error">✗ Failed</span>';
                if (msg.success) {
                    actions.refreshPackages();
                    window.PM.handlers.closeRemoveNodeModal();
                }
                break;
            }
        }
    });

    // Recover from missed initial messages and stale data after tab/window switches.
    requestAutoRefresh();

    window.addEventListener('focus', () => {
        requestAutoRefresh();
    });

    document.addEventListener('visibilitychange', () => {
        if (document.visibilityState === 'visible') {
            requestAutoRefresh();
        }
    });
})();
