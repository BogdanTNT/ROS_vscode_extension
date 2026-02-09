/* Package Manager Webview Index */
(function () {
    window.PM = window.PM || {};

    const { state, render, actions } = window.PM;
    const { toWebview } = window.PM.messages;
    let lastAutoRefreshAt = 0;
    const AUTO_REFRESH_COOLDOWN_MS = 500;

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
                state.otherPackages = (msg.packages || []).slice().sort((a, b) => a.localeCompare(b));
                state.otherPackagesLoaded = true;
                state.otherPackagesLoading = false;
                if (window.PM.dom?.btnLoadOtherPackages) {
                    window.PM.dom.btnLoadOtherPackages.disabled = false;
                    window.PM.dom.btnLoadOtherPackages.textContent = 'Reload';
                }
                render.renderOtherPackages();
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
                    actions.refreshPackages();
                    window.PM.handlers.closeAddNodeModal();
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
