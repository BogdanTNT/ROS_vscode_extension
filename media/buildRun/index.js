/* Build & Run Webview Script */
(function () {
    const vscode = acquireVsCodeApi();

    const commands = Object.freeze({
        toHost: Object.freeze({
            BUILD: 'build',
            SMART_BUILD: 'smartBuild',
            MARK_BUILT: 'markBuilt',
            REFRESH_PACKAGES: 'refreshPackages',
            REQUEST_BUILD_STATUS: 'requestBuildStatus',
        }),
        toWebview: Object.freeze({
            PACKAGE_LIST: 'packageList',
            FOCUS_BUILD: 'focusBuild',
            BUILD_STATUS_MAP: 'buildStatusMap',
            BUILD_STATUS: 'buildStatus',
        }),
    });

    const dom = {
        dropdownRoot: document.getElementById('buildDropdown'),
        dropdownBtn: document.getElementById('btnPackageDropdown'),
        dropdownMenu: document.getElementById('packageDropdownMenu'),
        packageList: document.getElementById('buildPackageList'),
        btnSelectAll: document.getElementById('btnSelectAllPackages'),
        btnClear: document.getElementById('btnClearPackages'),
        btnRefresh: document.getElementById('btnRefreshPackages'),
        btnBuild: document.getElementById('btnBuildSelected'),
        btnSmartBuild: document.getElementById('btnSmartBuild'),
        summary: document.getElementById('buildSelectionSummary'),
        statusSummary: document.getElementById('buildStatusSummary'),
        status: document.getElementById('buildStatus'),
    };

    const state = {
        workspacePackages: [],
        selectedPackages: new Set(),
        dropdownOpen: false,
        /** @type {Record<string, {needsBuild: boolean, reason: string, reasonCode: string}>} */
        buildStatusMap: {},
    };

    const normalizePackages = (rawList) => {
        if (!Array.isArray(rawList)) {
            return [];
        }
        const seen = new Set();
        const valid = [];

        rawList.forEach((value) => {
            if (typeof value !== 'string') {
                return;
            }
            const name = value.trim();
            if (!name || seen.has(name)) {
                return;
            }
            seen.add(name);
            valid.push(name);
        });

        valid.sort((a, b) => a.localeCompare(b));
        return valid;
    };

    const setDropdownOpen = (open) => {
        state.dropdownOpen = open;
        if (open) {
            dom.dropdownMenu.classList.remove('hidden');
        } else {
            dom.dropdownMenu.classList.add('hidden');
        }
        dom.dropdownBtn.setAttribute('aria-expanded', open ? 'true' : 'false');
    };

    const renderStatus = (text) => {
        dom.status.textContent = text || '';
    };

    const renderSummary = () => {
        const selected = Array.from(state.selectedPackages).sort((a, b) => a.localeCompare(b));
        const count = selected.length;

        if (count === 0) {
            dom.dropdownBtn.textContent = 'Select packages';
            dom.summary.textContent = 'No packages selected.';
            dom.btnBuild.disabled = true;
            dom.btnSmartBuild.disabled = true;
            renderBuildStatusSummary();
            return;
        }

        dom.dropdownBtn.textContent = count === 1 ? '1 package selected' : count + ' packages selected';
        const preview = selected.slice(0, 3).join(', ');
        const suffix = count > 3 ? ' +' + (count - 3) + ' more' : '';
        dom.summary.textContent = 'Selected: ' + preview + suffix;
        dom.btnBuild.disabled = false;
        dom.btnSmartBuild.disabled = false;
        renderBuildStatusSummary();
    };

    const renderBuildStatusSummary = () => {
        if (!dom.statusSummary) {
            return;
        }
        const selected = Array.from(state.selectedPackages);
        if (selected.length === 0 || Object.keys(state.buildStatusMap).length === 0) {
            dom.statusSummary.textContent = '';
            return;
        }

        const stale = [];
        const fresh = [];
        for (const name of selected) {
            const info = state.buildStatusMap[name];
            if (!info) {
                stale.push(name);
            } else if (info.needsBuild) {
                stale.push(name);
            } else {
                fresh.push(name);
            }
        }

        if (stale.length === 0) {
            dom.statusSummary.textContent = '✅ All selected packages are up to date.';
            dom.statusSummary.className = 'text-sm mt';
            dom.statusSummary.style.color = 'var(--success)';
        } else {
            const names = stale.slice(0, 3).join(', ');
            const extra = stale.length > 3 ? ' +' + (stale.length - 3) + ' more' : '';
            dom.statusSummary.textContent = '🔨 Needs build: ' + names + extra;
            dom.statusSummary.className = 'text-sm mt';
            dom.statusSummary.style.color = '';
        }
    };

    const renderPackageOptions = () => {
        dom.packageList.innerHTML = '';

        if (!state.workspacePackages.length) {
            const li = document.createElement('li');
            li.className = 'text-muted';
            li.textContent = 'No workspace packages found.';
            dom.packageList.appendChild(li);
            return;
        }

        state.workspacePackages.forEach((name) => {
            const li = document.createElement('li');
            li.className = 'build-package-item';

            const label = document.createElement('label');
            label.className = 'build-package-label';

            const input = document.createElement('input');
            input.type = 'checkbox';
            input.dataset.pkg = name;
            input.checked = state.selectedPackages.has(name);

            const text = document.createElement('span');
            text.textContent = name;

            label.appendChild(input);
            label.appendChild(text);

            // Build-status badge
            const info = state.buildStatusMap[name];
            if (info) {
                const badge = document.createElement('span');
                badge.className = 'badge ' + (info.needsBuild ? 'error' : 'success');
                badge.textContent = info.needsBuild ? '⬤' : '✓';
                badge.title = info.reason;
                badge.style.marginLeft = '6px';
                badge.style.fontSize = '0.7em';
                label.appendChild(badge);
            }

            li.appendChild(label);
            dom.packageList.appendChild(li);
        });
    };

    const updateSelectedFromInputs = () => {
        const selected = new Set();
        dom.packageList.querySelectorAll('input[type="checkbox"][data-pkg]').forEach((input) => {
            if (input instanceof HTMLInputElement && input.checked && input.dataset.pkg) {
                selected.add(input.dataset.pkg);
            }
        });
        state.selectedPackages = selected;
        renderSummary();
    };

    dom.dropdownBtn.addEventListener('click', (event) => {
        event.preventDefault();
        setDropdownOpen(!state.dropdownOpen);
    });

    dom.btnSelectAll.addEventListener('click', (event) => {
        event.preventDefault();
        state.selectedPackages = new Set(state.workspacePackages);
        renderPackageOptions();
        renderSummary();
    });

    dom.btnClear.addEventListener('click', (event) => {
        event.preventDefault();
        state.selectedPackages = new Set();
        renderPackageOptions();
        renderSummary();
    });

    dom.btnRefresh.addEventListener('click', () => {
        renderStatus('Refreshing workspace packages…');
        vscode.postMessage({ command: commands.toHost.REFRESH_PACKAGES });
    });

    dom.packageList.addEventListener('change', (event) => {
        const target = event.target;
        if (!(target instanceof HTMLInputElement)) {
            return;
        }
        if (target.type !== 'checkbox') {
            return;
        }
        updateSelectedFromInputs();
    });

    dom.btnBuild.addEventListener('click', () => {
        const packages = Array.from(state.selectedPackages);
        if (!packages.length) {
            return;
        }
        renderStatus('Build started for ' + packages.length + ' package(s)…');
        vscode.postMessage({ command: commands.toHost.BUILD, packages });
    });

    dom.btnSmartBuild.addEventListener('click', () => {
        const packages = Array.from(state.selectedPackages);
        if (!packages.length) {
            return;
        }
        renderStatus('Smart build checking ' + packages.length + ' package(s)…');
        vscode.postMessage({ command: commands.toHost.SMART_BUILD, packages });
    });

    document.addEventListener('click', (event) => {
        if (!state.dropdownOpen) {
            return;
        }
        const target = event.target;
        if (!(target instanceof Node)) {
            return;
        }
        if (!dom.dropdownRoot.contains(target)) {
            setDropdownOpen(false);
        }
    });

    document.addEventListener('keydown', (event) => {
        if (event.key === 'Escape') {
            setDropdownOpen(false);
            dom.dropdownBtn.focus();
        }
    });

    window.addEventListener('message', (event) => {
        const msg = event.data;
        if (msg.command === commands.toWebview.PACKAGE_LIST) {
            state.workspacePackages = normalizePackages(msg.packages);
            const selectedStillValid = new Set();
            state.selectedPackages.forEach((name) => {
                if (state.workspacePackages.includes(name)) {
                    selectedStillValid.add(name);
                }
            });
            state.selectedPackages = selectedStillValid;
            renderPackageOptions();
            renderSummary();
            renderStatus('Loaded ' + state.workspacePackages.length + ' workspace package(s).');
            return;
        }
        if (msg.command === commands.toWebview.FOCUS_BUILD) {
            dom.dropdownBtn.focus();
            setDropdownOpen(true);
            return;
        }
        if (msg.command === commands.toWebview.BUILD_STATUS_MAP) {
            state.buildStatusMap = msg.statusMap || {};
            renderPackageOptions();
            renderBuildStatusSummary();
            return;
        }
        if (msg.command === commands.toWebview.BUILD_STATUS) {
            renderStatus(msg.text || '');
            return;
        }
    });

    renderSummary();
    vscode.postMessage({ command: commands.toHost.REFRESH_PACKAGES });
    vscode.postMessage({ command: commands.toHost.REQUEST_BUILD_STATUS });
})();
