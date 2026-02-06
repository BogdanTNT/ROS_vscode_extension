/* Package Manager Webview Rendering */
(function () {
    window.PM = window.PM || {};

    const { dom, state, actions } = window.PM;

    let launchListEventsBound = false;
    let terminalEventsBound = false;
    const killFeedbackUntil = new Map();
    const KILL_FEEDBACK_MS = 1200;

    const getPkgList = () => document.getElementById('pkgList');
    const isPackageExpanded = (packageName) => Boolean(state.expandedPackages?.[packageName]);
    const setPackageExpanded = (packageName, expanded) => {
        if (!packageName) {
            return;
        }
        if (!state.expandedPackages || typeof state.expandedPackages !== 'object') {
            state.expandedPackages = {};
        }
        state.expandedPackages[packageName] = expanded;
    };
    const togglePackageExpanded = (packageName) => {
        setPackageExpanded(packageName, !isPackageExpanded(packageName));
        renderPackages();
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
    const escapeAttr = (value) => escapeHtml(value);

    const getRunningLaunchPaths = () => {
        const running = new Set();
        (state.terminals || []).forEach((t) => {
            if (t.launchPath) {
                running.add(t.launchPath);
            }
        });
        return running;
    };

    const getLaunchItemData = (launchItem) => {
        if (!launchItem) {
            return { pkg: '', file: '', path: '' };
        }

        return {
            pkg: launchItem.dataset.pkg || '',
            file: launchItem.dataset.file || '',
            path: launchItem.dataset.path || '',
        };
    };

    const launchFromItem = (launchItem, args, argsName) => {
        const { pkg, file, path } = getLaunchItemData(launchItem);
        if (!pkg || !file || !path) {
            return;
        }
        actions.launchFile(pkg, file, path, args || '', argsName || '');
    };

    const openFromItem = (launchItem) => {
        const { path } = getLaunchItemData(launchItem);
        if (!path) {
            return;
        }
        actions.openLaunch(path);
    };

    const togglePinFromItem = (launchItem) => {
        const { path } = getLaunchItemData(launchItem);
        if (!path) {
            return;
        }
        actions.togglePin(path);
    };

    const openArgsFromItem = (launchItem) => {
        const { path } = getLaunchItemData(launchItem);
        if (!path) {
            return;
        }
        if (window.PM.handlers?.openArgsModal) {
            window.PM.handlers.openArgsModal(path);
        }
    };

    const launchFromConfigButton = (configBtn, launchItem) => {
        const id = configBtn?.dataset?.id;
        const path = configBtn?.dataset?.path;

        if (!id || !path) {
            return;
        }

        const cfg = state.launchArgConfigs[path];
        const match = cfg?.configs?.find((c) => c.id === id);
        launchFromItem(launchItem, match?.args || '', match?.name || '');
    };

    const handleLaunchListClick = (event) => {
        const target = event.target;
        if (!(target instanceof Element)) {
            return;
        }

        const pkgToggleEl = target.closest('.pkg-toggle');
        if (pkgToggleEl) {
            togglePackageExpanded(pkgToggleEl.dataset.pkgName || '');
            return;
        }

        const pkgNameEl = target.closest('.pkg-name');
        if (pkgNameEl) {
            copyPackage(pkgNameEl.dataset.name);
            return;
        }

        const launchItem = target.closest('.launch-item');
        if (!launchItem) {
            return;
        }

        const configBtn = target.closest('.config-pill');
        if (configBtn) {
            event.stopPropagation();
            launchFromConfigButton(configBtn, launchItem);
            return;
        }

        if (target.closest('.launch-run')) {
            launchFromItem(launchItem, '', '');
            return;
        }

        if (target.closest('.launch-open')) {
            openFromItem(launchItem);
            return;
        }

        if (target.closest('.args-btn')) {
            openArgsFromItem(launchItem);
            return;
        }

        if (target.closest('.pin-btn')) {
            togglePinFromItem(launchItem);
        }
    };

    const handleLaunchListKeydown = (event) => {
        if (event.key !== 'Enter') {
            return;
        }

        const target = event.target;
        if (!(target instanceof Element)) {
            return;
        }

        const pkgNameEl = target.closest('.pkg-name');
        if (pkgNameEl) {
            event.preventDefault();
            copyPackage(pkgNameEl.dataset.name);
            return;
        }

        const launchItem = target.closest('.launch-item');
        if (!launchItem) {
            return;
        }

        if (target.closest('.launch-run')) {
            event.preventDefault();
            launchFromItem(launchItem, '', '');
            return;
        }

        if (target.closest('.launch-open')) {
            event.preventDefault();
            openFromItem(launchItem);
        }
    };

    const handleTerminalListClick = (event) => {
        const target = event.target;
        if (!(target instanceof Element)) {
            return;
        }

        const terminalItem = target.closest('.terminal-item');
        const id = terminalItem?.dataset?.id;

        if (!id) {
            return;
        }

        if (target.closest('.term-use')) {
            actions.setPreferredTerminal(id);
            return;
        }

        if (target.closest('.term-focus')) {
            actions.focusTerminal(id);
            return;
        }

        if (target.closest('.term-kill')) {
            const pendingUntil = killFeedbackUntil.get(id) || 0;
            if (pendingUntil > Date.now()) {
                return;
            }
            actions.killTerminal(id);
            const expiresAt = Date.now() + KILL_FEEDBACK_MS;
            killFeedbackUntil.set(id, expiresAt);
            renderTerminals();
            setTimeout(() => {
                if ((killFeedbackUntil.get(id) || 0) <= Date.now()) {
                    killFeedbackUntil.delete(id);
                    renderTerminals();
                }
            }, KILL_FEEDBACK_MS + 50);
        }
    };

    const bindDelegatedEvents = () => {
        if (!launchListEventsBound) {
            const pkgList = getPkgList();
            if (pkgList) {
                pkgList.addEventListener('click', handleLaunchListClick);
                pkgList.addEventListener('keydown', handleLaunchListKeydown);
            }

            if (dom.pinnedList) {
                dom.pinnedList.addEventListener('click', handleLaunchListClick);
                dom.pinnedList.addEventListener('keydown', handleLaunchListKeydown);
            }

            launchListEventsBound = true;
        }

        if (!terminalEventsBound && dom.terminalList) {
            dom.terminalList.addEventListener('click', handleTerminalListClick);
            terminalEventsBound = true;
        }
    };

    const getFileName = (path) => path.split('/').pop() || path;

    const buildConfigButtonsHtml = (path, cfg) => {
        if (!cfg || !cfg.configs || !cfg.configs.length) {
            return '';
        }

        return cfg.configs
            .map((c) => (
                '<button class="config-pill small" data-path="' +
                escapeAttr(path) +
                '" data-id="' +
                escapeAttr(c.id) +
                '">' +
                escapeHtml(c.name) +
                '</button>'
            ))
            .join('');
    };

    const buildLaunchItemHtml = ({
        packageName,
        filePath,
        openLabel,
        isPinned,
        isRunning,
    }) => {
        const fileName = getFileName(filePath);
        const cfg = state.launchArgConfigs[filePath];
        const configButtons = buildConfigButtonsHtml(filePath, cfg);
        const runningBadge = isRunning ? '<span class="badge success">running</span>' : '';
        const runningClass = isRunning ? ' running' : '';
        const pinTitle = isPinned ? 'Unpin' : 'Pin';

        return (
            '<li class="launch-item' +
            runningClass +
            '" data-path="' +
            escapeAttr(filePath) +
            '" data-pkg="' +
            escapeAttr(packageName) +
            '" data-file="' +
            escapeAttr(fileName) +
            '">' +
            '<span class="launch-action launch-run" tabindex="0">Run</span>' +
            '<span class="launch-action launch-open" tabindex="0">' +
            escapeHtml(openLabel) +
            '</span>' +
            '<button class="args-btn" title="Edit args">⚙</button>' +
            '<span class="config-pill-group">' +
            configButtons +
            '</span>' +
            runningBadge +
            '<button class="pin-btn ' +
            (isPinned ? 'pinned' : '') +
            '" title="' +
            escapeAttr(pinTitle) +
            '">★</button>' +
            '</li>'
        );
    };

    const renderArgsOptions = () => {
        if (!state.argsOptions.length) {
            dom.argsList.innerHTML = '<li class="text-muted">No arguments detected</li>';
            return;
        }

        dom.argsList.innerHTML = state.argsOptions
            .map((opt) => {
                const value = opt.defaultValue ? opt.name + ':=' + opt.defaultValue : opt.name + ':=';
                const label = opt.defaultValue ? opt.name + ' (default ' + opt.defaultValue + ')' : opt.name + ' (no default)';
                return (
                    '<li class="arg-item" data-value="' + escapeAttr(value) + '">' +
                    '<span>' + escapeHtml(label) + '</span>' +
                    '<button class="secondary small">Add</button>' +
                    '</li>'
                );
            })
            .join('');

        dom.argsList.querySelectorAll('.arg-item').forEach((el) => {
            const btn = el.querySelector('button');
            btn.addEventListener('click', () => {
                const val = el.dataset.value;
                if (!val) {
                    return;
                }
                const current = dom.argsInput.value.trim();
                dom.argsInput.value = current ? current + ' ' + val : val;
            });
        });
    };

    const renderConfigTabs = () => {
        if (!dom.configList) {
            return;
        }
        const cfg = state.launchArgConfigs[state.currentArgsPath];
        if (!cfg || !cfg.configs?.length) {
            dom.configList.innerHTML = '<span class="text-muted text-sm">No configs</span>';
            return;
        }

        dom.configList.innerHTML = cfg.configs
            .map((c) => {
                const active = c.id === state.currentConfigId ? ' active' : '';
                return '<button class="config-pill' + active + '" data-id="' + escapeAttr(c.id) + '">' + escapeHtml(c.name) + '</button>';
            })
            .join('');

        dom.configList.querySelectorAll('.config-pill').forEach((el) => {
            el.addEventListener('click', () => {
                const id = el.dataset.id;
                if (!id) {
                    return;
                }
                state.currentConfigId = id;
                const cfgItem = cfg.configs.find((c) => c.id === state.currentConfigId);
                dom.argsInput.value = cfgItem?.args || '';
                dom.configName.value = cfgItem?.name || '';
                renderConfigTabs();
            });
        });
    };

    const renderTerminals = () => {
        bindDelegatedEvents();

        if (!dom.terminalList) {
            return;
        }

        const now = Date.now();
        for (const [id, until] of killFeedbackUntil.entries()) {
            if (until <= now) {
                killFeedbackUntil.delete(id);
            }
        }

        if (!state.terminals.length) {
            dom.terminalList.innerHTML = '<li class="text-muted">No active launch terminals</li>';
            return;
        }

        const activeTerminalIds = new Set(state.terminals.map((t) => t.id));
        for (const id of killFeedbackUntil.keys()) {
            if (!activeTerminalIds.has(id)) {
                killFeedbackUntil.delete(id);
            }
        }

        dom.terminalList.innerHTML = state.terminals
            .map((t) => {
                const kindLabel = t.kind === 'external' ? 'External' : 'VS Code';
                const kindBadge = '<span class="badge info">' + kindLabel + '</span>';
                const statusBadge =
                    t.status === 'running'
                        ? '<span class="badge success">running</span>'
                        : '<span class="badge error">closed</span>';
                const launchLabel = t.launchLabel ? t.launchLabel : 'Idle';
                const preferred = t.isPreferred ? '<span class="badge info">active</span>' : '';
                const useBtn =
                    t.kind === 'integrated'
                        ? '<button class="secondary small term-use">Use</button>'
                        : '';
                const focusBtn =
                    t.kind === 'integrated'
                        ? '<button class="secondary small term-focus">Focus</button>'
                        : '';
                const isKillFeedbackActive = (killFeedbackUntil.get(t.id) || 0) > now;
                const killBtnClass = 'danger small term-kill' + (isKillFeedbackActive ? ' is-pending' : '');
                const killBtnDisabled = isKillFeedbackActive ? ' disabled' : '';
                const killBtnLabel = isKillFeedbackActive ? 'Sent' : 'Kill';

                return (
                    '<li class="terminal-item" data-id="' +
                    escapeAttr(t.id) +
                    '">' +
                    '<div class="terminal-main">' +
                    '<div class="terminal-title">' +
                    '<span class="terminal-name">' +
                    escapeHtml(t.name) +
                    '</span>' +
                    kindBadge +
                    statusBadge +
                    preferred +
                    '</div>' +
                    '<div class="terminal-sub text-muted text-sm">' +
                    escapeHtml(launchLabel) +
                    '</div>' +
                    '</div>' +
                    '<div class="terminal-actions">' +
                    useBtn +
                    focusBtn +
                    '<button class="' + killBtnClass + '"' + killBtnDisabled + '>' + killBtnLabel + '</button>' +
                    '</div>' +
                    '</li>'
                );
            })
            .join('');
    };

    const renderPackages = () => {
        bindDelegatedEvents();

        const list = getPkgList();
        if (!list) {
            return;
        }

        const filter = dom.filterInput.value.trim().toLowerCase();
        const running = getRunningLaunchPaths();
        const filtered = state.allPackages.filter((p) => {
            if (!filter) {
                return true;
            }
            const nameMatch = p.name.toLowerCase().includes(filter);
            const launchMatch = (p.launchFiles || []).some((f) => f.toLowerCase().includes(filter));
            return nameMatch || launchMatch;
        });
        dom.pkgCount.textContent = String(filtered.length);

        if (filtered.length === 0) {
            list.innerHTML = '<li class="text-muted">No packages found</li>';
            return;
        }

        list.innerHTML = filtered
            .map((pkg) => {
                const launchCount = pkg.launchFiles?.length || 0;
                const hasLaunchFiles = launchCount > 0;
                const isExpanded = (filter.length > 0) || !hasLaunchFiles || isPackageExpanded(pkg.name);
                const launchItemsHtml = (pkg.launchFiles || [])
                    .map((filePath) => {
                        const fileName = getFileName(filePath);
                        return buildLaunchItemHtml({
                            packageName: pkg.name,
                            filePath,
                            openLabel: fileName,
                            isPinned: state.pinnedPaths.includes(filePath),
                            isRunning: running.has(filePath),
                        });
                    })
                    .join('');

                const launchSection = launchItemsHtml
                    ? '<ul class="launch-list">' + launchItemsHtml + '</ul>'
                    : '<div class="text-muted text-sm">No launch files</div>';
                const toggleControl = hasLaunchFiles
                    ? (
                        '<button class="pkg-toggle" data-pkg-name="' +
                        escapeAttr(pkg.name) +
                        '" aria-expanded="' +
                        (isExpanded ? 'true' : 'false') +
                        '" title="' +
                        escapeAttr(isExpanded ? 'Collapse package' : 'Expand package') +
                        '">' +
                        (isExpanded ? '▾' : '▸') +
                        '</button>'
                    )
                    : '<span class="pkg-toggle-placeholder"></span>';

                return (
                    '<li class="pkg-row">' +
                    '<div class="pkg-header">' +
                    '<div class="pkg-main">' +
                    toggleControl +
                    '<span class="pkg-name" tabindex="0" data-name="' +
                    escapeAttr(pkg.name) +
                    '">' +
                    escapeHtml(pkg.name) +
                    '</span>' +
                    '</div>' +
                    '<span class="text-muted text-sm">' +
                    launchCount +
                    ' launch</span>' +
                    '</div>' +
                    '<div class="pkg-launches' + (isExpanded ? '' : ' hidden') + '">' +
                    launchSection +
                    '</div>' +
                    '</li>'
                );
            })
            .join('');
    };

    const renderPinned = () => {
        bindDelegatedEvents();

        if (!state.pinnedPaths.length) {
            dom.pinnedList.innerHTML = '<li class="text-muted">No pinned launch files</li>';
            return;
        }

        const running = getRunningLaunchPaths();
        const pinnedItems = [];

        for (const pkg of state.allPackages) {
            for (const filePath of pkg.launchFiles || []) {
                if (!state.pinnedPaths.includes(filePath)) {
                    continue;
                }
                pinnedItems.push(buildLaunchItemHtml({
                    packageName: pkg.name,
                    filePath,
                    openLabel: pkg.name + ' / ' + getFileName(filePath),
                    isPinned: true,
                    isRunning: running.has(filePath),
                }));
            }
        }

        dom.pinnedList.innerHTML = pinnedItems.length
            ? pinnedItems.join('')
            : '<li class="text-muted">No pinned launch files</li>';
    };

    const copyPackage = async (name) => {
        try {
            await navigator.clipboard.writeText(name);
            dom.pkgStatusEl.textContent = 'Copied "' + name + '"';
            setTimeout(() => {
                dom.pkgStatusEl.textContent = '';
            }, 1200);
        } catch {
            dom.pkgStatusEl.textContent = 'Failed to copy "' + name + '"';
        }
    };

    window.PM.render = {
        renderArgsOptions,
        renderConfigTabs,
        renderPackages,
        renderPinned,
        renderTerminals,
        copyPackage,
    };
})();
