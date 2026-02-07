/* Package Manager Webview Rendering */
(function () {
    window.PM = window.PM || {};

    const { dom, state, actions } = window.PM;

    let launchListEventsBound = false;
    let terminalEventsBound = false;
    const killFeedbackUntil = new Map();
    const KILL_FEEDBACK_MS = 1200;

    const getPkgList = () => document.getElementById('pkgList');
    const getSectionStateKey = (packageName, sectionKey) => {
        if (!packageName || !sectionKey) {
            return '';
        }
        return packageName + '::' + sectionKey;
    };
    const isPackageSectionExpanded = (packageName, sectionKey) => {
        const key = getSectionStateKey(packageName, sectionKey);
        if (!key) {
            return false;
        }
        return Boolean(state.expandedPackages?.[key]);
    };
    const setPackageSectionExpanded = (packageName, sectionKey, expanded) => {
        const key = getSectionStateKey(packageName, sectionKey);
        if (!key) {
            return;
        }
        if (!state.expandedPackages || typeof state.expandedPackages !== 'object') {
            state.expandedPackages = {};
        }
        state.expandedPackages[key] = expanded;
    };
    const togglePackageSectionExpanded = (packageName, sectionKey) => {
        setPackageSectionExpanded(packageName, sectionKey, !isPackageSectionExpanded(packageName, sectionKey));
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
            if (t.launchLabel) {
                running.add(t.launchLabel);
            }
        });
        return running;
    };

    const getLaunchItemData = (launchItem) => {
        if (!launchItem) {
            return { pkg: '', file: '', path: '', argsKey: '', pinKey: '' };
        }

        return {
            pkg: launchItem.dataset.pkg || '',
            file: launchItem.dataset.file || '',
            path: launchItem.dataset.path || '',
            argsKey: launchItem.dataset.argsKey || launchItem.dataset.path || '',
            pinKey: launchItem.dataset.pinKey || launchItem.dataset.path || '',
        };
    };

    const getNodeItemData = (nodeItem) => {
        if (!nodeItem) {
            return { pkg: '', executable: '', path: '', argsKey: '', pinKey: '' };
        }
        return {
            pkg: nodeItem.dataset.pkg || '',
            executable: nodeItem.dataset.node || '',
            path: nodeItem.dataset.path || '',
            argsKey: nodeItem.dataset.argsKey || '',
            pinKey: nodeItem.dataset.pinKey || nodeItem.dataset.argsKey || '',
        };
    };

    const launchFromItem = (launchItem, args, argsName) => {
        const { pkg, file, path } = getLaunchItemData(launchItem);
        if (!pkg || !file || !path) {
            return;
        }
        actions.launchFile(pkg, file, path, args || '', argsName || '');
    };

    const runNodeFromItem = (nodeItem, args, argsName) => {
        const { pkg, executable, path } = getNodeItemData(nodeItem);
        if (!pkg || !executable) {
            return;
        }
        actions.runNode(pkg, executable, args || '', argsName || '', path || '');
    };

    const openFromItem = (launchItem) => {
        const { path } = getLaunchItemData(launchItem);
        if (!path) {
            return;
        }
        actions.openLaunch(path);
    };

    const openNodeFromItem = (nodeItem) => {
        const { path } = getNodeItemData(nodeItem);
        if (!path) {
            return;
        }
        actions.openNode(path);
    };

    const togglePinFromItem = (launchItem) => {
        const { pinKey } = getLaunchItemData(launchItem);
        if (!pinKey) {
            return;
        }
        actions.togglePin(pinKey);
    };

    const togglePinFromNodeItem = (nodeItem) => {
        const { pinKey } = getNodeItemData(nodeItem);
        if (!pinKey) {
            return;
        }
        actions.togglePin(pinKey);
    };

    const openArgsFromItem = (argsKey, sourcePath) => {
        if (!argsKey) {
            return;
        }
        if (window.PM.handlers?.openArgsModal) {
            window.PM.handlers.openArgsModal(argsKey, sourcePath || '');
        }
    };

    const launchFromConfigButton = (configBtn, launchItem) => {
        const id = configBtn?.dataset?.id;
        const argsKey = configBtn?.dataset?.argsKey;

        if (!id || !argsKey) {
            return;
        }

        const cfg = state.launchArgConfigs[argsKey];
        const match = cfg?.configs?.find((c) => c.id === id);
        launchFromItem(launchItem, match?.args || '', match?.name || '');
    };

    const runNodeFromConfigButton = (configBtn, nodeItem) => {
        const id = configBtn?.dataset?.id;
        const argsKey = configBtn?.dataset?.argsKey;
        if (!id || !argsKey) {
            return;
        }
        const cfg = state.launchArgConfigs[argsKey];
        const match = cfg?.configs?.find((c) => c.id === id);
        runNodeFromItem(nodeItem, match?.args || '', match?.name || '');
    };

    const handleLaunchListClick = (event) => {
        const target = event.target;
        if (!(target instanceof Element)) {
            return;
        }

        const pkgToggleEl = target.closest('.pkg-toggle');
        if (pkgToggleEl) {
            togglePackageSectionExpanded(pkgToggleEl.dataset.pkgName || '', pkgToggleEl.dataset.sectionKey || '');
            return;
        }

        const pkgNameEl = target.closest('.pkg-name');
        if (pkgNameEl) {
            copyPackage(pkgNameEl.dataset.name);
            return;
        }

        const configBtn = target.closest('.config-pill');
        const launchItem = target.closest('.launch-item');
        const nodeItem = target.closest('.node-item');
        if (configBtn && launchItem) {
            event.stopPropagation();
            launchFromConfigButton(configBtn, launchItem);
            return;
        }
        if (configBtn && nodeItem) {
            event.stopPropagation();
            runNodeFromConfigButton(configBtn, nodeItem);
            return;
        }

        if (launchItem) {
            if (target.closest('.launch-run')) {
                launchFromItem(launchItem, '', '');
                return;
            }

            if (target.closest('.launch-open')) {
                openFromItem(launchItem);
                return;
            }

            if (target.closest('.args-btn')) {
                const { argsKey, path } = getLaunchItemData(launchItem);
                openArgsFromItem(argsKey, path);
                return;
            }

            if (target.closest('.pin-btn')) {
                togglePinFromItem(launchItem);
            }
            return;
        }

        if (nodeItem) {
            if (target.closest('.node-run')) {
                runNodeFromItem(nodeItem, '', '');
                return;
            }

            if (target.closest('.node-open') && !target.closest('.node-open.disabled')) {
                openNodeFromItem(nodeItem);
                return;
            }

            if (target.closest('.pin-btn')) {
                togglePinFromNodeItem(nodeItem);
                return;
            }

            if (target.closest('.args-btn')) {
                const { argsKey, path } = getNodeItemData(nodeItem);
                openArgsFromItem(argsKey, path || '');
            }
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
        if (launchItem) {
            if (target.closest('.launch-run')) {
                event.preventDefault();
                launchFromItem(launchItem, '', '');
                return;
            }

            if (target.closest('.launch-open')) {
                event.preventDefault();
                openFromItem(launchItem);
            }
            return;
        }

        const nodeItem = target.closest('.node-item');
        if (nodeItem) {
            if (target.closest('.node-run')) {
                event.preventDefault();
                runNodeFromItem(nodeItem, '', '');
                return;
            }
            if (target.closest('.node-open') && !target.closest('.node-open.disabled')) {
                event.preventDefault();
                openNodeFromItem(nodeItem);
            }
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

    const buildConfigButtonsHtml = (argsKey, cfg) => {
        if (!cfg || !cfg.configs || !cfg.configs.length) {
            return '';
        }

        return cfg.configs
            .map((c) => (
                '<button class="config-pill small" data-args-key="' +
                escapeAttr(argsKey) +
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
        const argsKey = filePath;
        const cfg = state.launchArgConfigs[argsKey];
        const configButtons = buildConfigButtonsHtml(argsKey, cfg);
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
            '" data-args-key="' +
            escapeAttr(argsKey) +
            '" data-pin-key="' +
            escapeAttr(filePath) +
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

    const buildNodeItemHtml = ({ packageName, nodeName, sourcePath, openLabel, isPinned, isRunning }) => {
        const argsKey = 'node::' + packageName + '::' + nodeName;
        const pinKey = argsKey;
        const cfg = state.launchArgConfigs[argsKey];
        const configButtons = buildConfigButtonsHtml(argsKey, cfg);
        const hasSourcePath = Boolean(sourcePath);
        const openClasses = hasSourcePath ? 'node-action node-open' : 'node-action node-open disabled';
        const openTitle = hasSourcePath ? 'Open node source' : 'No source path detected';
        const pinTitle = isPinned ? 'Unpin' : 'Pin';
        const runningBadge = isRunning ? '<span class="badge success">running</span>' : '';
        const runningClass = isRunning ? ' running' : '';

        return (
            '<li class="node-item' +
            runningClass +
            '" data-pkg="' +
            escapeAttr(packageName) +
            '" data-node="' +
            escapeAttr(nodeName) +
            '" data-path="' +
            escapeAttr(sourcePath || '') +
            '" data-args-key="' +
            escapeAttr(argsKey) +
            '" data-pin-key="' +
            escapeAttr(pinKey) +
            '">' +
            '<span class="node-action node-run" tabindex="0">Run</span>' +
            '<span class="' +
            openClasses +
            '" tabindex="0" title="' +
            escapeAttr(openTitle) +
            '">' +
            escapeHtml(openLabel || nodeName) +
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

    const buildPackageDropdownHtml = ({
        packageName,
        sectionKey,
        label,
        count,
        countLabel,
        bodyHtml,
        emptyText,
        expanded,
    }) => {
        const toggleTitle = expanded ? 'Collapse ' + label : 'Expand ' + label;
        const badgeText = count + ' ' + countLabel;
        const body = bodyHtml || ('<div class="text-muted text-sm">' + escapeHtml(emptyText) + '</div>');

        return (
            '<div class="pkg-dropdown">' +
            '<div class="pkg-dropdown-header">' +
            '<button class="pkg-toggle" data-pkg-name="' +
            escapeAttr(packageName) +
            '" data-section-key="' +
            escapeAttr(sectionKey) +
            '" aria-expanded="' +
            (expanded ? 'true' : 'false') +
            '" title="' +
            escapeAttr(toggleTitle) +
            '">' +
            (expanded ? '▾' : '▸') +
            '</button>' +
            '<span class="pkg-dropdown-title">' +
            escapeHtml(label) +
            '</span>' +
            '<span class="text-muted text-sm">' +
            escapeHtml(badgeText) +
            '</span>' +
            '</div>' +
            '<div class="pkg-dropdown-body' + (expanded ? '' : ' hidden') + '">' +
            body +
            '</div>' +
            '</div>'
        );
    };

    const buildLaunchDropdownModel = (pkg, runningLaunchPaths, filter) => {
        const launchFiles = pkg.launchFiles || [];
        const launchItemsHtml = launchFiles
            .map((filePath) => {
                const fileName = getFileName(filePath);
                return buildLaunchItemHtml({
                    packageName: pkg.name,
                    filePath,
                    openLabel: fileName,
                    isPinned: state.pinnedPaths.includes(filePath),
                    isRunning: runningLaunchPaths.has(filePath),
                });
            })
            .join('');

        return {
            sectionKey: 'launchFiles',
            label: 'Launch Files',
            count: launchFiles.length,
            countLabel: launchFiles.length === 1 ? 'file' : 'files',
            matchesFilter: !filter || launchFiles.some((filePath) => filePath.toLowerCase().includes(filter)),
            bodyHtml: launchItemsHtml ? '<ul class="launch-list">' + launchItemsHtml + '</ul>' : '',
            emptyText: 'No launch files detected',
        };
    };

    const buildNodesDropdownModel = (pkg, runningLaunchPaths, filter) => {
        const nodes = pkg.nodes || [];
        const nodeItemsHtml = nodes
            .map((node) => {
                const nodeLabel = pkg.name + ' / ' + node.name;
                return buildNodeItemHtml({
                    packageName: pkg.name,
                    nodeName: node.name,
                    sourcePath: node.sourcePath,
                    isPinned: state.pinnedPaths.includes('node::' + pkg.name + '::' + node.name),
                    isRunning: runningLaunchPaths.has(nodeLabel) || (node.sourcePath && runningLaunchPaths.has(node.sourcePath)),
                });
            })
            .join('');

        return {
            sectionKey: 'nodes',
            label: 'Nodes',
            count: nodes.length,
            countLabel: nodes.length === 1 ? 'node' : 'nodes',
            matchesFilter: !filter || nodes.some((node) => node.name.toLowerCase().includes(filter)),
            bodyHtml: nodeItemsHtml ? '<ul class="node-list">' + nodeItemsHtml + '</ul>' : '',
            emptyText: 'No runnable nodes detected',
        };
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
        const cfg = state.launchArgConfigs[state.currentArgsKey];
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
            const nodeMatch = (p.nodes || []).some((n) => (n.name || '').toLowerCase().includes(filter));
            return nameMatch || launchMatch || nodeMatch;
        });
        dom.pkgCount.textContent = String(filtered.length);

        if (filtered.length === 0) {
            list.innerHTML = '<li class="text-muted">No packages found</li>';
            return;
        }

        list.innerHTML = filtered
            .map((pkg) => {
                const launchCount = pkg.launchFiles?.length || 0;
                const nodeCount = pkg.nodes?.length || 0;
                const launchLabel = launchCount === 1 ? 'launch' : 'launches';
                const nodeLabel = nodeCount === 1 ? 'node' : 'nodes';
                const packageNameMatches = filter.length > 0 && pkg.name.toLowerCase().includes(filter);
                const dropdownModels = [
                    buildLaunchDropdownModel(pkg, running, filter),
                    buildNodesDropdownModel(pkg, running, filter),
                ];
                const dropdownHtml = dropdownModels
                    .map((section) => {
                        const expanded = (filter.length > 0 && (packageNameMatches || section.matchesFilter))
                            || isPackageSectionExpanded(pkg.name, section.sectionKey);
                        return buildPackageDropdownHtml({
                            packageName: pkg.name,
                            sectionKey: section.sectionKey,
                            label: section.label,
                            count: section.count,
                            countLabel: section.countLabel,
                            bodyHtml: section.bodyHtml,
                            emptyText: section.emptyText,
                            expanded,
                        });
                    })
                    .join('');

                return (
                    '<li class="pkg-row">' +
                    '<div class="pkg-header">' +
                    '<div class="pkg-main">' +
                    '<span class="pkg-name" tabindex="0" data-name="' +
                    escapeAttr(pkg.name) +
                    '">' +
                    escapeHtml(pkg.name) +
                    '</span>' +
                    '</div>' +
                    '<span class="text-muted text-sm">' +
                    launchCount +
                    ' ' +
                    launchLabel +
                    ' / ' +
                    nodeCount +
                    ' ' +
                    nodeLabel +
                    '</span>' +
                    '</div>' +
                    '<div class="pkg-dropdowns">' +
                    dropdownHtml +
                    '</div>' +
                    '</li>'
                );
            })
            .join('');
    };

    const renderPinned = () => {
        bindDelegatedEvents();

        if (!state.pinnedPaths.length) {
            dom.pinnedList.innerHTML = '<li class="text-muted">No pinned items</li>';
            return;
        }

        const running = getRunningLaunchPaths();
        const pinnedItems = [];

        for (const pinKey of state.pinnedPaths) {
            if (pinKey.startsWith('node::')) {
                const nodeMatch = state.allPackages
                    .flatMap((pkg) => (pkg.nodes || []).map((node) => ({ pkg, node })))
                    .find(({ pkg, node }) => ('node::' + pkg.name + '::' + node.name) === pinKey);
                if (!nodeMatch) {
                    continue;
                }
                const nodeLabel = nodeMatch.pkg.name + ' / ' + nodeMatch.node.name;
                pinnedItems.push(buildNodeItemHtml({
                    packageName: nodeMatch.pkg.name,
                    nodeName: nodeMatch.node.name,
                    sourcePath: nodeMatch.node.sourcePath,
                    openLabel: nodeLabel,
                    isPinned: true,
                    isRunning: running.has(nodeLabel) || (nodeMatch.node.sourcePath && running.has(nodeMatch.node.sourcePath)),
                }));
                continue;
            }

            const launchMatch = state.allPackages
                .flatMap((pkg) => (pkg.launchFiles || []).map((filePath) => ({ pkg, filePath })))
                .find(({ filePath }) => filePath === pinKey);
            if (!launchMatch) {
                continue;
            }
            pinnedItems.push(buildLaunchItemHtml({
                packageName: launchMatch.pkg.name,
                filePath: launchMatch.filePath,
                openLabel: launchMatch.pkg.name + ' / ' + getFileName(launchMatch.filePath),
                isPinned: true,
                isRunning: running.has(launchMatch.filePath),
            }));
        }

        dom.pinnedList.innerHTML = pinnedItems.length
            ? pinnedItems.join('')
            : '<li class="text-muted">No pinned items</li>';
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
