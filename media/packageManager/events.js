/* Package Manager Webview Events */
(function () {
    window.PM = window.PM || {};

    const { dom, state, actions, render } = window.PM;
    const uiInteractions = window.RosUi?.interactions;

    const defaultDepsByBuildType = {
        ament_cmake: 'rclcpp std_msgs',
        ament_python: 'rclpy std_msgs',
    };
    const defaultPackageDescription = 'TO DO: A very good package description';
    const ARGS_AUTOSAVE_DELAY_MS = 180;
    let argsAutosaveTimer = 0;
    let argsSummaryRefreshPending = false;

    const getDefaultDeps = (buildType) => defaultDepsByBuildType[buildType] || '';
    const bindRecursiveToggleTrigger = (element, onToggle, onRecursiveToggle) => {
        if (!element) {
            return;
        }

        if (uiInteractions?.bindRecursiveToggleClick) {
            uiInteractions.bindRecursiveToggleClick(
                element,
                onToggle,
                onRecursiveToggle,
            );
        } else {
            element.addEventListener('click', (event) => {
                event.preventDefault();
                if (event?.altKey) {
                    onRecursiveToggle();
                    return;
                }
                onToggle();
            });
        }

        if (element instanceof HTMLElement) {
            element.addEventListener('keydown', (event) => {
                if (event.key !== 'Enter' && event.key !== ' ') {
                    return;
                }
                event.preventDefault();
                onToggle();
            });
        }
    };
    const bindModalEscapeClose = (modal, onClose) => {
        if (!uiInteractions?.bindModalEscapeClose) {
            if (!(modal instanceof Element) || typeof onClose !== 'function') {
                return;
            }
            modal.addEventListener('keydown', (event) => {
                if (!(event instanceof KeyboardEvent)) {
                    return;
                }
                if (event.key !== 'Escape' || event.defaultPrevented || event.isComposing) {
                    return;
                }
                if (event.altKey || event.ctrlKey || event.metaKey || event.shiftKey) {
                    return;
                }
                if (modal.classList.contains('hidden')) {
                    return;
                }
                event.preventDefault();
                onClose(event);
            });
            return;
        }
        uiInteractions.bindModalEscapeClose({
            modal,
            onClose,
        });
    };

    const normalizePackageName = (rawName) => String(rawName || '').trim().replace(/\s+/g, '_');
    const normalizeNodeName = (rawName) => String(rawName || '').trim().replace(/\s+/g, '_');
    const normalizeNodeTopic = (rawTopic) => String(rawTopic || '').trim().replace(/\s+/g, '_');
    const normalizeLaunchName = (rawName) => String(rawName || '').trim().replace(/\s+/g, '_');
    const nodeTemplateTopicSuggestions = Object.freeze({
        publisher: 'chatter',
        subscriber: 'chatter',
    });

    const updatePackageNamePreview = () => {
        if (!dom.pkgNameInput || !dom.pkgNameNormalizedHint) {
            return;
        }

        const rawName = String(dom.pkgNameInput.value || '');
        const normalizedName = normalizePackageName(rawName);

        if (!normalizedName) {
            dom.pkgNameNormalizedHint.textContent = 'Spaces are converted to "_" in package names.';
            dom.pkgNameNormalizedHint.classList.remove('hidden');
            return;
        }

        if (rawName.trim() !== normalizedName) {
            dom.pkgNameNormalizedHint.textContent =
                'Spaces are converted to "_". Package will be created as: ' + normalizedName;
            dom.pkgNameNormalizedHint.classList.remove('hidden');
            return;
        }

        dom.pkgNameNormalizedHint.textContent = 'Package will be created as: ' + normalizedName;
        dom.pkgNameNormalizedHint.classList.remove('hidden');
    };

    const updateNodeNamePreview = () => {
        if (!dom.addNodeName || !dom.addNodeNameNormalizedHint) {
            return;
        }

        const rawName = String(dom.addNodeName.value || '');
        const normalizedName = normalizeNodeName(rawName);

        if (!normalizedName) {
            dom.addNodeNameNormalizedHint.textContent = 'Spaces are converted to "_" in node names.';
            dom.addNodeNameNormalizedHint.classList.remove('hidden');
            return;
        }

        if (rawName.trim() !== normalizedName) {
            dom.addNodeNameNormalizedHint.textContent =
                'Spaces are converted to "_". Node will be created as: ' + normalizedName;
            dom.addNodeNameNormalizedHint.classList.remove('hidden');
            return;
        }

        dom.addNodeNameNormalizedHint.textContent = 'Node will be created as: ' + normalizedName;
        dom.addNodeNameNormalizedHint.classList.remove('hidden');
    };

    const updateNodeTopicPreview = () => {
        if (!dom.addNodeTopic || !dom.addNodeTopicNormalizedHint) {
            return;
        }

        const rawTopic = String(dom.addNodeTopic.value || '');
        const normalizedTopic = normalizeNodeTopic(rawTopic);

        if (!normalizedTopic) {
            dom.addNodeTopicNormalizedHint.textContent = 'Spaces are converted to "_" in topic names.';
            dom.addNodeTopicNormalizedHint.classList.remove('hidden');
            return;
        }

        if (rawTopic.trim() !== normalizedTopic) {
            dom.addNodeTopicNormalizedHint.textContent =
                'Spaces are converted to "_". Topic will be created as: ' + normalizedTopic;
            dom.addNodeTopicNormalizedHint.classList.remove('hidden');
            return;
        }

        dom.addNodeTopicNormalizedHint.textContent = 'Topic will be created as: ' + normalizedTopic;
        dom.addNodeTopicNormalizedHint.classList.remove('hidden');
    };

    const updateLaunchNamePreview = () => {
        if (!dom.createLaunchName || !dom.createLaunchNameNormalizedHint) {
            return;
        }

        const rawName = String(dom.createLaunchName.value || '');
        const normalizedName = normalizeLaunchName(rawName);

        if (!normalizedName) {
            dom.createLaunchNameNormalizedHint.textContent = 'Spaces are converted to "_" in launch file names.';
            dom.createLaunchNameNormalizedHint.classList.remove('hidden');
            return;
        }

        if (rawName.trim() !== normalizedName) {
            dom.createLaunchNameNormalizedHint.textContent =
                'Spaces are converted to "_". Launch will be created as: ' + normalizedName;
            dom.createLaunchNameNormalizedHint.classList.remove('hidden');
            return;
        }

        dom.createLaunchNameNormalizedHint.textContent = 'Launch will be created as: ' + normalizedName;
        dom.createLaunchNameNormalizedHint.classList.remove('hidden');
    };

    const updateNodeTemplateFields = (forceSuggestion = false) => {
        const template = String(dom.addNodeTemplate?.value || 'none').toLowerCase();
        const suggestedTopic = nodeTemplateTopicSuggestions[template] || '';
        const requiresTopic = Boolean(suggestedTopic);
        const rememberedTopic = normalizeNodeTopic(state.lastNodeTopic || '');

        if (dom.addNodeTopicRow) {
            dom.addNodeTopicRow.classList.toggle('hidden', !requiresTopic);
        }
        if (dom.addNodeTopicNormalizedHint) {
            dom.addNodeTopicNormalizedHint.classList.toggle('hidden', !requiresTopic);
        }
        if (!dom.addNodeTopic) {
            return;
        }

        dom.addNodeTopic.placeholder = suggestedTopic || 'chatter';
        const currentValue = normalizeNodeTopic(dom.addNodeTopic.value || '');
        if (requiresTopic) {
            if (forceSuggestion || !currentValue) {
                dom.addNodeTopic.value = rememberedTopic || suggestedTopic;
            } else {
                dom.addNodeTopic.value = currentValue;
            }
            updateNodeTopicPreview();
        } else if (forceSuggestion) {
            dom.addNodeTopic.value = '';
        }
    };

    const applyDefaultDepsForBuildType = (force = false) => {
        if (!dom.buildTypeInput || !dom.depsInput) {
            return;
        }
        if (force || !dom.depsInput.value.trim()) {
            dom.depsInput.value = getDefaultDeps(dom.buildTypeInput.value);
        }
    };

    const openCreate = () => {
        dom.createModal.classList.remove('hidden');
        dom.statusEl.className = 'mt hidden';
        dom.statusEl.textContent = '';
        if (dom.descriptionInput) {
            dom.descriptionInput.value = defaultPackageDescription;
        }
        if (dom.licenseInput) {
            dom.licenseInput.value = 'GPL-3.0';
        }
        applyDefaultDepsForBuildType(true);
        updatePackageNamePreview();
        if (dom.pkgNameInput) {
            dom.pkgNameInput.focus();
        }
    };

    const closeCreate = () => {
        dom.createModal.classList.add('hidden');
    };

    const sanitizeArgsText = (value) => String(value ?? '').replace(/\r?\n+/g, ' ');

    const autoSizeConfigArgsInput = () => {
        render.autoSizeArgsInput?.(dom.configArgsInput);
    };

    const setCurrentArgsText = (value, options = {}) => {
        const nextValue = sanitizeArgsText(value);
        state.currentArgsText = nextValue;
        if (!(dom.configArgsInput instanceof HTMLTextAreaElement)) {
            return;
        }
        dom.configArgsInput.value = nextValue;
        autoSizeConfigArgsInput();
        if (options.focusEnd) {
            const length = dom.configArgsInput.value.length;
            dom.configArgsInput.focus();
            dom.configArgsInput.setSelectionRange(length, length);
        }
    };

    const appendArgEntry = (value, options = {}) => {
        const nextValue = String(value || '').trim();
        if (!nextValue) {
            return;
        }
        const currentValue = String(state.currentArgsText || '');
        const separator = currentValue && !/\s$/.test(currentValue) ? ' ' : '';
        setCurrentArgsText(currentValue ? currentValue + separator + nextValue : nextValue, {
            focusEnd: options.focusEnd !== false,
        });
        queueArgsAutosave(options.immediate ? { immediate: true } : {});
    };

    const openArgsModal = (argsKey, sourcePath) => {
        if (!argsKey) {
            return;
        }
        if (argsAutosaveTimer) {
            window.clearTimeout(argsAutosaveTimer);
            argsAutosaveTimer = 0;
        }
        argsSummaryRefreshPending = false;
        state.currentArgsKey = argsKey;
        const cfg = state.launchArgConfigs[argsKey] || {
            configs: [{ id: 'default', name: 'default', args: '', runTarget: 'auto' }],
        };
        if (!Array.isArray(cfg.configs) || !cfg.configs.length) {
            cfg.configs = [{ id: 'default', name: 'default', args: '', runTarget: 'auto' }];
        }
        cfg.configs = cfg.configs.map((config, index) => {
            const id = String(config?.id || `cfg-${index + 1}`).trim() || `cfg-${index + 1}`;
            const name = String(config?.name || '').trim() || (id === 'default' ? 'default' : 'config');
            return {
                id,
                name,
                args: String(config?.args || ''),
                runTarget: normalizeRunTarget(config?.runTarget || 'auto'),
            };
        });
        state.launchArgConfigs[argsKey] = cfg;
        state.currentConfigId = cfg.configs[0]?.id || 'default';
        syncArgsConfigEditor();
        state.argsOptions = [];
        render.renderArgsOptions();
        render.renderConfigTabs();
        dom.argsModal.classList.remove('hidden');
        window.requestAnimationFrame(() => {
            autoSizeConfigArgsInput();
            if (dom.configArgsInput instanceof HTMLTextAreaElement) {
                const length = dom.configArgsInput.value.length;
                dom.configArgsInput.focus();
                dom.configArgsInput.setSelectionRange(length, length);
            }
        });
        if (state.currentArgsKey) {
            actions.requestLaunchArgs(state.currentArgsKey, sourcePath || '');
        }
    };

    const renderArgsConfigSummaries = () => {
        render.renderPackages();
        render.renderOtherPackages();
        render.renderPinned();
    };

    const clearArgsAutosaveTimer = () => {
        if (!argsAutosaveTimer) {
            return false;
        }
        window.clearTimeout(argsAutosaveTimer);
        argsAutosaveTimer = 0;
        return true;
    };

    const persistLaunchArgConfigs = (argsKey) => {
        const normalizedArgsKey = String(argsKey || '').trim();
        if (!normalizedArgsKey) {
            return;
        }
        const cfg = state.launchArgConfigs[normalizedArgsKey];
        if (!cfg || !Array.isArray(cfg.configs)) {
            return;
        }
        actions.setLaunchArgConfigs(normalizedArgsKey, cfg.configs.map((config, index) => {
            const id = String(config?.id || `cfg-${index + 1}`).trim() || `cfg-${index + 1}`;
            const normalizedName = String(config?.name || '').trim() || (id === 'default' ? 'default' : 'config');
            return {
                id,
                name: normalizedName,
                args: String(config?.args || ''),
                runTarget: normalizeConfigRunTarget(config?.runTarget || 'auto'),
            };
        }));
    };

    const updateCurrentArgsConfigDraft = () => {
        const argsKey = String(state.currentArgsKey || '').trim();
        if (!argsKey) {
            return { argsKey: '', changed: false };
        }
        const cfg = state.launchArgConfigs[argsKey];
        if (!cfg || !Array.isArray(cfg.configs) || !cfg.configs.length) {
            return { argsKey, changed: false };
        }

        const currentCfg = getCurrentArgsConfig();
        if (!currentCfg) {
            return { argsKey, changed: false };
        }

        const configIndex = cfg.configs.findIndex((config) => config.id === currentCfg.id);
        if (configIndex < 0) {
            return { argsKey, changed: false };
        }

        const normalizedCurrentName = String(currentCfg.name || '').trim() || (currentCfg.id === 'default' ? 'default' : 'config');
        const normalizedCurrentArgs = String(currentCfg.args || '');
        const normalizedCurrentRunTarget = normalizeConfigRunTarget(currentCfg.runTarget || 'auto');

        const nextName = String(dom.configName?.value ?? currentCfg.name ?? '').trim()
            || (currentCfg.id === 'default' ? 'default' : 'config');
        const nextArgs = String(state.currentArgsText || '').trim();
        const nextRunTarget = normalizeConfigRunTarget(dom.configRunTarget?.value || currentCfg.runTarget || 'auto');

        const changed = normalizedCurrentName !== nextName
            || normalizedCurrentArgs !== nextArgs
            || normalizedCurrentRunTarget !== nextRunTarget;

        if (!changed) {
            return { argsKey, changed: false };
        }

        cfg.configs[configIndex] = {
            ...currentCfg,
            name: nextName,
            args: nextArgs,
            runTarget: nextRunTarget,
        };
        state.launchArgConfigs[argsKey] = cfg;

        return {
            argsKey,
            changed: true,
            summaryChanged: normalizedCurrentName !== nextName,
        };
    };

    const flushArgsAutosave = (options = {}) => {
        const draft = updateCurrentArgsConfigDraft();
        if (options.refreshConfigTabs && draft.changed) {
            render.renderConfigTabs();
        }
        if (options.refreshConfigSummaries && draft.changed) {
            argsSummaryRefreshPending = true;
        }

        const hadPendingAutosave = clearArgsAutosaveTimer();
        const argsKey = String(draft.argsKey || state.currentArgsKey || '').trim();
        if (!argsKey || (!draft.changed && !hadPendingAutosave && !argsSummaryRefreshPending)) {
            return;
        }

        if (argsSummaryRefreshPending) {
            renderArgsConfigSummaries();
            argsSummaryRefreshPending = false;
        }
        persistLaunchArgConfigs(argsKey);
    };

    const queueArgsAutosave = (options = {}) => {
        const draft = updateCurrentArgsConfigDraft();
        if (options.refreshConfigTabs && draft.changed) {
            render.renderConfigTabs();
        }
        if (options.refreshConfigSummaries && draft.changed) {
            argsSummaryRefreshPending = true;
        }

        const argsKey = String(draft.argsKey || state.currentArgsKey || '').trim();
        const hadPendingAutosave = clearArgsAutosaveTimer();
        if (!argsKey || (!draft.changed && !hadPendingAutosave)) {
            return;
        }

        if (options.immediate) {
            if (argsSummaryRefreshPending) {
                renderArgsConfigSummaries();
                argsSummaryRefreshPending = false;
            }
            persistLaunchArgConfigs(argsKey);
            return;
        }

        argsAutosaveTimer = window.setTimeout(() => {
            argsAutosaveTimer = 0;
            if (argsSummaryRefreshPending) {
                renderArgsConfigSummaries();
                argsSummaryRefreshPending = false;
            }
            persistLaunchArgConfigs(argsKey);
        }, ARGS_AUTOSAVE_DELAY_MS);
    };

    const closeArgsModal = (options = {}) => {
        if (options.persist !== false) {
            flushArgsAutosave({ refreshConfigSummaries: true });
        }
        dom.argsModal.classList.add('hidden');
    };

    const normalizeRunTarget = (value) => {
        const normalized = String(value || '').trim() || 'auto';
        if (normalized === 'wsl:default') {
            return 'wsl-external:default';
        }
        if (normalized.startsWith('wsl:')) {
            return 'wsl-external:' + normalized.slice('wsl:'.length);
        }
        return normalized;
    };

    const parseRunTargetToSelection = (target) => {
        const normalized = normalizeRunTarget(target);
        if (normalized === 'auto') {
            return { environmentId: 'auto', terminalModeId: 'auto' };
        }
        if (normalized === 'integrated' || normalized === 'external') {
            return { environmentId: 'host', terminalModeId: normalized };
        }
        if (normalized.startsWith('wsl-integrated:')) {
            const distro = String(normalized.slice('wsl-integrated:'.length) || 'default').trim() || 'default';
            return { environmentId: 'wsl:' + distro, terminalModeId: 'integrated' };
        }
        if (normalized.startsWith('wsl-external:')) {
            const distro = String(normalized.slice('wsl-external:'.length) || 'default').trim() || 'default';
            return { environmentId: 'wsl:' + distro, terminalModeId: 'external' };
        }
        return { environmentId: 'auto', terminalModeId: 'auto' };
    };

    const getEnvironmentLabel = (environmentId) => {
        if (environmentId === 'auto') {
            return 'Auto';
        }
        if (environmentId === 'host') {
            return 'Current environment';
        }
        if (environmentId === 'wsl:default') {
            return 'WSL (default distro)';
        }
        if (environmentId.startsWith('wsl:')) {
            const distro = String(environmentId.slice('wsl:'.length) || '').trim();
            return distro ? ('WSL (' + distro + ')') : 'WSL';
        }
        return environmentId;
    };

    const getEnvironmentDescription = (environmentId) => {
        if (environmentId === 'auto') {
            return 'Use the extension default environment and terminal behavior.';
        }
        if (environmentId === 'host') {
            return 'Run commands in the current workspace environment.';
        }
        if (environmentId === 'wsl:default') {
            return 'Run commands in the default WSL distro.';
        }
        if (environmentId.startsWith('wsl:')) {
            const distro = String(environmentId.slice('wsl:'.length) || '').trim();
            return distro
                ? ('Run commands in WSL distro "' + distro + '".')
                : 'Run commands in WSL.';
        }
        return '';
    };

    const getTerminalModeLabel = (terminalModeId) => {
        if (terminalModeId === 'auto') {
            return 'Auto';
        }
        if (terminalModeId === 'integrated') {
            return 'VS Code terminal';
        }
        if (terminalModeId === 'external') {
            return 'External terminal';
        }
        return terminalModeId;
    };

    const getTerminalModeDescription = (terminalModeId) => {
        if (terminalModeId === 'auto') {
            return 'Use rosDevToolkit.launchInExternalTerminal to decide where Run actions open.';
        }
        if (terminalModeId === 'integrated') {
            return 'Always open Run actions in a VS Code terminal tab.';
        }
        if (terminalModeId === 'external') {
            return 'Always open Run actions in a native external terminal window.';
        }
        return '';
    };

    const environmentSortRank = (environmentId) => {
        if (environmentId === 'auto') {
            return 0;
        }
        if (environmentId === 'host') {
            return 1;
        }
        if (environmentId === 'wsl:default') {
            return 2;
        }
        if (environmentId.startsWith('wsl:')) {
            return 3;
        }
        return 4;
    };

    const terminalModeSortRank = (terminalModeId) => {
        if (terminalModeId === 'auto') {
            return 0;
        }
        if (terminalModeId === 'integrated') {
            return 1;
        }
        if (terminalModeId === 'external') {
            return 2;
        }
        return 3;
    };

    const buildRunTargetSelectionModel = (requestedEnvironmentId, requestedTerminalModeId) => {
        const rawOptions = Array.isArray(state.runTerminalTargetOptions)
            ? state.runTerminalTargetOptions
            : [];
        const optionIds = rawOptions
            .map((opt) => normalizeRunTarget(opt?.id))
            .filter((id) => Boolean(id));
        const validTargets = new Set(optionIds.length ? optionIds : ['auto']);

        const environmentMap = new Map();
        const registerPair = (targetId) => {
            const selection = parseRunTargetToSelection(targetId);
            const envId = String(selection.environmentId || 'auto').trim() || 'auto';
            const modeId = String(selection.terminalModeId || 'auto').trim() || 'auto';
            if (!environmentMap.has(envId)) {
                environmentMap.set(envId, {
                    id: envId,
                    terminalModeToTarget: new Map(),
                });
            }
            const envEntry = environmentMap.get(envId);
            if (!envEntry.terminalModeToTarget.has(modeId)) {
                envEntry.terminalModeToTarget.set(modeId, targetId);
            }
        };

        validTargets.forEach(registerPair);
        if (!environmentMap.size) {
            registerPair('auto');
        }

        const environmentOptions = Array.from(environmentMap.values())
            .map((env) => ({
                id: env.id,
                label: getEnvironmentLabel(env.id),
                description: getEnvironmentDescription(env.id),
            }))
            .sort((a, b) => {
                const rankDiff = environmentSortRank(a.id) - environmentSortRank(b.id);
                return rankDiff !== 0 ? rankDiff : a.label.localeCompare(b.label);
            });

        const normalizedCurrentTarget = normalizeRunTarget(state.runTerminalTarget);
        const currentSelection = parseRunTargetToSelection(
            validTargets.has(normalizedCurrentTarget) ? normalizedCurrentTarget : 'auto',
        );
        const requestedEnvironment = String(requestedEnvironmentId || '').trim();
        const requestedTerminalMode = String(requestedTerminalModeId || '').trim();

        const selectedEnvironmentId = environmentMap.has(requestedEnvironment)
            ? requestedEnvironment
            : (
                environmentMap.has(currentSelection.environmentId)
                    ? currentSelection.environmentId
                    : (environmentOptions[0]?.id || 'auto')
            );

        const terminalModeToTarget = environmentMap.get(selectedEnvironmentId)?.terminalModeToTarget || new Map([
            ['auto', 'auto'],
        ]);
        const terminalModeOptions = Array.from(terminalModeToTarget.keys())
            .map((id) => ({
                id,
                label: getTerminalModeLabel(id),
                description: getTerminalModeDescription(id),
            }))
            .sort((a, b) => {
                const rankDiff = terminalModeSortRank(a.id) - terminalModeSortRank(b.id);
                return rankDiff !== 0 ? rankDiff : a.label.localeCompare(b.label);
            });

        const selectedTerminalModeId = terminalModeToTarget.has(requestedTerminalMode)
            ? requestedTerminalMode
            : (
                terminalModeToTarget.has(currentSelection.terminalModeId)
                    ? currentSelection.terminalModeId
                    : (terminalModeOptions[0]?.id || 'auto')
            );

        const resolvedTarget = normalizeRunTarget(
            terminalModeToTarget.get(selectedTerminalModeId)
            || 'auto',
        );

        return {
            environmentOptions,
            terminalModeOptions,
            selectedEnvironmentId,
            selectedTerminalModeId,
            resolvedTarget: validTargets.has(resolvedTarget) ? resolvedTarget : 'auto',
        };
    };

    const getConfigRunTargetOptions = () => {
        const optionsById = new Map([
            [
                'auto',
                {
                    id: 'auto',
                    label: 'Auto',
                    description: 'Use the Environment Info run target for this configuration.',
                },
            ],
        ]);

        const rawOptions = Array.isArray(state.runTerminalTargetOptions)
            ? state.runTerminalTargetOptions
            : [];
        rawOptions.forEach((opt) => {
            const id = normalizeRunTarget(opt?.id);
            if (!id) {
                return;
            }
            const label = String(opt?.label || id);
            const description = String(opt?.description || '');
            const entry = { id, label, description };
            if (id === 'auto') {
                optionsById.set(id, entry);
                return;
            }
            if (!optionsById.has(id)) {
                optionsById.set(id, entry);
            }
        });

        return Array.from(optionsById.values());
    };

    const normalizeConfigRunTarget = (value) => {
        const normalized = normalizeRunTarget(value);
        const validTargets = new Set(getConfigRunTargetOptions().map((opt) => String(opt.id || '').trim()).filter(Boolean));
        return validTargets.has(normalized) ? normalized : 'auto';
    };

    const renderConfigRunTargetState = (requestedTargetId) => {
        if (!dom.configRunTarget) {
            return 'auto';
        }
        const options = getConfigRunTargetOptions();
        const selectedTargetId = normalizeConfigRunTarget(requestedTargetId);
        dom.configRunTarget.innerHTML = options
            .map((opt) => (
                '<option value="' + String(opt.id).replace(/"/g, '&quot;') + '">' + String(opt.label) + '</option>'
            ))
            .join('');
        dom.configRunTarget.value = selectedTargetId;
        dom.configRunTarget.disabled = options.length <= 1;

        const selected = options.find((opt) => String(opt.id) === selectedTargetId);
        if (dom.configRunTargetDescription) {
            dom.configRunTargetDescription.textContent = String(selected?.description || '');
        }
        return selectedTargetId;
    };

    const getCurrentArgsConfig = () => {
        const cfg = state.launchArgConfigs[state.currentArgsKey];
        if (!cfg || !Array.isArray(cfg.configs) || !cfg.configs.length) {
            return undefined;
        }
        const selected = cfg.configs.find((c) => c.id === state.currentConfigId) || cfg.configs[0];
        if (selected && selected.id !== state.currentConfigId) {
            state.currentConfigId = selected.id;
        }
        if (selected) {
            selected.runTarget = normalizeConfigRunTarget(selected.runTarget || 'auto');
        }
        return selected;
    };

    const syncArgsConfigEditor = () => {
        const currentCfg = getCurrentArgsConfig();
        setCurrentArgsText(currentCfg?.args || '');
        if (dom.configName) {
            dom.configName.value = currentCfg?.name || '';
        }
        renderConfigRunTargetState(currentCfg?.runTarget || 'auto');
    };

    const getSelectedRunTarget = () => {
        const model = buildRunTargetSelectionModel(
            dom.runEnvironmentTarget?.value || state.runEnvironmentTarget,
            dom.runTerminalMode?.value || state.runTerminalMode,
        );
        return model.resolvedTarget;
    };

    const renderEnvironmentDetailsState = () => {
        const expanded = state.environmentDetailsExpanded === true;
        if (dom.environmentDetailsContainer) {
            dom.environmentDetailsContainer.classList.toggle('hidden', !expanded);
        }
        if (dom.environmentDetailsToggle) {
            const arrow = expanded ? '▼' : '▶';
            const loadingSuffix = state.environmentLoading ? ' (loading...)' : '';
            dom.environmentDetailsToggle.textContent = arrow + ' Detected environment' + loadingSuffix;
            dom.environmentDetailsToggle.setAttribute('aria-expanded', expanded ? 'true' : 'false');
        }
        if (dom.environmentLoadingState) {
            dom.environmentLoadingState.classList.toggle('hidden', state.environmentLoading !== true);
        }
    };

    const renderEnvironmentModalState = (requestedEnvironmentId, requestedTerminalModeId) => {
        if (!dom.environmentDetails || !dom.runEnvironmentTarget || !dom.runTerminalMode) {
            return;
        }

        const report = String(state.environmentReport || '').trim();
        dom.environmentDetails.textContent = report || 'Collecting environment details...';

        const model = buildRunTargetSelectionModel(requestedEnvironmentId, requestedTerminalModeId);

        state.runEnvironmentOptions = model.environmentOptions;
        state.runTerminalModeOptions = model.terminalModeOptions;
        state.runEnvironmentTarget = model.selectedEnvironmentId;
        state.runTerminalMode = model.selectedTerminalModeId;

        dom.runEnvironmentTarget.innerHTML = model.environmentOptions
            .map((opt) => (
                '<option value="' + String(opt.id).replace(/"/g, '&quot;') + '">' + String(opt.label) + '</option>'
            ))
            .join('');
        dom.runEnvironmentTarget.value = model.selectedEnvironmentId;

        dom.runTerminalMode.innerHTML = model.terminalModeOptions
            .map((opt) => (
                '<option value="' + String(opt.id).replace(/"/g, '&quot;') + '">' + String(opt.label) + '</option>'
            ))
            .join('');
        dom.runTerminalMode.value = model.selectedTerminalModeId;
        dom.runTerminalMode.disabled = model.terminalModeOptions.length <= 1;

        const selectedEnvironment = model.environmentOptions.find(
            (opt) => String(opt?.id || '') === model.selectedEnvironmentId,
        );
        if (dom.runEnvironmentDescription) {
            dom.runEnvironmentDescription.textContent = String(selectedEnvironment?.description || '');
        }

        const selectedMode = model.terminalModeOptions.find(
            (opt) => String(opt?.id || '') === model.selectedTerminalModeId,
        );
        if (dom.runTerminalModeDescription) {
            dom.runTerminalModeDescription.textContent = String(selectedMode?.description || '');
        }

        if (dom.argsModal && !dom.argsModal.classList.contains('hidden')) {
            const requestedConfigTarget = dom.configRunTarget?.value
                || getCurrentArgsConfig()?.runTarget
                || 'auto';
            renderConfigRunTargetState(requestedConfigTarget);
        }

        renderEnvironmentDetailsState();

    };

    const openEnvironmentModal = () => {
        if (!dom.environmentModal) {
            return;
        }
        state.environmentDetailsExpanded = false;
        renderEnvironmentModalState();
        dom.environmentModal.classList.remove('hidden');
        if (dom.runEnvironmentTarget) {
            dom.runEnvironmentTarget.focus();
        }
    };

    const closeEnvironmentModal = () => {
        if (!dom.environmentModal) {
            return;
        }
        dom.environmentModal.classList.add('hidden');
    };

    const renderBuildCheckSettingsState = () => {
        const globalEnabled = Boolean(dom.toggleBuildCheck?.checked);
        if (dom.buildCheckScopeSettings) {
            dom.buildCheckScopeSettings.classList.toggle('disabled', !globalEnabled);
            dom.buildCheckScopeSettings.setAttribute('aria-disabled', globalEnabled ? 'false' : 'true');
        }
        if (dom.toggleBuildCheckLaunchFiles) {
            dom.toggleBuildCheckLaunchFiles.disabled = !globalEnabled;
        }
        if (dom.toggleBuildCheckRunNodes) {
            dom.toggleBuildCheckRunNodes.disabled = !globalEnabled;
        }
    };

    const openBuildCheckSettingsModal = () => {
        if (!dom.buildCheckSettingsModal) {
            return;
        }
        renderBuildCheckSettingsState();
        dom.buildCheckSettingsModal.classList.remove('hidden');
        if (dom.toggleBuildCheck) {
            dom.toggleBuildCheck.focus();
        }
    };

    const closeBuildCheckSettingsModal = () => {
        if (!dom.buildCheckSettingsModal) {
            return;
        }
        dom.buildCheckSettingsModal.classList.add('hidden');
    };

    const submitBuildCheckSettings = () => {
        const enabled = Boolean(dom.toggleBuildCheck?.checked);
        const launchFile = Boolean(dom.toggleBuildCheckLaunchFiles?.checked);
        const runNode = Boolean(dom.toggleBuildCheckRunNodes?.checked);
        actions.toggleBuildCheck(enabled);
        actions.setBuildCheckScopes({ launchFile, runNode });
        closeBuildCheckSettingsModal();
    };

    const submitCreatePackage = () => {
        const rawName = dom.pkgNameInput ? dom.pkgNameInput.value : '';
        const name = normalizePackageName(rawName);
        const buildType = dom.buildTypeInput ? dom.buildTypeInput.value : 'ament_cmake';
        const deps = dom.depsInput ? dom.depsInput.value.trim() : '';
        const license = dom.licenseInput ? String(dom.licenseInput.value || '').trim() : 'GPL-3.0';
        const description = dom.descriptionInput
            ? String(dom.descriptionInput.value || '').trim()
            : defaultPackageDescription;
        if (!name) {
            return;
        }
        if (dom.pkgNameInput) {
            dom.pkgNameInput.value = name;
        }
        updatePackageNamePreview();
        dom.btnCreate.disabled = true;
        dom.statusEl.className = 'mt';
        dom.statusEl.innerHTML = '<span class="spinner"></span> Creating…';
        actions.createPackage(
            name,
            buildType,
            deps,
            license || 'GPL-3.0',
            description || defaultPackageDescription,
        );
    };

    dom.btnCreate.addEventListener('click', submitCreatePackage);

    dom.btnOpenCreate.addEventListener('click', openCreate);
    dom.btnCloseCreate.addEventListener('click', closeCreate);
    dom.btnCancelCreate.addEventListener('click', closeCreate);
    dom.createBackdrop.addEventListener('click', closeCreate);
    if (dom.pkgNameInput) {
        dom.pkgNameInput.addEventListener('input', updatePackageNamePreview);
    }
    if (dom.buildTypeInput) {
        dom.buildTypeInput.addEventListener('change', () => applyDefaultDepsForBuildType(true));
    }
    if (uiInteractions?.bindModalEnterConfirm) {
        uiInteractions.bindModalEnterConfirm({
            modal: dom.createModal,
            confirmButton: dom.btnCreate,
        });
    }
    bindModalEscapeClose(dom.createModal, closeCreate);

    dom.btnRefresh.addEventListener('click', () => {
        actions.refreshPackages();
    });
    if (dom.btnShowEnvironment) {
        dom.btnShowEnvironment.addEventListener('click', () => {
            state.environmentLoading = true;
            openEnvironmentModal();
            actions.showEnvironmentInfo();
        });
    }
    if (dom.btnOpenBuildCheckSettings) {
        dom.btnOpenBuildCheckSettings.addEventListener('click', openBuildCheckSettingsModal);
    }
    if (dom.toggleBuildCheck) {
        dom.toggleBuildCheck.addEventListener('change', renderBuildCheckSettingsState);
    }
    if (dom.btnSaveBuildCheckSettings) {
        dom.btnSaveBuildCheckSettings.addEventListener('click', submitBuildCheckSettings);
    }
    if (dom.btnCancelBuildCheckSettings) {
        dom.btnCancelBuildCheckSettings.addEventListener('click', closeBuildCheckSettingsModal);
    }
    if (dom.btnCloseBuildCheckSettings) {
        dom.btnCloseBuildCheckSettings.addEventListener('click', closeBuildCheckSettingsModal);
    }
    if (dom.buildCheckSettingsBackdrop) {
        dom.buildCheckSettingsBackdrop.addEventListener('click', closeBuildCheckSettingsModal);
    }
    if (uiInteractions?.bindModalEnterConfirm && dom.buildCheckSettingsModal && dom.btnSaveBuildCheckSettings) {
        uiInteractions.bindModalEnterConfirm({
            modal: dom.buildCheckSettingsModal,
            confirmButton: dom.btnSaveBuildCheckSettings,
        });
    }
    bindModalEscapeClose(dom.buildCheckSettingsModal, closeBuildCheckSettingsModal);
    if (dom.btnNewRosTerminal) {
        dom.btnNewRosTerminal.addEventListener('click', () => {
            actions.openSourcedTerminal();
        });
    }

    if (dom.environmentDetailsToggle) {
        dom.environmentDetailsToggle.addEventListener('click', () => {
            state.environmentDetailsExpanded = !(state.environmentDetailsExpanded === true);
            renderEnvironmentDetailsState();
        });
    }

    if (dom.runEnvironmentTarget) {
        dom.runEnvironmentTarget.addEventListener('change', () => {
            renderEnvironmentModalState(dom.runEnvironmentTarget.value, undefined);
        });
    }
    if (dom.runTerminalMode) {
        dom.runTerminalMode.addEventListener('change', () => {
            renderEnvironmentModalState(
                dom.runEnvironmentTarget?.value || state.runEnvironmentTarget,
                dom.runTerminalMode.value,
            );
        });
    }
    if (dom.btnSaveEnvironment) {
        dom.btnSaveEnvironment.addEventListener('click', () => {
            const target = getSelectedRunTarget();
            actions.setRunTerminalTarget(target);
            closeEnvironmentModal();
        });
    }
    if (dom.btnCancelEnvironment) {
        dom.btnCancelEnvironment.addEventListener('click', closeEnvironmentModal);
    }
    if (dom.btnCloseEnvironment) {
        dom.btnCloseEnvironment.addEventListener('click', closeEnvironmentModal);
    }
    if (dom.environmentBackdrop) {
        dom.environmentBackdrop.addEventListener('click', closeEnvironmentModal);
    }
    if (uiInteractions?.bindModalEnterConfirm && dom.environmentModal && dom.btnSaveEnvironment) {
        uiInteractions.bindModalEnterConfirm({
            modal: dom.environmentModal,
            confirmButton: dom.btnSaveEnvironment,
        });
    }
    bindModalEscapeClose(dom.environmentModal, closeEnvironmentModal);

    const requestOtherPackagesLoad = (force = false) => {
        if (!force && (state.otherPackagesLoaded || state.otherPackagesLoading)) {
            return;
        }
        state.otherPackagesLoading = true;
        if (dom.btnLoadOtherPackages) {
            dom.btnLoadOtherPackages.disabled = true;
            dom.btnLoadOtherPackages.textContent = 'Loading...';
        }
        render.renderOtherPackages();
        actions.loadOtherPackages(force);
    };

    dom.filterInput.addEventListener('input', () => {
        render.renderPackages();
        render.renderOtherPackages();
        if (dom.filterInput.value.trim()) {
            requestOtherPackagesLoad();
        }
    });

    if (dom.btnLoadOtherPackages) {
        dom.btnLoadOtherPackages.addEventListener('click', () => {
            requestOtherPackagesLoad(true);
        });
    }

    if (dom.btnToggleWorkspacePackages) {
        const onToggleWorkspace = () => {
            render.toggleWorkspacePackages(false);
        };
        const onRecursiveToggleWorkspace = () => {
            // Unity-style behavior: Alt+click toggles all nested package sections.
            render.toggleWorkspacePackages(true);
        };
        bindRecursiveToggleTrigger(
            dom.btnToggleWorkspacePackages,
            onToggleWorkspace,
            onRecursiveToggleWorkspace,
        );
        bindRecursiveToggleTrigger(
            dom.lblToggleWorkspacePackages,
            onToggleWorkspace,
            onRecursiveToggleWorkspace,
        );
    }

    if (dom.btnToggleOtherPackages) {
        const onToggleOther = () => {
            render.toggleOtherPackages(false);
        };
        const onRecursiveToggleOther = () => {
            render.toggleOtherPackages(true);
        };
        bindRecursiveToggleTrigger(
            dom.btnToggleOtherPackages,
            onToggleOther,
            onRecursiveToggleOther,
        );
        bindRecursiveToggleTrigger(
            dom.lblToggleOtherPackages,
            onToggleOther,
            onRecursiveToggleOther,
        );
    }

    if (dom.btnToggleTerminalList) {
        const onToggleTerminals = () => {
            render.toggleTerminals();
        };
        bindRecursiveToggleTrigger(
            dom.btnToggleTerminalList,
            onToggleTerminals,
            onToggleTerminals,
        );
        bindRecursiveToggleTrigger(
            dom.lblToggleTerminalList,
            onToggleTerminals,
            onToggleTerminals,
        );
    }

    dom.btnInsertAll.addEventListener('click', () => {
        if (!state.argsOptions.length) {
            return;
        }
        appendArgEntry(
            state.argsOptions
                .map((opt) => (opt.defaultValue ? opt.name + ':=' + opt.defaultValue : opt.name + ':='))
                .join(' '),
            { immediate: true },
        );
    });

    if (dom.configArgsInput) {
        dom.configArgsInput.addEventListener('input', () => {
            const sanitized = sanitizeArgsText(dom.configArgsInput.value);
            if (dom.configArgsInput.value !== sanitized) {
                dom.configArgsInput.value = sanitized;
            }
            state.currentArgsText = sanitized;
            autoSizeConfigArgsInput();
            queueArgsAutosave();
        });
    }

    dom.btnSaveArgs.addEventListener('click', () => {
        flushArgsAutosave({ refreshConfigTabs: true, refreshConfigSummaries: true });
        closeArgsModal({ persist: false });
    });

    dom.btnAddConfig.addEventListener('click', () => {
        flushArgsAutosave({ refreshConfigTabs: true, refreshConfigSummaries: true });
        const cfg = state.launchArgConfigs[state.currentArgsKey] || { configs: [] };
        const id = 'cfg-' + Date.now();
        const name = 'config ' + (cfg.configs.length + 1);
        const currentRunTarget = normalizeConfigRunTarget(dom.configRunTarget?.value || 'auto');
        cfg.configs.push({ id, name, args: '', runTarget: currentRunTarget });
        state.launchArgConfigs[state.currentArgsKey] = cfg;
        state.currentConfigId = id;
        syncArgsConfigEditor();
        render.renderConfigTabs();
        renderArgsConfigSummaries();
        persistLaunchArgConfigs(state.currentArgsKey);
    });

    dom.btnRemoveConfig.addEventListener('click', () => {
        const cfg = state.launchArgConfigs[state.currentArgsKey];
        if (!cfg || cfg.configs.length <= 1) {
            return;
        }
        flushArgsAutosave({ refreshConfigTabs: true, refreshConfigSummaries: true });
        cfg.configs = cfg.configs.filter((c) => c.id !== state.currentConfigId);
        state.currentConfigId = cfg.configs[0].id;
        state.launchArgConfigs[state.currentArgsKey] = cfg;
        syncArgsConfigEditor();
        render.renderConfigTabs();
        renderArgsConfigSummaries();
        persistLaunchArgConfigs(state.currentArgsKey);
    });

    if (dom.configName) {
        dom.configName.addEventListener('input', () => {
            queueArgsAutosave({
                refreshConfigTabs: true,
                refreshConfigSummaries: true,
            });
        });
    }
    if (dom.configList) {
        dom.configList.addEventListener('click', (event) => {
            const target = event.target;
            if (!(target instanceof Element) || !target.closest('.config-pill')) {
                return;
            }
            flushArgsAutosave();
        }, true);
    }
    if (dom.configRunTarget) {
        dom.configRunTarget.addEventListener('change', () => {
            renderConfigRunTargetState(dom.configRunTarget.value);
            queueArgsAutosave({ immediate: true });
        });
    }

    dom.btnCancelArgs.addEventListener('click', closeArgsModal);
    dom.btnCloseArgs.addEventListener('click', closeArgsModal);
    dom.argsBackdrop.addEventListener('click', closeArgsModal);
    bindModalEscapeClose(dom.argsModal, closeArgsModal);

    // ── Add Node modal ─────────────────────────────────────────
    const openAddNodeModal = (pkgName, pkgPath) => {
        if (!pkgName) {
            return;
        }
        dom.addNodePkg.value = pkgName;
        dom.addNodeModal.dataset.pkgPath = typeof pkgPath === 'string' ? pkgPath : '';
        dom.addNodeName.value = '';
        if (dom.addNodeTemplate) {
            dom.addNodeTemplate.value = 'none';
        }
        updateNodeTemplateFields(true);
        dom.addNodeStatus.className = 'mt hidden';
        dom.addNodeStatus.textContent = '';
        updateNodeNamePreview();
        dom.addNodeModal.classList.remove('hidden');
        dom.addNodeName.focus();
    };

    const closeAddNodeModal = () => {
        dom.addNodeModal.dataset.pkgPath = '';
        dom.addNodeModal.classList.add('hidden');
    };

    const submitAddNode = () => {
        const pkg = dom.addNodePkg.value.trim();
        const nodeName = normalizeNodeName(dom.addNodeName.value);
        const pkgPath = (dom.addNodeModal.dataset.pkgPath || '').trim();
        const nodeTemplate = String(dom.addNodeTemplate?.value || 'none');
        const nodeTopic = normalizeNodeTopic(dom.addNodeTopic?.value || '');
        if (!pkg || !nodeName) {
            return;
        }
        dom.addNodeName.value = nodeName;
        if (dom.addNodeTopic) {
            dom.addNodeTopic.value = nodeTopic;
        }
        updateNodeNamePreview();
        updateNodeTopicPreview();
        dom.btnAddNode.disabled = true;
        dom.addNodeStatus.className = 'mt';
        dom.addNodeStatus.innerHTML = '<span class="spinner"></span> Adding node…';
        actions.addNode(pkg, nodeName, pkgPath, nodeTemplate, nodeTopic);
    };

    dom.btnAddNode.addEventListener('click', submitAddNode);

    dom.btnCancelAddNode.addEventListener('click', closeAddNodeModal);
    dom.btnCloseAddNode.addEventListener('click', closeAddNodeModal);
    dom.addNodeBackdrop.addEventListener('click', closeAddNodeModal);
    if (dom.addNodeName) {
        dom.addNodeName.addEventListener('input', updateNodeNamePreview);
    }
    if (dom.addNodeTopic) {
        dom.addNodeTopic.addEventListener('input', updateNodeTopicPreview);
    }
    if (dom.addNodeTemplate) {
        dom.addNodeTemplate.addEventListener('change', () => updateNodeTemplateFields(true));
    }
    if (uiInteractions?.bindModalEnterConfirm) {
        uiInteractions.bindModalEnterConfirm({
            modal: dom.addNodeModal,
            confirmButton: dom.btnAddNode,
        });
    }
    bindModalEscapeClose(dom.addNodeModal, closeAddNodeModal);

    const openCreateLaunchModal = (pkgName, pkgPath) => {
        if (!pkgName) {
            return;
        }
        dom.createLaunchPkg.value = pkgName;
        dom.createLaunchModal.dataset.pkgPath = typeof pkgPath === 'string' ? pkgPath : '';
        dom.createLaunchName.value = '';
        dom.createLaunchStatus.className = 'mt hidden';
        dom.createLaunchStatus.textContent = '';
        updateLaunchNamePreview();
        dom.createLaunchModal.classList.remove('hidden');
        dom.createLaunchName.focus();
    };

    const closeCreateLaunchModal = () => {
        dom.createLaunchModal.dataset.pkgPath = '';
        dom.createLaunchModal.classList.add('hidden');
    };

    const submitCreateLaunch = () => {
        const pkg = String(dom.createLaunchPkg?.value || '').trim();
        const launchName = normalizeLaunchName(dom.createLaunchName?.value || '');
        const pkgPath = String(dom.createLaunchModal?.dataset?.pkgPath || '').trim();
        if (!pkg || !launchName) {
            return;
        }

        dom.createLaunchName.value = launchName;
        updateLaunchNamePreview();
        dom.btnCreateLaunch.disabled = true;
        dom.createLaunchStatus.className = 'mt';
        dom.createLaunchStatus.innerHTML = '<span class="spinner"></span> Creating launch file...';
        actions.createLaunch(pkg, launchName, pkgPath);
    };

    dom.btnCreateLaunch.addEventListener('click', submitCreateLaunch);
    dom.btnCancelCreateLaunch.addEventListener('click', closeCreateLaunchModal);
    dom.btnCloseCreateLaunch.addEventListener('click', closeCreateLaunchModal);
    dom.createLaunchBackdrop.addEventListener('click', closeCreateLaunchModal);
    if (dom.createLaunchName) {
        dom.createLaunchName.addEventListener('input', updateLaunchNamePreview);
    }
    if (uiInteractions?.bindModalEnterConfirm) {
        uiInteractions.bindModalEnterConfirm({
            modal: dom.createLaunchModal,
            confirmButton: dom.btnCreateLaunch,
        });
    }
    bindModalEscapeClose(dom.createLaunchModal, closeCreateLaunchModal);

    const openRemoveLaunchModal = (pkgName, launchName, pkgPath, launchPath) => {
        if (!pkgName || !launchName) {
            return;
        }
        dom.removeLaunchPkg.value = pkgName;
        dom.removeLaunchName.value = launchName;
        dom.removeLaunchPath.value = launchPath || '(not detected)';
        dom.removeLaunchModal.dataset.pkgPath = typeof pkgPath === 'string' ? pkgPath : '';
        dom.removeLaunchModal.dataset.launchPath = typeof launchPath === 'string' ? launchPath : '';
        dom.removeLaunchStatus.className = 'mt hidden';
        dom.removeLaunchStatus.textContent = '';
        dom.removeLaunchModal.classList.remove('hidden');
        dom.btnRemoveLaunch.focus();
    };

    const closeRemoveLaunchModal = () => {
        dom.removeLaunchModal.dataset.pkgPath = '';
        dom.removeLaunchModal.dataset.launchPath = '';
        dom.removeLaunchModal.classList.add('hidden');
    };

    const submitRemoveLaunch = () => {
        const pkg = String(dom.removeLaunchPkg?.value || '').trim();
        const launchName = String(dom.removeLaunchName?.value || '').trim();
        const pkgPath = String(dom.removeLaunchModal?.dataset?.pkgPath || '').trim();
        const launchPath = String(dom.removeLaunchModal?.dataset?.launchPath || '').trim();
        if (!pkg || !launchName) {
            return;
        }

        dom.btnRemoveLaunch.disabled = true;
        dom.removeLaunchStatus.className = 'mt';
        dom.removeLaunchStatus.innerHTML = '<span class="spinner"></span> Removing launch file...';
        actions.removeLaunch(pkg, launchName, pkgPath, launchPath);
    };

    dom.btnRemoveLaunch.addEventListener('click', submitRemoveLaunch);
    dom.btnCancelRemoveLaunch.addEventListener('click', closeRemoveLaunchModal);
    dom.btnCloseRemoveLaunch.addEventListener('click', closeRemoveLaunchModal);
    dom.removeLaunchBackdrop.addEventListener('click', closeRemoveLaunchModal);
    if (uiInteractions?.bindModalEnterConfirm) {
        uiInteractions.bindModalEnterConfirm({
            modal: dom.removeLaunchModal,
            confirmButton: dom.btnRemoveLaunch,
        });
    }
    bindModalEscapeClose(dom.removeLaunchModal, closeRemoveLaunchModal);

    // ── Remove Node modal ──────────────────────────────────────
    const openRemoveNodeModal = (pkgName, nodeName, pkgPath, nodePath) => {
        if (!pkgName || !nodeName) {
            return;
        }
        dom.removeNodePkg.value = pkgName;
        dom.removeNodeName.value = nodeName;
        dom.removeNodePath.value = nodePath || '(not detected)';
        dom.removeNodeModal.dataset.pkgPath = typeof pkgPath === 'string' ? pkgPath : '';
        dom.removeNodeModal.dataset.nodePath = typeof nodePath === 'string' ? nodePath : '';
        dom.removeNodeStatus.className = 'mt hidden';
        dom.removeNodeStatus.textContent = '';
        dom.removeNodeModal.classList.remove('hidden');
        dom.btnRemoveNode.focus();
    };

    const closeRemoveNodeModal = () => {
        dom.removeNodeModal.dataset.pkgPath = '';
        dom.removeNodeModal.dataset.nodePath = '';
        dom.removeNodeModal.classList.add('hidden');
    };

    const submitRemoveNode = () => {
        const pkg = String(dom.removeNodePkg?.value || '').trim();
        const nodeName = String(dom.removeNodeName?.value || '').trim();
        const pkgPath = String(dom.removeNodeModal?.dataset?.pkgPath || '').trim();
        const nodePath = String(dom.removeNodeModal?.dataset?.nodePath || '').trim();
        if (!pkg || !nodeName) {
            return;
        }

        dom.btnRemoveNode.disabled = true;
        dom.removeNodeStatus.className = 'mt';
        dom.removeNodeStatus.innerHTML = '<span class="spinner"></span> Removing node…';
        actions.removeNode(pkg, nodeName, pkgPath, nodePath);
    };

    dom.btnRemoveNode.addEventListener('click', submitRemoveNode);
    dom.btnCancelRemoveNode.addEventListener('click', closeRemoveNodeModal);
    dom.btnCloseRemoveNode.addEventListener('click', closeRemoveNodeModal);
    dom.removeNodeBackdrop.addEventListener('click', closeRemoveNodeModal);
    if (uiInteractions?.bindModalEnterConfirm) {
        uiInteractions.bindModalEnterConfirm({
            modal: dom.removeNodeModal,
            confirmButton: dom.btnRemoveNode,
        });
    }
    bindModalEscapeClose(dom.removeNodeModal, closeRemoveNodeModal);

    window.PM.handlers = {
        openArgsModal,
        closeArgsModal,
        openCreate,
        closeCreate,
        openEnvironmentModal,
        closeEnvironmentModal,
        renderEnvironmentModalState,
        renderBuildCheckSettingsState,
        openBuildCheckSettingsModal,
        closeBuildCheckSettingsModal,
        openAddNodeModal,
        closeAddNodeModal,
        openCreateLaunchModal,
        closeCreateLaunchModal,
        openRemoveLaunchModal,
        closeRemoveLaunchModal,
        openRemoveNodeModal,
        closeRemoveNodeModal,
        syncArgsConfigEditor,
        appendArgEntry,
    };
})();
