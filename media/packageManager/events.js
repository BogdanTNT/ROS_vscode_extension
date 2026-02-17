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

    const normalizePackageName = (rawName) => String(rawName || '').trim().replace(/\s+/g, '_');
    const normalizeNodeName = (rawName) => String(rawName || '').trim().replace(/\s+/g, '_');
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

    const updateNodeTemplateFields = (forceSuggestion = false) => {
        const template = String(dom.addNodeTemplate?.value || 'none').toLowerCase();
        const suggestedTopic = nodeTemplateTopicSuggestions[template] || '';
        const requiresTopic = Boolean(suggestedTopic);
        const rememberedTopic = String(state.lastNodeTopic || '').trim();

        if (dom.addNodeTopicRow) {
            dom.addNodeTopicRow.classList.toggle('hidden', !requiresTopic);
        }
        if (!dom.addNodeTopic) {
            return;
        }

        dom.addNodeTopic.placeholder = suggestedTopic || 'chatter';
        const currentValue = String(dom.addNodeTopic.value || '').trim();
        if (requiresTopic) {
            if (forceSuggestion || !currentValue) {
                dom.addNodeTopic.value = rememberedTopic || suggestedTopic;
            }
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

    const openArgsModal = (argsKey, sourcePath) => {
        if (!argsKey) {
            return;
        }
        state.currentArgsKey = argsKey;
        const cfg = state.launchArgConfigs[argsKey] || {
            configs: [{ id: 'default', name: 'default', args: '' }],
        };
        state.launchArgConfigs[argsKey] = cfg;
        state.currentConfigId = cfg.configs[0]?.id || 'default';
        const currentCfg = cfg.configs.find((c) => c.id === state.currentConfigId) || cfg.configs[0];
        dom.argsInput.value = currentCfg?.args || '';
        dom.configName.value = currentCfg?.name || '';
        state.argsOptions = [];
        render.renderArgsOptions();
        render.renderConfigTabs();
        dom.argsModal.classList.remove('hidden');
        dom.argsInput.focus();
        if (state.currentArgsKey) {
            actions.requestLaunchArgs(state.currentArgsKey, sourcePath || '');
        }
    };

    const closeArgsModal = () => {
        dom.argsModal.classList.add('hidden');
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

    dom.btnRefresh.addEventListener('click', () => {
        actions.refreshPackages();
    });

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

    dom.toggleBuildCheck.addEventListener('change', () => {
        actions.toggleBuildCheck(dom.toggleBuildCheck.checked);
    });

    dom.btnInsertAll.addEventListener('click', () => {
        if (!state.argsOptions.length) {
            return;
        }
        const values = state.argsOptions
            .map((opt) => (opt.defaultValue ? opt.name + ':=' + opt.defaultValue : opt.name + ':='))
            .join(' ');
        const current = dom.argsInput.value.trim();
        dom.argsInput.value = current ? current + ' ' + values : values;
    });

    dom.btnSaveArgs.addEventListener('click', () => {
        const cfg = state.launchArgConfigs[state.currentArgsKey] || { configs: [] };
        const list = cfg.configs.length ? cfg.configs : [{ id: 'default', name: 'default', args: '' }];
        const idx = list.findIndex((c) => c.id === state.currentConfigId);
        if (idx >= 0) {
            list[idx].args = dom.argsInput.value;
            list[idx].name = dom.configName.value || list[idx].name || 'config';
        }
        cfg.configs = list;
        state.launchArgConfigs[state.currentArgsKey] = cfg;
        actions.setLaunchArgConfigs(state.currentArgsKey, cfg.configs);
        closeArgsModal();
    });

    dom.btnAddConfig.addEventListener('click', () => {
        const cfg = state.launchArgConfigs[state.currentArgsKey] || { configs: [] };
        const id = 'cfg-' + Date.now();
        const name = 'config ' + (cfg.configs.length + 1);
        cfg.configs.push({ id, name, args: '' });
        state.launchArgConfigs[state.currentArgsKey] = cfg;
        state.currentConfigId = id;
        dom.configName.value = name;
        dom.argsInput.value = '';
        render.renderConfigTabs();
    });

    dom.btnRemoveConfig.addEventListener('click', () => {
        const cfg = state.launchArgConfigs[state.currentArgsKey];
        if (!cfg || cfg.configs.length <= 1) {
            return;
        }
        cfg.configs = cfg.configs.filter((c) => c.id !== state.currentConfigId);
        state.currentConfigId = cfg.configs[0].id;
        const currentCfg = cfg.configs[0];
        dom.configName.value = currentCfg?.name || '';
        dom.argsInput.value = currentCfg?.args || '';
        state.launchArgConfigs[state.currentArgsKey] = cfg;
        render.renderConfigTabs();
    });

    dom.btnCancelArgs.addEventListener('click', closeArgsModal);
    dom.btnCloseArgs.addEventListener('click', closeArgsModal);
    dom.argsBackdrop.addEventListener('click', closeArgsModal);

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
        const nodeTopic = String(dom.addNodeTopic?.value || '').trim();
        if (!pkg || !nodeName) {
            return;
        }
        dom.addNodeName.value = nodeName;
        updateNodeNamePreview();
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
    if (dom.addNodeTemplate) {
        dom.addNodeTemplate.addEventListener('change', () => updateNodeTemplateFields(true));
    }
    if (uiInteractions?.bindModalEnterConfirm) {
        uiInteractions.bindModalEnterConfirm({
            modal: dom.addNodeModal,
            confirmButton: dom.btnAddNode,
        });
    }

    window.PM.handlers = {
        openArgsModal,
        closeArgsModal,
        openCreate,
        closeCreate,
        openAddNodeModal,
        closeAddNodeModal,
    };
})();
