/* Package Manager Webview Events */
(function () {
    window.PM = window.PM || {};

    const { dom, state, actions, render } = window.PM;

    const openCreate = () => {
        dom.createModal.classList.remove('hidden');
        dom.statusEl.className = 'mt hidden';
        dom.statusEl.textContent = '';
        document.getElementById('pkgName').focus();
    };

    const closeCreate = () => {
        dom.createModal.classList.add('hidden');
    };

    const openArgsModal = (path) => {
        state.currentArgsPath = path;
        const cfg = state.launchArgConfigs[path] || {
            configs: [{ id: 'default', name: 'default', args: '' }],
        };
        state.launchArgConfigs[path] = cfg;
        state.currentConfigId = cfg.configs[0]?.id || 'default';
        const currentCfg = cfg.configs.find((c) => c.id === state.currentConfigId) || cfg.configs[0];
        dom.argsInput.value = currentCfg?.args || '';
        dom.configName.value = currentCfg?.name || '';
        state.argsOptions = [];
        render.renderArgsOptions();
        render.renderConfigTabs();
        dom.argsModal.classList.remove('hidden');
        dom.argsInput.focus();
        if (state.currentArgsPath) {
            actions.requestLaunchArgs(state.currentArgsPath);
        }
    };

    const closeArgsModal = () => {
        dom.argsModal.classList.add('hidden');
    };

    dom.btnCreate.addEventListener('click', () => {
        const name = document.getElementById('pkgName').value.trim();
        const buildType = document.getElementById('buildType').value;
        const deps = document.getElementById('deps').value.trim();
        if (!name) {
            return;
        }
        dom.btnCreate.disabled = true;
        dom.statusEl.className = 'mt';
        dom.statusEl.innerHTML = '<span class="spinner"></span> Creating…';
        actions.createPackage(name, buildType, deps);
    });

    dom.btnOpenCreate.addEventListener('click', openCreate);
    dom.btnCloseCreate.addEventListener('click', closeCreate);
    dom.btnCancelCreate.addEventListener('click', closeCreate);
    dom.createBackdrop.addEventListener('click', closeCreate);

    dom.btnRefresh.addEventListener('click', () => {
        actions.refreshPackages();
    });

    dom.filterInput.addEventListener('input', render.renderPackages);

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
        const cfg = state.launchArgConfigs[state.currentArgsPath] || { configs: [] };
        const list = cfg.configs.length ? cfg.configs : [{ id: 'default', name: 'default', args: '' }];
        const idx = list.findIndex((c) => c.id === state.currentConfigId);
        if (idx >= 0) {
            list[idx].args = dom.argsInput.value;
            list[idx].name = dom.configName.value || list[idx].name || 'config';
        }
        cfg.configs = list;
        state.launchArgConfigs[state.currentArgsPath] = cfg;
        actions.setLaunchArgConfigs(state.currentArgsPath, cfg.configs);
        closeArgsModal();
    });

    dom.btnAddConfig.addEventListener('click', () => {
        const cfg = state.launchArgConfigs[state.currentArgsPath] || { configs: [] };
        const id = 'cfg-' + Date.now();
        const name = 'config ' + (cfg.configs.length + 1);
        cfg.configs.push({ id, name, args: '' });
        state.launchArgConfigs[state.currentArgsPath] = cfg;
        state.currentConfigId = id;
        dom.configName.value = name;
        dom.argsInput.value = '';
        render.renderConfigTabs();
    });

    dom.btnRemoveConfig.addEventListener('click', () => {
        const cfg = state.launchArgConfigs[state.currentArgsPath];
        if (!cfg || cfg.configs.length <= 1) {
            return;
        }
        cfg.configs = cfg.configs.filter((c) => c.id !== state.currentConfigId);
        state.currentConfigId = cfg.configs[0].id;
        const currentCfg = cfg.configs[0];
        dom.configName.value = currentCfg?.name || '';
        dom.argsInput.value = currentCfg?.args || '';
        state.launchArgConfigs[state.currentArgsPath] = cfg;
        render.renderConfigTabs();
    });

    dom.btnCancelArgs.addEventListener('click', closeArgsModal);
    dom.btnCloseArgs.addEventListener('click', closeArgsModal);
    dom.argsBackdrop.addEventListener('click', closeArgsModal);

    window.PM.handlers = {
        openArgsModal,
        closeArgsModal,
        openCreate,
        closeCreate,
    };
})();
