/* Package Manager Webview State */
(function () {
    window.PM = window.PM || {};

    const LAST_NODE_TOPIC_STORAGE_KEY = 'rosDevToolkit.packageManager.lastNodeTopic';
    const readLastNodeTopic = () => {
        try {
            const raw = localStorage.getItem(LAST_NODE_TOPIC_STORAGE_KEY);
            return String(raw || '').trim();
        } catch {
            return '';
        }
    };

    window.PM.state = {
        allPackages: [],
        otherPackages: [],
        otherPackagesLoaded: false,
        otherPackagesLoading: false,
        pinnedPaths: [],
        expandedPackages: {},
        expandedPackageRows: {},
        workspacePackagesVisible: true,
        otherPackagesVisible: true,
        launchArgConfigs: {},
        argsOptions: [],
        currentArgsKey: '',
        terminals: [],
        preferredTerminalId: '',
        currentConfigId: '',
        lastNodeTopic: readLastNodeTopic(),
    };

    window.PM.setState = (patch) => {
        Object.assign(window.PM.state, patch);
    };

    window.PM.setLastNodeTopic = (topic) => {
        const normalizedTopic = String(topic || '').trim();
        window.PM.state.lastNodeTopic = normalizedTopic;
        try {
            if (normalizedTopic) {
                localStorage.setItem(LAST_NODE_TOPIC_STORAGE_KEY, normalizedTopic);
            } else {
                localStorage.removeItem(LAST_NODE_TOPIC_STORAGE_KEY);
            }
        } catch {
            // Ignore localStorage failures in restricted webview contexts.
        }
    };
})();
