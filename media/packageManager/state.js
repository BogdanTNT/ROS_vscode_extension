/* Package Manager Webview State */
(function () {
    window.PM = window.PM || {};

    window.PM.state = {
        allPackages: [],
        pinnedPaths: [],
        expandedPackages: {},
        launchArgConfigs: {},
        argsOptions: [],
        currentArgsPath: '',
        terminals: [],
        preferredTerminalId: '',
        currentConfigId: '',
    };

    window.PM.setState = (patch) => {
        Object.assign(window.PM.state, patch);
    };
})();
