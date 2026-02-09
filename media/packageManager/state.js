/* Package Manager Webview State */
(function () {
    window.PM = window.PM || {};

    window.PM.state = {
        allPackages: [],
        otherPackages: [],
        otherPackagesLoaded: false,
        otherPackagesLoading: false,
        pinnedPaths: [],
        expandedPackages: {},
        expandedPackageRows: {},
        workspacePackagesVisible: true,
        launchArgConfigs: {},
        argsOptions: [],
        currentArgsKey: '',
        terminals: [],
        preferredTerminalId: '',
        currentConfigId: '',
    };

    window.PM.setState = (patch) => {
        Object.assign(window.PM.state, patch);
    };
})();
