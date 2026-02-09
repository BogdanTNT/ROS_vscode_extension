/* Package Manager Webview Message Constants */
(function () {
    window.PM = window.PM || {};

    window.PM.messages = Object.freeze({
        toHost: Object.freeze({
            CREATE_PACKAGE: 'createPackage',
            REFRESH_PACKAGES: 'refreshPackages',
            LOAD_OTHER_PACKAGES: 'loadOtherPackages',
            OPEN_LAUNCH: 'openLaunch',
            OPEN_NODE: 'openNode',
            LAUNCH_FILE: 'launchFile',
            RUN_NODE: 'runNode',
            TOGGLE_PIN: 'togglePin',
            SET_LAUNCH_ARG_CONFIGS: 'setLaunchArgConfigs',
            REQUEST_LAUNCH_ARGS: 'requestLaunchArgs',
            KILL_TERMINAL: 'killTerminal',
            FOCUS_TERMINAL: 'focusTerminal',
            SET_PREFERRED_TERMINAL: 'setPreferredTerminal',
            TOGGLE_BUILD_CHECK: 'toggleBuildCheck',
            ADD_NODE: 'addNode',
        }),
        toWebview: Object.freeze({
            PACKAGE_LIST: 'packageList',
            OTHER_PACKAGE_LIST: 'otherPackageList',
            CREATE_DONE: 'createDone',
            FOCUS_CREATE: 'focusCreate',
            LAUNCH_ARGS_OPTIONS: 'launchArgsOptions',
            TERMINAL_LIST: 'terminalList',
            BUILD_CHECK_STATE: 'buildCheckState',
            ADD_NODE_DONE: 'addNodeDone',
        }),
    });
})();
