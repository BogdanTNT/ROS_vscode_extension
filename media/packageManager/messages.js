/* Package Manager Webview Message Constants */
(function () {
    window.PM = window.PM || {};

    window.PM.messages = Object.freeze({
        toHost: Object.freeze({
            CREATE_PACKAGE: 'createPackage',
            REFRESH_PACKAGES: 'refreshPackages',
            OPEN_LAUNCH: 'openLaunch',
            LAUNCH_FILE: 'launchFile',
            TOGGLE_PIN: 'togglePin',
            SET_LAUNCH_ARG_CONFIGS: 'setLaunchArgConfigs',
            REQUEST_LAUNCH_ARGS: 'requestLaunchArgs',
            KILL_TERMINAL: 'killTerminal',
            FOCUS_TERMINAL: 'focusTerminal',
            SET_PREFERRED_TERMINAL: 'setPreferredTerminal',
        }),
        toWebview: Object.freeze({
            PACKAGE_LIST: 'packageList',
            CREATE_DONE: 'createDone',
            FOCUS_CREATE: 'focusCreate',
            LAUNCH_ARGS_OPTIONS: 'launchArgsOptions',
            TERMINAL_LIST: 'terminalList',
        }),
    });
})();
