export const PMToHostCommand = {
    CREATE_PACKAGE: 'createPackage',
    REFRESH_PACKAGES: 'refreshPackages',
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
} as const;

export const PMToWebviewCommand = {
    PACKAGE_LIST: 'packageList',
    CREATE_DONE: 'createDone',
    FOCUS_CREATE: 'focusCreate',
    LAUNCH_ARGS_OPTIONS: 'launchArgsOptions',
    TERMINAL_LIST: 'terminalList',
    BUILD_CHECK_STATE: 'buildCheckState',
} as const;
