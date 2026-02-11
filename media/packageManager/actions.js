/* Package Manager Webview Actions */
(function () {
    window.PM = window.PM || {};
    const vscode = acquireVsCodeApi();
    const { toHost } = window.PM.messages;

    window.PM.actions = {
        post: (payload) => vscode.postMessage(payload),
        openLaunch: (path) => vscode.postMessage({ command: toHost.OPEN_LAUNCH, path }),
        openNode: (path) => vscode.postMessage({ command: toHost.OPEN_NODE, path }),
        launchFile: (pkg, file, path, args, argsName) => vscode.postMessage({
            command: toHost.LAUNCH_FILE,
            pkg,
            file,
            path,
            args,
            argsName,
        }),
        runNode: (pkg, executable, args, argsName, path) => vscode.postMessage({
            command: toHost.RUN_NODE,
            pkg,
            executable,
            args,
            argsName,
            path,
        }),
        togglePin: (path) => vscode.postMessage({ command: toHost.TOGGLE_PIN, path }),
        killTerminal: (id) => vscode.postMessage({ command: toHost.KILL_TERMINAL, id }),
        focusTerminal: (id) => vscode.postMessage({ command: toHost.FOCUS_TERMINAL, id }),
        setPreferredTerminal: (id) => vscode.postMessage({ command: toHost.SET_PREFERRED_TERMINAL, id }),
        requestLaunchArgs: (argsKey, sourcePath) => vscode.postMessage({
            command: toHost.REQUEST_LAUNCH_ARGS,
            argsKey,
            sourcePath,
        }),
        refreshPackages: () => vscode.postMessage({ command: toHost.REFRESH_PACKAGES }),
        loadOtherPackages: (force) => vscode.postMessage({
            command: toHost.LOAD_OTHER_PACKAGES,
            force: Boolean(force),
        }),
        loadOtherPackageDetails: (name) => vscode.postMessage({
            command: toHost.LOAD_OTHER_PACKAGE_DETAILS,
            name,
        }),
        toggleBuildCheck: (enabled) => vscode.postMessage({ command: toHost.TOGGLE_BUILD_CHECK, enabled }),
        createPackage: (name, buildType, deps, license) => vscode.postMessage({
            command: toHost.CREATE_PACKAGE,
            name,
            buildType,
            deps,
            license,
        }),
        addNode: (pkg, nodeName) => vscode.postMessage({ command: toHost.ADD_NODE, pkg, nodeName }),
        setLaunchArgConfigs: (argsKey, configs) => vscode.postMessage({
            command: toHost.SET_LAUNCH_ARG_CONFIGS,
            path: argsKey,
            configs,
        }),
    };
})();
