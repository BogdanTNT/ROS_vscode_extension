/* Package Manager Webview Actions */
(function () {
    window.PM = window.PM || {};
    const vscode = window.__rosVscodeApi || (window.__rosVscodeApi = acquireVsCodeApi());
    const { toHost } = window.PM.messages;

    window.PM.actions = {
        post: (payload) => vscode.postMessage(payload),
        openLaunch: (path) => vscode.postMessage({ command: toHost.OPEN_LAUNCH, path }),
        openNode: (path) => vscode.postMessage({ command: toHost.OPEN_NODE, path }),
        launchFile: (pkg, file, path, args, argsName, runTarget) => vscode.postMessage({
            command: toHost.LAUNCH_FILE,
            pkg,
            file,
            path,
            args,
            argsName,
            ...(runTarget ? { runTarget } : {}),
        }),
        runNode: (pkg, executable, args, argsName, path, runTarget) => vscode.postMessage({
            command: toHost.RUN_NODE,
            pkg,
            executable,
            args,
            argsName,
            path,
            ...(runTarget ? { runTarget } : {}),
        }),
        togglePin: (path) => vscode.postMessage({ command: toHost.TOGGLE_PIN, path }),
        killTerminal: (id) => vscode.postMessage({ command: toHost.KILL_TERMINAL, id }),
        focusTerminal: (id) => vscode.postMessage({ command: toHost.FOCUS_TERMINAL, id }),
        relaunchTerminal: (id) => vscode.postMessage({ command: toHost.RELAUNCH_TERMINAL, id }),
        setPreferredTerminal: (id) => vscode.postMessage({ command: toHost.SET_PREFERRED_TERMINAL, id }),
        requestLaunchArgs: (argsKey, sourcePath) => vscode.postMessage({
            command: toHost.REQUEST_LAUNCH_ARGS,
            argsKey,
            sourcePath,
        }),
        refreshPackages: () => vscode.postMessage({ command: toHost.REFRESH_PACKAGES }),
        requestEnvironmentDialogState: () => vscode.postMessage({
            command: toHost.REQUEST_ENV_DIALOG_STATE,
        }),
        showEnvironmentInfo: () => vscode.postMessage({ command: toHost.SHOW_ENV_INFO }),
        openSourcedTerminal: () => vscode.postMessage({ command: toHost.OPEN_SOURCED_TERMINAL }),
        setRunTerminalTarget: (target) => vscode.postMessage({
            command: toHost.SET_RUN_TERMINAL_TARGET,
            target,
        }),
        loadOtherPackages: (force) => vscode.postMessage({
            command: toHost.LOAD_OTHER_PACKAGES,
            force: Boolean(force),
        }),
        loadOtherPackageDetails: (name) => vscode.postMessage({
            command: toHost.LOAD_OTHER_PACKAGE_DETAILS,
            name,
        }),
        toggleBuildCheck: (enabled) => vscode.postMessage({ command: toHost.TOGGLE_BUILD_CHECK, enabled }),
        createPackage: (name, buildType, deps, license, description) => vscode.postMessage({
            command: toHost.CREATE_PACKAGE,
            name,
            buildType,
            deps,
            license,
            description,
        }),
        addNode: (pkg, nodeName, pkgPath, nodeTemplate, nodeTopic) => vscode.postMessage({
            command: toHost.ADD_NODE,
            pkg,
            nodeName,
            pkgPath,
            nodeTemplate,
            nodeTopic,
        }),
        removeNode: (pkg, nodeName, pkgPath, nodePath) => vscode.postMessage({
            command: toHost.REMOVE_NODE,
            pkg,
            nodeName,
            pkgPath,
            nodePath,
        }),
        createLaunch: (pkg, launchName, pkgPath) => vscode.postMessage({
            command: toHost.CREATE_LAUNCH,
            pkg,
            launchName,
            pkgPath,
        }),
        removeLaunch: (pkg, launchName, pkgPath, launchPath) => vscode.postMessage({
            command: toHost.REMOVE_LAUNCH,
            pkg,
            launchName,
            pkgPath,
            launchPath,
        }),
        setLaunchArgConfigs: (argsKey, configs) => vscode.postMessage({
            command: toHost.SET_LAUNCH_ARG_CONFIGS,
            path: argsKey,
            configs,
        }),
    };
})();
