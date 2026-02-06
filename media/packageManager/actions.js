/* Package Manager Webview Actions */
(function () {
    window.PM = window.PM || {};
    const vscode = acquireVsCodeApi();
    const { toHost } = window.PM.messages;

    window.PM.actions = {
        post: (payload) => vscode.postMessage(payload),
        openLaunch: (path) => vscode.postMessage({ command: toHost.OPEN_LAUNCH, path }),
        launchFile: (pkg, file, path, args, argsName) => vscode.postMessage({
            command: toHost.LAUNCH_FILE,
            pkg,
            file,
            path,
            args,
            argsName,
        }),
        togglePin: (path) => vscode.postMessage({ command: toHost.TOGGLE_PIN, path }),
        killTerminal: (id) => vscode.postMessage({ command: toHost.KILL_TERMINAL, id }),
        focusTerminal: (id) => vscode.postMessage({ command: toHost.FOCUS_TERMINAL, id }),
        setPreferredTerminal: (id) => vscode.postMessage({ command: toHost.SET_PREFERRED_TERMINAL, id }),
        requestLaunchArgs: (path) => vscode.postMessage({ command: toHost.REQUEST_LAUNCH_ARGS, path }),
        refreshPackages: () => vscode.postMessage({ command: toHost.REFRESH_PACKAGES }),
        createPackage: (name, buildType, deps) => vscode.postMessage({ command: toHost.CREATE_PACKAGE, name, buildType, deps }),
        setLaunchArgConfigs: (path, configs) => vscode.postMessage({
            command: toHost.SET_LAUNCH_ARG_CONFIGS,
            path,
            configs,
        }),
    };
})();
