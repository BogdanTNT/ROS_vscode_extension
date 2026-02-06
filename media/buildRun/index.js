/* Build & Run Webview Script */
(function () {
    const vscode = acquireVsCodeApi();

    const commands = Object.freeze({
        toHost: Object.freeze({
            BUILD: 'build',
            RUN: 'run',
            LAUNCH: 'launch',
            REFRESH_PACKAGES: 'refreshPackages',
            SOURCE_WORKSPACE: 'sourceWorkspace',
            CLEAN_BUILD: 'cleanBuild',
        }),
        toWebview: Object.freeze({
            PACKAGE_LIST: 'packageList',
            FOCUS_BUILD: 'focusBuild',
            FOCUS_RUN: 'focusRun',
        }),
    });

    const escapeAttr = (value) => String(value ?? '').replace(/[&<>"']/g, (ch) => {
        if (ch === '&') {
            return '&amp;';
        }
        if (ch === '<') {
            return '&lt;';
        }
        if (ch === '>') {
            return '&gt;';
        }
        if (ch === '"') {
            return '&quot;';
        }
        return '&#39;';
    });

    document.getElementById('btnBuild').addEventListener('click', () => {
        const raw = document.getElementById('buildPkgs').value.trim();
        const packages = raw ? raw.split(/[\s,]+/).filter(Boolean) : [];
        vscode.postMessage({ command: commands.toHost.BUILD, packages });
    });

    document.getElementById('btnClean').addEventListener('click', () => {
        vscode.postMessage({ command: commands.toHost.CLEAN_BUILD });
    });

    document.getElementById('btnSource').addEventListener('click', () => {
        vscode.postMessage({ command: commands.toHost.SOURCE_WORKSPACE });
    });

    document.getElementById('btnRun').addEventListener('click', () => {
        const pkg = document.getElementById('runPkg').value.trim();
        const exec = document.getElementById('runExec').value.trim();
        const args = document.getElementById('runArgs').value.trim();
        if (!pkg || !exec) {
            return;
        }
        vscode.postMessage({ command: commands.toHost.RUN, pkg, executable: exec, args });
    });

    document.getElementById('btnLaunch').addEventListener('click', () => {
        const pkg = document.getElementById('launchPkg').value.trim();
        const file = document.getElementById('launchFile').value.trim();
        if (!pkg || !file) {
            return;
        }
        vscode.postMessage({ command: commands.toHost.LAUNCH, pkg, launchFile: file });
    });

    window.addEventListener('message', (event) => {
        const msg = event.data;
        if (msg.command === commands.toWebview.PACKAGE_LIST) {
            const dl = document.getElementById('pkgSuggestions');
            dl.innerHTML = (msg.packages || [])
                .map((p) => '<option value="' + escapeAttr(p) + '">')
                .join('');
            return;
        }
        if (msg.command === commands.toWebview.FOCUS_BUILD) {
            document.getElementById('buildPkgs').focus();
            return;
        }
        if (msg.command === commands.toWebview.FOCUS_RUN) {
            document.getElementById('runPkg').focus();
        }
    });

    vscode.postMessage({ command: commands.toHost.REFRESH_PACKAGES });
})();
