/* Shared webview UI preferences (font size + margins) */
(function () {
    const vscode = window.__rosVscodeApi || (window.__rosVscodeApi = acquireVsCodeApi());
    const LOCAL_STORAGE_KEY = 'rosDevToolkit.webviewUiPreferences.local';
    const COMMAND_REQUEST = 'requestUiPreferences';
    const COMMAND_SET = 'setUiPreferences';
    const COMMAND_STATE = 'uiPreferencesState';
    const DEFAULTS = Object.freeze({
        fontSizePx: 13,
        compactMargins: false,
    });
    const LIMITS = Object.freeze({
        minFontSizePx: 10,
        maxFontSizePx: 24,
    });

    let prefs = normalize(readLocalPreferences());
    let dom = null;

    function clamp(value, min, max) {
        return Math.min(max, Math.max(min, value));
    }

    function normalize(raw) {
        if (!raw || typeof raw !== 'object') {
            return { ...DEFAULTS };
        }
        const candidate = raw;
        const numericFontSize = Number(candidate.fontSizePx);
        const fontSizePx = Number.isFinite(numericFontSize)
            ? clamp(Math.round(numericFontSize), LIMITS.minFontSizePx, LIMITS.maxFontSizePx)
            : DEFAULTS.fontSizePx;
        return {
            fontSizePx,
            compactMargins: candidate.compactMargins === true,
        };
    }

    function readLocalPreferences() {
        try {
            const raw = localStorage.getItem(LOCAL_STORAGE_KEY);
            return raw ? JSON.parse(raw) : undefined;
        } catch {
            return undefined;
        }
    }

    function writeLocalPreferences(value) {
        try {
            localStorage.setItem(LOCAL_STORAGE_KEY, JSON.stringify(value));
        } catch {
            // Ignore storage failures in restricted webview contexts.
        }
    }

    function applyPreferences(value) {
        document.documentElement.style.setProperty('--rdt-ui-font-size', value.fontSizePx + 'px');
        document.body.classList.toggle('rdt-ui-compact', value.compactMargins);
    }

    function commitAndBroadcast() {
        applyPreferences(prefs);
        writeLocalPreferences(prefs);
        vscode.postMessage({
            command: COMMAND_SET,
            preferences: prefs,
        });
    }

    function syncControls() {
        if (!dom) {
            return;
        }
        dom.fontSizeRange.value = String(prefs.fontSizePx);
        dom.fontSizeValue.textContent = String(prefs.fontSizePx) + 'px';
        dom.compactMarginsToggle.checked = prefs.compactMargins;
    }

    function closePanel() {
        if (!dom || dom.panel.classList.contains('hidden')) {
            return;
        }
        dom.panel.classList.add('hidden');
        dom.button.setAttribute('aria-expanded', 'false');
    }

    function togglePanel() {
        if (!dom) {
            return;
        }
        const willOpen = dom.panel.classList.contains('hidden');
        dom.panel.classList.toggle('hidden', !willOpen);
        dom.button.setAttribute('aria-expanded', willOpen ? 'true' : 'false');
    }

    function installUi() {
        if (document.getElementById('rdtUiSettingsButton')) {
            return;
        }

        const targetToolbar = document.querySelector('.toolbar') || document.querySelector('.nv-toolbar');
        const mount = document.createElement('div');
        mount.className = 'rdt-ui-settings';

        const button = document.createElement('button');
        button.id = 'rdtUiSettingsButton';
        button.type = 'button';
        button.className = 'secondary small rdt-ui-settings-btn';
        button.textContent = '⚙ UI';
        button.title = 'Customize font size and spacing';
        button.setAttribute('aria-expanded', 'false');
        button.setAttribute('aria-controls', 'rdtUiSettingsPanel');

        const panel = document.createElement('div');
        panel.id = 'rdtUiSettingsPanel';
        panel.className = 'rdt-ui-settings-panel hidden';

        const title = document.createElement('h4');
        title.className = 'rdt-ui-settings-title';
        title.textContent = 'UI Settings';

        const fontRow = document.createElement('div');
        fontRow.className = 'rdt-ui-settings-row';
        const fontLabel = document.createElement('label');
        fontLabel.textContent = 'Font size';
        fontLabel.htmlFor = 'rdtUiFontSizeRange';
        const fontValue = document.createElement('span');
        fontValue.className = 'rdt-ui-settings-value';
        fontValue.textContent = String(prefs.fontSizePx) + 'px';
        fontRow.appendChild(fontLabel);
        fontRow.appendChild(fontValue);

        const fontRange = document.createElement('input');
        fontRange.id = 'rdtUiFontSizeRange';
        fontRange.type = 'range';
        fontRange.min = String(LIMITS.minFontSizePx);
        fontRange.max = String(LIMITS.maxFontSizePx);
        fontRange.step = '1';
        fontRange.value = String(prefs.fontSizePx);
        fontRange.className = 'rdt-ui-settings-range';

        const compactRow = document.createElement('label');
        compactRow.className = 'rdt-ui-settings-check';
        const compactToggle = document.createElement('input');
        compactToggle.type = 'checkbox';
        compactToggle.checked = prefs.compactMargins;
        const compactText = document.createElement('span');
        compactText.textContent = 'Compact margins';
        compactRow.appendChild(compactToggle);
        compactRow.appendChild(compactText);

        const actionsRow = document.createElement('div');
        actionsRow.className = 'rdt-ui-settings-actions';
        const resetButton = document.createElement('button');
        resetButton.type = 'button';
        resetButton.className = 'secondary small';
        resetButton.textContent = 'Reset';
        actionsRow.appendChild(resetButton);

        panel.appendChild(title);
        panel.appendChild(fontRow);
        panel.appendChild(fontRange);
        panel.appendChild(compactRow);
        panel.appendChild(actionsRow);

        mount.appendChild(button);
        mount.appendChild(panel);

        if (targetToolbar) {
            targetToolbar.appendChild(mount);
        } else {
            mount.classList.add('rdt-ui-settings-floating');
            document.body.appendChild(mount);
        }

        dom = {
            button,
            panel,
            fontSizeRange: fontRange,
            fontSizeValue: fontValue,
            compactMarginsToggle: compactToggle,
            resetButton,
            mount,
        };

        button.addEventListener('click', () => {
            togglePanel();
        });

        fontRange.addEventListener('input', () => {
            prefs.fontSizePx = clamp(
                Number(fontRange.value),
                LIMITS.minFontSizePx,
                LIMITS.maxFontSizePx,
            );
            fontValue.textContent = String(prefs.fontSizePx) + 'px';
            commitAndBroadcast();
        });

        compactToggle.addEventListener('change', () => {
            prefs.compactMargins = compactToggle.checked;
            commitAndBroadcast();
        });

        resetButton.addEventListener('click', () => {
            prefs = { ...DEFAULTS };
            syncControls();
            commitAndBroadcast();
        });

        document.addEventListener('click', (event) => {
            if (!dom || dom.panel.classList.contains('hidden')) {
                return;
            }
            const target = event.target;
            if (!(target instanceof Node)) {
                return;
            }
            if (!dom.mount.contains(target)) {
                closePanel();
            }
        });

        document.addEventListener('keydown', (event) => {
            if (event.key === 'Escape') {
                closePanel();
            }
        });
    }

    function onWebviewMessage(event) {
        const msg = event.data;
        if (!msg || msg.command !== COMMAND_STATE) {
            return;
        }
        prefs = normalize(msg.preferences);
        applyPreferences(prefs);
        writeLocalPreferences(prefs);
        syncControls();
    }

    function boot() {
        applyPreferences(prefs);
        installUi();
        syncControls();
        window.addEventListener('message', onWebviewMessage);
        vscode.postMessage({ command: COMMAND_REQUEST });
    }

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', boot, { once: true });
    } else {
        boot();
    }
})();
