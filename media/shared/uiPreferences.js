/* Shared webview UI preferences (font size + spacing + panel style) */
(function () {
    const vscode = window.__rosVscodeApi || (window.__rosVscodeApi = acquireVsCodeApi());
    const LOCAL_STORAGE_KEY = 'rosDevToolkit.webviewUiPreferences.local';
    const COMMAND_REQUEST = 'requestUiPreferences';
    const COMMAND_SET = 'setUiPreferences';
    const COMMAND_STATE = 'uiPreferencesState';
    const STYLE_VARIANTS = Object.freeze({
        MODERN: 'modern',
        MODERN_COMPACT: 'modernCompact',
        UTILITARIAN: 'utilitarian',
    });
    const DEFAULTS = Object.freeze({
        fontSizePx: 13,
        compactMargins: false,
        styleVariant: STYLE_VARIANTS.MODERN,
        linkAcrossPanels: true,
    });
    const LIMITS = Object.freeze({
        minFontSizePx: 10,
        maxFontSizePx: 24,
    });
    const SHARED_STYLE_OPTIONS = Object.freeze([
        Object.freeze({ value: STYLE_VARIANTS.MODERN, label: 'Modern' }),
        Object.freeze({ value: STYLE_VARIANTS.MODERN_COMPACT, label: 'Modern (compact)' }),
        Object.freeze({ value: STYLE_VARIANTS.UTILITARIAN, label: 'Utilitarian (SCM)' }),
    ]);
    const PANEL_CONFIGS = Object.freeze([
        Object.freeze({
            id: 'packageManager',
            detectSelectors: Object.freeze(['#pkgList', '#otherPkgList', '#btnOpenCreate']),
            toolbarSelectors: Object.freeze(['.toolbar']),
            styleVariantAriaLabel: 'Package manager panel style',
            styleOptions: SHARED_STYLE_OPTIONS,
        }),
        Object.freeze({
            id: 'nodeVisualizer',
            detectSelectors: Object.freeze(['#entityList', '#detailsCard', '#overviewSections']),
            toolbarSelectors: Object.freeze(['.nv-toolbar', '.nv-details-header']),
            styleVariantAriaLabel: 'Node visualizer panel style',
            styleOptions: SHARED_STYLE_OPTIONS,
        }),
        Object.freeze({
            id: 'generic',
            detectSelectors: Object.freeze([]),
            toolbarSelectors: Object.freeze(['.toolbar', '.nv-toolbar']),
            styleVariantAriaLabel: 'Panel style',
            styleOptions: Object.freeze([]),
        }),
    ]);

    const panelConfig = resolvePanelConfig();

    let prefs = normalize(readLocalPreferences());
    let dom = null;

    function clamp(value, min, max) {
        return Math.min(max, Math.max(min, value));
    }

    function normalizeStyleVariant(raw, compactMargins) {
        if (raw === STYLE_VARIANTS.UTILITARIAN) {
            return STYLE_VARIANTS.UTILITARIAN;
        }
        if (raw === STYLE_VARIANTS.MODERN_COMPACT) {
            return STYLE_VARIANTS.MODERN_COMPACT;
        }
        if (compactMargins) {
            return STYLE_VARIANTS.MODERN_COMPACT;
        }
        return STYLE_VARIANTS.MODERN;
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
        const compactMargins = candidate.compactMargins === true;

        return {
            fontSizePx,
            compactMargins,
            styleVariant: normalizeStyleVariant(candidate.styleVariant, compactMargins),
            linkAcrossPanels: candidate.linkAcrossPanels !== false,
        };
    }

    function hasStyleVariantControl() {
        return panelConfig.styleOptions.length > 0;
    }

    function resolvePanelConfig() {
        for (const config of PANEL_CONFIGS) {
            if (config.detectSelectors.length === 0) {
                continue;
            }
            const allMatched = config.detectSelectors.every((selector) => Boolean(document.querySelector(selector)));
            if (allMatched) {
                return config;
            }
        }
        return PANEL_CONFIGS[PANEL_CONFIGS.length - 1];
    }

    function findToolbar(selectors) {
        for (const selector of selectors) {
            const toolbar = document.querySelector(selector);
            if (toolbar) {
                return toolbar;
            }
        }
        return null;
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

    function resolvePanelStyleVariant(value) {
        return normalizeStyleVariant(
            value?.styleVariant,
            value?.compactMargins === true,
        );
    }

    function resolveCompactMargins(value) {
        if (!hasStyleVariantControl()) {
            return value?.compactMargins === true;
        }
        return resolvePanelStyleVariant(value) === STYLE_VARIANTS.MODERN_COMPACT;
    }

    function applyStyleVariantClass(styleVariant) {
        document.body.classList.remove('rdt-ui-style-modern', 'rdt-ui-style-utilitarian');
        if (!hasStyleVariantControl()) {
            return;
        }
        const classVariant = styleVariant === STYLE_VARIANTS.UTILITARIAN
            ? STYLE_VARIANTS.UTILITARIAN
            : STYLE_VARIANTS.MODERN;
        document.body.classList.add('rdt-ui-style-' + classVariant);
    }

    function applyPreferences(value) {
        const panelStyle = resolvePanelStyleVariant(value);
        document.documentElement.style.setProperty('--rdt-ui-font-size', value.fontSizePx + 'px');
        document.body.classList.toggle('rdt-ui-compact', resolveCompactMargins(value));
        applyStyleVariantClass(panelStyle);
    }

    function commitAndBroadcast() {
        applyPreferences(prefs);
        writeLocalPreferences(prefs);
        if (prefs.linkAcrossPanels) {
            vscode.postMessage({
                command: COMMAND_SET,
                preferences: prefs,
            });
        }
    }

    function syncControls() {
        if (!dom) {
            return;
        }
        dom.fontSizeRange.value = String(prefs.fontSizePx);
        dom.fontSizeValue.textContent = String(prefs.fontSizePx) + 'px';
        if (dom.compactMarginsToggle) {
            dom.compactMarginsToggle.checked = prefs.compactMargins;
        }
        if (dom.styleVariantSelect) {
            dom.styleVariantSelect.value = resolvePanelStyleVariant(prefs);
        }
        if (dom.linkAcrossPanelsToggle) {
            dom.linkAcrossPanelsToggle.checked = prefs.linkAcrossPanels;
        }
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

        const canSelectStyle = hasStyleVariantControl();
        const targetToolbar = findToolbar(panelConfig.toolbarSelectors);
        const mount = document.createElement('div');
        mount.className = 'rdt-ui-settings';

        const button = document.createElement('button');
        button.id = 'rdtUiSettingsButton';
        button.type = 'button';
        button.className = 'secondary small rdt-ui-settings-btn';
        button.textContent = 'UI';
        button.title = canSelectStyle
            ? 'Customize font size and panel style'
            : 'Customize font size and spacing';
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

        let compactRow = null;
        let compactToggle = null;
        if (!canSelectStyle) {
            compactRow = document.createElement('label');
            compactRow.className = 'rdt-ui-settings-check';
            compactToggle = document.createElement('input');
            compactToggle.type = 'checkbox';
            compactToggle.checked = prefs.compactMargins;
            const compactText = document.createElement('span');
            compactText.textContent = 'Compact margins';
            compactRow.appendChild(compactToggle);
            compactRow.appendChild(compactText);
        }

        let styleVariantSelect = null;
        let styleVariantWrap = null;
        if (canSelectStyle) {
            styleVariantWrap = document.createElement('div');
            styleVariantWrap.className = 'rdt-ui-settings-field';
            const styleVariantLabel = document.createElement('label');
            styleVariantLabel.className = 'rdt-ui-settings-label';
            styleVariantLabel.htmlFor = 'rdtUiStyleVariantSelect';
            styleVariantLabel.textContent = 'Panel style';
            styleVariantSelect = document.createElement('select');
            styleVariantSelect.id = 'rdtUiStyleVariantSelect';
            styleVariantSelect.className = 'rdt-ui-settings-select';
            styleVariantSelect.setAttribute('aria-label', panelConfig.styleVariantAriaLabel);

            for (const option of panelConfig.styleOptions) {
                const optionNode = document.createElement('option');
                optionNode.value = option.value;
                optionNode.textContent = option.label;
                styleVariantSelect.appendChild(optionNode);
            }

            styleVariantWrap.appendChild(styleVariantLabel);
            styleVariantWrap.appendChild(styleVariantSelect);
        }

        const linkRow = document.createElement('label');
        linkRow.className = 'rdt-ui-settings-check';
        const linkToggle = document.createElement('input');
        linkToggle.type = 'checkbox';
        linkToggle.checked = prefs.linkAcrossPanels;
        const linkText = document.createElement('span');
        linkText.textContent = 'Link settings across panels';
        linkRow.appendChild(linkToggle);
        linkRow.appendChild(linkText);

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
        if (compactRow) {
            panel.appendChild(compactRow);
        }
        if (styleVariantWrap) {
            panel.appendChild(styleVariantWrap);
        }
        panel.appendChild(linkRow);
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
            styleVariantSelect,
            linkAcrossPanelsToggle: linkToggle,
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

        if (compactToggle) {
            compactToggle.addEventListener('change', () => {
                prefs.compactMargins = compactToggle.checked;
                if (prefs.styleVariant !== STYLE_VARIANTS.UTILITARIAN) {
                    prefs.styleVariant = compactToggle.checked
                        ? STYLE_VARIANTS.MODERN_COMPACT
                        : STYLE_VARIANTS.MODERN;
                }
                commitAndBroadcast();
            });
        }

        if (styleVariantSelect) {
            styleVariantSelect.addEventListener('change', () => {
                const nextVariant = normalizeStyleVariant(styleVariantSelect.value, false);
                prefs.styleVariant = nextVariant;
                prefs.compactMargins = nextVariant === STYLE_VARIANTS.MODERN_COMPACT;
                commitAndBroadcast();
            });
        }

        linkToggle.addEventListener('change', () => {
            prefs.linkAcrossPanels = linkToggle.checked;
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
        if (!prefs.linkAcrossPanels) {
            return;
        }
        prefs = normalize(msg.preferences);
        applyPreferences(prefs);
        writeLocalPreferences(prefs);
        syncControls();
    }

    function boot() {
        installUi();
        applyPreferences(prefs);
        syncControls();
        window.addEventListener('message', onWebviewMessage);
        if (prefs.linkAcrossPanels) {
            vscode.postMessage({ command: COMMAND_REQUEST });
        }
    }

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', boot, { once: true });
    } else {
        boot();
    }
})();
