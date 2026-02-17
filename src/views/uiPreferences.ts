export interface WebviewUiPreferences {
    fontSizePx: number;
    compactMargins: boolean;
}

export const UI_PREFERENCES_KEY = 'rosDevToolkit.webviewUiPreferences';

export const DEFAULT_WEBVIEW_UI_PREFERENCES: Readonly<WebviewUiPreferences> = {
    fontSizePx: 13,
    compactMargins: false,
};

export function normalizeWebviewUiPreferences(raw: unknown): WebviewUiPreferences {
    if (!raw || typeof raw !== 'object') {
        return { ...DEFAULT_WEBVIEW_UI_PREFERENCES };
    }

    const candidate = raw as Partial<WebviewUiPreferences>;
    const numericFontSize = Number(candidate.fontSizePx);
    const fontSizePx = Number.isFinite(numericFontSize)
        ? clamp(Math.round(numericFontSize), 10, 24)
        : DEFAULT_WEBVIEW_UI_PREFERENCES.fontSizePx;

    return {
        fontSizePx,
        compactMargins: candidate.compactMargins === true,
    };
}

function clamp(value: number, min: number, max: number): number {
    return Math.min(max, Math.max(min, value));
}
