export type WebviewUiStyleVariant = 'modern' | 'modernCompact' | 'utilitarian';

export interface WebviewUiPreferences {
    fontSizePx: number;
    compactMargins: boolean;
    styleVariant: WebviewUiStyleVariant;
    linkAcrossPanels: boolean;
}

export interface WebviewUiPreferencesReadStore {
    get<T>(key: string, defaultValue: T): T;
}

export interface WebviewUiPreferencesWriteStore {
    update(key: string, value: WebviewUiPreferences): PromiseLike<void>;
}

export const UI_PREFERENCES_KEY = 'rosDevToolkit.webviewUiPreferences';

export const DEFAULT_WEBVIEW_UI_PREFERENCES: Readonly<WebviewUiPreferences> = {
    fontSizePx: 13,
    compactMargins: false,
    styleVariant: 'modern',
    linkAcrossPanels: true,
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
    const compactMargins = candidate.compactMargins === true;
    const styleVariant = normalizeStyleVariant(candidate.styleVariant, compactMargins);

    return {
        fontSizePx,
        compactMargins,
        styleVariant,
        linkAcrossPanels: candidate.linkAcrossPanels !== false,
    };
}

export function loadWebviewUiPreferences(store: WebviewUiPreferencesReadStore): WebviewUiPreferences {
    const stored = store.get<Partial<WebviewUiPreferences>>(UI_PREFERENCES_KEY, {});
    return normalizeWebviewUiPreferences(stored);
}

export async function saveWebviewUiPreferences(
    store: WebviewUiPreferencesWriteStore,
    raw: unknown,
): Promise<WebviewUiPreferences> {
    const normalized = normalizeWebviewUiPreferences(raw);
    await store.update(UI_PREFERENCES_KEY, normalized);
    return normalized;
}

function normalizeStyleVariant(raw: unknown, compactMargins: boolean): WebviewUiStyleVariant {
    if (raw === 'utilitarian') {
        return 'utilitarian';
    }
    if (raw === 'modernCompact') {
        return 'modernCompact';
    }
    if (compactMargins) {
        return 'modernCompact';
    }
    return 'modern';
}

function clamp(value: number, min: number, max: number): number {
    return Math.min(max, Math.max(min, value));
}
