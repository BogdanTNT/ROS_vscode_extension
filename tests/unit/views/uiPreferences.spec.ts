import { describe, expect, it } from 'vitest';
import {
    DEFAULT_WEBVIEW_UI_PREFERENCES,
    UI_PREFERENCES_KEY,
    loadWebviewUiPreferences,
    normalizeWebviewUiPreferences,
    saveWebviewUiPreferences,
} from '../../../src/views/uiPreferences';

describe('normalizeWebviewUiPreferences', () => {
    it('returns defaults when input is invalid', () => {
        expect(normalizeWebviewUiPreferences(undefined)).toEqual(DEFAULT_WEBVIEW_UI_PREFERENCES);
        expect(normalizeWebviewUiPreferences(null)).toEqual(DEFAULT_WEBVIEW_UI_PREFERENCES);
        expect(normalizeWebviewUiPreferences('invalid')).toEqual(DEFAULT_WEBVIEW_UI_PREFERENCES);
    });

    it('normalizes known values and clamps font size', () => {
        expect(
            normalizeWebviewUiPreferences({
                fontSizePx: 999,
                compactMargins: true,
                styleVariant: 'utilitarian',
            }),
        ).toEqual({
            fontSizePx: 24,
            compactMargins: true,
            styleVariant: 'utilitarian',
            linkAcrossPanels: true,
        });
    });

    it('maps legacy compact+modern state to modernCompact', () => {
        expect(
            normalizeWebviewUiPreferences({
                fontSizePx: 13,
                compactMargins: true,
                styleVariant: 'modern',
            }),
        ).toEqual({
            fontSizePx: 13,
            compactMargins: true,
            styleVariant: 'modernCompact',
            linkAcrossPanels: true,
        });
    });

    it('keeps explicit modernCompact style variant', () => {
        expect(
            normalizeWebviewUiPreferences({
                fontSizePx: 11,
                compactMargins: false,
                styleVariant: 'modernCompact',
            }),
        ).toEqual({
            fontSizePx: 11,
            compactMargins: false,
            styleVariant: 'modernCompact',
            linkAcrossPanels: true,
        });
    });

    it('falls back unknown style variant to modern', () => {
        expect(
            normalizeWebviewUiPreferences({
                fontSizePx: 12,
                compactMargins: false,
                styleVariant: 'legacy',
            }),
        ).toEqual({
            fontSizePx: 12,
            compactMargins: false,
            styleVariant: 'modern',
            linkAcrossPanels: true,
        });
    });

    it('allows disabling cross-panel linking', () => {
        expect(
            normalizeWebviewUiPreferences({
                fontSizePx: 12,
                compactMargins: false,
                styleVariant: 'modern',
                linkAcrossPanels: false,
            }),
        ).toEqual({
            fontSizePx: 12,
            compactMargins: false,
            styleVariant: 'modern',
            linkAcrossPanels: false,
        });
    });

    it('loads stored preferences with shared helper', () => {
        const loaded = loadWebviewUiPreferences({
            get: () => ({
                fontSizePx: 20,
                compactMargins: false,
                styleVariant: 'modernCompact',
                linkAcrossPanels: true,
            }),
        });

        expect(loaded).toEqual({
            fontSizePx: 20,
            compactMargins: false,
            styleVariant: 'modernCompact',
            linkAcrossPanels: true,
        });
    });

    it('normalizes and stores preferences with shared helper', async () => {
        let updatedKey = '';
        let updatedValue: unknown;
        const saved = await saveWebviewUiPreferences(
            {
                update: async (key, value) => {
                    updatedKey = key;
                    updatedValue = value;
                },
            },
            {
                fontSizePx: 80,
                compactMargins: true,
                styleVariant: 'legacy',
                linkAcrossPanels: false,
            },
        );

        expect(updatedKey).toBe(UI_PREFERENCES_KEY);
        expect(saved).toEqual({
            fontSizePx: 24,
            compactMargins: true,
            styleVariant: 'modernCompact',
            linkAcrossPanels: false,
        });
        expect(updatedValue).toEqual(saved);
    });
});
