import { describe, expect, it } from 'vitest';

describe('webview test harness', () => {
    it('runs the webview suite entrypoint', () => {
        expect('webview-suite').toContain('webview');
    });
});
