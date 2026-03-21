import fs from 'node:fs';
import path from 'node:path';
import { describe, expect, it } from 'vitest';

function readSharedStyles(): string {
    return fs.readFileSync(
        path.resolve(__dirname, '..', '..', '..', 'media', 'style.css'),
        'utf8',
    );
}

describe('shared modal styles', () => {
    it('keeps modal dialogs scrollable within the viewport', () => {
        const styles = readSharedStyles();

        expect(styles).toMatch(/\.modal\s*\{[^}]*padding:\s*12px;[^}]*overflow-y:\s*auto;[^}]*overscroll-behavior:\s*contain;[^}]*\}/s);
        expect(styles).toMatch(/\.modal-card\s*\{[^}]*max-width:\s*calc\(100vw - 24px\);[^}]*max-width:\s*calc\(100dvw - 24px\);[^}]*max-height:\s*calc\(100vh - 24px\);[^}]*max-height:\s*calc\(100dvh - 24px\);[^}]*display:\s*flex;[^}]*flex-direction:\s*column;[^}]*overflow:\s*hidden;[^}]*\}/s);
        expect(styles).toMatch(/\.modal-body\s*\{[^}]*flex:\s*1 1 auto;[^}]*min-height:\s*0;[^}]*overflow-y:\s*auto;[^}]*\}/s);
    });
});
