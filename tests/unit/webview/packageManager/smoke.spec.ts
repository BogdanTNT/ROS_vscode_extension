import { describe, expect, it, vi } from 'vitest';
import { Uri } from '../../../helpers/mocks/vscode';
import { getNonce, getWebviewHtml } from '../../../../src/views/webviewHelper';

vi.mock('vscode', () => import('../../../helpers/mocks/vscode'));

describe('webview helper smoke', () => {
    it('generates a 32-char alphanumeric nonce', () => {
        const nonce = getNonce();

        expect(nonce).toHaveLength(32);
        expect(nonce).toMatch(/^[A-Za-z0-9]{32}$/);
    });

    it('builds HTML with CSP, stylesheet, and script tags using a shared nonce', () => {
        const extensionUri = Uri.file('/tmp/ros-dev-toolkit-ext');
        const legacyScriptUri = Uri.file('/tmp/ros-dev-toolkit-ext/media/legacy.js');
        const scriptUris = [
            Uri.file('/tmp/ros-dev-toolkit-ext/media/shared.js'),
            Uri.file('/tmp/ros-dev-toolkit-ext/media/page.js'),
        ];

        const webview = {
            cspSource: 'vscode-webview://abc123',
            asWebviewUri: (uri: Uri) => `webview://${uri.fsPath}`,
        };

        const html = getWebviewHtml(
            webview as unknown as Parameters<typeof getWebviewHtml>[0],
            extensionUri,
            '<div id="root">Package manager</div>',
            'window.__pmBoot = true;',
            legacyScriptUri,
            scriptUris,
        );

        expect(html).toContain("content=\"default-src 'none';");
        expect(html).toContain("style-src vscode-webview://abc123 'unsafe-inline';");
        expect(html).toContain('webview:///tmp/ros-dev-toolkit-ext/media/style.css');
        expect(html).toContain('webview:///tmp/ros-dev-toolkit-ext/media/legacy.js');
        expect(html).toContain('webview:///tmp/ros-dev-toolkit-ext/media/shared.js');
        expect(html).toContain('webview:///tmp/ros-dev-toolkit-ext/media/page.js');
        expect(html).toContain('<div id="root">Package manager</div>');
        expect(html).toContain('window.__pmBoot = true;');

        const nonceMatch = html.match(/script-src 'nonce-([^']+)'/);
        expect(nonceMatch?.[1]).toBeDefined();
        const nonce = nonceMatch![1];
        expect(html).toContain(`nonce="${nonce}"`);
    });
});
