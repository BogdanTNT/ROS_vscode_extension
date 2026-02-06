import * as vscode from 'vscode';

/**
 * Returns a nonce string for Content-Security-Policy in webviews.
 */
export function getNonce(): string {
    let text = '';
    const chars = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789';
    for (let i = 0; i < 32; i++) {
        text += chars.charAt(Math.floor(Math.random() * chars.length));
    }
    return text;
}

/**
 * Wraps an HTML body with common boilerplate, CSP, and the shared stylesheet.
 */
export function getWebviewHtml(
    webview: vscode.Webview,
    extensionUri: vscode.Uri,
    body: string,
    scriptContent: string,
): string {
    const nonce = getNonce();
    const styleUri = webview.asWebviewUri(vscode.Uri.joinPath(extensionUri, 'media', 'style.css'));

    return /* html */ `<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta http-equiv="Content-Security-Policy"
          content="default-src 'none';
                   style-src ${webview.cspSource} 'unsafe-inline';
                   script-src 'nonce-${nonce}';">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <link rel="stylesheet" href="${styleUri}">
    <title>ROS Dev Toolkit</title>
</head>
<body>
    ${body}
    <script nonce="${nonce}">
        const vscode = acquireVsCodeApi();
        ${scriptContent}
    </script>
</body>
</html>`;
}
