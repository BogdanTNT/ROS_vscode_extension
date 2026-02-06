import * as vscode from 'vscode';
import { RosWorkspace } from '../ros/rosWorkspace';
import { getWebviewHtml } from './webviewHelper';

/**
 * Sidebar webview that visualises running ROS nodes, topics, and their
 * publish/subscribe connections as an interactive graph.
 */
export class NodeVisualizerViewProvider implements vscode.WebviewViewProvider {
    private _view?: vscode.WebviewView;

    constructor(
        private readonly _extensionUri: vscode.Uri,
        private readonly _ros: RosWorkspace,
    ) {}

    resolveWebviewView(
        webviewView: vscode.WebviewView,
        _context: vscode.WebviewViewResolveContext,
        _token: vscode.CancellationToken,
    ): void {
        this._view = webviewView;

        webviewView.webview.options = {
            enableScripts: true,
            localResourceRoots: [this._extensionUri],
        };

        webviewView.webview.html = this._getHtml(webviewView.webview);

        webviewView.webview.onDidReceiveMessage(async (msg) => {
            switch (msg.command) {
                case 'refresh':
                    await this._sendGraphData();
                    break;
            }
        });

        this._sendGraphData();
    }

    /** Externally triggered refresh (e.g. from command palette). */
    refreshGraph(): void {
        this._view?.show(true);
        this._sendGraphData();
    }

    // ── Private ────────────────────────────────────────────────
    private async _sendGraphData() {
        this._view?.webview.postMessage({ command: 'loading' });

        const nodes = await this._ros.getNodeList();
        const topics = await this._ros.getTopicList();

        const nodeEntries = await Promise.all(
            nodes.map(async (node) => {
                const info = await this._ros.getNodeInfo(node);
                return [node, info] as const;
            }),
        );
        const connections: Record<string, { publishers: string[]; subscribers: string[] }> =
            Object.fromEntries(nodeEntries);

        this._view?.webview.postMessage({
            command: 'graphData',
            nodes,
            topics,
            connections,
        });
    }

    private _getHtml(webview: vscode.Webview): string {
        const body = /* html */ `
<h2>🔗 Node Visualizer</h2>

<div class="btn-row mb">
    <button id="btnRefresh">↻ Refresh Graph</button>
    <button class="secondary" id="btnAutoLayout">⊞ Auto-Layout</button>
</div>

<div id="status" class="mb text-muted text-sm">Connecting to ROS…</div>

<div class="graph-canvas" id="graphCanvas">
    <svg id="edgeSvg" style="position:absolute;top:0;left:0;width:100%;height:100%;z-index:1;pointer-events:none;"></svg>
</div>

<div class="section mt">
    <h3>Details</h3>
    <div class="card" id="detailsCard">
        <span class="text-muted">Click a node or topic above to see details.</span>
    </div>
</div>

<!-- Legend -->
<div class="section mt">
    <h3>Legend</h3>
    <div style="display:flex;gap:12px;align-items:center;flex-wrap:wrap;">
        <span class="graph-node node-type" style="position:static;">Node</span>
        <span class="graph-node topic-type" style="position:static;">Topic</span>
        <span class="text-sm text-muted">→ publishes &nbsp; ← subscribes</span>
    </div>
</div>
`;
        const scriptUris = [
            vscode.Uri.joinPath(this._extensionUri, 'media', 'nodeVisualizer', 'index.js'),
        ];
        return getWebviewHtml(webview, this._extensionUri, body, '', undefined, scriptUris);
    }
}
