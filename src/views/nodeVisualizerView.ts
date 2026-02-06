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

        // Build connection map: node → {publishers, subscribers}
        const connections: Record<string, { publishers: string[]; subscribers: string[] }> = {};
        for (const node of nodes) {
            connections[node] = await this._ros.getNodeInfo(node);
        }

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

        const script = /* js */ `
const canvas    = document.getElementById('graphCanvas');
const svgEl     = document.getElementById('edgeSvg');
const statusEl  = document.getElementById('status');
const detailEl  = document.getElementById('detailsCard');

let graphState = { nodes: [], topics: [], connections: {} };

document.getElementById('btnRefresh').addEventListener('click', () => {
    vscode.postMessage({ command: 'refresh' });
});

document.getElementById('btnAutoLayout').addEventListener('click', () => {
    renderGraph(graphState);
});

window.addEventListener('message', (event) => {
    const msg = event.data;
    if (msg.command === 'loading') {
        statusEl.innerHTML = '<span class="spinner"></span> Fetching graph data…';
    }
    if (msg.command === 'graphData') {
        graphState = msg;
        statusEl.textContent = msg.nodes.length + ' nodes, ' + msg.topics.length + ' topics';
        renderGraph(msg);
    }
});

function renderGraph(data) {
    // Clear old nodes (keep SVG)
    canvas.querySelectorAll('.graph-node').forEach(el => el.remove());
    svgEl.innerHTML = '';

    const nodePositions = {};
    const topicPositions = {};

    const canvasW = canvas.clientWidth || 360;
    const nodeCount = data.nodes.length;
    const topicCount = data.topics.length;

    // ── Place nodes on the left column ──
    data.nodes.forEach((name, i) => {
        const el = document.createElement('div');
        el.className = 'graph-node node-type';
        el.textContent = name;
        el.title = name;
        const x = 20;
        const y = 20 + i * 50;
        el.style.left = x + 'px';
        el.style.top  = y + 'px';
        el.addEventListener('click', () => showNodeDetails(name, data.connections[name]));
        canvas.appendChild(el);
        nodePositions[name] = { x: x + el.offsetWidth, y: y + 12 };
    });

    // ── Place topics on the right column ──
    data.topics.forEach((t, i) => {
        const el = document.createElement('div');
        el.className = 'graph-node topic-type';
        el.textContent = t.name;
        el.title = t.type;
        const x = Math.max(canvasW - 160, 200);
        const y = 20 + i * 50;
        el.style.left = x + 'px';
        el.style.top  = y + 'px';
        el.addEventListener('click', () => showTopicDetails(t));
        canvas.appendChild(el);
        topicPositions[t.name] = { x: x, y: y + 12 };
    });

    // ── Draw edges ──
    const totalHeight = Math.max(
        (nodeCount * 50) + 40,
        (topicCount * 50) + 40,
        300
    );
    canvas.style.minHeight = totalHeight + 'px';
    svgEl.setAttribute('viewBox', '0 0 ' + canvasW + ' ' + totalHeight);
    svgEl.style.width  = canvasW + 'px';
    svgEl.style.height = totalHeight + 'px';

    for (const [node, info] of Object.entries(data.connections)) {
        const nPos = nodePositions[node];
        if (!nPos) continue;

        (info.publishers || []).forEach(topic => {
            const tPos = topicPositions[topic];
            if (!tPos) return;
            drawEdge(nPos, tPos, '#27ae60'); // green = publishes
        });

        (info.subscribers || []).forEach(topic => {
            const tPos = topicPositions[topic];
            if (!tPos) return;
            drawEdge(tPos, nPos, '#e67e22'); // orange = subscribes
        });
    }
}

function drawEdge(from, to, color) {
    const line = document.createElementNS('http://www.w3.org/2000/svg', 'line');
    line.setAttribute('x1', from.x);
    line.setAttribute('y1', from.y);
    line.setAttribute('x2', to.x);
    line.setAttribute('y2', to.y);
    line.setAttribute('stroke', color);
    line.setAttribute('stroke-width', '1.5');
    line.setAttribute('stroke-opacity', '0.7');
    svgEl.appendChild(line);
}

function showNodeDetails(name, info) {
    const pubs = (info && info.publishers) || [];
    const subs = (info && info.subscribers) || [];
    detailEl.innerHTML =
        '<strong>' + name + '</strong> (node)' +
        '<br><br><em>Publishes:</em> ' + (pubs.length ? pubs.join(', ') : '—') +
        '<br><em>Subscribes:</em> ' + (subs.length ? subs.join(', ') : '—');
}

function showTopicDetails(topic) {
    detailEl.innerHTML =
        '<strong>' + topic.name + '</strong> (topic)' +
        '<br>Type: <code>' + topic.type + '</code>';
}

// Initial fetch
vscode.postMessage({ command: 'refresh' });
`;

        return getWebviewHtml(webview, this._extensionUri, body, script);
    }
}
