/* Node Visualizer Webview Script */
(function () {
    const vscode = acquireVsCodeApi();

    const commands = Object.freeze({
        toHost: Object.freeze({
            REFRESH: 'refresh',
        }),
        toWebview: Object.freeze({
            LOADING: 'loading',
            GRAPH_DATA: 'graphData',
        }),
    });

    const canvas = document.getElementById('graphCanvas');
    const svgEl = document.getElementById('edgeSvg');
    const statusEl = document.getElementById('status');
    const detailEl = document.getElementById('detailsCard');

    let graphState = { nodes: [], topics: [], connections: {} };

    const escapeHtml = (value) => String(value ?? '').replace(/[&<>"']/g, (ch) => {
        if (ch === '&') {
            return '&amp;';
        }
        if (ch === '<') {
            return '&lt;';
        }
        if (ch === '>') {
            return '&gt;';
        }
        if (ch === '"') {
            return '&quot;';
        }
        return '&#39;';
    });

    document.getElementById('btnRefresh').addEventListener('click', () => {
        vscode.postMessage({ command: commands.toHost.REFRESH });
    });

    document.getElementById('btnAutoLayout').addEventListener('click', () => {
        renderGraph(graphState);
    });

    window.addEventListener('message', (event) => {
        const msg = event.data;
        if (msg.command === commands.toWebview.LOADING) {
            statusEl.innerHTML = '<span class="spinner"></span> Fetching graph data…';
            return;
        }
        if (msg.command === commands.toWebview.GRAPH_DATA) {
            graphState = msg;
            statusEl.textContent = msg.nodes.length + ' nodes, ' + msg.topics.length + ' topics';
            renderGraph(msg);
        }
    });

    function renderGraph(data) {
        canvas.querySelectorAll('.graph-node').forEach((el) => el.remove());
        svgEl.innerHTML = '';

        const nodePositions = {};
        const topicPositions = {};

        const canvasW = canvas.clientWidth || 360;
        const nodeCount = data.nodes.length;
        const topicCount = data.topics.length;

        data.nodes.forEach((name, i) => {
            const el = document.createElement('div');
            el.className = 'graph-node node-type';
            el.textContent = name;
            el.title = name;
            const x = 20;
            const y = 20 + i * 50;
            el.style.left = x + 'px';
            el.style.top = y + 'px';
            el.addEventListener('click', () => showNodeDetails(name, data.connections[name]));
            canvas.appendChild(el);
            nodePositions[name] = { x: x + el.offsetWidth, y: y + 12 };
        });

        data.topics.forEach((topic, i) => {
            const el = document.createElement('div');
            el.className = 'graph-node topic-type';
            el.textContent = topic.name;
            el.title = topic.type;
            const x = Math.max(canvasW - 160, 200);
            const y = 20 + i * 50;
            el.style.left = x + 'px';
            el.style.top = y + 'px';
            el.addEventListener('click', () => showTopicDetails(topic));
            canvas.appendChild(el);
            topicPositions[topic.name] = { x, y: y + 12 };
        });

        const totalHeight = Math.max(
            (nodeCount * 50) + 40,
            (topicCount * 50) + 40,
            300,
        );
        canvas.style.minHeight = totalHeight + 'px';
        svgEl.setAttribute('viewBox', '0 0 ' + canvasW + ' ' + totalHeight);
        svgEl.style.width = canvasW + 'px';
        svgEl.style.height = totalHeight + 'px';

        for (const [node, info] of Object.entries(data.connections)) {
            const nPos = nodePositions[node];
            if (!nPos) {
                continue;
            }

            (info.publishers || []).forEach((topic) => {
                const tPos = topicPositions[topic];
                if (!tPos) {
                    return;
                }
                drawEdge(nPos, tPos, '#27ae60');
            });

            (info.subscribers || []).forEach((topic) => {
                const tPos = topicPositions[topic];
                if (!tPos) {
                    return;
                }
                drawEdge(tPos, nPos, '#e67e22');
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
            '<strong>' + escapeHtml(name) + '</strong> (node)' +
            '<br><br><em>Publishes:</em> ' + (pubs.length ? pubs.map(escapeHtml).join(', ') : '—') +
            '<br><em>Subscribes:</em> ' + (subs.length ? subs.map(escapeHtml).join(', ') : '—');
    }

    function showTopicDetails(topic) {
        detailEl.innerHTML =
            '<strong>' + escapeHtml(topic.name) + '</strong> (topic)' +
            '<br>Type: <code>' + escapeHtml(topic.type) + '</code>';
    }

    vscode.postMessage({ command: commands.toHost.REFRESH });
})();
