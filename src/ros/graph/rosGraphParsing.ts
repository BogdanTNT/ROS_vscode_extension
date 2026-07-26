/**
 * Pure parsers for ROS 1/2 CLI text output (node/topic/service/action/parameter
 * lists, `node info` graphs, topic role listings and interface definitions).
 *
 * Extracted from {@link RosWorkspace}: these functions take raw command stdout
 * and return structured data with no side effects, which keeps the workspace
 * class focused on process orchestration and makes the parsers directly
 * unit-testable.
 */
import type {
    RosGraphEntityInfo,
    RosParameterInfo,
    RosNodeGraphInfo,
} from '../rosWorkspace';

type NodeGraphSection =
    | 'pub'
    | 'sub'
    | 'srvServer'
    | 'srvClient'
    | 'actionServer'
    | 'actionClient';

export function parseEntityList(raw: string): RosGraphEntityInfo[] {
    // ROS 2 uses "<name> [<type>]" while ROS 1 usually returns just "<name>".
    // This parser accepts both formats and normalizes duplicates by name.
    const lines = raw.split('\n').map((line) => line.trim()).filter(Boolean);
    const entities: RosGraphEntityInfo[] = [];
    const seen = new Set<string>();

    for (const line of lines) {
        const typedMatch = line.match(/^(\S+)\s+\[(.+)\]$/);
        const name = typedMatch?.[1] ?? line;
        const type = typedMatch?.[2] ?? 'unknown';

        if (!name.startsWith('/')) {
            continue;
        }
        if (seen.has(name)) {
            continue;
        }
        seen.add(name);
        entities.push({ name, type });
    }

    return entities;
}

export function parseNodeList(raw: string): { nodes: string[]; warnings: string[] } {
    const nodes: string[] = [];
    const warnings: string[] = [];

    for (const line of raw.split('\n')) {
        const trimmed = line.trim();
        if (!trimmed) {
            continue;
        }
        if (trimmed.startsWith('/')) {
            nodes.push(trimmed);
            continue;
        }
        if (/^warning\b/i.test(trimmed)) {
            warnings.push(trimmed);
            continue;
        }
        warnings.push(trimmed);
    }

    return { nodes, warnings };
}

export function parseRos2ParameterList(raw: string): RosParameterInfo[] {
    const params: RosParameterInfo[] = [];
    const seen = new Set<string>();
    let currentNode = '';

    for (const line of raw.split('\n')) {
        const trimmed = line.trim();
        if (!trimmed) {
            continue;
        }

        if (trimmed.startsWith('/') && trimmed.endsWith(':')) {
            const nodeName = trimmed.slice(0, -1).trim();
            currentNode = isValidRosResourceName(nodeName) ? nodeName : '';
            continue;
        }

        if (!currentNode) {
            continue;
        }

        const paramName = trimmed.replace(/^-+\s*/, '').trim();
        if (!isValidRosParameterName(paramName)) {
            continue;
        }

        const key = `${currentNode}:${paramName}`;
        if (seen.has(key)) {
            continue;
        }
        seen.add(key);
        params.push({ name: paramName, node: currentNode });
    }

    return params.sort(
        (a, b) => (a.node ?? '').localeCompare(b.node ?? '') || a.name.localeCompare(b.name),
    );
}

export function parseRos1ParameterList(raw: string): RosParameterInfo[] {
    const params: RosParameterInfo[] = [];
    const seen = new Set<string>();

    for (const line of raw.split('\n')) {
        const trimmed = line.trim();
        if (!trimmed || !isValidRosResourceName(trimmed) || seen.has(trimmed)) {
            continue;
        }
        seen.add(trimmed);
        params.push({ name: trimmed });
    }

    return params.sort((a, b) => a.name.localeCompare(b.name));
}

export function parseNodeGraphInfo(raw: string): RosNodeGraphInfo {
    // `ros2 node info` and `rosnode info` are section-based text outputs.
    // We track the active section while scanning and route each discovered
    // endpoint to its matching relation bucket.
    const publishers: string[] = [];
    const subscribers: string[] = [];
    const serviceServers: string[] = [];
    const serviceClients: string[] = [];
    const actionServers: string[] = [];
    const actionClients: string[] = [];

    let section: NodeGraphSection | null = null;

    for (const line of raw.split('\n')) {
        const trimmed = line.trim();
        if (!trimmed) {
            continue;
        }

        const lower = trimmed.toLowerCase();
        if (
            lower === 'publishers:' ||
            lower === 'publications:' ||
            lower.startsWith('publishers:')
        ) {
            section = 'pub';
            continue;
        }
        if (
            lower === 'subscribers:' ||
            lower === 'subscriptions:' ||
            lower.startsWith('subscribers:')
        ) {
            section = 'sub';
            continue;
        }
        if (lower === 'service servers:' || lower.startsWith('service servers:')) {
            section = 'srvServer';
            continue;
        }
        if (lower === 'service clients:' || lower.startsWith('service clients:')) {
            section = 'srvClient';
            continue;
        }
        if (lower === 'services:' || lower.startsWith('services:')) {
            section = 'srvServer';
            continue;
        }
        if (lower === 'action servers:' || lower.startsWith('action servers:')) {
            section = 'actionServer';
            continue;
        }
        if (lower === 'action clients:' || lower.startsWith('action clients:')) {
            section = 'actionClient';
            continue;
        }

        const endpoint = parseNodeInfoEndpoint(trimmed);
        if (!endpoint || !section) {
            continue;
        }

        const normalizedEndpoint = normalizeGraphEndpoint(section, endpoint);

        if (section === 'pub') {
            publishers.push(normalizedEndpoint);
            continue;
        }
        if (section === 'sub') {
            subscribers.push(normalizedEndpoint);
            continue;
        }
        if (section === 'srvServer') {
            serviceServers.push(normalizedEndpoint);
            continue;
        }
        if (section === 'srvClient') {
            serviceClients.push(normalizedEndpoint);
            continue;
        }
        if (section === 'actionServer') {
            actionServers.push(normalizedEndpoint);
            continue;
        }
        actionClients.push(normalizedEndpoint);
    }

    return {
        publishers: Array.from(new Set(publishers)),
        subscribers: Array.from(new Set(subscribers)),
        serviceServers: Array.from(new Set(serviceServers)),
        serviceClients: Array.from(new Set(serviceClients)),
        actionServers: Array.from(new Set(actionServers)),
        actionClients: Array.from(new Set(actionClients)),
    };
}

export function parseNodeInfoEndpoint(trimmedLine: string): string | undefined {
    // Node info lines typically include one absolute ROS name first
    // (topic/service/action path). We extract that first path token.
    const pathMatch = trimmedLine.match(/(\/[^\s:[]+)/);
    if (!pathMatch?.[1]) {
        return undefined;
    }
    return pathMatch[1];
}

export function parseRos2TopicRoles(raw: string): { publishers: string[]; subscribers: string[] } {
    const publishers: string[] = [];
    const subscribers: string[] = [];
    let currentNodeName = '';

    for (const line of raw.split('\n')) {
        const trimmed = line.trim();
        if (!trimmed) {
            continue;
        }

        const nodePrefix = 'Node name:';
        if (trimmed.startsWith(nodePrefix)) {
            currentNodeName = trimmed.slice(nodePrefix.length).trim();
            continue;
        }

        const endpointPrefix = 'Endpoint type:';
        if (!trimmed.startsWith(endpointPrefix)) {
            continue;
        }

        const endpointType = trimmed.slice(endpointPrefix.length).trim().toUpperCase();
        const normalizedNodeName = normalizeTopicRoleNodeName(currentNodeName);
        if (!normalizedNodeName) {
            continue;
        }

        if (endpointType.includes('PUBLISHER')) {
            publishers.push(normalizedNodeName);
            continue;
        }
        if (endpointType.includes('SUBSCRIPTION')) {
            subscribers.push(normalizedNodeName);
        }
    }

    return {
        publishers: Array.from(new Set(publishers)),
        subscribers: Array.from(new Set(subscribers)),
    };
}

export function parseRos1TopicRoles(raw: string): { publishers: string[]; subscribers: string[] } {
    const publishers: string[] = [];
    const subscribers: string[] = [];
    let section: 'publishers' | 'subscribers' | null = null;

    for (const line of raw.split('\n')) {
        const trimmed = line.trim();
        if (!trimmed) {
            continue;
        }

        const lower = trimmed.toLowerCase();
        if (lower.startsWith('publishers:')) {
            section = 'publishers';
            continue;
        }
        if (lower.startsWith('subscribers:')) {
            section = 'subscribers';
            continue;
        }

        const nodeName = parseTopicRoleNodeEntry(trimmed);
        if (!nodeName || !section) {
            continue;
        }

        if (section === 'publishers') {
            publishers.push(nodeName);
            continue;
        }
        subscribers.push(nodeName);
    }

    return {
        publishers: Array.from(new Set(publishers)),
        subscribers: Array.from(new Set(subscribers)),
    };
}

export function parseTopicRoleNodeEntry(trimmedLine: string): string | undefined {
    if (!(trimmedLine.startsWith('*') || trimmedLine.startsWith('-'))) {
        return undefined;
    }

    const withoutBullet = trimmedLine.replace(/^[-*]\s*/, '').trim();
    if (!withoutBullet) {
        return undefined;
    }

    const token = withoutBullet.split(/\s+/)[0] ?? '';
    return normalizeTopicRoleNodeName(token);
}

export function normalizeTopicRoleNodeName(rawName: string): string | undefined {
    const trimmed = String(rawName || '').trim();
    if (!trimmed) {
        return undefined;
    }

    if (isValidRosResourceName(trimmed)) {
        return trimmed;
    }

    // ROS 2 topic info can print relative node names (without leading '/').
    if (/^[A-Za-z0-9_./~-]+$/.test(trimmed)) {
        const normalized = '/' + trimmed.replace(/^\/+/, '');
        return isValidRosResourceName(normalized) ? normalized : undefined;
    }

    return undefined;
}

export function isValidRosResourceName(name: string): boolean {
    // Reject whitespace and shell metacharacters because names are used
    // in CLI commands. ROS names we care about are absolute paths.
    return /^\/[A-Za-z0-9_./~-]+$/.test(name);
}

export function isValidRosParameterName(name: string): boolean {
    // ROS parameter names are identifiers with namespace separators, used
    // in CLI calls and therefore must not contain shell metacharacters.
    return /^[A-Za-z0-9_./~-]+$/.test(name);
}

export function isValidRosTypeName(typeName: string): boolean {
    // ROS type names are slash-delimited package/type tokens.
    return /^[A-Za-z][A-Za-z0-9_]*(?:\/[A-Za-z][A-Za-z0-9_]*)+$/.test(typeName);
}

export function buildMessageDefaultsFromInterface(rawInterface: string): Record<string, unknown> {
    type ParseContext = {
        indent: number;
        target: Record<string, unknown>;
    };

    const root: Record<string, unknown> = {};
    const stack: ParseContext[] = [{ indent: -1, target: root }];

    // `ros2 interface show` and `rosmsg show` present nested message fields
    // as indentation-based trees. We reconstruct that tree and assign simple
    // scalar defaults so users get an editable publish-ready payload.
    for (const line of rawInterface.split('\n')) {
        const withoutComment = line.split('#')[0].trimEnd();
        if (!withoutComment.trim()) {
            continue;
        }
        if (withoutComment.trim() === '---') {
            // Service request/response separator (not expected for topics),
            // but ignored defensively.
            continue;
        }

        const indent = withoutComment.length - withoutComment.trimStart().length;
        const trimmed = withoutComment.trim();
        const parts = trimmed.split(/\s+/);
        if (parts.length < 2) {
            continue;
        }

        const typeToken = parts[0];
        const fieldToken = parts[1];
        if (!fieldToken || fieldToken.includes('=')) {
            // Constant line (e.g. uint8 FOO=1), not a message field.
            continue;
        }
        if (!/^[A-Za-z][A-Za-z0-9_]*$/.test(fieldToken)) {
            continue;
        }

        while (stack.length > 1 && indent <= stack[stack.length - 1].indent) {
            stack.pop();
        }
        const parent = stack[stack.length - 1].target;

        const descriptor = parseInterfaceTypeDescriptor(typeToken);
        const primitiveDefault = getPrimitiveDefaultValue(descriptor.baseType);

        if (!descriptor.isArray && primitiveDefault !== undefined) {
            parent[fieldToken] = primitiveDefault;
            continue;
        }

        if (descriptor.isArray && primitiveDefault !== undefined) {
            if (descriptor.fixedLength && descriptor.fixedLength > 0) {
                parent[fieldToken] = Array.from({ length: descriptor.fixedLength }, () => primitiveDefault);
            } else {
                parent[fieldToken] = [];
            }
            continue;
        }

        // Complex message field. For arrays we pre-populate one sample item
        // so nested structure is visible/editable in the modal by default.
        const nestedTemplate: Record<string, unknown> = {};
        if (descriptor.isArray) {
            if (descriptor.fixedLength && descriptor.fixedLength > 0) {
                parent[fieldToken] = Array.from({ length: descriptor.fixedLength }, () => nestedTemplate);
            } else {
                parent[fieldToken] = [nestedTemplate];
            }
        } else {
            parent[fieldToken] = nestedTemplate;
        }

        stack.push({ indent, target: nestedTemplate });
    }

    return root;
}

export function extractFirstInterfaceSection(rawInterface: string): string {
    const lines: string[] = [];
    for (const line of String(rawInterface || '').split('\n')) {
        if (line.trim() === '---') {
            break;
        }
        lines.push(line);
    }
    return lines.join('\n');
}

export function parseInterfaceTypeDescriptor(typeToken: string): {
    baseType: string;
    isArray: boolean;
    fixedLength?: number;
} {
    const arrayMatch = typeToken.match(/^(.+)\[([^\]]*)\]$/);
    if (!arrayMatch) {
        return { baseType: typeToken, isArray: false };
    }

    const baseType = arrayMatch[1].trim();
    const lengthToken = arrayMatch[2].trim();
    const fixedLength = /^[0-9]+$/.test(lengthToken)
        ? parseInt(lengthToken, 10)
        : undefined;

    return { baseType, isArray: true, fixedLength };
}

export function getPrimitiveDefaultValue(baseType: string): string | number | boolean | undefined {
    const normalized = baseType.trim().toLowerCase();

    if (
        normalized === 'string'
        || normalized === 'wstring'
        || normalized.startsWith('string<=')
        || normalized.startsWith('wstring<=')
    ) {
        return '';
    }
    if (normalized === 'bool') {
        return false;
    }
    if (
        normalized === 'byte'
        || normalized === 'char'
        || normalized === 'float'
        || normalized === 'double'
        || /^u?int(8|16|32|64)$/.test(normalized)
        || /^float(32|64)$/.test(normalized)
    ) {
        return 0;
    }
    return undefined;
}

export function normalizeGraphEndpoint(
    section: NodeGraphSection,
    endpoint: string,
): string {
    if (section !== 'actionServer' && section !== 'actionClient') {
        return endpoint;
    }

    // Some ROS 2 node-info outputs expose internal action transport topics
    // (e.g. /fibonacci/_action/feedback). Normalize those back to /fibonacci
    // so they match `ros2 action list -t` entries used by the UI.
    const actionMatch = endpoint.match(/^(\/.+?)\/_action(?:\/.*)?$/);
    if (actionMatch?.[1]) {
        return actionMatch[1];
    }

    return endpoint;
}
