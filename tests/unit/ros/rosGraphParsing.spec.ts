import { describe, expect, it } from 'vitest';
import {
    parseEntityList,
    parseNodeList,
    parseRos2ParameterList,
    parseRos1ParameterList,
    parseNodeGraphInfo,
    parseRos2TopicRoles,
    parseRos1TopicRoles,
    isValidRosResourceName,
    isValidRosParameterName,
    isValidRosTypeName,
    buildMessageDefaultsFromInterface,
    parseInterfaceTypeDescriptor,
    getPrimitiveDefaultValue,
    normalizeGraphEndpoint,
} from '../../../src/ros/graph/rosGraphParsing';

describe('rosGraphParsing/parseEntityList', () => {
    it('parses typed and untyped entries, dedups by name, and drops non-absolute names', () => {
        const raw = [
            '/topic1 [std_msgs/msg/String]',
            '/topic2',
            '/topic1 [duplicate]',
            'notslash',
        ].join('\n');
        expect(parseEntityList(raw)).toEqual([
            { name: '/topic1', type: 'std_msgs/msg/String' },
            { name: '/topic2', type: 'unknown' },
        ]);
    });
});

describe('rosGraphParsing/parseNodeList', () => {
    it('separates absolute node names from warnings', () => {
        const raw = ['/node1', '/node2', 'WARNING: discovery slow', 'stray line'].join('\n');
        expect(parseNodeList(raw)).toEqual({
            nodes: ['/node1', '/node2'],
            warnings: ['WARNING: discovery slow', 'stray line'],
        });
    });
});

describe('rosGraphParsing/parseRos2ParameterList', () => {
    it('groups parameters by node and sorts them', () => {
        const raw = [
            '/node_b:',
            '  param_z',
            '/node_a:',
            '  param_y',
            '  param_x',
            '  param_x',
        ].join('\n');
        expect(parseRos2ParameterList(raw)).toEqual([
            { name: 'param_x', node: '/node_a' },
            { name: 'param_y', node: '/node_a' },
            { name: 'param_z', node: '/node_b' },
        ]);
    });
});

describe('rosGraphParsing/parseRos1ParameterList', () => {
    it('dedups and sorts parameter names', () => {
        expect(parseRos1ParameterList('/foo\n/bar\n/foo\n')).toEqual([
            { name: '/bar' },
            { name: '/foo' },
        ]);
    });
});

describe('rosGraphParsing/parseNodeGraphInfo', () => {
    it('routes endpoints to their sections', () => {
        const raw = [
            'Publishers:',
            '  /chatter: std_msgs/msg/String',
            'Subscribers:',
            '  /cmd: std_msgs/msg/String',
            'Service Servers:',
            '  /add: example_interfaces/srv/AddTwoInts',
        ].join('\n');
        const graph = parseNodeGraphInfo(raw);
        expect(graph.publishers).toEqual(['/chatter']);
        expect(graph.subscribers).toEqual(['/cmd']);
        expect(graph.serviceServers).toEqual(['/add']);
        expect(graph.actionServers).toEqual([]);
    });
});

describe('rosGraphParsing/validity checks', () => {
    it('validates ROS resource names', () => {
        expect(isValidRosResourceName('/foo/bar')).toBe(true);
        expect(isValidRosResourceName('foo')).toBe(false);
        expect(isValidRosResourceName('/bad name')).toBe(false);
    });

    it('validates parameter names', () => {
        expect(isValidRosParameterName('use_sim_time')).toBe(true);
        expect(isValidRosParameterName('bad param')).toBe(false);
    });

    it('validates type names', () => {
        expect(isValidRosTypeName('std_msgs/msg/String')).toBe(true);
        expect(isValidRosTypeName('String')).toBe(false);
    });
});

describe('rosGraphParsing/interface defaults', () => {
    it('assigns scalar defaults for primitive fields', () => {
        const raw = ['int32 count', 'string label', 'bool active'].join('\n');
        expect(buildMessageDefaultsFromInterface(raw)).toEqual({
            count: 0,
            label: '',
            active: false,
        });
    });

    it('handles fixed and dynamic arrays', () => {
        const raw = ['float64[3] coords', 'int32[] data'].join('\n');
        expect(buildMessageDefaultsFromInterface(raw)).toEqual({
            coords: [0, 0, 0],
            data: [],
        });
    });

    it('parses type descriptors', () => {
        expect(parseInterfaceTypeDescriptor('int32')).toEqual({ baseType: 'int32', isArray: false });
        expect(parseInterfaceTypeDescriptor('float64[3]')).toEqual({ baseType: 'float64', isArray: true, fixedLength: 3 });
        expect(parseInterfaceTypeDescriptor('float64[]')).toEqual({ baseType: 'float64', isArray: true, fixedLength: undefined });
    });

    it('maps primitive base types to defaults', () => {
        expect(getPrimitiveDefaultValue('string')).toBe('');
        expect(getPrimitiveDefaultValue('bool')).toBe(false);
        expect(getPrimitiveDefaultValue('int32')).toBe(0);
        expect(getPrimitiveDefaultValue('float64')).toBe(0);
        expect(getPrimitiveDefaultValue('geometry_msgs/Point')).toBeUndefined();
    });
});

describe('rosGraphParsing/normalizeGraphEndpoint', () => {
    it('collapses action transport topics back to the action name', () => {
        expect(normalizeGraphEndpoint('actionServer', '/fibonacci/_action/feedback')).toBe('/fibonacci');
        expect(normalizeGraphEndpoint('pub', '/chatter')).toBe('/chatter');
    });
});

describe('rosGraphParsing/topic roles', () => {
    it('parses ROS 2 topic role output', () => {
        const raw = [
            'Node name: talker',
            'Endpoint type: PUBLISHER',
            'Node name: listener',
            'Endpoint type: SUBSCRIPTION',
        ].join('\n');
        expect(parseRos2TopicRoles(raw)).toEqual({
            publishers: ['/talker'],
            subscribers: ['/listener'],
        });
    });

    it('parses ROS 1 topic role output', () => {
        const raw = ['Publishers:', ' * /talker', 'Subscribers:', ' * /listener'].join('\n');
        expect(parseRos1TopicRoles(raw)).toEqual({
            publishers: ['/talker'],
            subscribers: ['/listener'],
        });
    });
});
