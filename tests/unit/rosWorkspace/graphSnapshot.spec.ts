import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { __resetMockState } from '../../helpers/mocks/vscode';
import { RosWorkspace } from '../../../src/ros/rosWorkspace';

vi.mock('vscode', () => import('../../helpers/mocks/vscode'));

type GraphSnapshotKey = 'nodes' | 'topics' | 'services' | 'actions' | 'parameters';

function getMarkerPrefix(command: string): string {
    const match = command.match(/(__RDT_GRAPH_SNAPSHOT__\d+_\d+__)/);
    if (!match?.[1]) {
        throw new Error(`Marker prefix was missing from command: ${command}`);
    }
    return match[1];
}

function buildSection(
    markerPrefix: string,
    key: GraphSnapshotKey,
    status: number,
    output: string,
): string {
    const lines = [
        `${markerPrefix}BEGIN:${key}`,
    ];
    if (output) {
        lines.push(output);
    }
    lines.push(`${markerPrefix}STATUS:${key}:${status}`);
    lines.push(`${markerPrefix}END:${key}`);
    return lines.join('\n');
}

describe('RosWorkspace graph snapshot batching', () => {
    beforeEach(() => {
        __resetMockState();
    });

    afterEach(() => {
        vi.restoreAllMocks();
        __resetMockState();
    });

    it('fetches all requested graph categories in a single composite command and parses results', async () => {
        const ros = new RosWorkspace();
        (ros as unknown as { _env: unknown })._env = {
            distro: 'jazzy',
            version: 2,
            workspacePath: '/tmp/ws',
        };

        const execSpy = vi.spyOn(ros as unknown as { exec: (cmd: string) => Promise<string> }, 'exec')
            .mockImplementation(async (command: string) => {
                const markerPrefix = getMarkerPrefix(command);
                return [
                    buildSection(markerPrefix, 'nodes', 0, '/talker\n/listener'),
                    buildSection(markerPrefix, 'topics', 0, '/chatter [std_msgs/msg/String]'),
                    buildSection(markerPrefix, 'services', 0, '/reset [std_srvs/srv/Trigger]'),
                    buildSection(markerPrefix, 'actions', 0, '/navigate_to_pose [nav2_msgs/action/NavigateToPose]'),
                    buildSection(markerPrefix, 'parameters', 0, '/talker:\n  use_sim_time'),
                ].join('\n');
            });

        const snapshot = await ros.getGraphSnapshot({
            nodes: true,
            topics: true,
            services: true,
            actions: true,
            parameters: true,
        });

        expect(execSpy).toHaveBeenCalledTimes(1);
        expect(snapshot.nodes).toEqual({
            ok: true,
            data: ['/talker', '/listener'],
        });
        expect(snapshot.topics).toEqual({
            ok: true,
            data: [{ name: '/chatter', type: 'std_msgs/msg/String' }],
        });
        expect(snapshot.services).toEqual({
            ok: true,
            data: [{ name: '/reset', type: 'std_srvs/srv/Trigger' }],
        });
        expect(snapshot.actions).toEqual({
            ok: true,
            data: [{ name: '/navigate_to_pose', type: 'nav2_msgs/action/NavigateToPose' }],
        });
        expect(snapshot.parameters).toEqual({
            ok: true,
            data: [{ name: 'use_sim_time', node: '/talker' }],
        });
    });

    it('builds a valid parallel composite command without background separator syntax errors', async () => {
        const ros = new RosWorkspace();
        (ros as unknown as { _env: unknown })._env = {
            distro: 'jazzy',
            version: 2,
            workspacePath: '/tmp/ws',
        };

        const execSpy = vi.spyOn(ros as unknown as { exec: (cmd: string) => Promise<string> }, 'exec')
            .mockImplementation(async (command: string) => {
                const markerPrefix = getMarkerPrefix(command);
                return [
                    buildSection(markerPrefix, 'nodes', 0, '/talker'),
                ].join('\n');
            });

        await ros.getGraphSnapshot({
            nodes: true,
            topics: false,
            services: false,
            actions: false,
            parameters: false,
        });

        const compositeCommand = String(execSpy.mock.calls[0]?.[0] || '');
        expect(compositeCommand).not.toContain('&;');
        expect(compositeCommand).toContain('wait');
    });

    it('reports per-category failures without clearing successful categories', async () => {
        const ros = new RosWorkspace();
        (ros as unknown as { _env: unknown })._env = {
            distro: 'jazzy',
            version: 2,
            workspacePath: '/tmp/ws',
        };

        vi.spyOn(ros as unknown as { exec: (cmd: string) => Promise<string> }, 'exec')
            .mockImplementation(async (command: string) => {
                const markerPrefix = getMarkerPrefix(command);
                return [
                    buildSection(markerPrefix, 'nodes', 0, '/talker'),
                    buildSection(markerPrefix, 'topics', 124, 'Command timed out'),
                    buildSection(markerPrefix, 'services', 0, '/reset [std_srvs/srv/Trigger]'),
                ].join('\n');
            });

        const snapshot = await ros.getGraphSnapshot({
            nodes: true,
            topics: true,
            services: true,
            actions: false,
            parameters: false,
        });

        expect(snapshot.nodes).toEqual({
            ok: true,
            data: ['/talker'],
        });
        expect(snapshot.services).toEqual({
            ok: true,
            data: [{ name: '/reset', type: 'std_srvs/srv/Trigger' }],
        });
        expect(snapshot.topics.ok).toBe(false);
        expect(snapshot.topics.data).toEqual([]);
        expect(snapshot.topics.error).toContain('Topics graph query failed');
    });

    it('keeps duplicate-name warnings out of the node list and exposes them as warnings', async () => {
        const ros = new RosWorkspace();
        (ros as unknown as { _env: unknown })._env = {
            distro: 'jazzy',
            version: 2,
            workspacePath: '/tmp/ws',
        };

        vi.spyOn(ros as unknown as { exec: (cmd: string) => Promise<string> }, 'exec')
            .mockImplementation(async (command: string) => {
                const markerPrefix = getMarkerPrefix(command);
                return buildSection(
                    markerPrefix,
                    'nodes',
                    0,
                    [
                        'WARNING: Be aware that there are nodes in the graph that share an exact name.',
                        '/talker',
                        '/listener',
                    ].join('\n'),
                );
            });

        const snapshot = await ros.getGraphSnapshot({
            nodes: true,
            topics: false,
            services: false,
            actions: false,
            parameters: false,
        });

        expect(snapshot.nodes).toEqual({
            ok: true,
            data: ['/talker', '/listener'],
            warnings: ['WARNING: Be aware that there are nodes in the graph that share an exact name.'],
        });
    });

    it('fails scoped categories when composite execution throws', async () => {
        const ros = new RosWorkspace();
        vi.spyOn(
            ros as unknown as {
                execGraphSnapshotCompositeCommand: (
                    compositeCommand: string,
                    context: unknown,
                    sectionCount: number,
                ) => Promise<string>;
            },
            'execGraphSnapshotCompositeCommand',
        ).mockRejectedValue(new Error('composite timeout'));

        const snapshot = await ros.getGraphSnapshot({
            nodes: true,
            topics: true,
            services: false,
            actions: false,
            parameters: false,
        });

        expect(snapshot.nodes.ok).toBe(false);
        expect(snapshot.nodes.error).toContain('composite timeout');
        expect(snapshot.topics.ok).toBe(false);
        expect(snapshot.topics.error).toContain('composite timeout');
        expect(snapshot.services).toEqual({ ok: true, data: [] });
    });

    it('handles ROS 1 snapshots and keeps actions category as successful empty data', async () => {
        const ros = new RosWorkspace();
        (ros as unknown as { _env: unknown })._env = {
            distro: 'noetic',
            version: 1,
            workspacePath: '/tmp/ws',
        };

        const execSpy = vi.spyOn(ros as unknown as { exec: (cmd: string) => Promise<string> }, 'exec')
            .mockImplementation(async (command: string) => {
                expect(command.includes('ros2 action list -t')).toBe(false);
                const markerPrefix = getMarkerPrefix(command);
                return [
                    buildSection(markerPrefix, 'nodes', 0, '/rosout'),
                    buildSection(markerPrefix, 'topics', 0, '/rosout'),
                    buildSection(markerPrefix, 'services', 0, '/rosout/get_loggers'),
                    buildSection(markerPrefix, 'parameters', 0, '/use_sim_time'),
                ].join('\n');
            });

        const snapshot = await ros.getGraphSnapshot({
            nodes: true,
            topics: true,
            services: true,
            actions: true,
            parameters: true,
        });

        expect(execSpy).toHaveBeenCalledTimes(1);
        expect(snapshot.actions).toEqual({ ok: true, data: [] });
        expect(snapshot.parameters).toEqual({
            ok: true,
            data: [{ name: '/use_sim_time' }],
        });
    });

    it('falls back to one-shot execution when persistent runner fails', async () => {
        const ros = new RosWorkspace();
        const warnSpy = vi.spyOn(console, 'warn').mockImplementation(() => {});
        const fakeRunner = {
            exec: vi.fn().mockRejectedValue(new Error('runner unavailable')),
        };

        vi.spyOn(
            ros as unknown as {
                shouldUsePersistentWslForGraph: (context: unknown) => boolean;
            },
            'shouldUsePersistentWslForGraph',
        ).mockReturnValue(true);
        vi.spyOn(
            ros as unknown as {
                getPersistentWslGraphRunner: (context: unknown) => { exec: (cmd: string, timeoutMs: number) => Promise<string> };
            },
            'getPersistentWslGraphRunner',
        ).mockReturnValue(fakeRunner);

        const execSpy = vi.spyOn(ros as unknown as { exec: (cmd: string) => Promise<string> }, 'exec')
            .mockImplementation(async (command: string) => {
                const markerPrefix = getMarkerPrefix(command);
                return buildSection(markerPrefix, 'nodes', 0, '/talker');
            });

        const snapshot = await ros.getGraphSnapshot({
            nodes: true,
            topics: false,
            services: false,
            actions: false,
            parameters: false,
        });

        expect(fakeRunner.exec).toHaveBeenCalled();
        expect(execSpy).toHaveBeenCalledTimes(1);
        expect(warnSpy).toHaveBeenCalled();
        expect(snapshot.nodes).toEqual({
            ok: true,
            data: ['/talker'],
        });
    });

    it('filters warning lines when node list is queried directly', async () => {
        const ros = new RosWorkspace();
        (ros as unknown as { _env: unknown })._env = {
            distro: 'jazzy',
            version: 2,
            workspacePath: '/tmp/ws',
        };

        vi.spyOn(ros as unknown as { exec: (cmd: string) => Promise<string> }, 'exec')
            .mockResolvedValue([
                'WARNING: Be aware that there are nodes in the graph that share an exact name.',
                '/talker',
                '/listener',
            ].join('\n'));

        const nodes = await ros.getNodeList();
        expect(nodes).toEqual(['/talker', '/listener']);
    });
});
