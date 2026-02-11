import path from 'node:path';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import {
    __resetMockState,
    __setConfiguration,
    __setWorkspaceFolder,
    __getCreatedTerminals,
    __fireShellExecutionEnd,
} from '../../helpers/mocks/vscode';
import { RosWorkspace } from '../../../src/ros/rosWorkspace';
import { createTempWorkspace, removeTempWorkspace } from '../../helpers/workspaceFactory/tempWorkspace';

vi.mock('vscode', () => import('../../helpers/mocks/vscode'));

type ExecutionCase = {
    name: string;
    invoke: (ros: RosWorkspace) => Promise<void>;
    expectedCommand: string;
    expectedLabel: string;
};

describe('RosWorkspace create and launch commands', () => {
    let workspaceRoot = '';

    const directExecutionCases: ExecutionCase[] = [
        {
            name: 'launch files',
            invoke: (ros) => ros.launchFile('demo_pkg', 'robot.launch.py'),
            expectedCommand: 'ros2 launch demo_pkg robot.launch.py',
            expectedLabel: 'demo_pkg / robot.launch.py',
        },
        {
            name: 'nodes',
            invoke: (ros) => ros.runNode('demo_pkg', 'talker'),
            expectedCommand: 'ros2 run demo_pkg talker',
            expectedLabel: 'demo_pkg / talker',
        },
    ];

    const staleExecutionCases: ExecutionCase[] = [
        {
            name: 'launch files',
            invoke: (ros) => ros.launchFile('demo_pkg', 'robot.launch.py', 'use_sim_time:=true'),
            expectedCommand: 'ros2 launch demo_pkg robot.launch.py use_sim_time:=true',
            expectedLabel: 'demo_pkg / robot.launch.py',
        },
        {
            name: 'nodes',
            invoke: (ros) => ros.runNode('demo_pkg', 'talker', 'some_arg:=true'),
            expectedCommand: 'ros2 run demo_pkg talker some_arg:=true',
            expectedLabel: 'demo_pkg / talker',
        },
    ];

    const staleBuildEvaluation = {
        packagesNeedingBuild: ['demo_pkg'],
        upToDate: [],
        details: new Map([['demo_pkg', { reason: 'source-changed' as const }]]),
    };

    const upToDateBuildEvaluation = {
        packagesNeedingBuild: [],
        upToDate: ['demo_pkg'],
        details: new Map([['demo_pkg', { reason: 'Up to date' as const }]]),
    };

    const setMaintainerDefaults = () => {
        __setConfiguration('rosDevToolkit', 'defaultMaintainerName', 'Test Maintainer');
        __setConfiguration('rosDevToolkit', 'defaultMaintainerEmail', 'test@example.com');
    };

    beforeEach(() => {
        __resetMockState();
    });

    afterEach(() => {
        if (workspaceRoot) {
            removeTempWorkspace(workspaceRoot);
            workspaceRoot = '';
        }
        __resetMockState();
    });

    it.each([
        {
            name: 'adds default Python dependencies and merges custom dependencies',
            packageName: 'demo_pkg',
            buildType: 'ament_python',
            deps: ['geometry_msgs', 'std_msgs'],
            expectedDeps: ['rclpy', 'std_msgs', 'geometry_msgs'],
        },
        {
            name: 'adds default C++ dependencies when none are provided',
            packageName: 'demo_cpp_pkg',
            buildType: 'ament_cmake',
            deps: [] as string[],
            expectedDeps: ['rclcpp', 'std_msgs'],
        },
    ])('$name', async ({ packageName, buildType, deps, expectedDeps }) => {
        workspaceRoot = createTempWorkspace({
            'src/.keep': '',
        });
        __setWorkspaceFolder(workspaceRoot);
        setMaintainerDefaults();

        const ros = new RosWorkspace();
        const runSpy = vi.spyOn(ros, 'runInTerminal').mockImplementation(() => {});

        const result = await ros.createPackage(packageName, buildType, deps);

        expect(result).toBe(true);
        expect(runSpy).toHaveBeenCalledTimes(1);

        const cmd = runSpy.mock.calls[0][0] as string;
        expect(cmd).toContain(
            `cd "${path.join(workspaceRoot, 'src')}" && ros2 pkg create ${packageName} --build-type ${buildType}`,
        );
        expect(cmd).toContain(` --dependencies ${expectedDeps.join(' ')}`);
        expect(cmd).toContain("--maintainer-name 'Test Maintainer'");
        expect(cmd).toContain("--maintainer-email 'test@example.com'");
        expect(cmd).toContain("--license 'GPL-3.0'");
    });

    it.each([
        {
            name: 'launches in external terminal when launchInExternalTerminal is enabled',
            useExternal: true,
        },
        {
            name: 'launches in integrated terminal when launchInExternalTerminal is disabled',
            useExternal: false,
        },
    ])('$name', async ({ useExternal }) => {
        __setConfiguration('rosDevToolkit', 'launchInExternalTerminal', useExternal);
        const ros = new RosWorkspace();
        const externalSpy = vi.spyOn(ros, 'runInExternalTerminal').mockImplementation(() => {});
        const integratedSpy = vi.spyOn(ros, 'runInLaunchTerminal').mockImplementation(() => {});

        await ros.launchFile('demo_pkg', 'robot.launch.py', 'use_sim_time:=true', '/tmp/robot.launch.py', 'sim');

        if (useExternal) {
            expect(externalSpy).toHaveBeenCalledWith(
                'ros2 launch demo_pkg robot.launch.py use_sim_time:=true',
                'demo_pkg / robot.launch.py [sim]',
                '/tmp/robot.launch.py',
            );
            expect(integratedSpy).not.toHaveBeenCalled();
        } else {
            expect(integratedSpy).toHaveBeenCalledWith(
                'ros2 launch demo_pkg robot.launch.py use_sim_time:=true',
                'demo_pkg / robot.launch.py [sim]',
                '/tmp/robot.launch.py',
            );
            expect(externalSpy).not.toHaveBeenCalled();
        }
    });

    // ── --symlink-install ──────────────────────────────────────
    it('includes --symlink-install and cleans stale build+install dirs', () => {
        workspaceRoot = createTempWorkspace({ 'src/.keep': '' });
        __setWorkspaceFolder(workspaceRoot);

        const ros = new RosWorkspace();
        const rawSpy = vi.spyOn(ros, 'sendRawCommand').mockImplementation(() => {});

        ros.buildPackages(['my_pkg']);

        expect(rawSpy).toHaveBeenCalledTimes(1);
        const cmd = rawSpy.mock.calls[0][0] as string;
        expect(cmd).toContain('--symlink-install');
        expect(cmd).toContain('--packages-select my_pkg');
        // Stale build AND install artifacts are removed before the build
        expect(cmd).toContain(`"${workspaceRoot}/build/my_pkg"`);
        expect(cmd).toContain(`"${workspaceRoot}/install/my_pkg"`);
        // The rm must come BEFORE colcon build
        expect(cmd.indexOf('rm -rf')).toBeLessThan(cmd.indexOf('colcon build'));
        // The overlay is NOT sourced before the build (only ROS base + post-build)
        expect(cmd).not.toMatch(/source.*install\/setup\.bash.*colcon build/);
        // The overlay IS sourced AFTER the build
        expect(cmd.indexOf('colcon build')).toBeLessThan(cmd.indexOf('install/setup.bash'));
    });

    it('cleans both build and install dirs for multiple packages', () => {
        workspaceRoot = createTempWorkspace({ 'src/.keep': '' });
        __setWorkspaceFolder(workspaceRoot);

        const ros = new RosWorkspace();
        const rawSpy = vi.spyOn(ros, 'sendRawCommand').mockImplementation(() => {});

        ros.buildPackages(['pkg_a', 'pkg_b']);

        const cmd = rawSpy.mock.calls[0][0] as string;
        expect(cmd).toContain(`"${workspaceRoot}/build/pkg_a"`);
        expect(cmd).toContain(`"${workspaceRoot}/install/pkg_a"`);
        expect(cmd).toContain(`"${workspaceRoot}/build/pkg_b"`);
        expect(cmd).toContain(`"${workspaceRoot}/install/pkg_b"`);
    });

    it('still cleans build+install dirs when symlinkInstall is disabled', () => {
        workspaceRoot = createTempWorkspace({ 'src/.keep': '' });
        __setWorkspaceFolder(workspaceRoot);
        __setConfiguration('rosDevToolkit', 'symlinkInstall', false);

        const ros = new RosWorkspace();
        const rawSpy = vi.spyOn(ros, 'sendRawCommand').mockImplementation(() => {});

        ros.buildPackages(['my_pkg']);

        expect(rawSpy).toHaveBeenCalledTimes(1);
        const cmd = rawSpy.mock.calls[0][0] as string;
        expect(cmd).not.toContain('--symlink-install');
        // Cleanup still happens to handle stale CMakeCache
        expect(cmd).toContain('rm -rf');
        expect(cmd).toContain(`"${workspaceRoot}/build/my_pkg"`);
    });

    // ── Pre-launch/run build checks ────────────────────────────
    it.each(directExecutionCases)(
        'skips pre-execution build check for $name when toggle is off',
        async ({ invoke, expectedCommand, expectedLabel }) => {
            __setConfiguration('rosDevToolkit', 'preLaunchBuildCheck', false);
            __setConfiguration('rosDevToolkit', 'launchInExternalTerminal', true);

            const ros = new RosWorkspace();
            const evaluateSpy = vi.spyOn(ros, 'evaluateBuildNeeds').mockReturnValue(staleBuildEvaluation);
            const buildThenRunSpy = vi.spyOn(ros, 'buildThenRun').mockImplementation(() => {});
            const externalSpy = vi.spyOn(ros, 'runInExternalTerminal').mockImplementation(() => {});
            const integratedSpy = vi.spyOn(ros, 'runInLaunchTerminal').mockImplementation(() => {});

            await invoke(ros);

            expect(evaluateSpy).not.toHaveBeenCalled();
            expect(buildThenRunSpy).not.toHaveBeenCalled();
            expect(externalSpy).toHaveBeenCalledWith(expectedCommand, expectedLabel, undefined);
            expect(integratedSpy).not.toHaveBeenCalled();
        },
    );

    it('buildThenRun constructs a combined build+launch command', () => {
        workspaceRoot = createTempWorkspace({ 'src/.keep': '' });
        __setWorkspaceFolder(workspaceRoot);
        __setConfiguration('rosDevToolkit', 'launchInExternalTerminal', true);

        const ros = new RosWorkspace();
        const externalSpy = vi.spyOn(ros, 'runInExternalTerminal').mockImplementation(() => {});

        ros.buildThenRun(
            ['my_pkg'],
            'ros2 launch my_pkg robot.launch.py',
            'my_pkg / robot.launch.py',
        );

        expect(externalSpy).toHaveBeenCalledTimes(1);
        const cmd = externalSpy.mock.calls[0][0] as string;
        // Build section present
        expect(cmd).toContain('colcon build');
        expect(cmd).toContain('--packages-select my_pkg');
        expect(cmd).toContain('--symlink-install');
        // Cleanup before build
        expect(cmd).toContain(`"${workspaceRoot}/build/my_pkg"`);
        expect(cmd).toContain(`"${workspaceRoot}/install/my_pkg"`);
        // Overlay sourced after build, before launch
        expect(cmd).toContain('install/setup.bash');
        // Launch command appended at the end
        expect(cmd).toContain('ros2 launch my_pkg robot.launch.py');
        // Build comes before launch (chained with &&)
        expect(cmd.indexOf('colcon build')).toBeLessThan(cmd.indexOf('ros2 launch'));
        // Raw mode: 4th arg is true
        expect(externalSpy.mock.calls[0][3]).toBe(true);
    });

    it('buildThenLaunch delegates to buildThenRun', () => {
        const ros = new RosWorkspace();
        const buildThenRunSpy = vi.spyOn(ros, 'buildThenRun').mockImplementation(() => {});

        ros.buildThenLaunch(['pkg_a'], 'ros2 launch pkg_a app.launch.py', 'pkg_a / app.launch.py');

        expect(buildThenRunSpy).toHaveBeenCalledWith(
            ['pkg_a'],
            'ros2 launch pkg_a app.launch.py',
            'pkg_a / app.launch.py',
            undefined,
        );
    });

    it('buildThenRunNode delegates to buildThenRun', () => {
        const ros = new RosWorkspace();
        const buildThenRunSpy = vi.spyOn(ros, 'buildThenRun').mockImplementation(() => {});

        ros.buildThenRunNode(['demo_pkg'], 'ros2 run demo_pkg talker', 'demo_pkg / talker');

        expect(buildThenRunSpy).toHaveBeenCalledWith(
            ['demo_pkg'],
            'ros2 run demo_pkg talker',
            'demo_pkg / talker',
            undefined,
        );
    });

    it.each(staleExecutionCases)(
        'auto builds then executes stale $name when build check is enabled',
        async ({ invoke, expectedCommand, expectedLabel }) => {
            __setConfiguration('rosDevToolkit', 'launchInExternalTerminal', true);

            const ros = new RosWorkspace();
            const evaluateSpy = vi.spyOn(ros, 'evaluateBuildNeeds').mockReturnValue(staleBuildEvaluation);
            const buildThenRunSpy = vi.spyOn(ros, 'buildThenRun').mockImplementation(() => {});
            const externalSpy = vi.spyOn(ros, 'runInExternalTerminal').mockImplementation(() => {});
            const integratedSpy = vi.spyOn(ros, 'runInLaunchTerminal').mockImplementation(() => {});

            await invoke(ros);

            expect(evaluateSpy).toHaveBeenCalled();
            expect(buildThenRunSpy).toHaveBeenCalledWith(
                ['demo_pkg'],
                expectedCommand,
                expectedLabel,
                undefined,
            );
            // Direct terminal execution must not be called.
            expect(externalSpy).not.toHaveBeenCalled();
            expect(integratedSpy).not.toHaveBeenCalled();
        },
    );

    it.each(directExecutionCases)(
        'executes $name directly when all packages are up to date (external terminal)',
        async ({ invoke, expectedCommand, expectedLabel }) => {
            __setConfiguration('rosDevToolkit', 'launchInExternalTerminal', true);

            const ros = new RosWorkspace();
            vi.spyOn(ros, 'evaluateBuildNeeds').mockReturnValue(upToDateBuildEvaluation);
            const buildThenRunSpy = vi.spyOn(ros, 'buildThenRun').mockImplementation(() => {});
            const externalSpy = vi.spyOn(ros, 'runInExternalTerminal').mockImplementation(() => {});
            const integratedSpy = vi.spyOn(ros, 'runInLaunchTerminal').mockImplementation(() => {});

            await invoke(ros);

            expect(buildThenRunSpy).not.toHaveBeenCalled();
            expect(externalSpy).toHaveBeenCalledWith(expectedCommand, expectedLabel, undefined);
            expect(integratedSpy).not.toHaveBeenCalled();
        },
    );

    it.each(directExecutionCases)(
        'executes $name directly in integrated terminal when external launch terminal is disabled',
        async ({ invoke, expectedCommand, expectedLabel }) => {
            __setConfiguration('rosDevToolkit', 'launchInExternalTerminal', false);

            const ros = new RosWorkspace();
            vi.spyOn(ros, 'evaluateBuildNeeds').mockReturnValue(upToDateBuildEvaluation);
            const buildThenRunSpy = vi.spyOn(ros, 'buildThenRun').mockImplementation(() => {});
            const externalSpy = vi.spyOn(ros, 'runInExternalTerminal').mockImplementation(() => {});
            const integratedSpy = vi.spyOn(ros, 'runInLaunchTerminal').mockImplementation(() => {});

            await invoke(ros);

            expect(buildThenRunSpy).not.toHaveBeenCalled();
            expect(integratedSpy).toHaveBeenCalledWith(expectedCommand, expectedLabel, undefined);
            expect(externalSpy).not.toHaveBeenCalled();
        },
    );

    it('onDidEndTerminalShellExecution marks terminal status as closed', () => {
        const ros = new RosWorkspace();
        ros.runInLaunchTerminal('ros2 run demo_pkg talker', 'demo_pkg / talker');

        const tracked = ros.getTrackedTerminals();
        expect(tracked).toHaveLength(1);
        expect(tracked[0].status).toBe('running');

        const terminal = __getCreatedTerminals()[0];
        __fireShellExecutionEnd(terminal as unknown as Parameters<typeof __fireShellExecutionEnd>[0]);

        const updated = ros.getTrackedTerminals();
        expect(updated[0].status).toBe('closed');
    });

    it('kill sends Ctrl+C when process is still running', async () => {
        const ros = new RosWorkspace();
        ros.runInLaunchTerminal('ros2 run demo_pkg talker', 'demo_pkg / talker');

        const tracked = ros.getTrackedTerminals();
        expect(tracked).toHaveLength(1);
        expect(tracked[0].status).toBe('running');

        const terminal = __getCreatedTerminals()[0] as unknown as {
            sentTexts: string[];
            disposed: boolean;
        };

        await ros.killTrackedTerminal(tracked[0].id);

        expect(terminal.disposed).toBe(false);
        expect(terminal.sentTexts[terminal.sentTexts.length - 1]).toBe('\x03');
        expect(ros.getTrackedTerminals()).toHaveLength(1);
    });

    it('kill disposes terminal after process has finished (status closed)', async () => {
        const ros = new RosWorkspace();
        ros.runInLaunchTerminal('ros2 run demo_pkg talker', 'demo_pkg / talker');

        const terminal = __getCreatedTerminals()[0];
        // Simulate the shell execution ending (ROS process finished)
        __fireShellExecutionEnd(terminal as unknown as Parameters<typeof __fireShellExecutionEnd>[0]);

        const tracked = ros.getTrackedTerminals();
        expect(tracked[0].status).toBe('closed');

        const mockTerminal = terminal as unknown as { disposed: boolean; sentTexts: string[] };
        await ros.killTrackedTerminal(tracked[0].id);

        expect(mockTerminal.disposed).toBe(true);
        expect(mockTerminal.sentTexts).not.toContain('\x03');
        expect(ros.getTrackedTerminals()).toHaveLength(0);
    });

    it('second kill force-closes terminal after Ctrl+C was already sent', async () => {
        const ros = new RosWorkspace();
        ros.runInLaunchTerminal('ros2 run demo_pkg talker', 'demo_pkg / talker');

        const tracked = ros.getTrackedTerminals();
        const terminal = __getCreatedTerminals()[0] as unknown as {
            sentTexts: string[];
            disposed: boolean;
        };

        // First kill while running: sends Ctrl+C
        await ros.killTrackedTerminal(tracked[0].id);
        expect(terminal.disposed).toBe(false);
        expect(terminal.sentTexts[terminal.sentTexts.length - 1]).toBe('\x03');

        // Second kill: ctrlCSent flag causes force-dispose
        await ros.killTrackedTerminal(tracked[0].id);
        expect(terminal.disposed).toBe(true);
        expect(ros.getTrackedTerminals()).toHaveLength(0);
    });
});
