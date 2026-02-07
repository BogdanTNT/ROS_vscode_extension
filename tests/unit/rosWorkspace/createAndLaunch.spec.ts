import path from 'node:path';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import {
    __resetMockState,
    __setConfiguration,
    __setWorkspaceFolder,
    __getMessages,
    __setWarningMessageResponse,
    __getCreatedTerminals,
    __fireShellExecutionEnd,
} from '../../helpers/mocks/vscode';
import { RosWorkspace } from '../../../src/ros/rosWorkspace';
import { createTempWorkspace, removeTempWorkspace } from '../../helpers/workspaceFactory/tempWorkspace';

vi.mock('vscode', () => import('../../helpers/mocks/vscode'));

describe('RosWorkspace create and launch commands', () => {
    let workspaceRoot = '';

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

    it('adds default Python dependencies and merges custom dependencies', async () => {
        workspaceRoot = createTempWorkspace({
            'src/.keep': '',
        });
        __setWorkspaceFolder(workspaceRoot);

        const ros = new RosWorkspace();
        const runSpy = vi.spyOn(ros, 'runInTerminal').mockImplementation(() => {});

        const result = await ros.createPackage('demo_pkg', 'ament_python', ['geometry_msgs', 'std_msgs']);

        expect(result).toBe(true);
        expect(runSpy).toHaveBeenCalledTimes(1);
        expect(runSpy).toHaveBeenCalledWith(
            `cd "${path.join(workspaceRoot, 'src')}" && ros2 pkg create demo_pkg --build-type ament_python --dependencies rclpy std_msgs geometry_msgs`
        );
    });

    it('adds default C++ dependencies when none are provided', async () => {
        workspaceRoot = createTempWorkspace({
            'src/.keep': '',
        });
        __setWorkspaceFolder(workspaceRoot);

        const ros = new RosWorkspace();
        const runSpy = vi.spyOn(ros, 'runInTerminal').mockImplementation(() => {});

        const result = await ros.createPackage('demo_cpp_pkg', 'ament_cmake', []);

        expect(result).toBe(true);
        expect(runSpy).toHaveBeenCalledTimes(1);
        expect(runSpy).toHaveBeenCalledWith(
            `cd "${path.join(workspaceRoot, 'src')}" && ros2 pkg create demo_cpp_pkg --build-type ament_cmake --dependencies rclcpp std_msgs`
        );
    });

    it('launches in external terminal when launchInExternalTerminal is enabled', async () => {
        __setConfiguration('rosDevToolkit', 'launchInExternalTerminal', true);
        const ros = new RosWorkspace();
        const externalSpy = vi.spyOn(ros, 'runInExternalTerminal').mockImplementation(() => {});
        const integratedSpy = vi.spyOn(ros, 'runInLaunchTerminal').mockImplementation(() => {});

        await ros.launchFile('demo_pkg', 'robot.launch.py', 'use_sim_time:=true', '/tmp/robot.launch.py', 'sim');

        expect(externalSpy).toHaveBeenCalledWith(
            'ros2 launch demo_pkg robot.launch.py use_sim_time:=true',
            'demo_pkg / robot.launch.py [sim]',
            '/tmp/robot.launch.py',
        );
        expect(integratedSpy).not.toHaveBeenCalled();
    });

    it('launches in integrated terminal when launchInExternalTerminal is disabled', async () => {
        __setConfiguration('rosDevToolkit', 'launchInExternalTerminal', false);
        const ros = new RosWorkspace();
        const externalSpy = vi.spyOn(ros, 'runInExternalTerminal').mockImplementation(() => {});
        const integratedSpy = vi.spyOn(ros, 'runInLaunchTerminal').mockImplementation(() => {});

        await ros.launchFile('demo_pkg', 'robot.launch.py', 'use_sim_time:=true', '/tmp/robot.launch.py', 'sim');

        expect(integratedSpy).toHaveBeenCalledWith(
            'ros2 launch demo_pkg robot.launch.py use_sim_time:=true',
            'demo_pkg / robot.launch.py [sim]',
            '/tmp/robot.launch.py',
        );
        expect(externalSpy).not.toHaveBeenCalled();
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

    // ── Pre-launch build check ─────────────────────────────────
    it('skips pre-launch build check when toggle is off', async () => {
        __setConfiguration('rosDevToolkit', 'preLaunchBuildCheck', false);
        __setConfiguration('rosDevToolkit', 'launchInExternalTerminal', true);

        const ros = new RosWorkspace();
        vi.spyOn(ros, 'evaluateBuildNeeds').mockReturnValue({
            packagesNeedingBuild: ['demo_pkg'],
            upToDate: [],
            details: new Map([['demo_pkg', { reason: 'source-changed' as const }]]),
        });
        const buildThenLaunchSpy = vi.spyOn(ros, 'buildThenLaunch').mockImplementation(() => {});
        const externalSpy = vi.spyOn(ros, 'runInExternalTerminal').mockImplementation(() => {});

        await ros.launchFile('demo_pkg', 'robot.launch.py');

        // Toggle off → no build, launch directly
        expect(buildThenLaunchSpy).not.toHaveBeenCalled();
        expect(externalSpy).toHaveBeenCalledTimes(1);
    });

    it('buildThenLaunch constructs a combined build+launch command', () => {
        workspaceRoot = createTempWorkspace({ 'src/.keep': '' });
        __setWorkspaceFolder(workspaceRoot);
        __setConfiguration('rosDevToolkit', 'launchInExternalTerminal', true);

        const ros = new RosWorkspace();
        const externalSpy = vi.spyOn(ros, 'runInExternalTerminal').mockImplementation(() => {});

        ros.buildThenLaunch(
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

    it('auto builds then launches when toggle is on and packages are stale', async () => {
        __setConfiguration('rosDevToolkit', 'launchInExternalTerminal', true);

        const ros = new RosWorkspace();
        vi.spyOn(ros, 'evaluateBuildNeeds').mockReturnValue({
            packagesNeedingBuild: ['demo_pkg'],
            upToDate: [],
            details: new Map([['demo_pkg', { reason: 'source-changed' as const }]]),
        });
        const buildThenRunSpy = vi.spyOn(ros, 'buildThenRun').mockImplementation(() => {});
        const externalSpy = vi.spyOn(ros, 'runInExternalTerminal').mockImplementation(() => {});

        await ros.launchFile('demo_pkg', 'robot.launch.py', 'use_sim_time:=true');

        expect(buildThenRunSpy).toHaveBeenCalledWith(
            ['demo_pkg'],
            'ros2 launch demo_pkg robot.launch.py use_sim_time:=true',
            'demo_pkg / robot.launch.py',
            undefined,
        );
        // Direct launch must NOT be called — buildThenRun handles both
        expect(externalSpy).not.toHaveBeenCalled();
    });

    it('launches directly when all packages are up to date', async () => {
        __setConfiguration('rosDevToolkit', 'launchInExternalTerminal', true);

        const ros = new RosWorkspace();
        vi.spyOn(ros, 'evaluateBuildNeeds').mockReturnValue({
            packagesNeedingBuild: [],
            upToDate: ['demo_pkg'],
            details: new Map([['demo_pkg', { reason: 'Up to date' as const }]]),
        });
        const buildThenLaunchSpy = vi.spyOn(ros, 'buildThenLaunch').mockImplementation(() => {});
        const externalSpy = vi.spyOn(ros, 'runInExternalTerminal').mockImplementation(() => {});

        await ros.launchFile('demo_pkg', 'robot.launch.py');

        expect(buildThenLaunchSpy).not.toHaveBeenCalled();
        expect(externalSpy).toHaveBeenCalledTimes(1);
    });

    // ── Pre-run build check (nodes) ────────────────────────────
    it('skips pre-run build check for nodes when toggle is off', async () => {
        __setConfiguration('rosDevToolkit', 'preLaunchBuildCheck', false);
        __setConfiguration('rosDevToolkit', 'launchInExternalTerminal', true);

        const ros = new RosWorkspace();
        vi.spyOn(ros, 'evaluateBuildNeeds').mockReturnValue({
            packagesNeedingBuild: ['demo_pkg'],
            upToDate: [],
            details: new Map([['demo_pkg', { reason: 'source-changed' as const }]]),
        });
        const buildThenRunNodeSpy = vi.spyOn(ros, 'buildThenRunNode').mockImplementation(() => {});
        const externalSpy = vi.spyOn(ros, 'runInExternalTerminal').mockImplementation(() => {});
        const integratedSpy = vi.spyOn(ros, 'runInLaunchTerminal').mockImplementation(() => {});

        await ros.runNode('demo_pkg', 'talker');

        // Toggle off → no build, run directly
        expect(buildThenRunNodeSpy).not.toHaveBeenCalled();
        expect(externalSpy).toHaveBeenCalledTimes(1);
        expect(externalSpy).toHaveBeenCalledWith(
            'ros2 run demo_pkg talker',
            'demo_pkg / talker',
            undefined,
        );
        expect(integratedSpy).not.toHaveBeenCalled();
    });

    it('auto builds then runs node when toggle is on and packages are stale', async () => {
        const ros = new RosWorkspace();
        vi.spyOn(ros, 'evaluateBuildNeeds').mockReturnValue({
            packagesNeedingBuild: ['demo_pkg'],
            upToDate: [],
            details: new Map([['demo_pkg', { reason: 'source-changed' as const }]]),
        });
        const buildThenRunSpy = vi.spyOn(ros, 'buildThenRun').mockImplementation(() => {});
        const externalSpy = vi.spyOn(ros, 'runInExternalTerminal').mockImplementation(() => {});
        const integratedSpy = vi.spyOn(ros, 'runInLaunchTerminal').mockImplementation(() => {});

        await ros.runNode('demo_pkg', 'talker', 'some_arg:=true');

        expect(buildThenRunSpy).toHaveBeenCalledWith(
            ['demo_pkg'],
            'ros2 run demo_pkg talker some_arg:=true',
            'demo_pkg / talker',
            undefined,
        );
        // Direct run must NOT be called — buildThenRun handles both
        expect(externalSpy).not.toHaveBeenCalled();
        expect(integratedSpy).not.toHaveBeenCalled();
    });

    it('runs node directly when all packages are up to date', async () => {
        __setConfiguration('rosDevToolkit', 'launchInExternalTerminal', true);

        const ros = new RosWorkspace();
        vi.spyOn(ros, 'evaluateBuildNeeds').mockReturnValue({
            packagesNeedingBuild: [],
            upToDate: ['demo_pkg'],
            details: new Map([['demo_pkg', { reason: 'Up to date' as const }]]),
        });
        const buildThenRunNodeSpy = vi.spyOn(ros, 'buildThenRunNode').mockImplementation(() => {});
        const externalSpy = vi.spyOn(ros, 'runInExternalTerminal').mockImplementation(() => {});
        const integratedSpy = vi.spyOn(ros, 'runInLaunchTerminal').mockImplementation(() => {});

        await ros.runNode('demo_pkg', 'talker');

        expect(buildThenRunNodeSpy).not.toHaveBeenCalled();
        expect(externalSpy).toHaveBeenCalledTimes(1);
        expect(externalSpy).toHaveBeenCalledWith(
            'ros2 run demo_pkg talker',
            'demo_pkg / talker',
            undefined,
        );
        expect(integratedSpy).not.toHaveBeenCalled();
    });

    it('runs node directly in integrated terminal when external launch terminal is disabled', async () => {
        __setConfiguration('rosDevToolkit', 'launchInExternalTerminal', false);

        const ros = new RosWorkspace();
        vi.spyOn(ros, 'evaluateBuildNeeds').mockReturnValue({
            packagesNeedingBuild: [],
            upToDate: ['demo_pkg'],
            details: new Map([['demo_pkg', { reason: 'Up to date' as const }]]),
        });
        const externalSpy = vi.spyOn(ros, 'runInExternalTerminal').mockImplementation(() => {});
        const integratedSpy = vi.spyOn(ros, 'runInLaunchTerminal').mockImplementation(() => {});

        await ros.runNode('demo_pkg', 'talker');

        expect(integratedSpy).toHaveBeenCalledTimes(1);
        expect(integratedSpy).toHaveBeenCalledWith(
            'ros2 run demo_pkg talker',
            'demo_pkg / talker',
            undefined,
        );
        expect(externalSpy).not.toHaveBeenCalled();
    });

    it('buildThenRunNode constructs a combined build+run command', () => {
        workspaceRoot = createTempWorkspace({ 'src/.keep': '' });
        __setWorkspaceFolder(workspaceRoot);
        __setConfiguration('rosDevToolkit', 'launchInExternalTerminal', true);

        const ros = new RosWorkspace();
        const externalSpy = vi.spyOn(ros, 'runInExternalTerminal').mockImplementation(() => {});
        const integratedSpy = vi.spyOn(ros, 'runInLaunchTerminal').mockImplementation(() => {});

        ros.buildThenRunNode(
            ['demo_pkg'],
            'ros2 run demo_pkg talker',
        );

        expect(externalSpy).toHaveBeenCalledTimes(1);
        const cmd = externalSpy.mock.calls[0][0] as string;
        // Build section present
        expect(cmd).toContain('colcon build');
        expect(cmd).toContain('--packages-select demo_pkg');
        expect(cmd).toContain('--symlink-install');
        // Cleanup before build
        expect(cmd).toContain(`"${workspaceRoot}/build/demo_pkg"`);
        expect(cmd).toContain(`"${workspaceRoot}/install/demo_pkg"`);
        // Overlay sourced after build, before run
        expect(cmd).toContain('install/setup.bash');
        // Run command appended at the end
        expect(cmd).toContain('ros2 run demo_pkg talker');
        // Build comes before run (chained with &&)
        expect(cmd.indexOf('colcon build')).toBeLessThan(cmd.indexOf('ros2 run'));
        expect(externalSpy).toHaveBeenCalledWith(cmd, undefined, undefined, true);
        expect(integratedSpy).not.toHaveBeenCalled();
    });

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
