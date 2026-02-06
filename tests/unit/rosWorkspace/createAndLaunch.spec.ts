import path from 'node:path';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import {
    __resetMockState,
    __setConfiguration,
    __setWorkspaceFolder,
    __getMessages,
    __setWarningMessageResponse,
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

    it('builds the ROS 2 create package command and sends it to the terminal', async () => {
        workspaceRoot = createTempWorkspace({
            'src/.keep': '',
        });
        __setWorkspaceFolder(workspaceRoot);

        const ros = new RosWorkspace();
        const runSpy = vi.spyOn(ros, 'runInTerminal').mockImplementation(() => {});

        const result = await ros.createPackage('demo_pkg', 'ament_python', ['rclcpp', 'std_msgs']);

        expect(result).toBe(true);
        expect(runSpy).toHaveBeenCalledTimes(1);
        expect(runSpy).toHaveBeenCalledWith(
            `cd "${path.join(workspaceRoot, 'src')}" && ros2 pkg create demo_pkg --build-type ament_python --dependencies rclcpp std_msgs`
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
        const buildThenLaunchSpy = vi.spyOn(ros, 'buildThenLaunch').mockImplementation(() => {});
        const externalSpy = vi.spyOn(ros, 'runInExternalTerminal').mockImplementation(() => {});

        await ros.launchFile('demo_pkg', 'robot.launch.py', 'use_sim_time:=true');

        expect(buildThenLaunchSpy).toHaveBeenCalledWith(
            ['demo_pkg'],
            'ros2 launch demo_pkg robot.launch.py use_sim_time:=true',
            'demo_pkg / robot.launch.py',
            undefined,
        );
        // Direct launch must NOT be called — buildThenLaunch handles both
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
});
