import path from 'node:path';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { __resetMockState, __setConfiguration, __setWorkspaceFolder } from '../../helpers/mocks/vscode';
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

    it('launches in external terminal when launchInExternalTerminal is enabled', () => {
        __setConfiguration('rosDevToolkit', 'launchInExternalTerminal', true);
        const ros = new RosWorkspace();
        const externalSpy = vi.spyOn(ros, 'runInExternalTerminal').mockImplementation(() => {});
        const integratedSpy = vi.spyOn(ros, 'runInLaunchTerminal').mockImplementation(() => {});

        ros.launchFile('demo_pkg', 'robot.launch.py', 'use_sim_time:=true', '/tmp/robot.launch.py', 'sim');

        expect(externalSpy).toHaveBeenCalledWith(
            'ros2 launch demo_pkg robot.launch.py use_sim_time:=true',
            'demo_pkg / robot.launch.py [sim]',
            '/tmp/robot.launch.py'
        );
        expect(integratedSpy).not.toHaveBeenCalled();
    });

    it('launches in integrated terminal when launchInExternalTerminal is disabled', () => {
        __setConfiguration('rosDevToolkit', 'launchInExternalTerminal', false);
        const ros = new RosWorkspace();
        const externalSpy = vi.spyOn(ros, 'runInExternalTerminal').mockImplementation(() => {});
        const integratedSpy = vi.spyOn(ros, 'runInLaunchTerminal').mockImplementation(() => {});

        ros.launchFile('demo_pkg', 'robot.launch.py', 'use_sim_time:=true', '/tmp/robot.launch.py', 'sim');

        expect(integratedSpy).toHaveBeenCalledWith(
            'ros2 launch demo_pkg robot.launch.py use_sim_time:=true',
            'demo_pkg / robot.launch.py [sim]',
            '/tmp/robot.launch.py'
        );
        expect(externalSpy).not.toHaveBeenCalled();
    });
});
