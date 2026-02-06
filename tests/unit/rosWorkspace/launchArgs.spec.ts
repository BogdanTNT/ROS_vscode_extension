import path from 'node:path';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { __resetMockState } from '../../helpers/mocks/vscode';
import { RosWorkspace } from '../../../src/ros/rosWorkspace';
import { createTempWorkspace, removeTempWorkspace } from '../../helpers/workspaceFactory/tempWorkspace';

vi.mock('vscode', () => import('../../helpers/mocks/vscode'));

describe('RosWorkspace launch argument parsing', () => {
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

    it('parses and deduplicates ROS 2 Python launch arguments', () => {
        workspaceRoot = createTempWorkspace({
            'launch/test.launch.py': `
DeclareLaunchArgument('robot_name', default_value='turtlebot3')
DeclareLaunchArgument("use_sim_time", default_value="false")
DeclareLaunchArgument('robot_name', default_value='duplicate_ignored')
`,
        });

        const ros = new RosWorkspace();
        const filePath = path.join(workspaceRoot, 'launch/test.launch.py');
        const args = ros.getLaunchArgOptions(filePath);

        expect(args).toEqual([
            { name: 'robot_name', defaultValue: 'turtlebot3' },
            { name: 'use_sim_time', defaultValue: 'false' },
        ]);
    });

    it('parses and deduplicates XML launch arguments', () => {
        workspaceRoot = createTempWorkspace({
            'launch/test.launch.xml': `
<launch>
    <arg name="namespace" default="robot1"/>
    <arg name="debug"/>
    <arg name="namespace" default="duplicate_ignored"/>
</launch>
`,
        });

        const ros = new RosWorkspace();
        const filePath = path.join(workspaceRoot, 'launch/test.launch.xml');
        const args = ros.getLaunchArgOptions(filePath);

        expect(args).toEqual([
            { name: 'namespace', defaultValue: 'robot1' },
            { name: 'debug', defaultValue: undefined },
        ]);
    });
});
