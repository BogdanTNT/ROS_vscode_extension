import { describe, expect, it } from 'vitest';
import {
    parsePythonLaunchArgs,
    parseXmlLaunchArgs,
    deduplicateArgs,
} from '../../../src/ros/launch/launchArgsParsing';

describe('launchArgsParsing/parsePythonLaunchArgs', () => {
    it('extracts names and default values', () => {
        const content = [
            "DeclareLaunchArgument('use_sim_time', default_value='false')",
            "DeclareLaunchArgument('robot_name', default_value=\"turtlebot\")",
            "DeclareLaunchArgument('no_default')",
        ].join('\n');
        expect(parsePythonLaunchArgs(content)).toEqual([
            { name: 'use_sim_time', defaultValue: 'false' },
            { name: 'robot_name', defaultValue: 'turtlebot' },
            { name: 'no_default', defaultValue: undefined },
        ]);
    });

    it('dedups repeated argument declarations', () => {
        const content = [
            "DeclareLaunchArgument('x', default_value='1')",
            "DeclareLaunchArgument('x', default_value='2')",
        ].join('\n');
        expect(parsePythonLaunchArgs(content)).toEqual([
            { name: 'x', defaultValue: '1' },
        ]);
    });
});

describe('launchArgsParsing/parseXmlLaunchArgs', () => {
    it('extracts names and defaults from <arg> tags', () => {
        const content = [
            '<arg name="use_sim_time" default="false"/>',
            '<arg name="port"/>',
        ].join('\n');
        expect(parseXmlLaunchArgs(content)).toEqual([
            { name: 'use_sim_time', defaultValue: 'false' },
            { name: 'port', defaultValue: undefined },
        ]);
    });
});

describe('launchArgsParsing/deduplicateArgs', () => {
    it('keeps the first occurrence of each name', () => {
        expect(
            deduplicateArgs([
                { name: 'a', defaultValue: '1' },
                { name: 'a', defaultValue: '2' },
                { name: 'b', defaultValue: undefined },
            ]),
        ).toEqual([
            { name: 'a', defaultValue: '1' },
            { name: 'b', defaultValue: undefined },
        ]);
    });
});
