import path from 'node:path';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { __resetMockState, __setWorkspaceFolder } from '../../helpers/mocks/vscode';
import { RosWorkspace } from '../../../src/ros/rosWorkspace';
import { createTempWorkspace, removeTempWorkspace } from '../../helpers/workspaceFactory/tempWorkspace';

vi.mock('vscode', () => import('../../helpers/mocks/vscode'));

describe('RosWorkspace package discovery', () => {
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

    it('loads all workspace packages and launch files while skipping ignored folders', async () => {
        workspaceRoot = createTempWorkspace({
            'src/pkg_b/package.xml': '<package><name>pkg_b</name></package>',
            'src/pkg_b/CMakeLists.txt': `
cmake_minimum_required(VERSION 3.8)
project(pkg_b)
add_executable(cpp_node src/cpp_node.cpp)
`,
            'src/pkg_b/src/cpp_node.cpp': 'int main() { return 0; }',
            'src/pkg_a/package.xml': '<package><name>pkg_a</name></package>',
            'src/pkg_a/setup.py': `
from setuptools import setup

setup(
    name='pkg_a',
    entry_points={
        'console_scripts': [
            'talker = pkg_a.talker:main',
        ],
    },
)
`,
            'src/pkg_a/pkg_a/talker.py': 'def main(): pass',
            'src/pkg_a/launch/main.launch.py': '# launch file',
            'src/pkg_a/launch/nested/extra.xml': '<launch/>',
            'src/pkg_a/launch/notes.txt': 'ignored by extension filter',
            'src/build/ignored_pkg/package.xml': '<package><name>ignored_pkg</name></package>',
            'src/install/ignored_pkg/package.xml': '<package><name>ignored_install</name></package>',
            'src/pkg_bad/package.xml': '<package><name></name></package>',
        });

        __setWorkspaceFolder(workspaceRoot);
        const ros = new RosWorkspace();

        const names = await ros.listWorkspacePackages();
        const details = await ros.listWorkspacePackageDetails();

        expect(names).toEqual(['pkg_a', 'pkg_b']);
        expect(details.map((pkg) => pkg.name)).toEqual(['pkg_a', 'pkg_b']);

        const pkgA = details.find((pkg) => pkg.name === 'pkg_a');
        expect(pkgA?.launchFiles).toEqual([
            path.join(workspaceRoot, 'src/pkg_a/launch/main.launch.py'),
            path.join(workspaceRoot, 'src/pkg_a/launch/nested/extra.xml'),
        ]);
        expect(pkgA?.nodes).toEqual([
            {
                name: 'talker',
                sourcePath: path.join(workspaceRoot, 'src/pkg_a/pkg_a/talker.py'),
            },
        ]);

        const pkgB = details.find((pkg) => pkg.name === 'pkg_b');
        expect(pkgB?.launchFiles).toEqual([]);
        expect(pkgB?.nodes).toEqual([
            {
                name: 'cpp_node',
                sourcePath: path.join(workspaceRoot, 'src/pkg_b/src/cpp_node.cpp'),
            },
        ]);
    });

    it('loads non-workspace package details with launch files and ROS CLI executables', async () => {
        workspaceRoot = createTempWorkspace({
            'opt/share/other_pkg/package.xml': '<package><name>other_pkg</name></package>',
            'opt/share/other_pkg/launch/demo.launch.py': '# launch',
            'opt/share/other_pkg/launch/extra.xml': '<launch/>',
        });

        const otherPkgSharePath = path.join(workspaceRoot, 'opt/share/other_pkg');
        const ros = new RosWorkspace();
        vi.spyOn(ros as any, 'exec').mockImplementation(async (cmd: string) => {
            if (cmd === 'ros2 pkg prefix --share other_pkg') {
                return `${otherPkgSharePath}\n`;
            }
            if (cmd === 'ros2 pkg executables other_pkg') {
                return 'other_pkg talker\nother_pkg listener\n';
            }
            return '';
        });

        const details = await ros.listPackageDetails(['other_pkg']);

        expect(details).toEqual([
            {
                name: 'other_pkg',
                packagePath: otherPkgSharePath,
                launchFiles: [
                    path.join(otherPkgSharePath, 'launch/demo.launch.py'),
                    path.join(otherPkgSharePath, 'launch/extra.xml'),
                ],
                nodes: [
                    { name: 'listener' },
                    { name: 'talker' },
                ],
                isPython: false,
            },
        ]);
    });
});
