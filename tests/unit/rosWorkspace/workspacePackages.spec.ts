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
            'src/pkg_a/package.xml': '<package><name>pkg_a</name></package>',
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

        const pkgB = details.find((pkg) => pkg.name === 'pkg_b');
        expect(pkgB?.launchFiles).toEqual([]);
    });
});
