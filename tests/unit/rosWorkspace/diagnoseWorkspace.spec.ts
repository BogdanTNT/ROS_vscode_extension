import path from 'node:path';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import {
    __getMessages,
    __resetMockState,
    __setWarningMessageResponse,
    __setWorkspaceFolder,
} from '../../helpers/mocks/vscode';
import { RosWorkspace } from '../../../src/ros/rosWorkspace';
import { createTempWorkspace, removeTempWorkspace } from '../../helpers/workspaceFactory/tempWorkspace';

vi.mock('vscode', () => import('../../helpers/mocks/vscode'));

describe('RosWorkspace.diagnoseWorkspace', () => {
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

    it('offers to point the workspace root at a nested colcon workspace and applies the fix', async () => {
        workspaceRoot = createTempWorkspace({
            'ars-motus/src/ars_motus/package.xml': '<package><name>ars_motus</name></package>',
        });
        __setWorkspaceFolder(workspaceRoot);
        __setWarningMessageResponse('Use "ars-motus" as ROS workspace root');

        const ros = new RosWorkspace();
        expect(ros.getSrcDir()).toBeUndefined();

        await ros.diagnoseWorkspace(false);

        expect(__getMessages().warn.some((m) => m.includes('No "src/" folder found'))).toBe(true);
        expect(ros.getSrcDir()).toBe(path.join(workspaceRoot, 'ars-motus', 'src'));
    });

    it('reports no ROS packages found when nothing exists and interactive is true', async () => {
        workspaceRoot = createTempWorkspace({
            'README.md': '# empty project',
        });
        __setWorkspaceFolder(workspaceRoot);

        const ros = new RosWorkspace();
        await ros.diagnoseWorkspace(true);

        expect(__getMessages().info.some((m) => m.includes('No ROS packages'))).toBe(true);
    });

    it('warns about duplicate package names found under src/', async () => {
        workspaceRoot = createTempWorkspace({
            'src/ars_motus/package.xml': '<package><name>ars_motus</name></package>',
            'src/vendor/ars_motus_old/package.xml': '<package><name>ars_motus</name></package>',
        });
        __setWorkspaceFolder(workspaceRoot);

        const ros = new RosWorkspace();
        await ros.diagnoseWorkspace(false);

        expect(__getMessages().warn.some((m) => m.includes('Duplicate package name'))).toBe(true);
    });

    it('reports no issues when src/ exists and there are no duplicates, when interactive', async () => {
        workspaceRoot = createTempWorkspace({
            'src/pkg_a/package.xml': '<package><name>pkg_a</name></package>',
        });
        __setWorkspaceFolder(workspaceRoot);

        const ros = new RosWorkspace();
        await ros.diagnoseWorkspace(true);

        expect(__getMessages().info.some((m) => m.includes('No workspace issues detected'))).toBe(true);
    });
});
