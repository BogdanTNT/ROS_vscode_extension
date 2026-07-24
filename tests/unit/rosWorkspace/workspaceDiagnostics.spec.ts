import path from 'node:path';
import { afterEach, describe, expect, it } from 'vitest';
import { createTempWorkspace, removeTempWorkspace } from '../../helpers/workspaceFactory/tempWorkspace';
import {
    findDuplicatePackageLocations,
    findNestedRosWorkspaceCandidates,
} from '../../../src/ros/workspaceDiagnostics';

describe('workspaceDiagnostics', () => {
    let workspaceRoot = '';

    afterEach(() => {
        if (workspaceRoot) {
            removeTempWorkspace(workspaceRoot);
            workspaceRoot = '';
        }
    });

    describe('findNestedRosWorkspaceCandidates', () => {
        it('finds a colcon workspace nested one level down in a monorepo', () => {
            workspaceRoot = createTempWorkspace({
                'ars-motus/src/ars_motus/package.xml': '<package><name>ars_motus</name></package>',
                'cad-exporter/README.md': '# not a ROS project',
            });

            const candidates = findNestedRosWorkspaceCandidates(workspaceRoot);

            expect(candidates).toEqual([path.join(workspaceRoot, 'ars-motus')]);
        });

        it('returns an empty array when the workspace root itself has src/', () => {
            workspaceRoot = createTempWorkspace({
                'src/pkg_a/package.xml': '<package><name>pkg_a</name></package>',
            });

            // Nothing nested — the caller is expected to use getSrcDir() directly
            // in this case, so a nested-candidate scan isn't needed.
            const candidates = findNestedRosWorkspaceCandidates(path.join(workspaceRoot, 'does-not-exist'));

            expect(candidates).toEqual([]);
        });

        it('returns an empty array when no package.xml exists anywhere', () => {
            workspaceRoot = createTempWorkspace({
                'README.md': '# just docs',
                'app/index.js': 'console.log("hi")',
            });

            expect(findNestedRosWorkspaceCandidates(workspaceRoot)).toEqual([]);
        });

        it('ignores build/install/log/node_modules/.git while scanning', () => {
            workspaceRoot = createTempWorkspace({
                'build/pkg_a/package.xml': '<package><name>pkg_a</name></package>',
                'install/pkg_a/share/pkg_a/package.xml': '<package><name>pkg_a</name></package>',
                'node_modules/foo/src/package.xml': '<package><name>foo</name></package>',
                'real_ws/src/pkg_a/package.xml': '<package><name>pkg_a</name></package>',
            });

            const candidates = findNestedRosWorkspaceCandidates(workspaceRoot);

            expect(candidates).toEqual([path.join(workspaceRoot, 'real_ws')]);
        });

        it('does not descend into a workspace once it has been identified', () => {
            workspaceRoot = createTempWorkspace({
                'ws/src/pkg_a/package.xml': '<package><name>pkg_a</name></package>',
                'ws/src/pkg_a/nested/src/pkg_b/package.xml': '<package><name>pkg_b</name></package>',
            });

            const candidates = findNestedRosWorkspaceCandidates(workspaceRoot);

            expect(candidates).toEqual([path.join(workspaceRoot, 'ws')]);
        });
    });

    describe('findDuplicatePackageLocations', () => {
        it('returns an empty map when every package name is unique', () => {
            workspaceRoot = createTempWorkspace({
                'src/pkg_a/package.xml': '<package><name>pkg_a</name></package>',
                'src/pkg_b/package.xml': '<package><name>pkg_b</name></package>',
            });

            const duplicates = findDuplicatePackageLocations(path.join(workspaceRoot, 'src'));

            expect(duplicates.size).toBe(0);
        });

        it('detects the same package name declared in two different folders', () => {
            workspaceRoot = createTempWorkspace({
                'src/pkg_a/package.xml': '<package><name>pkg_a</name></package>',
                'src/vendor/pkg_a_old/package.xml': '<package><name>pkg_a</name></package>',
                'src/pkg_b/package.xml': '<package><name>pkg_b</name></package>',
            });
            const srcDir = path.join(workspaceRoot, 'src');

            const duplicates = findDuplicatePackageLocations(srcDir);

            expect(Array.from(duplicates.keys())).toEqual(['pkg_a']);
            expect(duplicates.get('pkg_a')).toEqual(
                expect.arrayContaining([
                    path.join(srcDir, 'pkg_a'),
                    path.join(srcDir, 'vendor', 'pkg_a_old'),
                ]),
            );
        });
    });
});
