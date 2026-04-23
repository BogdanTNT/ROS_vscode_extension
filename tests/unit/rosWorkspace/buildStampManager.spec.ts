import path from 'node:path';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { createTempWorkspace, removeTempWorkspace } from '../../helpers/workspaceFactory/tempWorkspace';
import { BuildStampManager, StampStorage, BuildStampMap } from '../../../src/ros/buildStampManager';

describe('BuildStampManager', () => {
    let workspaceRoot = '';

    /** Simple in-memory storage that mimics vscode.workspaceState. */
    function createMockStorage(): StampStorage & { data: Record<string, BuildStampMap> } {
        const data: Record<string, BuildStampMap> = {};
        return {
            data,
            get: (key: string) => data[key],
            update: (key: string, value: BuildStampMap) => { data[key] = value; },
        };
    }

    afterEach(() => {
        if (workspaceRoot) {
            removeTempWorkspace(workspaceRoot);
            workspaceRoot = '';
        }
    });

    // ── Stamp persistence ──────────────────────────────────────

    it('returns undefined for a package that has never been built', () => {
        const storage = createMockStorage();
        const manager = new BuildStampManager(storage);

        expect(manager.getStamp('my_pkg')).toBeUndefined();
    });

    it('records and retrieves a build stamp', () => {
        const storage = createMockStorage();
        const manager = new BuildStampManager(storage);

        manager.markBuilt('my_pkg', 'ament_cmake', 1000);

        const stamp = manager.getStamp('my_pkg');
        expect(stamp).toEqual({
            lastSuccessfulBuild: 1000,
            buildType: 'ament_cmake',
        });
    });

    it('persists stamps to storage on markBuilt', () => {
        const storage = createMockStorage();
        const manager = new BuildStampManager(storage);

        manager.markBuilt('pkg_a', 'ament_python', 2000);

        // A new manager reading the same storage sees the stamp
        const manager2 = new BuildStampManager(storage);
        expect(manager2.getStamp('pkg_a')).toEqual({
            lastSuccessfulBuild: 2000,
            buildType: 'ament_python',
        });
    });

    it('invalidate removes stamp for one package', () => {
        const storage = createMockStorage();
        const manager = new BuildStampManager(storage);

        manager.markBuilt('pkg_a', 'ament_cmake', 1000);
        manager.markBuilt('pkg_b', 'ament_cmake', 2000);

        manager.invalidate('pkg_a');

        expect(manager.getStamp('pkg_a')).toBeUndefined();
        expect(manager.getStamp('pkg_b')).toBeDefined();
    });

    it('invalidateAll removes all stamps', () => {
        const storage = createMockStorage();
        const manager = new BuildStampManager(storage);

        manager.markBuilt('pkg_a', 'ament_cmake', 1000);
        manager.markBuilt('pkg_b', 'ament_python', 2000);

        manager.invalidateAll();

        expect(manager.getAllStamps()).toEqual({});
    });

    // ── Mtime scanning ─────────────────────────────────────────

    it('reports needsBuild=true when no stamp exists', () => {
        workspaceRoot = createTempWorkspace({
            'src/my_pkg/package.xml': '<package><name>my_pkg</name></package>',
            'src/my_pkg/main.cpp': '// source',
        });

        const storage = createMockStorage();
        const manager = new BuildStampManager(storage);
        const pkgPath = path.join(workspaceRoot, 'src/my_pkg');

        expect(manager.needsBuild('my_pkg', pkgPath)).toBe(true);
    });

    it('reports needsBuild=false when stamp is newer than all sources', () => {
        workspaceRoot = createTempWorkspace({
            'src/my_pkg/package.xml': '<package><name>my_pkg</name></package>',
            'src/my_pkg/main.cpp': '// source',
        });

        const storage = createMockStorage();
        const manager = new BuildStampManager(storage);
        const pkgPath = path.join(workspaceRoot, 'src/my_pkg');

        // Stamp far in the future
        manager.markBuilt('my_pkg', 'ament_cmake', Date.now() + 100_000);

        expect(manager.needsBuild('my_pkg', pkgPath)).toBe(false);
    });

    it('reports needsBuild=true when source is newer than stamp', () => {
        workspaceRoot = createTempWorkspace({
            'src/my_pkg/package.xml': '<package><name>my_pkg</name></package>',
            'src/my_pkg/main.cpp': '// source',
        });

        const storage = createMockStorage();
        const manager = new BuildStampManager(storage);
        const pkgPath = path.join(workspaceRoot, 'src/my_pkg');

        // Stamp far in the past
        manager.markBuilt('my_pkg', 'ament_cmake', 1);

        expect(manager.needsBuild('my_pkg', pkgPath)).toBe(true);
    });

    it('ignores build/, install/, log/, .git/ directories when scanning', () => {
        workspaceRoot = createTempWorkspace({
            'src/my_pkg/package.xml': '<package><name>my_pkg</name></package>',
            'src/my_pkg/main.cpp': '// source',
            'src/my_pkg/build/stale.o': 'object file',
            'src/my_pkg/install/setup.bash': '#!/bin/bash',
            'src/my_pkg/log/latest_build': 'log',
            'src/my_pkg/.git/HEAD': 'ref: refs/heads/main',
        });

        const storage = createMockStorage();
        const manager = new BuildStampManager(storage);
        const pkgPath = path.join(workspaceRoot, 'src/my_pkg');

        // Stamp slightly in the future relative to the real source files
        const mtime = manager.getNewestSourceMtime(pkgPath);
        manager.markBuilt('my_pkg', 'ament_cmake', mtime + 1);

        expect(manager.needsBuild('my_pkg', pkgPath)).toBe(false);
    });

    it('ignores *.egg-info directories when scanning (colcon build artifact)', () => {
        workspaceRoot = createTempWorkspace({
            'src/my_pkg/package.xml': '<package><name>my_pkg</name></package>',
            'src/my_pkg/my_pkg/__init__.py': '# source',
            'src/my_pkg/my_pkg.egg-info/PKG-INFO': 'build artifact',
            'src/my_pkg/my_pkg.egg-info/SOURCES.txt': 'build artifact',
        });

        const storage = createMockStorage();
        const manager = new BuildStampManager(storage);
        const pkgPath = path.join(workspaceRoot, 'src/my_pkg');

        // Stamp based on real source files only
        const mtime = manager.getNewestSourceMtime(pkgPath);
        manager.markBuilt('my_pkg', 'ament_python', mtime + 1);

        // Even though egg-info files exist, they should be ignored
        expect(manager.needsBuild('my_pkg', pkgPath)).toBe(false);
    });

    it('getNewestSourceMtime returns 0 for empty or nonexistent directory', () => {
        const storage = createMockStorage();
        const manager = new BuildStampManager(storage);

        expect(manager.getNewestSourceMtime('/nonexistent/path')).toBe(0);
    });
});
