import path from 'node:path';
import fs from 'node:fs';
import { afterEach, describe, expect, it } from 'vitest';
import { createTempWorkspace, removeTempWorkspace } from '../../helpers/workspaceFactory/tempWorkspace';
import { BuildStampManager, StampStorage, BuildStampMap } from '../../../src/ros/buildStampManager';
import { DependencyResolver } from '../../../src/ros/dependencyResolver';
import { BuildPolicy, BuildPolicyConfig } from '../../../src/ros/buildPolicy';

describe('BuildPolicy', () => {
    let workspaceRoot = '';

    function createMockStorage(): StampStorage {
        const data: Record<string, BuildStampMap> = {};
        return {
            get: (key: string) => data[key],
            update: (key: string, value: BuildStampMap) => { data[key] = value; },
        };
    }

    function setup(files: Record<string, string>) {
        workspaceRoot = createTempWorkspace(files);
        const storage = createMockStorage();
        const stamps = new BuildStampManager(storage);
        const resolver = new DependencyResolver();
        const policy = new BuildPolicy(stamps, resolver);
        const srcDir = path.join(workspaceRoot, 'src');
        return { stamps, resolver, policy, srcDir };
    }

    afterEach(() => {
        if (workspaceRoot) {
            removeTempWorkspace(workspaceRoot);
            workspaceRoot = '';
        }
    });

    // ── Basic evaluation ───────────────────────────────────────

    it('marks never-built packages as needing build', () => {
        const { policy, srcDir } = setup({
            'src/pkg_a/package.xml': '<package><name>pkg_a</name></package>',
        });

        const result = policy.evaluateBuildNeeds(srcDir, ['pkg_a']);

        expect(result.packagesNeedingBuild).toEqual(['pkg_a']);
        expect(result.upToDate).toEqual([]);

        const detail = result.details.get('pkg_a');
        expect(detail?.reasonCode).toBe('never-built');
    });

    it('marks up-to-date packages correctly', () => {
        const { stamps, policy, srcDir } = setup({
            'src/pkg_a/package.xml': '<package><name>pkg_a</name></package>',
            'src/pkg_a/main.cpp': '// source',
        });

        // Mark as built in the far future
        stamps.markBuilt('pkg_a', 'ament_cmake', Date.now() + 100_000);

        const result = policy.evaluateBuildNeeds(srcDir, ['pkg_a']);

        expect(result.packagesNeedingBuild).toEqual([]);
        expect(result.upToDate).toEqual(['pkg_a']);
    });

    it('detects source changes since last build', () => {
        const { stamps, policy, srcDir } = setup({
            'src/pkg_a/package.xml': '<package><name>pkg_a</name></package>',
            'src/pkg_a/main.cpp': '// source',
        });

        // Stamp far in the past
        stamps.markBuilt('pkg_a', 'ament_cmake', 1);

        const result = policy.evaluateBuildNeeds(srcDir, ['pkg_a']);

        expect(result.packagesNeedingBuild).toEqual(['pkg_a']);
        const detail = result.details.get('pkg_a');
        expect(detail?.reasonCode).toBe('source-changed');
    });

    // ── Dependency propagation ─────────────────────────────────

    it('does NOT propagate build need to dependents (only changed package rebuilds)', () => {
        const { stamps, policy, srcDir } = setup({
            'src/app/package.xml': '<package><name>app</name><depend>lib</depend></package>',
            'src/app/main.cpp': '// app source',
            'src/lib/package.xml': '<package><name>lib</name></package>',
            'src/lib/src/lib.cpp': '// lib source',
        });

        // Both were built, but lib's source is newer
        stamps.markBuilt('lib', 'ament_cmake', 1);
        stamps.markBuilt('app', 'ament_cmake', Date.now() + 100_000);

        const config: BuildPolicyConfig = { policy: 'ask', fastDependencyMode: false };
        const result = policy.evaluateBuildNeeds(srcDir, ['app'], config);

        // lib needs build because of source change, app does NOT
        expect(result.packagesNeedingBuild).toContain('lib');
        expect(result.packagesNeedingBuild).not.toContain('app');
        expect(result.upToDate).toContain('app');
    });

    it('in fast mode, does not propagate even for implementation changes', () => {
        const { stamps, policy, srcDir } = setup({
            'src/app/package.xml': '<package><name>app</name><depend>lib</depend></package>',
            'src/app/main.cpp': '// app source',
            'src/lib/package.xml': '<package><name>lib</name></package>',
            'src/lib/src/impl.cpp': '// implementation change only',
        });

        // Stamp after file creation so package.xml is NOT detected as changed
        const now = Date.now();
        stamps.markBuilt('lib', 'ament_cmake', now + 500);
        stamps.markBuilt('app', 'ament_cmake', now + 100_000);

        // Touch impl.cpp to the far future so lib's source IS newer than stamp
        const implPath = path.join(srcDir, 'lib/src/impl.cpp');
        const futureTime = (now + 50_000) / 1000;
        fs.utimesSync(implPath, futureTime, futureTime);

        const config: BuildPolicyConfig = { policy: 'ask', fastDependencyMode: true };
        const result = policy.evaluateBuildNeeds(srcDir, ['app'], config);

        // Only lib needs build, app does NOT
        expect(result.packagesNeedingBuild).toContain('lib');
        expect(result.packagesNeedingBuild).not.toContain('app');
    });

    it('in fast mode, does not propagate even when dependency has interface changes', () => {
        const { stamps, policy, srcDir } = setup({
            'src/app/package.xml': '<package><name>app</name><depend>lib</depend></package>',
            'src/app/main.cpp': '// app source',
            'src/lib/package.xml': '<package><name>lib</name></package>',
            'src/lib/msg/Status.msg': 'uint8 status',
        });

        stamps.markBuilt('lib', 'ament_cmake', 1);
        stamps.markBuilt('app', 'ament_cmake', Date.now() + 100_000);

        const config: BuildPolicyConfig = { policy: 'ask', fastDependencyMode: true };
        const result = policy.evaluateBuildNeeds(srcDir, ['app'], config);

        // Only lib needs build, app does NOT
        expect(result.packagesNeedingBuild).toContain('lib');
        expect(result.packagesNeedingBuild).not.toContain('app');
        expect(result.upToDate).toContain('app');
    });

    // ── Topological ordering ───────────────────────────────────

    it('returns packages in dependency-first order', () => {
        const { policy, srcDir } = setup({
            'src/app/package.xml': '<package><name>app</name><depend>lib</depend></package>',
            'src/lib/package.xml': '<package><name>lib</name><depend>core</depend></package>',
            'src/core/package.xml': '<package><name>core</name></package>',
        });

        const result = policy.evaluateBuildNeeds(srcDir, ['app']);

        // All are never-built, order should be core → lib → app
        expect(result.packagesNeedingBuild).toEqual(['core', 'lib', 'app']);
    });

    // ── Symlink-install shortcut ───────────────────────────────

    it('skips build for ament_python when only .py files changed', () => {
        const { stamps, policy, srcDir } = setup({
            'src/py_pkg/package.xml': '<package><name>py_pkg</name></package>',
            'src/py_pkg/setup.py': 'from setuptools import setup',
            'src/py_pkg/py_pkg/__init__.py': '',
            'src/py_pkg/py_pkg/node.py': '# changed source',
        });

        // Stamp after file creation so setup.py/package.xml are NOT detected
        const now = Date.now();
        stamps.markBuilt('py_pkg', 'ament_python', now + 500);

        // Touch only node.py to the far future so source IS newer than stamp
        const nodePy = path.join(srcDir, 'py_pkg/py_pkg/node.py');
        const futureTime = (now + 50_000) / 1000;
        fs.utimesSync(nodePy, futureTime, futureTime);

        const result = policy.evaluateBuildNeeds(srcDir, ['py_pkg']);

        expect(result.packagesNeedingBuild).toEqual([]);
        const detail = result.details.get('py_pkg');
        expect(detail?.reasonCode).toBe('symlink-skip');
    });

    it('does NOT skip for ament_python when interface files changed', () => {
        const { stamps, policy, srcDir } = setup({
            'src/py_pkg/package.xml': '<package><name>py_pkg</name></package>',
            'src/py_pkg/setup.py': 'from setuptools import setup',
            'src/py_pkg/msg/Data.msg': 'float64 value',
        });

        stamps.markBuilt('py_pkg', 'ament_python', 1);

        const result = policy.evaluateBuildNeeds(srcDir, ['py_pkg']);

        // msg/ changed → must rebuild
        expect(result.packagesNeedingBuild).toContain('py_pkg');
    });

    it('does NOT skip for ament_cmake packages even with only source changes', () => {
        const { stamps, policy, srcDir } = setup({
            'src/cpp_pkg/package.xml': `
<package><name>cpp_pkg</name>
  <export><build_type>ament_cmake</build_type></export>
</package>`,
            'src/cpp_pkg/CMakeLists.txt': '',
            'src/cpp_pkg/src/main.cpp': '// changed',
        });

        stamps.markBuilt('cpp_pkg', 'ament_cmake', 1);

        const result = policy.evaluateBuildNeeds(srcDir, ['cpp_pkg']);

        expect(result.packagesNeedingBuild).toContain('cpp_pkg');
    });

    // ── Edge cases ─────────────────────────────────────────────

    it('does NOT rebuild bringup when only URDF in robot_description changes', () => {
        const { stamps, policy, srcDir } = setup({
            // bringup has a launch file that references robot_description
            'src/bringup/package.xml': '<package><name>bringup</name></package>',
            'src/bringup/launch/robot.launch.py': `
from launch_ros.substitutions import FindPackageShare
pkg = FindPackageShare('robot_description')
`,
            // robot_description has the URDF
            'src/robot_description/package.xml': '<package><name>robot_description</name></package>',
            'src/robot_description/urdf/robot.urdf': '<robot name="test"/>',
        });

        // Both were built, but then the URDF was modified
        const now = Date.now();
        stamps.markBuilt('robot_description', 'ament_cmake', now + 500);
        stamps.markBuilt('bringup', 'ament_cmake', now + 100_000);

        // Touch the URDF to the far future
        const urdfPath = path.join(srcDir, 'robot_description/urdf/robot.urdf');
        const futureTime = (now + 50_000) / 1000;
        fs.utimesSync(urdfPath, futureTime, futureTime);

        const config: BuildPolicyConfig = { policy: 'ask', fastDependencyMode: true };
        const result = policy.evaluateBuildNeeds(srcDir, ['bringup'], config);

        // robot_description needs build (source changed)
        expect(result.packagesNeedingBuild).toContain('robot_description');
        // bringup does NOT need rebuild — only the changed package rebuilds
        expect(result.packagesNeedingBuild).not.toContain('bringup');
        expect(result.upToDate).toContain('bringup');
    });

    it('does NOT rebuild nav when only config yaml in nav_params changes', () => {
        const { stamps, policy, srcDir } = setup({
            'src/nav/package.xml': '<package><name>nav</name><depend>nav_params</depend></package>',
            'src/nav/src/main.cpp': '// nav source',
            'src/nav_params/package.xml': '<package><name>nav_params</name></package>',
            'src/nav_params/config/params.yaml': 'speed: 1.0',
        });

        const now = Date.now();
        stamps.markBuilt('nav_params', 'ament_cmake', now + 500);
        stamps.markBuilt('nav', 'ament_cmake', now + 100_000);

        // Touch params.yaml
        const yamlPath = path.join(srcDir, 'nav_params/config/params.yaml');
        const futureTime = (now + 50_000) / 1000;
        fs.utimesSync(yamlPath, futureTime, futureTime);

        const config: BuildPolicyConfig = { policy: 'ask', fastDependencyMode: true };
        const result = policy.evaluateBuildNeeds(srcDir, ['nav'], config);

        // Only nav_params needs build, nav does NOT
        expect(result.packagesNeedingBuild).toContain('nav_params');
        expect(result.packagesNeedingBuild).not.toContain('nav');
        expect(result.upToDate).toContain('nav');
    });

    it('handles empty target list gracefully', () => {
        const { policy, srcDir } = setup({
            'src/pkg/package.xml': '<package><name>pkg</name></package>',
        });

        const result = policy.evaluateBuildNeeds(srcDir, []);

        expect(result.packagesNeedingBuild).toEqual([]);
        expect(result.upToDate).toEqual([]);
    });

    it('handles unknown target package gracefully', () => {
        const { policy, srcDir } = setup({
            'src/pkg/package.xml': '<package><name>pkg</name></package>',
        });

        const result = policy.evaluateBuildNeeds(srcDir, ['nonexistent']);

        expect(result.packagesNeedingBuild).toEqual([]);
    });
});
