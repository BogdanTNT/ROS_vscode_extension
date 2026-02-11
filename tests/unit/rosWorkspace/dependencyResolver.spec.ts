import path from 'node:path';
import { afterEach, describe, expect, it } from 'vitest';
import { createTempWorkspace, removeTempWorkspace } from '../../helpers/workspaceFactory/tempWorkspace';
import { DependencyResolver } from '../../../src/ros/dependencyResolver';

describe('DependencyResolver', () => {
    let workspaceRoot = '';

    afterEach(() => {
        if (workspaceRoot) {
            removeTempWorkspace(workspaceRoot);
            workspaceRoot = '';
        }
    });

    // ── Graph building ─────────────────────────────────────────

    it('builds a graph from package.xml files in src/', () => {
        workspaceRoot = createTempWorkspace({
            'src/pkg_a/package.xml': `
<package format="3">
  <name>pkg_a</name>
  <depend>rclcpp</depend>
  <depend>pkg_b</depend>
  <export><build_type>ament_cmake</build_type></export>
</package>`,
            'src/pkg_a/CMakeLists.txt': '',
            'src/pkg_b/package.xml': `
<package format="3">
  <name>pkg_b</name>
  <depend>rclcpp</depend>
  <export><build_type>ament_cmake</build_type></export>
</package>`,
            'src/pkg_b/CMakeLists.txt': '',
        });

        const resolver = new DependencyResolver();
        const srcDir = path.join(workspaceRoot, 'src');
        const graph = resolver.buildGraph(srcDir);

        expect(graph.size).toBe(2);

        const pkgA = graph.get('pkg_a');
        expect(pkgA).toBeDefined();
        expect(pkgA!.buildType).toBe('ament_cmake');
        // rclcpp is NOT workspace-local, so filtered out
        expect(pkgA!.localDeps).toEqual(['pkg_b']);

        const pkgB = graph.get('pkg_b');
        expect(pkgB).toBeDefined();
        expect(pkgB!.localDeps).toEqual([]);
    });

    it('parses multiple dependency tag types', () => {
        workspaceRoot = createTempWorkspace({
            'src/pkg_x/package.xml': `
<package format="3">
  <name>pkg_x</name>
  <build_depend>pkg_y</build_depend>
  <exec_depend>pkg_z</exec_depend>
  <depend>pkg_y</depend>
</package>`,
            'src/pkg_y/package.xml': '<package><name>pkg_y</name></package>',
            'src/pkg_z/package.xml': '<package><name>pkg_z</name></package>',
        });

        const resolver = new DependencyResolver();
        const graph = resolver.buildGraph(path.join(workspaceRoot, 'src'));

        const pkgX = graph.get('pkg_x');
        expect(pkgX!.localDeps.sort()).toEqual(['pkg_y', 'pkg_z']);
    });

    it('ignores build/, install/, log/ directories', () => {
        workspaceRoot = createTempWorkspace({
            'src/real_pkg/package.xml': '<package><name>real_pkg</name></package>',
            'src/build/hidden/package.xml': '<package><name>hidden</name></package>',
            'src/install/hidden2/package.xml': '<package><name>hidden2</name></package>',
        });

        const resolver = new DependencyResolver();
        const graph = resolver.buildGraph(path.join(workspaceRoot, 'src'));

        expect(graph.size).toBe(1);
        expect(graph.has('real_pkg')).toBe(true);
    });

    it('infers ament_python build type from setup.py', () => {
        workspaceRoot = createTempWorkspace({
            'src/py_pkg/package.xml': '<package><name>py_pkg</name></package>',
            'src/py_pkg/setup.py': 'from setuptools import setup',
        });

        const resolver = new DependencyResolver();
        const graph = resolver.buildGraph(path.join(workspaceRoot, 'src'));

        expect(graph.get('py_pkg')!.buildType).toBe('ament_python');
    });

    // ── Dependency closure ─────────────────────────────────────

    it('returns transitive closure in topological order', () => {
        workspaceRoot = createTempWorkspace({
            'src/app/package.xml': `
<package><name>app</name><depend>lib</depend></package>`,
            'src/lib/package.xml': `
<package><name>lib</name><depend>core</depend></package>`,
            'src/core/package.xml': `
<package><name>core</name></package>`,
        });

        const resolver = new DependencyResolver();
        const graph = resolver.buildGraph(path.join(workspaceRoot, 'src'));
        const closure = resolver.getClosure(['app'], graph);

        // core must come before lib, lib before app
        expect(closure).toEqual(['core', 'lib', 'app']);
    });

    it('handles diamond dependencies without duplication', () => {
        workspaceRoot = createTempWorkspace({
            'src/top/package.xml': '<package><name>top</name><depend>left</depend><depend>right</depend></package>',
            'src/left/package.xml': '<package><name>left</name><depend>base</depend></package>',
            'src/right/package.xml': '<package><name>right</name><depend>base</depend></package>',
            'src/base/package.xml': '<package><name>base</name></package>',
        });

        const resolver = new DependencyResolver();
        const graph = resolver.buildGraph(path.join(workspaceRoot, 'src'));
        const closure = resolver.getClosure(['top'], graph);

        expect(closure).toContain('base');
        expect(closure).toContain('left');
        expect(closure).toContain('right');
        expect(closure).toContain('top');
        // No duplicates
        expect(new Set(closure).size).toBe(closure.length);
        // base before left and right
        expect(closure.indexOf('base')).toBeLessThan(closure.indexOf('left'));
        expect(closure.indexOf('base')).toBeLessThan(closure.indexOf('right'));
    });

    // ── Reverse dependencies ───────────────────────────────────

    it('finds dependents of a package', () => {
        workspaceRoot = createTempWorkspace({
            'src/app/package.xml': '<package><name>app</name><depend>lib</depend></package>',
            'src/lib/package.xml': '<package><name>lib</name></package>',
            'src/other/package.xml': '<package><name>other</name></package>',
        });

        const resolver = new DependencyResolver();
        const graph = resolver.buildGraph(path.join(workspaceRoot, 'src'));
        const dependents = resolver.getDependents('lib', graph);

        expect(dependents).toEqual(['app']);
    });

    // ── Interface change detection ─────────────────────────────

    it.each([
        {
            name: 'msg/ directory',
            packageName: 'msgs',
            relativePath: 'msg/Pose.msg',
            content: 'float64 x\nfloat64 y',
        },
        {
            name: 'CMakeLists.txt',
            packageName: 'pkg',
            relativePath: 'CMakeLists.txt',
            content: 'cmake_minimum_required(VERSION 3.8)',
        },
        {
            name: 'urdf/ directory',
            packageName: 'robot_desc',
            relativePath: 'urdf/robot.urdf',
            content: '<robot name="test"/>',
        },
        {
            name: '.xacro files',
            packageName: 'robot_desc',
            relativePath: 'urdf/robot.urdf.xacro',
            content: '<robot/>',
        },
        {
            name: 'config/ directory',
            packageName: 'nav_pkg',
            relativePath: 'config/params.yaml',
            content: 'speed: 1.0',
        },
        {
            name: '.yaml file extension',
            packageName: 'pkg',
            relativePath: 'params.yaml',
            content: 'key: value',
        },
    ])('detects interface changes in $name', ({ packageName, relativePath, content }) => {
        workspaceRoot = createTempWorkspace({
            [`src/${packageName}/package.xml`]: `<package><name>${packageName}</name></package>`,
            [`src/${packageName}/${relativePath}`]: content,
        });

        const resolver = new DependencyResolver();
        expect(resolver.hasInterfaceChanges(
            path.join(workspaceRoot, `src/${packageName}`), 0,
        )).toBe(true);
    });

    it.each([
        {
            name: 'implementation-only source file',
            packageName: 'pkg',
            relativePath: 'src/main.cpp',
        },
        {
            name: 'non-interface .cpp file',
            packageName: 'pkg',
            relativePath: 'src/impl.cpp',
        },
    ])('returns false when $name changed after the comparison timestamp', ({ packageName, relativePath }) => {
        workspaceRoot = createTempWorkspace({
            [`src/${packageName}/package.xml`]: `<package><name>${packageName}</name></package>`,
            [`src/${packageName}/${relativePath}`]: '// implementation only',
        });

        const resolver = new DependencyResolver();
        expect(resolver.hasInterfaceChanges(
            path.join(workspaceRoot, `src/${packageName}`), Date.now() + 100_000,
        )).toBe(false);
    });

    // ── Launch-file cross-reference detection ───────────────────

    it.each([
        {
            name: 'FindPackageShare',
            launchPath: 'src/bringup/launch/robot.launch.py',
            launchContent: `
from launch_ros.substitutions import FindPackageShare
pkg_share = FindPackageShare('robot_description')
`,
        },
        {
            name: 'get_package_share_directory',
            launchPath: 'src/bringup/launch/start.launch.py',
            launchContent: `
from ament_index_python.packages import get_package_share_directory
urdf = get_package_share_directory('robot_description')
`,
        },
        {
            name: 'get_package_share_path',
            launchPath: 'src/bringup/launch/path.launch.py',
            launchContent: `
from ament_index_python.packages import get_package_share_path
urdf = get_package_share_path('robot_description')
`,
        },
        {
            name: 'PackageShareDirectory(package=...)',
            launchPath: 'src/bringup/launch/share.launch.py',
            launchContent: `
from launch_ros.substitutions import PackageShareDirectory
urdf = PackageShareDirectory(package='robot_description')
`,
        },
        {
            name: '$(find pkg)',
            launchPath: 'src/bringup/launch/robot.launch',
            launchContent: `
<launch>
  <include file="$(find robot_description)/launch/desc.launch"/>
</launch>
`,
        },
        {
            name: '$(find-pkg-share pkg)',
            launchPath: 'src/bringup/launch/robot.launch.xml',
            launchContent: `
<launch>
  <let name="urdf" value="$(find-pkg-share robot_description)/urdf/robot.urdf"/>
</launch>
`,
        },
    ])('discovers launch-file refs via $name', ({ launchPath, launchContent }) => {
        workspaceRoot = createTempWorkspace({
            'src/bringup/package.xml': '<package><name>bringup</name></package>',
            [launchPath]: launchContent,
            'src/robot_description/package.xml': '<package><name>robot_description</name></package>',
        });

        const resolver = new DependencyResolver();
        const graph = resolver.buildGraph(path.join(workspaceRoot, 'src'));

        const bringup = graph.get('bringup');
        expect(bringup).toBeDefined();
        expect(bringup!.launchFileDeps).toContain('robot_description');
        expect(bringup!.localDeps).toContain('robot_description');
    });

    it('does not include self-references in launchFileDeps', () => {
        workspaceRoot = createTempWorkspace({
            'src/pkg/package.xml': '<package><name>pkg</name></package>',
            'src/pkg/launch/start.launch.py': `
pkg = FindPackageShare('pkg')
`,
        });

        const resolver = new DependencyResolver();
        const graph = resolver.buildGraph(path.join(workspaceRoot, 'src'));

        expect(graph.get('pkg')!.launchFileDeps).toEqual([]);
    });

    it('filters out non-workspace packages from launchFileDeps', () => {
        workspaceRoot = createTempWorkspace({
            'src/bringup/package.xml': '<package><name>bringup</name></package>',
            'src/bringup/launch/start.launch.py': `
gazebo = FindPackageShare('gazebo_ros')
robot = FindPackageShare('robot_description')
`,
            'src/robot_description/package.xml': '<package><name>robot_description</name></package>',
        });

        const resolver = new DependencyResolver();
        const graph = resolver.buildGraph(path.join(workspaceRoot, 'src'));

        // gazebo_ros is not in the workspace, only robot_description is
        expect(graph.get('bringup')!.launchFileDeps).toEqual(['robot_description']);
        expect(graph.get('bringup')!.localDeps).toContain('robot_description');
    });
});
