import path from 'node:path';
import fs from 'node:fs';
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

    it('detects interface changes in msg/ directory', () => {
        workspaceRoot = createTempWorkspace({
            'src/msgs/package.xml': '<package><name>msgs</name></package>',
            'src/msgs/msg/Pose.msg': 'float64 x\nfloat64 y',
        });

        const resolver = new DependencyResolver();
        // Since = 0 means everything is "changed"
        expect(resolver.hasInterfaceChanges(
            path.join(workspaceRoot, 'src/msgs'), 0,
        )).toBe(true);
    });

    it('detects interface changes in CMakeLists.txt', () => {
        workspaceRoot = createTempWorkspace({
            'src/pkg/package.xml': '<package><name>pkg</name></package>',
            'src/pkg/CMakeLists.txt': 'cmake_minimum_required(VERSION 3.8)',
        });

        const resolver = new DependencyResolver();
        expect(resolver.hasInterfaceChanges(
            path.join(workspaceRoot, 'src/pkg'), 0,
        )).toBe(true);
    });

    it('returns false when no interface files changed since timestamp', () => {
        workspaceRoot = createTempWorkspace({
            'src/pkg/package.xml': '<package><name>pkg</name></package>',
            'src/pkg/src/main.cpp': '// implementation only',
        });

        const resolver = new DependencyResolver();
        // Far in the future: nothing is newer
        expect(resolver.hasInterfaceChanges(
            path.join(workspaceRoot, 'src/pkg'), Date.now() + 100_000,
        )).toBe(false);
    });

    // ── Resource file change detection ──────────────────────────

    it('detects interface changes in urdf/ directory', () => {
        workspaceRoot = createTempWorkspace({
            'src/robot_desc/package.xml': '<package><name>robot_desc</name></package>',
            'src/robot_desc/urdf/robot.urdf': '<robot name="test"/>',
        });

        const resolver = new DependencyResolver();
        expect(resolver.hasInterfaceChanges(
            path.join(workspaceRoot, 'src/robot_desc'), 0,
        )).toBe(true);
    });

    it('detects interface changes for .xacro files', () => {
        workspaceRoot = createTempWorkspace({
            'src/robot_desc/package.xml': '<package><name>robot_desc</name></package>',
            'src/robot_desc/urdf/robot.urdf.xacro': '<robot/>',
        });

        const resolver = new DependencyResolver();
        expect(resolver.hasInterfaceChanges(
            path.join(workspaceRoot, 'src/robot_desc'), 0,
        )).toBe(true);
    });

    it('detects interface changes in config/ directory', () => {
        workspaceRoot = createTempWorkspace({
            'src/nav_pkg/package.xml': '<package><name>nav_pkg</name></package>',
            'src/nav_pkg/config/params.yaml': 'speed: 1.0',
        });

        const resolver = new DependencyResolver();
        expect(resolver.hasInterfaceChanges(
            path.join(workspaceRoot, 'src/nav_pkg'), 0,
        )).toBe(true);
    });

    it('detects .yaml file changes via extension match', () => {
        workspaceRoot = createTempWorkspace({
            'src/pkg/package.xml': '<package><name>pkg</name></package>',
            'src/pkg/params.yaml': 'key: value',
        });

        const resolver = new DependencyResolver();
        expect(resolver.hasInterfaceChanges(
            path.join(workspaceRoot, 'src/pkg'), 0,
        )).toBe(true);
    });

    it('does NOT detect .cpp as interface change', () => {
        workspaceRoot = createTempWorkspace({
            'src/pkg/package.xml': '<package><name>pkg</name></package>',
            'src/pkg/src/impl.cpp': '// just source',
        });

        const resolver = new DependencyResolver();
        // Far future: only package.xml would match, but we test with a
        // timestamp after creation
        expect(resolver.hasInterfaceChanges(
            path.join(workspaceRoot, 'src/pkg'), Date.now() + 100_000,
        )).toBe(false);
    });

    // ── Launch-file cross-reference detection ───────────────────

    it('discovers Python launch file refs via FindPackageShare', () => {
        workspaceRoot = createTempWorkspace({
            'src/bringup/package.xml': '<package><name>bringup</name></package>',
            'src/bringup/launch/robot.launch.py': `
from launch_ros.substitutions import FindPackageShare
pkg_share = FindPackageShare('robot_description')
`,
            'src/robot_description/package.xml': '<package><name>robot_description</name></package>',
            'src/robot_description/urdf/robot.urdf': '<robot/>',
        });

        const resolver = new DependencyResolver();
        const graph = resolver.buildGraph(path.join(workspaceRoot, 'src'));

        const bringup = graph.get('bringup');
        expect(bringup).toBeDefined();
        expect(bringup!.launchFileDeps).toContain('robot_description');
        expect(bringup!.localDeps).toContain('robot_description');
    });

    it('discovers Python launch file refs via get_package_share_directory', () => {
        workspaceRoot = createTempWorkspace({
            'src/bringup/package.xml': '<package><name>bringup</name></package>',
            'src/bringup/launch/start.launch.py': `
from ament_index_python.packages import get_package_share_directory
urdf = get_package_share_directory('robot_description')
`,
            'src/robot_description/package.xml': '<package><name>robot_description</name></package>',
        });

        const resolver = new DependencyResolver();
        const graph = resolver.buildGraph(path.join(workspaceRoot, 'src'));

        expect(graph.get('bringup')!.localDeps).toContain('robot_description');
    });

    it('discovers XML launch file refs via $(find pkg)', () => {
        workspaceRoot = createTempWorkspace({
            'src/bringup/package.xml': '<package><name>bringup</name></package>',
            'src/bringup/launch/robot.launch': `
<launch>
  <include file="$(find robot_description)/launch/desc.launch"/>
</launch>
`,
            'src/robot_description/package.xml': '<package><name>robot_description</name></package>',
        });

        const resolver = new DependencyResolver();
        const graph = resolver.buildGraph(path.join(workspaceRoot, 'src'));

        expect(graph.get('bringup')!.localDeps).toContain('robot_description');
    });

    it('discovers XML launch file refs via $(find-pkg-share pkg)', () => {
        workspaceRoot = createTempWorkspace({
            'src/bringup/package.xml': '<package><name>bringup</name></package>',
            'src/bringup/launch/robot.launch.xml': `
<launch>
  <let name="urdf" value="$(find-pkg-share robot_description)/urdf/robot.urdf"/>
</launch>
`,
            'src/robot_description/package.xml': '<package><name>robot_description</name></package>',
        });

        const resolver = new DependencyResolver();
        const graph = resolver.buildGraph(path.join(workspaceRoot, 'src'));

        expect(graph.get('bringup')!.localDeps).toContain('robot_description');
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
