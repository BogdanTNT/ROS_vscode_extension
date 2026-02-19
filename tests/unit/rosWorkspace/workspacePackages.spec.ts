import path from 'node:path';
import fs from 'node:fs';
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

    it('loads workspace packages from the opened folder (not only src/) while skipping ignored folders', async () => {
        workspaceRoot = createTempWorkspace({
            'tools_pkg/package.xml': '<package><name>tools_pkg</name></package>',
            'tools_pkg/launch/tools.launch.py': '# launch file',
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
            'build/ignored_pkg/package.xml': '<package><name>ignored_root_build</name></package>',
            'install/ignored_pkg/package.xml': '<package><name>ignored_root_install</name></package>',
            'src/build/ignored_pkg/package.xml': '<package><name>ignored_pkg</name></package>',
            'src/install/ignored_pkg/package.xml': '<package><name>ignored_install</name></package>',
            'src/pkg_bad/package.xml': '<package><name></name></package>',
        });

        __setWorkspaceFolder(workspaceRoot);
        const ros = new RosWorkspace();

        const names = await ros.listWorkspacePackages();
        const details = await ros.listWorkspacePackageDetails();

        expect(names).toEqual(['pkg_a', 'pkg_b', 'tools_pkg']);
        expect(details.map((pkg) => pkg.name)).toEqual(['pkg_a', 'pkg_b', 'tools_pkg']);

        const pkgA = details.find((pkg) => pkg.name === 'pkg_a');
        expect(pkgA?.launchFiles).toEqual([
            path.join(workspaceRoot, 'src/pkg_a/launch/main.launch.py'),
            path.join(workspaceRoot, 'src/pkg_a/launch/nested/extra.xml'),
        ]);
        expect(pkgA?.isPython).toBe(true);
        expect(pkgA?.isCmake).toBe(false);
        expect(pkgA?.nodes).toEqual([
            {
                name: 'talker',
                sourcePath: path.join(workspaceRoot, 'src/pkg_a/pkg_a/talker.py'),
            },
        ]);

        const pkgB = details.find((pkg) => pkg.name === 'pkg_b');
        expect(pkgB?.launchFiles).toEqual([]);
        expect(pkgB?.isPython).toBe(false);
        expect(pkgB?.isCmake).toBe(true);
        expect(pkgB?.nodes).toEqual([
            {
                name: 'cpp_node',
                sourcePath: path.join(workspaceRoot, 'src/pkg_b/src/cpp_node.cpp'),
            },
        ]);

        const toolsPkg = details.find((pkg) => pkg.name === 'tools_pkg');
        expect(toolsPkg?.launchFiles).toEqual([
            path.join(workspaceRoot, 'tools_pkg/launch/tools.launch.py'),
        ]);
        expect(toolsPkg?.isPython).toBe(false);
        expect(toolsPkg?.isCmake).toBe(false);
        expect(toolsPkg?.nodes).toEqual([]);
    });

    it('discovers workspace packages even when the opened folder has no src/ directory', async () => {
        workspaceRoot = createTempWorkspace({
            'bringup/package.xml': '<package><name>bringup</name></package>',
            'bringup/launch/demo.launch.xml': '<launch/>',
        });

        const srcPath = path.join(workspaceRoot, 'src');
        expect(fs.existsSync(srcPath)).toBe(false);

        __setWorkspaceFolder(workspaceRoot);
        const ros = new RosWorkspace();

        const names = await ros.listWorkspacePackages();
        const details = await ros.listWorkspacePackageDetails();

        expect(names).toEqual(['bringup']);
        expect(details.map((pkg) => pkg.name)).toEqual(['bringup']);
        expect(details[0].launchFiles).toEqual([
            path.join(workspaceRoot, 'bringup/launch/demo.launch.xml'),
        ]);
        // Listing packages should not create src/ as a side effect.
        expect(fs.existsSync(srcPath)).toBe(false);
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
                isCmake: false,
            },
        ]);
    });

    it('adds a node to a workspace package outside root src/ using the provided package path hint', async () => {
        workspaceRoot = createTempWorkspace({
            'test_export/src/test_with_git_user/package.xml': '<package><name>test_with_git_user</name></package>',
            'test_export/src/test_with_git_user/setup.py': `
from setuptools import setup

setup(
    name='test_with_git_user',
    entry_points={
        'console_scripts': [
        ],
    },
)
`,
            'test_export/src/test_with_git_user/test_with_git_user/__init__.py': '',
        });

        __setWorkspaceFolder(workspaceRoot);
        const ros = new RosWorkspace();

        const packagePath = path.join(workspaceRoot, 'test_export/src/test_with_git_user');
        const created = await ros.addNodeToPackage(
            'test_with_git_user',
            'new_node',
            packagePath,
        );

        expect(created).toBe(true);
        expect(
            fs.existsSync(path.join(packagePath, 'test_with_git_user/new_node.py')),
        ).toBe(true);

        const setupPy = fs.readFileSync(path.join(packagePath, 'setup.py'), 'utf8');
        expect(setupPy).toContain("'new_node = test_with_git_user.new_node:main'");
    });

    it('adds a node to a workspace package outside root src/ by resolving package path from workspace scan', async () => {
        workspaceRoot = createTempWorkspace({
            'test_export/src/test_with_git_user/package.xml': '<package><name>test_with_git_user</name></package>',
            'test_export/src/test_with_git_user/setup.py': `
from setuptools import setup

setup(
    name='test_with_git_user',
    entry_points={
        'console_scripts': [
        ],
    },
)
`,
            'test_export/src/test_with_git_user/test_with_git_user/__init__.py': '',
        });

        __setWorkspaceFolder(workspaceRoot);
        const ros = new RosWorkspace();

        const created = await ros.addNodeToPackage('test_with_git_user', 'scan_node');
        expect(created).toBe(true);

        const packagePath = path.join(workspaceRoot, 'test_export/src/test_with_git_user');
        expect(
            fs.existsSync(path.join(packagePath, 'test_with_git_user/scan_node.py')),
        ).toBe(true);
    });

    it('uses the selected publisher template when creating a python node', async () => {
        workspaceRoot = createTempWorkspace({
            'src/pkg_a/package.xml': '<package><name>pkg_a</name></package>',
            'src/pkg_a/setup.py': `
from setuptools import setup

setup(
    name='pkg_a',
    entry_points={
        'console_scripts': [
        ],
    },
)
`,
            'src/pkg_a/pkg_a/__init__.py': '',
        });

        __setWorkspaceFolder(workspaceRoot);
        const ros = new RosWorkspace();

        const created = await ros.addNodeToPackage('pkg_a', 'pub_node', undefined, 'publisher');
        expect(created).toBe(true);

        const nodeFilePath = path.join(workspaceRoot, 'src/pkg_a/pkg_a/pub_node.py');
        const nodeFile = fs.readFileSync(nodeFilePath, 'utf8');
        expect(nodeFile).toContain('from std_msgs.msg import String');
        expect(nodeFile).toContain('self.create_publisher(String, \'chatter\', 10)');
    });

    it('normalizes topic spaces to underscores when creating a subscriber template node', async () => {
        workspaceRoot = createTempWorkspace({
            'src/pkg_a/package.xml': '<package><name>pkg_a</name></package>',
            'src/pkg_a/setup.py': `
from setuptools import setup

setup(
    name='pkg_a',
    entry_points={
        'console_scripts': [
        ],
    },
)
`,
            'src/pkg_a/pkg_a/__init__.py': '',
        });

        __setWorkspaceFolder(workspaceRoot);
        const ros = new RosWorkspace();

        const created = await ros.addNodeToPackage(
            'pkg_a',
            'sub_node',
            undefined,
            'subscriber',
            '/scan lidar',
        );
        expect(created).toBe(true);

        const nodeFilePath = path.join(workspaceRoot, 'src/pkg_a/pkg_a/sub_node.py');
        const nodeFile = fs.readFileSync(nodeFilePath, 'utf8');
        expect(nodeFile).toContain('self.create_subscription(');
        expect(nodeFile).toContain('\'/scan_lidar\'');
    });

    it('uses tutorial-style AddTwoInts service template for python nodes', async () => {
        workspaceRoot = createTempWorkspace({
            'src/pkg_a/package.xml': '<package><name>pkg_a</name></package>',
            'src/pkg_a/setup.py': `
from setuptools import setup

setup(
    name='pkg_a',
    entry_points={
        'console_scripts': [
        ],
    },
)
`,
            'src/pkg_a/pkg_a/__init__.py': '',
        });

        __setWorkspaceFolder(workspaceRoot);
        const ros = new RosWorkspace();

        const created = await ros.addNodeToPackage('pkg_a', 'srv_node', undefined, 'service');
        expect(created).toBe(true);

        const nodeFilePath = path.join(workspaceRoot, 'src/pkg_a/pkg_a/srv_node.py');
        const nodeFile = fs.readFileSync(nodeFilePath, 'utf8');
        expect(nodeFile).toContain('from example_interfaces.srv import AddTwoInts');
        expect(nodeFile).toContain("self.service_ = self.create_service(AddTwoInts, 'add_two_ints'");
        expect(nodeFile).toContain('response.sum = request.a + request.b');
        expect(nodeFile).toContain('def add_two_ints_callback(');
    });

    it('uses tutorial-style AddTwoInts client template for python nodes', async () => {
        workspaceRoot = createTempWorkspace({
            'src/pkg_a/package.xml': '<package><name>pkg_a</name></package>',
            'src/pkg_a/setup.py': `
from setuptools import setup

setup(
    name='pkg_a',
    entry_points={
        'console_scripts': [
        ],
    },
)
`,
            'src/pkg_a/pkg_a/__init__.py': '',
        });

        __setWorkspaceFolder(workspaceRoot);
        const ros = new RosWorkspace();

        const created = await ros.addNodeToPackage('pkg_a', 'cli_node', undefined, 'client');
        expect(created).toBe(true);

        const nodeFilePath = path.join(workspaceRoot, 'src/pkg_a/pkg_a/cli_node.py');
        const nodeFile = fs.readFileSync(nodeFilePath, 'utf8');
        expect(nodeFile).toContain('from example_interfaces.srv import AddTwoInts');
        expect(nodeFile).toContain("self.client_ = self.create_client(AddTwoInts, 'add_two_ints')");
        expect(nodeFile).toContain('future = node.send_request(2, 3)');
        expect(nodeFile).toContain('rclpy.spin_until_future_complete(node, future)');
    });

    it('uses publisher template for C++ node and adds std_msgs dependency', async () => {
        workspaceRoot = createTempWorkspace({
            'test_export/src/test_cpp_pkg/package.xml': '<package><name>test_cpp_pkg</name></package>',
            'test_export/src/test_cpp_pkg/CMakeLists.txt': `
cmake_minimum_required(VERSION 3.8)
project(test_cpp_pkg)

find_package(ament_cmake REQUIRED)

ament_package()
`,
        });

        __setWorkspaceFolder(workspaceRoot);
        const ros = new RosWorkspace();

        const packagePath = path.join(workspaceRoot, 'test_export/src/test_cpp_pkg');
        const created = await ros.addNodeToPackage(
            'test_cpp_pkg',
            'cpp_pub',
            packagePath,
            'publisher',
            '/scan',
        );

        expect(created).toBe(true);
        const cppFile = fs.readFileSync(path.join(packagePath, 'src/cpp_pub.cpp'), 'utf8');
        expect(cppFile).toContain('#include "std_msgs/msg/string.hpp"');
        expect(cppFile).toContain('create_publisher<std_msgs::msg::String>("/scan", 10)');
        expect(cppFile).toContain('msg.data = "hello world my name is cpp_pub"');
        expect(cppFile).toContain('timer_->cancel()');

        const cmake = fs.readFileSync(path.join(packagePath, 'CMakeLists.txt'), 'utf8');
        expect(cmake).toContain('find_package(std_msgs REQUIRED)');
        expect(cmake).toContain('ament_target_dependencies(cpp_pub rclcpp std_msgs)');
    });

    it('uses subscriber template for C++ node and prints received messages', async () => {
        workspaceRoot = createTempWorkspace({
            'test_export/src/test_cpp_pkg/package.xml': '<package><name>test_cpp_pkg</name></package>',
            'test_export/src/test_cpp_pkg/CMakeLists.txt': `
cmake_minimum_required(VERSION 3.8)
project(test_cpp_pkg)

find_package(ament_cmake REQUIRED)

ament_package()
`,
        });

        __setWorkspaceFolder(workspaceRoot);
        const ros = new RosWorkspace();

        const packagePath = path.join(workspaceRoot, 'test_export/src/test_cpp_pkg');
        const created = await ros.addNodeToPackage(
            'test_cpp_pkg',
            'cpp_sub',
            packagePath,
            'subscriber',
            '/scan',
        );

        expect(created).toBe(true);
        const cppFile = fs.readFileSync(path.join(packagePath, 'src/cpp_sub.cpp'), 'utf8');
        expect(cppFile).toContain('create_subscription<std_msgs::msg::String>(');
        expect(cppFile).toContain('"/scan"');
        expect(cppFile).toContain('Received: %s');
    });

    it('uses AddTwoInts service template for C++ nodes and adds example_interfaces dependency', async () => {
        workspaceRoot = createTempWorkspace({
            'test_export/src/test_cpp_pkg/package.xml': '<package><name>test_cpp_pkg</name></package>',
            'test_export/src/test_cpp_pkg/CMakeLists.txt': `
cmake_minimum_required(VERSION 3.8)
project(test_cpp_pkg)

find_package(ament_cmake REQUIRED)

ament_package()
`,
        });

        __setWorkspaceFolder(workspaceRoot);
        const ros = new RosWorkspace();

        const packagePath = path.join(workspaceRoot, 'test_export/src/test_cpp_pkg');
        const created = await ros.addNodeToPackage(
            'test_cpp_pkg',
            'cpp_service',
            packagePath,
            'service',
        );
        expect(created).toBe(true);

        const cppFile = fs.readFileSync(path.join(packagePath, 'src/cpp_service.cpp'), 'utf8');
        expect(cppFile).toContain('#include "example_interfaces/srv/add_two_ints.hpp"');
        expect(cppFile).toContain('create_service<example_interfaces::srv::AddTwoInts>(');
        expect(cppFile).toContain('response->sum = request->a + request->b;');

        const cmake = fs.readFileSync(path.join(packagePath, 'CMakeLists.txt'), 'utf8');
        expect(cmake).toContain('find_package(example_interfaces REQUIRED)');
        expect(cmake).toContain('ament_target_dependencies(cpp_service rclcpp example_interfaces)');
    });

    it('uses AddTwoInts client template for C++ nodes', async () => {
        workspaceRoot = createTempWorkspace({
            'test_export/src/test_cpp_pkg/package.xml': '<package><name>test_cpp_pkg</name></package>',
            'test_export/src/test_cpp_pkg/CMakeLists.txt': `
cmake_minimum_required(VERSION 3.8)
project(test_cpp_pkg)

find_package(ament_cmake REQUIRED)

ament_package()
`,
        });

        __setWorkspaceFolder(workspaceRoot);
        const ros = new RosWorkspace();

        const packagePath = path.join(workspaceRoot, 'test_export/src/test_cpp_pkg');
        const created = await ros.addNodeToPackage(
            'test_cpp_pkg',
            'cpp_client',
            packagePath,
            'client',
        );
        expect(created).toBe(true);

        const cppFile = fs.readFileSync(path.join(packagePath, 'src/cpp_client.cpp'), 'utf8');
        expect(cppFile).toContain('#include "example_interfaces/srv/add_two_ints.hpp"');
        expect(cppFile).toContain('create_client<example_interfaces::srv::AddTwoInts>("add_two_ints")');
        expect(cppFile).toContain('rclcpp::spin_until_future_complete(node, future)');
    });

    it('adds a C++ node and updates CMakeLists.txt for a workspace ament_cmake package', async () => {
        workspaceRoot = createTempWorkspace({
            'test_export/src/test_cpp_pkg/package.xml': '<package><name>test_cpp_pkg</name></package>',
            'test_export/src/test_cpp_pkg/CMakeLists.txt': `
cmake_minimum_required(VERSION 3.8)
project(test_cpp_pkg)

find_package(ament_cmake REQUIRED)

ament_package()
`,
        });

        __setWorkspaceFolder(workspaceRoot);
        const ros = new RosWorkspace();

        const packagePath = path.join(workspaceRoot, 'test_export/src/test_cpp_pkg');
        const createdFirst = await ros.addNodeToPackage(
            'test_cpp_pkg',
            'new_cpp_node',
            packagePath,
        );
        const createdSecond = await ros.addNodeToPackage(
            'test_cpp_pkg',
            'new_cpp_node',
            packagePath,
        );

        expect(createdFirst).toBe(true);
        expect(createdSecond).toBe(true);
        expect(fs.existsSync(path.join(packagePath, 'src/new_cpp_node.cpp'))).toBe(true);

        const cmake = fs.readFileSync(path.join(packagePath, 'CMakeLists.txt'), 'utf8');
        expect(cmake).toContain('find_package(rclcpp REQUIRED)');
        expect(cmake).toContain('add_executable(new_cpp_node src/new_cpp_node.cpp)');
        expect(cmake).toContain('ament_target_dependencies(new_cpp_node rclcpp)');
        expect(cmake).toContain('install(TARGETS');
        expect(cmake).toContain('DESTINATION lib/${PROJECT_NAME}');
        expect((cmake.match(/add_executable\(new_cpp_node\s+src\/new_cpp_node\.cpp\)/g) || []).length).toBe(1);
    });

    it('removes a python node source and unregisters it from setup.py', async () => {
        workspaceRoot = createTempWorkspace({
            'src/pkg_a/package.xml': '<package><name>pkg_a</name></package>',
            'src/pkg_a/setup.py': `
from setuptools import setup

setup(
    name='pkg_a',
    entry_points={
        'console_scripts': [
            'talker = pkg_a.talker:main',
            'listener = pkg_a.listener:main',
        ],
    },
)
`,
            'src/pkg_a/pkg_a/__init__.py': '',
            'src/pkg_a/pkg_a/talker.py': 'def main(): pass',
            'src/pkg_a/pkg_a/listener.py': 'def main(): pass',
        });

        __setWorkspaceFolder(workspaceRoot);
        const ros = new RosWorkspace();

        const removed = await ros.removeNodeFromPackage('pkg_a', 'talker');
        expect(removed).toBe(true);

        const packagePath = path.join(workspaceRoot, 'src/pkg_a');
        expect(fs.existsSync(path.join(packagePath, 'pkg_a/talker.py'))).toBe(false);
        expect(fs.existsSync(path.join(packagePath, 'pkg_a/listener.py'))).toBe(true);

        const setupPy = fs.readFileSync(path.join(packagePath, 'setup.py'), 'utf8');
        expect(setupPy).not.toContain("'talker = pkg_a.talker:main'");
        expect(setupPy).toContain("'listener = pkg_a.listener:main'");

        const details = await ros.listWorkspacePackageDetails();
        expect(details.find((pkg) => pkg.name === 'pkg_a')?.nodes).toEqual([
            {
                name: 'listener',
                sourcePath: path.join(packagePath, 'pkg_a/listener.py'),
            },
        ]);
    });

    it('removes a python node registration from setup.cfg when entry points are defined there', async () => {
        workspaceRoot = createTempWorkspace({
            'src/pkg_cfg/package.xml': '<package><name>pkg_cfg</name></package>',
            'src/pkg_cfg/setup.py': `
from setuptools import setup

setup(
    name='pkg_cfg',
)
`,
            'src/pkg_cfg/setup.cfg': `
[metadata]
name = pkg_cfg

[options.entry_points]
console_scripts =
    cfg_node = pkg_cfg.cfg_node:main
    keep_node = pkg_cfg.keep_node:main
`,
            'src/pkg_cfg/pkg_cfg/__init__.py': '',
            'src/pkg_cfg/pkg_cfg/cfg_node.py': 'def main(): pass',
            'src/pkg_cfg/pkg_cfg/keep_node.py': 'def main(): pass',
        });

        __setWorkspaceFolder(workspaceRoot);
        const ros = new RosWorkspace();

        const removed = await ros.removeNodeFromPackage('pkg_cfg', 'cfg_node');
        expect(removed).toBe(true);

        const packagePath = path.join(workspaceRoot, 'src/pkg_cfg');
        expect(fs.existsSync(path.join(packagePath, 'pkg_cfg/cfg_node.py'))).toBe(false);
        expect(fs.existsSync(path.join(packagePath, 'pkg_cfg/keep_node.py'))).toBe(true);

        const setupCfg = fs.readFileSync(path.join(packagePath, 'setup.cfg'), 'utf8');
        expect(setupCfg).not.toContain('cfg_node = pkg_cfg.cfg_node:main');
        expect(setupCfg).toContain('keep_node = pkg_cfg.keep_node:main');
    });

    it('removes a C++ node target from CMakeLists.txt while keeping other node targets intact', async () => {
        workspaceRoot = createTempWorkspace({
            'src/cpp_pkg/package.xml': '<package><name>cpp_pkg</name></package>',
            'src/cpp_pkg/CMakeLists.txt': `
cmake_minimum_required(VERSION 3.8)
project(cpp_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(node_a src/node_a.cpp)
ament_target_dependencies(node_a rclcpp)

add_executable(node_b src/node_b.cpp)
ament_target_dependencies(node_b rclcpp)

install(TARGETS
  node_a
  node_b
  DESTINATION lib/\${PROJECT_NAME}
)

ament_package()
`,
            'src/cpp_pkg/src/node_a.cpp': 'int main() { return 0; }',
            'src/cpp_pkg/src/node_b.cpp': 'int main() { return 0; }',
        });

        __setWorkspaceFolder(workspaceRoot);
        const ros = new RosWorkspace();

        const packagePath = path.join(workspaceRoot, 'src/cpp_pkg');
        const removed = await ros.removeNodeFromPackage(
            'cpp_pkg',
            'node_a',
            packagePath,
            path.join(packagePath, 'src/node_a.cpp'),
        );
        expect(removed).toBe(true);

        const cmake = fs.readFileSync(path.join(packagePath, 'CMakeLists.txt'), 'utf8');
        expect(cmake).not.toContain('add_executable(node_a');
        expect(cmake).not.toContain('ament_target_dependencies(node_a');
        expect(cmake).toContain('add_executable(node_b src/node_b.cpp)');
        expect(cmake).toContain('ament_target_dependencies(node_b rclcpp)');
        expect(cmake).not.toContain('\n  node_a\n');
        expect(cmake).toContain('node_b');

        expect(fs.existsSync(path.join(packagePath, 'src/node_a.cpp'))).toBe(false);
        expect(fs.existsSync(path.join(packagePath, 'src/node_b.cpp'))).toBe(true);

        const details = await ros.listWorkspacePackageDetails();
        expect(details.find((pkg) => pkg.name === 'cpp_pkg')?.nodes).toEqual([
            {
                name: 'node_b',
                sourcePath: path.join(packagePath, 'src/node_b.cpp'),
            },
        ]);
    });
});
