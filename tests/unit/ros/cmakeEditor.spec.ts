import { describe, expect, it } from 'vitest';
import {
    ensureFindPackageDependency,
    ensureCppNodeRegisteredInCmake,
    ensureLaunchDirectoryInstalledInCmake,
    removeTargetFromCmake,
} from '../../../src/ros/build/cmakeEditor';

const BASE_CMAKE = [
    'cmake_minimum_required(VERSION 3.8)',
    'project(my_pkg)',
    'find_package(ament_cmake REQUIRED)',
    '',
    'ament_package()',
    '',
].join('\n');

describe('cmakeEditor/ensureFindPackageDependency', () => {
    it('adds a find_package after ament_cmake', () => {
        const result = ensureFindPackageDependency(BASE_CMAKE, 'rclcpp');
        expect(result).toContain('find_package(rclcpp REQUIRED)');
    });

    it('is idempotent', () => {
        const once = ensureFindPackageDependency(BASE_CMAKE, 'rclcpp');
        const twice = ensureFindPackageDependency(once, 'rclcpp');
        expect(twice).toBe(once);
    });

    it('falls back to inserting after project() when ament_cmake is absent', () => {
        const cmake = 'cmake_minimum_required(VERSION 3.8)\nproject(my_pkg)\n';
        const result = ensureFindPackageDependency(cmake, 'rclcpp');
        expect(result).toContain('find_package(rclcpp REQUIRED)');
        expect(result.indexOf('project(my_pkg)')).toBeLessThan(result.indexOf('find_package(rclcpp REQUIRED)'));
    });
});

describe('cmakeEditor/ensureCppNodeRegisteredInCmake', () => {
    it('registers add_executable, dependencies and install for a new node', () => {
        const result = ensureCppNodeRegisteredInCmake(BASE_CMAKE, 'my_node', 'src/my_node.cpp', ['std_msgs']);
        expect(result).toContain('add_executable(my_node src/my_node.cpp)');
        expect(result).toContain('ament_target_dependencies(my_node');
        expect(result).toContain('rclcpp');
        expect(result).toContain('std_msgs');
        expect(result).toMatch(/install\(TARGETS[\s\S]*my_node/);
        // The block must be inserted before ament_package().
        expect(result.indexOf('add_executable(my_node')).toBeLessThan(result.indexOf('ament_package()'));
    });
});

describe('cmakeEditor/ensureLaunchDirectoryInstalledInCmake', () => {
    it('installs the launch directory', () => {
        const result = ensureLaunchDirectoryInstalledInCmake(BASE_CMAKE);
        expect(result).toContain('DIRECTORY launch');
        expect(result).toContain('DESTINATION share/${PROJECT_NAME}');
    });

    it('is idempotent', () => {
        const once = ensureLaunchDirectoryInstalledInCmake(BASE_CMAKE);
        const twice = ensureLaunchDirectoryInstalledInCmake(once);
        expect(twice).toBe(once);
    });
});

describe('cmakeEditor/removeTargetFromCmake', () => {
    it('removes a target and prunes it from a shared install(TARGETS) block', () => {
        const cmake = [
            'add_executable(my_node src/my_node.cpp)',
            'ament_target_dependencies(my_node rclcpp)',
            'install(TARGETS',
            '  my_node',
            '  other_node',
            '  DESTINATION lib/${PROJECT_NAME}',
            ')',
        ].join('\n');
        const result = removeTargetFromCmake(cmake, 'my_node');
        expect(result).not.toContain('my_node');
        expect(result).toContain('other_node');
        expect(result).toMatch(/install\(TARGETS[\s\S]*other_node/);
    });
});
