/**
 * Pure string transforms for editing `CMakeLists.txt` when scaffolding or
 * removing ROS 2 (ament_cmake) nodes and launch directories.
 *
 * Extracted from {@link RosWorkspace}. Every function takes the current CMake
 * text and returns the updated text without touching the filesystem.
 */
import { escapeRegex } from '../utils/strings';

/** Ensure `find_package(<dependency> REQUIRED)` is present. */
export function ensureFindPackageDependency(cmake: string, dependency: string): string {
    const escapedDep = escapeRegex(dependency);
    if (new RegExp(`find_package\\s*\\(\\s*${escapedDep}\\b`, 'i').test(cmake)) {
        return cmake;
    }

    const findAmentPattern = /find_package\s*\(\s*ament_cmake\s+REQUIRED\s*\)\s*\n?/i;
    if (findAmentPattern.test(cmake)) {
        return cmake.replace(
            findAmentPattern,
            (match) => `${match}find_package(${dependency} REQUIRED)\n`,
        );
    }

    const projectPattern = /project\s*\([^)]*\)\s*\n?/i;
    if (projectPattern.test(cmake)) {
        return cmake.replace(
            projectPattern,
            (match) => `${match}find_package(${dependency} REQUIRED)\n`,
        );
    }

    return `find_package(${dependency} REQUIRED)\n${cmake}`;
}

/** Ensure `ament_target_dependencies(<target> ... <dependency>)` includes the dependency. */
export function ensureTargetDependency(cmake: string, targetName: string, dependency: string): string {
    const escapedTarget = escapeRegex(targetName);
    const targetDepPattern = new RegExp(
        `ament_target_dependencies\\s*\\(\\s*${escapedTarget}[\\s\\S]*?\\)`,
        'm',
    );
    const existingCall = cmake.match(targetDepPattern)?.[0];
    if (existingCall) {
        const hasDependency = new RegExp(`\\b${escapeRegex(dependency)}\\b`, 'm').test(existingCall);
        if (hasDependency) {
            return cmake;
        }
        return cmake.replace(
            targetDepPattern,
            existingCall.replace(/\)\s*$/, ` ${dependency})`),
        );
    }

    const executablePattern = new RegExp(
        `add_executable\\s*\\(\\s*${escapedTarget}[\\s\\S]*?\\)`,
        'm',
    );
    if (executablePattern.test(cmake)) {
        return cmake.replace(
            executablePattern,
            (match) => `${match}\nament_target_dependencies(${targetName} ${dependency})`,
        );
    }

    return insertBeforeAmentPackage(
        cmake,
        `ament_target_dependencies(${targetName} ${dependency})`,
    );
}

/** Ensure the target is present in an `install(TARGETS ...)` block. */
export function ensureInstallTarget(cmake: string, targetName: string): string {
    const escapedTarget = escapeRegex(targetName);
    const installTargetPattern = new RegExp(
        `install\\s*\\(\\s*TARGETS[\\s\\S]*?\\b${escapedTarget}\\b[\\s\\S]*?\\)`,
        'm',
    );
    if (installTargetPattern.test(cmake)) {
        return cmake;
    }

    const installBlock = [
        'install(TARGETS',
        `  ${targetName}`,
        '  DESTINATION lib/${PROJECT_NAME}',
        ')',
    ].join('\n');
    return insertBeforeAmentPackage(cmake, installBlock);
}

/** Insert a block just before `ament_package()`, or append it if absent. */
export function insertBeforeAmentPackage(cmake: string, block: string): string {
    const normalizedBlock = block.trim();
    const amentPackagePattern = /^\s*ament_package\s*\(\s*\)\s*$/m;
    if (amentPackagePattern.test(cmake)) {
        return cmake.replace(
            amentPackagePattern,
            `${normalizedBlock}\n\nament_package()`,
        );
    }

    const trailingNewline = cmake.endsWith('\n') ? '' : '\n';
    return `${cmake}${trailingNewline}\n${normalizedBlock}\n`;
}

/**
 * Register a freshly created C++ node in CMake: find_package, add_executable,
 * ament_target_dependencies and install(TARGETS ...).
 */
export function ensureCppNodeRegisteredInCmake(
    cmake: string,
    nodeName: string,
    relativeSourcePath: string,
    extraDependencies: string[] = [],
): string {
    let updated = cmake;
    const dependencies = Array.from(new Set(['rclcpp', ...extraDependencies]));
    dependencies.forEach((dependency) => {
        updated = ensureFindPackageDependency(updated, dependency);
    });

    const escapedNode = escapeRegex(nodeName);
    const hasExecutable = new RegExp(
        `add_executable\\s*\\(\\s*${escapedNode}\\b`,
        'm',
    ).test(updated);
    if (!hasExecutable) {
        const block = [
            `add_executable(${nodeName} ${relativeSourcePath})`,
            `ament_target_dependencies(${nodeName} ${dependencies.join(' ')})`,
            'install(TARGETS',
            `  ${nodeName}`,
            '  DESTINATION lib/${PROJECT_NAME}',
            ')',
        ].join('\n');
        updated = insertBeforeAmentPackage(updated, block);
    }

    dependencies.forEach((dependency) => {
        updated = ensureTargetDependency(updated, nodeName, dependency);
    });
    updated = ensureInstallTarget(updated, nodeName);
    return updated;
}

/** Ensure the `launch/` directory is installed to `share/${PROJECT_NAME}`. */
export function ensureLaunchDirectoryInstalledInCmake(cmake: string): string {
    const hasLaunchInstall = /install\s*\(\s*DIRECTORY\s+launch\b[\s\S]*?DESTINATION\s+share\s*\/\s*\$\{PROJECT_NAME\}[\s\S]*?\)/im.test(cmake);
    if (hasLaunchInstall) {
        return cmake;
    }
    const installBlock = [
        'install(',
        '  DIRECTORY launch',
        '  DESTINATION share/${PROJECT_NAME}',
        ')',
    ].join('\n');
    return insertBeforeAmentPackage(cmake, installBlock);
}

/** Remove all CMake commands/install entries that reference a target. */
export function removeTargetFromCmake(cmake: string, targetName: string): string {
    let updated = cmake;
    const targetAwareCommands = [
        'add_executable',
        'ament_target_dependencies',
        'target_link_libraries',
        'target_include_directories',
        'target_compile_definitions',
        'target_compile_options',
        'target_compile_features',
        'set_target_properties',
    ];

    targetAwareCommands.forEach((commandName) => {
        updated = removeCmakeCommandBlockForTarget(updated, commandName, targetName);
    });
    updated = removeTargetFromInstallTargetsBlocks(updated, targetName);
    updated = updated.replace(/\n{3,}/g, '\n\n');

    return updated;
}

function removeCmakeCommandBlockForTarget(cmake: string, commandName: string, targetName: string): string {
    const escapedCommand = escapeRegex(commandName);
    const escapedTarget = escapeRegex(targetName);
    const commandPattern = new RegExp(
        `^[ \\t]*${escapedCommand}\\s*\\(\\s*${escapedTarget}\\b[\\s\\S]*?\\)\\s*\\n?`,
        'gmi',
    );
    return cmake.replace(commandPattern, '');
}

function removeTargetFromInstallTargetsBlocks(cmake: string, targetName: string): string {
    const installTargetsPattern = /install\s*\(\s*TARGETS[\s\S]*?\)/gim;
    return cmake.replace(
        installTargetsPattern,
        (block: string) => removeTargetFromSingleInstallTargetsBlock(block, targetName),
    );
}

function removeTargetFromSingleInstallTargetsBlock(block: string, targetName: string): string {
    const openParen = block.indexOf('(');
    const closeParen = block.lastIndexOf(')');
    if (openParen < 0 || closeParen <= openParen) {
        return block;
    }

    const inner = block.slice(openParen + 1, closeParen).trim();
    if (!inner) {
        return block;
    }
    const tokens = inner.split(/\s+/).filter(Boolean);
    if (tokens.length < 2 || tokens[0].toUpperCase() !== 'TARGETS') {
        return block;
    }

    const installKeywords = new Set([
        'ARCHIVE',
        'BUNDLE',
        'COMPONENT',
        'CONFIGURATIONS',
        'DESTINATION',
        'EXCLUDE_FROM_ALL',
        'EXPORT',
        'FRAMEWORK',
        'INCLUDES',
        'LIBRARY',
        'NAMELINK_COMPONENT',
        'NAMELINK_ONLY',
        'NAMELINK_SKIP',
        'OPTIONAL',
        'PERMISSIONS',
        'PRIVATE_HEADER',
        'PUBLIC_HEADER',
        'RESOURCE',
        'RUNTIME',
    ]);

    let targetEnd = tokens.length;
    for (let idx = 1; idx < tokens.length; idx += 1) {
        if (installKeywords.has(tokens[idx].toUpperCase())) {
            targetEnd = idx;
            break;
        }
    }

    const declaredTargets = tokens.slice(1, targetEnd);
    const remainingTargets = declaredTargets.filter((token) => token !== targetName);
    if (remainingTargets.length === declaredTargets.length) {
        return block;
    }
    if (remainingTargets.length === 0) {
        return '';
    }

    const trailingTokens = tokens.slice(targetEnd);
    const rebuiltInner = ['TARGETS', ...remainingTargets, ...trailingTokens].join(' ');
    return `${block.slice(0, openParen + 1)}${rebuiltInner}${block.slice(closeParen)}`;
}
