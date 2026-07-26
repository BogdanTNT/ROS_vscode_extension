/**
 * Pure parsers that extract `DeclareLaunchArgument` / `<arg>` declarations from
 * ROS 2 Python and XML launch files.
 *
 * Extracted from {@link RosWorkspace}: the class still reads the file from disk
 * and dispatches by extension, then hands the raw text to these side-effect-free
 * parsers so they can be unit-tested directly.
 */
import type { LaunchArgOption } from '../rosWorkspace';

/** Remove duplicate launch arguments, keeping the first occurrence of each name. */
export function deduplicateArgs(args: LaunchArgOption[]): LaunchArgOption[] {
    const seen = new Set<string>();
    const results: LaunchArgOption[] = [];
    for (const arg of args) {
        if (seen.has(arg.name)) {
            continue;
        }
        seen.add(arg.name);
        results.push(arg);
    }
    return results;
}

/** Parse `DeclareLaunchArgument('name', default_value=...)` calls from Python launch files. */
export function parsePythonLaunchArgs(content: string): LaunchArgOption[] {
    const results: LaunchArgOption[] = [];
    const regex = /DeclareLaunchArgument\(\s*['"]([^'"]+)['"][^)]*\)/g;

    let match: RegExpExecArray | null;
    while ((match = regex.exec(content)) !== null) {
        const block = match[0];
        const name = match[1];

        const defaultMatch = block.match(/default_value\s*=\s*([^,)]+)/);
        const defaultValue = defaultMatch
            ? defaultMatch[1].trim().replace(/^['"]|['"]$/g, '')
            : undefined;

        results.push({ name, defaultValue });
    }

    return deduplicateArgs(results);
}

/** Parse `<arg name="..." default="...">` declarations from XML/`.launch` files. */
export function parseXmlLaunchArgs(content: string): LaunchArgOption[] {
    const results: LaunchArgOption[] = [];
    const regex = /<arg\s+[^>]*name=["']([^"']+)["'][^>]*>/g;

    let match: RegExpExecArray | null;
    while ((match = regex.exec(content)) !== null) {
        const tag = match[0];
        const name = match[1];
        const defaultMatch = tag.match(/default=["']([^"']+)["']/);
        const defaultValue = defaultMatch ? defaultMatch[1] : undefined;

        results.push({ name, defaultValue });
    }

    return deduplicateArgs(results);
}
