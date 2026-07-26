/**
 * Shared, stateless string helpers used across the ROS workspace modules
 * (code templates, CMake/setup.py editing, CLI-name sanitisation, etc.).
 *
 * These were previously private methods on {@link RosWorkspace}; extracting
 * them keeps that class a thin facade and lets the helpers be unit-tested and
 * reused directly.
 */

/** Escape a string so it can be embedded literally inside a `RegExp`. */
export function escapeRegex(value: string): string {
    return value.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
}

/** Convert a `snake_case`/`kebab-case` identifier into `PascalCase`. */
export function toPascalCase(name: string): string {
    return name
        .split(/[_-]/)
        .map((part) => part.charAt(0).toUpperCase() + part.slice(1))
        .join('');
}

/** Escape a value for embedding inside a Python single-quoted string literal. */
export function escapePythonSingleQuotedString(value: string): string {
    return String(value || '').replace(/\\/g, '\\\\').replace(/'/g, '\\\'');
}

/** Escape a value for embedding inside a C++ double-quoted string literal. */
export function escapeCppDoubleQuotedString(value: string): string {
    return String(value || '').replace(/\\/g, '\\\\').replace(/"/g, '\\"');
}
