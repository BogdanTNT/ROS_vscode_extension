/**
 * Pure string transforms for editing `setup.py` / `setup.cfg` of ament_python
 * packages when scaffolding launch installs or adding/removing node entry
 * points.
 *
 * Extracted from {@link RosWorkspace}. No filesystem access — callers read the
 * file, pass its contents here, and write back the returned text.
 */
import { escapeRegex } from '../utils/strings';

/** Ensure `setup.py` installs the package's `launch/` directory via data_files. */
export function ensurePythonLaunchInstallInSetupPy(setupPy: string, packageName: string): string {
    let updated = setupPy;
    const hasFromGlobImport = /^\s*from\s+glob\s+import\s+glob\b/m.test(updated);
    if (!hasFromGlobImport) {
        updated = `from glob import glob\n${updated}`;
    }

    const escapedPkgName = escapeRegex(packageName);
    const hasLaunchDataFilesEntry = new RegExp(`['"]share/${escapedPkgName}/launch['"]`).test(updated);
    if (hasLaunchDataFilesEntry) {
        return updated;
    }

    const launchEntry = `        ('share/${packageName}/launch', glob('launch/*')),`;
    const dataFilesBlockPattern = /(data_files\s*=\s*\[)([\s\S]*?)(\]\s*,)/m;
    if (dataFilesBlockPattern.test(updated)) {
        return updated.replace(
            dataFilesBlockPattern,
            (_full, prefix: string, body: string, suffix: string) => {
                const trimmedBody = body.replace(/\s*$/, '');
                const nextBody = trimmedBody
                    ? `${trimmedBody}\n${launchEntry}\n`
                    : `\n${launchEntry}\n`;
                return `${prefix}${nextBody}${suffix}`;
            },
        );
    }

    const setupCallPattern = /setup\s*\(\s*/m;
    if (setupCallPattern.test(updated)) {
        return updated.replace(
            setupCallPattern,
            (match) => `${match}data_files=[\n${launchEntry}\n    ],\n    `,
        );
    }
    return updated;
}

/** Ensure `setup.cfg` installs the package's `launch/` directory via data_files. */
export function ensurePythonLaunchInstallInSetupCfg(setupCfg: string, packageName: string): string {
    const lineEnding = setupCfg.includes('\r\n') ? '\r\n' : '\n';
    const launchKey = `share/${packageName}/launch`;
    const launchKeyPattern = new RegExp(`^\\s*${escapeRegex(launchKey)}\\s*=`, 'm');
    if (launchKeyPattern.test(setupCfg)) {
        return setupCfg;
    }

    const lines = setupCfg.split(/\r?\n/);
    const dataFilesSectionIndex = lines.findIndex((line) => /^\s*\[options\.data_files\]\s*$/i.test(line));
    if (dataFilesSectionIndex < 0) {
        const trimmed = setupCfg.trimEnd();
        const separator = trimmed ? `${lineEnding}${lineEnding}` : '';
        return `${trimmed}${separator}[options.data_files]${lineEnding}${launchKey} =${lineEnding}    launch/*${lineEnding}`;
    }

    let insertIndex = lines.length;
    for (let idx = dataFilesSectionIndex + 1; idx < lines.length; idx += 1) {
        if (/^\s*\[.*\]\s*$/.test(lines[idx])) {
            insertIndex = idx;
            break;
        }
    }

    lines.splice(
        insertIndex,
        0,
        `${launchKey} =`,
        '    launch/*',
    );
    return lines.join(lineEnding);
}

/** Remove a node's `console_scripts` entry from `setup.py`. */
export function removeNodeFromSetupPyConsoleScripts(setupPy: string, nodeName: string): string {
    const escapedNode = escapeRegex(nodeName);
    const consoleScriptsBlockPattern = /(['"]console_scripts['"]\s*:\s*\[)([\s\S]*?)(\])/gm;
    let changed = false;

    const updated = setupPy.replace(
        consoleScriptsBlockPattern,
        (full, prefix: string, body: string, suffix: string) => {
            const updatedBody = removeNodeFromPythonConsoleScriptBody(body, escapedNode);
            if (updatedBody !== body) {
                changed = true;
            }
            return `${prefix}${updatedBody}${suffix}`;
        },
    );

    if (!changed) {
        return setupPy;
    }
    return updated.replace(/\n{3,}/g, '\n\n');
}

function removeNodeFromPythonConsoleScriptBody(body: string, escapedNodeName: string): string {
    let updatedBody = body;

    // Primary path: remove normal list entries on their own lines.
    const linePattern = new RegExp(
        `^[ \\t]*['"]\\s*${escapedNodeName}\\s*=\\s*[^'"]+['"]\\s*,?\\s*(?:#.*)?\\r?\\n?`,
        'gm',
    );
    updatedBody = updatedBody.replace(linePattern, '');

    // Fallback for one-line lists.
    if (!updatedBody.includes('\n') && !updatedBody.includes('\r')) {
        const inlinePattern = new RegExp(
            `(^|\\s*,\\s*)['"]\\s*${escapedNodeName}\\s*=\\s*[^'"]+['"]\\s*(?=\\s*,|\\s*$)`,
            'g',
        );
        updatedBody = updatedBody
            .replace(inlinePattern, '')
            .replace(/^\s*,\s*/, '')
            .replace(/\s*,\s*$/, '');
    }

    return updatedBody;
}

/** Remove a node's entry-point line from `setup.cfg`'s `[options.entry_points]`. */
export function removeNodeFromSetupCfgConsoleScripts(setupCfg: string, nodeName: string): string {
    const escapedNode = escapeRegex(nodeName);
    const lineEnding = setupCfg.includes('\r\n') ? '\r\n' : '\n';
    const lines = setupCfg.split(/\r?\n/);
    const nextLines: string[] = [];
    let inEntryPointsSection = false;
    const entryPattern = new RegExp(`^\\s*${escapedNode}\\s*=`);

    for (const line of lines) {
        if (/^\s*\[options\.entry_points\]\s*$/i.test(line)) {
            inEntryPointsSection = true;
            nextLines.push(line);
            continue;
        }
        if (inEntryPointsSection && /^\s*\[.*\]\s*$/.test(line)) {
            inEntryPointsSection = false;
        }
        if (inEntryPointsSection && entryPattern.test(line)) {
            continue;
        }
        nextLines.push(line);
    }

    return nextLines.join(lineEnding).replace(/\n{3,}/g, '\n\n');
}
