import * as fs from 'fs';
import * as path from 'path';

const SKIP_DIRS = new Set(['.git', 'node_modules', 'build', 'install', 'log', '.vscode-test']);

function isScannable(entry: fs.Dirent): boolean {
    return entry.isDirectory() && !entry.name.startsWith('.') && !SKIP_DIRS.has(entry.name);
}

function readDirSafe(dir: string): fs.Dirent[] {
    try {
        return fs.readdirSync(dir, { withFileTypes: true });
    } catch {
        return [];
    }
}

function hasPackageXmlUnder(dir: string, depth: number): boolean {
    if (depth < 0 || !fs.existsSync(dir)) {
        return false;
    }
    for (const entry of readDirSafe(dir)) {
        if (entry.isFile() && entry.name === 'package.xml') {
            return true;
        }
        if (isScannable(entry) && hasPackageXmlUnder(path.join(dir, entry.name), depth - 1)) {
            return true;
        }
    }
    return false;
}

/**
 * Finds directories under `rootDir` that look like a colcon/catkin
 * workspace root — i.e. they directly contain a `src/` folder with at
 * least one `package.xml` underneath. Used to help point the extension at
 * the right workspace when the VS Code workspace folder itself has no
 * `src/` (e.g. a monorepo where the ROS workspace lives in a subfolder).
 */
export function findNestedRosWorkspaceCandidates(rootDir: string, maxDepth = 4): string[] {
    const candidates: string[] = [];

    const walk = (dir: string, depth: number): void => {
        if (depth < 0 || !fs.existsSync(dir)) {
            return;
        }

        const srcDir = path.join(dir, 'src');
        if (fs.existsSync(srcDir) && hasPackageXmlUnder(srcDir, 3)) {
            candidates.push(dir);
            return;
        }

        for (const entry of readDirSafe(dir)) {
            if (isScannable(entry)) {
                walk(path.join(dir, entry.name), depth - 1);
            }
        }
    };

    walk(rootDir, maxDepth);
    return candidates;
}

function readPackageName(xmlPath: string): string | undefined {
    try {
        const xml = fs.readFileSync(xmlPath, 'utf8');
        return xml.match(/<name>\s*([^<\s]+)\s*<\/name>/)?.[1];
    } catch {
        return undefined;
    }
}

/**
 * Scans `srcDir` for package.xml files and groups their containing
 * directories by declared package name, returning only names that
 * resolve to more than one location. colcon refuses to build a workspace
 * with a duplicate package name until one copy is removed or ignored.
 */
export function findDuplicatePackageLocations(srcDir: string, maxDepth = 6): Map<string, string[]> {
    const byName = new Map<string, string[]>();

    const walk = (dir: string, depth: number): void => {
        for (const entry of readDirSafe(dir)) {
            const fullPath = path.join(dir, entry.name);
            if (isScannable(entry)) {
                if (depth > 0) {
                    walk(fullPath, depth - 1);
                }
                continue;
            }
            if (entry.isFile() && entry.name === 'package.xml') {
                const name = readPackageName(fullPath);
                if (!name) {
                    continue;
                }
                const packagePath = path.dirname(fullPath);
                const paths = byName.get(name) ?? [];
                if (!paths.includes(packagePath)) {
                    paths.push(packagePath);
                }
                byName.set(name, paths);
            }
        }
    };

    walk(srcDir, maxDepth);

    for (const [name, paths] of Array.from(byName.entries())) {
        if (paths.length < 2) {
            byName.delete(name);
        }
    }

    return byName;
}
