import * as fs from 'fs';
import * as path from 'path';

/**
 * Resolved dependency information for a workspace package.
 */
export interface PackageDependency {
    name: string;
    packagePath: string;
    buildType: string;
    /** Direct workspace-local dependency names (from package.xml). */
    localDeps: string[];
    /** Implicit deps discovered by scanning launch files for cross-package refs. */
    launchFileDeps: string[];
}

/**
 * Maps package names → their resolved info.
 * Only packages that live inside the workspace `src/` directory are included.
 */
export type DependencyGraph = Map<string, PackageDependency>;

/**
 * Directories whose changes affect the public interface / shared
 * resources of a package and therefore require dependents to rebuild.
 */
const INTERFACE_PATTERNS: ReadonlySet<string> = new Set([
    'msg',
    'srv',
    'action',
    'include',
    // Resource directories commonly shared across packages
    'urdf',
    'meshes',
    'config',
    'params',
    'rviz',
    'worlds',
    'models',
    'launch',
]);

const INTERFACE_FILES: ReadonlySet<string> = new Set([
    'CMakeLists.txt',
    'package.xml',
    'setup.py',
    'setup.cfg',
]);

/** File extensions that count as "interface / resource" changes. */
const INTERFACE_EXTENSIONS: ReadonlySet<string> = new Set([
    '.msg',
    '.srv',
    '.action',
    '.urdf',
    '.xacro',
    '.yaml',
    '.yml',
    '.rviz',
    '.sdf',
    '.world',
    '.dae',
    '.stl',
]);

/**
 * Parses `package.xml` files in the workspace and builds a directed
 * dependency graph restricted to workspace-local packages.
 *
 * All filesystem access is injectable for testing.
 */
export class DependencyResolver {
    constructor(
        private readonly _readFile: (p: string) => string = (p) =>
            fs.readFileSync(p, 'utf8'),
        private readonly _readDir: (d: string) => fs.Dirent[] = (d) =>
            fs.readdirSync(d, { withFileTypes: true }),
        private readonly _exists: (p: string) => boolean = (p) =>
            fs.existsSync(p),
        private readonly _stat: (p: string) => fs.Stats = (p) =>
            fs.statSync(p),
    ) {}

    // ── Public API ─────────────────────────────────────────────

    /**
     * Scan `srcDir` for `package.xml` files and return the full
     * workspace dependency graph (only workspace-local deps are linked).
     */
    buildGraph(srcDir: string): DependencyGraph {
        const graph: DependencyGraph = new Map();
        const xmlPaths = this._findPackageXmlFiles(srcDir, 4);

        // First pass: parse every package.xml
        for (const xmlPath of xmlPaths) {
            const info = this._parsePackageXml(xmlPath);
            if (info && !graph.has(info.name)) {
                graph.set(info.name, info);
            }
        }

        // Second pass: scan launch files for cross-package references
        for (const pkg of graph.values()) {
            const launchRefs = this._scanLaunchFileDeps(pkg.packagePath);
            // Keep only refs that point to other workspace-local packages
            pkg.launchFileDeps = launchRefs.filter(
                (ref) => graph.has(ref) && ref !== pkg.name,
            );
        }

        // Third pass: merge all dep sources and filter to workspace-local
        for (const pkg of graph.values()) {
            const allDeps = new Set([
                ...pkg.localDeps.filter((dep) => graph.has(dep)),
                ...pkg.launchFileDeps,
            ]);
            pkg.localDeps = Array.from(allDeps);
        }

        return graph;
    }

    /**
     * Get the transitive closure of local dependencies for a set of
     * root packages ("up-to" style).
     *
     * Returns the root packages **plus** all their transitive deps,
     * in topological (dependency-first) order.
     */
    getClosure(roots: string[], graph: DependencyGraph): string[] {
        const visited = new Set<string>();
        const order: string[] = [];

        const visit = (name: string) => {
            if (visited.has(name)) {
                return;
            }
            visited.add(name);
            const pkg = graph.get(name);
            if (pkg) {
                for (const dep of pkg.localDeps) {
                    visit(dep);
                }
            }
            order.push(name);
        };

        for (const root of roots) {
            visit(root);
        }

        return order;
    }

    /**
     * Get packages that **depend on** a given package (reverse deps).
     * Useful for propagating interface changes upward.
     */
    getDependents(packageName: string, graph: DependencyGraph): string[] {
        const dependents: string[] = [];
        for (const [name, info] of graph) {
            if (info.localDeps.includes(packageName)) {
                dependents.push(name);
            }
        }
        return dependents;
    }

    /**
     * Check whether files that changed in `packagePath` since
     * `sinceTimestamp` include interface or resource files
     * (msg, srv, action, include, urdf, config, meshes, …).
     *
     * Returns `true` when **at least one** such file changed.
     */
    hasInterfaceChanges(packagePath: string, sinceTimestamp: number): boolean {
        return this._scanForInterfaceChanges(packagePath, sinceTimestamp, 6);
    }

    // ── Internal helpers ───────────────────────────────────────

    private _parsePackageXml(xmlPath: string): PackageDependency | undefined {
        try {
            const xml = this._readFile(xmlPath);
            const nameMatch = xml.match(/<name>\s*([^<\s]+)\s*<\/name>/);
            if (!nameMatch?.[1]) {
                return undefined;
            }
            const name = nameMatch[1];
            const packagePath = path.dirname(xmlPath);

            // Build type from <build_type> (ROS 2) or infer catkin
            const buildTypeMatch = xml.match(/<build_type>\s*([^<\s]+)\s*<\/build_type>/);
            const buildType = buildTypeMatch?.[1] ?? this._inferBuildType(packagePath);

            // Dependency tags we care about
            const deps = new Set<string>();
            const depRegex = /<(?:depend|build_depend|exec_depend|build_export_depend|run_depend)>\s*([^<\s]+)\s*<\/(?:depend|build_depend|exec_depend|build_export_depend|run_depend)>/g;
            let match: RegExpExecArray | null;
            while ((match = depRegex.exec(xml)) !== null) {
                if (match[1] && match[1] !== name) {
                    deps.add(match[1]);
                }
            }

            return {
                name,
                packagePath,
                buildType,
                localDeps: Array.from(deps),
                launchFileDeps: [],
            };
        } catch {
            return undefined;
        }
    }

    private _inferBuildType(packagePath: string): string {
        if (this._exists(path.join(packagePath, 'CMakeLists.txt'))) {
            return 'ament_cmake';
        }
        if (this._exists(path.join(packagePath, 'setup.py'))) {
            return 'ament_python';
        }
        return 'catkin';
    }

    private _findPackageXmlFiles(dir: string, depth: number): string[] {
        if (depth < 0 || !this._exists(dir)) {
            return [];
        }

        const results: string[] = [];
        let entries: fs.Dirent[];

        try {
            entries = this._readDir(dir);
        } catch {
            return results;
        }

        for (const entry of entries) {
            const fullPath = path.join(dir, entry.name);
            if (entry.isDirectory()) {
                if (
                    entry.name.startsWith('.') ||
                    ['build', 'install', 'log', 'node_modules'].includes(entry.name)
                ) {
                    continue;
                }
                results.push(...this._findPackageXmlFiles(fullPath, depth - 1));
            } else if (entry.isFile() && entry.name === 'package.xml') {
                results.push(fullPath);
            }
        }

        return results;
    }

    private _scanForInterfaceChanges(
        dir: string,
        since: number,
        depth: number,
    ): boolean {
        if (depth < 0 || !this._exists(dir)) {
            return false;
        }

        let entries: fs.Dirent[];
        try {
            entries = this._readDir(dir);
        } catch {
            return false;
        }

        for (const entry of entries) {
            const fullPath = path.join(dir, entry.name);

            if (entry.isDirectory()) {
                if (entry.name.startsWith('.') ||
                    ['build', 'install', 'log', 'node_modules', '__pycache__'].includes(entry.name)) {
                    continue;
                }
                // Only recurse into interface directories
                if (INTERFACE_PATTERNS.has(entry.name)) {
                    if (this._anyFileNewerThan(fullPath, since, depth - 1)) {
                        return true;
                    }
                }
                // For 'include' at deeper levels
                if (this._scanForInterfaceChanges(fullPath, since, depth - 1)) {
                    return true;
                }
            } else if (entry.isFile()) {
                if (INTERFACE_FILES.has(entry.name) ||
                    INTERFACE_EXTENSIONS.has(path.extname(entry.name).toLowerCase())) {
                    try {
                        if (this._stat(fullPath).mtimeMs > since) {
                            return true;
                        }
                    } catch {
                        // skip
                    }
                }
            }
        }

        return false;
    }

    private _anyFileNewerThan(dir: string, since: number, depth: number): boolean {
        if (depth < 0 || !this._exists(dir)) {
            return false;
        }

        let entries: fs.Dirent[];
        try {
            entries = this._readDir(dir);
        } catch {
            return false;
        }

        for (const entry of entries) {
            const fullPath = path.join(dir, entry.name);
            if (entry.isDirectory()) {
                if (this._anyFileNewerThan(fullPath, since, depth - 1)) {
                    return true;
                }
            } else if (entry.isFile()) {
                try {
                    if (this._stat(fullPath).mtimeMs > since) {
                        return true;
                    }
                } catch {
                    // skip
                }
            }
        }

        return false;
    }

    // ── Launch-file cross-reference scanning ────────────────────

    /**
     * Scan all launch files in a package for references to other
     * packages via:
     *  - Python: `FindPackageShare('pkg')`, `get_package_share_directory('pkg')`
     *  - XML:    `$(find pkg)`, `$(find-pkg-share pkg)`
     *
     * Returns a deduplicated list of referenced package names.
     */
    private _scanLaunchFileDeps(packagePath: string): string[] {
        const launchDir = path.join(packagePath, 'launch');
        const refs = new Set<string>();

        // Also check top-level Python/XML files (some packages put launch
        // files at root or in subdirectories)
        const dirsToScan = [launchDir, packagePath];

        for (const dir of dirsToScan) {
            this._collectLaunchRefs(dir, refs, dir === packagePath ? 1 : 3);
        }

        return Array.from(refs);
    }

    private _collectLaunchRefs(
        dir: string,
        refs: Set<string>,
        depth: number,
    ): void {
        if (depth < 0 || !this._exists(dir)) {
            return;
        }

        let entries: fs.Dirent[];
        try {
            entries = this._readDir(dir);
        } catch {
            return;
        }

        const launchExts = new Set(['.py', '.xml', '.launch', '.yaml', '.yml']);

        for (const entry of entries) {
            const fullPath = path.join(dir, entry.name);
            if (entry.isDirectory()) {
                if (
                    entry.name.startsWith('.') ||
                    ['build', 'install', 'log', 'node_modules', '__pycache__'].includes(entry.name)
                ) {
                    continue;
                }
                // Only recurse into 'launch' sub-dirs (not all of src/)
                if (entry.name === 'launch' || depth > 1) {
                    this._collectLaunchRefs(fullPath, refs, depth - 1);
                }
            } else if (entry.isFile() && launchExts.has(path.extname(entry.name).toLowerCase())) {
                try {
                    const content = this._readFile(fullPath);
                    this._extractPackageRefs(content, refs);
                } catch {
                    // skip unreadable
                }
            }
        }
    }

    /**
     * Extract package names from common ROS launch-file patterns.
     */
    private _extractPackageRefs(content: string, refs: Set<string>): void {
        // Python: FindPackageShare('pkg') or FindPackageShare("pkg")
        const findPkgShare = /FindPackageShare\(\s*['"]([a-zA-Z_][a-zA-Z0-9_]*)['"]/g;
        let m: RegExpExecArray | null;
        while ((m = findPkgShare.exec(content)) !== null) {
            refs.add(m[1]);
        }

        // Python: get_package_share_directory('pkg')
        const getPkgShare = /get_package_share_directory\(\s*['"]([a-zA-Z_][a-zA-Z0-9_]*)['"]/g;
        while ((m = getPkgShare.exec(content)) !== null) {
            refs.add(m[1]);
        }

        // Python: get_package_share_path('pkg')
        const getPkgPath = /get_package_share_path\(\s*['"]([a-zA-Z_][a-zA-Z0-9_]*)['"]/g;
        while ((m = getPkgPath.exec(content)) !== null) {
            refs.add(m[1]);
        }

        // Python: PackageShareDirectory(package='pkg') or positional
        const pkgShareDir = /PackageShareDirectory\(\s*(?:package\s*=\s*)?['"]([a-zA-Z_][a-zA-Z0-9_]*)['"]/g;
        while ((m = pkgShareDir.exec(content)) !== null) {
            refs.add(m[1]);
        }

        // XML ROS 1: $(find pkg)
        const xmlFind = /\$\(find\s+([a-zA-Z_][a-zA-Z0-9_]*)\)/g;
        while ((m = xmlFind.exec(content)) !== null) {
            refs.add(m[1]);
        }

        // XML ROS 2: $(find-pkg-share pkg)
        const xmlFindPkgShare = /\$\(find-pkg-share\s+([a-zA-Z_][a-zA-Z0-9_]*)\)/g;
        while ((m = xmlFindPkgShare.exec(content)) !== null) {
            refs.add(m[1]);
        }
    }
}
