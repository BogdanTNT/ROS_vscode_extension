import * as fs from 'fs';
import * as path from 'path';

/**
 * Persisted record for a single package's last successful build.
 */
export interface BuildStamp {
    /** Unix-ms timestamp of the last successful build. */
    lastSuccessfulBuild: number;
    /** Build type at time of last build (ament_cmake, ament_python, catkin). */
    buildType: string;
}

/**
 * Full map stored in workspace state.
 * Keys are package names, values are their build stamps.
 */
export type BuildStampMap = Record<string, BuildStamp>;

/**
 * Abstraction over the persistence layer so the manager
 * is easily testable without a real VS Code context.
 */
export interface StampStorage {
    get(key: string): BuildStampMap | undefined;
    update(key: string, value: BuildStampMap): void;
}

/** Directories that should never be scanned for source changes. */
const IGNORED_DIRS = new Set([
    'build',
    'install',
    'log',
    'node_modules',
    '__pycache__',
    '.git',
    '.vscode',
]);

const STORAGE_KEY = 'rosDevToolkit.buildStamps';

/**
 * Manages per-package build stamps and source-mtime scanning.
 *
 * Stamps are stored in the injected `StampStorage` (typically
 * `context.workspaceState`). The filesystem is accessed through
 * the injected `readDir` / `stat` helpers so tests can supply
 * in-memory fakes if desired.
 */
export class BuildStampManager {
    private _stamps: BuildStampMap;

    constructor(
        private readonly _storage: StampStorage,
        private readonly _readDir: (dir: string) => fs.Dirent[] = (d) =>
            fs.readdirSync(d, { withFileTypes: true }),
        private readonly _stat: (p: string) => fs.Stats = (p) => fs.statSync(p),
        private readonly _exists: (p: string) => boolean = (p) => fs.existsSync(p),
    ) {
        this._stamps = this._storage.get(STORAGE_KEY) ?? {};
    }

    // ── Queries ────────────────────────────────────────────────

    /** Return the stamp for a package, or `undefined` if never built. */
    getStamp(packageName: string): BuildStamp | undefined {
        return this._stamps[packageName];
    }

    /** Return a shallow copy of all stamps. */
    getAllStamps(): BuildStampMap {
        return { ...this._stamps };
    }

    /**
     * Check whether `packagePath` has source files newer than its last
     * successful build. Returns `true` if a rebuild is needed.
     *
     * If no stamp exists the package always needs a build.
     */
    needsBuild(packageName: string, packagePath: string): boolean {
        const stamp = this._stamps[packageName];
        if (!stamp) {
            return true;
        }
        const newest = this.getNewestSourceMtime(packagePath);
        return newest > stamp.lastSuccessfulBuild;
    }

    /**
     * Walk `packagePath` recursively and return the most-recent mtime
     * (in Unix-ms) across all source files, excluding build artefacts.
     *
     * Returns `0` when the directory is empty or unreadable.
     */
    getNewestSourceMtime(packagePath: string): number {
        return this._walkMtime(packagePath, 10);
    }

    // ── Mutations ──────────────────────────────────────────────

    /** Record a successful build at the current time. */
    markBuilt(packageName: string, buildType: string, timestamp?: number): void {
        this._stamps[packageName] = {
            lastSuccessfulBuild: timestamp ?? Date.now(),
            buildType,
        };
        this._persist();
    }

    /** Remove the stamp for a package (forces a rebuild on next check). */
    invalidate(packageName: string): void {
        delete this._stamps[packageName];
        this._persist();
    }

    /** Remove all stamps (e.g. after a "clean" command). */
    invalidateAll(): void {
        this._stamps = {};
        this._persist();
    }

    // ── Internals ──────────────────────────────────────────────

    private _walkMtime(dir: string, depth: number): number {
        if (depth < 0 || !this._exists(dir)) {
            return 0;
        }

        let newest = 0;
        let entries: fs.Dirent[];

        try {
            entries = this._readDir(dir);
        } catch {
            return 0;
        }

        for (const entry of entries) {
            if (entry.name.startsWith('.') && IGNORED_DIRS.has(entry.name)) {
                continue;
            }

            const fullPath = path.join(dir, entry.name);

            if (entry.isDirectory()) {
                if (IGNORED_DIRS.has(entry.name) || entry.name.endsWith('.egg-info')) {
                    continue;
                }
                const childMax = this._walkMtime(fullPath, depth - 1);
                if (childMax > newest) {
                    newest = childMax;
                }
            } else if (entry.isFile()) {
                try {
                    const mtime = this._stat(fullPath).mtimeMs;
                    if (mtime > newest) {
                        newest = mtime;
                    }
                } catch {
                    // unreadable file – skip
                }
            }
        }

        return newest;
    }

    private _persist(): void {
        this._storage.update(STORAGE_KEY, this._stamps);
    }
}
