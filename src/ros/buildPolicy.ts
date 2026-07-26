import * as path from 'path';
import { BuildStampManager } from './buildStampManager';
import { DependencyResolver } from './dependencyResolver';

/** User-selectable smart-build behaviour. */
export type SmartBuildPolicy = 'always' | 'ask' | 'never';

/** Why a particular package needs a rebuild. */
export type BuildReason =
    | 'never-built'
    | 'source-changed'
    | 'dependency-interface-changed'
    | 'dependency-resource-changed'
    | 'dependency-changed';

/** Per-package evaluation result. */
export interface PackageBuildStatus {
    name: string;
    packagePath: string;
    buildType: string;
    needsBuild: boolean;
    /** Human-readable explanation shown in the UI. */
    reason: string;
    /** Machine-readable tag for filtering / sorting. */
    reasonCode: BuildReason | 'up-to-date' | 'symlink-skip';
}

/** Aggregate result returned by `evaluateBuildNeeds`. */
export interface BuildEvaluation {
    /** Packages that require a build, in dependency-first order. */
    packagesNeedingBuild: string[];
    /** Packages that can skip the build. */
    upToDate: string[];
    /** Per-package details. */
    details: Map<string, PackageBuildStatus>;
}

/** Configuration read from the user settings. */
export interface BuildPolicyConfig {
    /** `always` | `ask` | `never` */
    policy: SmartBuildPolicy;
    /**
     * When `true`, propagate rebuild only when a dependency changed
     * interface files (msg, srv, action, include, CMakeLists.txt, etc.).
     * When `false`, any change in a dependency causes dependents to rebuild.
     */
    fastDependencyMode: boolean;
    /** Reflects whether builds currently use ROS 2 `--symlink-install`. */
    symlinkInstall?: boolean;
}

export const DEFAULT_BUILD_POLICY_CONFIG: Readonly<BuildPolicyConfig> = {
    policy: 'ask',
    fastDependencyMode: true,
    symlinkInstall: true,
};

/**
 * Evaluates which packages need rebuilding before a launch or explicit
 * build, considering build stamps, source mtimes, dependency propagation,
 * and symlink-install shortcuts for runtime-served files.
 */
export class BuildPolicy {
    constructor(
        private readonly _stamps: BuildStampManager,
        private readonly _resolver: DependencyResolver,
    ) {}

    /**
     * Given a workspace `srcDir` and a set of target packages,
     * determine which packages (including transitive local deps)
     * need rebuilding and why.
     */
    evaluateBuildNeeds(
        srcDir: string,
        targetPackages: string[],
        config: BuildPolicyConfig = DEFAULT_BUILD_POLICY_CONFIG,
    ): BuildEvaluation {
        const graph = this._resolver.buildGraph(srcDir);
        const closure = this._resolver.getClosure(targetPackages, graph);

        const details = new Map<string, PackageBuildStatus>();
        const needsBuild: string[] = [];
        const upToDate: string[] = [];

        for (const name of closure) {
            const pkg = graph.get(name);
            if (!pkg) {
                continue;
            }

            const status = this._evaluatePackage(name, pkg.packagePath, pkg.buildType);
            details.set(name, status);
        }

        // We intentionally do not propagate rebuilds to dependents. Only
        // packages whose own source files changed are rebuilt.
        for (const [name, status] of details) {
            if (status.needsBuild && this._canSkipViaSymlink(name, status, config)) {
                status.needsBuild = false;
                status.reason = 'Symlink-install: only runtime-served files changed - skip build';
                status.reasonCode = 'symlink-skip';
            }
        }

        for (const name of closure) {
            const status = details.get(name);
            if (!status) {
                continue;
            }
            if (status.needsBuild) {
                needsBuild.push(name);
            } else {
                upToDate.push(name);
            }
        }

        return { packagesNeedingBuild: needsBuild, upToDate, details };
    }

    private _evaluatePackage(
        name: string,
        packagePath: string,
        buildType: string,
    ): PackageBuildStatus {
        const stamp = this._stamps.getStamp(name);

        if (!stamp) {
            return {
                name,
                packagePath,
                buildType,
                needsBuild: true,
                reason: 'Never built',
                reasonCode: 'never-built',
            };
        }

        if (this._stamps.needsBuild(name, packagePath)) {
            return {
                name,
                packagePath,
                buildType,
                needsBuild: true,
                reason: 'Source newer than last successful build',
                reasonCode: 'source-changed',
            };
        }

        return {
            name,
            packagePath,
            buildType,
            needsBuild: false,
            reason: 'Up to date',
            reasonCode: 'up-to-date',
        };
    }

    /**
     * With `--symlink-install`, runtime-served file changes can often be
     * consumed directly from the source tree without re-running colcon.
     */
    private _canSkipViaSymlink(
        name: string,
        status: PackageBuildStatus,
        config: BuildPolicyConfig,
    ): boolean {
        if (!(config.symlinkInstall ?? DEFAULT_BUILD_POLICY_CONFIG.symlinkInstall)) {
            return false;
        }

        if (
            status.reasonCode !== 'source-changed' &&
            status.reasonCode !== 'never-built'
        ) {
            return false;
        }

        if (status.reasonCode === 'never-built') {
            return false;
        }

        const stamp = this._stamps.getStamp(name);
        if (!stamp) {
            return false;
        }

        const changedFiles = this._stamps.getChangedFilesSince(
            status.packagePath,
            stamp.lastSuccessfulBuild,
        );
        if (changedFiles.length === 0) {
            return false;
        }

        return changedFiles.every((relativePath) =>
            this._isSymlinkSafeChange(relativePath, status.buildType),
        );
    }

    private _isSymlinkSafeChange(relativePath: string, buildType: string): boolean {
        const normalizedPath = relativePath.replace(/\\/g, '/').toLowerCase();
        const extension = path.posix.extname(normalizedPath);
        const baseName = path.posix.basename(normalizedPath);

        if (extension === '.yaml' || extension === '.yml') {
            return true;
        }

        if (buildType === 'ament_python' && extension === '.py' && baseName !== 'setup.py') {
            return true;
        }

        return false;
    }
}
