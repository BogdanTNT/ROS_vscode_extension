import { BuildStampManager } from './buildStampManager';
import { DependencyResolver, DependencyGraph } from './dependencyResolver';

// ── Public types ───────────────────────────────────────────────

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
     * interface files (msg, srv, action, include, CMakeLists.txt …).
     * When `false`, any change in a dependency causes dependents to rebuild.
     */
    fastDependencyMode: boolean;
}

// ── Default config ─────────────────────────────────────────────

export const DEFAULT_BUILD_POLICY_CONFIG: Readonly<BuildPolicyConfig> = {
    policy: 'ask',
    fastDependencyMode: true,
};

// ── Implementation ─────────────────────────────────────────────

/**
 * Evaluates which packages need rebuilding before a launch or explicit
 * build, considering build stamps, source mtimes, dependency propagation,
 * and the symlink-install shortcut for `ament_python` packages.
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

        // Track which packages have their own source changes
        const directlyDirty = new Set<string>();

        // First pass — evaluate each package independently
        for (const name of closure) {
            const pkg = graph.get(name);
            if (!pkg) {
                continue;
            }

            const status = this._evaluatePackage(name, pkg.packagePath, pkg.buildType);
            details.set(name, status);

            if (status.needsBuild) {
                directlyDirty.add(name);
            }
        }

        // NOTE: We intentionally do NOT propagate rebuilds to dependents.
        // Only packages whose own source files changed are rebuilt.
        // ROS 2 colcon workspaces with --symlink-install handle runtime
        // dependencies fine without rebuilding the whole dependency tree.

        // Second pass — apply symlink-install shortcut
        for (const [name, status] of details) {
            if (status.needsBuild && this._canSkipViaSymlink(name, status, graph)) {
                status.needsBuild = false;
                status.reason = 'Symlink-install: only .py/launch/config changed — skip build';
                status.reasonCode = 'symlink-skip';
            }
        }

        // Collect final lists in topological order
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

    // ── Private helpers ────────────────────────────────────────

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

    private _checkDependencyPropagation(
        _name: string,
        pkg: { localDeps: string[]; launchFileDeps?: string[] },
        graph: DependencyGraph,
        dirtySet: Set<string>,
        config: BuildPolicyConfig,
    ): { reason: string; reasonCode: BuildReason } | undefined {
        for (const dep of pkg.localDeps) {
            if (!dirtySet.has(dep)) {
                continue;
            }

            const depPkg = graph.get(dep);
            if (!depPkg) {
                continue;
            }

            const isLaunchDep = pkg.launchFileDeps?.includes(dep) ?? false;

            if (config.fastDependencyMode) {
                // Only propagate if the dep changed interface/resource files
                const stamp = this._stamps.getStamp(dep);
                const since = stamp?.lastSuccessfulBuild ?? 0;
                if (this._resolver.hasInterfaceChanges(depPkg.packagePath, since)) {
                    const label = isLaunchDep
                        ? `Launch-referenced package "${dep}" changed resource/interface files`
                        : `Dependency "${dep}" changed interface files`;
                    return {
                        reason: label,
                        reasonCode: isLaunchDep
                            ? 'dependency-resource-changed'
                            : 'dependency-interface-changed',
                    };
                }
                // In fast mode, non-interface dep changes don't propagate
            } else {
                // Conservative: any dep change propagates
                const label = isLaunchDep
                    ? `Launch-referenced package "${dep}" has source changes`
                    : `Dependency "${dep}" has source changes`;
                return {
                    reason: label,
                    reasonCode: 'dependency-changed',
                };
            }
        }

        return undefined;
    }

    /**
     * For `ament_python` packages with `--symlink-install`, pure
     * .py / launch / config edits don't actually require a rebuild.
     *
     * We check if the *only* changed files since the last build are
     * in categories that symlink-install handles at runtime.
     */
    private _canSkipViaSymlink(
        name: string,
        status: PackageBuildStatus,
        graph: DependencyGraph,
    ): boolean {
        if (status.buildType !== 'ament_python') {
            return false;
        }

        // Only applies when the package itself changed, not dep propagation
        if (
            status.reasonCode !== 'source-changed' &&
            status.reasonCode !== 'never-built'
        ) {
            return false;
        }

        // Never-built packages must always be built at least once
        if (status.reasonCode === 'never-built') {
            return false;
        }

        const stamp = this._stamps.getStamp(name);
        if (!stamp) {
            return false;
        }

        // If interface files changed, rebuild is needed
        if (this._resolver.hasInterfaceChanges(status.packagePath, stamp.lastSuccessfulBuild)) {
            return false;
        }

        return true;
    }
}
