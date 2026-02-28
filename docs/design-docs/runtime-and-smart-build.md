# Runtime and Smart-Build Design

## Scope
This document defines the current smart-build behavior used before run/launch actions.
Source files:
- `src/ros/buildStampManager.ts`
- `src/ros/dependencyResolver.ts`
- `src/ros/buildPolicy.ts`
- `src/ros/rosWorkspace.ts`

## Core Components

### BuildStampManager
- Persistence key: `rosDevToolkit.buildStamps` (workspace state).
- Stores per-package stamp:
  - `lastSuccessfulBuild`
  - `buildType`
- Rebuild decision primitive:
  - `needsBuild(packageName, packagePath)` compares latest source mtime to stamp.
- Ignores non-source directories while scanning:
  - `build`, `install`, `log`, `node_modules`, `__pycache__`, `.git`, `.vscode`, `*.egg-info`.

### DependencyResolver
- Builds workspace-local dependency graph from `package.xml`.
- Includes:
  - explicit dependency tags (`depend`, `build_depend`, `exec_depend`, etc.)
  - launch-file inferred cross-package references.
- Produces topological closure for target package sets.
- Interface-change detector exists (`hasInterfaceChanges`) for msg/srv/action/include/resource files.

### BuildPolicy
- Main API: `evaluateBuildNeeds(srcDir, targetPackages, config?)`.
- Current effective behavior:
  1. Build graph and compute transitive local dependency closure (dependency-first order).
  2. Evaluate each package independently:
     - no stamp -> `never-built`
     - newer source than stamp -> `source-changed`
     - else `up-to-date`
  3. Do not propagate rebuild to dependents (intentional current behavior).
  4. Apply symlink shortcut:
     - `ament_python` + `source-changed` + no interface/resource changes since last stamp -> `symlink-skip`
  5. Return:
     - `packagesNeedingBuild`
     - `upToDate`
     - `details` map

## Runtime Integration

### Before run/launch
- `RosWorkspace.preLaunchBuildCheck(pkg)`:
  - reads `rosDevToolkit.preLaunchBuildCheck` (default true).
  - if disabled -> `{ action: 'launch' }`
  - else calls `evaluateBuildNeeds([pkg])`
  - stale packages -> `{ action: 'build-and-launch', stalePackages }`

### Build and execute
- `RosWorkspace.buildThenRun(...)` composes:
  1. ROS source command
  2. workspace `cd`
  3. clean stale `build/<pkg>` and `install/<pkg>` (or full dirs)
  4. `colcon build [--symlink-install] [--packages-select ...]`
  5. source workspace overlay
  6. run/launch command
- Stamps are updated immediately after dispatching build command so subsequent checks do not loop-trigger.
- Windows+WSL missing-package fallback:
  - run/launch commands first probe `ros2 pkg prefix <pkg>`.
  - if not found in sourced environment, fallback build uses:
    - `colcon build [--symlink-install] --packages-up-to <pkg>`
  - This includes workspace-local dependencies so first-run launches do not fail when only the target was selected.

### Node Visualizer Graph Snapshot Runtime
- `RosWorkspace.getGraphSnapshot(scope)` runs scoped graph-list commands in one composite shell execution:
  - nodes, topics, services, actions, parameters
- Scoped list commands run in parallel within that shell execution and are merged back into marker-framed sections, reducing end-to-end refresh latency for broad scopes.
- Each category is framed with begin/status/end markers so per-category failures are detected explicitly.
- On Windows+WSL:
  - default path routes snapshot commands through a persistent WSL bash runner with idle TTL shutdown.
  - set `rosDevToolkit.nodeVisualizerUsePersistentWslShell=false` to force one-shot `wsl.exe ... bash -lc`.

## Current Intentional Behavior
- Dependents are not rebuilt automatically when only dependencies changed.
- This favors faster iteration with `--symlink-install`, especially for Python packages.

## Implications and Risks
- Fast local iteration is improved.
- There is a potential false-negative window for environments that still require conservative dependent rebuilds.
- `BuildPolicyConfig.policy` and `fastDependencyMode` are currently non-effective in practice because dependency propagation path is not used.

## Invariants
- Never-built package is always built at least once.
- `ament_python` never-build shortcut is not allowed.
- Result order is dependency-first.
- Source scan excludes common build artifact folders.

## Regression Checklist
- Unit:
  - `tests/unit/rosWorkspace/buildStampManager.spec.ts`
  - `tests/unit/rosWorkspace/dependencyResolver.spec.ts`
  - `tests/unit/rosWorkspace/buildPolicy.spec.ts`
- Integration:
  - `tests/integration/extensionHost/packageManagerUserFlow.spec.ts`
  - `tests/integration/extensionHost/runtimeMatrix.spec.ts`
