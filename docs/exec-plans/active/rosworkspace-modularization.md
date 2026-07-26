# Execution Plan: RosWorkspace Modularization

## Objective
Split `src/ros/rosWorkspace.ts` into focused internal services without changing externally observable behavior.

## Current Problem
- `RosWorkspace` is a large multi-domain class (~4.8k lines) combining:
  - environment probing
  - package discovery and scaffolding
  - smart build orchestration
  - terminal process tracking
  - graph querying and publish helpers
- High coupling increases regression risk and review cost.

## Non-Goals
- No public command/setting changes.
- No behavior changes to existing run/launch, build, package, or graph flows.
- No UI redesign.

## Constraints
- Preserve current exports of `RosWorkspace` consumed by view providers.
- Keep existing tests passing before/after each phase.
- Maintain backward compatibility of deprecated wrappers for now.

## Target Internal Module Layout
- `src/ros/rosWorkspace.ts`
  - Facade only; delegates to services below.
- `src/ros/runtime/environmentService.ts`
  - Environment detection/reporting, run target normalization.
- `src/ros/runtime/terminalService.ts`
  - Terminal routing/tracking/focus/kill/preferred behavior.
- `src/ros/workspace/packageCatalogService.ts`
  - Workspace/all-package discovery and package detail extraction.
- `src/ros/workspace/packageScaffoldService.ts`
  - `createPackage` and maintainer identity resolution.
- `src/ros/workspace/nodeScaffoldService.ts`
  - Add/remove node for Python/CMake packages.
- `src/ros/runtime/buildExecutionService.ts`
  - `buildPackages`, `buildThenRun`, prelaunch build integration.
- `src/ros/runtime/graphService.ts`
  - Node/topic/service/action/parameter list queries, node info, topic roles.
- `src/ros/runtime/pubsubService.ts`
  - Topic echo subscriptions, publish template/publish result, action goal template/send.

## Progress Log
- **Phase 1 (pure helpers): in progress.** Extracted, each with direct unit tests:
  - `utils/strings.ts`, `templates/nodeTemplates.ts`,
    `build/cmakeEditor.ts`, `build/pythonSetupEditor.ts`,
    `graph/rosGraphParsing.ts`, `launch/launchArgsParsing.ts`,
    `environment/wslEnvironment.ts`, `runtime/processUtils.ts`.
  - Remaining Phase-1 item: a shared runtime-context object for the stateful
    command-execution helpers (`exec`, `buildSourcedCommand*`, run-target
    resolution) — not yet extracted because these are coupled to terminal state.
- Removed dead code surfaced along the way (`killProcessTree`,
  `pickLaunchTerminal`, `matchesIntegratedLaunchTerminalProfile`,
  `_checkDependencyPropagation`) and enabled `noUnusedLocals` to guard against
  re-accumulation.
- `rosWorkspace.ts`: ~5.3k → ~5.1k lines so far.

## Phase Plan

### Phase 1: Extract Shared Runtime Context and Utilities
- Create a shared context object for:
  - workspace path getters
  - ROS source command builders
  - command execution helpers (`exec`, `execFileSafe`, shell escaping)
- Move pure helper functions first (no stateful logic). ✅ done (see Progress Log)
- Keep facade method signatures unchanged.

### Phase 2: Extract TerminalService
- Move terminal lifecycle state and APIs:
  - track/focus/kill/preferred
  - integrated/external/WSL launch paths
  - PID file handling and close-state transitions
- Keep event emitter contract:
  - `onTerminalsChanged`

#### Phase 2 coupling analysis (blueprint)
This is the highest-risk phase. A grep of `this.*` calls out of the
terminal/run-launch cluster shows it depends on ~15 non-terminal methods, so
`TerminalService` cannot be a clean leaf — it needs a host-context interface
injected at construction. Proposed `TerminalHostContext`:

- Command building: `buildSourcedCommand`, `buildSourcedCommandForRunTarget`,
  `buildSourcedCommandForContext`, `getRosSourceCommand`,
  `getWorkspaceOverlayCommand`, `resolveRosSetupPath`.
- Run-target / WSL resolution: `normalizeRunTerminalTarget`,
  `isWindowsWslTarget`, `isWindowsWslIntegratedTarget`, `getWslDistroFromTarget`,
  `resolveInstalledWslDistro`, `getWorkspacePathForRunTarget`.
- Shell/env: `resolveBashPath`, `getTerminalEnv`, `buildCleanEnv`,
  `resolveExternalTerminal`, `buildExternalTerminalArgs`.
- Process signalling is already extracted → `runtime/processUtils.ts`.

Sequencing note: extract a `RunTargetResolver` + `CommandRunner`/context object
(the remaining Phase-1 item) **before** `TerminalService`, so the host-context
interface is small and stable rather than a grab-bag of facade methods.

**Verification for this phase is not covered by unit tests.** After extraction,
manually exercise in a running VS Code host: create/focus/kill/relaunch tracked
terminals; integrated + external launch; on Windows, WSL integrated + external
launch and Ctrl-C interrupt of both `ros2 run` and `ros2 launch`.

### Phase 3: Extract Workspace Package and Scaffolding Services
- Move package listing/detail and package/node create/remove flows.
- Keep existing validation and normalization behavior exactly.
- Preserve global-state integration behavior in view providers.

### Phase 4: Extract BuildExecutionService + Smart-Build Bridge
- Move build and prelaunch decision orchestration.
- Keep `BuildStampManager`, `DependencyResolver`, and `BuildPolicy` integration behavior unchanged.
- Preserve stamp update timing and clean-dir semantics.

### Phase 5: Extract GraphService and PubSubService
- Move graph list and info methods.
- Move topic/action template and send methods.
- Move long-running topic subscription management.

### Phase 6: Facade Cleanup and Compatibility
- `RosWorkspace` remains the public facade.
- Keep deprecated wrappers:
  - `buildThenLaunch`
  - `buildThenRunNode`
- Add internal comments documenting delegation boundaries.

## Interface Contract During Refactor
- Public `RosWorkspace` methods and exported interfaces remain stable.
- Provider files (`src/views/packageManagerView.ts`, `src/views/nodeVisualizerView.ts`) should not need call-site changes.

## Verification Strategy
- Per phase:
  - `npm run test:unit`
  - `npm run lint`
  - targeted subset for touched domain
- Gate before final merge:
  - `npm test` (full suite)
  - `npm run test:integration`
- Terminal/run-launch phases (Phase 2) additionally require **manual verification
  in a running VS Code host** — create/kill/relaunch terminals, external + WSL
  launch paths — because that subsystem has no automated integration coverage.

## Rollback Plan
- Refactor lands in small commits per phase.
- If a phase regresses behavior:
  - revert that phase only
  - keep previously extracted phases
  - preserve facade behavior.

## Exit Criteria
- `src/ros/rosWorkspace.ts` reduced to facade/orchestration role.
- No user-facing behavior diffs in package manager or node visualizer flows.
- Existing tests pass without relaxing assertions.
- `ARCHITECTURE.md` and design docs updated to reflect final module map.

