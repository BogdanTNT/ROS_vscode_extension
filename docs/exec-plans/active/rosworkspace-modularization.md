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

## Phase Plan

### Phase 1: Extract Shared Runtime Context and Utilities
- Create a shared context object for:
  - workspace path getters
  - ROS source command builders
  - command execution helpers (`exec`, `execFileSafe`, shell escaping)
- Move pure helper functions first (no stateful logic).
- Keep facade method signatures unchanged.

### Phase 2: Extract TerminalService
- Move terminal lifecycle state and APIs:
  - track/focus/kill/preferred
  - integrated/external/WSL launch paths
  - PID file handling and close-state transitions
- Keep event emitter contract:
  - `onTerminalsChanged`

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
  - targeted subset for touched domain
- Gate before final merge:
  - `npm run test:matrix`
  - `npm run test:integration`

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

