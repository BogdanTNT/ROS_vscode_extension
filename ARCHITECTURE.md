# Architecture

## Overview
ROS Dev Toolkit is a VS Code extension that combines package management, run/launch orchestration, and runtime ROS graph tooling.

The architecture has 3 main layers:

1. Extension host orchestration
   - Activation, command registration, provider registration.
2. Runtime service layer
   - `RosWorkspace` + smart-build components (`BuildStampManager`, `DependencyResolver`, `BuildPolicy`).
3. Webview UI layer
   - Package Manager and Node Visualizer webviews (`media/*`) with host/webview message contracts.

## Component Map

### 1) Activation and Wiring
- File: `src/extension.ts`
- Responsibilities:
  - Creates one shared `RosWorkspace`.
  - Initializes smart-build (`initSmartBuild`).
  - Registers webview providers:
    - `rosPackageManager` -> `PackageManagerViewProvider`
    - `rosNodeVisualizer` -> `NodeVisualizerViewProvider`
  - Registers commands:
    - `rosDevToolkit.createPackage`
    - `rosDevToolkit.openSourcedTerminal`
    - `rosDevToolkit.refreshGraph`
  - Broadcasts shared UI preferences across both webviews.

### 2) Runtime Service Layer
- Facade / orchestration:
  - `src/ros/rosWorkspace.ts` — still the primary orchestrator; holds terminal
    state, run/launch flows, command-execution context, and graph orchestration.
- Smart-build subsystem:
  - `src/ros/buildStampManager.ts`
  - `src/ros/dependencyResolver.ts`
  - `src/ros/buildPolicy.ts`
- Extracted pure / stateless helper modules (side-effect-free, directly
  unit-tested — the workspace class still performs the `exec`/`fs` calls and
  hands raw output to these):
  - `src/ros/utils/strings.ts` — regex/name/quote escaping helpers.
  - `src/ros/templates/nodeTemplates.ts` — Python/C++ node source templates.
  - `src/ros/build/cmakeEditor.ts` — `CMakeLists.txt` text transforms.
  - `src/ros/build/pythonSetupEditor.ts` — `setup.py`/`setup.cfg` transforms.
  - `src/ros/graph/rosGraphParsing.ts` — parsers for ROS 1/2 CLI output.
  - `src/ros/launch/launchArgsParsing.ts` — Python/XML launch-argument parsers.
  - `src/ros/environment/wslEnvironment.ts` — WSL/os-release parsing/formatting.
  - `src/ros/runtime/processUtils.ts` — POSIX process-tree signalling helpers.
- Stateful runtime helper:
  - `src/ros/runtime/wslPersistentGraphRunner.ts` — warm WSL bash process for
    batched graph snapshot commands.
- Responsibilities (of the facade):
  - ROS environment detection and command execution context resolution.
  - Workspace/package discovery and package details extraction.
  - Package/node creation and removal.
  - Run/launch execution with terminal routing.
  - Smart-build preflight checks and build stamp persistence.
  - Graph/query helpers for nodes, topics, services, actions, parameters.
  - Topic publish and action goal helpers.

### 3) Host Webview Controllers
- Files:
  - `src/views/packageManagerView.ts`
  - `src/views/nodeVisualizerView.ts`
  - `src/views/packageManagerMessages.ts`
  - `src/views/uiPreferences.ts`
- Responsibilities:
  - Validate/route inbound webview messages.
  - Call `RosWorkspace` APIs.
  - Persist and emit view state (pins, args configs, terminal selection, UI preferences).
  - Push normalized outbound payloads to webviews.

### 4) Webview UI Layer
- Files:
  - `media/packageManager/*`
  - `media/nodeVisualizer/index.js`
  - `media/shared/uiPreferences.js`
  - `media/shared/interactions.js`
  - `media/style.css`
- Responsibilities:
  - Render UI and local state.
  - Post commands to host.
  - Handle host responses.
  - Manage panel-local interaction behavior and shared UI settings.

## Runtime Data Flows

### Flow A: Create Package
1. Package Manager webview posts `createPackage`.
2. `PackageManagerViewProvider._handleCreate` normalizes inputs and delegates to `RosWorkspace.createPackage(...)`.
3. `RosWorkspace` resolves workspace/run target and maintainer identity, builds ROS CLI command, and dispatches it in a terminal.
4. Host returns `createDone` and refreshes package list.

### Flow B: Run Node / Launch File with Smart Build
1. Package Manager posts `runNode` or `launchFile`.
2. `PackageManagerViewProvider` delegates to `RosWorkspace.runNode(...)` or `RosWorkspace.launchFile(...)`.
3. `RosWorkspace.preLaunchBuildCheck(pkg)`:
   - If `rosDevToolkit.preLaunchBuildCheck` is false, launch directly.
   - Else run `evaluateBuildNeeds([pkg])`.
4. If stale packages exist:
   - Execute `buildThenRun(...)` with clean + `colcon build` + source + run command.
   - Mark build stamps for selected packages.
5. Else:
   - Run command directly in configured terminal target.

### Flow C: Node Visualizer Graph Refresh
1. Node Visualizer posts `refresh` (optionally scoped by category).
2. `NodeVisualizerViewProvider._sendGraphData(...)` fetches scoped list data through one batched runtime call:
   - `RosWorkspace.getGraphSnapshot(scope)` executes nodes/topics/services/actions/parameters in one composite shell command.
3. Host updates only successful categories, keeps last-good data for failed categories, syncs tracked topic subscriptions, and posts `graphData` with `refreshMeta` (stale map + partialFailure + effective interval).
4. Additional detail calls are on-demand:
   - `fetchNodeInfo` -> `nodeInfo`
   - `refreshConnections` -> `connectionData`
   - topic publish/action goal template/result round-trips.

## Persistence and State

### Workspace State
- Build stamps:
  - Key: `rosDevToolkit.buildStamps`
  - Owner: `BuildStampManager`

### Global State
- Package manager:
  - `pinnedLaunchFiles`
  - `launchArgConfigs`
  - legacy `launchArgs` (migration source)
  - `runTerminalTarget`
- Node visualizer:
  - `rosDevToolkit.nodeVisualizerPrefs`
- Shared UI:
  - `rosDevToolkit.webviewUiPreferences`

## Terminal and Environment Modes
- Terminal modes:
  - `auto`, `integrated`, `external`, plus Windows WSL-specific targets.
- Environment probing:
  - Linux and Windows WSL probes are provided by `RosWorkspace.getEnvironmentInfoReport()`.
- Run target selection:
  - Managed in Package Manager environment dialog and applied through `RosWorkspace.setRunTerminalTarget(...)`.

## Known Drift and Debt
- `RosWorkspace` is a high-complexity monolith (~5.1k lines) and the primary
  modularization target. Pure/stateless concerns have been peeled off into the
  helper modules listed above; the remaining stateful clusters (terminal
  management, run/launch orchestration, command-execution context, graph
  orchestration) are tightly coupled and still live in the facade. See
  `docs/exec-plans/active/rosworkspace-modularization.md` for the extraction plan.
- Terminal/process orchestration has no automated integration coverage; changes
  there must be verified manually in a running VS Code host.

## Test Coverage Map
Run with `npm test` (all suites), or the scoped scripts in `package.json`.

- Activation and command wiring:
  - `tests/integration/extensionHost/smoke.spec.ts`
- Extracted pure helper modules (characterization tests):
  - `tests/unit/ros/strings.spec.ts`
  - `tests/unit/ros/nodeTemplates.spec.ts`
  - `tests/unit/ros/cmakeEditor.spec.ts`
  - `tests/unit/ros/pythonSetupEditor.spec.ts`
  - `tests/unit/ros/rosGraphParsing.spec.ts`
  - `tests/unit/ros/launchArgsParsing.spec.ts`
  - `tests/unit/ros/wslEnvironment.spec.ts`
  - `tests/unit/ros/processUtils.spec.ts`
- Smart-build / dependency / build stamps / workspace flows:
  - `tests/unit/rosWorkspace/*.spec.ts`
- Persistent WSL graph runner:
  - `tests/unit/rosWorkspace/wslPersistentGraphRunner.spec.ts`
- View helpers and UI preferences:
  - `tests/unit/views/uiPreferences.spec.ts`
- Webview DOM / modal styles:
  - `tests/unit/webview/packageManager/smoke.spec.ts`
  - `tests/unit/webview/modalStyles.spec.ts`

> Note: the `packageManagerUserFlow` / `nodeVisualizerUserFlow` / `runtimeMatrix`
> extension-host specs referenced by earlier drafts were never implemented and
> are not part of the suite. Building them out is tracked as future work.
