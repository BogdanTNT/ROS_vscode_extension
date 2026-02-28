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
- Files:
  - `src/ros/rosWorkspace.ts`
  - `src/ros/buildStampManager.ts`
  - `src/ros/dependencyResolver.ts`
  - `src/ros/buildPolicy.ts`
- Responsibilities:
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
- `src/views/buildRunView.ts` is present but not wired in `src/extension.ts` activation.
  - It is currently an inactive implementation path.
- `RosWorkspace` is a high-complexity monolith and the primary modularization target.

## Test Coverage Map
- Activation and command wiring:
  - `tests/integration/extensionHost/smoke.spec.ts`
- Package manager user flow:
  - `tests/integration/extensionHost/packageManagerUserFlow.spec.ts`
- Node visualizer user flow:
  - `tests/integration/extensionHost/nodeVisualizerUserFlow.spec.ts`
- Runtime matrix:
  - `tests/integration/extensionHost/runtimeMatrix.spec.ts`
- Smart-build/dependency/build stamps:
  - `tests/unit/rosWorkspace/*.spec.ts`
- Webview helper/package manager DOM:
  - `tests/unit/webview/packageManager/*.spec.ts`
