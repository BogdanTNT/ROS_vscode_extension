# Core Workflows Spec

## Scope
Defines acceptance criteria for workflows currently advertised in `README.md`:
- Create package
- Create node
- Run node / launch file with argument presets
- Smart pre-launch build check
- Node Visualizer inspect and publish flows

## Out of Scope
- Local E2E golden-image orchestration (`e2e/`, `goldens/`, local `scripts/`)
- Unwired Build/Run sidebar (`src/views/buildRunView.ts`)

## Workflow A: Create Package

### Preconditions
- A workspace folder is open in VS Code.
- ROS environment is available for selected run target.

### User Actions
1. Open Package Manager panel.
2. Open New Package modal.
3. Enter package name, build type, dependencies, and optional metadata.
4. Submit.

### Acceptance Criteria
- Package name is normalized (whitespace -> underscores) and validated non-empty.
- Maintainer identity is resolved from settings/git/prompt flow.
- Correct ROS command is dispatched:
  - ROS 2: `ros2 pkg create ...`
  - ROS 1: `catkin_create_pkg ...`
- UI receives `createDone`.
- On success:
  - success feedback shown
  - package list refresh triggered

## Workflow B: Add / Remove Node in Existing Package

### Preconditions
- Target package exists in workspace and is discoverable by panel.

### User Actions
- Add node:
  1. Choose package.
  2. Enter node name and optional template/topic.
  3. Confirm add.
- Remove node:
  1. Choose package/node.
  2. Confirm remove.

### Acceptance Criteria
- Node/package names are validated as ROS-friendly identifiers.
- For Python packages:
  - source file is created/removed correctly
  - `setup.py` or `setup.cfg` entry points updated.
- For CMake packages:
  - source file is created/removed correctly
  - `CMakeLists.txt` targets updated.
- UI receives `addNodeDone` or `removeNodeDone` with boolean status.
- Package list refreshes on success.

## Workflow C: Run / Launch with Arguments and Terminal Target

### Preconditions
- Package/node or package/launch file is discoverable.

### User Actions
1. Select launch file or node.
2. Optionally choose saved args preset and run target.
3. Start run/launch.

### Acceptance Criteria
- Launch args presets can be stored and reused per source path.
- Terminal target selection supports:
  - `auto`, `integrated`, `external`
  - WSL target variants on Windows.
- Active terminal list is tracked and can be focused/killed/preferred.
- If run target override is provided, it is applied for that action.

## Workflow D: Smart Pre-Launch Build Check

### Preconditions
- `rosDevToolkit.preLaunchBuildCheck` setting available (default enabled).

### User Actions
1. Trigger run/launch.
2. Optional: toggle auto build check in UI.

### Acceptance Criteria
- When disabled:
  - run/launch executes directly.
- When enabled:
  - stale package evaluation runs for selected package closure.
  - if stale packages exist, build is chained before run/launch.
  - if no stale packages exist, run/launch starts immediately.
- For ROS 2 builds:
  - `--symlink-install` usage follows setting.
  - stale build/install dirs are cleaned before build.
- For Windows+WSL run targets:
  - if the selected package is missing from sourced environment at runtime, fallback build runs with `--packages-up-to <pkg>` so workspace-local dependencies are also built.

## Workflow E: Node Visualizer Inspect and Publish

### Preconditions
- ROS runtime is active and graph APIs are callable.

### User Actions
1. Refresh graph.
2. Filter/switch tabs.
3. Inspect node details and connection data.
4. Inspect topic roles.
5. Optionally publish a topic message or send action goal.
6. Optionally pin topic and monitor latest message stream.

### Acceptance Criteria
- Graph payload includes nodes/topics/services/actions/parameters and ROS version.
- Graph payload includes `refreshMeta` (effective interval, stale categories, partial-failure state).
- Refresh is sequential (no overlapping refresh loops) and supports adaptive backoff on failures.
- Auto-refresh scheduling keeps cadence relative to cycle start, so long cycles trigger the next pass immediately instead of waiting an extra full interval.
- Auto-refresh interval can be edited in-panel and is persisted to workspace settings.
- Node info and connection refresh are on-demand and resilient to failures.
- Failed list categories keep last-good data instead of blanking the full graph.
- Topic publish template and action goal template resolve type-aware defaults.
- Publish/send operations report success/error result payloads to UI.
- Tracked topic subscriptions stream latest messages and recover after closure.

## Non-Functional Acceptance
- UI should remain responsive when ROS CLI calls fail transiently.
- Behavior must be covered by existing unit/integration suites:
  - `tests/unit/rosWorkspace/*`
  - `tests/integration/extensionHost/*`
  - `tests/unit/webview/packageManager/*`
