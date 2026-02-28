# Test Workspace Layout

This folder is organized so each test domain is isolated and easy to maintain.

## Structure

- `tests/unit/rosWorkspace/`
  - Unit tests for `src/ros/rosWorkspace.ts`.
- `tests/unit/views/`
  - Unit tests for view providers in `src/views/`.
- `tests/unit/webview/packageManager/`
  - Webview-side unit tests for `media/packageManager/*.js` (DOM/events/render logic).
- `tests/integration/extensionHost/`
  - Extension-host integration, button/user-flow tests, and runtime matrix tests.
- `tests/integration/uiClick/`
  - Selective literal button click tests that execute real webview JS bundles in JSDOM.
- `tests/fixtures/workspaces/`
  - Reusable fake ROS workspace fixtures (`package.xml`, launch files, etc.).
- `tests/fixtures/workspaces/exampleProjects/`
  - Optional real/sample ROS projects provided by users for fixture-driven validation.
- `tests/fixtures/launchFiles/`
  - Launch-file fixtures for launch arg parsing coverage.
- `tests/helpers/mocks/`
  - Shared mocks (for `vscode`, `child_process`, etc.).
- `tests/helpers/workspaceFactory/`
  - Helpers to create/remove temporary test workspaces.
- `tests/setup/`
  - Test-runner setup files (globals, jsdom setup, custom matchers).
- `tests/suites/`
  - Optional grouped suite entrypoints (run only unit, only webview, etc.).

## Naming Convention

- Keep test files next to their domain folder.
- Use `*.spec.ts` for unit/integration tests.
- Use `*.spec.js` only for pure JS webview tests if not transpiling.

## Modular Add/Remove Rules

- Add a new feature area by creating one folder under `tests/unit/` or `tests/integration/`.
- Keep shared utilities in `tests/helpers/` only (avoid cross-importing test files).
- Keep fixture data in `tests/fixtures/` only.
- Remove a test domain by deleting its folder; no other folder should depend on it.

## Run

- `npm run test`
- `npm run test:unit`
- `npm run test:webview`
- `npm run test:integration`
- `npm run test:matrix`
- `npm run test:ui-click`

## Runtime Matrix Coverage

- `tests/integration/extensionHost/runtimeMatrix.spec.ts`
  - ROS 1 and ROS 2 command families (`roslaunch`/`rosrun` vs `ros2 launch`/`ros2 run`).
  - Terminal routing matrix (`auto`, `integrated`, `external`, WSL integrated/external targets).
- `tests/integration/extensionHost/packageManagerUserFlow.spec.ts`
  - Package Manager webview host commands (equivalent to user button flows).
- `tests/integration/extensionHost/nodeVisualizerUserFlow.spec.ts`
  - Node Visualizer webview host commands (refresh, topic publish, action goals, tracked topics).

## Using Example Projects As Fixtures

- Put sample projects under `tests/fixtures/workspaces/exampleProjects/<project-name>/`.
- Keep each fixture self-contained (workspace root with packages inside it).
- Add or extend tests to point `__setWorkspaceFolder(...)` at those fixture paths.

## Notes

- The current setup is config-less Vitest for Node 18 compatibility.
- `vscode` is mocked per test file via `vi.mock('vscode', ...)`.
