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
  - Extension-host integration and activation smoke tests.
- `tests/fixtures/workspaces/`
  - Reusable fake ROS workspace fixtures (`package.xml`, launch files, etc.).
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

## Notes

- The current setup is config-less Vitest for Node 18 compatibility.
- `vscode` is mocked per test file via `vi.mock('vscode', ...)`.
