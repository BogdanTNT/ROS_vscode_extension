# Agent Playbook

## Purpose
This file is the operational contract for human and AI contributors working in this repository.
Use it to decide what to run, where to make changes, and what must be kept in sync.

## Repository Map
- `src/extension.ts`
  - Extension activation, command registration, and webview provider wiring.
- `src/ros/*`
  - Runtime and ROS CLI integration (`RosWorkspace`, smart-build policy, dependency graph, build stamps).
- `src/views/*`
  - Extension-host webview controllers for Package Manager and Node Visualizer.
- `media/*`
  - Webview-side JavaScript, shared interactions, and shared CSS.
- `tests/*`
  - Unit and integration suites (see `tests/README.md`).
- `README.md`
  - User-facing extension overview.
- `ARCHITECTURE.md`
  - High-level component and data-flow reference.
- `docs/*`
  - Design docs, product specs, execution plans, and references.

## Canonical Commands
Source of truth: `package.json` scripts.

- Install/build
  - `npm install`
  - `npm run compile`
  - `npm run watch`
  - `npm run lint`
- Test
  - `npm run test`
  - `npm run test:unit`
  - `npm run test:webview`
  - `npm run test:integration`
  - `npm run test:matrix`
  - `npm run test:ui-click`
- Local E2E orchestration
  - `npm run e2e:smoke`
  - `npm run e2e:full`
  - `npm run e2e:build-goldens`
  - `npm run e2e:clean`

## Change-to-Test Matrix
Run at least the first column for fast feedback, then the second column before merging.

| Change Area | Run First | Run Before Merge | Notes |
| --- | --- | --- | --- |
| `src/ros/**` | `npm run test:unit -- tests/unit/rosWorkspace` | `npm run test:matrix` | `RosWorkspace` and smart-build behavior drive run/launch outcomes. |
| `src/views/packageManagerView.ts` or `src/views/packageManagerMessages.ts` | `npm run test:unit` | `npm run test:matrix` | Covers host-side package-manager flows and contracts. |
| `src/views/nodeVisualizerView.ts` | `npm run test:unit` | `npm run test:matrix` | Covers graph refresh, topic tools, and action flows. |
| `media/packageManager/**` | `npm run test:webview` | `npm run test:ui-click` | Webview DOM and message wiring changes. |
| `media/nodeVisualizer/**` | `npm run test:unit` | `npm run test:ui-click` | Node visualizer interaction and message payload changes. |
| `media/shared/**` or `src/views/uiPreferences.ts` | `npm run test:unit -- tests/unit/views/uiPreferences.spec.ts` | `npm run test:unit` | Shared UI preference behavior is consumed by both panels. |
| `src/extension.ts` or command/settings wiring in `package.json` | `npm run test:integration -- tests/integration/extensionHost/smoke.spec.ts` | `npm run test:integration` | Verifies activation wiring and command routing. |
| `tests/**` only | Targeted changed tests | `npm run test` | Keep suite structure and naming consistent with `tests/README.md`. |

## Local Ops Boundary
Treat these as local-ops surfaces by default unless you intentionally promote them to committed product surface:

- `e2e/` and `goldens/`
  - Currently ignored by `.gitignore`.
- `scripts/`
  - Present locally and used by npm scripts, but currently untracked in this working tree.

If you decide to commit or formalize any of these, also update:
- `docs/references/test-matrix-reference.md`
- `docs/index.md`
- `README.md` (if user-facing)

## Documentation Maintenance Rules
Update docs in the same change when any of the following occurs:

- New command, activation event, or setting:
  - Update `docs/references/settings-reference.md` and `ARCHITECTURE.md`.
- Webview host/webview message contract changes:
  - Update `docs/design-docs/webview-message-contracts.md`.
- Smart-build/runtime behavior changes:
  - Update `docs/design-docs/runtime-and-smart-build.md` and `docs/product-specs/core-workflows.md`.
- Significant component wiring changes:
  - Update `ARCHITECTURE.md`.
- New or retired debt items:
  - Update `docs/exec-plans/tech-debt-tracker.md`.

