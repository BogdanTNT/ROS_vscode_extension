# Tech Debt Tracker

## Priority Legend
- `P0`: urgent correctness/reliability risk
- `P1`: high-impact maintainability/perf debt
- `P2`: medium debt
- `P3`: low debt

## Open Debt Items
| Priority | Area | Debt | Evidence | Owner | Exit Criteria |
| --- | --- | --- | --- | --- | --- |
| P1 | Runtime architecture | `RosWorkspace` monolith couples many domains, raising regression risk and review cost. Pure/stateless concerns now extracted (`utils/strings`, `templates/nodeTemplates`, `build/*`, `graph/rosGraphParsing`, `launch/launchArgsParsing`, `environment/wslEnvironment`, `runtime/processUtils`); stateful terminal/run-launch/context clusters remain. | `src/ros/rosWorkspace.ts` | Extension maintainers | Complete plan in `exec-plans/active/rosworkspace-modularization.md`; keep behavior parity and passing tests. |
| P1 | Smart-build policy | Dependency propagation config knobs (`BuildPolicyConfig.fastDependencyMode` etc.) are effectively dormant: the current implementation intentionally does not propagate rebuilds. The dead `_checkDependencyPropagation` path has been removed. | `src/ros/buildPolicy.ts` (`BuildPolicyConfig`) | Runtime maintainers | Either remove dormant config, or wire propagation into the active decision path with tests/documentation. |
| P2 | Legacy migration path | Legacy launch-args migration still present with explicit removal deadline comment. | `src/views/packageManagerView.ts` (`LEGACY_LAUNCH_ARGS_KEY`, TODO remove-by v0.3.0 / 2026-06-30) | Package Manager maintainers | Remove migration path after confirming no active users need legacy key. |
| P2 | Deprecated API wrappers | Deprecated wrappers still retained and increase surface area. | `src/ros/rosWorkspace.ts` (`buildThenLaunch`, `buildThenRunNode`) | Runtime maintainers | Remove wrappers once internal and external call sites are fully migrated. |
| P2 | Contract duplication | Message command constants are duplicated across host TS and webview JS, increasing drift risk. | `src/views/packageManagerMessages.ts`, `media/packageManager/messages.js`, `media/nodeVisualizer/index.js` | UI maintainers | Introduce shared generated or single-source contract definitions, with CI check for divergence. |
| P3 | Local ops discoverability | E2E/golden operational surfaces are mostly local and not part of committed docs contract. | `.gitignore`, local `scripts/`, local `e2e/`, local `goldens/` | Repo maintainers | Decide promotion boundary; if promoted, add committed docs and CI hooks. |

## Recently Resolved
- **View wiring drift** — the unregistered `BuildRunViewProvider` and its
  `media/buildRun/` script were dead (superseded by the Package Manager view) and
  have been deleted, along with the empty `tests/unit/views/buildRun/` placeholder.
- **Smart-build dead path** — the never-called `_checkDependencyPropagation`
  method was removed (surfaced by enabling `noUnusedLocals`). The dormant config
  knobs it referenced remain tracked above.
- **Dead terminal-reuse code** — the unused `killProcessTree`,
  `pickLaunchTerminal`, and `matchesIntegratedLaunchTerminalProfile` methods were
  removed from `RosWorkspace`.
- **Tooling** — ESLint was wired up (flat config, `npm run lint`), and the
  `test:matrix` / `test:ui-click` scripts (which pointed at non-existent specs)
  were removed.

## Tracking Rules
- Every debt item must include:
  - concrete code anchor
  - owner
  - explicit exit criteria
- Move completed items to release notes and remove from this table.

