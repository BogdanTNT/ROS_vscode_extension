# Tech Debt Tracker

## Priority Legend
- `P0`: urgent correctness/reliability risk
- `P1`: high-impact maintainability/perf debt
- `P2`: medium debt
- `P3`: low debt

## Open Debt Items
| Priority | Area | Debt | Evidence | Owner | Exit Criteria |
| --- | --- | --- | --- | --- | --- |
| P1 | Runtime architecture | `RosWorkspace` monolith couples many domains, raising regression risk and review cost. | `src/ros/rosWorkspace.ts` | Extension maintainers | Complete plan in `exec-plans/active/rosworkspace-modularization.md`; keep behavior parity and passing tests. |
| P1 | Smart-build policy | Dependency propagation config exists but current implementation intentionally does not propagate rebuilds; config knobs are effectively dormant. | `src/ros/buildPolicy.ts` (`_checkDependencyPropagation`, `BuildPolicyConfig`) | Runtime maintainers | Either remove dormant config and dead path, or wire policy into active decision path with tests/documentation updates. |
| P2 | View wiring drift | Build/Run provider exists but is not registered on activation. | `src/views/buildRunView.ts`, `src/extension.ts` | UI maintainers | Decide to delete or wire provider; update architecture and tests accordingly. |
| P2 | Legacy migration path | Legacy launch-args migration still present with explicit removal deadline comment. | `src/views/packageManagerView.ts` (`LEGACY_LAUNCH_ARGS_KEY`, TODO remove-by v0.3.0 / 2026-06-30) | Package Manager maintainers | Remove migration path after confirming no active users need legacy key. |
| P2 | Deprecated API wrappers | Deprecated wrappers still retained and increase surface area. | `src/ros/rosWorkspace.ts` (`buildThenLaunch`, `buildThenRunNode`) | Runtime maintainers | Remove wrappers once internal and external call sites are fully migrated. |
| P2 | Contract duplication | Message command constants are duplicated across host TS and webview JS, increasing drift risk. | `src/views/packageManagerMessages.ts`, `media/packageManager/messages.js`, `media/nodeVisualizer/index.js` | UI maintainers | Introduce shared generated or single-source contract definitions, with CI check for divergence. |
| P3 | Local ops discoverability | E2E/golden operational surfaces are mostly local and not part of committed docs contract. | `.gitignore`, local `scripts/`, local `e2e/`, local `goldens/` | Repo maintainers | Decide promotion boundary; if promoted, add committed docs and CI hooks. |

## Tracking Rules
- Every debt item must include:
  - concrete code anchor
  - owner
  - explicit exit criteria
- Move completed items to release notes and remove from this table.

