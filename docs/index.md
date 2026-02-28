# Documentation Index

## Purpose
`docs/` is the internal engineering documentation spine for ROS Dev Toolkit.
It complements:
- `README.md` (user-facing overview)
- `ARCHITECTURE.md` (top-level system view)
- `AGENTS.md` (contributor/agent execution rules)

## Read Order
1. `../README.md`
2. `../ARCHITECTURE.md`
3. `design-docs/index.md`
4. `product-specs/index.md`
5. `references/settings-reference.md`
6. `references/test-matrix-reference.md`
7. `exec-plans/tech-debt-tracker.md`

## Ownership
| Area | Primary Owner | Backup Owner |
| --- | --- | --- |
| Architecture and contracts | Extension maintainers | Any active contributor touching `src/` |
| Product specs | Extension maintainers | Feature author |
| References | Extension maintainers | Release owner |
| Execution plans and debt tracker | Extension maintainers | Refactor author |

## Update Cadence
- On every PR that changes:
  - command/settings wiring
  - host/webview message contracts
  - smart-build behavior
  - acceptance criteria of key user flows
- Weekly cleanup:
  - move completed plans from `exec-plans/active/` to `exec-plans/completed/`
  - prune or re-prioritize debt entries

## Scope Boundary (This Pass)
Current docs are committed-surface-first.

- In scope:
  - `src/`, `media/`, `tests/`, `README.md`, package scripts/settings.
- Out of scope by default in this pass:
  - local E2E/golden operations (`e2e/`, `goldens/`, and local `scripts/`) except explicit boundary notes.

