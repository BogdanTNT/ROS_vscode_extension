# Test Matrix Reference

Source of truth:
- `package.json` scripts
- `tests/README.md`

## Script-to-Suite Mapping
| Script | Scope | Primary Purpose |
| --- | --- | --- |
| `npm run test` | all Vitest suites | Full regression baseline. |
| `npm run test:unit` | `tests/unit` | Unit behavior of runtime/view helpers. |
| `npm run test:webview` | `tests/unit/webview` | Webview-side DOM/message smoke tests. |
| `npm run test:integration` | `tests/integration` | Extension-host behavior and integrated flows. |
| `npm run lint` / `npm run lint:fix` | `src/**/*.ts` | ESLint (flat config, `typescript-eslint`). |
| `npm run test:watch` | watch mode | Local iterative testing. |

> Removed: `test:matrix` and `test:ui-click` previously pointed at spec files and
> directories that were never created (`runtimeMatrix.spec.ts`,
> `packageManagerUserFlow.spec.ts`, `nodeVisualizerUserFlow.spec.ts`,
> `tests/integration/uiClick/`). Building those extension-host user-flow suites is
> tracked as future work; until then, `test:integration` covers the real
> extension-host specs.

## Local E2E Scripts
These are local operations and not part of committed CI surface by default:
- `npm run e2e:smoke`
- `npm run e2e:full`
- `npm run e2e:build-goldens`
- `npm run e2e:clean`

## Recommended Selection by Change Type
| Change Type | Fast Loop | Merge Gate |
| --- | --- | --- |
| Runtime (`src/ros/**`) | `npm run test:unit -- tests/unit/ros` | `npm test` + `npm run lint` |
| Host view controllers (`src/views/**`) | `npm run test:unit` | `npm run test:integration` |
| Package Manager webview (`media/packageManager/**`) | `npm run test:webview` | `npm run test:webview` |
| Node Visualizer webview (`media/nodeVisualizer/**`) | `npm run test:unit` | `npm run test:integration` |
| Command/activation/settings wiring (`src/extension.ts`, `package.json`) | `npm run test:integration -- tests/integration/extensionHost/smoke.spec.ts` | `npm run test:integration` |

