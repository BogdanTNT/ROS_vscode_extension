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
| `npm run test:matrix` | selected extension-host specs | Runtime/terminal matrix + user-flow smoke. |
| `npm run test:ui-click` | `tests/integration/uiClick` | Literal button-click level webview interaction checks. |
| `npm run test:watch` | watch mode | Local iterative testing. |

## Extension-Host Matrix Targets
`npm run test:matrix` currently runs:
- `tests/integration/extensionHost/runtimeMatrix.spec.ts`
- `tests/integration/extensionHost/packageManagerUserFlow.spec.ts`
- `tests/integration/extensionHost/nodeVisualizerUserFlow.spec.ts`

## Local E2E Scripts
These are local operations and not part of committed CI surface by default:
- `npm run e2e:smoke`
- `npm run e2e:full`
- `npm run e2e:build-goldens`
- `npm run e2e:clean`

## Recommended Selection by Change Type
| Change Type | Fast Loop | Merge Gate |
| --- | --- | --- |
| Runtime (`src/ros/**`) | `npm run test:unit -- tests/unit/rosWorkspace` | `npm run test:matrix` |
| Host view controllers (`src/views/**`) | `npm run test:unit` | `npm run test:integration` |
| Package Manager webview (`media/packageManager/**`) | `npm run test:webview` | `npm run test:ui-click` |
| Node Visualizer webview (`media/nodeVisualizer/**`) | `npm run test:unit` | `npm run test:ui-click` |
| Command/activation/settings wiring (`src/extension.ts`, `package.json`) | `npm run test:integration -- tests/integration/extensionHost/smoke.spec.ts` | `npm run test:integration` |

