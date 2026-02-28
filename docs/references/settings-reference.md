# Settings Reference

Source of truth: `package.json` -> `contributes.configuration.properties`.

## `rosDevToolkit.*` Settings
| Key | Type | Default | Operational Impact | Consumed In |
| --- | --- | --- | --- | --- |
| `rosDevToolkit.rosSetupPath` | `string` | `""` | Overrides ROS setup script used for sourcing commands. If empty, runtime auto-detect/fallback is used. | `src/ros/rosWorkspace.ts` |
| `rosDevToolkit.launchInExternalTerminal` | `boolean` | `true` | Controls default run/launch terminal routing when run target is `auto`. | `src/ros/rosWorkspace.ts`, Package Manager environment UI |
| `rosDevToolkit.preLaunchBuildCheck` | `boolean` | `true` | Enables smart prelaunch stale-package evaluation and build chaining before run/launch. | `src/ros/rosWorkspace.ts`, `src/views/packageManagerView.ts` |
| `rosDevToolkit.symlinkInstall` | `boolean` | `true` | Adds/removes `--symlink-install` for ROS 2 builds. Affects rebuild speed and build artifact behavior. | `src/ros/rosWorkspace.ts` |
| `rosDevToolkit.defaultMaintainerName` | `string` | `""` | Default maintainer name for package creation; prompts/user git fallback if empty. | `src/ros/rosWorkspace.ts` |
| `rosDevToolkit.defaultMaintainerEmail` | `string` | `""` | Default maintainer email for package creation; prompts/user git fallback if empty. | `src/ros/rosWorkspace.ts` |
| `rosDevToolkit.nodeVisualizerAutoRefreshIntervalMs` | `number` | `3000` | Node Visualizer auto-refresh interval; clamped in code to `[250, 120000]`. | `src/views/nodeVisualizerView.ts` |
| `rosDevToolkit.graphListTimeoutSeconds` | `number` | `6` | Timeout for graph list CLI calls (nodes/topics/services/actions/params). | `src/ros/rosWorkspace.ts` |
| `rosDevToolkit.nodeInfoTimeoutSeconds` | `number` | `3` | Timeout for node info CLI queries. | `src/ros/rosWorkspace.ts` |
| `rosDevToolkit.topicInfoTimeoutSeconds` | `number` | `3` | Timeout for topic info CLI queries. | `src/ros/rosWorkspace.ts` |
| `rosDevToolkit.nodeVisualizerUsePersistentWslShell` | `boolean` | `true` | Enables persistent WSL bash process for Node Visualizer graph snapshots on Windows+WSL to reduce startup overhead. Enabled by default so WSL optimization is automatic when WSL run context is detected. | `src/ros/rosWorkspace.ts` |

## Behavior Notes
- `preLaunchBuildCheck` can be toggled from Package Manager UI and is persisted in VS Code settings.
- `nodeVisualizerAutoRefreshIntervalMs` is runtime-clamped even if user sets out-of-range values.
- Node Visualizer panel can edit `nodeVisualizerAutoRefreshIntervalMs`; host persists edits at workspace scope.
- `launchInExternalTerminal` applies only when run target is `auto`; explicit run target overrides it.

## Change Protocol
When adding/removing/changing any `rosDevToolkit.*` setting:

1. Update `package.json`.
2. Update consumers in `src/`.
3. Update this reference file.
4. Update `ARCHITECTURE.md` if behavior flow changes.
