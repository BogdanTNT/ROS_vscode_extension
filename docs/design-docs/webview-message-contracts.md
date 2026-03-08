# Webview Message Contracts

## Scope and Source of Truth
This document captures host/webview command contracts for:
- Package Manager
- Node Visualizer
- Shared UI preferences bridge

Authoritative code:
- `src/views/packageManagerMessages.ts`
- `src/views/packageManagerView.ts`
- `src/views/nodeVisualizerView.ts`
- `media/packageManager/messages.js`
- `media/nodeVisualizer/index.js`
- `media/shared/uiPreferences.js`

## Package Manager Contracts

### Webview -> Host (`PMToHostCommand` + shared UI commands)
| Command | Payload (required fields) | Host Handler |
| --- | --- | --- |
| `createPackage` | `name`, `buildType`, `deps`, optional `license`, `description` | `_handleCreate` |
| `refreshPackages` | none | `_sendPackageList` + `_sendBuildCheckState` |
| `requestEnvironmentDialogState` | none | `_sendEnvironmentDialogState` (cache-first) |
| `showEnvironmentInfo` | none | `_showEnvironmentInfo` |
| `setRunTerminalTarget` | `target` | `_setRunTerminalTarget` |
| `loadOtherPackages` | optional `force` | `_sendOtherPackageList` |
| `loadOtherPackageDetails` | `name` | `_sendOtherPackageDetails` |
| `openLaunch` | `path` | `_openLaunchFile` |
| `openNode` | `path` | `_openNodeFile` |
| `launchFile` | `pkg`, `file`, `path`, optional `args`, `argsName`, `runTarget` | `_launchFile` |
| `runNode` | `pkg`, `executable`, optional `args`, `argsName`, `path`, `runTarget` | `_runNode` |
| `togglePin` | `path` | `_togglePin` |
| `setLaunchArgConfigs` | `path`, `configs[]` | `_setLaunchArgConfigs` |
| `requestLaunchArgs` | `argsKey`, optional `sourcePath` | `_sendLaunchArgOptions` |
| `killTerminal` | `id` | `_killTerminal` |
| `focusTerminal` | `id` | `_focusTerminal` |
| `setPreferredTerminal` | `id` | `_setPreferredTerminal` |
| `toggleBuildCheck` | `enabled` | `_toggleBuildCheck` |
| `addNode` | `pkg`, `nodeName`, optional `pkgPath`, `nodeTemplate`, `nodeTopic` | `_handleAddNode` |
| `removeNode` | `pkg`, `nodeName`, optional `pkgPath`, `nodePath` | `_handleRemoveNode` |
| `createLaunch` | `pkg`, `launchName`, optional `pkgPath` | `_handleCreateLaunch` |
| `removeLaunch` | `pkg`, `launchName`, optional `pkgPath`, `launchPath` | `_handleRemoveLaunch` |
| `requestUiPreferences` | none | `_sendUiPreferences` |
| `setUiPreferences` | `preferences` | `_setUiPreferences` |

### Host -> Webview (`PMToWebviewCommand` + shared UI state)
| Command | Payload Summary | Produced By |
| --- | --- | --- |
| `packageList` | `packages[]`, `pinned[]`, `launchArgConfigs`, `terminals[]`, `preferredTerminalId` | `_sendPackageList` |
| `otherPackageList` | `packages[]` | `_sendOtherPackageList` |
| `otherPackageDetails` | `package` details object | `_sendOtherPackageDetails` |
| `environmentDialogState` | `report`, `target`, `targetOptions[]`, `autoLaunchInExternalTerminal` | `_sendEnvironmentDialogState` |
| `createDone` | `success` | `_handleCreate` |
| `focusCreate` | none | `focusCreateForm` |
| `launchArgsOptions` | `argsKey`, `options[]` | `_sendLaunchArgOptions` |
| `terminalList` | `terminals[]` | terminal change subscription |
| `buildCheckState` | `enabled` | `_sendBuildCheckState` |
| `addNodeDone` | `success` | `_handleAddNode` |
| `removeNodeDone` | `success` | `_handleRemoveNode` |
| `createLaunchDone` | `success` | `_handleCreateLaunch` |
| `removeLaunchDone` | `success` | `_handleRemoveLaunch` |
| `uiPreferencesState` | `preferences` | `applyUiPreferences` |

## Node Visualizer Contracts

### Webview -> Host
| Command | Payload (required fields) | Host Handler |
| --- | --- | --- |
| `refresh` | optional `showLoading`, optional scope booleans (`nodes`, `topics`, `services`, `actions`, `parameters`) | `_sendGraphData` |
| `setAutoRefresh` | `enabled` | updates prefs + `_syncAutoRefresh` |
| `setAutoRefreshInterval` | `intervalMs` | updates `rosDevToolkit.nodeVisualizerAutoRefreshIntervalMs` (workspace scope) |
| `viewScope` | scope booleans | updates `_lastScope` |
| `fetchNodeInfo` | `nodeName` | `_fetchAndSendNodeInfo` |
| `refreshConnections` | none | `_refreshAndSendConnections` |
| `getTopicRoles` | `topicName` | `_sendTopicRoles` |
| `setOverviewToggles` | `showNodes`, `showTopics`, `showServices`, `showActions`, `showParameters` | prefs update |
| `setTrackedTopics` | `topicNames[]` | `_setTrackedTopics` |
| `getTopicPublishTemplate` | `topicName`, optional `topicTypeHint` | `_sendTopicPublishTemplate` |
| `publishTopicMessage` | `topicName`, `payload`, optional `topicTypeHint` | `_publishTopicMessage` |
| `getActionGoalTemplate` | `actionName`, optional `actionTypeHint` | `_sendActionGoalTemplate` |
| `sendActionGoal` | `actionName`, `payload`, optional `actionTypeHint` | `_sendActionGoal` |
| `requestUiPreferences` | none | `_sendUiPreferences` |
| `setUiPreferences` | `preferences` | `_setUiPreferences` |

### Host -> Webview
| Command | Payload Summary | Produced By |
| --- | --- | --- |
| `loading` | none | `_sendGraphData` queued/loading states |
| `graphData` | list categories + `nodeWarnings[]`, `connections`, `topicLatestMessages`, `topicLatestMessageTimes`, `rosVersion`, `prefs`, `refreshMeta` (`autoRefreshIntervalMs`, `stale`, `partialFailure`) | `_sendGraphData` |
| `nodeInfo` | `nodeName`, `info` graph roles | `_fetchAndSendNodeInfo` |
| `connectionData` | full `connections` map | `_refreshAndSendConnections` |
| `topicRoles` | `topicName`, `publishers[]`, `subscribers[]` | `_sendTopicRoles` |
| `topicLatestMessage` | `topicName`, `message`, `receivedAt` | topic subscription handlers |
| `topicPublishTemplate` | topic template result (`success`, `topicType`, `template` or `error`) | `_sendTopicPublishTemplate` |
| `topicPublishResult` | publish result (`success`, `topicType` or `error`) | `_publishTopicMessage` |
| `actionGoalTemplate` | action template result (`success`, `actionType`, `template` or `error`) | `_sendActionGoalTemplate` |
| `actionGoalResult` | action send result (`success`, `actionType` or `error`) | `_sendActionGoal` |
| `uiPreferencesState` | `preferences` | `applyUiPreferences` |

## Error and Resilience Rules
- Host handlers should fail soft where possible:
  - Node Visualizer graph refresh keeps last-good category data, marks stale categories in `refreshMeta`, and continues the refresh loop.
  - Package Manager handlers validate required identifiers before taking action.
- Contract payload changes must be synchronized in both:
  - host TypeScript controller
  - corresponding webview JavaScript handler/constants

## Contract Change Procedure
When adding/renaming/removing a command:

1. Update host constant or switch handler (`src/views/*`).
2. Update webview sender/listener (`media/*`).
3. Update this document.
4. Run targeted suites:
   - `npm run test:webview`
   - `npm run test:matrix`
   - `npm run test:ui-click` for interaction changes.
