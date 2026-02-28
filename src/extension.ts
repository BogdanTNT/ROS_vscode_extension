import * as vscode from 'vscode';
import { PackageManagerViewProvider } from './views/packageManagerView';
import { NodeVisualizerViewProvider } from './views/nodeVisualizerView';
import { RosWorkspace } from './ros/rosWorkspace';
import { WebviewUiPreferences } from './views/uiPreferences';

export function activate(context: vscode.ExtensionContext) {
    console.log('ROS Dev Toolkit is now active');

    const rosWorkspace = new RosWorkspace();

    // Initialise smart-build subsystem (stamps, deps, policy).
    rosWorkspace.initSmartBuild(context);

    // Prevent CMake Tools from prompting for a kit on startup.
    const cmakeConfig = vscode.workspace.getConfiguration('cmake');
    cmakeConfig.update('configureOnOpen', false, vscode.ConfigurationTarget.Workspace);
    cmakeConfig.update('configureOnEdit', false, vscode.ConfigurationTarget.Workspace);

    // ── Sidebar Webview Providers ──────────────────────────────
    const providers: {
        packageManager?: PackageManagerViewProvider;
        nodeVisualizer?: NodeVisualizerViewProvider;
    } = {};

    const broadcastUiPreferences = (preferences: WebviewUiPreferences) => {
        providers.packageManager?.applyUiPreferences(preferences);
        providers.nodeVisualizer?.applyUiPreferences(preferences);
    };

    const packageManagerProvider = new PackageManagerViewProvider(
        context.extensionUri,
        rosWorkspace,
        context,
        broadcastUiPreferences,
    );
    const nodeVisualizerProvider = new NodeVisualizerViewProvider(
        context.extensionUri,
        rosWorkspace,
        context,
        broadcastUiPreferences,
    );

    providers.packageManager = packageManagerProvider;
    providers.nodeVisualizer = nodeVisualizerProvider;

    // Probe environment once on activation and cache results for panel restore.
    packageManagerProvider.warmEnvironmentCacheOnStartup();

    context.subscriptions.push(
        vscode.window.registerWebviewViewProvider('rosPackageManager', packageManagerProvider),
        vscode.window.registerWebviewViewProvider('rosNodeVisualizer', nodeVisualizerProvider),
    );

    // ── Commands ───────────────────────────────────────────────
    context.subscriptions.push(
        vscode.commands.registerCommand('rosDevToolkit.createPackage', () => {
            packageManagerProvider.focusCreateForm();
        }),
        vscode.commands.registerCommand('rosDevToolkit.openSourcedTerminal', () => {
            rosWorkspace.openSourcedTerminal();
        }),
        vscode.commands.registerCommand('rosDevToolkit.refreshGraph', () => {
            nodeVisualizerProvider.refreshGraph();
        }),
    );

    // Detect ROS distro on startup
    rosWorkspace.detectEnvironment().then((info) => {
        if (info) {
            vscode.window.setStatusBarMessage(
                `ROS Dev Toolkit: detected ${info.distro} (ROS ${info.version})`,
                4000
            );
        }
    });
}

export function deactivate() {
    // Cleanup is handled by VS Code's disposable subscriptions,
    // but the file watcher in the smart-build subsystem needs
    // explicit disposal since it's not pushed to subscriptions.
}
