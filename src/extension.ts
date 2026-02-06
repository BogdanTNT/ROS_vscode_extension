import * as vscode from 'vscode';
import { PackageManagerViewProvider } from './views/packageManagerView';
import { BuildRunViewProvider } from './views/buildRunView';
import { NodeVisualizerViewProvider } from './views/nodeVisualizerView';
import { RosWorkspace } from './ros/rosWorkspace';

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
    const packageManagerProvider = new PackageManagerViewProvider(context.extensionUri, rosWorkspace, context);
    const buildRunProvider = new BuildRunViewProvider(context.extensionUri, rosWorkspace);
    const nodeVisualizerProvider = new NodeVisualizerViewProvider(context.extensionUri, rosWorkspace);

    context.subscriptions.push(
        vscode.window.registerWebviewViewProvider('rosPackageManager', packageManagerProvider),
        vscode.window.registerWebviewViewProvider('rosBuildRun', buildRunProvider),
        vscode.window.registerWebviewViewProvider('rosNodeVisualizer', nodeVisualizerProvider),
    );

    // ── Commands ───────────────────────────────────────────────
    context.subscriptions.push(
        vscode.commands.registerCommand('rosDevToolkit.createPackage', () => {
            packageManagerProvider.focusCreateForm();
        }),
        vscode.commands.registerCommand('rosDevToolkit.buildPackage', () => {
            buildRunProvider.triggerBuild();
        }),
        vscode.commands.registerCommand('rosDevToolkit.runNode', () => {
            buildRunProvider.triggerRun();
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
