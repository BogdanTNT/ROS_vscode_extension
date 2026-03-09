import * as vscode from 'vscode';
import { RosWorkspace, RosWorkspacePackageDetails, RunTerminalTargetOption } from '../ros/rosWorkspace';
import { getWebviewHtml } from './webviewHelper';
import { PMToHostCommand, PMToWebviewCommand } from './packageManagerMessages';
import {
    loadWebviewUiPreferences,
    saveWebviewUiPreferences,
    WebviewUiPreferences,
} from './uiPreferences';

type LaunchArgConfig = { id: string; name: string; args: string; runTarget?: string };
type LaunchArgConfigMap = Record<string, { configs: LaunchArgConfig[] }>;

const PINNED_LAUNCH_FILES_KEY = 'pinnedLaunchFiles';
const LAUNCH_ARG_CONFIGS_KEY = 'launchArgConfigs';
const LEGACY_LAUNCH_ARGS_KEY = 'launchArgs';
const RUN_TERMINAL_TARGET_KEY = 'runTerminalTarget';
const ENVIRONMENT_DIALOG_CACHE_KEY = 'rosDevToolkit.environmentDialogCache';
const DEFAULT_CREATE_PACKAGE_DESCRIPTION = 'TO DO: A very good package description';
const PACKAGE_MANAGER_BASE_TITLE = 'Package Manager';
// TODO(remove-by: v0.3.0 / 2026-06-30): Remove migration from legacy launch args.

interface EnvironmentDialogCacheSnapshot {
    report: string;
    targetOptions: RunTerminalTargetOption[];
    autoLaunchInExternalTerminal: boolean;
    refreshedAt: number;
}

/**
 * Sidebar webview that lets the user create new ROS packages.
 */
export class PackageManagerViewProvider implements vscode.WebviewViewProvider {
    private _view?: vscode.WebviewView;
    private _pinnedLaunchFiles: string[] = [];
    private _launchArgConfigs: LaunchArgConfigMap = {};
    private _cachedOtherPackages?: string[];
    private _cachedOtherPackageDetails = new Map<string, RosWorkspacePackageDetails>();
    private _environmentDialogCache?: EnvironmentDialogCacheSnapshot;
    private _environmentDialogRefreshPromise?: Promise<EnvironmentDialogCacheSnapshot | undefined>;

    constructor(
        private readonly _extensionUri: vscode.Uri,
        private readonly _ros: RosWorkspace,
        private readonly _context: vscode.ExtensionContext,
        private readonly _onUiPreferencesChanged?: (preferences: WebviewUiPreferences) => PromiseLike<void> | void,
    ) {
        this._environmentDialogCache = this._readEnvironmentDialogCache();
    }

    // ── WebviewViewProvider ────────────────────────────────────
    resolveWebviewView(
        webviewView: vscode.WebviewView,
        _context: vscode.WebviewViewResolveContext,
        _token: vscode.CancellationToken,
    ): void {
        this._view = webviewView;
        this._setViewTitle(this._getStoredRunTerminalTarget());

        webviewView.webview.options = {
            enableScripts: true,
            localResourceRoots: [this._extensionUri],
        };

        webviewView.webview.html = this._getHtml(webviewView.webview);

        // Handle messages from the webview
        webviewView.webview.onDidReceiveMessage(async (msg) => {
            switch (msg.command) {
                case PMToHostCommand.CREATE_PACKAGE:
                    await this._handleCreate(msg);
                    break;
                case PMToHostCommand.REFRESH_PACKAGES:
                    await this._sendPackageList();
                    this._sendBuildCheckState();
                    break;
                case PMToHostCommand.REQUEST_ENV_DIALOG_STATE:
                    await this._sendEnvironmentDialogState();
                    break;
                case PMToHostCommand.SHOW_ENV_INFO:
                    await this._showEnvironmentInfo();
                    break;
                case PMToHostCommand.OPEN_SOURCED_TERMINAL:
                    this._ros.openSourcedTerminal();
                    break;
                case PMToHostCommand.SET_RUN_TERMINAL_TARGET:
                    await this._setRunTerminalTarget(msg.target);
                    break;
                case PMToHostCommand.LOAD_OTHER_PACKAGES:
                    await this._sendOtherPackageList(Boolean(msg.force));
                    break;
                case PMToHostCommand.LOAD_OTHER_PACKAGE_DETAILS:
                    await this._sendOtherPackageDetails(msg.name);
                    break;
                case PMToHostCommand.OPEN_LAUNCH:
                    await this._openLaunchFile(msg.path);
                    break;
                case PMToHostCommand.OPEN_NODE:
                    await this._openNodeFile(msg.path);
                    break;
                case PMToHostCommand.LAUNCH_FILE:
                    await this._launchFile(msg.pkg, msg.file, msg.path, msg.args, msg.argsName, msg.runTarget);
                    break;
                case PMToHostCommand.RUN_NODE:
                    await this._runNode(msg.pkg, msg.executable, msg.args, msg.argsName, msg.path, msg.runTarget);
                    break;
                case PMToHostCommand.TOGGLE_PIN:
                    await this._togglePin(msg.path);
                    break;
                case PMToHostCommand.SET_LAUNCH_ARG_CONFIGS:
                    await this._setLaunchArgConfigs(msg.path, msg.configs);
                    break;
                case PMToHostCommand.REQUEST_LAUNCH_ARGS:
                    await this._sendLaunchArgOptions(msg.argsKey ?? msg.path, msg.sourcePath ?? msg.path);
                    break;
                case PMToHostCommand.KILL_TERMINAL:
                    await this._killTerminal(msg.id);
                    break;
                case PMToHostCommand.FOCUS_TERMINAL:
                    await this._focusTerminal(msg.id);
                    break;
                case PMToHostCommand.RELAUNCH_TERMINAL:
                    await this._relaunchTerminal(msg.id);
                    break;
                case PMToHostCommand.SET_PREFERRED_TERMINAL:
                    await this._setPreferredTerminal(msg.id);
                    break;
                case PMToHostCommand.TOGGLE_BUILD_CHECK:
                    await this._toggleBuildCheck(msg.enabled);
                    break;
                case PMToHostCommand.ADD_NODE:
                    await this._handleAddNode(msg.pkg, msg.nodeName, msg.pkgPath, msg.nodeTemplate, msg.nodeTopic);
                    break;
                case PMToHostCommand.REMOVE_NODE:
                    await this._handleRemoveNode(msg.pkg, msg.nodeName, msg.pkgPath, msg.nodePath);
                    break;
                case PMToHostCommand.CREATE_LAUNCH:
                    await this._handleCreateLaunch(msg.pkg, msg.launchName, msg.pkgPath);
                    break;
                case PMToHostCommand.REMOVE_LAUNCH:
                    await this._handleRemoveLaunch(msg.pkg, msg.launchName, msg.pkgPath, msg.launchPath);
                    break;
                case 'requestUiPreferences':
                    this._sendUiPreferences();
                    break;
                case 'setUiPreferences':
                    await this._setUiPreferences(msg.preferences);
                    break;
            }
        });

        this._ros.onTerminalsChanged((terminals) => {
            this._view?.webview.postMessage({
                command: PMToWebviewCommand.TERMINAL_LIST,
                terminals,
            });
        });

        webviewView.onDidChangeVisibility(() => {
            if (webviewView.visible) {
                void this._sendEnvironmentDialogState();
            }
        });

        this._applyStoredRunTerminalTarget();
        void this._initializeEnvironmentOnStartup();

        // Send the initial package list once the view is visible
        this._sendPackageList();
        this._sendBuildCheckState();
        this._sendUiPreferences();
    }

    /** Called from the "ROS: Create Package" command. */
    focusCreateForm(): void {
        this._view?.show(true);
        this._view?.webview.postMessage({ command: PMToWebviewCommand.FOCUS_CREATE });
    }

    /**
     * Refreshes the environment cache in the background.
     * Safe to call during activation when the webview is not visible yet.
     */
    warmEnvironmentCacheOnStartup(): void {
        void this._ensureEnvironmentDialogCache(true).catch(() => {
            // Keep activation non-disruptive; explicit Environment Info refresh can retry.
        });
    }

    // ── Private ────────────────────────────────────────────────
    private _sendBuildCheckState() {
        const enabled = vscode.workspace
            .getConfiguration('rosDevToolkit')
            .get<boolean>('preLaunchBuildCheck', true);
        this._view?.webview.postMessage({
            command: PMToWebviewCommand.BUILD_CHECK_STATE,
            enabled,
        });
    }

    private _getUiPreferences(): WebviewUiPreferences {
        return loadWebviewUiPreferences(this._context.globalState);
    }

    private _sendUiPreferences() {
        this.applyUiPreferences(this._getUiPreferences());
    }

    private async _setUiPreferences(preferences: unknown) {
        const normalized = await saveWebviewUiPreferences(this._context.globalState, preferences);
        if (this._onUiPreferencesChanged) {
            await this._onUiPreferencesChanged(normalized);
            return;
        }
        this.applyUiPreferences(normalized);
    }

    public applyUiPreferences(preferences: WebviewUiPreferences) {
        this._view?.webview.postMessage({
            command: 'uiPreferencesState',
            preferences,
        });
    }

    private async _handleAddNode(
        pkg: string,
        nodeName: string,
        pkgPath?: string,
        nodeTemplate?: string,
        nodeTopic?: string,
    ) {
        if (!pkg || !nodeName) {
            this._view?.webview.postMessage({
                command: PMToWebviewCommand.ADD_NODE_DONE,
                success: false,
            });
            return;
        }

        let ok = false;
        try {
            ok = await this._ros.addNodeToPackage(pkg, nodeName, pkgPath, nodeTemplate, nodeTopic);
            if (ok) {
                vscode.window.showInformationMessage(
                    `Node "${nodeName}" added to package "${pkg}".`,
                );
            }
        } catch (error) {
            const message = error instanceof Error ? error.message : String(error);
            vscode.window.showErrorMessage(`Failed to add node "${nodeName}": ${message}`);
        } finally {
            this._view?.webview.postMessage({
                command: PMToWebviewCommand.ADD_NODE_DONE,
                success: ok,
            });
        }

        if (ok) {
            await this._sendPackageList();
        }
    }

    private async _handleRemoveNode(
        pkg: string,
        nodeName: string,
        pkgPath?: string,
        nodePath?: string,
    ) {
        if (!pkg || !nodeName) {
            this._view?.webview.postMessage({
                command: PMToWebviewCommand.REMOVE_NODE_DONE,
                success: false,
            });
            return;
        }

        let ok = false;
        try {
            ok = await this._ros.removeNodeFromPackage(pkg, nodeName, pkgPath, nodePath);
            if (ok) {
                vscode.window.showInformationMessage(
                    `Node "${nodeName}" removed from package "${pkg}".`,
                );
            }
        } catch (error) {
            const message = error instanceof Error ? error.message : String(error);
            vscode.window.showErrorMessage(`Failed to remove node "${nodeName}": ${message}`);
        } finally {
            this._view?.webview.postMessage({
                command: PMToWebviewCommand.REMOVE_NODE_DONE,
                success: ok,
            });
        }

        if (ok) {
            await this._sendPackageList();
        }
    }

    private async _handleCreateLaunch(
        pkg: string,
        launchName: string,
        pkgPath?: string,
    ) {
        if (!pkg || !launchName) {
            this._view?.webview.postMessage({
                command: PMToWebviewCommand.CREATE_LAUNCH_DONE,
                success: false,
            });
            return;
        }

        let ok = false;
        try {
            ok = await this._ros.createLaunchFileInPackage(pkg, launchName, pkgPath);
            if (ok) {
                vscode.window.showInformationMessage(
                    `Launch file "${launchName}" created in package "${pkg}".`,
                );
            }
        } catch (error) {
            const message = error instanceof Error ? error.message : String(error);
            vscode.window.showErrorMessage(`Failed to create launch file "${launchName}": ${message}`);
        } finally {
            this._view?.webview.postMessage({
                command: PMToWebviewCommand.CREATE_LAUNCH_DONE,
                success: ok,
            });
        }

        if (ok) {
            await this._sendPackageList();
        }
    }

    private async _handleRemoveLaunch(
        pkg: string,
        launchName: string,
        pkgPath?: string,
        launchPath?: string,
    ) {
        if (!pkg || !launchName) {
            this._view?.webview.postMessage({
                command: PMToWebviewCommand.REMOVE_LAUNCH_DONE,
                success: false,
            });
            return;
        }

        let ok = false;
        try {
            ok = await this._ros.removeLaunchFileFromPackage(pkg, launchName, pkgPath, launchPath);
            if (ok) {
                vscode.window.showInformationMessage(
                    `Launch file "${launchName}" removed from package "${pkg}".`,
                );
            }
        } catch (error) {
            const message = error instanceof Error ? error.message : String(error);
            vscode.window.showErrorMessage(`Failed to remove launch file "${launchName}": ${message}`);
        } finally {
            this._view?.webview.postMessage({
                command: PMToWebviewCommand.REMOVE_LAUNCH_DONE,
                success: ok,
            });
        }

        if (ok) {
            await this._sendPackageList();
        }
    }

    private async _toggleBuildCheck(enabled: boolean) {
        await vscode.workspace
            .getConfiguration('rosDevToolkit')
            .update('preLaunchBuildCheck', enabled, vscode.ConfigurationTarget.Global);
    }

    private _getStoredRunTerminalTarget(): string {
        return String(this._context.globalState.get<string>(RUN_TERMINAL_TARGET_KEY, 'auto') || 'auto')
            .trim()
            || 'auto';
    }

    private _resolveRunTerminalTarget(
        requested: string,
        options: RunTerminalTargetOption[],
    ): string {
        const normalized = String(requested || '').trim();
        if (!normalized) {
            return 'auto';
        }
        const migrated = normalized === 'wsl:default'
            ? 'wsl-external:default'
            : (normalized.startsWith('wsl:') ? `wsl-external:${normalized.slice('wsl:'.length)}` : normalized);
        const validIds = new Set(options.map((opt) => String(opt.id || '').trim()).filter(Boolean));
        return validIds.has(migrated) ? migrated : 'auto';
    }

    private _normalizeRunTerminalTargetId(target: unknown): string {
        const normalized = String(target || '').trim();
        if (!normalized) {
            return 'auto';
        }
        if (normalized === 'auto' || normalized === 'integrated' || normalized === 'external') {
            return normalized;
        }
        if (normalized === 'wsl:default') {
            return 'wsl-external:default';
        }
        if (normalized.startsWith('wsl:')) {
            return `wsl-external:${normalized.slice('wsl:'.length)}`;
        }
        if (
            normalized === 'wsl-integrated:default'
            || normalized.startsWith('wsl-integrated:')
            || normalized === 'wsl-external:default'
            || normalized.startsWith('wsl-external:')
        ) {
            return normalized;
        }
        return 'auto';
    }

    private _applyStoredRunTerminalTarget() {
        const target = this._getStoredRunTerminalTarget();
        this._ros.setRunTerminalTarget(target);
        this._setViewTitle(target);
    }

    private async _initializeEnvironmentOnStartup() {
        try {
            await this._sendEnvironmentDialogState();
        } catch {
            // Keep startup non-disruptive; the user can still open Environment Info manually.
        }
    }

    private _setViewTitle(
        target: string,
        options?: RunTerminalTargetOption[],
    ) {
        if (!this._view) {
            return;
        }

        const normalizedTarget = String(target || '').trim() || 'auto';
        const option = options?.find((candidate) => candidate.id === normalizedTarget);
        const envLabel = option
            ? (option.id === 'auto' ? 'Auto' : option.label)
            : (normalizedTarget === 'auto' ? 'Auto' : normalizedTarget);
        this._view.title = `${PACKAGE_MANAGER_BASE_TITLE} - ${envLabel}`;
    }

    private _readEnvironmentDialogCache(): EnvironmentDialogCacheSnapshot | undefined {
        const raw = this._context.globalState.get<Partial<EnvironmentDialogCacheSnapshot>>(
            ENVIRONMENT_DIALOG_CACHE_KEY,
        );
        if (!raw || typeof raw !== 'object') {
            return undefined;
        }

        const report = typeof raw.report === 'string' ? raw.report : '';
        const targetOptions = Array.isArray(raw.targetOptions)
            ? raw.targetOptions
                .map((option) => ({
                    id: String(option?.id || '').trim(),
                    label: String(option?.label || '').trim(),
                    description: String(option?.description || '').trim(),
                }))
                .filter((option) => Boolean(option.id))
            : [];
        const autoLaunchInExternalTerminal = raw.autoLaunchInExternalTerminal !== false;
        const refreshedAt = Number.isFinite(raw.refreshedAt)
            ? Number(raw.refreshedAt)
            : 0;

        if (!targetOptions.length) {
            return undefined;
        }

        return {
            report,
            targetOptions,
            autoLaunchInExternalTerminal,
            refreshedAt,
        };
    }

    private async _refreshEnvironmentDialogCache(): Promise<EnvironmentDialogCacheSnapshot> {
        const [report, options] = await Promise.all([
            this._ros.getEnvironmentInfoReport(),
            this._ros.listRunTerminalTargets(),
        ]);
        const autoLaunchInExternalTerminal = vscode.workspace
            .getConfiguration('rosDevToolkit')
            .get<boolean>('launchInExternalTerminal', true);

        const snapshot: EnvironmentDialogCacheSnapshot = {
            report: String(report || ''),
            targetOptions: options,
            autoLaunchInExternalTerminal,
            refreshedAt: Date.now(),
        };
        this._environmentDialogCache = snapshot;
        await this._context.globalState.update(ENVIRONMENT_DIALOG_CACHE_KEY, snapshot);
        return snapshot;
    }

    private async _ensureEnvironmentDialogCache(
        forceRefresh: boolean = false,
    ): Promise<EnvironmentDialogCacheSnapshot | undefined> {
        if (!forceRefresh && this._environmentDialogCache) {
            return this._environmentDialogCache;
        }

        if (!forceRefresh && this._environmentDialogRefreshPromise) {
            return await this._environmentDialogRefreshPromise;
        }

        const refreshPromise = this._refreshEnvironmentDialogCache();
        this._environmentDialogRefreshPromise = refreshPromise;

        try {
            return await refreshPromise;
        } finally {
            if (this._environmentDialogRefreshPromise === refreshPromise) {
                this._environmentDialogRefreshPromise = undefined;
            }
        }
    }

    private async _sendEnvironmentDialogState(forceRefresh: boolean = false) {
        const snapshot = await this._ensureEnvironmentDialogCache(forceRefresh);
        if (!snapshot || snapshot.targetOptions.length === 0) {
            return;
        }
        const autoLaunchInExternalTerminal = vscode.workspace
            .getConfiguration('rosDevToolkit')
            .get<boolean>('launchInExternalTerminal', true);

        const storedTarget = this._getStoredRunTerminalTarget();
        const selectedTarget = this._resolveRunTerminalTarget(storedTarget, snapshot.targetOptions);
        if (storedTarget !== selectedTarget) {
            await this._context.globalState.update(RUN_TERMINAL_TARGET_KEY, selectedTarget);
        }
        this._ros.setRunTerminalTarget(selectedTarget);
        this._setViewTitle(selectedTarget, snapshot.targetOptions);

        this._view?.webview.postMessage({
            command: PMToWebviewCommand.ENVIRONMENT_DIALOG_STATE,
            report: snapshot.report,
            target: selectedTarget,
            targetOptions: snapshot.targetOptions,
            autoLaunchInExternalTerminal,
        });
    }

    private async _showEnvironmentInfo() {
        try {
            await this._sendEnvironmentDialogState(true);
        } catch (error) {
            const message = error instanceof Error ? error.message : String(error);
            vscode.window.showErrorMessage(`Failed to collect environment info: ${message}`);
        }
    }

    private async _setRunTerminalTarget(target: string) {
        const requested = String(target || '').trim() || 'auto';
        try {
            const cache = await this._ensureEnvironmentDialogCache();
            const options = cache?.targetOptions?.length
                ? cache.targetOptions
                : await this._ros.listRunTerminalTargets();
            const selected = this._resolveRunTerminalTarget(requested, options);
            await this._context.globalState.update(RUN_TERMINAL_TARGET_KEY, selected);
            this._ros.setRunTerminalTarget(selected);
            await this._sendEnvironmentDialogState();
        } catch (error) {
            const message = error instanceof Error ? error.message : String(error);
            vscode.window.showErrorMessage(`Failed to update run terminal target: ${message}`);
        }
    }

    private async _handleCreate(
        msg: { name: string; buildType: string; deps: string; license?: string; description?: string },
    ) {
        const normalizedName = String(msg.name || '').trim().replace(/\s+/g, '_');
        if (!normalizedName) {
            this._view?.webview.postMessage({ command: PMToWebviewCommand.CREATE_DONE, success: false });
            return;
        }

        const deps = msg.deps
            .split(/[\s,]+/)
            .map((d: string) => d.trim())
            .filter(Boolean);
        const license = String(msg.license || '').trim() || 'GPL-3.0';
        const description = String(msg.description || '').trim() || DEFAULT_CREATE_PACKAGE_DESCRIPTION;
        const ok = await this._ros.createPackage(normalizedName, msg.buildType, deps, license, description);
        if (ok) {
            vscode.window.showInformationMessage(`Package "${normalizedName}" created.`);
        }
        this._view?.webview.postMessage({ command: PMToWebviewCommand.CREATE_DONE, success: ok });
    }

    private async _sendPackageList() {
        this._pinnedLaunchFiles = this._context.globalState.get<string[]>(PINNED_LAUNCH_FILES_KEY, []);
        this._launchArgConfigs = await this._loadLaunchArgConfigsWithLegacyMigration();
        const packages = await this._ros.listWorkspacePackageDetails();
        this._view?.webview.postMessage({
            command: PMToWebviewCommand.PACKAGE_LIST,
            packages,
            pinned: this._pinnedLaunchFiles,
            launchArgConfigs: this._launchArgConfigs,
            terminals: this._ros.getTrackedTerminals(),
            preferredTerminalId: this._ros.getPreferredTerminalId(),
        });
    }

    private async _sendOtherPackageList(forceRefresh: boolean = false) {
        if (!forceRefresh && this._cachedOtherPackages) {
            this._view?.webview.postMessage({
                command: PMToWebviewCommand.OTHER_PACKAGE_LIST,
                packages: this._cachedOtherPackages,
            });
            return;
        }

        const [workspacePackages, allPackages] = await Promise.all([
            this._ros.listWorkspacePackages(),
            this._ros.listPackages(),
        ]);
        const workspaceSet = new Set(workspacePackages);
        const otherPackageNames = allPackages
            .filter((name) => !workspaceSet.has(name))
            .sort((a, b) => a.localeCompare(b));
        const otherPackageSet = new Set(otherPackageNames);
        for (const name of this._cachedOtherPackageDetails.keys()) {
            if (!otherPackageSet.has(name)) {
                this._cachedOtherPackageDetails.delete(name);
            }
        }

        this._cachedOtherPackages = otherPackageNames;
        this._view?.webview.postMessage({
            command: PMToWebviewCommand.OTHER_PACKAGE_LIST,
            packages: otherPackageNames,
        });
    }

    private async _sendOtherPackageDetails(name: string) {
        const packageName = String(name || '').trim();
        if (!packageName) {
            return;
        }

        let detail = this._cachedOtherPackageDetails.get(packageName);
        if (!detail) {
            const details = await this._ros.listPackageDetails([packageName]);
            detail = details[0] || {
                name: packageName,
                packagePath: '',
                launchFiles: [],
                nodes: [],
                isPython: false,
                isCmake: false,
            };
            this._cachedOtherPackageDetails.set(packageName, detail);
        }

        this._view?.webview.postMessage({
            command: PMToWebviewCommand.OTHER_PACKAGE_DETAILS,
            package: detail,
        });
    }

    private async _loadLaunchArgConfigsWithLegacyMigration(): Promise<LaunchArgConfigMap> {
        const launchArgConfigs = this._context.globalState.get<LaunchArgConfigMap>(LAUNCH_ARG_CONFIGS_KEY, {});
        const legacyArgs = this._context.globalState.get<Record<string, string>>(LEGACY_LAUNCH_ARGS_KEY, {});

        let migrated = false;
        for (const [path, args] of Object.entries(legacyArgs)) {
            if (!launchArgConfigs[path]) {
                launchArgConfigs[path] = {
                    configs: [{ id: 'default', name: 'default', args, runTarget: 'auto' }],
                };
                migrated = true;
            }
        }

        for (const [argsKey, configSet] of Object.entries(launchArgConfigs)) {
            const existing = Array.isArray(configSet?.configs) ? configSet.configs : [];
            if (!existing.length) {
                launchArgConfigs[argsKey] = {
                    configs: [{ id: 'default', name: 'default', args: '', runTarget: 'auto' }],
                };
                migrated = true;
                continue;
            }

            const normalized = existing.map((config, index) => {
                const id = String(config?.id || `cfg-${index + 1}`).trim() || `cfg-${index + 1}`;
                const name = String(config?.name || '').trim() || (id === 'default' ? 'default' : 'config');
                const args = String(config?.args || '').trim();
                const runTarget = this._normalizeRunTerminalTargetId(config?.runTarget);
                return { id, name, args, runTarget };
            });
            if (JSON.stringify(normalized) !== JSON.stringify(existing)) {
                launchArgConfigs[argsKey] = { configs: normalized };
                migrated = true;
            }
        }

        if (migrated) {
            await this._context.globalState.update(LAUNCH_ARG_CONFIGS_KEY, launchArgConfigs);
        }

        return launchArgConfigs;
    }

    private async _openLaunchFile(filePath: string) {
        await this._openWorkspaceFile(filePath, 'Launch file');
    }

    private async _openNodeFile(filePath?: string) {
        if (!filePath) {
            vscode.window.showWarningMessage('No source file detected for this node.');
            return;
        }
        await this._openWorkspaceFile(filePath, 'Node source');
    }

    private async _openWorkspaceFile(filePath: string, label: string) {
        if (!filePath) {
            return;
        }

        const uri = vscode.Uri.file(filePath);
        try {
            await vscode.workspace.fs.stat(uri);
        } catch {
            vscode.window.showErrorMessage(`${label} not found.`);
            return;
        }

        await vscode.window.showTextDocument(uri, { preview: false });
    }

    private async _launchFile(
        pkg: string,
        file: string,
        path: string,
        argsOverride?: string,
        argsNameOverride?: string,
        runTargetOverride?: string,
    ) {
        if (!pkg || !file) {
            return;
        }
        const args = argsOverride !== undefined
            ? argsOverride
            : undefined;

        const argsLabel = argsNameOverride ?? undefined;
        const normalizedRunTarget = this._normalizeRunTerminalTargetId(runTargetOverride);
        const effectiveRunTarget = normalizedRunTarget !== 'auto' ? normalizedRunTarget : undefined;

        if (effectiveRunTarget) {
            this._ros.launchFile(pkg, file, args, path, argsLabel, effectiveRunTarget);
            return;
        }
        this._ros.launchFile(pkg, file, args, path, argsLabel);
    }

    private async _runNode(
        pkg: string,
        executable: string,
        argsOverride?: string,
        argsNameOverride?: string,
        sourcePath?: string,
        runTargetOverride?: string,
    ) {
        if (!pkg || !executable) {
            return;
        }
        const args = argsOverride !== undefined
            ? argsOverride
            : '';
        const argsLabel = argsNameOverride?.trim() ? argsNameOverride : undefined;
        const nodePath = sourcePath?.trim() ? sourcePath : undefined;
        const normalizedRunTarget = this._normalizeRunTerminalTargetId(runTargetOverride);
        const effectiveRunTarget = normalizedRunTarget !== 'auto' ? normalizedRunTarget : undefined;
        if (effectiveRunTarget) {
            this._ros.runNode(pkg, executable, args, nodePath, argsLabel, effectiveRunTarget);
            return;
        }
        this._ros.runNode(pkg, executable, args, nodePath, argsLabel);
    }

    private async _killTerminal(id: string) {
        if (!id) {
            return;
        }
        await this._ros.killTrackedTerminal(id);
    }

    private async _focusTerminal(id: string) {
        if (!id) {
            return;
        }
        this._ros.focusTrackedTerminal(id);
    }

    private async _relaunchTerminal(id: string) {
        if (!id) {
            return;
        }
        await this._ros.relaunchTrackedTerminal(id);
    }

    private async _setPreferredTerminal(id: string) {
        this._ros.setPreferredTerminal(id);
    }

    private async _togglePin(path: string) {
        if (!path) {
            return;
        }

        const pinned = this._context.globalState.get<string[]>(PINNED_LAUNCH_FILES_KEY, []);
        const next = pinned.includes(path)
            ? pinned.filter((p) => p !== path)
            : [...pinned, path];

        await this._context.globalState.update(PINNED_LAUNCH_FILES_KEY, next);
        await this._sendPackageList();
    }

    private async _setLaunchArgConfigs(
        path: string,
        configs: LaunchArgConfig[],
    ) {
        if (!path) {
            return;
        }

        const current = this._context.globalState.get<LaunchArgConfigMap>(LAUNCH_ARG_CONFIGS_KEY, {});
        current[path] = {
            configs: configs.map((c) => ({
                id: c.id,
                name: c.name?.trim() || 'config',
                args: c.args?.trim() || '',
                runTarget: this._normalizeRunTerminalTargetId(c.runTarget),
            })),
        };

        await this._context.globalState.update(LAUNCH_ARG_CONFIGS_KEY, current);
        await this._sendPackageList();
    }


    private async _sendLaunchArgOptions(argsKey: string, sourcePath?: string) {
        if (!argsKey) {
            return;
        }

        const options = sourcePath
            ? this._ros.getLaunchArgOptions(sourcePath)
            : [];
        this._view?.webview.postMessage({
            command: PMToWebviewCommand.LAUNCH_ARGS_OPTIONS,
            argsKey,
            options,
        });
    }

    private _getHtml(webview: vscode.Webview): string {
        const body = /* html */ `
<div class="section">
    <div class="toolbar">
        <button id="btnOpenCreate">＋ New Package</button>
        <button class="secondary small" id="btnRefresh">↻ Refresh</button>
        <button class="secondary small" id="btnShowEnvironment">Environment Info</button>
    </div>

    <div class="toggle-row">
        <label for="toggleBuildCheck">⚡ Auto build check before launch</label>
        <span class="toggle-switch">
            <input type="checkbox" id="toggleBuildCheck" checked />
            <span class="toggle-slider"></span>
        </span>
    </div>

    <div class="search-row">
        <input type="text" id="pkgFilter" placeholder="Filter packages, launch files, or nodes…" />
        <span id="pkgCount" class="badge info">0</span>
    </div>

    <div id="pkgStatus" class="text-muted text-sm mb"></div>

    <div class="subsection">
        <div class="section-header">
            <div class="section-header-main">
                <button class="secondary small" id="btnToggleTerminalList" title="Collapse/expand active terminal list">▾</button>
                <h3 id="lblToggleTerminalList" class="section-toggle-label" tabindex="0" title="Collapse/expand active terminal list">Active Terminals</h3>
            </div>
            <div class="section-header-actions">
                <button class="secondary small" id="btnNewRosTerminal">New ROS Terminal</button>
            </div>
        </div>
        <ul class="item-list" id="terminalList">
            <li class="text-muted">No active terminals</li>
        </ul>
    </div>

    <div class="subsection">
        <h3>Pinned Items</h3>
        <ul class="item-list" id="pinnedList">
            <li class="text-muted">No pinned items</li>
        </ul>
    </div>

    <div class="subsection">
        <div class="section-header">
            <div class="section-header-main">
                <button class="secondary small" id="btnToggleWorkspacePackages" title="Collapse/expand workspace package list (Alt+click: expand/collapse all nested items)">▾</button>
                <h3 id="lblToggleWorkspacePackages" class="section-toggle-label" tabindex="0" title="Collapse/expand workspace package list (Alt+click: expand/collapse all nested items)">Workspace Packages</h3>
            </div>
        </div>
        <ul class="item-list" id="pkgList">
            <li class="text-muted">Loading…</li>
        </ul>
    </div>

    <div class="subsection">
        <div class="section-header">
            <div class="section-header-main">
                <button class="secondary small" id="btnToggleOtherPackages" title="Collapse/expand other ROS package list (Alt+click: expand/collapse all nested items)">▾</button>
                <h3 id="lblToggleOtherPackages" class="section-toggle-label" tabindex="0" title="Collapse/expand other ROS package list (Alt+click: expand/collapse all nested items)">Other ROS Packages</h3>
            </div>
            <div class="section-header-actions">
                <button class="secondary small" id="btnLoadOtherPackages">Load</button>
            </div>
        </div>
        <ul class="item-list" id="otherPkgList">
            <li class="text-muted">Not loaded</li>
        </ul>
    </div>
</div>

<div class="modal hidden" id="createModal" role="dialog" aria-modal="true">
    <div class="modal-backdrop" id="createBackdrop"></div>
    <div class="modal-card">
        <div class="modal-header">
            <h3>Create New Package</h3>
            <button class="secondary small" id="btnCloseCreate">✕</button>
        </div>
        <div class="modal-body">
            <label for="pkgName">Package name</label>
            <input type="text" id="pkgName" placeholder="my_robot_pkg" />
            <div id="pkgNameNormalizedHint" class="normalized-hint text-muted text-sm hidden"></div>

            <label for="buildType">Build type</label>
            <select id="buildType">
                <option value="ament_cmake">ament_cmake (C++)</option>
                <option value="ament_python">ament_python (Python)</option>
            </select>

            <label for="pkgLicense">License</label>
            <select id="pkgLicense">
                <option value="GPL-3.0" selected>GPL-3.0 (default)</option>
                <option value="Apache-2.0">Apache-2.0</option>
                <option value="MIT">MIT</option>
                <option value="BSD-3-Clause">BSD-3-Clause</option>
            </select>

            <label for="pkgDescription">Description</label>
            <textarea id="pkgDescription" rows="3" spellcheck="false">${DEFAULT_CREATE_PACKAGE_DESCRIPTION}</textarea>

            <label for="deps">Dependencies <span class="text-muted text-sm">(space / comma separated)</span></label>
            <input type="text" id="deps" placeholder="auto: rclcpp std_msgs / rclpy std_msgs" />
        </div>
        <div class="modal-footer">
            <button class="secondary" id="btnCancelCreate">Cancel</button>
            <button id="btnCreate">Create Package</button>
        </div>
        <div id="createStatus" class="mt hidden"></div>
    </div>
</div>

<div class="modal hidden" id="argsModal" role="dialog" aria-modal="true">
    <div class="modal-backdrop" id="argsBackdrop"></div>
    <div class="modal-card">
        <div class="modal-header">
            <h3>Launch file arguments</h3>
            <button class="secondary small" id="btnCloseArgs">✕</button>
        </div>
        <div class="modal-body">
                    <div class="config-row">
                        <label>Configurations</label>
                        <div class="config-actions">
                            <button class="secondary small" id="btnAddConfig">＋ Add</button>
                            <button class="secondary small" id="btnRemoveConfig">－ Remove</button>
                        </div>
                    </div>
                    <div class="config-list" id="configList"></div>

                    <label for="configName">Config name</label>
                    <input type="text" id="configName" placeholder="default" />

                    <label for="configRunTarget">Run terminal</label>
                    <select id="configRunTarget"></select>
                    <div id="configRunTargetDescription" class="text-muted text-sm mb"></div>

            <label for="argsInput">Arguments</label>
            <input type="text" id="argsInput" placeholder="use_sim_time:=true namespace:=robot1" />

            <label>Available arguments</label>
            <ul class="arg-list" id="argsList">
                <li class="text-muted">No arguments detected</li>
            </ul>
            <div class="btn-row mt">
                <button class="secondary small" id="btnInsertAll">Insert all defaults</button>
            </div>
        </div>
        <div class="modal-footer">
            <button class="secondary" id="btnCancelArgs">Cancel</button>
            <button id="btnSaveArgs">Save</button>
        </div>
    </div>
</div>

<div class="modal hidden" id="addNodeModal" role="dialog" aria-modal="true">
    <div class="modal-backdrop" id="addNodeBackdrop"></div>
    <div class="modal-card">
        <div class="modal-header">
            <h3>Add Node to Package</h3>
            <button class="secondary small" id="btnCloseAddNode">✕</button>
        </div>
        <div class="modal-body">
            <label for="addNodePkg">Package</label>
            <input type="text" id="addNodePkg" readonly />

            <label for="addNodeName">Node name</label>
            <input type="text" id="addNodeName" placeholder="my_node" />
            <div id="addNodeNameNormalizedHint" class="normalized-hint text-muted text-sm hidden"></div>

            <label for="addNodeTemplate">Node template</label>
            <select id="addNodeTemplate">
                <option value="none" selected>None</option>
                <option value="publisher">Publisher</option>
                <option value="subscriber">Subscriber</option>
                <option value="service">Service</option>
                <option value="client">Client</option>
                <option value="timer">Timer</option>
            </select>

            <div id="addNodeTopicRow" class="hidden">
                <label for="addNodeTopic">Topic</label>
                <input type="text" id="addNodeTopic" placeholder="chatter" />
                <div id="addNodeTopicNormalizedHint" class="normalized-hint text-muted text-sm hidden"></div>
            </div>
        </div>
        <div class="modal-footer">
            <button class="secondary" id="btnCancelAddNode">Cancel</button>
            <button id="btnAddNode">Add Node</button>
        </div>
        <div id="addNodeStatus" class="mt hidden"></div>
    </div>
</div>

<div class="modal hidden" id="createLaunchModal" role="dialog" aria-modal="true">
    <div class="modal-backdrop" id="createLaunchBackdrop"></div>
    <div class="modal-card">
        <div class="modal-header">
            <h3>Create Launch File</h3>
            <button class="secondary small" id="btnCloseCreateLaunch">✕</button>
        </div>
        <div class="modal-body">
            <label for="createLaunchPkg">Package</label>
            <input type="text" id="createLaunchPkg" readonly />

            <label for="createLaunchName">Launch file name</label>
            <input type="text" id="createLaunchName" placeholder="bringup.launch.py" />
            <div id="createLaunchNameNormalizedHint" class="normalized-hint text-muted text-sm hidden"></div>

            <div class="text-muted text-sm">
                If no extension is provided, <code>.launch.py</code> is appended automatically.
            </div>
        </div>
        <div class="modal-footer">
            <button class="secondary" id="btnCancelCreateLaunch">Cancel</button>
            <button id="btnCreateLaunch">Create Launch</button>
        </div>
        <div id="createLaunchStatus" class="mt hidden"></div>
    </div>
</div>

<div class="modal hidden" id="removeLaunchModal" role="dialog" aria-modal="true">
    <div class="modal-backdrop" id="removeLaunchBackdrop"></div>
    <div class="modal-card">
        <div class="modal-header">
            <h3>Remove Launch File</h3>
            <button class="secondary small" id="btnCloseRemoveLaunch">✕</button>
        </div>
        <div class="modal-body">
            <label for="removeLaunchPkg">Package</label>
            <input type="text" id="removeLaunchPkg" readonly />

            <label for="removeLaunchName">Launch file</label>
            <input type="text" id="removeLaunchName" readonly />

            <label for="removeLaunchPath">Path</label>
            <input type="text" id="removeLaunchPath" readonly />

            <div class="text-muted text-sm">
                Removes the selected launch file from the package launch directory.
            </div>
        </div>
        <div class="modal-footer">
            <button class="secondary" id="btnCancelRemoveLaunch">Cancel</button>
            <button class="danger" id="btnRemoveLaunch">Remove Launch</button>
        </div>
        <div id="removeLaunchStatus" class="mt hidden"></div>
    </div>
</div>

<div class="modal hidden" id="removeNodeModal" role="dialog" aria-modal="true">
    <div class="modal-backdrop" id="removeNodeBackdrop"></div>
    <div class="modal-card">
        <div class="modal-header">
            <h3>Remove Node</h3>
            <button class="secondary small" id="btnCloseRemoveNode">✕</button>
        </div>
        <div class="modal-body">
            <label for="removeNodePkg">Package</label>
            <input type="text" id="removeNodePkg" readonly />

            <label for="removeNodeName">Node name</label>
            <input type="text" id="removeNodeName" readonly />

            <label for="removeNodePath">Source path</label>
            <input type="text" id="removeNodePath" readonly />

            <div class="text-muted text-sm">
                Removes the node source file when found and unregisters it from setup/CMake files.
            </div>
        </div>
        <div class="modal-footer">
            <button class="secondary" id="btnCancelRemoveNode">Cancel</button>
            <button class="danger" id="btnRemoveNode">Remove Node</button>
        </div>
        <div id="removeNodeStatus" class="mt hidden"></div>
    </div>
</div>

<div class="modal hidden" id="environmentModal" role="dialog" aria-modal="true">
    <div class="modal-backdrop" id="environmentBackdrop"></div>
    <div class="modal-card">
        <div class="modal-header">
            <h3>Environment & Run Terminal</h3>
            <button class="secondary small" id="btnCloseEnvironment">✕</button>
        </div>
        <div class="modal-body">
            <label for="runEnvironmentTarget">Run button environment</label>
            <select id="runEnvironmentTarget"></select>
            <div id="runEnvironmentDescription" class="text-muted text-sm mb"></div>

            <label for="runTerminalMode">Run button terminal</label>
            <select id="runTerminalMode"></select>
            <div id="runTerminalModeDescription" class="text-muted text-sm mb"></div>

            <div id="environmentLoadingState" class="text-muted text-sm mb hidden">
                <span class="spinner"></span> Detecting environment...
            </div>

            <button class="secondary small env-details-toggle" id="environmentDetailsToggle" aria-expanded="false">
                ▶ Detected environment
            </button>
            <div id="environmentDetailsContainer" class="hidden">
                <pre id="environmentDetails" class="env-report">Collecting environment details...</pre>
            </div>
        </div>
        <div class="modal-footer">
            <button class="secondary" id="btnCancelEnvironment">Close</button>
            <button id="btnSaveEnvironment">Save target</button>
        </div>
    </div>
</div>
`;
        const scriptUris = [
            vscode.Uri.joinPath(this._extensionUri, 'media', 'shared', 'interactions.js'),
            vscode.Uri.joinPath(this._extensionUri, 'media', 'shared', 'uiPreferences.js'),
            vscode.Uri.joinPath(this._extensionUri, 'media', 'packageManager', 'state.js'),
            vscode.Uri.joinPath(this._extensionUri, 'media', 'packageManager', 'dom.js'),
            vscode.Uri.joinPath(this._extensionUri, 'media', 'packageManager', 'messages.js'),
            vscode.Uri.joinPath(this._extensionUri, 'media', 'packageManager', 'actions.js'),
            vscode.Uri.joinPath(this._extensionUri, 'media', 'packageManager', 'render.js'),
            vscode.Uri.joinPath(this._extensionUri, 'media', 'packageManager', 'events.js'),
            vscode.Uri.joinPath(this._extensionUri, 'media', 'packageManager', 'index.js'),
        ];
        return getWebviewHtml(webview, this._extensionUri, body, '', undefined, scriptUris);
    }
}
