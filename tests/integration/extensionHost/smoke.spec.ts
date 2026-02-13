import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import {
    __resetMockState,
    Uri,
    commands,
    window,
    workspace,
} from '../../helpers/mocks/vscode';
import { activate } from '../../../src/extension';
import { RosWorkspace } from '../../../src/ros/rosWorkspace';
import { PackageManagerViewProvider } from '../../../src/views/packageManagerView';
import { NodeVisualizerViewProvider } from '../../../src/views/nodeVisualizerView';

vi.mock('vscode', () => import('../../helpers/mocks/vscode'));

type MockMemento = {
    get: <T>(key: string, defaultValue?: T) => T;
    update: (key: string, value: unknown) => Promise<void>;
};

function createMockMemento(): MockMemento {
    const values = new Map<string, unknown>();
    return {
        get: <T>(key: string, defaultValue?: T): T => {
            return (values.has(key) ? values.get(key) : defaultValue) as T;
        },
        update: async (key: string, value: unknown): Promise<void> => {
            values.set(key, value);
        },
    };
}

describe('extension activation wiring', () => {
    beforeEach(() => {
        __resetMockState();
    });

    afterEach(() => {
        vi.restoreAllMocks();
        __resetMockState();
    });

    it('registers providers/commands and routes command callbacks to providers', async () => {
        const commandHandlers = new Map<string, () => void>();

        const registerCommandSpy = vi.spyOn(commands, 'registerCommand').mockImplementation(((id: string, cb: () => void) => {
            commandHandlers.set(id, cb);
            return { dispose: () => {} };
        }) as unknown as typeof commands.registerCommand);

        const registerWebviewSpy = vi.spyOn(window, 'registerWebviewViewProvider').mockImplementation((() => {
            return { dispose: () => {} };
        }) as unknown as typeof window.registerWebviewViewProvider);

        const initSmartBuildSpy = vi.spyOn(RosWorkspace.prototype, 'initSmartBuild').mockImplementation(() => {});
        const detectEnvironmentSpy = vi.spyOn(RosWorkspace.prototype, 'detectEnvironment').mockResolvedValue(undefined);

        const focusCreateSpy = vi.spyOn(PackageManagerViewProvider.prototype, 'focusCreateForm').mockImplementation(() => {});
        const openSourcedTerminalSpy = vi.spyOn(RosWorkspace.prototype, 'openSourcedTerminal').mockImplementation(() => {});
        const refreshGraphSpy = vi.spyOn(NodeVisualizerViewProvider.prototype, 'refreshGraph').mockImplementation(() => {});

        const context = {
            extensionUri: Uri.file('/tmp/ros-dev-toolkit-ext'),
            workspaceState: createMockMemento(),
            globalState: createMockMemento(),
            subscriptions: [] as Array<{ dispose: () => void }>,
        };

        activate(context as unknown as Parameters<typeof activate>[0]);

        expect(initSmartBuildSpy).toHaveBeenCalledWith(context);
        expect(detectEnvironmentSpy).toHaveBeenCalledTimes(1);

        expect(registerWebviewSpy).toHaveBeenCalledTimes(2);
        expect(registerWebviewSpy.mock.calls.map((call) => call[0])).toEqual([
            'rosPackageManager',
            'rosNodeVisualizer',
        ]);

        expect(registerCommandSpy).toHaveBeenCalledTimes(3);
        expect(Array.from(commandHandlers.keys()).sort()).toEqual([
            'rosDevToolkit.createPackage',
            'rosDevToolkit.openSourcedTerminal',
            'rosDevToolkit.refreshGraph',
        ]);

        expect(context.subscriptions).toHaveLength(5);

        commandHandlers.get('rosDevToolkit.createPackage')?.();
        commandHandlers.get('rosDevToolkit.openSourcedTerminal')?.();
        commandHandlers.get('rosDevToolkit.refreshGraph')?.();

        expect(focusCreateSpy).toHaveBeenCalledTimes(1);
        expect(openSourcedTerminalSpy).toHaveBeenCalledTimes(1);
        expect(refreshGraphSpy).toHaveBeenCalledTimes(1);

        const cmakeConfig = workspace.getConfiguration('cmake');
        expect(cmakeConfig.get('configureOnOpen')).toBe(false);
        expect(cmakeConfig.get('configureOnEdit')).toBe(false);
    });
});
