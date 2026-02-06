import path from 'node:path';

type Listener<T> = (event: T) => void;

class Disposable {
    constructor(private readonly _dispose: () => void = () => {}) {}

    dispose(): void {
        this._dispose();
    }
}

export class EventEmitter<T> {
    private readonly listeners = new Set<Listener<T>>();

    readonly event = (listener: Listener<T>): Disposable => {
        this.listeners.add(listener);
        return new Disposable(() => this.listeners.delete(listener));
    };

    fire(data: T): void {
        for (const listener of this.listeners) {
            listener(data);
        }
    }

    dispose(): void {
        this.listeners.clear();
    }
}

type MockTerminalOptions = { name?: string; shellPath?: string; env?: Record<string, string> };

class MockTerminal {
    readonly name: string;
    readonly sentTexts: string[] = [];
    exitStatus: { code: number } | undefined;

    constructor(name: string) {
        this.name = name;
    }

    show(): void {}

    sendText(text: string): void {
        this.sentTexts.push(text);
    }
}

export class Uri {
    readonly fsPath: string;

    constructor(fsPath: string) {
        this.fsPath = fsPath;
    }

    static file(fsPath: string): Uri {
        return new Uri(fsPath);
    }

    static joinPath(base: Uri, ...parts: string[]): Uri {
        return new Uri(path.join(base.fsPath, ...parts));
    }
}

export enum ConfigurationTarget {
    Global = 1,
    Workspace = 2,
    WorkspaceFolder = 3,
}

type ConfigurationStore = Record<string, Record<string, unknown>>;

const closeTerminalEmitter = new EventEmitter<MockTerminal>();
const createdTerminals: MockTerminal[] = [];
const configuration: ConfigurationStore = {
    rosDevToolkit: {
        rosSetupPath: '',
        launchInExternalTerminal: true,
    },
    cmake: {},
};

const infoMessages: string[] = [];
const warningMessages: string[] = [];
const errorMessages: string[] = [];

let workspaceFolders: Array<{ uri: Uri }> = [{ uri: Uri.file(process.cwd()) }];

let warningMessageResponse: string | undefined = undefined;

export const window = {
    onDidCloseTerminal: closeTerminalEmitter.event,
    createTerminal(options?: string | MockTerminalOptions): MockTerminal {
        const name = typeof options === 'string'
            ? options
            : options?.name || 'Terminal';
        const terminal = new MockTerminal(name);
        createdTerminals.push(terminal);
        return terminal;
    },
    showInformationMessage(message: string): Promise<undefined> {
        infoMessages.push(message);
        return Promise.resolve(undefined);
    },
    showWarningMessage(message: string, ..._args: unknown[]): Promise<string | undefined> {
        warningMessages.push(message);
        return Promise.resolve(warningMessageResponse);
    },
    showErrorMessage(message: string): Promise<undefined> {
        errorMessages.push(message);
        return Promise.resolve(undefined);
    },
    showTextDocument(): Promise<undefined> {
        return Promise.resolve(undefined);
    },
    setStatusBarMessage(): Disposable {
        return new Disposable();
    },
    registerWebviewViewProvider(): Disposable {
        return new Disposable();
    },
};

export const workspace = {
    get workspaceFolders(): Array<{ uri: Uri }> {
        return workspaceFolders;
    },
    set workspaceFolders(next: Array<{ uri: Uri }>) {
        workspaceFolders = next;
    },
    getConfiguration(section?: string) {
        const key = section || '';
        if (!configuration[key]) {
            configuration[key] = {};
        }

        return {
            get<T>(name: string, defaultValue?: T): T {
                const value = configuration[key][name];
                return (value === undefined ? defaultValue : value) as T;
            },
            update(name: string, value: unknown): Promise<void> {
                configuration[key][name] = value;
                return Promise.resolve();
            },
        };
    },
};

export const commands = {
    registerCommand(): Disposable {
        return new Disposable();
    },
};

export function __setWorkspaceFolder(fsPath: string): void {
    workspaceFolders = [{ uri: Uri.file(fsPath) }];
}

export function __setConfiguration(section: string, key: string, value: unknown): void {
    if (!configuration[section]) {
        configuration[section] = {};
    }
    configuration[section][key] = value;
}

export function __getMessages(): { info: string[]; warn: string[]; error: string[] } {
    return {
        info: [...infoMessages],
        warn: [...warningMessages],
        error: [...errorMessages],
    };
}

export function __closeTerminal(terminal: MockTerminal): void {
    closeTerminalEmitter.fire(terminal);
}

export function __getCreatedTerminals(): MockTerminal[] {
    return [...createdTerminals];
}

export function __setWarningMessageResponse(response: string | undefined): void {
    warningMessageResponse = response;
}

export function __resetMockState(): void {
    workspaceFolders = [{ uri: Uri.file(process.cwd()) }];
    createdTerminals.length = 0;
    infoMessages.length = 0;
    warningMessages.length = 0;
    errorMessages.length = 0;
    warningMessageResponse = undefined;

    configuration.rosDevToolkit = {
        rosSetupPath: '',
        launchInExternalTerminal: true,
    };
    configuration.cmake = {};
}
