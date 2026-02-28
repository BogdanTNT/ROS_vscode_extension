import { EventEmitter } from 'node:events';
import * as cp from 'child_process';
import { afterEach, describe, expect, it, vi } from 'vitest';
import { WslPersistentGraphRunner } from '../../../src/ros/runtime/wslPersistentGraphRunner';

vi.mock('child_process', async () => {
    const actual = await vi.importActual<typeof import('child_process')>('child_process');
    return {
        ...actual,
        spawn: vi.fn(),
    };
});

type MockRunnerResponse = {
    status: number;
    output?: string;
};

type MockChildWithControls = cp.ChildProcessWithoutNullStreams & {
    emitClose: (code?: number | null, signal?: NodeJS.Signals | null) => void;
    responses: MockRunnerResponse[];
    kill: ReturnType<typeof vi.fn>;
};

function createMockChild(initialResponses: MockRunnerResponse[]): MockChildWithControls {
    const childEvents = new EventEmitter();
    const stdout = new EventEmitter() as unknown as cp.ChildProcessWithoutNullStreams['stdout'];
    const stderr = new EventEmitter() as unknown as cp.ChildProcessWithoutNullStreams['stderr'];
    (stdout as unknown as { setEncoding: (encoding: BufferEncoding) => void }).setEncoding = () => {};
    (stderr as unknown as { setEncoding: (encoding: BufferEncoding) => void }).setEncoding = () => {};

    const responses = [...initialResponses];
    const child = {
        stdout,
        stderr,
        stdin: {
            write: (chunk: string) => {
                const beginMatch = chunk.match(/printf '%s\\n' '(__RDT_PERSIST_BEGIN__:[^']+)'/);
                const endMatch = chunk.match(/printf '%s\\n' '(__RDT_PERSIST_END__:[^']+:)'/);
                const beginToken = beginMatch?.[1] || '__RDT_PERSIST_BEGIN__:unknown';
                const endPrefix = endMatch?.[1] || '__RDT_PERSIST_END__:unknown:';
                const next = responses.shift() || { status: 0, output: '' };
                setTimeout(() => {
                    (stdout as unknown as EventEmitter).emit('data', `${beginToken}\n`);
                    if (next.output) {
                        (stdout as unknown as EventEmitter).emit('data', `${next.output}\n`);
                    }
                    (stdout as unknown as EventEmitter).emit('data', `${endPrefix}${next.status}\n`);
                }, 0);
            },
            end: vi.fn(),
        },
        on: childEvents.on.bind(childEvents),
        removeAllListeners: childEvents.removeAllListeners.bind(childEvents),
        killed: false,
        kill: vi.fn((signal?: NodeJS.Signals) => {
            child.killed = true;
            childEvents.emit('close', null, signal ?? 'SIGTERM');
            return true;
        }),
        emitClose: (code: number | null = 0, signal: NodeJS.Signals | null = null) => {
            childEvents.emit('close', code, signal);
        },
        responses,
    } as unknown as MockChildWithControls;

    return child;
}

describe('WslPersistentGraphRunner', () => {
    afterEach(() => {
        vi.clearAllMocks();
        vi.restoreAllMocks();
    });

    it('executes queued commands in a single warm shell process', async () => {
        const child = createMockChild([
            { status: 0, output: 'first-output' },
            { status: 0, output: 'second-output' },
        ]);
        const spawnSpy = vi.mocked(cp.spawn).mockReturnValue(child);
        const runner = new WslPersistentGraphRunner({ distro: 'Ubuntu', idleTimeoutMs: 5000 });

        const [first, second] = await Promise.all([
            runner.exec('echo first'),
            runner.exec('echo second'),
        ]);

        expect(spawnSpy).toHaveBeenCalledTimes(1);
        expect(first.trim()).toBe('first-output');
        expect(second.trim()).toBe('second-output');

        runner.dispose();
    });

    it('restarts the shell after an unexpected close', async () => {
        const firstChild = createMockChild([{ status: 0, output: 'first-pass' }]);
        const secondChild = createMockChild([{ status: 0, output: 'second-pass' }]);
        const spawnSpy = vi.mocked(cp.spawn)
            .mockReturnValueOnce(firstChild)
            .mockReturnValueOnce(secondChild);
        const runner = new WslPersistentGraphRunner({ idleTimeoutMs: 5000 });

        const first = await runner.exec('echo first');
        expect(first.trim()).toBe('first-pass');

        firstChild.emitClose(1, null);

        const second = await runner.exec('echo second');
        expect(second.trim()).toBe('second-pass');
        expect(spawnSpy).toHaveBeenCalledTimes(2);

        runner.dispose();
    });

});
