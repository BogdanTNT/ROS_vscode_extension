import * as cp from 'child_process';

export interface WslPersistentGraphRunnerOptions {
    distro?: string;
    idleTimeoutMs?: number;
}

type PendingRequest = {
    id: string;
    command: string;
    timeoutMs: number;
    resolve: (stdout: string) => void;
    reject: (reason: Error) => void;
    started: boolean;
    lines: string[];
    timer?: NodeJS.Timeout;
};

const DEFAULT_IDLE_TIMEOUT_MS = 90000;
const DEFAULT_EXEC_TIMEOUT_MS = 30000;

/**
 * Keeps a warm WSL bash process to execute sequential graph snapshot commands.
 * Requests are framed with unique markers so stdout can be demultiplexed.
 */
export class WslPersistentGraphRunner {
    private _disposed = false;
    private _child?: cp.ChildProcessWithoutNullStreams;
    private _stdoutRemainder = '';
    private _queue: PendingRequest[] = [];
    private _pending?: PendingRequest;
    private _seq = 0;
    private _idleTimer?: NodeJS.Timeout;

    constructor(private readonly _options: WslPersistentGraphRunnerOptions = {}) {}

    async exec(command: string, timeoutMs: number = DEFAULT_EXEC_TIMEOUT_MS): Promise<string> {
        if (this._disposed) {
            throw new Error('WSL persistent graph runner is disposed.');
        }

        const normalizedTimeout = Number.isFinite(timeoutMs) && timeoutMs > 0
            ? Math.floor(timeoutMs)
            : DEFAULT_EXEC_TIMEOUT_MS;

        return new Promise<string>((resolve, reject) => {
            const request: PendingRequest = {
                id: `rdt_${Date.now()}_${++this._seq}`,
                command: String(command || ''),
                timeoutMs: normalizedTimeout,
                resolve,
                reject,
                started: false,
                lines: [],
            };
            this._queue.push(request);
            this._drainQueue();
        });
    }

    dispose(): void {
        if (this._disposed) {
            return;
        }
        this._disposed = true;
        this._clearIdleTimer();

        if (this._pending) {
            this._clearPendingTimer(this._pending);
            this._pending.reject(new Error('WSL persistent graph runner disposed while request was in flight.'));
            this._pending = undefined;
        }

        while (this._queue.length > 0) {
            const queued = this._queue.shift();
            queued?.reject(new Error('WSL persistent graph runner disposed before request execution.'));
        }

        this._destroyChild();
    }

    private _drainQueue() {
        if (this._disposed || this._pending || this._queue.length === 0) {
            this._scheduleIdleShutdownIfNeeded();
            return;
        }

        const next = this._queue.shift();
        if (!next) {
            this._scheduleIdleShutdownIfNeeded();
            return;
        }

        this._clearIdleTimer();

        try {
            const child = this._ensureChild();
            const beginToken = this._beginToken(next.id);
            const endPrefix = this._endPrefix(next.id);

            const wrappedScript = [
                `printf '%s\\n' '${beginToken}'`,
                `( ${next.command} ) 2>&1`,
                '__rdt_status=$?',
                `printf '%s\\n' '${endPrefix}'"$__rdt_status"`,
            ].join('\n');

            this._pending = next;
            next.timer = setTimeout(() => {
                if (this._pending !== next) {
                    return;
                }
                this._failCurrentRequest(
                    new Error(`Persistent WSL graph command timed out after ${next.timeoutMs}ms.`),
                );
            }, next.timeoutMs);
            next.timer.unref();

            child.stdin.write(`${wrappedScript}\n`);
        } catch (err) {
            next.reject(
                new Error(
                    err instanceof Error
                        ? err.message
                        : 'Failed to start persistent WSL graph command.',
                ),
            );
            this._pending = undefined;
            this._drainQueue();
        }
    }

    private _ensureChild(): cp.ChildProcessWithoutNullStreams {
        if (this._child && !this._child.killed) {
            return this._child;
        }

        const args: string[] = [];
        const distro = String(this._options.distro || '').trim();
        if (distro) {
            args.push('-d', distro);
        }
        args.push('--exec', 'bash', '-s');

        const child = cp.spawn('wsl.exe', args, {
            env: process.env,
            windowsHide: true,
            stdio: 'pipe',
        });

        this._stdoutRemainder = '';

        child.stdout.setEncoding('utf8');
        child.stderr.setEncoding('utf8');
        child.stdout.on('data', (chunk: string) => {
            this._handleStdoutChunk(chunk);
        });
        child.stderr.on('data', (chunk: string) => {
            this._handleStderrChunk(chunk);
        });

        child.on('error', (err) => {
            this._handleChildFailure(
                new Error(`Persistent WSL graph runner failed: ${err.message}`),
            );
        });

        child.on('close', (code, signal) => {
            if (this._disposed) {
                return;
            }
            this._handleChildFailure(
                new Error(`Persistent WSL graph runner closed (code=${code ?? 'null'}, signal=${signal ?? 'null'}).`),
            );
        });

        this._child = child;
        return child;
    }

    private _handleStdoutChunk(chunk: string) {
        this._stdoutRemainder += chunk;
        const lines = this._stdoutRemainder.split(/\r?\n/);
        this._stdoutRemainder = lines.pop() ?? '';
        for (const line of lines) {
            this._handleStdoutLine(line);
        }
    }

    private _handleStdoutLine(lineRaw: string) {
        const line = lineRaw.replace(/\r$/, '');
        const pending = this._pending;
        if (!pending) {
            return;
        }

        const beginToken = this._beginToken(pending.id);
        const endPrefix = this._endPrefix(pending.id);

        if (!pending.started) {
            if (line === beginToken) {
                pending.started = true;
                pending.lines = [];
            }
            return;
        }

        if (line.startsWith(endPrefix)) {
            const statusText = line.slice(endPrefix.length).trim();
            const status = Number.parseInt(statusText, 10);
            const normalizedStatus = Number.isFinite(status) ? status : 1;
            const stdout = pending.lines.join('\n');

            this._clearPendingTimer(pending);
            this._pending = undefined;

            if (normalizedStatus === 0) {
                pending.resolve(stdout);
            } else {
                const errorText = stdout.trim()
                    || `Persistent WSL graph command failed with exit code ${normalizedStatus}.`;
                pending.reject(new Error(errorText));
            }

            this._drainQueue();
            return;
        }

        pending.lines.push(line);
    }

    private _handleStderrChunk(chunk: string) {
        const text = String(chunk || '').trim();
        if (!text) {
            return;
        }

        // Stderr noise can indicate a dead shell; if there is no in-flight
        // request we keep it as debug output only.
        if (!this._pending) {
            return;
        }

        this._pending.lines.push(text);
    }

    private _failCurrentRequest(err: Error) {
        const pending = this._pending;
        if (!pending) {
            return;
        }

        this._clearPendingTimer(pending);
        this._pending = undefined;
        pending.reject(err);
        this._destroyChild();
        this._drainQueue();
    }

    private _handleChildFailure(err: Error) {
        if (this._disposed) {
            return;
        }

        const pending = this._pending;
        if (pending) {
            this._clearPendingTimer(pending);
            this._pending = undefined;
            pending.reject(err);
        }

        this._destroyChild();
        this._drainQueue();
    }

    private _destroyChild() {
        if (!this._child) {
            return;
        }

        const child = this._child;
        this._child = undefined;

        child.stdout.removeAllListeners();
        child.stderr.removeAllListeners();
        child.removeAllListeners();

        if (!child.killed) {
            try {
                child.kill('SIGTERM');
            } catch {
                // ignore process teardown errors
            }
        }
    }

    private _scheduleIdleShutdownIfNeeded() {
        if (this._disposed || this._pending || this._queue.length > 0 || !this._child) {
            return;
        }
        if (this._idleTimer) {
            return;
        }

        const idleTimeoutMs = Number.isFinite(this._options.idleTimeoutMs)
            ? Math.max(1000, Math.floor(this._options.idleTimeoutMs as number))
            : DEFAULT_IDLE_TIMEOUT_MS;

        this._idleTimer = setTimeout(() => {
            this._idleTimer = undefined;
            if (this._pending || this._queue.length > 0) {
                return;
            }
            this._destroyChild();
        }, idleTimeoutMs);
        this._idleTimer.unref();
    }

    private _clearIdleTimer() {
        if (!this._idleTimer) {
            return;
        }
        clearTimeout(this._idleTimer);
        this._idleTimer = undefined;
    }

    private _clearPendingTimer(pending: PendingRequest) {
        if (!pending.timer) {
            return;
        }
        clearTimeout(pending.timer);
        pending.timer = undefined;
    }

    private _beginToken(id: string): string {
        return `__RDT_PERSIST_BEGIN__:${id}`;
    }

    private _endPrefix(id: string): string {
        return `__RDT_PERSIST_END__:${id}:`;
    }
}
