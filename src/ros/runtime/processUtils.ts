/**
 * Pure POSIX process-tree helpers used when interrupting or inspecting the
 * child processes spawned by launch/run terminals.
 *
 * Extracted from {@link RosWorkspace}: these functions only touch
 * `child_process`/`process` and hold no instance state, which keeps the
 * workspace class focused on orchestration and makes signalling behaviour
 * reusable.
 */
import * as cp from 'child_process';

/** Return the lower-cased command name of a PID, or undefined if unknown. */
export function getProcessName(pid: number): string | undefined {
    try {
        const out = cp
            .execSync(`ps -p ${pid} -o comm= 2>/dev/null`, { encoding: 'utf8' })
            .trim()
            .toLowerCase();
        return out || undefined;
    } catch {
        return undefined;
    }
}

/** Whether a process command name looks like an interactive shell. */
export function isShellLikeProcess(name: string): boolean {
    const shellNames = new Set([
        'bash',
        'sh',
        'zsh',
        'fish',
        'dash',
        'ksh',
        'tmux',
    ]);
    return shellNames.has(name);
}

/**
 * Recursively collect all descendant PIDs of `parentPid`,
 * returned in leaf-first (deepest-first) order.
 */
export function getDescendantPids(parentPid: number): number[] {
    const result: number[] = [];
    try {
        // `pgrep -P <pid>` lists direct children.
        const raw = cp.execSync(`pgrep -P ${parentPid} 2>/dev/null`, { encoding: 'utf8' }).trim();
        if (!raw) {
            return result;
        }
        const children = raw.split('\n').map(s => parseInt(s, 10)).filter(n => n > 0);
        // Recurse into each child first (depth-first) …
        for (const child of children) {
            result.push(...getDescendantPids(child));
        }
        // … then add the direct children.
        result.push(...children);
    } catch { /* pgrep returns non-zero when there are no children */ }
    return result;
}

/**
 * Send SIGINT (Ctrl+C) to the inner bash's child processes.
 * This interrupts ros2 launch without closing the terminal.
 *
 * For `ros2 run` the actual node is a grandchild of the bash, so we
 * must walk the full descendant tree — `pkill -P` only hits direct
 * children and would leave the node process orphaned.
 */
export function interruptProcess(pid: number, includeProcessGroup: boolean = false): void {
    // Collect ALL descendant PIDs (recursive), leaf-first, so children
    // are signalled before parents.
    const descendants = getDescendantPids(pid);

    if (includeProcessGroup) {
        // Best-effort: signal the shell's process group to mimic Ctrl+C.
        try {
            process.kill(-pid, 'SIGINT');
        } catch { /* ignore */ }
    }

    // Signal every descendant individually (leaf → root order).
    for (const desc of descendants) {
        try {
            process.kill(desc, 'SIGINT');
        } catch { /* ignore */ }
    }

    // Also send to the bash itself in case it's the foreground
    try {
        process.kill(pid, 'SIGINT');
    } catch { /* ignore */ }
}
