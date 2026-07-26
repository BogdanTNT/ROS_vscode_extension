import { describe, expect, it } from 'vitest';
import {
    isShellLikeProcess,
    getProcessName,
    getDescendantPids,
} from '../../../src/ros/runtime/processUtils';

describe('processUtils/isShellLikeProcess', () => {
    it('recognises common interactive shells', () => {
        expect(isShellLikeProcess('bash')).toBe(true);
        expect(isShellLikeProcess('zsh')).toBe(true);
        expect(isShellLikeProcess('tmux')).toBe(true);
    });

    it('rejects non-shell process names', () => {
        expect(isShellLikeProcess('node')).toBe(false);
        expect(isShellLikeProcess('ros2')).toBe(false);
        expect(isShellLikeProcess('')).toBe(false);
    });
});

describe('processUtils/getProcessName', () => {
    it('returns undefined for an implausible PID', () => {
        // PID 2^30 is virtually guaranteed not to exist.
        expect(getProcessName(1 << 30)).toBeUndefined();
    });
});

describe('processUtils/getDescendantPids', () => {
    it('returns an empty array for a PID with no children', () => {
        // A non-existent PID has no descendants; pgrep exits non-zero.
        expect(getDescendantPids(1 << 30)).toEqual([]);
    });
});
