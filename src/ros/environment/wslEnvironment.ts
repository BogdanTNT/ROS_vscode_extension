/**
 * Pure parsing/formatting helpers for WSL and Linux environment detection:
 * `wsl.exe --list` output, `/etc/os-release` contents, distro-name
 * sanitisation and shell-argument escaping.
 *
 * Extracted from {@link RosWorkspace}. No filesystem or process access — the
 * class still performs the actual `exec`/`fs` calls and passes raw output here.
 */

export interface OsReleaseInfo {
    name?: string;
    version?: string;
    versionId?: string;
    prettyName?: string;
}

export interface WslDistroSummary {
    name: string;
    state: string;
    version: string;
    isDefault: boolean;
}

/** Strip control/zero-width characters and surrounding quotes from a distro name. */
export function sanitizeWslDistroName(value?: string): string {
    return String(value || '')
        .replace(/\u0000/g, '')
        .replace(/\uFEFF/g, '')
        .replace(/[\u0001-\u001F\u007F-\u009F\u200B-\u200F\u202A-\u202E\u2060-\u206F]/g, '')
        .replace(/^"+|"+$/g, '')
        .trim();
}

/** Decode raw `wsl.exe` output, which is frequently UTF-16LE with a BOM. */
export function decodeWslCliOutput(raw: Buffer | string): string {
    if (typeof raw === 'string') {
        return raw;
    }
    if (!raw || raw.length === 0) {
        return '';
    }

    const hasUtf16Bom = raw.length >= 2 && raw[0] === 0xFF && raw[1] === 0xFE;
    const looksUtf16Le = raw.length >= 4 && raw[1] === 0x00 && raw[3] === 0x00;
    if (hasUtf16Bom || looksUtf16Le) {
        return raw.toString('utf16le');
    }
    return raw.toString('utf8');
}

/** Format a distro name for use as a `wsl.exe -d` argument, quoting if needed. */
export function formatWslDistroArg(distro: string): string {
    const normalized = sanitizeWslDistroName(distro);
    if (!normalized) {
        return '';
    }
    if (/^[A-Za-z0-9._-]+$/.test(normalized)) {
        return normalized;
    }
    return `"${normalized.replace(/"/g, '\\"')}"`;
}

/** Single-quote-escape a value for safe embedding in a POSIX shell command. */
export function escapeShellArg(value: string): string {
    const normalized = String(value ?? '');
    return `'${normalized.replace(/'/g, `'\\''`)}'`;
}

/** Parse `/etc/os-release` file contents into a structured descriptor. */
export function parseOsReleaseInfo(content: string): OsReleaseInfo | undefined {
    if (!content.trim()) {
        return undefined;
    }

    const info: OsReleaseInfo = {};
    for (const line of content.split(/\r?\n/)) {
        const trimmed = line.trim();
        if (!trimmed || trimmed.startsWith('#')) {
            continue;
        }

        const eqIdx = trimmed.indexOf('=');
        if (eqIdx <= 0) {
            continue;
        }

        const key = trimmed.slice(0, eqIdx).trim();
        let value = trimmed.slice(eqIdx + 1).trim();
        if (
            (value.startsWith('"') && value.endsWith('"'))
            || (value.startsWith('\'') && value.endsWith('\''))
        ) {
            value = value.slice(1, -1);
        }

        if (key === 'NAME') {
            info.name = value;
        } else if (key === 'VERSION') {
            info.version = value;
        } else if (key === 'VERSION_ID') {
            info.versionId = value;
        } else if (key === 'PRETTY_NAME') {
            info.prettyName = value;
        }
    }

    if (!info.prettyName && !info.name && !info.version && !info.versionId) {
        return undefined;
    }
    return info;
}

/** Produce a human-readable Linux distro label from an os-release descriptor. */
export function formatLinuxDistro(info: OsReleaseInfo): string {
    if (info.prettyName?.trim()) {
        return info.prettyName.trim();
    }

    const name = info.name?.trim() || '';
    const version = info.version?.trim() || info.versionId?.trim() || '';
    const joined = [name, version].filter(Boolean).join(' ').trim();
    return joined || 'unknown';
}

/** Produce a human-readable ROS environment label from distro/version. */
export function formatRosEnvironment(rosDistro?: string, rosVersion?: string): string {
    const distro = String(rosDistro || '').trim();
    const version = String(rosVersion || '').trim();

    if (!distro && !version) {
        return 'not set';
    }
    if (distro && version) {
        return `${distro} (ROS ${version})`;
    }
    if (distro) {
        return `${distro} (ROS version unknown)`;
    }
    return `unknown distro (ROS ${version})`;
}

/** Parse `wsl.exe --list --verbose` output into per-distro summaries. */
export function parseWslList(raw: string): WslDistroSummary[] {
    const distros: WslDistroSummary[] = [];
    const cleanedRaw = raw.replace(/\u0000/g, '').replace(/\uFEFF/g, '');

    for (const line of cleanedRaw.split(/\r?\n/)) {
        const trimmed = line.trim();
        if (!trimmed || /^name\s+state\s+version$/i.test(trimmed)) {
            continue;
        }

        let normalized = trimmed;
        let isDefault = false;
        if (normalized.startsWith('*')) {
            isDefault = true;
            normalized = normalized.slice(1).trim();
        }

        const parts = normalized.split(/\s{2,}/).map((part) => part.trim()).filter(Boolean);
        if (parts.length < 3) {
            continue;
        }

        const name = sanitizeWslDistroName(parts[0]);
        if (!name) {
            continue;
        }

        distros.push({
            name,
            state: parts[1] || 'unknown',
            version: parts[2] || '?',
            isDefault,
        });
    }

    return distros;
}

/** Extract a `KEY=value` environment variable from probe output. */
export function extractEnvVar(content: string, key: string): string | undefined {
    const escapedKey = key.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
    const match = content.match(new RegExp(`^${escapedKey}=(.*)$`, 'm'));
    const value = match?.[1]?.trim();
    return value || undefined;
}
