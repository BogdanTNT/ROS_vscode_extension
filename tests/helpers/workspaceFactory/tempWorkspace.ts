import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';

export function createTempWorkspace(files: Record<string, string>): string {
    const root = fs.mkdtempSync(path.join(os.tmpdir(), 'ros-dev-toolkit-test-'));

    for (const [relativePath, content] of Object.entries(files)) {
        const fullPath = path.join(root, relativePath);
        fs.mkdirSync(path.dirname(fullPath), { recursive: true });
        fs.writeFileSync(fullPath, content, 'utf8');
    }

    return root;
}

export function removeTempWorkspace(root: string): void {
    fs.rmSync(root, { recursive: true, force: true });
}
