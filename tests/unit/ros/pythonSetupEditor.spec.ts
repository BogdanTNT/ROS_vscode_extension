import { describe, expect, it } from 'vitest';
import {
    ensurePythonLaunchInstallInSetupPy,
    ensurePythonLaunchInstallInSetupCfg,
    removeNodeFromSetupPyConsoleScripts,
    removeNodeFromSetupCfgConsoleScripts,
} from '../../../src/ros/build/pythonSetupEditor';

const SETUP_PY = [
    'from setuptools import setup',
    '',
    "package_name = 'my_pkg'",
    '',
    'setup(',
    '    name=package_name,',
    "    version='0.0.0',",
    '    data_files=[',
    "        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),",
    "        ('share/' + package_name, ['package.xml']),",
    '    ],',
    '    entry_points={',
    "        'console_scripts': [",
    "            'my_node = my_pkg.my_node:main',",
    "            'other = my_pkg.other:main',",
    '        ],',
    '    },',
    ')',
    '',
].join('\n');

describe('pythonSetupEditor/ensurePythonLaunchInstallInSetupPy', () => {
    it('adds the glob import and a launch data_files entry', () => {
        const result = ensurePythonLaunchInstallInSetupPy(SETUP_PY, 'my_pkg');
        expect(result).toContain('from glob import glob');
        expect(result).toContain("('share/my_pkg/launch', glob('launch/*'))");
    });

    it('is idempotent', () => {
        const once = ensurePythonLaunchInstallInSetupPy(SETUP_PY, 'my_pkg');
        const twice = ensurePythonLaunchInstallInSetupPy(once, 'my_pkg');
        expect(twice).toBe(once);
    });
});

describe('pythonSetupEditor/ensurePythonLaunchInstallInSetupCfg', () => {
    it('appends an [options.data_files] section when none exists', () => {
        const result = ensurePythonLaunchInstallInSetupCfg('[metadata]\nname = my_pkg\n', 'my_pkg');
        expect(result).toContain('[options.data_files]');
        expect(result).toContain('share/my_pkg/launch =');
        expect(result).toContain('launch/*');
    });

    it('is idempotent', () => {
        const once = ensurePythonLaunchInstallInSetupCfg('[metadata]\nname = my_pkg\n', 'my_pkg');
        const twice = ensurePythonLaunchInstallInSetupCfg(once, 'my_pkg');
        expect(twice).toBe(once);
    });
});

describe('pythonSetupEditor/removeNodeFromSetupPyConsoleScripts', () => {
    it('removes only the targeted console_scripts entry', () => {
        const result = removeNodeFromSetupPyConsoleScripts(SETUP_PY, 'my_node');
        expect(result).not.toContain("'my_node = my_pkg.my_node:main'");
        expect(result).toContain("'other = my_pkg.other:main'");
    });

    it('returns the input unchanged when the node is not present', () => {
        expect(removeNodeFromSetupPyConsoleScripts(SETUP_PY, 'ghost')).toBe(SETUP_PY);
    });
});

describe('pythonSetupEditor/removeNodeFromSetupCfgConsoleScripts', () => {
    it('removes a node entry from [options.entry_points]', () => {
        const cfg = [
            '[options.entry_points]',
            'console_scripts =',
            '    my_node = my_pkg.my_node:main',
            '    other = my_pkg.other:main',
        ].join('\n');
        const result = removeNodeFromSetupCfgConsoleScripts(cfg, 'my_node');
        expect(result).not.toContain('my_node = my_pkg.my_node:main');
        expect(result).toContain('other = my_pkg.other:main');
    });
});
