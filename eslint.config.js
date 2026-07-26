// @ts-check
const js = require('@eslint/js');
const tseslint = require('typescript-eslint');

module.exports = tseslint.config(
    {
        ignores: ['out/**', 'node_modules/**', 'media/**', '.vscode-test/**'],
    },
    js.configs.recommended,
    ...tseslint.configs.recommended,
    {
        files: ['src/**/*.ts'],
        languageOptions: {
            ecmaVersion: 2021,
            sourceType: 'module',
        },
        rules: {
            // The codebase leans on `any` in a handful of VS Code interop spots;
            // warn rather than error so lint stays actionable during refactors.
            '@typescript-eslint/no-explicit-any': 'warn',
            '@typescript-eslint/no-unused-vars': [
                'warn',
                { argsIgnorePattern: '^_', varsIgnorePattern: '^_' },
            ],
            // Empty catch/blocks are sometimes intentional (best-effort cleanup).
            'no-empty': ['warn', { allowEmptyCatch: true }],
            // Several regexes intentionally match ASCII control characters when
            // sanitising WSL CLI output and distro names — this is by design.
            'no-control-regex': 'off',
        },
    },
    {
        files: ['tests/**/*.ts'],
        rules: {
            '@typescript-eslint/no-explicit-any': 'off',
            '@typescript-eslint/no-unused-vars': 'off',
        },
    },
);
