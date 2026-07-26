import { describe, expect, it } from 'vitest';
import {
    escapeRegex,
    toPascalCase,
    escapePythonSingleQuotedString,
    escapeCppDoubleQuotedString,
} from '../../../src/ros/utils/strings';

describe('strings/escapeRegex', () => {
    it('escapes regex metacharacters so the value matches literally', () => {
        const escaped = escapeRegex('a.b*c(d)');
        expect(new RegExp(`^${escaped}$`).test('a.b*c(d)')).toBe(true);
        expect(new RegExp(`^${escaped}$`).test('axbxxcxdx')).toBe(false);
    });

    it('leaves plain identifiers untouched', () => {
        expect(escapeRegex('rclcpp')).toBe('rclcpp');
    });
});

describe('strings/toPascalCase', () => {
    it('converts snake_case and kebab-case to PascalCase', () => {
        expect(toPascalCase('my_node')).toBe('MyNode');
        expect(toPascalCase('my-cool-node')).toBe('MyCoolNode');
    });

    it('is stable for already-cased single tokens', () => {
        expect(toPascalCase('MyNode')).toBe('MyNode');
    });
});

describe('strings/escapePythonSingleQuotedString', () => {
    it('escapes single quotes and backslashes', () => {
        expect(escapePythonSingleQuotedString("it's")).toBe("it\\'s");
        expect(escapePythonSingleQuotedString('a\\b')).toBe('a\\\\b');
    });

    it('coerces nullish input to an empty string', () => {
        expect(escapePythonSingleQuotedString(undefined as unknown as string)).toBe('');
    });
});

describe('strings/escapeCppDoubleQuotedString', () => {
    it('escapes double quotes and backslashes', () => {
        expect(escapeCppDoubleQuotedString('say "hi"')).toBe('say \\"hi\\"');
        expect(escapeCppDoubleQuotedString('a\\b')).toBe('a\\\\b');
    });
});
