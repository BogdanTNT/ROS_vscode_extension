import { describe, expect, it } from 'vitest';
import {
    normalizeNodeTemplateKind,
    normalizeNodeTemplateTopic,
    getCppTemplateDependencies,
    buildPythonNodeTemplate,
    buildCppNodeTemplate,
} from '../../../src/ros/templates/nodeTemplates';

describe('nodeTemplates/normalizeNodeTemplateKind', () => {
    it('accepts supported kinds case-insensitively', () => {
        expect(normalizeNodeTemplateKind('Publisher')).toBe('publisher');
        expect(normalizeNodeTemplateKind('  timer ')).toBe('timer');
    });

    it('falls back to "none" for unknown or missing kinds', () => {
        expect(normalizeNodeTemplateKind('bogus')).toBe('none');
        expect(normalizeNodeTemplateKind(undefined)).toBe('none');
    });
});

describe('nodeTemplates/normalizeNodeTemplateTopic', () => {
    it('defaults to "chatter" and replaces whitespace with underscores', () => {
        expect(normalizeNodeTemplateTopic('')).toBe('chatter');
        expect(normalizeNodeTemplateTopic('  ')).toBe('chatter');
        expect(normalizeNodeTemplateTopic('my topic')).toBe('my_topic');
    });
});

describe('nodeTemplates/getCppTemplateDependencies', () => {
    it('maps template kinds to their ament dependencies', () => {
        expect(getCppTemplateDependencies('publisher')).toEqual(['std_msgs']);
        expect(getCppTemplateDependencies('subscriber')).toEqual(['std_msgs']);
        expect(getCppTemplateDependencies('service')).toEqual(['example_interfaces']);
        expect(getCppTemplateDependencies('client')).toEqual(['example_interfaces']);
        expect(getCppTemplateDependencies('timer')).toEqual([]);
        expect(getCppTemplateDependencies('none')).toEqual([]);
    });
});

describe('nodeTemplates/buildPythonNodeTemplate', () => {
    it('emits a runnable publisher node named after the class', () => {
        const src = buildPythonNodeTemplate('my_node', 'publisher', 'chatter');
        expect(src).toContain('import rclpy');
        expect(src).toContain('class MyNodeNode(Node):');
        expect(src).toContain("super().__init__('my_node')");
        expect(src).toContain('create_publisher');
        expect(src).toContain("'chatter'");
        expect(src).toContain('def main(');
    });

    it('emits a subscriber node for the subscriber kind', () => {
        const src = buildPythonNodeTemplate('listener', 'subscriber', 'chatter');
        expect(src).toContain('create_subscription');
    });

    it('escapes single quotes in the topic name', () => {
        const src = buildPythonNodeTemplate('n', 'publisher', "it's");
        expect(src).toContain("it\\'s");
    });
});

describe('nodeTemplates/buildCppNodeTemplate', () => {
    it('emits a C++ node using rclcpp and the node name', () => {
        const src = buildCppNodeTemplate('my_node', 'publisher', 'chatter');
        expect(src).toContain('#include');
        expect(src).toContain('rclcpp');
        expect(src).toContain('my_node');
        expect(src).toContain('int main');
    });
});
