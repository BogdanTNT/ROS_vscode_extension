/* Shared Webview Interaction Helpers */
(function () {
    window.RosUi = window.RosUi || {};
    window.RosUi.interactions = window.RosUi.interactions || {};

    const interactions = window.RosUi.interactions;
    const DEFAULT_CLICK_THROUGH_SELECTOR = [
        'button',
        'input[type="button"]',
        'input[type="submit"]',
        'input[type="reset"]',
        'input[type="checkbox"]',
        'input[type="radio"]',
        '[role="button"]',
        '[tabindex="0"]',
    ].join(', ');

    /**
     * Alt is used as the "apply recursively / to all children" modifier
     * across dropdown-style controls in all webviews.
     */
    interactions.isRecursiveToggleEvent = (event) => {
        if (!event) {
            return false;
        }
        if (typeof event.getModifierState === 'function') {
            return event.getModifierState('Alt');
        }
        return Boolean(event.altKey);
    };

    /**
     * Binds a click handler with an optional recursive variant on Alt+click.
     * Returns a disposer for callers that need teardown.
     */
    interactions.bindRecursiveToggleClick = (element, onToggle, onRecursiveToggle, options) => {
        if (!element || typeof onToggle !== 'function') {
            return () => {};
        }

        const preventDefault = options?.preventDefault !== false;
        const listener = (event) => {
            if (preventDefault && typeof event.preventDefault === 'function') {
                event.preventDefault();
            }

            if (interactions.isRecursiveToggleEvent(event) && typeof onRecursiveToggle === 'function') {
                onRecursiveToggle(event);
                return;
            }

            onToggle(event);
        };

        element.addEventListener('click', listener);
        return () => {
            element.removeEventListener('click', listener);
        };
    };

    /**
     * Utility for list-style dropdowns:
     * if all children are selected, the recursive action clears all;
     * otherwise it selects all.
     */
    interactions.shouldEnableAllFromSelection = (allCount, selectedCount) => {
        const total = Number(allCount) || 0;
        const selected = Number(selectedCount) || 0;
        if (total <= 0) {
            return false;
        }
        return selected < total;
    };

    /**
     * Some VS Code webview contexts can consume the first click only for focus,
     * forcing users to click again to trigger a control.
     *
     * This helper mirrors a normal click on pointer down for actionable controls,
     * preserving modifier keys (Alt/Ctrl/Shift/Meta), then suppresses the
     * immediate native click to avoid double execution.
     *
     * It is intentionally generic so all extension webviews get consistent
     * first-click behavior.
     */
    interactions.enableSingleClickActivation = (options) => {
        const root = options?.root instanceof Document ? options.root : document;
        const selector = String(options?.selector || DEFAULT_CLICK_THROUGH_SELECTOR);
        const suppressWindowMs = Number(options?.suppressWindowMs) > 0
            ? Number(options?.suppressWindowMs)
            : 450;
        const suppressNativeClickUntil = new WeakMap();

        const isEligibleControl = (element) => {
            if (!element) {
                return false;
            }
            if (element.hasAttribute('disabled')) {
                return false;
            }
            if (element.getAttribute('aria-disabled') === 'true') {
                return false;
            }
            if (element instanceof HTMLInputElement && element.disabled) {
                return false;
            }
            return true;
        };

        const resolveActivationTarget = (eventTarget) => {
            if (!(eventTarget instanceof Element)) {
                return undefined;
            }
            const target = eventTarget.closest(selector);
            if (!(target instanceof Element)) {
                return undefined;
            }
            if (!isEligibleControl(target)) {
                return undefined;
            }
            return target;
        };

        const cloneModifierState = (event) => ({
            altKey: Boolean(event?.altKey),
            ctrlKey: Boolean(event?.ctrlKey),
            metaKey: Boolean(event?.metaKey),
            shiftKey: Boolean(event?.shiftKey),
        });

        const onPointerDownCapture = (event) => {
            if (!(event instanceof MouseEvent)) {
                return;
            }
            if (event.button !== 0) {
                return;
            }
            if (event.defaultPrevented) {
                return;
            }

            const target = resolveActivationTarget(event.target);
            if (!target) {
                return;
            }

            try {
                if (typeof target.focus === 'function') {
                    target.focus({ preventScroll: true });
                }
            } catch {
                // Fallback focus failures should not block click dispatch.
            }

            suppressNativeClickUntil.set(target, Date.now() + suppressWindowMs);

            const syntheticClick = new MouseEvent('click', {
                bubbles: true,
                cancelable: true,
                composed: true,
                button: 0,
                buttons: 1,
                clientX: event.clientX,
                clientY: event.clientY,
                ...cloneModifierState(event),
            });
            target.dispatchEvent(syntheticClick);

            // Avoid text-selection side effects and reduce duplicate native events.
            if (typeof event.preventDefault === 'function') {
                event.preventDefault();
            }
        };

        const onClickCapture = (event) => {
            if (!(event instanceof MouseEvent) || !event.isTrusted) {
                return;
            }

            const target = resolveActivationTarget(event.target);
            if (!target) {
                return;
            }

            const suppressUntil = suppressNativeClickUntil.get(target) || 0;
            if (Date.now() > suppressUntil) {
                return;
            }

            suppressNativeClickUntil.delete(target);
            event.preventDefault();
            event.stopImmediatePropagation();
        };

        // Capture phase ensures this runs before feature-specific click handlers.
        root.addEventListener('pointerdown', onPointerDownCapture, true);
        root.addEventListener('click', onClickCapture, true);

        return () => {
            root.removeEventListener('pointerdown', onPointerDownCapture, true);
            root.removeEventListener('click', onClickCapture, true);
        };
    };

    /**
     * Bind Enter-key submission for a modal confirm action.
     * - Ignores textarea/contenteditable targets to preserve multiline editing.
     * - Ignores controls where Enter already has native action semantics.
     */
    interactions.bindModalEnterConfirm = (options) => {
        const modal = options?.modal;
        const confirmButton = options?.confirmButton;
        const allowTextarea = options?.allowTextarea === true;
        const shouldSubmit = typeof options?.shouldSubmit === 'function'
            ? options.shouldSubmit
            : () => true;

        if (!(modal instanceof Element) || !(confirmButton instanceof HTMLElement)) {
            return () => {};
        }

        const onKeyDown = (event) => {
            if (!(event instanceof KeyboardEvent)) {
                return;
            }
            if (event.key !== 'Enter' || event.defaultPrevented || event.isComposing) {
                return;
            }
            if (event.altKey || event.ctrlKey || event.metaKey || event.shiftKey) {
                return;
            }
            if (modal.classList.contains('hidden')) {
                return;
            }

            const target = event.target;
            if (!(target instanceof Element)) {
                return;
            }
            if (!modal.contains(target)) {
                return;
            }
            if (!allowTextarea && target.closest('textarea, [contenteditable="true"]')) {
                return;
            }
            if (target.closest('button, a[href], summary, select, input[type="button"], input[type="submit"], input[type="checkbox"], input[type="radio"]')) {
                return;
            }
            if (confirmButton.hasAttribute('disabled') || confirmButton.getAttribute('aria-disabled') === 'true') {
                return;
            }
            if (!shouldSubmit(event)) {
                return;
            }

            event.preventDefault();
            event.stopPropagation();
            confirmButton.click();
        };

        modal.addEventListener('keydown', onKeyDown);
        return () => {
            modal.removeEventListener('keydown', onKeyDown);
        };
    };

    // Install globally once per webview document.
    if (!window.__rosSingleClickActivationInstalled) {
        window.__rosSingleClickActivationInstalled = true;
        interactions.enableSingleClickActivation();
    }
})();
