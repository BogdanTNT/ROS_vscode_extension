/* Shared Webview Interaction Helpers */
(function () {
    window.RosUi = window.RosUi || {};
    window.RosUi.interactions = window.RosUi.interactions || {};

    const interactions = window.RosUi.interactions;

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
})();
