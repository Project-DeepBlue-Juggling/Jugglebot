/**
 * fake_ros_bridge.js — a controllable stand-in for ros_ws/gui/js/ros-bridge.js,
 * copied into the bb_aim_harness.js sandbox AS `ros-bridge.js` (the exact
 * specifier bb-aim.js imports) by tests/ros/test_gui_bb_aim_timeout.py.
 *
 * `withTimeout` is inlined below, byte-identical to the real
 * `ros_ws/gui/js/ros-bridge.js::withTimeout` — inlined rather than imported
 * from a copy of the real file so this fake module loads standalone under
 * the PRE-FIX `bb-aim.js`/`ros-bridge.js` too (which has no `withTimeout`
 * export at all), letting the same harness demonstrate the fail-before /
 * pass-after contrast. `callService` is faked so the harness can make a
 * call hang forever (never resolve or reject) the way an unanswered
 * ball_butler_node does on the real robot.
 */

export function withTimeout(promise, ms, label) {
    return new Promise((resolve, reject) => {
        const t = setTimeout(
            () => reject(new Error(`${label} timed out after ${ms / 1000} s`)),
            ms);
        promise.then(
            (v) => { clearTimeout(t); resolve(v); },
            (e) => { clearTimeout(t); reject(e); },
        );
    });
}

export const __control = {
    calls: [],
    /** 'hang' (never settles) or { type: 'resolve', value } / { type: 'reject', error }. */
    defaultBehavior: 'hang',
    /** Optional per-call overrides, consumed in order; falls back to defaultBehavior. */
    behaviors: [],
};

export function callService(serviceName, serviceType, request) {
    __control.calls.push({ serviceName, serviceType, request });
    const behavior = __control.behaviors.shift() || __control.defaultBehavior;
    return new Promise((resolve, reject) => {
        if (behavior === 'hang') return; // never settles — the bug this harness reproduces
        if (behavior.type === 'resolve') { resolve(behavior.value); return; }
        if (behavior.type === 'reject') { reject(behavior.error); return; }
        throw new Error(`fake_ros_bridge: unknown behavior ${JSON.stringify(behavior)}`);
    });
}
