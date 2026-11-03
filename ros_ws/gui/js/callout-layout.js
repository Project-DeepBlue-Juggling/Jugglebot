/**
 * callout-layout.js -- 1-D label layout for the chart hover callout pills (JB-159).
 *
 * Why: each pill is placed at its own curve's y.  When two curves are closer
 * than one pill height (position and temperature, say) the pills stacked on
 * top of each other and every pill behind the front one was invisible.  uPlot's
 * cursor points are off, so the pills ARE the readout.
 *
 * Pure (no DOM) so it can be driven under node.  Only vertical positions move;
 * the order of pills by y is preserved (a curve above stays above).
 */

/**
 * @param {number[]} desired  desired centre y of each pill (px), any order
 * @param {number} h          pill height (px)
 * @param {number} plotH      plot height (px)
 * @returns {number[]} centre y per pill, same index order as `desired`, no two
 *   closer than `h`, each as near its desired y as possible, inside
 *   [h/2, plotH - h/2] when they fit; stacked from the top when they cannot.
 */
export function layoutCallouts(desired, h, plotH) {
    const n = desired.length;
    if (n === 0) return [];
    const order = desired.map((_, i) => i).sort((a, b) => (desired[a] - desired[b]) || (a - b));
    const lo = h / 2;
    const hi = plotH - h / 2;
    const y = order.map((i) => desired[i]);
    if (!(hi - lo >= (n - 1) * h)) {
        // Cannot fit: stack from the top.
        const out = new Array(n);
        order.forEach((i, k) => { out[i] = lo + k * h; });
        return out;
    }
    // Sweep down: push each below the previous.
    y[0] = Math.max(y[0], lo);
    for (let k = 1; k < n; k++) y[k] = Math.max(y[k], y[k - 1] + h);
    // Sweep up from the bottom if the last overflows.
    if (y[n - 1] > hi) {
        y[n - 1] = hi;
        for (let k = n - 2; k >= 0; k--) y[k] = Math.min(y[k], y[k + 1] - h);
    }
    const out = new Array(n);
    order.forEach((i, k) => { out[i] = y[k]; });
    return out;
}
