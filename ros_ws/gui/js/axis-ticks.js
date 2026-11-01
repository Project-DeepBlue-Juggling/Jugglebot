/**
 * axis-ticks.js — uPlot axis options that guarantee at least two labelled ticks
 * on every chart axis, at every size and zoom (JB-101). Imports nothing, so
 * tests/ros/js/axis_ticks_harness.js runs it under node.
 *
 * Why one tick happened: uPlot picks the smallest step from its ladder whose
 * on-screen spacing is at least `space` px. On a short chart (or a zoom that
 * leaves a narrow span) `space` is a large fraction of the plot, so the step it
 * picks can be most of the visible span, and only one multiple of it falls in
 * the range.
 *
 * The guarantee: if `space` <= plotDim / (2 * F), where F is the largest ratio
 * between neighbouring steps of the ladder, the chosen step is below
 * F * (span / (2 * F)) = span / 2, and any interval at least two steps long
 * holds at least two multiples of the step. uPlot's numeric ladder is
 * {1, 2, 2.5, 5} x 10^k (F = 2); its seconds ladder jumps 1 s -> 5 s and
 * 1 min -> 5 min (F = 5), so TIME_INCRS below is uPlot's with 2 s and 2 min
 * added (F = 2.5). On a chart big enough for the default spacing nothing
 * changes: `space` is min(default, plotDim / (2 * F)).
 */

/** Largest neighbouring-step ratio of uPlot's default numeric ladder. */
export const NUM_LADDER_RATIO = 2;
/** Largest neighbouring-step ratio of TIME_INCRS. */
export const TIME_LADDER_RATIO = 2.5;

const SUB_SECOND = [0.001, 0.002, 0.0025, 0.005, 0.01, 0.02, 0.025, 0.05, 0.1, 0.2, 0.25, 0.5];
const M = 60, H = 3600, D = 86400;

/** Time-axis steps in seconds (uPlot's own ladder plus 2 s and 2 min). */
export const TIME_INCRS = SUB_SECOND.concat([
    1, 2, 5, 10, 15, 30,
    M, 2 * M, 5 * M, 10 * M, 15 * M, 30 * M,
    H, 2 * H, 3 * H, 4 * H, 6 * H, 8 * H, 12 * H,
    D, 2 * D, 3 * D, 4 * D, 5 * D, 6 * D, 7 * D, 8 * D, 9 * D, 10 * D, 15 * D,
]);

/** uPlot `axis.space` for a numeric axis whose default spacing is `defaultPx`. */
export function numTickSpace(defaultPx) {
    return (_u, _axisIdx, _min, _max, plotDim) =>
        Math.min(defaultPx, plotDim / (2 * NUM_LADDER_RATIO));
}

/** uPlot `axis.space` for a time axis using TIME_INCRS. */
export function timeTickSpace(defaultPx) {
    return (_u, _axisIdx, _min, _max, plotDim) =>
        Math.min(defaultPx, plotDim / (2 * TIME_LADDER_RATIO));
}

/**
 * Local HH:MM:SS labels for time splits (epoch seconds), with as many
 * fractional digits as the step needs. Without them a zoom below 1 s per
 * tick labels both ticks "12:34:56", which gives no scale either.
 */
export function timeTickLabels(splits) {
    let step = Infinity;
    for (let i = 1; i < splits.length; i++) step = Math.min(step, splits[i] - splits[i - 1]);
    const decimals = !(step < 1) ? 0 : step >= 0.1 - 1e-9 ? 1 : step >= 0.01 - 1e-9 ? 2 : 3;
    return splits.map(v => {
        const ms = Math.round(v * 1000);
        const d = new Date(ms);
        const hms = [d.getHours(), d.getMinutes(), d.getSeconds()]
            .map(n => String(n).padStart(2, '0')).join(':');
        if (decimals === 0) return hms;
        const frac = String(((ms % 1000) + 1000) % 1000).padStart(3, '0').slice(0, decimals);
        return hms + '.' + frac;
    });
}
