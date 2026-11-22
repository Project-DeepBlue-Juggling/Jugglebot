/**
 * imu-bands.js — the IMU captures of the open recording as thin bands on the replay trackbar (trackbar.js).
 * Pure helpers (geometry, tooltip text, the one fetch) so a node harness can pin them; the DOM is trackbar.js's.
 * Independent of the IMU chart group: the list is fetched whether or not that group is shown.
 */

const p2 = (n) => (n < 10 ? '0' : '') + n;

/** "IMU capture HH:MM:SS · <label> · <span> s" (local time of the first sample). */
export function bandTitle(c) {
    const d = new Date(c.t_start * 1000);
    const hms = p2(d.getHours()) + ':' + p2(d.getMinutes()) + ':' + p2(d.getSeconds());
    const span = Number.isFinite(c.t_end - c.t_start) ? (c.t_end - c.t_start).toFixed(1) : '?';
    const label = (c.label && String(c.label).trim()) || 'no label';
    return 'IMU capture ' + hms + ' · ' + label + ' · ' + span + ' s';
}

/** Captures with usable times (the /api/imu/captures listing's `captures`), sorted by t_start. */
export function usableCaptures(listing) {
    const raw = listing && Array.isArray(listing.captures) ? listing.captures : [];
    return raw.filter((c) => c && Number.isFinite(c.t_start) && Number.isFinite(c.t_end) && c.t_end >= c.t_start)
        .slice().sort((a, b) => a.t_start - b.t_start);
}

/**
 * Band rectangles for the visible range [v0, v1]: {left, width} in percent of the bar, clipped to the range
 * (captures wholly outside are dropped), never thinner than MIN_PCT so a short capture stays visible.
 * @returns {{left:number, width:number, title:string, stem:string}[]}
 */
export const MIN_PCT = 0.4;
export function bandGeometry(captures, v0, v1) {
    const out = [];
    const span = v1 - v0;
    if (!(span > 0)) return out;
    for (const c of captures) {
        if (c.t_end < v0 || c.t_start > v1) continue;
        const a = Math.max(v0, c.t_start), b = Math.min(v1, c.t_end);
        const left = ((a - v0) / span) * 100;
        const width = Math.max(MIN_PCT, ((b - a) / span) * 100);
        out.push({ left: Math.min(left, 100 - MIN_PCT), width, title: bandTitle(c), stem: c.stem || '' });
    }
    return out;
}

/** GET /api/imu/captures?from=&to= -> captures; any failure (no fetch, HTTP error, bad JSON) is []. */
export async function fetchBandCaptures(fetchFn, t0, t1) {
    if (typeof fetchFn !== 'function') return [];
    try {
        const r = await fetchFn('/api/imu/captures?from=' + encodeURIComponent(t0) + '&to=' + encodeURIComponent(t1));
        if (!r || !r.ok) return [];
        return usableCaptures(await r.json());
    } catch (e) {
        return [];
    }
}
