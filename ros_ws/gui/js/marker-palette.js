/**
 * marker-palette.js — the GUI's mocap label palette, shared by the marker
 * spheres (mocap-markers.js) and the trails (trail-feed.js). Pure: no imports,
 * no DOM, loadable under node.
 *
 *   Platform*    -> blue    (#3b82f6)
 *   Base*        -> red     (#ef4444)
 *   Ball Butler* -> yellow  (#eab308)
 *   Ball*        -> green   (#22c55e)
 *   unlabelled   -> light grey (#d1d5db)
 */

/** Colour lookup by label prefix (ordered: longest prefix first for correct matching) */
export const COLOUR_MAP = [
    { prefix: 'Catching_Cone', color: 0xa78bfa, group: 'Catching Cone' },
    { prefix: 'Catching Cone', color: 0xa78bfa, group: 'Catching Cone' },
    { prefix: 'Ball Butler', color: 0xeab308, group: 'Ball Butler' }, // yellow
    { prefix: 'Platform',    color: 0x3b82f6, group: 'Platform' },    // blue
    { prefix: 'Base',        color: 0xef4444, group: 'Base' },        // red
    { prefix: 'Ball',        color: 0x22c55e, group: 'Ball' },        // green
];
export const DEFAULT_COLOUR = 0xd1d5db; // light grey for unlabelled

export function labelToGroup(label) {
    if (!label) return 'Unlabelled';
    for (const entry of COLOUR_MAP) {
        if (label.startsWith(entry.prefix)) return entry.group;
    }
    return 'Unlabelled';
}

export function labelToColour(label) {
    if (!label) return DEFAULT_COLOUR;
    for (const entry of COLOUR_MAP) {
        if (label.startsWith(entry.prefix)) return entry.color;
    }
    return DEFAULT_COLOUR;
}

/**
 * Tracked-ball hues, indexed by ball id. Deliberately disjoint from the marker
 * palette above (blue/red/yellow/green/purple/grey) so a ball trail is never
 * mistaken for a marker trail.
 */
export const BALL_HUES = [
    0xf97316, // orange
    0xec4899, // pink
    0x06b6d4, // cyan
    0xd946ef, // fuchsia
    0x14b8a6, // teal
    0xc2410c, // burnt orange
    0x67e8f9, // light cyan
    0xf0abfc, // light fuchsia
];

export function ballColour(id) {
    const n = BALL_HUES.length;
    return BALL_HUES[(((id | 0) % n) + n) % n];
}
