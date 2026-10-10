// Pins slot.js::slotOf (main thread twin) == mcap-decode.js::slotOf (worker) on a vector. Prints JSON.
const a = await import('./js/replay/slot.js');
const b = await import('./js/replay/mcap-decode.js');
const vec = JSON.parse(process.argv[2]);
process.stdout.write(JSON.stringify(vec.map(([t, t0]) => [a.slotOf(t, t0), b.slotOf(t, t0)])));
