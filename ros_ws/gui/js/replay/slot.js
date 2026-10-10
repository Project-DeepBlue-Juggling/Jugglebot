// 10 s slot math for the main thread (Phase 4 design § 3). Mirrors schema.chunk_index bit-for-bit
// (CPython float.__floordiv__ then int()). A deliberate twin of slotOf in mcap-decode.js: that module
// imports the 119 kB MCAP bundle, which the main thread (and every node sandbox that loads
// sources.js) must not pull in. tests/ros/test_gui_replay_mcap_source.py pins the two equal.
export const CHUNK_S = 10;

export function slotOf(t, t0) {
  const a = t - t0, b = CHUNK_S;
  let mod = a % b;
  let div = (a - mod) / b;
  if (mod && mod < 0) { mod += b; div -= 1; }
  let f;
  if (div) { f = Math.floor(div); if (div - f > 0.5) f += 1; } else { f = 0; }
  return Math.trunc(f);
}
