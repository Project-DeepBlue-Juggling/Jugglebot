#!/usr/bin/env python3
"""Replay a BOUNDARY-ONLY rectangle search against dense pass grids (no new solves).

Reads the JSON files ``family_run.py`` writes (every cell of a real sweep family)
and, per (site pair, flight):

* **Soundness.** Does the sweep's own extracted rectangle (``_flight_band_and_rect``,
  room-first rule) ever have a passing perimeter but a failing interior cell? Does
  the boundary-search rectangle below? A zero count is what licenses certifying a
  rectangle from its boundary.
* **Cost.** How many grid cells does this search touch against the full grid:
  bisection on the half-width of a centred square (perimeter cells only), then
  greedy per-edge growth checking only the new edge strip.

The count is at GRID resolution (17 offsets per axis); a continuous search with
corner + edge-midpoint checks touches fewer still, which the Lipschitz slope from
``margin_rays.py`` justifies for the pose planner.

usage: python tools/probes/admissible_obviation/boundary_replay.py <family_run json>...

Part of the admissible-box sweep obviation investigation
(``logbook/2026-11-04-admissible-box-sweep-obviation-investigation.md``).
"""
import collections, json, os, sys
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.insert(0, ROOT); sys.path.insert(0, os.path.join(ROOT, 'ros_ws', 'src', 'jugglebot'))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
import admissible_sweep as asw                                        # noqa: E402


def main(files):
    tot_cells = tot_touch = unsound = nrects = nonconvex = 0
    for fn in files:
        d = json.load(open(fn))
        off = [r for r in d['rows'] if r.get('offset_mm') is not None]
        by = collections.defaultdict(dict)
        for r in off:
            by[(tuple(r['site_pair']), r['flight_s'])][tuple(r['offset_mm'])] = r['ok']
        offs = sorted(set(k[0] for g in by.values() for k in g))
        for key, g in sorted(by.items()):
            touched = set()

            def ev(x, y):
                touched.add((x, y))
                return g[(x, y)]
            tot_cells += len(g)
            if not ev(0.0, 0.0):
                tot_touch += len(touched)
                continue
            pg = {(key[1], x, y): v for (x, y), v in g.items()}
            _band, rect = asw._flight_band_and_rect([key[1]], offs, pg, key[1])
            nrects += 1
            if rect:
                xlo, xhi, ylo, yhi = rect
                inside = [(x, y) for (x, y) in g if xlo <= x <= xhi and ylo <= y <= yhi]
                perim = [(x, y) for (x, y) in inside if x in (xlo, xhi) or y in (ylo, yhi)]
                if all(g[p] for p in perim) and not all(g[p] for p in inside):
                    unsound += 1
            pos = [o for o in offs if o > 0]

            def square_ok(h):
                return all(ev(x, y) for (x, y) in g
                           if (abs(x) == h and abs(y) <= h) or (abs(y) == h and abs(x) <= h))
            lo, hi, best = 0, len(pos), None
            while lo < hi:
                mid = (lo + hi) // 2
                if square_ok(pos[mid]):
                    best = pos[mid]; lo = mid + 1
                else:
                    hi = mid
            if best is None:
                tot_touch += len(touched)
                continue
            xlo, xhi, ylo, yhi = -best, best, -best, best
            grew = True
            while grew:
                grew = False
                for side in ('xhi', 'xlo', 'yhi', 'ylo'):
                    cur = dict(xlo=xlo, xhi=xhi, ylo=ylo, yhi=yhi)[side]
                    nxt = [o for o in offs if (o > cur if side.endswith('hi') else o < cur)]
                    if not nxt:
                        continue
                    cand = min(nxt) if side.endswith('hi') else max(nxt)
                    strip = ([(cand, y) for y in offs if ylo <= y <= yhi] if side.startswith('x')
                             else [(x, cand) for x in offs if xlo <= x <= xhi])
                    if all(ev(*p) for p in strip):
                        if side == 'xhi': xhi = cand
                        elif side == 'xlo': xlo = cand
                        elif side == 'yhi': yhi = cand
                        else: ylo = cand
                        grew = True
            inside = [(x, y) for (x, y) in g if xlo <= x <= xhi and ylo <= y <= yhi]
            if not all(g[p] for p in inside):
                nonconvex += 1
            tot_touch += len(touched)
            if rect and rect != (xlo, xhi, ylo, yhi):
                print(os.path.basename(fn), key, 'sweep rect', rect, 'boundary rect', (xlo, xhi, ylo, yhi),
                      'touched', len(touched))
    print('rectangles %d | sweep rect: passing perimeter but failing interior = %d | boundary rect with a '
          'failing interior cell = %d' % (nrects, unsound, nonconvex))
    print('grid cells %d, touched by boundary search %d (%.1fx fewer)'
          % (tot_cells, tot_touch, tot_cells / max(tot_touch, 1)))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
