"""Label every Catch re-aim of a launch log for ``bump_replay.py --chain`` (BUG-75, 2026-11-26).

Parses the RESEND / RESEND-REFUSED lines of a ``launch.log`` into one row per re-aim attempt carrying what the
replay must know to put it on the right sim record: the turned pair's angle, Apex and separation (from the
``skills/nominate`` line before the Juggle request), the Catch it belongs to (``skill``, ``run``), its
position in that Catch (``seq`` = 1-based order among all attempts; ``n`` = accepted re-aims so far + 1, so a
refused attempt carries the number it would have had), the flown verdict (``code`` = INSTALLED or the refusal
code), the timing shift ``ds`` and the time to the committed landing (``to_land``, from the Catch's CATCH-AIM
line).  The re-aim's direction is NOT logged; the replay tries four.

    python tools/probes/late_reaim/label_rows.py <out.json> <tag>=<launch-log-dir-or-path> ...
"""
import json, os, re, sys

LOG_ROOT = '/home/jetson/.ros/log'
RX_NOM = re.compile(r"skills/nominate '(\w+),apex_m=([\d.]+),separation_mm=(\d+).*?columns_angle_deg=(\d+)")
RX_RUN = re.compile(r"Juggle requested.*pattern='([a-z_0-9]+)'")
RX_AIM = re.compile(r"(\d+\.\d+) CATCH-AIM skill (\d+): .*? t_land=(\d+\.\d+)")
RX_OK = re.compile(r"(\d+\.\d+) RESEND skill (\d+): the fit moved the landing ([\d.]+) mm / ([-+\d.]+) s \(re-aim (\d+) of")
RX_REF = re.compile(r"(\d+\.\d+) RESEND-REFUSED (\S+) at skill (\d+): the fit moved the landing ([\d.]+) mm / ([-+\d.]+) s")


def label_lines(lines, tag, patterns=('columns_turned',)):
    """Rows for the Juggle runs of ``patterns`` found in ``lines`` (an iterable of log lines)."""
    nom, run, run_i, aims, groups, rows = {}, None, 0, {}, {}, []
    for ln in lines:
        m = RX_NOM.search(ln)
        if m:
            nom = dict(pat=m.group(1), apex=float(m.group(2)), sep=float(m.group(3)), ang=int(m.group(4)))
        m = RX_RUN.search(ln)
        if m:
            run_i += 1
            run = dict(nom) if m.group(1) in patterns and nom.get('pat') == m.group(1) else None
            aims, groups = {}, {}
            continue
        if run is None:
            continue
        m = RX_AIM.search(ln)
        if m:
            aims[int(m.group(2))] = (float(m.group(1)), float(m.group(3)))
            continue
        m = RX_OK.search(ln)
        if m:
            t, k, mm, ds = float(m.group(1)), int(m.group(2)), float(m.group(3)), float(m.group(4))
            code, n_logged = 'INSTALLED', int(m.group(5))
        else:
            m = RX_REF.search(ln)
            if not m:
                continue
            t, code, k, mm, ds = float(m.group(1)), m.group(2), int(m.group(3)), float(m.group(4)), float(m.group(5))
            n_logged = None
        g = groups.setdefault(k, dict(seq=0, acc=0))
        g['seq'] += 1
        n = n_logged if n_logged is not None else g['acc'] + 1
        if code == 'INSTALLED':
            g['acc'] = n
        aim = aims.get(k)
        rows.append(dict(log=tag, run=run_i, t=t, skill=k, seq=g['seq'], n=n, code=code, mm=mm, ds=ds,
                         to_land=(aim[1] - t) if aim else None, apex=run['apex'], sep=run['sep'],
                         ang=run['ang'], pat=run['pat']))
    return rows


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    out, logs = argv[0], dict(a.split('=', 1) for a in argv[1:])
    assert logs, 'give at least one tag=logdir'
    rows = []
    for tag, d in logs.items():
        path = d if os.path.isabs(d) else os.path.join(LOG_ROOT, d)
        with open(os.path.join(path, 'launch.log'), errors='replace') as f:
            rows += label_lines(f, tag)
    with open(out, 'w') as f:
        json.dump(rows, f)
    from collections import Counter
    print(len(rows), 'rows;', dict(Counter((r['log'], r['ang'], r['apex'], r['sep'], min(r['n'], 2), r['code'] == 'INSTALLED') for r in rows)))


if __name__ == '__main__':
    main()
