#!/usr/bin/env python3
"""BUG-129 launch-series parser: per-launch table + verdict (offline, reads files only).

Reads what ``tools/probes/bug129_launch_series.sh`` leaves for a series date:
  temp/logs/bug129_series_<date>_<A|B>_<nn>[r<k>].log    the launch's own output (ANSI colour)
  temp/logs/bug129_series_<date>_<A|B>_<nn>[r<k>].meta   key=value facts the runner measured
  temp/logs/stall_teensy_bridge_node_*.txt               stall-watchdog dumps (path is in the WARN)
  ~/Desktop/rosbags/<ts>/                                the launch's bag (path is in the log)
and writes temp/reports/bug129_series_<date>.md (one row per launch, then the verdict).

Rules (from the series spec, 2026-11-24):
  * VALID launch: log has `stall watchdog on: timeout 150 ms`, `shm clean: removed`,
    `bridge shared memory on:` (arm A) / `bridge shared memory off:` (arm B),
    `BRIDGE_FW_CHECK: OK`, a non-empty /robot_state in the bag, and the runner's
    manipulation check passed (fastrtps maps > 0 in A, exactly 0 in B).
  * STALL = a `stalled (largest gap` WARN with gap >= 250 ms CONFIRMED by a bridge-side bag
    gap >= 250 ms (the /robot_state log-time gap AND the header-stamp gap both exceed it;
    a log gap with stamps 10 ms apart is recorder-side and does not count).
  * Each stall's dump is classified P (a thread whose top frame is inside rclpy publish),
    K (kernel-pause shape: the watchdog inside its own _arm and the RX thread draining the
    backlog in _drain_socket) or O. Only P counts for the shm question; K counts for neither arm.
  * WARNs with 150 <= gap < 250 ms are listed separately.
  * Verdict is asymmetric (see verdict()).

Usage:  python tools/probes/bug129_series_report.py --date 2026-11-25 [--main DIR] [--no-bags]
Python stdlib + (lazily) mcap/mcap_ros2-free CDR header read; scipy only for Mann-Whitney.
"""
from __future__ import annotations

import argparse
import glob
import math
import os
import re
import struct
import sys
import time
from datetime import datetime
from typing import Dict, List, Optional, Sequence, Tuple

ANSI_RE = re.compile(r'\x1b\[[0-9;]*[A-Za-z]')
STALL_THRESHOLD_MS = 250.0     # the guard's SETPOINT_STALE limit
SOFT_WARN_MS = 150.0           # the watchdog's own timeout
SECONDARY_GAP_MS = 50.0
CONFIRM_WINDOW_S = 3.0         # a WARN is reported just after the stall ends

LOG_NAME_RE = re.compile(r'^bug129_series_(?P<date>\d{4}-\d{2}-\d{2})_(?P<arm>[AB])_(?P<slot>\d+)(?:r(?P<retry>\d+))?\.log$')
STALL_RE = re.compile(
    r'^(?P<hms>\d{2}:\d{2}:\d{2}\.\d{3})\s.*stall watchdog: (?P<tag>\S+) stalled \(largest gap '
    r'(?P<gap>[0-9.]+) ms, timeout (?P<timeout>\d+) ms\).*?(?P<path>/\S+\.txt|\(not written[^)]*\))\s*$')
BAG_RE = re.compile(r'recording bag -> (?P<path>\S+)')
# ODrive undervoltage ERRORs are expected on every unpowered launch: not a side finding.
EXPECTED_ERROR_RE = re.compile(r'ODrive error on|FAULT LATCHED|FAULT cause|fault_state=ODRIVE_FATAL|Undervoltage')


def strip_ansi(text: str) -> str:
    return ANSI_RE.sub('', text)


# ── log parsing ───────────────────────────────────────────────────────────────────────────

def parse_log(text: str) -> Dict:
    """Validity flags, stall WARNs and side-finding counts from one launch log (ANSI ok)."""
    text = strip_ansi(text)
    r: Dict = {
        'watchdog_on': 'stall watchdog on: timeout 150 ms' in text,
        'shm_clean': 'shm clean: removed' in text,
        'shm_on': 'bridge shared memory on:' in text,
        'shm_off': 'bridge shared memory off:' in text,
        'fw_ok': 'BRIDGE_FW_CHECK: OK' in text,
        'bag': None, 'stalls': [], 'setpoint_stale': 0, 'traceback': 0, 'errors_other': 0,
    }
    for line in text.splitlines():
        m = BAG_RE.search(line)
        if m and r['bag'] is None:
            r['bag'] = m.group('path')
        m = STALL_RE.match(line)
        if m:
            r['stalls'].append({'hms': m.group('hms'), 'gap_ms': float(m.group('gap')),
                                'path': m.group('path'), 'tag': m.group('tag')})
        if 'SETPOINT_STALE' in line:
            r['setpoint_stale'] += 1
        if 'Traceback' in line:
            r['traceback'] += 1
        if re.search(r'\bERROR\b', line) and not EXPECTED_ERROR_RE.search(line):
            r['errors_other'] += 1
    return r


def parse_meta(text: str) -> Dict[str, str]:
    out = {}
    for line in text.splitlines():
        if '=' in line and not line.lstrip().startswith('#'):
            k, v = line.split('=', 1)
            out[k.strip()] = v.strip()
    return out


# ── dump classification ──────────────────────────────────────────────────────────────────

def _dump_threads(text: str) -> List[List[str]]:
    """Each thread's frames (most recent first) from a faulthandler dump."""
    threads: List[List[str]] = []
    for line in text.splitlines():
        if line.startswith('Thread ') or line.startswith('Current thread '):
            threads.append([])
        elif line.lstrip().startswith('File ') and threads:
            threads[-1].append(line.strip())
    return threads


def classify_dump(text: str) -> str:
    """P: a thread's TOP frame is rclpy's publish (stuck inside the C publish call).
    K: kernel-pause shape: the watchdog thread is inside its own _arm AND a thread is in
       _drain_socket (the RX thread draining the backlog after the pause).
    O: anything else (or an empty dump)."""
    threads = _dump_threads(text)
    for frames in threads:
        if frames and 'rclpy/publisher.py' in frames[0] and ' in publish' in frames[0]:
            return 'P'
    wd_in_arm = any(frames and 'stall_watchdog.py' in frames[0] and ' in _arm' in frames[0]
                    for frames in threads)
    draining = any(any(' in _drain_socket' in f for f in frames) for frames in threads)
    if wd_in_arm and draining:
        return 'K'
    return 'O'


# ── bag gaps ──────────────────────────────────────────────────────────────────────────────

def classify_gaps(rows: Sequence[Tuple[int, float]], thr_ms: float) -> Dict[str, List[Tuple[float, float]]]:
    """rows = [(log_time_ns, header_stamp_s)] in log order. Returns, for gaps whose LOG gap is
    >= thr_ms: 'bridge' (stamp gap also >= thr_ms: the bridge did not produce) and 'recorder'
    (stamps still ~10 ms apart: the recorder/transport dropped). Entries are (end_log_epoch_s,
    log_gap_ms)."""
    bridge, recorder = [], []
    for (t0, s0), (t1, s1) in zip(rows, rows[1:]):
        log_gap = (t1 - t0) / 1e6
        if log_gap < thr_ms:
            continue
        stamp_gap = (s1 - s0) * 1e3
        (bridge if stamp_gap >= thr_ms else recorder).append((t1 / 1e9, log_gap))
    return {'bridge': bridge, 'recorder': recorder}


def bridge_gap_stats(rows: Sequence[Tuple[int, float]]) -> Tuple[int, float]:
    """(count of bridge-side gaps > 50 ms, largest bridge-side gap ms). A gap is bridge-side
    when BOTH the log-time gap and the header-stamp gap exceed the threshold."""
    n, mx = 0, 0.0
    for (t0, s0), (t1, s1) in zip(rows, rows[1:]):
        log_gap = (t1 - t0) / 1e6
        stamp_gap = (s1 - s0) * 1e3
        g = min(log_gap, stamp_gap)
        if g > SECONDARY_GAP_MS:
            n += 1
            mx = max(mx, g)
    return n, mx


def read_bag_rows(bag_dir: str) -> Optional[List[Tuple[int, float]]]:
    """/robot_state (log_time_ns, header stamp s), streamed message by message. None if the
    bag is unreadable or mcap is not importable."""
    try:
        from mcap.records import Channel, Message  # type: ignore
        from mcap.stream_reader import StreamReader  # type: ignore
    except Exception:  # noqa: BLE001
        return None
    files = sorted(glob.glob(os.path.join(bag_dir, '*.mcap')))
    if not files:
        return None
    rows: List[Tuple[int, float]] = []
    for f in files:
        # StreamReader, not NonSeekingReader.iter_messages: the latter reads the whole file to sort
        # before yielding anything, so a bag whose recorder was killed with the launch (it ends
        # inside the message-index / summary section, after every data chunk: all 24 bags of the
        # first series, 2026-11-25) raised before the first message and every launch was marked
        # invalid. Here messages are taken as they stream and a truncated tail is ignored; an
        # unreadable bag yields no rows and stays invalid.
        robot_state_ids = set()
        try:
            with open(f, 'rb') as fh:
                for rec in StreamReader(fh).records:
                    if isinstance(rec, Channel):
                        if rec.topic == '/robot_state':
                            robot_state_ids.add(rec.id)
                    elif isinstance(rec, Message) and rec.channel_id in robot_state_ids:
                        sec, nsec = struct.unpack_from('<iI', rec.data, 4)   # CDR: header.stamp first
                        rows.append((rec.log_time, sec + nsec * 1e-9))
        except Exception:  # noqa: BLE001
            continue
    if not rows:
        return None
    rows.sort()
    return rows


# ── per-launch analysis ──────────────────────────────────────────────────────────────────

def hms_to_epoch(hms: str, ref_epoch: float) -> float:
    """Wall-clock HH:MM:SS.mmm -> epoch on the day nearest ref_epoch."""
    h, m, s = hms.split(':')
    d = datetime.fromtimestamp(ref_epoch).replace(hour=int(h), minute=int(m), second=0, microsecond=0)
    best = d.timestamp() + float(s)
    for off in (-86400, 0, 86400):
        c = best + off
        if abs(c - ref_epoch) < abs(best - ref_epoch):
            best = c
    return best


def analyse_launch(name: str, arm: str, log_text: str, meta: Dict[str, str],
                   rows: Optional[List[Tuple[int, float]]], dumps: Dict[str, str],
                   bag_expected: bool = True) -> Dict:
    """Pure function: everything the table row and the verdict need for one launch."""
    p = parse_log(log_text)
    invalid: List[str] = []
    if not p['watchdog_on']:
        invalid.append('no stall-watchdog banner')
    if not p['shm_clean']:
        invalid.append('no shm-clean line')
    if arm == 'A' and not p['shm_on']:
        invalid.append('no "bridge shared memory on"')
    if arm == 'B' and not p['shm_off']:
        invalid.append('no "bridge shared memory off"')
    if not p['fw_ok']:
        invalid.append('no BRIDGE_FW_CHECK OK')
    maps = meta.get('fastrtps_maps')
    if maps is None or not maps.lstrip('-').isdigit():
        invalid.append('manipulation check missing')
    elif arm == 'A' and int(maps) <= 0:
        invalid.append('manipulation check failed (A has no fastrtps maps)')
    elif arm == 'B' and int(maps) != 0:
        invalid.append('manipulation check failed (B maps fastrtps)')
    if bag_expected:
        if rows is None:
            invalid.append('no readable bag')
        elif not rows:
            invalid.append('empty /robot_state')

    if rows is not None and rows:
        gaps = classify_gaps(rows, STALL_THRESHOLD_MS)
        n50, max_gap = bridge_gap_stats(rows)
    else:
        gaps = {'bridge': [], 'recorder': []}
        n50, max_gap = 0, 0.0
    ref = float(meta.get('start_epoch', '0') or 0) or (rows[0][0] / 1e9 if rows else time.time())

    stalls, soft = [], []
    for ev in p['stalls']:
        if ev['gap_ms'] < SOFT_WARN_MS:
            continue
        if ev['gap_ms'] < STALL_THRESHOLD_MS:
            soft.append(ev)
            continue
        t_warn = hms_to_epoch(ev['hms'], ref)
        if rows is None:
            confirmed: Optional[bool] = None
        else:
            confirmed = any(abs(t - t_warn) <= CONFIRM_WINDOW_S for t, _g in gaps['bridge'])
        shape = '-'
        if ev['path'] in dumps:
            shape = classify_dump(dumps[ev['path']])
        stalls.append({'hms': ev['hms'], 'gap_ms': ev['gap_ms'], 'confirmed': confirmed,
                       'shape': shape, 'path': ev['path']})
    valid = not invalid
    # A counts toward an arm only when valid, confirmed and P-shaped.
    p_stalls = [s for s in stalls if s['confirmed'] and s['shape'] == 'P']
    return {
        'name': name, 'arm': arm, 'valid': valid, 'invalid': invalid, 'meta': meta,
        'n_msgs': None if rows is None else len(rows),
        'stalls': stalls, 'soft_warns': soft,
        'p_stalls': len(p_stalls) if valid else 0,
        'k_stalls': sum(1 for s in stalls if s['confirmed'] and s['shape'] == 'K') if valid else 0,
        'o_stalls': sum(1 for s in stalls if s['confirmed'] and s['shape'] == 'O') if valid else 0,
        'unconfirmed': sum(1 for s in stalls if s['confirmed'] is not True),
        'recorder_gaps': len(gaps['recorder']),
        'n_gaps_50': n50, 'max_gap_ms': max_gap,
        'setpoint_stale': p['setpoint_stale'], 'traceback': p['traceback'],
        'errors_other': p['errors_other'], 'bag': p['bag'],
    }


# ── statistics and verdict ───────────────────────────────────────────────────────────────

def fisher_one_sided(a_hit: int, a_n: int, b_hit: int, b_n: int) -> float:
    """P(A has >= a_hit of the total hits | margins), hypergeometric, one-sided (A > B)."""
    hits, n = a_hit + b_hit, a_n + b_n
    if hits == 0 or a_n == 0 or b_n == 0:
        return 1.0
    denom = math.comb(n, hits)
    return sum(math.comb(a_n, k) * math.comb(b_n, hits - k)
               for k in range(a_hit, min(a_n, hits) + 1)) / denom


def mann_whitney(a: Sequence[float], b: Sequence[float]) -> str:
    if len(a) < 2 or len(b) < 2:
        return 'n/a (too few launches)'
    try:
        from scipy.stats import mannwhitneyu  # type: ignore
        r = mannwhitneyu(a, b, alternative='two-sided')
        return 'U=%.1f p=%.3f' % (r.statistic, r.pvalue)
    except Exception as e:  # noqa: BLE001
        return 'n/a (%s)' % type(e).__name__


def verdict(n_a: int, n_b: int, a_p: int, b_p: int) -> str:
    """The spec's asymmetric reading. n_* = VALID launches per arm, *_p = P-shape confirmed
    stalls (K stalls count for neither arm)."""
    if b_p >= 1:
        return ('REFUTES "bridge shm is necessary" (decisive): %d P-shape stall(s) in arm B '
                '(bridge_shm:=false).' % b_p)
    if a_p >= 2:
        p = fisher_one_sided(a_p, n_a, b_p, n_b)
        sig = 'significant (one-sided Fisher p=%.3f)' % p if p < 0.05 else \
            'NOT yet significant (one-sided Fisher p=%.3f; needs about 4+ of 15)' % p
        return ('SUPPORTS shm as the cause: A=%d/%d, B=0/%d, %s. Fix candidate: bridge_shm:=false '
                'as the default.' % (a_p, n_a, n_b, sig))
    if a_p == 0 and n_a >= 15:
        return ('A=0 after %d valid launches: rate < 0.1 per launch (rule of three), shm_clean '
                'probably fixed it. Close only after 10 more clean real launches.' % n_a)
    if a_p == 0 and n_a >= 10:
        return ('POSITIVE CONTROL FAILED: A=0 after %d valid launches. Extend both arms to 15.' % n_a)
    if a_p == 0:
        return 'A=0 after %d valid launches: too few to read; continue to 10.' % n_a
    return ('INCONCLUSIVE: A=%d, B=0 with %d/%d valid launches; extend toward 15 per arm.'
            % (a_p, n_a, n_b))


# ── report ────────────────────────────────────────────────────────────────────────────────

def _fmt_stalls(rec: Dict) -> str:
    if not rec['stalls']:
        return '-'
    return '; '.join('%s %.0fms %s%s' % (s['hms'], s['gap_ms'], s['shape'],
                                         '' if s['confirmed'] else (' unconf' if s['confirmed'] is None else ' NOT-BAG'))
                     for s in rec['stalls'])


def render_report(date: str, records: List[Dict], header: List[str]) -> str:
    L = ['# BUG-129 launch series %s' % date, '']
    L += header + ['']
    L += ['| launch | valid | why invalid | fastrtps maps | /dev/shm | load1 | msgs | stalls (>=250 ms: time gap shape) '
          '| 150-250 ms WARNs | recorder-side gaps | bridge gaps >50 ms | max bridge gap ms | SETPOINT_STALE '
          '| Traceback | other ERROR | bridge exit | live_session@30s |',
          '|' + '---|' * 17]
    for r in records:
        m = r['meta']
        L.append('| %s | %s | %s | %s | %s | %s | %s | %s | %d | %d | %d | %.0f | %d | %d | %d | %s | %s |' % (
            r['name'], 'yes' if r['valid'] else 'NO', '; '.join(r['invalid']) or '-',
            m.get('fastrtps_maps', '?'), m.get('shm_files_t60', '?'), m.get('load1_before', '?'),
            '?' if r['n_msgs'] is None else r['n_msgs'], _fmt_stalls(r), len(r['soft_warns']),
            r['recorder_gaps'], r['n_gaps_50'], r['max_gap_ms'], r['setpoint_stale'], r['traceback'],
            r['errors_other'], m.get('bridge_exit', '?'), m.get('live_session_30s', '-')))
    valid = [r for r in records if r['valid']]
    by = {arm: [r for r in valid if r['arm'] == arm] for arm in 'AB'}
    n_a, n_b = len(by['A']), len(by['B'])
    a_p = sum(r['p_stalls'] for r in by['A'])
    b_p = sum(r['p_stalls'] for r in by['B'])
    k = {arm: sum(r['k_stalls'] for r in by[arm]) for arm in 'AB'}
    o = {arm: sum(r['o_stalls'] for r in by[arm]) for arm in 'AB'}
    unc = sum(r['unconfirmed'] for r in valid)
    L += ['',
          '## Summary', '',
          '- valid launches: A=%d, B=%d (invalid: %d)' % (n_a, n_b, len(records) - len(valid)),
          '- P-shape confirmed stalls: A=%d, B=%d' % (a_p, b_p),
          '- K-shape (count for neither arm): A=%d, B=%d; O-shape: A=%d, B=%d' % (k['A'], k['B'], o['A'], o['B']),
          '- WARN >=250 ms not confirmed by a bridge-side bag gap (or no bag): %d' % unc,
          '- bridge-side gaps >50 ms per launch, Mann-Whitney A vs B: %s' % mann_whitney(
              [r['n_gaps_50'] for r in by['A']], [r['n_gaps_50'] for r in by['B']]),
          '- largest bridge-side gap per launch, Mann-Whitney A vs B: %s' % mann_whitney(
              [r['max_gap_ms'] for r in by['A']], [r['max_gap_ms'] for r in by['B']]),
          '', '## Verdict', '', verdict(n_a, n_b, a_p, b_p), '']
    return '\n'.join(L)


def collect(main: str, date: str, use_bags: bool = True) -> Tuple[List[Dict], List[str]]:
    logs_dir = os.path.join(main, 'temp', 'logs')
    records = []
    for path in sorted(glob.glob(os.path.join(logs_dir, 'bug129_series_%s_*.log' % date))):
        m = LOG_NAME_RE.match(os.path.basename(path))
        if not m:
            continue
        name = os.path.basename(path)[:-4].replace('bug129_series_%s_' % date, '')
        with open(path, errors='replace') as fh:
            text = fh.read()
        meta_path = path[:-4] + '.meta'
        meta = parse_meta(open(meta_path).read()) if os.path.exists(meta_path) else {}
        parsed = parse_log(text)
        dumps = {}
        for ev in parsed['stalls']:
            if os.path.isfile(ev['path']):
                with open(ev['path'], errors='replace') as fh:
                    dumps[ev['path']] = fh.read()
        rows = None
        if use_bags and parsed['bag']:
            rows = read_bag_rows(parsed['bag'])
        records.append(analyse_launch(name, m.group('arm'), text, meta, rows, dumps, bag_expected=use_bags))
    header = []
    series_log = os.path.join(logs_dir, 'bug129_series_%s_series.log' % date)
    if os.path.exists(series_log):
        header.append('Runner log (`%s`), first lines:' % series_log)
        header.append('')
        header += ['    ' + ln for ln in open(series_log).read().splitlines()[:12]]
    return records, header


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--date', default=time.strftime('%Y-%m-%d'))
    ap.add_argument('--main', default=os.path.expanduser('~/Desktop/Jugglebot'))
    ap.add_argument('--no-bags', action='store_true', help='skip bag reading (bag checks then not validity rules)')
    ap.add_argument('--out', default=None)
    a = ap.parse_args(argv)
    records, header = collect(a.main, a.date, use_bags=not a.no_bags)
    if not records:
        print('no launches found for %s under %s/temp/logs' % (a.date, a.main), file=sys.stderr)
        return 1
    out = a.out or os.path.join(a.main, 'temp', 'reports', 'bug129_series_%s.md' % a.date)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    text = render_report(a.date, records, header)
    with open(out, 'w') as fh:
        fh.write(text)
    print(out)
    print(text.split('## Verdict')[-1].strip())
    return 0


if __name__ == '__main__':
    sys.exit(main())
