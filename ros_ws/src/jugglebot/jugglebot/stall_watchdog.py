"""BUG-129 stall watchdog: catch a whole-process freeze red-handed (pure Python, no ROS).

Question it answers: when ``teensy_bridge_node`` stops for about 340 ms (four
one-off freezes 2026-11-18..22, at most one per launch), WHICH Python thread is
inside WHICH call? A bag can only show that every thread stopped; it cannot name
the holder. ``faulthandler.dump_traceback_later`` can: its timer is a C thread
that needs no GIL, so when nobody re-arms it within ``timeout_s`` it writes the
stack of EVERY Python thread, including the one that holds the GIL in a blocking
C call, while the freeze is still in progress.

Design (each choice closes a way the instrument could itself hurt the stream):

* Re-armed from its OWN daemon thread every ``period_s`` (20 Hz), never from the
  setpoint thread: nothing is added to the setpoint path. A whole-process freeze
  stops this thread too (it needs the GIL), so the timer runs out. A freeze of
  the watched loop alone (``heartbeat`` age past ``period_s``) also stops the
  re-arm, so a setpoint-thread-only stall is dumped as well.
* The timer writes to an anonymous file on tmpfs (``/dev/shm``). Re-arming cancels
  the previous timer, and CPython's cancel waits for a dump in progress WITH THE
  GIL HELD; a dump into a slow disk could therefore stall the whole process.
  A tmpfs write is a memory copy, so that wait is bounded by microseconds.
* The dump is copied to ``out_dir`` (``temp/logs/``) by this thread with plain
  Python file I/O, which releases the GIL around ``write()``: a slow disk delays
  only this thread. Reports are capped at ``max_reports`` files per process.
* Shutdown (:meth:`stop`) cancels the timer BEFORE closing the file, under the
  same lock the re-arm takes: the C thread can never write into a closed (and
  possibly reused) file descriptor.

Process-global: faulthandler supports one ``dump_traceback_later`` timer per
process, so at most one :class:`StallWatchdog` may run per process.

Logbook: logbook/2026-11-23-bridge-stall-watchdog.md
"""
from __future__ import annotations

import faulthandler
import os
import tempfile
import threading
import time
from typing import Callable, Optional

DEFAULT_TIMEOUT_S = 0.15
DEFAULT_PERIOD_S = 0.05
DEFAULT_MAX_REPORTS = 20
TMPFS_DIR = '/dev/shm'


def _tmp_dir() -> Optional[str]:
    return TMPFS_DIR if os.path.isdir(TMPFS_DIR) and os.access(TMPFS_DIR, os.W_OK) else None


class StallWatchdog:
    """Dump every Python thread's stack when the process (or a watched loop) stalls.

    ``heartbeat``: optional callable returning the age in seconds of the watched
    loop's last tick, or ``None`` while that loop is not running. Called from the
    watchdog thread only; it must not block or take a lock the watched loop takes.
    ``log_warn``: called with one line after each caught stall (on the watchdog
    thread, after recovery).
    """

    def __init__(self, out_dir: str, tag: str,
                 timeout_s: float = DEFAULT_TIMEOUT_S,
                 period_s: float = DEFAULT_PERIOD_S,
                 heartbeat: Optional[Callable[[], Optional[float]]] = None,
                 log_warn: Optional[Callable[[str], None]] = None,
                 max_reports: int = DEFAULT_MAX_REPORTS,
                 names_every: int = 20,
                 clock: Callable[[], float] = time.monotonic):
        if not (timeout_s > 2.0 * period_s > 0.0):
            raise ValueError('timeout_s must exceed 2 x period_s > 0 '
                             f'(got timeout_s={timeout_s}, period_s={period_s})')
        self.out_dir = str(out_dir)
        self.tag = str(tag)
        self.timeout_s = float(timeout_s)
        self.period_s = float(period_s)
        self._heartbeat = heartbeat
        self._hb_failed = False
        self._log_warn = log_warn or (lambda _m: None)
        self.max_reports = int(max_reports)
        self._clock = clock
        self._lock = threading.Lock()      # re-arm vs stop; never seen by the watched loop
        self._stop = threading.Event()
        self._closed = False
        self._file = None
        self._thread: Optional[threading.Thread] = None
        self._reported = 0                 # bytes of the tmpfs file already reported
        self._pending = -1                 # size seen last tick while a dump was growing
        self._last_tick: Optional[float] = None
        self._max_gap_s = 0.0              # largest own-tick gap / heartbeat age since last report
        self._names = {}                   # thread ident -> name, refreshed every names_every ticks
        self._names_every = max(1, int(names_every))
        self._ticks = 0
        self.stalls = 0                    # dumps caught
        self.reports = []                  # paths written
        self.tmp_dir = None

    # ── lifecycle ────────────────────────────────────────────────────────
    def start(self) -> 'StallWatchdog':
        self.tmp_dir = _tmp_dir()
        # Anonymous (unlinked) file: nothing is left behind if the process dies.
        self._file = tempfile.TemporaryFile(mode='w+b', dir=self.tmp_dir)
        self._arm()
        self._last_tick = self._clock()
        self._thread = threading.Thread(target=self._run, name='stall_watchdog', daemon=True)
        self._thread.start()
        return self

    def stop(self, join_timeout_s: float = 1.0) -> None:
        """Idempotent. Cancel the timer first, then close the file."""
        self._stop.set()
        t = self._thread
        if t is not None and t is not threading.current_thread():
            t.join(timeout=join_timeout_s)
        with self._lock:
            if self._closed:
                return
            self._closed = True
            faulthandler.cancel_dump_traceback_later()
            if self._file is not None:
                try:
                    self._file.close()
                except OSError:
                    pass

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive() and not self._closed

    # ── internals ────────────────────────────────────────────────────────
    def _arm(self) -> None:
        with self._lock:
            if self._closed:
                return
            faulthandler.dump_traceback_later(self.timeout_s, repeat=False, file=self._file)

    def _run(self) -> None:
        while not self._stop.wait(self.period_s):
            try:
                self.tick()
            except Exception as e:  # noqa: BLE001 - a diagnostic must never die loudly
                try:
                    self._log_warn(f'stall watchdog {self.tag}: tick failed: {e!r}')
                except Exception:  # noqa: BLE001
                    pass

    def tick(self) -> None:
        """One watchdog iteration (public for tests): report a finished dump, then re-arm."""
        now = self._clock()
        gap = 0.0 if self._last_tick is None else now - self._last_tick
        self._last_tick = now
        if self._ticks % self._names_every == 0:
            # Names of threads alive BEFORE a stall (one that ends during it is
            # still named in the report); 1 Hz at the default period.
            self._names.update((t.ident, t.name) for t in threading.enumerate())
        self._ticks += 1
        hb = None
        if self._heartbeat is not None:
            try:
                hb = self._heartbeat()
            except Exception as e:  # noqa: BLE001 - unknown heartbeat: still re-arm
                if not self._hb_failed:
                    self._hb_failed = True   # log the first failure only (20 Hz thread)
                    self._log_warn(f'stall watchdog {self.tag}: heartbeat raised {e!r}; '
                                   'treating it as unknown')
        self._max_gap_s = max(self._max_gap_s, gap, hb or 0.0)
        size = os.fstat(self._file.fileno()).st_size
        if size > self._reported:
            if size != self._pending:
                # A dump appeared or is still growing: report once it is stable.
                self._pending = size
            else:
                self._report(size)
                self._pending = -1
        if hb is not None and hb > self.period_s:
            return  # the watched loop is stuck: let the timer run out
        self._arm()

    def _report(self, size: int) -> None:
        self.stalls += 1
        dump = os.pread(self._file.fileno(), size - self._reported, self._reported)
        self._reported = size
        gap_ms = self._max_gap_s * 1e3
        self._max_gap_s = 0.0
        path = None
        if len(self.reports) < self.max_reports:
            names = dict(self._names)
            names.update((t.ident, t.name) for t in threading.enumerate())
            stamp = time.strftime('%Y-%m-%dT%H-%M-%S')
            path = os.path.join(self.out_dir, f'stall_{self.tag}_{stamp}_{os.getpid()}_{self.stalls}.txt')
            os.makedirs(self.out_dir, exist_ok=True)
            with open(path, 'wb') as f:
                f.write((f'# {self.tag} stall #{self.stalls}, pid {os.getpid()}, '
                         f'reported {time.strftime("%Y-%m-%d %H:%M:%S")}\n'
                         f'# watchdog timeout {self.timeout_s * 1e3:.0f} ms (stacks taken that long '
                         f'after the last re-arm); largest gap seen {gap_ms:.1f} ms\n'
                         '# thread ids -> names (threads seen in the last second, and at report time):\n').encode())
                for ident, name in sorted(names.items(), key=lambda kv: kv[1] or ''):
                    if ident is not None:
                        f.write(f'#   0x{ident:016x} {name}\n'.encode())
                f.write(dump)
            self.reports.append(path)
        where = path if path else f'(not written: cap of {self.max_reports} reports reached)'
        self._log_warn(
            f'stall watchdog: {self.tag} stalled (largest gap {gap_ms:.1f} ms, timeout '
            f'{self.timeout_s * 1e3:.0f} ms); every Python thread\'s stack, taken mid-stall: {where}')


def measure_rearm_cost(n: int = 2000, timeout_s: float = DEFAULT_TIMEOUT_S) -> float:
    """Mean seconds per ``dump_traceback_later`` re-arm (cancel + new C thread)."""
    with tempfile.TemporaryFile(dir=_tmp_dir()) as f:
        t0 = time.perf_counter()
        for _ in range(n):
            faulthandler.dump_traceback_later(timeout_s, repeat=False, file=f)
        dt = time.perf_counter() - t0
        faulthandler.cancel_dump_traceback_later()
    return dt / n
