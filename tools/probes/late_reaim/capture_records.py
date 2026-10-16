"""Capture the plan records a sim columns run installs, for the late-catch
re-aim bump prototype (``bump_replay.py``).

Runs ``sim/skills_gate.py``'s columns gate at the live session limits with
``executor.install_segment`` wrapped, and pickles every install attempt as
``(t_now_s, kind, accepted, record_after, event_abs_s, limits, geom)``.
Output: ``temp/probes/late_reaim/records_<seed>.pkl``.

    python tools/probes/late_reaim/capture_records.py [--seed 0] [--throws 8]
"""
import argparse, os, pickle, sys
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path[:0] = [ROOT, os.path.join(ROOT, 'ros_ws', 'src', 'jugglebot'),
                os.path.join(ROOT, 'sim')]
from jugglebot.motion.skills import executor as ex  # noqa: E402
import skills_gate as sgate  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--throws', type=int, default=8)
    ap.add_argument('--apex', type=float, default=0.95)
    ap.add_argument('--sep', type=float, default=125.0)
    ap.add_argument('--tag', default='columns')
    a = ap.parse_args(argv)
    log = []
    real = ex.install_segment

    def wrapped(record, seed_rest, kind, terminal, t_now_s, *args, **kw):
        new, res, seg = real(record, seed_rest, kind, terminal, t_now_s, *args, **kw)
        limits = args[0] if len(args) > 0 else kw.get('limits')
        geom = args[1] if len(args) > 1 else kw.get('geom')
        ev = (float(res.t0_s) + float(res.event_t_s)) if (res.accepted and res.event_t_s is not None and res.t0_s is not None) else None
        log.append(dict(t_now_s=float(t_now_s), kind=kind, accepted=res.accepted,
                        code=res.code, record=new, event_abs_s=ev,
                        limits=limits, geom=geom))
        return new, res, seg
    ex.install_segment = wrapped
    cfg = sgate.SkillsGateConfig(n_throws=a.throws, seeds=(a.seed,), apex_m=a.apex, separation_mm=a.sep)
    rep = sgate.run_gate(cfg)
    out = os.path.join(ROOT, 'temp', 'probes', 'late_reaim', 'records_%s_%d.pkl' % (a.tag, a.seed))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'wb') as f:
        pickle.dump(log, f)
    print('captured %d installs (%d CATCH accepted) -> %s'
          % (len(log), sum(1 for r in log if r['kind'] == ex.CATCH and r['accepted']), out))


if __name__ == '__main__':
    main()
