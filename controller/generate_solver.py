"""Ahead-of-time compile the MPC NLP solver.

Run this script once on the target machine (typically the Jetson) after any
change to ``controller/mpc.py``, ``controller/params.py``, or
``config/hardware_config.yaml``.  The output is a shared library
(``controller/generated/mpc_gen.so``) that ``MPCController._build_problem``
loads directly, eliminating the 27–100ms first-solve JIT penalty we observed
in hardware sessions (see logbook entries on cold-start saturation).

Usage::

    python controller/generate_solver.py

After a successful run, ``controller/generated/`` will contain:
  * ``mpc_gen.c``   — CasADi-generated C source
  * ``mpc_gen.so``  — compiled shared library (arm64 on Jetson)
  * ``mpc_gen.hash`` — sha256 of everything that affects the NLP structure,
                       checked at runtime to catch stale binaries

Staleness handling at runtime (see ``MPCController._build_problem``):
  * ``.so`` present + hash matches    →  load (production path, fast start)
  * ``.so`` present + hash mismatches →  hard-fail with clear error message
  * ``.so`` absent                    →  fall back to in-process build
                                         (dev / test convenience)

The compile takes ~15–30s on a Jetson Orin Nano, producing a ~1–3 MB .so.
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def _project_root() -> Path:
    return Path(__file__).resolve().parent.parent


def _generated_dir() -> Path:
    return Path(__file__).resolve().parent / 'generated'


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--keep-c', action='store_true',
        help="Keep the generated .c file after compilation (default: remove)")
    parser.add_argument(
        '--verbose', '-v', action='store_true',
        help="Print gcc command and extra diagnostics")
    args = parser.parse_args(argv)

    # Ensure the project root is importable (when run as a script, __main__
    # has no package context).
    sys.path.insert(0, str(_project_root()))

    # Late imports so the --help message works without casadi/mujoco.
    import casadi as cs  # noqa: F401 — required for the MPC build
    from controller.params import MPCParams
    from controller.mpc import MPCController

    # Lazy import: MuJoCo plant provides geometry consistent with
    # hardware_config.yaml.  We build against the MuJoCo geometry because
    # HardwarePlant and MuJoCoPlant both derive their geom from the same
    # config, so the NLP is structurally identical.
    from sim.plant.mujoco_plant import MuJoCoPlant

    out_dir = _generated_dir()
    out_dir.mkdir(parents=True, exist_ok=True)
    c_path = out_dir / 'mpc_gen.c'
    so_path = out_dir / 'mpc_gen.so'
    hash_path = out_dir / 'mpc_gen.hash'

    # Remove any existing .so / .hash BEFORE constructing MPCController.
    # Otherwise the staleness check in _load_nlpsol would trip the very
    # first time we try to build the NLP (we are regenerating precisely
    # because the old .so is out of date).
    for stale in (so_path, hash_path):
        if stale.exists():
            stale.unlink()

    # Construct the MPC with prime_solver disabled — we don't want a warmup
    # solve during generation.
    plant = MuJoCoPlant()
    params = MPCParams(prime_solver=False)
    mpc = MPCController.from_plant(params, plant)

    # Emit C code for the NLP callbacks (cost, constraints, Jacobian,
    # Hessian).  CasADi routes the generator output through the file we
    # pass in — it writes relative to cwd, so we chdir to out_dir.
    print(f"Generating C code -> {c_path} ...")
    cwd = Path.cwd()
    try:
        import os
        os.chdir(out_dir)
        mpc._solver.generate_dependencies('mpc_gen.c')
    finally:
        os.chdir(cwd)

    if not c_path.exists():
        print(f"error: expected {c_path} to exist after generate_dependencies",
              file=sys.stderr)
        return 2

    # Compile.  -O3 matches typical AOT workflows; -fPIC is required for a
    # shared library; -shared produces the .so.  -o forces the output
    # filename (bypassing gcc's default a.out).
    cmd = [
        'gcc', '-O3', '-fPIC', '-shared',
        str(c_path), '-o', str(so_path),
    ]
    if args.verbose:
        print("Compiling:", ' '.join(cmd))
    else:
        print(f"Compiling -> {so_path} ...")

    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print("gcc failed:", file=sys.stderr)
        print(result.stderr, file=sys.stderr)
        return result.returncode

    # Stamp the hash alongside the .so so MPCController can detect staleness.
    hash_hex = mpc._nlp_source_hash()
    hash_path.write_text(hash_hex)
    print(f"Hash: {hash_hex}")

    if not args.keep_c:
        c_path.unlink()

    size_kb = so_path.stat().st_size / 1024
    print(f"Success: {so_path} ({size_kb:.1f} KB)")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
