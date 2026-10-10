"""Oracle chunks as JSON for the node harnesses (the browser no longer reads msgpack).

NOT collected by pytest. ``export_chunks`` decodes the converter's ``chunk-*.msgpack.gz``
(the Python oracle) with msgpack in the venv and writes ``chunk-NNNNN.json`` beside a copy
of ``manifest.json``/``overview.json``; non-finite floats are tagged ``{"$f": "nan"|"inf"|"-inf"}``
because JSON cannot carry them (``replay_test_support.js::revive`` undoes it).
"""
from __future__ import annotations

import gzip
import json
import math
import shutil
from pathlib import Path

import msgpack


def tagged(v):
    if isinstance(v, float) and not math.isfinite(v):
        return {"$f": "nan" if math.isnan(v) else ("inf" if v > 0 else "-inf")}
    if isinstance(v, dict):
        return {k: tagged(x) for k, x in v.items()}
    if isinstance(v, list):
        return [tagged(x) for x in v]
    return v


def export_chunks(cache, dest) -> int:
    cache, dest = Path(cache), Path(dest)
    dest.mkdir(parents=True, exist_ok=True)
    for name in ("manifest.json", "overview.json"):
        if (cache / name).exists():
            shutil.copy(cache / name, dest / name)
    n = 0
    for f in sorted(cache.glob("chunk-*.msgpack.gz")):
        rec = msgpack.unpackb(gzip.decompress(f.read_bytes()), raw=False)
        (dest / (f.name[: -len(".msgpack.gz")] + ".json")).write_text(json.dumps(tagged(rec)))
        n += 1
    return n
