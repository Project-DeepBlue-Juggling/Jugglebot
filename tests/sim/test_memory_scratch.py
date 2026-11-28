"""tools/memory_scratch.py: scratch copy of a plant's Memory (E2(c)).

Parallel-safe: tmp_path only. The tool is imported by path and sys.path /
os.environ are left untouched.
"""
from __future__ import annotations

import ast
import hashlib
import importlib.util
import os

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
NODE_PKG = os.path.join(REPO, 'ros_ws', 'src', 'jugglebot', 'jugglebot')


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


ms = _load('_memory_scratch_under_test', os.path.join(REPO, 'tools', 'memory_scratch.py'))


def _sha(path):
    with open(path, 'rb') as h:
        return hashlib.sha256(h.read()).hexdigest()


@pytest.fixture
def root(tmp_path):
    src = tmp_path / 'jugglebot'
    src.mkdir()
    (src / 'memory_v2.csv').write_bytes(b'a,b\n1,2\n3,4\n5,6\n')
    (src / 'memory_v2.csv.seed').write_bytes(b'{"rows": 3}\n')
    return str(tmp_path)


def test_copy_identical_and_source_untouched(root, capsys):
    src = os.path.join(root, 'jugglebot')
    before = {n: (os.path.getsize(os.path.join(src, n)), _sha(os.path.join(src, n)))
              for n in os.listdir(src)}
    assert ms.main(['exp1', '--root', root]) == 0
    dst = os.path.join(root, 'exp1')
    assert sorted(os.listdir(dst)) == sorted(os.listdir(src))
    assert _sha(os.path.join(dst, 'memory_v2.csv')) == _sha(os.path.join(src, 'memory_v2.csv'))
    for n in os.listdir(src):
        assert before[n] == (os.path.getsize(os.path.join(src, n)),
                             _sha(os.path.join(src, n)))
    out = capsys.readouterr().out
    assert 'rows copied: 3' in out
    assert 'plant_id exp1' in out and 'plant_id jugglebot' in out
    assert [n for n in os.listdir(root) if n.startswith('.')] == []


def test_scratch_seed_is_empty_and_the_debrief_merge_accepts_it(root):
    # The main plant's seed carries the main tracked file's sha256. Copied to a scratch plant, the
    # debrief merge (tools/runtime_records.py) finds no tracked file for that plant and refuses.
    # The scratch plant therefore gets the empty seed write_seed gives an absent tracked file.
    import json
    import sys
    assert ms.main(['exp1', '--root', root]) == 0
    dst = os.path.join(root, 'exp1', 'memory_v2.csv')
    with open(dst + '.seed') as h:
        seed = json.load(h)
    assert seed['sha256'] is None and seed['rows'] == 0
    assert seed['tracked'].endswith(os.path.join('learn', 'exp1', 'memory_v2.csv'))
    saved = list(sys.path)
    try:
        rr = _load('_runtime_records_for_scratch', os.path.join(REPO, 'tools', 'runtime_records.py'))
    finally:
        sys.path[:] = saved
    with open(dst, 'rb') as h:
        runtime = h.read()
    assert rr.append_memory(None, runtime, seed) == runtime      # every row archived, no Refusal
    with open(os.path.join(root, 'jugglebot', 'memory_v2.csv.seed')) as h:
        main_seed = dict(json.load(h), sha256='0' * 64)
    with pytest.raises(rr.Refusal):                              # what a copied seed would do
        rr.append_memory(None, runtime, main_seed)


@pytest.mark.parametrize('args', [
    ['jugglebot'],                 # equals source
    ['nope', '--from', 'ghost'],   # source missing
    ['../evil'], ['a/b'], ['..'], ['.'],
])
def test_refusals_write_nothing(root, args, capsys):
    names = sorted(os.listdir(root))
    assert ms.main(args + ['--root', root]) == 1
    assert 'refused' in capsys.readouterr().err
    assert sorted(os.listdir(root)) == names


def test_existing_scratch_refused_and_kept(root):
    assert ms.main(['exp1', '--root', root]) == 0
    marker = os.path.join(root, 'exp1', 'memory_v2.csv')
    with open(marker, 'ab') as h:
        h.write(b'7,8\n')
    assert ms.main(['exp1', '--root', root]) == 1
    assert _sha(marker) != _sha(os.path.join(root, 'jugglebot', 'memory_v2.csv'))


def test_list(root, capsys):
    assert ms.main(['--list', '--root', root]) == 0
    assert 'jugglebot' in capsys.readouterr().out


def test_default_root_agrees_with_node(tmp_path):
    rtp = _load('_rtp_under_test', os.path.join(NODE_PKG, 'runtime_paths.py'))
    # The node's layout under a tmp root (the resolver refuses a real
    # checkout under pytest); the tool's default must be the same layout
    # relative to the repo root.
    node = rtp.runtime_memory_path(str(tmp_path), 'x')
    node_rel = os.path.relpath(os.path.dirname(os.path.dirname(node)), str(tmp_path))
    assert os.path.relpath(ms.DEFAULT_ROOT, REPO) == node_rel
    assert ms.MEMORY_FILE == os.path.basename(node)


def test_plant_id_rule_matches_memory_source():
    tree = ast.parse(open(os.path.join(NODE_PKG, 'motion', 'skills', 'memory.py')).read())
    fn = [n for n in ast.walk(tree)
          if isinstance(n, ast.FunctionDef) and n.name == '_check_plant_id'][0]
    mine = [n for n in ast.walk(ast.parse(open(ms.__file__).read()))
            if isinstance(n, ast.FunctionDef) and n.name == 'check_plant_id'][0]

    def consts(f):
        return sorted(repr(n.value) for n in ast.walk(f)
                      if isinstance(n, ast.Constant) and isinstance(n.value, str)
                      and len(n.value) < 5)
    assert consts(fn) == consts(mine)
