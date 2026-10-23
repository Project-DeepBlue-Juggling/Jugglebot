"""mapkit gen: idempotency, --check exit codes, cell escaping, row order (tmp_path repo)."""
from __future__ import annotations

import os

from mapkit.__main__ import main
from mapkit.gen_index import cell, render_all


def _w(root, board, name, text):
    d = root / "plans" / board
    d.mkdir(parents=True, exist_ok=True)
    (d / name).write_text(text, encoding="utf-8")


def _repo(tmp_path):
    _w(tmp_path, "active", "a-old.md",
       "---\ntitle: A\nstatus: active\nlast_updated: 2026-01-01\npathway: x\nscope: has | pipe\n---\nbody\n")
    _w(tmp_path, "active", "b-new.md",
       "---\ntitle: B\nstatus: active\nlast_updated: 2026-05-01\npathway: x\nscope: >-\n  line one\n  line two\n---\nbody\n")
    _w(tmp_path, "active", "ref.md",
       "---\ntitle: R\nstatus: active\nkind: reference\nlast_updated: 2026-09-09\nscope: doc\n---\nbody\n")
    _w(tmp_path, "parked", "p.md",
       "---\ntitle: P\nstatus: parked\nparked: 2026-02-02\nparked_why: why\nunpark: when\npathway: x\n---\nbody\n")
    _w(tmp_path, "archived", "z-done.md", "---\ntitle: Z\nstatus: complete\narchived: 2026-03-03\n---\nbody\n")
    _w(tmp_path, "archived", "y-nostatus.md", "---\ntitle: Y\narchived: 2026-03-03\n---\nbody\n")
    return tmp_path


def test_second_run_changes_nothing(tmp_path, capsys):
    repo = _repo(tmp_path)
    assert main(["gen", "--repo", str(repo)]) == 0
    out1 = capsys.readouterr().out
    assert out1.count("wrote ") == 3
    before = {b: (repo / "plans" / b / "INDEX.md").read_text() for b in ("active", "parked", "archived")}
    assert main(["gen", "--repo", str(repo)]) == 0
    assert capsys.readouterr().out == ""
    after = {b: (repo / "plans" / b / "INDEX.md").read_text() for b in before}
    assert before == after


def test_check_exit_codes(tmp_path, capsys):
    repo = _repo(tmp_path)
    assert main(["gen", "--repo", str(repo), "--check"]) == 1
    assert not (repo / "plans" / "active" / "INDEX.md").exists()
    main(["gen", "--repo", str(repo)])
    assert main(["gen", "--repo", str(repo), "--check"]) == 0


def test_escaping_and_collapse(tmp_path):
    text = render_all(str(_repo(tmp_path)))["active"]
    assert "has \\| pipe" in text
    assert "line one line two" in text
    assert cell(None) == "—" and cell("a\nb") == "a b"


def test_row_order_and_statuses(tmp_path):
    out = render_all(str(_repo(tmp_path)))
    rows = [l for l in out["active"].splitlines() if l.startswith("| [")]
    assert [r.split("]")[0] for r in rows] == ["| [b-new.md", "| [a-old.md", "| [ref.md"]
    assert "| reference |" in rows[2]
    arch = [l for l in out["archived"].splitlines() if l.startswith("| 2026")]
    assert "y-nostatus.md" in arch[0] and "—" in arch[0]
    assert "z-done.md" in arch[1] and "| completed |" in arch[1]
