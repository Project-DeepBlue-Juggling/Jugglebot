#!/usr/bin/env python3
"""Render a Markdown document to a self-contained, styled HTML view.

Markdown is the single source of truth. The generated HTML is a disposable
*view* of it — never hand-edit the HTML; edit the ``.md`` and re-run this tool.

The HTML supports light and dark themes: it defaults to the operating system
preference and provides a toggle button (top-right) whose choice persists via
``localStorage``.

Usage:
    python tools/render_report.py FILE.md [FILE.md ...]
    python tools/render_report.py --all              # every plans/active/*.md
    python tools/render_report.py --out-dir DIR FILE.md

Output defaults to ``<repo>/temp/reports/<name>.html`` (a gitignored runtime
area). Each HTML file is fully self-contained (embedded CSS + JS) — openable
and shareable offline.

Requires: markdown, pygments, pyyaml — all present in the project venv
(``source ~/Desktop/PDJ_venv/venv/bin/activate``).
"""
from __future__ import annotations

import argparse
import datetime as _dt
import re
import sys
from pathlib import Path

try:
    import markdown as _markdown
except ImportError:
    sys.exit("error: the 'markdown' package is required — run `pip install markdown`")

from pygments.formatters import HtmlFormatter
from pygments.styles import get_all_styles
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUT_DIR = REPO_ROOT / "temp" / "reports"

# Status string -> badge colour. Keys are matched case-insensitively.
_STATUS_COLOURS = {
    "active": "#2563eb",
    "in progress": "#2563eb",
    "not started": "#64748b",
    "complete": "#16a34a",
    "completed": "#16a34a",
    "draft": "#64748b",
    "archived": "#64748b",
    "superseded": "#9333ea",
    "blocked": "#dc2626",
}

_CSS = """
/* ---- light theme (default) ---- */
:root {
  --fg:#1f2328; --muted:#656d76; --line:#d8dee4; --bg:#ffffff;
  --soft:#f6f8fa; --accent:#2563eb; --table-even:#fbfcfd;
  --code-bg:rgba(175,184,193,0.2);
  --note-bg:#fff8e6; --note-border:#f0e0a8; --note-fg:#6b5a1e;
  --note-code-bg:#f3e9c4;
}
/* ---- dark theme ---- */
:root[data-theme="dark"] {
  --fg:#e6edf3; --muted:#8b949e; --line:#30363d; --bg:#0d1117;
  --soft:#161b22; --accent:#4493f8; --table-even:#11161d;
  --code-bg:rgba(110,118,129,0.4);
  --note-bg:#2a2410; --note-border:#54481d; --note-fg:#d3c184;
  --note-code-bg:#3d3414;
}
* { box-sizing: border-box; }
html, body { margin: 0; padding: 0; }
body { background: var(--soft); color: var(--fg);
       font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto,
       Helvetica, Arial, sans-serif; line-height: 1.6; font-size: 16px;
       -webkit-font-smoothing: antialiased; }
.layout { display: grid; grid-template-columns: 270px minmax(0, 880px);
          gap: 0; max-width: 1150px; margin: 0 auto; }
.sidebar { border-right: 1px solid var(--line); background: var(--soft); }
.sidebar-inner { position: sticky; top: 0; max-height: 100vh; overflow-y: auto;
                 padding: 28px 20px; }
.toc-title { font-size: 12px; font-weight: 700; letter-spacing: 0.08em;
             text-transform: uppercase; color: var(--muted); margin-bottom: 10px; }
.toc ul { list-style: none; margin: 0; padding-left: 14px; }
.toc > ul { padding-left: 0; }
.toc li { margin: 3px 0; }
.toc a { color: var(--muted); text-decoration: none; font-size: 13.5px; }
.toc a:hover { color: var(--accent); }
.content { background: var(--bg); padding: 36px 48px 80px; min-height: 100vh; }
.doc-header { border-bottom: 1px solid var(--line); padding-bottom: 16px;
              margin-bottom: 8px; }
.doc-title { font-size: 30px; line-height: 1.25; margin: 0 0 10px; }
.doc-meta { display: flex; flex-wrap: wrap; gap: 14px; align-items: center;
            font-size: 13px; color: var(--muted); }
.badge { display: inline-block; padding: 2px 10px; border-radius: 999px;
         color: #fff; font-size: 11.5px; font-weight: 700;
         letter-spacing: 0.04em; text-transform: uppercase; }
.src { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
       font-size: 12px; }
.generated-note { background: var(--note-bg); border: 1px solid var(--note-border);
                  border-radius: 6px; padding: 8px 12px; font-size: 12.5px;
                  color: var(--note-fg); margin: 18px 0 26px; }
.generated-note code { background: var(--note-code-bg); }
.markdown-body h1 { font-size: 25px; margin-top: 36px; }
.markdown-body h2 { font-size: 21px; margin-top: 34px;
                    border-bottom: 1px solid var(--line); padding-bottom: 5px; }
.markdown-body h3 { font-size: 17.5px; margin-top: 26px; }
.markdown-body h4 { font-size: 15.5px; margin-top: 22px; }
.markdown-body a { color: var(--accent); }
.markdown-body table { border-collapse: collapse; width: 100%; margin: 18px 0;
                       font-size: 14px; display: block; overflow-x: auto; }
.markdown-body th, .markdown-body td { border: 1px solid var(--line);
                                       padding: 7px 11px; text-align: left; }
.markdown-body th { background: var(--soft); font-weight: 600; }
.markdown-body tr:nth-child(even) td { background: var(--table-even); }
.markdown-body code { background: var(--code-bg); padding: 0.15em 0.4em;
                      border-radius: 5px; font-size: 85%;
                      font-family: ui-monospace, SFMono-Regular, Menlo, Consolas,
                      monospace; }
.markdown-body pre { background: var(--soft); border: 1px solid var(--line);
                     border-radius: 8px; padding: 14px 16px; overflow-x: auto;
                     line-height: 1.45; }
.markdown-body .codehilite { border: 1px solid var(--line); border-radius: 8px;
                             overflow: hidden; margin: 16px 0; }
.markdown-body .codehilite pre { border: 0; border-radius: 0; margin: 0; }
.markdown-body pre code { background: none; padding: 0; font-size: 13px; }
.markdown-body blockquote { border-left: 3px solid var(--line); margin: 16px 0;
                            padding: 2px 16px; color: var(--muted); }
.markdown-body hr { border: 0; border-top: 1px solid var(--line); margin: 28px 0; }
.markdown-body img { max-width: 100%; }
.doc-footer { margin-top: 60px; padding-top: 16px;
              border-top: 1px solid var(--line); font-size: 12px;
              color: var(--muted); }
.theme-toggle { position: fixed; top: 14px; right: 16px; z-index: 50;
                width: 38px; height: 38px; border-radius: 50%;
                border: 1px solid var(--line); background: var(--soft);
                color: var(--fg); font-size: 17px; cursor: pointer;
                line-height: 1; display: flex; align-items: center;
                justify-content: center; }
.theme-toggle:hover { border-color: var(--accent); }
@media (max-width: 800px) {
  .layout { grid-template-columns: 1fr; }
  .sidebar { display: none; }
  .content { padding: 24px 18px 60px; }
}
"""

# Set the theme attribute before first paint (no flash-of-wrong-theme).
_HEAD_SCRIPT = """
<script>
(function(){var t;try{t=localStorage.getItem('rr-theme');}catch(e){}
if(t!=='dark'&&t!=='light'){t=(window.matchMedia&&window.matchMedia(
'(prefers-color-scheme: dark)').matches)?'dark':'light';}
document.documentElement.setAttribute('data-theme',t);})();
</script>
"""

_TOGGLE = """
<button class="theme-toggle" id="rr-theme-btn" onclick="rrToggleTheme()"
        title="Toggle light / dark" aria-label="Toggle colour theme"></button>
<script>
function rrTheme(){return document.documentElement.getAttribute('data-theme')||'light';}
function rrSyncBtn(){var b=document.getElementById('rr-theme-btn');
if(b)b.textContent=rrTheme()==='dark'?'\\u2600':'\\u263E';}
function rrToggleTheme(){var n=rrTheme()==='dark'?'light':'dark';
document.documentElement.setAttribute('data-theme',n);
try{localStorage.setItem('rr-theme',n);}catch(e){}rrSyncBtn();}
rrSyncBtn();
</script>
"""

_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>__TITLE__</title>
__HEAD_SCRIPT__
<style>
__CSS__
__PYGMENTS__
</style>
</head>
<body>
<div class="layout">
  <aside class="sidebar"><div class="sidebar-inner">
    <div class="toc-title">Contents</div>
    __TOC__
  </div></aside>
  <main class="content">
    <header class="doc-header">
      <h1 class="doc-title">__TITLE__</h1>
      <div class="doc-meta">__BADGE____CREATED__<span class="src">source: __SRC__</span></div>
    </header>
    <div class="generated-note">Generated view &mdash; edit the Markdown source, not this file.
      Regenerate with <code>tools/render_report.py</code>.</div>
    <article class="markdown-body">
__CONTENT__
    </article>
    <footer class="doc-footer">Generated __TIMESTAMP__ from <span class="src">__SRC__</span></footer>
  </main>
</div>
__TOGGLE__
</body>
</html>
"""


def _dark_style_name():
    """Pick the best available dark Pygments style."""
    available = set(get_all_styles())
    for name in ("github-dark", "monokai", "dracula", "native"):
        if name in available:
            return name
    return "default"


def split_frontmatter(text):
    """Return (metadata, body), splitting a leading YAML frontmatter block.

    Frontmatter is recognised only when the very first line is exactly ``---``
    and a matching closing ``---`` line follows, and the block parses to a
    non-empty mapping. A document that merely opens with a ``----`` horizontal
    rule, or whose ``---``-delimited block is not a mapping, is treated as
    having no frontmatter (the whole text is the body).
    """
    lines = text.splitlines(keepends=True)
    if not lines or lines[0].rstrip() != "---":
        return {}, text
    for i in range(1, len(lines)):
        if lines[i].rstrip() == "---":
            block = "".join(lines[1:i])
            body = "".join(lines[i + 1:]).lstrip("\n")
            try:
                meta = yaml.safe_load(block)
            except yaml.YAMLError:
                meta = None
            if isinstance(meta, dict) and meta:
                return meta, body
            return {}, text
    return {}, text


def render_one(md_path, out_dir):
    """Render one Markdown file to HTML in out_dir. Return the output Path."""
    text = md_path.read_text(encoding="utf-8")
    meta, body = split_frontmatter(text)

    md = _markdown.Markdown(
        extensions=["extra", "toc", "codehilite", "sane_lists", "admonition"],
        extension_configs={"codehilite": {"guess_lang": False}},
    )
    content = md.convert(body)
    toc = getattr(md, "toc", "") or "<p class='src'>(no headings)</p>"

    title = str(meta.get("title") or md_path.stem.replace("-", " ").title())
    status = str(meta.get("status", "")).strip()
    created = str(meta.get("created", "")).strip()
    src_rel = md_path.resolve().relative_to(REPO_ROOT) if _within_repo(md_path) else md_path

    badge = ""
    if status:
        colour = _STATUS_COLOURS.get(status.lower(), "#64748b")
        badge = '<span class="badge" style="background:%s">%s</span>' % (
            colour, _escape(status))
    created_html = '<span>created %s</span>' % _escape(created) if created else ""

    # Light styles apply unconditionally; dark styles are scoped to the
    # data-theme="dark" attribute so both ship in one self-contained file.
    pygments = "\n".join([
        HtmlFormatter(style="default").get_style_defs(".codehilite"),
        HtmlFormatter(style=_dark_style_name()).get_style_defs(
            ':root[data-theme="dark"] .codehilite'),
    ])

    # Single-pass token substitution: re.sub scans the template once, so a
    # substituted value that itself contains a __TOKEN__ is never re-expanded.
    tokens = {
        "HEAD_SCRIPT": _HEAD_SCRIPT,
        "TOGGLE": _TOGGLE,
        "CSS": _CSS,
        "PYGMENTS": pygments,
        "TOC": toc,
        "CONTENT": content,
        "BADGE": badge,
        "CREATED": created_html,
        "SRC": _escape(str(src_rel)),
        "TIMESTAMP": _dt.datetime.now().strftime("%Y-%m-%d %H:%M"),
        "TITLE": _escape(title),
    }
    page = re.sub(r"__([A-Z_]+)__",
                  lambda m: tokens.get(m.group(1), m.group(0)),
                  _PAGE)

    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / (md_path.stem + ".html")
    out_path.write_text(page, encoding="utf-8")
    return out_path


def _within_repo(path):
    try:
        path.resolve().relative_to(REPO_ROOT)
        return True
    except ValueError:
        return False


def _escape(s):
    return (str(s).replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;").replace('"', "&quot;"))


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Render Markdown reports to self-contained styled HTML.")
    parser.add_argument("files", nargs="*", type=Path,
                        help="Markdown files to render.")
    parser.add_argument("--all", action="store_true",
                        help="Render every plans/active/*.md file.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR,
                        help="Output directory (default: temp/reports/).")
    args = parser.parse_args(argv)

    targets = list(args.files)
    if args.all:
        targets += sorted((REPO_ROOT / "plans" / "active").glob("*.md"))
    if not targets:
        parser.error("no input files (pass FILE.md or --all)")

    failures = 0
    for md_path in targets:
        if not md_path.is_file():
            print("skip (not found): %s" % md_path, file=sys.stderr)
            failures += 1
            continue
        try:
            out_path = render_one(md_path, args.out_dir)
        except Exception as exc:  # batch tool: one bad file must not abort the rest
            print("error rendering %s: %s" % (md_path, exc), file=sys.stderr)
            failures += 1
            continue
        print("rendered  %s  ->  %s" % (md_path, out_path))

    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
