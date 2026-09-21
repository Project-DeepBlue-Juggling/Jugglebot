"""Contract test: links out of ``docs/`` are absolute GitHub URLs, and they resolve.

``docs/`` is published by ``mkdocs build --strict`` (``.github/workflows/docs.yml``),
and MkDocs cannot resolve a relative link that leaves ``docs/`` -- an ADR pointing
at ``../../plans/archived/teensy-can-offload.md`` or at a firmware source file.
Two things go wrong at once:

* strict mode warns, so the docs CI gate goes red.  A strict build on 2026-09-20
  gave 33 such warnings across ``docs/adr/`` and ``docs/can_bridge/`` (39 links
  counting the directory links MkDocs only logs at INFO).  They were latent, not
  absent: CI builds only on pushes to ``main``/``refactor``, and neither carries
  these pages yet -- and a gate that opens red cannot tell you about the 34th;
* on the published site the same link is a 404, whatever strict says.

The fix is a rule, not a per-link patch (``DOCUMENTATION_GUIDE.md`` § 2.3): a link
from a docs page to a repo path outside ``docs/`` is the absolute URL
``https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/main/<path>``
(``tree/main`` for a directory).  That URL works on the site and on GitHub, but
MkDocs cannot check it -- so this test does the checking MkDocs no longer can:

* **no escape** -- no relative link in a docs page resolves outside ``docs/``;
* **no rot** -- the ``<path>`` half of every such URL exists in the working tree,
  so renaming or archiving a plan or source file fails here rather than 404ing
  on the site (``plans/archived`` renames have broken inbound links silently
  before; see CLAUDE.md).  Existence is checked against the working tree, not
  git, so a link to a gitignored path (``temp/``) would pass here and 404 on
  GitHub.

``docs/agents/`` is excluded from the site (``mkdocs.yml`` ``exclude_docs``) and
skipped here.  The last three tests are the detector's own negative controls: a
scanner that finds nothing would read as green.

Pure text, no fixtures, no temp files, no network -- safe under xdist.
"""

from __future__ import annotations

import os
import posixpath
import re
from urllib.parse import unquote

_TESTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_REPO_ROOT = os.path.dirname(_TESTS_DIR)
_DOCS_DIR = os.path.join(_REPO_ROOT, 'docs')

# Directories under docs/ that mkdocs.yml ``exclude_docs`` keeps off the site.
_EXCLUDED_PREFIXES = ('agents/',)

# The one blessed way out of docs/: an absolute GitHub URL on ``main``.
_REPO_URL_RE = re.compile(
    r'^https://github\.com/Project-DeepBlue-Juggling/Jugglebot/'
    r'(?:blob|tree)/main/(?P<path>[^#?]+)'
)
_SCHEME_RE = re.compile(r'^[A-Za-z][A-Za-z0-9+.\-]*:')
_FENCE_RE = re.compile(r'^\s{0,3}(?:```|~~~)')

# Inline ``[text](target)`` / ``![alt](target)``, and reference definitions
# ``[label]: target``.  Both forms are policed because MkDocs resolves both.
_INLINE_LINK_RE = re.compile(r'\]\(\s*<?([^)\s>]+)')
_REFDEF_RE = re.compile(r'^\s{0,3}\[[^\]]+\]:\s*<?(\S+?)>?(?:\s+"[^"]*")?\s*$')


def _link_targets(text):
    """Every link target in *text*, skipping fenced code blocks."""
    in_fence = False
    for line in text.splitlines():
        if _FENCE_RE.match(line):
            in_fence = not in_fence
            continue
        if in_fence:
            continue
        for target in _INLINE_LINK_RE.findall(line):
            yield target
        ref = _REFDEF_RE.match(line)
        if ref:
            yield ref.group(1)


def _escapes_docs(doc_rel, target):
    """Repo-relative path *target* resolves to if it is a relative link that
    leaves ``docs/``; otherwise ``None``.

    ``doc_rel`` is the page's path under ``docs/`` (posix separators).
    """
    if not target or _SCHEME_RE.match(target) or target.startswith(('#', '/')):
        return None
    path = target.split('#', 1)[0].split('?', 1)[0]
    if not path:
        return None
    resolved = posixpath.normpath(
        posixpath.join('docs', posixpath.dirname(doc_rel), path))
    if resolved == 'docs' or resolved.startswith('docs/'):
        return None
    return resolved


def _site_pages():
    """``(path under docs/, text)`` for every page that is published."""
    pages = []
    for dirpath, _dirs, files in os.walk(_DOCS_DIR):
        for name in files:
            if not name.endswith('.md'):
                continue
            full = os.path.join(dirpath, name)
            rel = os.path.relpath(full, _DOCS_DIR).replace(os.sep, '/')
            if rel.startswith(_EXCLUDED_PREFIXES):
                continue
            with open(full, encoding='utf-8') as fh:
                pages.append((rel, fh.read()))
    return sorted(pages)


def test_scanner_sees_the_docs():
    """Vacuous-pass guard: an empty scan must not read as green."""
    pages = _site_pages()
    assert pages, 'no docs/**/*.md found -- the scanner is looking in the wrong place'
    assert any(
        _REPO_URL_RE.match(target)
        for _rel, text in pages for target in _link_targets(text)
    ), 'no repo URLs found in docs/ -- the scanner or the rule has drifted'


def test_no_relative_link_leaves_docs():
    bad = ['docs/%s -> %s' % (rel, target)
           for rel, text in _site_pages()
           for target in _link_targets(text)
           if _escapes_docs(rel, target)]
    assert not bad, (
        'relative links that leave docs/ -- MkDocs cannot resolve them (--strict '
        'warns, and the published link is a 404).  Use '
        'https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/main/<path> '
        '(tree/main for a directory); DOCUMENTATION_GUIDE.md 2.3:\n  '
        + '\n  '.join(bad))


def _missing_repo_paths(text):
    """Repo paths named by GitHub URLs in *text* that are absent from the tree."""
    missing = []
    for target in _link_targets(text):
        match = _REPO_URL_RE.match(target)
        if match and not os.path.exists(
                os.path.join(_REPO_ROOT, unquote(match.group('path')))):
            missing.append(match.group('path'))
    return missing


def test_repo_urls_point_at_existing_paths():
    missing = ['docs/%s -> %s' % (rel, path)
               for rel, text in _site_pages()
               for path in _missing_repo_paths(text)]
    assert not missing, (
        'GitHub URLs in docs/ whose path is not in the working tree (renamed, '
        'archived or deleted -- the published link would 404):\n  '
        + '\n  '.join(missing))


def test_detector_flags_escaping_links_only():
    text = '\n'.join([
        '[a](../../plans/x.md)',        # escapes docs/: flagged
        '[b](sibling.md#anchor)',       # stays inside docs/
        '[c](https://example.com/x)',   # absolute URL
        '[d](#local-anchor)',           # in-page anchor
        '[e]: ../../src/y.py',          # reference definition that escapes
        '```',
        '[f](../../inside/a/fence.md)',  # fenced code: ignored
        '```',
    ])
    flagged = [t for t in _link_targets(text) if _escapes_docs('adr/x.md', t)]
    assert flagged == ['../../plans/x.md', '../../src/y.py']


def test_detector_resolves_relative_to_the_page():
    # One level up from docs/adr/ is still docs/; two levels leaves it.
    assert _escapes_docs('adr/x.md', '../index.md') is None
    assert _escapes_docs('adr/x.md', '../../plans/x.md') == 'plans/x.md'
    assert _escapes_docs('index.md', '../plans/x.md') == 'plans/x.md'


def test_detector_flags_a_repo_url_whose_path_is_gone():
    base = 'https://github.com/Project-DeepBlue-Juggling/Jugglebot/'
    text = '\n'.join([
        '[a](%sblob/main/no/such/file.md)' % base,  # gone: flagged
        '[b](%sblob/main/CLAUDE.md)' % base,        # exists: fine
        '[c](%stree/main/docs)' % base,             # an existing directory: fine
    ])
    assert _missing_repo_paths(text) == ['no/such/file.md']
