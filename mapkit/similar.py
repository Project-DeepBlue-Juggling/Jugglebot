"""Duplicate and near-duplicate Item titles: the one check every filing path runs.

The map server's create (the app's new-item form and the GUI's quick capture)
and ``mapkit item new`` all call :func:`screen` before writing anything.

* **Duplicate** -- the same title as an open Item once case, punctuation and
  spacing are ignored (:func:`normalize`). Always refused: the 2026-10-30
  Robot-tab retries filed one bug four times (JB-103..106) because each POST
  outlived the client's timeout under load and a retry had no way to know.
* **Similar** -- a near title, or the same title as a closed Item (a recurrence is
  legitimate, but reopening may be the better move). Refused unless the caller
  says ``allow_similar``: the person has seen the list and chose to file.
  Near is any of: character ratio >= :data:`RATIO_MIN` (a truncated or
  retyped title); >= :data:`SHARED_MIN` shared content words with word
  overlap (Jaccard) >= :data:`OVERLAP_MIN` (a reword); >= :data:`CONTAIN_SHARED_MIN`
  shared words making up >= :data:`CONTAIN_MIN` of the shorter title's (a
  short reword of a long, sentence-length title).

Thresholds were set on the 103-Item map of 2026-10-30: no existing pair
reaches any bar (closest: JB-40 / JB-41 at ratio 0.61; JB-66 / JB-64 at
overlap 0.38; JB-43 / JB-68 at containment 0.75 with only 3 shared words),
while reworded copies of JB-103 score overlap 0.55-0.60, its truncated twin
ratio 0.99, and a short reword of JB-101 containment 0.88 (7 of 8 words).
Lexical only: a reword that shares no words (``panel`` for ``window``) is
not caught.
"""
from __future__ import annotations

import difflib
import re
from dataclasses import dataclass
from typing import Iterable, List, Optional, Set, Tuple

from mapkit.items import Item

CLOSED_STATUSES = ("complete", "dropped", "promoted")
RATIO_MIN = 0.80
OVERLAP_MIN = 0.40
SHARED_MIN = 3
CONTAIN_MIN = 0.75
CONTAIN_SHARED_MIN = 4
MAX_SHOWN = 5

_STOP = frozenset((
    "a an and are as at be by does doesn don for from has have in into is it its "
    "not of on or s should t that the then this to was when which while will with").split())
_SUFFIXES = ("ing", "ed", "es", "s")


@dataclass(frozen=True)
class Match:
    item: Item
    exact: bool      # same normalised title
    ratio: float     # character similarity of the normalised titles, 0..1
    overlap: float   # content-word Jaccard, 0..1
    shared: int      # content words in common
    contain: float   # shared / content words of the shorter title, 0..1

    @property
    def closed(self) -> bool:
        return self.item.status in CLOSED_STATUSES

    def describe(self) -> str:
        sid = self.item.short_id or self.item.slug
        return "%s (%s) %s" % (sid, self.item.status, self.item.title)


def normalize(title: str) -> str:
    """Lower case, every run of non-alphanumerics one space, trimmed."""
    return " ".join(re.sub(r"[^a-z0-9]+", " ", (title or "").lower()).split())


def _stem(word: str) -> str:
    for suf in _SUFFIXES:
        if word.endswith(suf) and len(word) - len(suf) >= 3:
            return word[:-len(suf)]
    return word


def words(title: str) -> Set[str]:
    """Content words, crudely stemmed (``minimized`` and ``minimizing`` meet)."""
    return {_stem(w) for w in normalize(title).split() if w not in _STOP}


def compare(item: Item, title: str) -> Match:
    a, b = normalize(title), normalize(item.title)
    wa, wb = words(title), words(item.title)
    shared = len(wa & wb)
    union = len(wa | wb)
    shorter = min(len(wa), len(wb))
    return Match(item=item, exact=(a == b and a != ""),
                 ratio=difflib.SequenceMatcher(None, a, b).ratio(),
                 overlap=(shared / union) if union else 0.0, shared=shared,
                 contain=(shared / shorter) if shorter else 0.0)


def _near(m: Match) -> bool:
    return (m.ratio >= RATIO_MIN
            or (m.shared >= SHARED_MIN and m.overlap >= OVERLAP_MIN)
            or (m.shared >= CONTAIN_SHARED_MIN and m.contain >= CONTAIN_MIN))


def screen(items: Iterable[Item], title: str,
           exclude: Optional[str] = None) -> Tuple[Optional[Match], List[Match]]:
    """``(duplicate, similar)`` for a title about to be filed.

    ``duplicate`` is the open Item with the same normalised title (the oldest
    ID when several), else None. ``similar`` is every other near Item, closest
    first, at most :data:`MAX_SHOWN`; an exact match on a closed Item lands
    here. ``exclude`` is a stem to skip (an Item being renamed).
    """
    dup: Optional[Match] = None
    near: List[Match] = []
    for it in items:
        if exclude is not None and "%s-%s" % (it.kind, it.slug) == exclude:
            continue
        m = compare(it, title)
        if m.exact and not m.closed:
            if dup is None or _id_num(m.item) < _id_num(dup.item):
                dup = m
        elif m.exact or _near(m):
            near.append(m)
    near.sort(key=lambda m: (not m.exact, -_score(m), m.item.slug))
    return dup, near[:MAX_SHOWN]


def _score(m: Match) -> float:
    return max(m.ratio, m.overlap, m.contain if m.shared >= CONTAIN_SHARED_MIN else 0.0)


def _id_num(item: Item) -> float:
    sid = item.short_id
    return float(sid.split("-")[1]) if sid else float("inf")


def as_dict(m: Match) -> dict:
    """The wire shape the map server returns for one match."""
    return {"short_id": m.item.short_id, "stem": "%s-%s" % (m.item.kind, m.item.slug),
            "kind": m.item.kind, "title": m.item.title, "status": m.item.status,
            "exact": m.exact, "score": round(_score(m), 2)}
