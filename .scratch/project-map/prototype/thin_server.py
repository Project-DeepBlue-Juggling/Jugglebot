#!/usr/bin/env python3
"""PROTOTYPE for ticket #24 (wayfinder map #21). THROWAWAY -- not production.

A thin, stdlib-only file-backed server that speaks just enough of TaskTree's
REST contract (research/tasktree-frontend-contract.md, section 3) for the
STOCK TaskTree frontend to run against it:

    python thin_server.py [items_dir] [port]
    cd E:/PDJ/Github/TaskTree/frontend
    VITE_API_PROXY_TARGET=http://localhost:8765 npm run dev:local

Model: every markdown file in items/ is one map item (a Pathway, a plan, a
phase, an inbox Idea/Bug/Feature).  Frontmatter keys are the item's fields,
the body is its description.  The index is rebuilt from disk on EVERY request
(so a hand edit + browser refresh is live) and every mutation is written
straight back to the file (write-through).  No audit log, no SSE, no undo.

Skill note (mattpocock-skills:prototype): the question here is neither
"does this state model feel right" nor "what should it look like" but
"does the stock frontend survive a thin server", so the artefact is this
server plus the unmodified frontend, not a single HTML file or UI variants.
"""
from __future__ import annotations

import datetime as dt
import json
import re
import sys
import threading
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

NS = uuid.UUID("3f9d2c4e-7b1a-4e6f-9c2d-0a5b8e7f6d41")
ITEMS = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent / "items"
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8765
T0 = "2026-10-23T00:00:00+00:00"
STAGES = ["pathway", "plan", "phase", "idea", "feature", "bug", "task"]
LOCK = threading.Lock()


def uid(key: str) -> str:
    return str(uuid.uuid5(NS, key))


def now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds")


USERS = [
    {"id": uid("user:harrison"), "email": "harrison@jugglebot.local", "display_name": "Harrison",
     "color": "#2563eb", "dark_mode": "system", "created_at": T0,
     "is_availability_admin": False, "can_access_catalogue": False},
    {"id": uid("user:claude"), "email": "claude@jugglebot.local", "display_name": "Claude",
     "color": "#d97706", "dark_mode": "system", "created_at": T0,
     "is_availability_admin": False, "can_access_catalogue": False},
]
USER_BY_ID = {u["id"]: u for u in USERS}
USER_BY_NAME = {u["email"].split("@")[0]: u for u in USERS}
OWNER = USERS[0]["id"]
PROJECT = {"id": uid("project:jugglebot"), "name": "Jugglebot", "stages": STAGES,
           "due_date_warning_days": 7, "brief_enabled": False, "bom_enabled": False,
           "created_at": T0, "created_by": OWNER}
MEMBERSHIPS = [{"user_id": u["id"], "project_id": PROJECT["id"], "role": "admin",
                "can_view_bom": False, "created_at": T0, "created_by": OWNER} for u in USERS]

# ----------------------------------------------------------------- files ---

def parse_item(path: Path):
    text = path.read_text(encoding="utf-8")
    m = re.match(r"^---\r?\n(.*?)\r?\n---\r?\n?(.*)$", text, re.S)
    fm, body = {}, text
    if m:
        body = m.group(2)
        for line in m.group(1).splitlines():
            if ":" not in line or line.startswith("#"):
                continue
            k, v = line.split(":", 1)
            v = v.strip()
            try:
                fm[k.strip()] = json.loads(v)
            except ValueError:
                fm[k.strip()] = v
    return fm, body.strip()


def write_item(slug: str, fm: dict, body: str) -> None:
    lines = ["---"]
    for k, v in fm.items():
        if isinstance(v, str):
            try:
                rt = json.loads(v)
            except ValueError:
                rt = None
            lines.append(f"{k}: {v}" if rt is None or rt == v else f"{k}: {json.dumps(v)}")
        else:
            lines.append(f"{k}: {json.dumps(v)}")
    lines += ["---", "", body.strip(), ""]
    (ITEMS / f"{slug}.md").write_text("\n".join(lines), encoding="utf-8")


class Index:
    """Rebuilt from disk on every request.  slug -> (fm, body); plus id maps."""

    def __init__(self) -> None:
        self.items: dict[str, tuple[dict, str]] = {}
        for p in sorted(ITEMS.glob("*.md")):
            fm, body = parse_item(p)
            fm.setdefault("id", uid("item:" + p.stem))
            self.items[p.stem] = (fm, body)
        self.slug_by_id = {fm["id"]: s for s, (fm, _) in self.items.items()}

    # --- derived fields -------------------------------------------------
    def blockers(self, slug: str) -> list[str]:
        return [b for b in self.items[slug][0].get("blocked_by", []) if b in self.items]

    def effective_start(self, slug: str) -> str:
        fm = self.items[slug][0]
        if fm.get("start_date"):
            return str(fm["start_date"])[:10]
        dues = [str(self.items[b][0].get("due_date"))[:10] for b in self.blockers(slug)
                if self.items[b][0].get("due_date")]
        return max(dues) if dues else dt.date.today().isoformat()

    def due(self, slug: str):
        fm = self.items[slug][0]
        wd = fm.get("work_days")
        if wd:
            d = dt.date.fromisoformat(self.effective_start(slug))
            n = int(wd)
            while n > 0:
                d += dt.timedelta(days=1)
                if d.weekday() < 5:
                    n -= 1
            return d.isoformat()
        return fm.get("due_date")

    def task(self, slug: str, i: int = 0) -> dict:
        fm, body = self.items[slug]
        parent = fm.get("parent")
        canvas = fm.get("canvas") or {}
        owners = [USER_BY_NAME[o]["id"] for o in fm.get("owners", []) if o in USER_BY_NAME]
        owners.sort(key=lambda u: USER_BY_ID[u]["display_name"])
        return {
            "id": fm["id"], "project_id": PROJECT["id"],
            "parent_id": self.items[parent][0]["id"] if parent in self.items else None,
            "title": fm.get("title", slug), "description": body,
            "completion_criteria": fm.get("destination", ""),
            "assignee_ids": owners, "stage": fm.get("kind", "task"),
            "status": fm.get("status", "not_started"), "priority": fm.get("priority"),
            "is_section": bool(fm.get("section", False)),
            "backlog_rank": float(fm.get("rank", i)), "color": fm.get("color"),
            "canvas_x": float(canvas.get("x", 0)), "canvas_y": float(canvas.get("y", 0)),
            "canvas_collapsed": bool(canvas.get("collapsed", False)),
            "work_days": fm.get("work_days"), "start_date": fm.get("start_date"),
            "due_date": self.due(slug), "reviewer_id": None,
            "effective_start_date": self.effective_start(slug),
            "created_at": fm.get("created", T0), "created_by": OWNER,
            "updated_at": fm.get("updated", T0), "updated_by": OWNER,
        }

    def tasks(self) -> list[dict]:
        return [self.task(s, i) for i, s in enumerate(self.items)]

    def edges(self) -> list[dict]:
        out = []
        for s in self.items:
            for b in self.blockers(s):
                out.append({"id": uid(f"dep:{b}->{s}"), "blocker_id": self.items[b][0]["id"],
                            "blocked_id": self.items[s][0]["id"], "created_at": T0,
                            "created_by": OWNER})
        return out

    def ancestors(self, slug: str):
        seen = []
        p = self.items[slug][0].get("parent")
        while p in self.items and p not in seen:
            seen.append(p)
            p = self.items[p][0].get("parent")
        return seen

    def reaches(self, start: str, target: str) -> bool:
        """Does following blocked_by edges from `start` reach `target`?"""
        stack, seen = [start], set()
        while stack:
            s = stack.pop()
            if s == target:
                return True
            if s in seen:
                continue
            seen.add(s)
            stack.extend(self.blockers(s))
        return False

    def apply(self, slug: str, body_json: dict) -> None:
        """TaskUpdateBody / TaskCreateBody -> frontmatter + body, written through."""
        fm, body = self.items[slug]
        b = body_json
        if "title" in b: fm["title"] = b["title"]
        if "parent_id" in b:
            fm["parent"] = self.slug_by_id.get(b["parent_id"]) if b["parent_id"] else None
        if "description" in b: body = b["description"] or ""
        if "completion_criteria" in b: fm["destination"] = b["completion_criteria"] or ""
        if "assignee_ids" in b:
            fm["owners"] = [USER_BY_ID[u]["email"].split("@")[0] for u in b["assignee_ids"] if u in USER_BY_ID]
        if "stage" in b: fm["kind"] = b["stage"]
        if "status" in b: fm["status"] = b["status"]
        if "priority" in b: fm["priority"] = b["priority"]
        if "backlog_rank" in b: fm["rank"] = b["backlog_rank"]
        if "is_section" in b: fm["section"] = b["is_section"]
        if "color" in b: fm["color"] = b["color"]
        if any(k in b for k in ("canvas_x", "canvas_y", "canvas_collapsed")):
            c = dict(fm.get("canvas") or {})
            if "canvas_x" in b: c["x"] = round(b["canvas_x"], 1)
            if "canvas_y" in b: c["y"] = round(b["canvas_y"], 1)
            if "canvas_collapsed" in b: c["collapsed"] = b["canvas_collapsed"]
            fm["canvas"] = c
        for k in ("work_days", "start_date", "due_date"):
            if k in b: fm[k] = b[k]
        fm["updated"] = now()
        self.items[slug] = (fm, body)
        write_item(slug, fm, body)


def slugify(title: str) -> str:
    s = re.sub(r"[^a-z0-9]+", "-", title.lower()).strip("-") or "item"
    return s[:50]


# ---------------------------------------------------------------- server ---

HITS: dict[str, int] = {}
UNHANDLED: dict[str, int] = {}
P = PROJECT["id"]


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):  # quieter
        pass

    def _json(self, status: int, payload=None) -> None:
        data = b"" if payload is None else json.dumps(payload).encode()
        self.send_response(status)
        if status != 204:
            self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        if data:
            self.wfile.write(data)

    def _body(self) -> dict:
        n = int(self.headers.get("Content-Length") or 0)
        return json.loads(self.rfile.read(n) or b"{}") if n else {}

    def _route(self, method: str) -> None:
        path = urlparse(self.path).path
        if path.startswith("/api/v1"):
            path = path[len("/api/v1"):]
        with LOCK:
            try:
                status = self.dispatch(method, path)
            except Exception as e:  # prototype: surface, don't hide
                status = 500
                self._json(500, {"detail": f"{type(e).__name__}: {e}"})
        print(f"{status} {method:6s} {path}", flush=True)

    do_GET = lambda self: self._route("GET")
    do_POST = lambda self: self._route("POST")
    do_PATCH = lambda self: self._route("PATCH")
    do_PUT = lambda self: self._route("PUT")
    do_DELETE = lambda self: self._route("DELETE")

    def dispatch(self, m: str, path: str) -> int:
        ix = Index()
        pt = f"/projects/{P}/tasks"
        pd = f"/projects/{P}/dependencies"

        def hit(name: str) -> None:
            HITS[name] = HITS.get(name, 0) + 1

        # --- users -------------------------------------------------------
        if m == "GET" and path == "/users":
            hit("GET /users"); self._json(200, USERS); return 200
        if path == "/users/me" and m in ("GET", "PATCH"):
            hit(f"{m} /users/me")
            me = USER_BY_NAME.get((self.headers.get("X-Dev-User") or "harrison"), USERS[0])
            if m == "PATCH":
                me.update({k: v for k, v in self._body().items() if k in ("color", "dark_mode")})
            self._json(200, {**me, "is_availability_member": False}); return 200
        if m == "GET" and path == "/users/me/tasks":
            hit("GET /users/me/tasks (stub [])"); self._json(200, []); return 200
        # --- shell stubs ---------------------------------------------------
        if m == "GET" and path == "/users/me/notifications":
            hit("GET notifications (stub [])"); self._json(200, []); return 200
        if m == "GET" and path == "/users/me/notifications/unread_count":
            hit("GET unread_count (stub 0)"); self._json(200, {"count": 0}); return 200
        if path in ("/events", "/users/me/notifications/stream"):
            hit(f"GET {path} (stub 404)"); self._json(404, {"detail": "no SSE in prototype"}); return 404
        if m == "POST" and path == f"/projects/{P}/undo":
            hit("POST undo (stub 404)"); self._json(404, {"detail": "Nothing to undo"}); return 404
        if m == "GET" and re.fullmatch(pt + r"/[0-9a-f-]+/comments", path):
            hit("GET comments (stub [])"); self._json(200, []); return 200
        # --- projects ------------------------------------------------------
        if m == "GET" and path == "/projects":
            hit("GET /projects"); self._json(200, [PROJECT]); return 200
        if m == "GET" and path == f"/projects/{P}":
            hit("GET /projects/{p}"); self._json(200, PROJECT); return 200
        if m == "GET" and path == f"/projects/{P}/memberships":
            hit("GET memberships"); self._json(200, MEMBERSHIPS); return 200
        # --- tasks ---------------------------------------------------------
        if m == "GET" and path == pt:
            hit("GET tasks"); self._json(200, ix.tasks()); return 200
        if m == "POST" and path == pt:
            hit("POST tasks")
            b = self._body()
            base = slugify(b.get("title", "item"))
            slug = base
            while slug in ix.items:
                slug = f"{base}-{uuid.uuid4().hex[:4]}"
            fm = {"id": str(uuid.uuid4()), "title": b.get("title", ""), "kind": b.get("stage", "task"),
                  "parent": None, "status": "not_started", "created": now(),
                  "rank": max([t["backlog_rank"] for t in ix.tasks()] + [0]) + 1}
            ix.items[slug] = (fm, "")
            ix.slug_by_id[fm["id"]] = slug
            ix.apply(slug, b)
            self._json(201, ix.task(slug)); return 201
        mt = re.fullmatch(pt + r"/([0-9a-f-]+)", path)
        if mt:
            slug = ix.slug_by_id.get(mt.group(1))
            if slug is None:
                self._json(404, {"detail": "task not found"}); return 404
            if m == "GET":
                hit("GET tasks/{id}")
                tid = ix.items[slug][0]["id"]
                e = ix.edges()
                self._json(200, {**ix.task(slug), "blocking": [x for x in e if x["blocker_id"] == tid],
                                 "blocked_by": [x for x in e if x["blocked_id"] == tid]}); return 200
            if m == "PATCH":
                hit("PATCH tasks/{id}")
                b = self._body()
                if b.get("parent_id"):
                    new_parent = ix.slug_by_id.get(b["parent_id"])
                    if new_parent == slug or slug in ix.ancestors(new_parent):
                        via = ix.task(new_parent)
                        self._json(409, {"detail": "Reparenting would close a cycle",
                                         "would_close_via": {"id": via["id"], "title": via["title"]}}); return 409
                ix.apply(slug, b)
                self._json(200, ix.task(slug)); return 200
            if m == "DELETE":
                hit("DELETE tasks/{id}")
                tid = ix.items[slug][0]["id"]
                kids = [ix.task(s) for s, (fm, _) in ix.items.items() if fm.get("parent") == slug]
                e = ix.edges()
                blocking = [ix.task(ix.slug_by_id[x["blocked_id"]]) for x in e if x["blocker_id"] == tid]
                blocked_by = [ix.task(ix.slug_by_id[x["blocker_id"]]) for x in e if x["blocked_id"] == tid]
                if kids or blocking or blocked_by:
                    summ = lambda ts: [{"id": t["id"], "title": t["title"]} for t in ts]
                    self._json(409, {"detail": "Task has children or dependencies", "children": summ(kids),
                                     "blocking": summ(blocking), "blocked_by": summ(blocked_by)}); return 409
                (ITEMS / f"{slug}.md").unlink()
                self._json(204); return 204
        # --- dependencies --------------------------------------------------
        if m == "GET" and path == pd:
            hit("GET dependencies"); self._json(200, ix.edges()); return 200
        if m == "POST" and path == pd:
            hit("POST dependencies")
            b = self._body()
            blocker, blocked = ix.slug_by_id.get(b.get("blocker_id")), ix.slug_by_id.get(b.get("blocked_id"))
            if not blocker or not blocked:
                self._json(404, {"detail": "task not found"}); return 404
            if blocker == blocked or ix.reaches(blocker, blocked):
                self._json(409, {"detail": "Dependency would create a cycle"}); return 409
            fm, body = ix.items[blocked]
            fm["blocked_by"] = sorted(set(fm.get("blocked_by", [])) | {blocker})
            ix.apply(blocked, {})
            edge = next(x for x in ix.edges() if x["blocker_id"] == b["blocker_id"] and x["blocked_id"] == b["blocked_id"])
            self._json(201, edge); return 201
        md = re.fullmatch(pd + r"/([0-9a-f-]+)", path)
        if md and m == "DELETE":
            hit("DELETE dependencies/{id}")
            edge = next((x for x in ix.edges() if x["id"] == md.group(1)), None)
            if edge is None:
                self._json(404, {"detail": "dependency not found"}); return 404
            blocked, blocker = ix.slug_by_id[edge["blocked_id"]], ix.slug_by_id[edge["blocker_id"]]
            fm, _ = ix.items[blocked]
            fm["blocked_by"] = [x for x in fm.get("blocked_by", []) if x != blocker]
            ix.apply(blocked, {})
            self._json(204); return 204
        # --- anything else -------------------------------------------------
        key = f"{m} {re.sub(r'[0-9a-f]{8}-[0-9a-f-]{27}', '{id}', path)}"
        UNHANDLED[key] = UNHANDLED.get(key, 0) + 1
        self._json(404, {"detail": f"prototype does not implement {key}"}); return 404


if __name__ == "__main__":
    ITEMS.mkdir(exist_ok=True)
    print(f"thin_server: {len(list(ITEMS.glob('*.md')))} items in {ITEMS}, project {P}, port {PORT}", flush=True)
    srv = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        print("\n--- endpoints hit ---")
        for k, v in sorted(HITS.items()):
            print(f"{v:5d}  {k}")
        print("--- unhandled ---")
        for k, v in sorted(UNHANDLED.items()):
            print(f"{v:5d}  {k}")
