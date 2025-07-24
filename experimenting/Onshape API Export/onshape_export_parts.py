#!/usr/bin/env python3
"""
Export one or many parts from an Onshape Part Studio to STEP,
naming each file after the part.

Examples
--------
# every part in the Part Studio
python export_parts.py <did> <wid> <eid>

# just one part
python export_parts.py <did> <wid> <eid> --part <partId>

# still possible: whole Assembly tab
python export_parts.py <did> <wid> <eid> --assembly
"""
import sys, time, json, re, argparse
from onshape_client.client import Client
from pathlib import Path

BASE = "https://cad.onshape.com"
DOWNLOADS = Path.home() / "Downloads"

# ---------- helpers ----------------------------------------------------------

def element_meta(api, did, wid, eid):
    """Return {'name': str, 'type': str} for the given element id."""
    els = req_json(api, "GET",
                   f"{BASE}/api/v6/documents/d/{did}/w/{wid}/elements")
    hit = next((el for el in els if el["id"] == eid), None)
    if not hit:
        raise ValueError("Element ID not found in this workspace")
    return {"name": hit["name"], "type": hit["elementType"].upper()}

def req_json(api, method, url, **kw):
    kw.setdefault("query_params", [])
    return json.loads(api.request(method, url, **kw).data)

def safe_filename(name):
    """Replace chars the OS dislikes and collapse whitespace."""
    name = re.sub(r'[\\/*?:\"<>|]', "_", name)
    return re.sub(r"\s+", "_", name).strip("_") or "unnamed"

def wait_done(api, tid):
    while True:
        st = req_json(api, "GET", f"{BASE}/api/v8/translations/{tid}")
        state = st["requestState"]
        if state == "DONE":
            return st
        if state == "FAILED":
            raise RuntimeError(st.get("failureReason"))
        time.sleep(2)

def download_from_status(api, did, wid, status):
    """Try attachment → externalData → blob tab."""
    tid = status["id"]

    # 1) attachment
    try:
        resp = api.request("GET",
            f"{BASE}/api/v8/translations/{tid}/attachment",
            _preload_content=False, query_params=[])
        if resp.status == 200 and int(resp.headers.get("Content-Length", "0")):
            return resp.data
    except Exception:
        pass

    # 2) externalData bucket
    if status.get("resultExternalDataIds"):
        fid = status["resultExternalDataIds"][0]
        resp = api.request("GET",
            f"{BASE}/api/v6/documents/d/{did}/externaldata/{fid}",
            _preload_content=False, query_params=[])
        if resp.status == 200:
            return resp.data

    # 3) blob tab (if storeInDocument == True)
    if status.get("resultElementIds"):
        eid_blob = status["resultElementIds"][0]
        resp = api.request("GET",
            f"{BASE}/api/blobelements/d/{did}/w/{wid}/e/{eid_blob}/file",
            _preload_content=False, query_params=[])
        if resp.status == 200:
            return resp.data

    raise RuntimeError("Could not locate exported file bytes")

# ---------- main -------------------------------------------------------------

def get_element_name(api, did, wid, eid):
    """Return the tab (element) name for eid → fallback to eid if not found."""
    elements = req_json(api, "GET",
        f"{BASE}/api/v6/documents/d/{did}/w/{wid}/elements")   # list of tabs
    hit = next((el for el in elements if el["id"] == eid), None)
    return hit["name"] if hit else eid                       # :contentReference[oaicite:0]{index=0}

def export_parts(did, wid, eid, part_ids=None):
    api = Client().api_client
    meta = element_meta(api, did, wid, eid)
    is_asm = (meta["type"] == "ASSEMBLY")

    # If we're in Assembly mode, fall back to old behaviour (one big STEP)
    if is_asm:
        asm_name = safe_filename(meta["name"])
        payload  = {"formatName": "STEP", "storeInDocument": False}

        job  = req_json(api, "POST",
            f"{BASE}/api/v6/assemblies/d/{did}/w/{wid}/e/{eid}/translations",
            body=payload)
        st   = wait_done(api, job["id"])
        data = download_from_status(api, did, wid, st)

        out   = DOWNLOADS / f"{asm_name}.step"
        out.write_bytes(data)
        print(f"Wrote {out} ({len(data)/1024:.1f} kB)")
        return

    # ---------- Part Studio path ----------
    # get all parts so we have names
    parts = req_json(api, "GET",
        f"{BASE}/api/v6/parts/d/{did}/w/{wid}/e/{eid}")

    id_to_name = {p["partId"]: p["name"] for p in parts}

    target_ids = part_ids or list(id_to_name)

    for pid in target_ids:
        payload = {
            "formatName":       "STEP",
            "partIds":          pid,              # ← key trick
            "onePartPerDoc":    True,             # safety when multiple IDs
            "storeInDocument":  False
        }
        job = req_json(api, "POST",
            f"{BASE}/api/v6/partstudios/d/{did}/w/{wid}/e/{eid}/translations",
            body=payload)
        st   = wait_done(api, job["id"])
        data = download_from_status(api, did, wid, st)

        pname = id_to_name.get(pid, pid)
        fname = safe_filename(pname) + ".step"
        out   = DOWNLOADS / fname            # use your Downloads folder
        out.write_bytes(data)
        print(f"Wrote {out} ({len(data)/1024:.1f} kB)")

# ---------- CLI --------------------------------------------------------------

if __name__ == "__main__":
    try:
        ap = argparse.ArgumentParser()
        ap.add_argument("did"); ap.add_argument("wid"); ap.add_argument("eid")
        ap.add_argument("--part", help="comma‑separated partId list (default: all)")
        ap.add_argument("--assembly", action="store_true",
                        help="element is an Assembly (whole export)")
        args = ap.parse_args()

        part_ids = args.part.split(",") if args.part else None
        export_parts(args.did, args.wid, args.eid, part_ids)
        sys.exit(0)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)