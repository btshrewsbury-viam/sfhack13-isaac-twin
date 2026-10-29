"""The individual collider pieces of a part, so a fixture can be shaped to its real form.

A part's overall bounds are the wrong thing to build a nest from, and the flag proves it.
Its published y extent is 37 mm, so a pair of rails set 4 mm clear of that stood 22.5 mm
either side of the centreline - and touched nothing, because the 37 mm comes entirely from
a Ø10 grasp shaft and a Ø24 hub at ONE END. Over the other 250 mm the flag is a vertical
plate 2.2 mm thick. It is not a blade lying flat on the tray at all: it stands on that one
foot and cantilevers its plate out over the tray without touching it, which is exactly why
a belt jolt walks it 30 mm across the slot. A rail sized from the global bounds could never
have stopped that, and no amount of tuning the clearance would have found it.

So this reads the colliders themselves. `assembly_frames.json` publishes attributes and
overall bounds but not the pieces, and the probe venv has no USD runtime, so the geometry
is measured by the usd session's own interpreter and cached - keyed on the geo file's size
and mtime, so an asset change invalidates it rather than being silently ignored.
"""
from __future__ import annotations

import json
import subprocess
from pathlib import Path
from typing import Dict, List, Tuple

GEO = Path("/home/shrews/viam/usd/mailbox/assembly/geo")
# The usd session's interpreter, because it is the one with a USD runtime. Measuring with
# it rather than copying numbers out of it is the whole point: a figure transcribed from an
# asset is a figure that goes stale the next time the asset is built.
USD_PY = Path("/home/shrews/viam/usd/.venv/bin/python")
CACHE = Path(__file__).resolve().parent / "collider_cache.json"

_MEASURE = r"""
import json, sys
from pxr import Usd, UsdGeom, Gf

path, root = sys.argv[1], sys.argv[2]
stage = Usd.Stage.Open(path)
# 'guide' is in the list because the mailbox assets author their colliders as
# purpose=guide - the asset's own header says so, and a BBoxCache that leaves it out
# reports every collider as empty.
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                          ["default", "render", "proxy", "guide"])
scope = stage.GetPrimAtPath(root)
if not scope:
    print(json.dumps({"error": "no prim at " + root}))
    raise SystemExit(0)
out = []
for prim in Usd.PrimRange(scope):
    if prim == scope or not prim.GetTypeName():
        continue
    if not UsdGeom.Boundable(prim):
        continue
    # ComputeRelativeBound, not ComputeLocalBound composed by hand. The first version did
    # the composition itself and applied each piece's own transform TWICE - the grasp
    # shaft came back at x -415 when it sits at -205, and a Cube's `size` x `scale` was
    # dropped entirely, so the flag's 252 mm arm measured 33 mm and its plate measured 5.
    # Both wrong numbers were plausible, which is exactly why this is USD's job.
    box = cache.ComputeRelativeBound(prim, scope).ComputeAlignedRange()
    if box.IsEmpty():
        continue
    lo, hi = box.GetMin(), box.GetMax()
    out.append({
        "name": prim.GetName(),
        "path": str(prim.GetPath()),
        "min_mm": [lo[i] * 1000.0 for i in range(3)],
        "max_mm": [hi[i] * 1000.0 for i in range(3)],
    })
print(json.dumps({"pieces": out}))
"""



def _stamp(path: Path) -> str:
    st = path.stat()
    return f"{st.st_size}:{int(st.st_mtime)}"


def _measure(part_name: str) -> List[Dict]:
    geo = GEO / f"{part_name.lower()}.usdc"
    if not geo.exists():
        raise FileNotFoundError(f"no collider geometry for {part_name} at {geo}")
    if not USD_PY.exists():
        raise RuntimeError(
            f"{USD_PY} does not exist, so the colliders cannot be measured. The probe "
            f"venv has no USD runtime on purpose; this is the usd session's interpreter "
            f"and the fixture depends on it being there.")
    got = subprocess.run(
        [str(USD_PY), "-c", _MEASURE, str(geo), f"/Geo/Colliders/{part_name}"],
        capture_output=True, text=True, timeout=120)
    if got.returncode != 0:
        raise RuntimeError(f"measuring {part_name}'s colliders failed: "
                           f"{got.stderr.strip()[-400:]}")
    blob = json.loads(got.stdout.strip().splitlines()[-1])
    if "error" in blob:
        raise RuntimeError(f"measuring {part_name}'s colliders: {blob['error']}")
    return blob["pieces"]


def pieces(part_name: str) -> List[Dict]:
    """Every collider under the part, as axis-aligned boxes in the part's own frame."""
    geo = GEO / f"{part_name.lower()}.usdc"
    want = _stamp(geo)
    cache = {}
    if CACHE.exists():
        try:
            cache = json.loads(CACHE.read_text())
        except json.JSONDecodeError:
            cache = {}
    entry = cache.get(part_name)
    if entry and entry.get("stamp") == want:
        return entry["pieces"]
    got = _measure(part_name)
    cache[part_name] = {"stamp": want, "pieces": got}
    CACHE.write_text(json.dumps(cache, indent=1, sort_keys=True) + "\n")
    return got


def span_across(part_name: str, along: int, across: int,
                window: Tuple[float, float]) -> Tuple[float, float]:
    """How wide the part is ACROSS one axis, within a window along another.

    This is the number a side rail has to be built from. The flag's global y extent is
    37 mm and its extent over the run a rail can occupy is 2.2 - a rail sized from the
    first stands 20 mm clear of the part it is supposed to be guiding.

    Pieces are included when they OVERLAP the window at all rather than when they sit
    wholly inside it, because a rail only has to clear whatever it could touch.
    """
    lo, hi = None, None
    for piece in pieces(part_name):
        if piece["max_mm"][along] <= window[0] or piece["min_mm"][along] >= window[1]:
            continue
        a, b = piece["min_mm"][across], piece["max_mm"][across]
        lo = a if lo is None else min(lo, a)
        hi = b if hi is None else max(hi, b)
    if lo is None:
        raise ValueError(
            f"{part_name} has no collider at all between {window[0]:.1f} and "
            f"{window[1]:.1f} along axis {along}, so there is nothing there for a guide "
            f"to guide")
    return lo, hi


def reach_down_to(part_name: str, along: int, window: Tuple[float, float]) -> float:
    """The LOWEST z any collider reaches inside the window.

    A guide has to engage the part, and a part can be held clear of the tray by a foot
    somewhere else entirely: the flag's plate bottoms out 5.5 mm above the tray while its
    grasp shaft stands on it. A rail as tall as the part's own lowest point therefore
    touches nothing, and the height has to be measured where the rail actually is.
    """
    best = None
    for piece in pieces(part_name):
        if piece["max_mm"][along] <= window[0] or piece["min_mm"][along] >= window[1]:
            continue
        z = piece["min_mm"][2]
        best = z if best is None else min(best, z)
    if best is None:
        raise ValueError(f"{part_name} has no collider between {window[0]:.1f} and "
                         f"{window[1]:.1f} along axis {along}")
    return best
