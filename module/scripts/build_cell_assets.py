"""Build the cell's asset archive: every file the assembly cell's fragment loads (R2).

A learner's machine has none of this workstation's trees, so the fragment's USDs and
everything they pull in go into one Viam archive package, and the fragment points at it
with "${packages.archive.<name>}" (build_fragment.py --assets package). Layout, so every
relative reference inside the trees still resolves:

    usd/        ~/viam/usd's files (mailbox, cell_props, conveyor), as they sit there
    sim/        this repo's assets/ (the cut-down conveyor)
    isaac/      the vendor assets from isaac-package-shelf's mirror that Isaac 5.0's own
                asset server does not carry (the Schunk EZU 35) or that are loaded with
                physics (the UR short gripper: streaming at boot corrupts physics)

The file list is the USD dependency closure, computed with pxr - run this with a python
that has it (the usd tree's own venv does):

    /home/shrews/viam/usd/.venv/bin/python scripts/build_cell_assets.py [--out DIR]

It reads the live trees and copies; it edits nothing in them. The one rewrite is in the
copy of this repo's conveyor, whose reference to the full conveyor is absolute.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import tarfile
from pathlib import Path

from pxr import UsdUtils

REPO = Path(__file__).resolve().parents[1]
FRAGMENT = REPO / "fragments" / "assembly-cell.json"
# Where each tree lives here, and where it goes in the archive.
ROOTS = {
    "/home/shrews/viam/usd/": "usd/",
    str(REPO / "assets") + "/": "sim/",
    "/home/shrews/viam/isaac-package-shelf/assets/mirror/": "isaac/",
}
# Not packaged: streamed from Isaac 5.0's own asset server (visual only, 320 MB).
STREAMED = ("Environments/Simple_Warehouse/",)
# Read by the module, not referenced by any USD.
EXTRA = ["/home/shrews/viam/usd/mailbox/assembly/assembly_frames.json",
         "/home/shrews/viam/usd/cell_props/props_frames.json"]


def fragment_usds() -> list:
    found = set()

    def walk(o):
        if isinstance(o, dict):
            for v in o.values():
                walk(v)
        elif isinstance(o, list):
            for v in o:
                walk(v)
        elif isinstance(o, str) and o.endswith((".usd", ".usda", ".usdc")) \
                and any(o.startswith(r) for r in ROOTS) \
                and not any(s in o for s in STREAMED):
            found.add(o)
    walk(json.loads(FRAGMENT.read_text()))
    return sorted(found)


def closure(usds) -> tuple:
    files, unresolved = set(), set()
    for p in usds:
        layers, assets, unres = UsdUtils.ComputeAllDependencies(p)
        files |= {os.path.realpath(layer.realPath) for layer in layers}
        files |= {os.path.realpath(a) for a in assets}
        unresolved |= {u for u in unres if not u.endswith(".mdl")}   # Kit's own MDLs
    return sorted(files), sorted(unresolved)


def to_archive(path: str) -> str:
    for here, there in ROOTS.items():
        if path.startswith(here):
            return there + path[len(here):]
    raise ValueError(f"{path} is outside every packaged tree")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(REPO / ".scratch" / "package"))
    args = ap.parse_args()
    files, unresolved = closure(fragment_usds())
    files = sorted(set(files) | set(EXTRA))
    out = Path(args.out) / "cell-assets"
    if out.exists():
        shutil.rmtree(out)
    for f in files:
        dest = out / to_archive(f)
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(f, dest)
    # This repo's conveyor names the full conveyor by absolute path; in the archive it is
    # beside it, two levels up.
    for usda in (out / "sim").rglob("*.usda"):
        text = usda.read_text()
        rel = os.path.relpath(out / "usd", usda.parent)
        usda.write_text(text.replace("@/home/shrews/viam/usd/", f"@{rel}/"))
    manifest = {"files": len(files), "bytes": sum(os.path.getsize(f) for f in files),
                "unresolved": unresolved,
                "streamed": "Isaac 5.0 asset server: " + ", ".join(STREAMED)}
    (out / "MANIFEST.json").write_text(json.dumps(manifest, indent=1) + "\n")
    tar = Path(args.out) / "cell-assets.tar.gz"
    with tarfile.open(tar, "w:gz") as t:
        for p in sorted(out.rglob("*")):
            if p.is_file():                  # files only: adding a directory recurses
                t.add(p, arcname=str(p.relative_to(out)))
    print(f"{len(files)} files, {manifest['bytes'] / 1e6:.1f} MB -> {out}")
    print(f"archive {tar} ({tar.stat().st_size / 1e6:.1f} MB)")
    if unresolved:
        print("UNRESOLVED (missing on this workstation too):", *unresolved, sep="\n  ")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
