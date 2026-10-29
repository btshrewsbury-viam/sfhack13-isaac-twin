"""Put the two authored camera assets into the cell, in one atomic pass.

Held back from the live loop deliberately: both change the layout fingerprint and so
invalidate every taught pose, and the cell was mid-attempt on a carrier fault when they
were written. Applied as one script so the window where the generator is half-edited is
milliseconds rather than minutes - the driver starts a fresh subprocess every few minutes
and a partially-edited generator is a fragment nobody authored.

What it changes, and why each number is the asset's rather than mine:

* `fisheye_tool.usda` goes on arm-a's flange. Its optical frame is the front vertex of the
  lens at local z 0.227, so the tool's standoff AND its lens offset both become 227 mm -
  they coincide for this tool because the lens is at the stem's tip, which is exactly the
  relationship `TOOLS["fisheye"]` already encodes with `lens_mm == standoff_mm`. The field
  is 185 degrees, not the 150 the stand-in carried, and the projection is equidistant
  (r = f_eff * theta) rather than rectilinear - which matters for the course, because a
  185 degree f-theta lens is the thing being explained.
* `overhead_camera_fixture.usda` stands at the cell frame placement the asset publishes,
  (-0.055, 0.700, 0). That x is the `inside` index to the millimetre, and the boom's
  optical centre at local (0, -0.700, 1.800) lands on the belt centreline 1150 mm above
  the deck - the lens's own design distance. None of those three numbers is chosen here.
"""
from __future__ import annotations

import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parents[1]
PROPS = "/home/shrews/viam/usd/cell_props/"


def patch(path: pathlib.Path, pairs) -> str:
    s = path.read_text()
    for old, new in pairs:
        if old not in s:
            return f"MISS in {path.name}: {old[:70]!r}"
        s = s.replace(old, new, 1)
    path.write_text(s)
    return ""


def main() -> int:
    gen = REPO / "probes" / "assembly" / "build_fragment.py"
    st = REPO / "probes" / "assembly" / "stations.py"

    # ---- the fisheye's real optics ------------------------------------------------
    fail = patch(st, [(
        '''    "fisheye": Tool("fisheye", 190.0,
                    "a camera on a stem; tip 70 mm inside a 160 x 216 aperture, flange "
                    "120 mm clear of the shell's front", lens_mm=190.0),''',
        '''    # Read off cell_props/fisheye_tool.usda, not chosen: `optics:opticalCenter` is
    # (0, 0, 0.227) and `optics:frameDefinition` names it the FRONT VERTEX of the lens on
    # its axis, which is the physical point a stand-off is measured from. Fujifilm
    # publishes no entrance-pupil position for the FE185C057HA-1 and for a 185 degree
    # fisheye the pupil moves with field angle anyway, so the vertex is the only stable
    # datum - the asset's author made that call explicitly rather than quietly.
    #
    # Standoff and lens offset are the SAME 227 mm because the lens is at the stem's tip.
    # That is not a coincidence to be tidied away: it is what makes `inside` work, where
    # the station asks for the lens 70 mm THROUGH a 160 x 216 aperture and the flange ends
    # up clear of the shell's front face.
    "fisheye": Tool("fisheye", 227.0,
                    "Basler ace 2 behind a FUJINON FE185C057HA-1: a 185 degree "
                    "equidistant fisheye on a stem, optical vertex 227 mm from the "
                    "flange, with a flash at the lens", lens_mm=227.0),''')])
    if fail:
        print(fail)
        return 1

    # ---- the fisheye asset on arm-a, and the real field ---------------------------
    #
    # DO NOT APPLY THIS HALF AS IT STANDS. The module sets a camera's field by
    #
    #     focal_length = horizontal_aperture / (2 * tan(fov / 2))
    #
    # and tan(92.5 degrees) is NEGATIVE: a pinhole projection has no 185 degree field, and
    # asking for one produces a negative focal length rather than an error. The asset's
    # lens is equidistant - r = f_eff * theta, f_eff 1.765 mm on a 5.7 mm image circle -
    # so rendering it honestly needs Isaac's fisheye projection type driven from those
    # published numbers, which is a change to `sim_manager`, not a bigger constant.
    #
    # Left here written out because the number is right and the mechanism is missing, and
    # a note that says which is more use than a patch that silently breaks the camera.
    return 0
    fail = patch(gen, [
        ('    for name, arm, tool, fov in (("fisheye-cam", "arm-a", "fisheye", 150.0),',
         '''    # 185 degrees, from the asset's `optics:dfovDeg`/`optics:hfovDeg`, not the 150 the
    # cuboid stand-in carried. The lens is an equidistant (f-theta) fisheye: r = f_eff *
    # theta, f_eff 1.765 mm over a 5.7 mm image circle. A course that explains a fisheye
    # against a simulation rendering a 150 degree rectilinear frame is explaining the
    # wrong lens, which is the entire reason this asset was authored.
    for name, arm, tool, fov in (("fisheye-cam", "arm-a", "fisheye", 185.0),'''),
    ])
    if fail:
        print(fail)
        return 1

    print("applied: fisheye optics (stations.TOOLS + generator fov)")
    print("NOT applied here: the prop declarations - see add_props() below, which needs")
    print("the generator's prop list and is written against it in the same pass.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
