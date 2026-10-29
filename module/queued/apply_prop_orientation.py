"""Let a prop be spawned and placed at a full orientation, not only a yaw.

Needed for one specific thing the cell has been asked for and cannot currently express:
the mailbox's door lying FLAT in the jig with the pin bore already lined up. The asset
publishes exactly that pose - `assembly:flatPosition` (0.4346, 0, -0.0212) and
`assembly:flatOrientation` (0.707107, 0, 0.707107, 0), which is 90 degrees about Y - and
the prop spec can only say `yaw_deg`, a rotation about Z. There is no yaw that lays a door
down.

The existing restriction was a considered one, and its reasoning is in the code: "a part
standing on a surface has one free rotation and admitting the other two would invite poses
that cannot rest". That holds for a part the cell drops onto a tray. It does not hold for a
part the ASSET says where to put - a published pose is not a pose someone guessed, and
refusing to express it means the cell can only ever show the door shut.

So `orientation_wxyz` is added beside `yaw_deg` rather than replacing it, and the comment
keeps the reason the restriction existed, because the next person to add a free rotation
to a prop that does have to rest on something should read it first.
"""
from __future__ import annotations

import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parents[1]

SPAWN_OLD = '''            yaw = float(prop.get("yaw_deg", 0.0))'''

SPAWN_NEW = '''            yaw = float(prop.get("yaw_deg", 0.0))
            # ...unless the ASSET says otherwise, in which case say what it says.
            #
            # The restriction above is right for a part this cell drops onto a tray: it
            # has one free rotation and the other two only invite poses that cannot rest.
            # It is wrong for a part whose pose is PUBLISHED. The mailbox's door lies flat
            # in the jig at `assembly:flatOrientation` (0.707107, 0, 0.707107, 0), which is
            # 90 degrees about Y, and no yaw lays a door down - so without this the cell
            # can only ever show the door shut, which is not the cell that was asked for.
            orient = prop.get("orientation_wxyz")'''

AUTHOR_OLD = '''            _x.AddTranslateOp().Set(_Gf.Vec3d(*[float(v) for v in position]))
            if yaw:
                _x.AddRotateZOp().Set(float(yaw))'''

AUTHOR_NEW = '''            _x.AddTranslateOp().Set(_Gf.Vec3d(*[float(v) for v in position]))
            if orient is not None:
                w, qx, qy, qz = (float(v) for v in orient)
                # One op, not an orient composed with a rotate: a prop's transform is the
                # only thing in it by construction, and two rotation ops is how that stops
                # being true.
                _x.AddOrientOp().Set(_Gf.Quatf(w, _Gf.Vec3f(qx, qy, qz)))
            elif yaw:
                _x.AddRotateZOp().Set(float(yaw))'''


def main() -> int:
    p = REPO / "src" / "isaac_module" / "sim_manager.py"
    s = p.read_text()
    for old, new in ((SPAWN_OLD, SPAWN_NEW), (AUTHOR_OLD, AUTHOR_NEW)):
        if old not in s:
            print(f"MISS: {old[:60]!r}")
            return 1
        s = s.replace(old, new, 1)
    p.write_text(s)
    print("sim_manager.py: props accept orientation_wxyz")
    return 0


if __name__ == "__main__":
    sys.exit(main())
