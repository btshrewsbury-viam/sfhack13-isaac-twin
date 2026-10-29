"""Make the camera the thing that finds the hole, not the simulator.

The cell has pointed a RealSense at a hole and done nothing with what it saw for its whole
life. This is the half that acts on it, and it is written so the camera is LOAD-BEARING
rather than decorative - because a course about computer vision that quietly reads the
ground truth and shows you a picture afterwards is teaching the wrong lesson.

The split is deliberate and is stated in the cell's own terms:

* The bore's LATERAL position comes from the camera. Those are the two axes across the
  view, and they are the axes the carrier and the cradle leave an error in.
* The bore's DEPTH along its own axis comes from the belt index, which is closed-loop on
  the pallet's measured x to +/-1.5 mm. Depth into a 138 mm bore behind a 1.5 mm chamfer
  does not need better, and a single centroid cannot supply it anyway.
* The shell's ROTATION comes from the simulator. A single blob has no opinion about tilt,
  and saying otherwise would be the decoration this is written to avoid. The fixture's job
  is to keep that residual small; the camera's job is the error the fixture leaves.

The Jacobian is measured once per layout, during the teaching pass, by stepping the arm a
known distance and watching the centroid. That is what lets the correction carry the focal
length, the mount and the range without naming any of them - and what keeps a cycle down to
one frame and one short move.
"""
from __future__ import annotations

import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parents[1]

IMPORTS_OLD = "from assembly.replay import Legs, fly as fly_leg, straight_line  # noqa: E402"
IMPORTS_NEW = ("from assembly.replay import Legs, fly as fly_leg, straight_line  # noqa: E402\n"
               "from assembly import bore_cv                                   # noqa: E402")

CYCLE_OLD = '''    ("pin", "arm-b", "pin_pick", "grab:Pin", "arm-b: takes the hinge pin off the tray"),'''
CYCLE_NEW = '''    ("pin", "arm-b", "pin_bore_look", "look:Pin",
     "arm-b: the RealSense finds the hinge bore and measures the error"),
    ("pin", "arm-b", "pin_pick", "grab:Pin", "arm-b: takes the hinge pin off the tray"),'''

LOOK_OLD = '''    verb, _, which = (tool or "").partition(":")
    if verb == "grab" and cell is not None and which in ("Flag", "Pin", "Shell"):'''

LOOK_NEW = '''    verb, _, which = (tool or "").partition(":")
    if verb == "look":
        await look_for_the_hole(which, arm, pose, stamp, arms=arms, motion=motion,
                                world=world, state=state, cam=cam, legs=legs)
        if via is not None:
            await fly(via, "off")
        return
    if verb == "grab" and cell is not None and which in ("Flag", "Pin", "Shell"):'''

HELPER = '''
def origin_from_bore(rot, nom, seen, bore_local):
    """The shell's origin implied by where the camera says the BORE is.

    The camera measures a feature, not a frame. `into_frame` wants a frame. The two differ
    by the feature's own offset carried through the shell's rotation - on this shell the
    bore sits about 400 mm from the origin, so even a degree of residual tilt is 7 mm if it
    is quietly charged to translation instead.

        target = live + rot . bore,  and we want target = nom + bore + seen
        so      live = nom + seen - (rot . bore - bore)

    `seen` carries only the two axes across the camera's view; its component along the bore
    is zero by construction, so the depth stays with the belt index where it belongs.
    """
    spin = into_frame(rot, [0.0, 0.0, 0.0], bore_local, point=False)
    return [nom[i] + seen[i] - (spin[i] - bore_local[i]) for i in range(3)]


# What the camera last measured, per part, as a WORLD displacement in millimetres. Cleared
# at the top of every cycle: a correction from the previous mailbox is worse than none.
SEEN: Dict[str, List[float]] = {}


async def look_for_the_hole(which, arm, pose, stamp, *, arms, motion, world, state,
                            cam, legs):
    """Find the bore in the frame and remember how far off it is.

    Measured once per layout and then used: the teaching pass steps the arm a known
    distance along each axis it can correct along and watches what the centroid does,
    which is the Jacobian, and stores it beside the taught poses under the same
    fingerprint. Every pass after that is one frame and some arithmetic.

    The two axes are the world axes ACROSS the bore - the bore runs along Y, so they are X
    and Z. They are not chosen for the camera's convenience; they are the axes an error in
    the carrier or the cradle actually appears in, and the measured response is what maps
    them onto the image however the camera happens to be mounted.
    """
    SEEN.pop(which, None)
    axes = [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]

    async def step(delta):
        # Imported here rather than at module scope: the only other planned move in this
        # file goes through `replay.fly`, which does the same, and an import that exists
        # only for one closure is an import the next tidy-up deletes.
        from viam.proto.common import PoseInFrame

        here = await arms[arm].get_end_position()
        await motion.move(component_name=arm, destination=PoseInFrame(
            reference_frame="world", pose=Pose(
                x=here.x + delta[0], y=here.y + delta[1], z=here.z + delta[2],
                o_x=here.o_x, o_y=here.o_y, o_z=here.o_z, theta=here.theta)),
            world_state=state)
        await asyncio.sleep(0.4)

    try:
        jac, axes = bore_cv.load(arm, pose, stamp)
    except bore_cv.NoJacobian as why:
        if legs is None or not legs.teaching:
            # REPORT and carry on. A missing calibration is a reason to insert without a
            # vision correction, not a reason to abandon a mailbox: the chamfer still
            # captures 1.5 mm and the frame correction still runs.
            print(f"    {pose}: no camera calibration ({why}); "
                  f"inserting without a vision correction", flush=True)
            return
        print(f"    {pose}: measuring the camera's response (4 steps of "
              f"{bore_cv.PROBE_STEP_MM:.0f} mm)", flush=True)
        jac = await bore_cv.measure(cam, step, axes)
        bore_cv.save(arm, pose, stamp, jac, axes)
        print(f"    {pose}: {jac[0][0]:+.3f} {jac[0][1]:+.3f} / "
              f"{jac[1][0]:+.3f} {jac[1][1]:+.3f} px per mm", flush=True)

    try:
        off, far, how = await bore_cv.correction(cam, jac, axes)
    except RuntimeError as why:
        print(f"    {pose}: the camera did not find the bore ({why}); "
              f"inserting without a vision correction", flush=True)
        return
    if far > bore_cv.CEILING_MM:
        print(f"    {pose}: the camera puts the bore {far:.1f} mm off, past the "
              f"{bore_cv.CEILING_MM:.0f} mm this is willing to believe - that is not the "
              f"bore. Inserting without a vision correction. {how}", flush=True)
        return
    if far < bore_cv.FLOOR_MM:
        print(f"    {pose}: the camera puts the bore within {far:.2f} mm; nothing to "
              f"correct", flush=True)
        return
    print(f"    {pose}: the camera puts the bore {far:.2f} mm off "
          f"({['%+.2f' % v for v in off]}); pixel {how['pixel']} against centre "
          f"{how['centre']} from {how['dark_px']} dark px", flush=True)
    SEEN[which] = off


'''

SEAT_OLD = '''        f = blob["flange_mm"]
        at_f = into_frame(rot, live, [f[i] - nom[i] for i in range(3)], point=True)'''

SEAT_NEW = '''        # WHERE THE BORE IS COMES FROM THE CAMERA, where it can.
        #
        # `live` above is the simulator's answer, and using it for the translation makes
        # the camera decorative. What the camera measures is the LATERAL error - the two
        # axes across its view - so that is what it supplies, and the rest keeps its
        # source: the depth along the bore from the belt index, which is closed-loop to
        # 1.5 mm, and the rotation from the shell's frame, which no single centroid can
        # measure.
        # And the ROTATION has to be taken back out of it before it becomes an origin.
        #
        # What the camera measures is where the BORE is. What `into_frame` wants is where
        # the shell's ORIGIN is, and the two differ by the bore's offset carried through
        # the shell's rotation - about 400 mm of lever, so even a degree of residual tilt
        # is 7 mm if it is quietly charged to translation instead. Subtracting the spin
        # makes the corrected pose land the insert exactly on the bore the camera saw,
        # which is the one line of algebra this whole correction stands on:
        #
        #   target = live + rot . bore   and   live = nom + seen - (rot . bore - bore)
        #   so      target = nom + bore + seen, which IS the measured bore.
        seen = SEEN.pop(which, None)
        if seen is not None:
            live = origin_from_bore(rot, nom, seen,
                                    part("Shell").mm_at("assembly:pinBore:entry"))
            print(f"    {pose}: aiming at the bore the camera found, not the one the "
                  f"simulator reports ({['%+.2f' % v for v in seen]} mm lateral)",
                  flush=True)
        f = blob["flange_mm"]
        at_f = into_frame(rot, live, [f[i] - nom[i] for i in range(3)], point=True)'''


def main() -> int:
    p = REPO / "probes" / "assembly" / "film_cycle.py"
    s = p.read_text()
    for old, new in ((IMPORTS_OLD, IMPORTS_NEW), (CYCLE_OLD, CYCLE_NEW),
                     (LOOK_OLD, LOOK_NEW), (SEAT_OLD, SEAT_NEW)):
        if old not in s:
            print(f"MISS: {old.splitlines()[0][:70]!r}")
            return 1
        s = s.replace(old, new, 1)
    anchor = "def into_frame(rot, live, v, *, point: bool):"
    if anchor not in s:
        print("MISS: into_frame")
        return 1
    s = s.replace(anchor, HELPER.lstrip("\n") + anchor, 1)
    # `_station` needs the camera and a Dict/List import.
    s = s.replace("async def _station(arm, pose, tool, *, arms, grips, motion, world, "
                  "stamp, state,\n                   legs=None, cell=None):",
                  "async def _station(arm, pose, tool, *, arms, grips, motion, world, "
                  "stamp, state,\n                   legs=None, cell=None, cam=None):", 1)
    s = s.replace("                                   legs=legs, cell=cell)",
                  "                                   legs=legs, cell=cell, cam=realsense)", 1)
    # The RealSense, beside the overview camera that films the cycle. They are different
    # cameras doing different jobs and the names say so: `camera` watches, `realsense`
    # measures.
    old_cam = '    camera = Camera.from_robot(robot, "overview-cam")'
    if old_cam not in s:
        print("MISS: overview-cam open")
        return 1
    s = s.replace(old_cam, old_cam + '\n    realsense = Camera.from_robot(robot, "realsense-cam")', 1)
    # `SEEN` is annotated, so the names have to exist.
    old_imp = "import time\nfrom pathlib import Path"
    if old_imp not in s:
        print("MISS: import block")
        return 1
    s = s.replace(old_imp, "import time\nfrom pathlib import Path\nfrom typing import Dict, List", 1)
    # Nothing carries over between mailboxes.
    old_clear = '    cell = chosen()\n    stamp = layout_hash()'
    if old_clear not in s:
        print("MISS: cell/stamp block")
        return 1
    s = s.replace(old_clear, '    SEEN.clear()\n' + old_clear, 1)
    p.write_text(s)
    print("film_cycle.py: the camera now finds the bore")
    return 0


if __name__ == "__main__":
    sys.exit(main())
