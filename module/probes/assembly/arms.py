"""Keeping an arm inside the model the planner has of it.

The clamps in the fragment live in the PLANNER's kinematic model; the sim's joints carry
the asset's own wider limits. So a drive can be commanded past a clamp and come to rest
outside it, and from there IK cannot even seed - it reports "solver had an error: nlopt:
INVALID_ARGS", ten times over, which names nothing a reader can act on. Checking first
turns that into a recovery and costs one query.

Here rather than in a filming probe because the TEACHING pass needs it too: three stations
failed to teach in a row with exactly that error, each because the station before them had
left arm-b outside its own limits.
"""
from __future__ import annotations

import json
from typing import Dict, List, Tuple

from .build_fragment import FRAGMENT

ORDER = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
         "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

# How close to home still counts as parked. Not a precision pose - a park only has to be
# out of the way, and the arm it is getting out of the way of is 1 m long.
PARK_TOLERANCE_DEG = 8.0

# How far short of the deck's downstream end `clear_and_park` runs the pallet. A constant
# because a test holds it against the conveyor asset's uprights, and a test checking its
# own copy of a number checks nothing.
# 700, not 500: the door now lies flat in front of the shell, which takes the pallet's
# downstream reach from 362 to 461 mm.
CLEAR_BACKOFF_MM = 700.0

_LIMITS: Dict[str, List[Tuple[float, float]]] = {}
_HOMES: Dict[str, List[float]] = {}


def _read() -> None:
    frag = json.loads(FRAGMENT.read_text())
    for c in frag["components"]:
        if c.get("type") != "arm":
            continue
        wide = float(c["attributes"].get("joint_limit_deg", 180))
        named = c["attributes"].get("joint_limits_deg") or {}
        _LIMITS[c["name"]] = [tuple(named.get(j, (-wide, wide))) for j in ORDER]
        _HOMES[c["name"]] = list(c["attributes"]["home_joints_deg"])


def limits(name: str) -> List[Tuple[float, float]]:
    if not _LIMITS:
        _read()
    return _LIMITS[name]


def home(name: str) -> List[float]:
    if not _HOMES:
        _read()
    return _HOMES[name]


def adrift(name: str, joints) -> List[Tuple[int, float, Tuple[float, float]]]:
    """Which joints have come to rest outside the planner's model of this arm."""
    lim = limits(name)
    return [(i, round(v, 1), lim[i]) for i, v in enumerate(joints)
            if not lim[i][0] <= v <= lim[i][1]]


async def park_if_adrift(handle, name: str, quiet: bool = False) -> bool:
    """Send an arm home if any joint has come to rest outside the planner's model."""
    from viam.components.arm import JointPositions

    off = adrift(name, list((await handle.get_joint_positions()).values))
    if not off:
        return False
    if not quiet:
        print(f"    {name} is outside its limits at {off}; parking first", flush=True)
    await handle.move_to_joint_positions(JointPositions(values=home(name)), timeout=180)
    return True


async def clear_and_park(world, arms, cell, where_mm: float | None = None) -> None:
    """Get the cell back to a state something can be taught from.

    A pass that aborts leaves an arm mid-station and the carried set wherever the belt had
    indexed it, and the next pass then cannot even park: the park swings through the part
    that is standing in it, which reads as "something stopped the arm rather than
    out-torqued it". Parking first and clearing after does not work either, for the same
    reason. So the part is run downstream of every station, every arm parks, and the part
    comes back. Not out of the cell - a 2 m conveyor with 640 mm of upright at one end has
    nowhere to put it - just out from under whichever arm could not park.

    Running unattended, this is the difference between a bad run costing one pass and
    costing every pass after it.
    """
    from .fixture import index_to
    from viam.components.arm import JointPositions

    # RESET FIRST, and this is load-bearing rather than tidy.
    #
    # The belt will not move a pallet that has been sitting still since the module booted:
    # measured, twelve indexes in a row reported "pallet at -55, +207 mm to go" with the
    # pallet having moved 0.00 mm, while the conveyor reported running, unblocked, and
    # carrying surfaceVelocity (0.42, 0, 0) in world space with every attribute correct.
    # Run the same belt straight after a `reset` and the pallet covers 1413 mm in 7
    # seconds. Parking the arms alone does not do it; `reset` does, because it moves every
    # prop and re-reads the scene. Whatever PhysX is doing with a surface velocity applied
    # to a body it has already created, a reset is what makes it take.
    #
    # So the function whose job is "get the cell back to a state something can be taught
    # from" has to include it, rather than leaving every caller to remember.
    # `restage`, not a bare reset. A reset with the door's hinge live lands the shell
    # 4.15 degrees out of square and 5.2 mm high, every time - see `fixture.restage` for
    # the measurement. This function's whole job is "get the cell back to a state
    # something can be taught from", so a reset that leaves the datum crooked is exactly
    # what it must not do.
    from .fixture import restage

    await restage(world, settle=3.0)

    # Downstream of every station, and clear of the conveyor's own uprights.
    #
    # 300 mm was chosen against a cube deck, which could be any length and had nothing on
    # it. The real asset is 1999 mm with two full-height uprights at its far end - x
    # 1363..1438 in this cell - and the pallet reaches 235 mm downstream of its index, so
    # belt_to_x - 300 left 53 mm of clearance for a 40 kg pallet pushed by friction.
    #
    # 500 is the most the geometry allows: it puts the pallet's downstream edge at 1235,
    # 128 mm short of the posts, and the part 99 mm past the last station. Which is less
    # than this function's docstring claims - the part does not LEAVE the cell on a 2 m
    # conveyor with 640 mm of post at one end, it only comes out from under the arm that
    # could not park. That is the part of the job that matters: the first park is what
    # fails, and it fails because an arm at a station cannot swing through the part it is
    # working on. `tests/test_assembly_fragment.py` holds this number against the posts.
    far = cell.belt_to_x_mm - CLEAR_BACKOFF_MM if where_mm is None else where_mm

    # PARK, THEN INDEX, THEN PARK AGAIN - and the repeat is not belt and braces.
    #
    # The two orders fail in opposite ways and each fixes the other. Indexing first jams
    # the pallet against whichever arm is still standing over the belt: harmless while
    # indexing was a teleport, and not now a driven belt has to push 40 kg past it -
    # measured, "pallet at 330, +2570 mm to go" about a pallet with an arm in front of it.
    # Parking first fails the other way: an arm AT a station cannot always park through
    # the part it is working on, which is what this function was written for.
    #
    # So park what will park, move the part out of the way, and park the rest. Whichever
    # arm was stuck is no longer stuck once the part has gone.
    for _ in range(2):
        for name, handle in arms.items():
            try:
                await handle.move_to_joint_positions(
                    JointPositions(values=home(name)), timeout=180)
            except Exception:                                      # noqa: BLE001
                pass
        await index_to(world, cell, far)
    for name, handle in arms.items():
        for attempt in range(3):
            try:
                await handle.move_to_joint_positions(
                    JointPositions(values=home(name)), timeout=180)
                break
            except Exception as exc:                               # noqa: BLE001
                # "A short way short with effort to spare" is a drive that has stopped
                # moving because its stiffness times its error equals the load, which is
                # where a position-controlled drive lives. Retrying it twice more just
                # repeats the same measurement. What matters for a park is whether the arm
                # is BROADLY home, so that is what gets checked.
                near = max(abs(a - b) for a, b in
                           zip((await handle.get_joint_positions()).values, home(name)))
                if near <= PARK_TOLERANCE_DEG:
                    break
                print(f"    park {name} ({attempt}, {near:.1f} deg out): "
                      f"{str(exc)[:110]}", flush=True)
