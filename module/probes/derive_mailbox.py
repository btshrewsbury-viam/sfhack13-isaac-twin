"""Measure the mailbox asset in the running scene, so nothing about it is written down.

The cube part carried its marks as children, so the cell could read the fragment and work
out which face each mark sat on. A referenced asset carries them as geometry, and its own
published bounds file only covers the outer box - so the facts the layout needs (how big
the part is once scaled, where the cup should land on a flat side, which face each decal
is on) have to come from the composed stage.

They are measured here and written to `fragments/mailbox-derived.json`, which the layout
reads offline. Generated, not authored: `tests/test_mailbox_asset.py` re-measures against
a live cell and fails if the file and the asset have drifted apart.
"""
import asyncio, json, math, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from viam.components.generic import Generic
from viam.robot.client import RobotClient

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
PROP = os.environ.get("MAILBOX_PROP", "mailbox_probe")
OUT = Path(__file__).resolve().parents[1] / "fragments" / "mailbox-derived.json"
FRAGMENT = Path(__file__).resolve().parents[1] / "fragments" / "qc-cell.json"
DECALS = ("Smudge", "Drip", "Overspray")
# The collider pieces, measured separately, because the OUTER bound of the part is not
# the surface a cup can seal on. This asset's +y bound is its flag - a 14 mm arm standing
# 8.5 mm proud of the wall - so a cup aimed at the bound stops short of the wall
# everywhere the flag is not, and cannot seal on the flag where it is. The hull is the
# wall; the flag and the rails are the things to keep a cup away from.
WALL = "Body/Collision/Hull"
PROUD = ("Body/Collision/Flag", "Body/Collision/RailLeft", "Body/Collision/RailRight")
GROUND_TRUTH = ("mailbox:serial", "mailbox:scheme", "mailbox:schemeId",
                "mailbox:paintDefect", "mailbox:expectedPaintPass",
                "mailbox:doorCondition", "mailbox:expectedDoorPass",
                "mailbox:collisionBoundsMin", "mailbox:collisionBoundsMax")


def configured_yaw() -> float:
    """The yaw the fragment asks the part to be spawned at."""
    fragment = json.loads(FRAGMENT.read_text())
    world_cfg = next(c for c in fragment["components"]
                     if c["model"].endswith(":world"))
    prop = next(x for x in world_cfg["attributes"]["props"] if x["name"] == PROP)
    return float(prop.get("yaw_deg", 0.0))


def face_of(offset, half):
    """Which face a mark sits on: the axis it is furthest out along, as a fraction."""
    axis = max(range(3), key=lambda i: abs(offset[i]) / max(half[i], 1e-9))
    return f"{'+' if offset[axis] > 0 else '-'}{'xyz'[axis]}"


async def main() -> int:
    robot = await RobotClient.at_address(FQDN, RobotClient.Options.with_api_key(
        api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]))
    world = Generic.from_robot(robot, "sim-world")
    root = f"/World/{PROP}"
    try:
        paths = [root] + [f"{root}/Body/Decals/{d}" for d in DECALS]
        reply = await world.do_command({"command": "prim_bounds", "paths": paths})
        bounds = reply["bounds"]
        whole = bounds.get(root)
        if not whole:
            print(f"  {root} has no extent in the stage - is the prop spawned?")
            return 1
        dims_mm = [(whole["max"][i] - whole["min"][i]) * 1000.0 for i in range(3)]
        half = [v / 2.0 for v in dims_mm]
        centre = whole["centre"]
        # Where the geometry sits relative to the prim's ORIGIN, which is what the
        # fragment's `position` places. They are not the same point: this asset's origin is
        # at the bottom of the shell, so a part spawned at the height the cube part used
        # would hang 53 mm above the belt and drop, and a cup aimed at the prim would land
        # 39 mm below the middle of the panel. Both are measured rather than worked out.
        spawned = await world.do_command({"command": "prop_poses", "names": [PROP]})
        origin_m = spawned["props"][PROP]["position_mm"]
        # Refuse to measure a part that is not sitting the way it was configured.
        #
        # The offsets below are WORLD-frame - where the geometry sits relative to the prim
        # its pose is reported at - so they are only meaningful while the part is in its
        # intended orientation. Measured once from a part that had been spawned intersecting
        # the belt and shoved out by physx: the offsets came out of a tipped part, were
        # written to the derived file, and the spawn computed from them left the next part
        # 75 mm in the air. A settled part is the only honest thing to measure.
        want_yaw = float(configured_yaw())
        got = spawned["props"][PROP].get("orientation_wxyz")
        if got is not None:
            yaw = math.degrees(2.0 * math.atan2(
                abs(got[3]), abs(got[0]))) * (1 if got[3] * got[0] >= 0 else -1)
            drift = abs((yaw - want_yaw + 180.0) % 360.0 - 180.0)
            if drift > 5.0:
                print(f"  refusing to measure: the part is yawed {yaw:.1f} degrees and the "
                      f"config asks for {want_yaw:.1f}. It has been moved or tipped - let "
                      f"it settle in its intended pose first.")
                return 1
        origin_to_centre = [centre[i] * 1000.0 - origin_m[i] for i in range(3)]
        origin_to_bottom = whole["min"][2] * 1000.0 - origin_m[2]

        faces, drops, sizes = {}, [], {}
        for decal in DECALS:
            got = bounds.get(f"{root}/Body/Decals/{decal}")
            if not got:
                print(f"  decal {decal}: no extent (inactive variants have none)")
                continue
            offset = [(got["centre"][i] - centre[i]) * 1000.0 for i in range(3)]
            faces[decal.lower()] = face_of(offset, half)
            drops.append(offset[2])
            # How big the mark is, so the cell can still ask whether the camera could
            # resolve it. The cube part stated each mark's scale in the fragment; an
            # asset's marks are geometry, and this is the same number measured.
            sizes[decal.lower()] = [
                round((got["max"][i] - got["min"][i]) * 1000.0, 1) for i in range(3)]

        # Where a suction cup can actually seal: on the hull's own +y face, at an x the
        # flag and rails do not reach. Derived by measuring those pieces rather than read
        # off the asset's notes, so it follows the asset if its furniture moves.
        extra = [f"{root}/{WALL}"] + [f"{root}/{q}" for q in PROUD]
        pieces = (await world.do_command(
            {"command": "prim_bounds", "paths": extra}))["bounds"]
        wall = pieces.get(f"{root}/{WALL}")
        patches = {}
        if wall:
            # Every face a cup might take the part by, not just one. Which face the cell
            # SHOULD use depends on the arm's side of the cell and on the cantilever - a cup
            # on an end carries the whole length off its face - so the honest thing to
            # measure is all four and let the layout choose.
            for face, axis, sign in (("+x", 0, 1), ("-x", 0, -1),
                                     ("+y", 1, 1), ("-y", 1, -1)):
                along = 1 - axis          # the face's long direction, in world terms
                edge = ((wall["max"][axis] if sign > 0 else wall["min"][axis])
                        - centre[axis]) * 1000.0
                blocked = []
                for q in PROUD:
                    got = pieces.get(f"{root}/{q}")
                    if not got or got in (0,):
                        continue
                    # Only furniture standing proud of THIS face blocks a cup on it.
                    reach = ((got["max"][axis] if sign > 0 else got["min"][axis])
                             - centre[axis]) * 1000.0
                    if sign * (reach - edge) <= 0.2:
                        continue
                    blocked.append((
                        (got["min"][along] - centre[along]) * 1000.0,
                        (got["max"][along] - centre[along]) * 1000.0))
                span = ((wall["min"][along] - centre[along]) * 1000.0,
                        (wall["max"][along] - centre[along]) * 1000.0)
                edges = sorted([span[0]] + [e for pair in blocked for e in pair]
                               + [span[1]])
                windows = [(lo, hi) for lo, hi in zip(edges, edges[1:])
                           if not any(b[0] - 0.1 <= (lo + hi) / 2 <= b[1] + 0.1
                                      for b in blocked)]
                best = max(windows, key=lambda w: w[1] - w[0]) if windows else span
                patches[face] = {
                    "standoff_mm": round(abs(edge), 1),
                    "along_axis": "xyz"[along],
                    "offset_mm": round((best[0] + best[1]) / 2.0, 1),
                    "window_mm": [round(best[0], 1), round(best[1], 1)],
                    "clear_mm": round(best[1] - best[0], 1),
                    "blocked_mm": [[round(a, 1), round(b, 1)] for a, b in blocked],
                    "height_mm": [round((wall["min"][2] - centre[2]) * 1000.0, 1),
                                  round((wall["max"][2] - centre[2]) * 1000.0, 1)],
                }
        grip = patches.get("+y")

        masses = (await world.do_command(
            {"command": "prim_masses", "paths": [root]}))["masses"]
        truth = (await world.do_command(
            {"command": "prop_attributes", "prop": PROP,
             "names": list(GROUND_TRUTH)}))["attributes"]
        variants = (await world.do_command(
            {"command": "variants", "prop": PROP}))["variants"]

        derived = {
            "_comment": [
                "Measured from the mailbox asset in a running scene by",
                "probes/derive_mailbox.py. Do not edit: re-run the probe.",
                "tests/test_mailbox_asset.py re-measures and fails on drift.",
            ],
            "usd_path": "/home/shrews/viam/usd/mailbox/mailbox.usda",
            "dims_mm": [round(v, 1) for v in dims_mm],
            "origin_to_centre_mm": [round(v, 1) for v in origin_to_centre],
            "origin_to_bottom_mm": round(origin_to_bottom, 1),
            "grip_drop_mm": round(sum(drops) / len(drops), 1) if drops else 0.0,
            "decal_faces": faces,
            "decal_dims_mm": sizes,
            "grip_patch": grip,
            "grip_faces": patches,
            "variant_sets": variants,
            "ground_truth": truth,
        }
        OUT.write_text(json.dumps(derived, indent=2) + "\n")
        # And state the obstacle box in the fragment the module reads. The module can
        # measure this itself from a running stage, but the offline tests have no stage and
        # still have to build the obstacle list, so the declaration is what they read. It
        # is generated here rather than typed, and rounded OUTWARD - a box a tenth of a
        # millimetre short of the part is a box the planner will clear paths the part does
        # not fit through.
        import math as _math
        fragment = json.loads(FRAGMENT.read_text())
        world_cfg = next(c for c in fragment["components"]
                         if c["model"].endswith(":world"))
        prop = next(x for x in world_cfg["attributes"]["props"]
                    if x["name"] == PROP)
        lo = [_math.floor((origin_to_centre[i] - dims_mm[i] / 2.0) * 10.0) / 10000.0
              for i in range(3)]
        hi = [_math.ceil((origin_to_centre[i] + dims_mm[i] / 2.0) * 10.0) / 10000.0
              for i in range(3)]
        prop["bounds_m"] = {"min": lo, "max": hi}
        FRAGMENT.write_text(json.dumps(fragment, indent=2) + "\n")
        print(f"  wrote bounds_m into {FRAGMENT.name}: "
              f"{[round((hi[i]-lo[i])*1000, 1) for i in range(3)]} mm")
        for face, got in sorted(patches.items()):
            print(f"  face {face}: standoff {got['standoff_mm']:.0f} mm, "
                  f"{got['clear_mm']:.0f} mm clear along {got['along_axis']} "
                  f"at offset {got['offset_mm']:+.0f}, blocked {got['blocked_mm']}")
        print(f"  origin -> centre {derived['origin_to_centre_mm']} mm, "
              f"origin -> bottom {derived['origin_to_bottom_mm']} mm")
        print(f"  dims {derived['dims_mm']} mm, grip drop "
              f"{derived['grip_drop_mm']} mm, faces {faces}")
        print(f"  mass {masses}")
        print(f"  variants {variants}")
        print(f"  wrote {OUT.name}")
        return 0
    finally:
        await robot.close()


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
