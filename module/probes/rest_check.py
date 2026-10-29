"""T1's acceptance: does the part rest where it is put, on the real conveyor?

The slab this replaces could not hold a mailbox - it slid and tipped - and the fix was
never "tune the physics", it was that a flat slab is not a conveyor. ConveyorBelt_A05
brings a roller deck, side guides and its own legs. Its deck is 769.3 mm, the 30-inch
industry standard, where the slab was an invented 584.

The deck is ROLLERS, not a plane: 31 cylinders on a ~64 mm pitch, each its own collider.
A mailbox rests on rails, and rails crossing rollers meet at points rather than along
lines, so whether this is stable is a question for the simulator rather than for me.

Accept: within 5 mm of the configured pose, tilt under 0.02 DEGREES, unchanged across
three consecutive resets.
"""
import asyncio, math, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from qc.layout import Cell, part_origin_offset_mm
from viam.components.generic import Generic
from viam.robot.client import RobotClient

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
TOL_MM, TOL_TILT = 5.0, 0.02


def tilt_deg(q) -> float:
    """How far the part's own up-axis has fallen away from world up."""
    w, x, y, z = q
    up_z = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, up_z))))


async def main() -> int:
    robot = await RobotClient.at_address(FQDN, RobotClient.Options.with_api_key(
        api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]))
    world = Generic.from_robot(robot, "sim-world")
    cell = Cell.from_fragment()
    want = cell.part.centre_mm
    offset = part_origin_offset_mm()
    print(f"configured part centre {[round(v, 1) for v in want]}   deck top "
          f"{cell.belt.top_mm:.1f}")

    ok = True
    for label in ("settle t=3s", "RESET #1", "RESET #2", "RESET #3"):
        if label.startswith("RESET"):
            await world.do_command({"command": "reset"})
        await asyncio.sleep(3.0)
        poses = await world.do_command({"command": "prop_poses", "names": ["part"]})
        p = poses.get("props", poses)["part"]
        # prop_poses reports the prim ORIGIN; the layout works in geometric centres.
        centre = [p["position_mm"][i] + offset[i] for i in range(3)]
        if "orientation_wxyz" not in p:
            raise KeyError(
                "prop_poses gave no orientation for the part, so tilt was never "
                "measured - a default of upright would have passed this test blind")
        t = tilt_deg(p["orientation_wxyz"])
        drift = max(abs(centre[i] - want[i]) for i in range(3))
        good = drift <= TOL_MM and t <= TOL_TILT
        ok = ok and good
        print(f"  {label:12s} centre {[round(v, 1) for v in centre]}  "
              f"tilt {t:.3f}  drift {drift:.1f} mm   {'ok' if good else 'FAIL'}")
    print("\nT1 ACCEPTANCE:", "PASS" if ok else "FAIL")
    await robot.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
