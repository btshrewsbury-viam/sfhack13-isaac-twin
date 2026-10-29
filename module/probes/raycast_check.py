"""Does PhysX see the part? The question five failed hypotheses needed answering first.

A bounding box says where geometry IS. A scene query says what the simulator will REPORT,
and those are different: a collider can be composed into the stage, measure correctly, and
still not be in the physics scene. The suction gripper finds its payload by raycast, so
when it reports nothing on a face that measures solid and sits 0.4 mm from the cup, the
only way to tell which of the two is wrong is to cast the ray.

Three casts: straight down onto the roof, along the cup's own axis from the grip pose, and
at the belt as a control - if the belt answers and the part does not, the part is not in
the physics scene.
"""
import asyncio, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from qc.layout import Cell
from viam.components.generic import Generic
from viam.robot.client import RobotClient

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")


async def main() -> int:
    robot = await RobotClient.at_address(FQDN, RobotClient.Options.with_api_key(
        api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]))
    world = Generic.from_robot(robot, "sim-world")
    cell = Cell.from_fragment()
    try:
        await world.do_command({"command": "reset"})
        await asyncio.sleep(2.0)
        centre = cell.part.centre_mm
        (gx, gy, gz), _ = cell.pick("+y")
        # The cup face: the flange pose less the tool along the grip axis.
        cup = (gx, gy - cell.tool_mm, gz)
        casts = [
            ("down onto the part's roof",
             [centre[0], centre[1], centre[2] + 300.0], [0, 0, -1], 400.0),
            ("along the cup axis from the cup face", list(cup), [0, -1, 0], 100.0),
            ("along the cup axis from the flange", [gx, gy, gz], [0, -1, 0], 300.0),
            ("down onto the belt (control)",
             [centre[0], centre[1] - 300.0, centre[2] + 300.0], [0, 0, -1], 500.0),
        ]
        for label, origin, along, reach in casts:
            got = (await world.do_command({
                "command": "raycast", "from_mm": origin,
                "along": along, "distance_mm": reach}))["ray"]
            if got.get("hit"):
                print(f"  {label:42s} HIT {got['body']} at {got['distance_mm']} mm")
            else:
                print(f"  {label:42s} nothing within {reach:.0f} mm "
                      f"from {got['from_mm']} along {got['along']}")
        print(f"\n  part centre {[round(v, 1) for v in centre]}, "
              f"grip flange {[round(v, 1) for v in (gx, gy, gz)]}, "
              f"cup face {[round(v, 1) for v in cup]}")
        return 0
    finally:
        await robot.close()


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
