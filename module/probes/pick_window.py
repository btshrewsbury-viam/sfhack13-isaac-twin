"""Where can arm-a actually take the part off a conveyor, and at what deck height?

Two questions that have to be answered together, because the answer to each moves the
other. The real conveyor asset (ConveyorBelt_A08) has a 769.3 mm deck - the industry's
30-inch standard - where the slab it replaces was an invented 584. That spends 185 mm of
an arm whose reach is 1016.7 mm from a base at 391.4. And the break beam's position is
not a free choice either: it has to sit where the arm can reach the -x face, because the
beam is what defines the pick point.

So this sweeps the part down the conveyor's centreline at both deck heights and asks the
motion service, at the approach and at the grip, using the same `side_grip` the round
uses. The output is the window of y where a pick is possible - and the beam goes in it.
"""
import asyncio, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from qc.layout import Cell, FRAGMENT
from qc.round import Clients, lifted, props_of, world_state
from viam.components.arm import Arm
from viam.components.generic import Generic
from viam.components.gripper import Gripper
from viam.proto.common import Pose, PoseInFrame
from viam.robot.client import RobotClient
from viam.services.motion import MotionClient

def _props():
    import json
    world = next(c for c in json.loads(FRAGMENT.read_text())["components"]
                 if c["model"].endswith(":world"))
    return world["attributes"]["props"]


FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")

ORIGIN_TO_BOTTOM_MM = -7.2       # part bottom sits this far from its origin
ORIGIN_TO_CENTRE_Z_MM = 106.3
CONVEYOR_X_MM = 550.0            # the deck centreline
OVERHANG_MARGIN_MM = 50.0        # how much deck to leave under the leading edge


async def verdict(c, arm, pose, state) -> str:
    (x, y, z), orientation = pose
    try:
        await c.motion.move(
            component_name=arm,
            destination=PoseInFrame(reference_frame="world", pose=Pose(
                x=x, y=y, z=z, **orientation)),
            world_state=state)
        return "ok"
    except Exception as exc:  # noqa: BLE001
        text = str(exc)
        if "zero IK solutions" in text:
            return "no IK"
        if "did not settle" in text:
            return "no hold"
        if "collision" in text.lower():
            return "collide"
        # Never truncated. A planner failure names the geometry it hit, and that name
        # is the whole answer: `belt.base` turned out to be a box this repo invented
        # rather than anything the asset has, and 28 characters hid it for a cycle.
        return text


async def main() -> int:
    robot = await RobotClient.at_address(FQDN, RobotClient.Options.with_api_key(
        api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]))
    c = Clients(motion=MotionClient.from_robot(robot, "builtin"),
                world=Generic.from_robot(robot, "sim-world"),
                arms={n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b")},
                grippers={"arm-a": Gripper.from_robot(robot, "suction-a"),
                          "arm-b": Gripper.from_robot(robot, "suction-b")})
    cell = Cell.from_fragment()
    for g in c.grippers.values():
        await g.open()
    await c.world.do_command({"command": "reset"})
    await asyncio.sleep(3.0)
    # The part is parked far away so it is not its own obstacle during the sweep.
    props = await props_of(c)
    state = world_state(props, cell=cell, ignore=["part"])

    deck = cell.belt
    deck_far_y = deck.centre_mm[1] + deck.dims_mm[1] / 2.0
    half_len = cell.part.dims_mm[1] / 2.0
    # The part's centre sits this far from the origin its pose is placed at.
    spawn_y = next(pr["position"][1] * 1000.0 for pr in _props() if pr["name"] == "part")
    centre_off_y = cell.part.centre_mm[1] - spawn_y
    centre_z = deck.top_mm - ORIGIN_TO_BOTTOM_MM + ORIGIN_TO_CENTRE_Z_MM
    print(f"deck top {deck.top_mm:.1f}  deck runs to y {deck_far_y:.1f}  "
          f"part half-length {half_len:.1f}  part centre z {centre_z:.1f}")
    print(f"  {'pick y':>7} {'reach':>7}  {'on deck':<9} {'approach':<10} {'grip':<10}")
    best = []
    for y in range(-1100, -849, 10):
        centre = [CONVEYOR_X_MM, y + centre_off_y, centre_z]
        leading = centre[1] + half_len
        on_deck = leading <= deck_far_y - OVERHANG_MARGIN_MM
        pose = cell.side_grip(centre, "-x")
        r = cell.reach_mm("arm-a", pose[0])
        away = await verdict(c, "arm-a", lifted(pose, 140.0), state)
        near = await verdict(c, "arm-a", pose, state) if away == "ok" else "-"
        good = on_deck and near == "ok"
        if good:
            best.append((r, y))
        print(f"  {y:>7} {r:>7.0f}  {'yes' if on_deck else 'OVERHANGS':<9} "
              f"{away[:10]:<10} {near[:10]:<10}{'  <== USABLE' if good else ''}")
        if away not in ("ok",) and "obstacle" in away:
            hits = sorted({w.split()[0] for w in away.split("and ")[1:]})
            print(f"          blocked by {hits}")
    if best:
        shortest, where = min(best)
        print(f"\nbeam goes at y = {where}, reach {shortest:.0f} mm")
    else:
        print("\nNO usable pick point - move the conveyor rather than accept an "
              "off-deck stop")
    await robot.close()
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
