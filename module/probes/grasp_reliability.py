"""How often does a grasp actually hold, and does the gripper know when it does not?

A single successful pick proves the mechanism works; it does not prove the cell can run a
round. The recording showed the difference - `holding=True` with the carton still on the
belt - so this repeats the pick and scores it two ways that must agree:

  * what the gripper SAYS   - is_holding_something / status / gripped
  * what the simulator DID  - whether the part rose with the flange

A disagreement is worse than a failure. A round that trusts `holding` will carry nothing
to the inspection camera and score it.

    .venv/bin/python probes/grasp_reliability.py [attempts]
"""

import asyncio
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from obstacle_check import box, tool_transform  # noqa: E402
from qc.layout import Cell  # noqa: E402
from qc.round import Clients, feed, lifted, part_pose, props_of, world_state  # noqa: E402
from qc.round import move as round_move  # noqa: E402


TOOL_LENGTH_MM = 120.0


def grasp_height(props, part_pose):
    """Flange height that puts the cup on the part's top face.

    Derived from what the sim reports the part to be, not from a remembered number: the
    part changed from an 80 mm carton to a 6 mm steel blank, and a hard-coded half-height
    would have aimed the cup 37 mm inside it.
    """
    part = next((p for p in props if p["label"] == "part"), None)
    half = (part["dims_mm"][2] / 2.0) if part else 0.0
    return part_pose["position_mm"][2] + half + TOOL_LENGTH_MM
from viam.components.arm import Arm  # noqa: E402
from viam.components.camera import Camera  # noqa: E402
from viam.components.generic import Generic  # noqa: E402
from viam.components.gripper import Gripper  # noqa: E402
from viam.proto.common import (  # noqa: E402
    GeometriesInFrame, Pose, PoseInFrame, WorldState,
)
from viam.robot.client import RobotClient  # noqa: E402
from viam.services.motion import MotionClient  # noqa: E402

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
LIFT_MM = 250.0
CARRIED_TOLERANCE_MM = 30.0


# How far sideways the carry goes, along the part's own length.
CARRY_MM = 150.0


async def main():
    attempts = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    # `--film` pulls camera frames throughout, the way record_cell.py does. The pick is
    # 5/5 unfilmed and 0/N when recorded, and the scripts otherwise do the same moves -
    # so the question is whether rendering is what breaks it.
    filming = "--film" in sys.argv
    robot = await RobotClient.at_address(
        FQDN,
        RobotClient.Options.with_api_key(
            api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]
        ),
    )
    try:
        motion = MotionClient.from_robot(robot, "builtin")
        world = Generic.from_robot(robot, "sim-world")
        gripper = Gripper.from_robot(robot, "suction-a")
        arm = Arm.from_robot(robot, "arm-a")

        clients = Clients(
            motion=motion, world=world,
            arms={"arm-a": arm, "arm-b": Arm.from_robot(robot, "arm-b")},
            grippers={"arm-a": gripper,
                      "arm-b": Gripper.from_robot(robot, "suction-b")})

        frames = {"n": 0}
        camera = Camera.from_robot(robot, "overview-cam") if filming else None

        async def film():
            while filming:
                try:
                    await camera.get_images()
                    frames["n"] += 1
                except Exception:  # noqa: BLE001
                    pass
                await asyncio.sleep(1.0 / 12)

        film_task = asyncio.create_task(film()) if filming else None
        print(f"filming: {filming}")
        held, claimed, disagreed = 0, 0, 0
        print(f"{'try':>4} {'says':>6} {'status':>8} {'part rose':>10} {'verdict':>24}")
        for attempt in range(1, attempts + 1):
            await world.do_command({"command": "reset"})
            await asyncio.sleep(3.5)
            # Reset leaves the part upstream where it spawns; the conveyor and the beam
            # are what put it on the pick point. Scoring a grasp against a part still
            # 1.2 m up the belt would measure nothing at all.
            await feed(clients, Cell.from_fragment())
            props = list((await world.do_command({"command": "obstacles"}))["obstacles"])

            def state(with_part: bool):
                chosen = props if with_part else [
                    p for p in props if p["label"] != "part"]
                return WorldState(
                    obstacles=[GeometriesInFrame(
                        reference_frame="world",
                        geometries=[box(p["label"], p["center_mm"], p["dims_mm"])
                                    for p in chosen])],
                    transforms=[tool_transform("arm-a"), tool_transform("arm-b")],
                )

            # The pose the ROUND picks with, not one of this probe's own.
            #
            # This used to drive a top-down grasp - tool pointing -z, aimed at the part's
            # prim, at a computed roof height - which is not the grip the cell performs.
            # The round takes the part by a flat SIDE. On the cube both happened to work,
            # so the disagreement never showed; on a part with an arched roof the top-down
            # grasp cannot seal at all, and this probe reported 0/4 for a grip the cell
            # never attempts. A probe that scores the wrong motion is worse than none: it
            # answered "can the cell pick the part up" with evidence about something else.
            cell = Cell.from_fragment()
            props_all = await props_of(clients)
            # Where the part actually came to rest, which is what the round grips from.
            # The beam holds it to about 2 mm along the belt but its lateral rest varies
            # by 12 mm inside the guide channel, and the cup stands off only 6 mm.
            grip_pose = cell.side_grip(
                (await part_pose(clients))["position_mm"], cell.GRIP_SIDE)
            (gx, gy, gz), grip_orientation = grip_pose
            approach = lifted(grip_pose, 140.0)

            # Move the way the ROUND moves - its own `move`, its own world_state - rather
            # than with a state and tool transforms this probe builds itself.
            #
            # Asked directly, the grip pose reaches (probes/grip_face_reach.py). Asked
            # through this probe's hand-rolled state it produced "zero IK solutions" three
            # times running, and the difference was the asking, not the pose. A probe that
            # does not travel the round's code path answers a question nobody asked - which
            # is the second time that has cost a day here.
            async def move(pose, with_part, what="a move"):
                # Say WHICH move failed. One handler reporting "move failed" for the
                # approach, the grip, the lift and the carry alike sent me looking at the
                # grip pose when a probe had already proved that pose reaches.
                try:
                    await round_move(clients, "arm-a", pose,
                                     world_state(props_all, cell=cell,
                                                 ignore=[] if with_part else ["part"]))
                except Exception as exc:  # noqa: BLE001
                    raise RuntimeError(f"{what} to {[round(v) for v in pose[0]]}: "
                                       f"{str(exc)[:150]}") from exc

            try:
                await move(approach, True, "the approach")
                await move(grip_pose, False, "the grip")
                await gripper.grab()
                await asyncio.sleep(0.4)
                before = (await world.do_command(
                    {"command": "prop_poses", "names": ["part"]}))["props"]["part"]
                await move(lifted(grip_pose, LIFT_MM), False, "the lift")
                # A vertical lift only loads the cup along its axis. Carrying loads it
                # sideways, which is where a suction cup actually lets go - and it is what
                # the recorded take did when it dropped the carton.
                mid = (await world.do_command(
                    {"command": "prop_poses", "names": ["part"]}))["props"]["part"]
                rose_with_arm = abs(mid["position_mm"][2] - before["position_mm"][2]
                                    - LIFT_MM) < CARRIED_TOLERANCE_MM
                # Then carry it sideways, keeping the grip. A vertical lift only loads the
                # cup along its axis; this is the load that actually peels a cup off.
                # Carry ALONG the part's length rather than back towards the base: the
                # cup points away from the arm on this face, so pulling the flange inwards
                # while the tool points outwards is a pose the arm cannot make.
                (cx, cy, cz), carry_orientation = lifted(grip_pose, LIFT_MM)
                await move(((cx, cy + CARRY_MM, cz), carry_orientation), False,
                           "the carry")
            except Exception as exc:  # noqa: BLE001
                # The whole message. A settle failure says WHICH joint and whether a drive
                # is at its limit, and 40 characters of it said only "TimeoutError - ar".
                print(f"{attempt:>4} {'-':>6} {'-':>8} {'-':>10} {'move failed':>22}\n"
                      f"      {str(exc)[:400]}")
                continue

            after = (await world.do_command(
                {"command": "prop_poses", "names": ["part"]}))["props"]["part"]
            rose = after["position_mm"][2] - before["position_mm"][2]
            # Did the part come along on the sideways carry? Measured as the part's OWN
            # displacement, against the displacement the arm was asked for.
            #
            # This used to compare the part's x against `gx - 120`, the cup's position
            # if the tool pointed -x from the flange. On the -x face the cup points the
            # other way, at `gx + 120`, so the test looked for the part 330 mm from
            # where a perfectly good carry leaves it. It read 0/5 "dropped in the carry"
            # while the part was in fact being carried the whole way - a probe reporting
            # a cell failure that was its own.
            moved_y = after["position_mm"][1] - mid["position_mm"][1]
            held_z = abs(after["position_mm"][2] - mid["position_mm"][2])
            travelled = (abs(moved_y - CARRY_MM) < CARRIED_TOLERANCE_MM
                         and held_z < CARRIED_TOLERANCE_MM)
            says = (await gripper.is_holding_something()).is_holding_something
            status = (await gripper.do_command({"command": "status"}))["status"]
            carried = rose_with_arm and travelled

            held += carried
            claimed += says
            mismatch = says != carried
            disagreed += mismatch
            verdict = ("carried" if carried else
                       ("dropped in the carry" if rose_with_arm else "never lifted"))
            if mismatch:
                verdict += "  <-- GRIPPER LIED"
            print(f"{attempt:>4} {str(says):>6} {status:>8} {rose:>9.1f}mm {verdict:>24}")
            await gripper.open()

        if film_task:
            filming = False
            film_task.cancel()
            print(f"pulled {frames['n']} frames during the run")
        print(f"\nactually carried {held}/{attempts}; "
              f"claimed to be holding {claimed}/{attempts}; "
              f"disagreed {disagreed}/{attempts}")
        if disagreed:
            print("a round cannot trust is_holding_something until this is zero")
        return 0 if held == attempts and not disagreed else 1
    finally:
        await robot.close()


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
