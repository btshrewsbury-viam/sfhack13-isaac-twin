"""Does a grab actually HOLD anything? Measured, not reported.

`gripped_objects` saying something and the part moving with the tool are two different
claims, and the cell has been passing on neither. The seated-at-0.00-mm result comes from
`join` snapping the part onto its published frame, which happens whether or not the arm
ever carried it; and the cycle's arm-c steps have no tool at all, so its "takes the shell"
caption is motion over a part it never touches.

So this asks the only question that matters, for each pick, in order:

  1. what the cup's raycast hits at the taught pose, and how far away it is
  2. whether `gripped_objects` is non-empty after the grab
  3. whether the part MOVES WITH THE TOOL when the arm backs off to its stand-off

(3) is the one that cannot be faked. A part that stays behind while the flange leaves was
never held, whatever the gripper said.
"""
from __future__ import annotations

import asyncio
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from assembly.asset import insertions                              # noqa: E402
from assembly.build_fragment import FRAGMENT, chosen               # noqa: E402
from assembly.creds import options, FQDN                           # noqa: E402
from assembly.arms import clear_and_park, home as arm_home         # noqa: E402
from assembly.fixture import index_to, restock_all, settle_tray      # noqa: E402
from assembly.reach_check import (POSES, layout_hash,              # noqa: E402
                                  orientation_for)
from viam.proto.common import Pose, PoseInFrame                    # noqa: E402
from viam.services.motion import MotionClient                      # noqa: E402
from viam.components.arm import Arm, JointPositions                # noqa: E402
from viam.components.generic import Generic                        # noqa: E402
from viam.components.gripper import Gripper                        # noqa: E402
from viam.robot.client import RobotClient                          # noqa: E402

# arm, task, index, gripper, the prop it is supposed to pick up
PICKS = (("arm-b", "pin_pick", "pin", "grip-b", "pin"),
         ("arm-b", "flag_pick", "flag", "grip-b", "flag"),
         ("arm-c", "grasp", "handle", "grip-c", "shell"))
# How far a part has to follow the tool before it counts as held. The stand-off legs are
# 80-150 mm, so a part that is holding on moves most of that; one that was left behind
# moves the millimetre or two that letting go of it costs.
HELD_MM = 25.0


def _cup(name: str):
    """Where this gripper's cup sits on its flange, and how far it can reach."""
    blob = json.loads(FRAGMENT.read_text())
    for c in blob["components"]:
        if c["name"] == name:
            a = c["attributes"]
            return ([float(v) * 1000.0 for v in a["offset"]],
                    float(a.get("max_grip_distance", 0.02)) * 1000.0)
    raise KeyError(name)


async def main() -> int:
    robot = await RobotClient.at_address(FQDN, options())
    world = Generic.from_robot(robot, "sim-world")
    arms = {n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b", "arm-c")}
    grips = {n: Gripper.from_robot(robot, n) for n in ("grip-b", "grip-c")}
    motion = MotionClient.from_robot(robot, "builtin")
    cell, stamp = chosen(), layout_hash()
    print(f"  layout {stamp}\n")

    for which in ("flag", "door", "pin"):
        await world.do_command({"command": "unjoin", "name": f"{which}_seat"})
        # And the carrier CLIPS, in the same breath and for the same reason.
        #
        # Removing a joint prim does not take effect until physx re-reads the
        # scene, and `reset` is what makes that happen - so a clip unjoined
        # AFTER the reset is still live, and the restock that follows is
        # teleporting a body that is fixed to a 40 kg pallet. Measured: the pin
        # left at z 211219.8, x 2209, three placements in a row, every run.
        await world.do_command({"command": "unjoin",
                                "name": f"{which}_clip"})
    await world.do_command({"command": "reset"})
    await asyncio.sleep(3.0)
    await clear_and_park(world, arms, cell)
    await restock_all(world, cell)
    # The door, jointed, exactly as the cycle has it. arm-c's cup comes in at y -94 and
    # the door spans y +-83.7, so whether the door is there and where it is hanging is
    # part of the question this probe is asking.
    d = insertions()["Door"]
    await world.do_command({
        "command": "join", "name": "door_seat",
        "body0": "/World/shell", "body1": "/World/door", "kind": d.joint,
        "local0_mm": list(d.joint_anchor_mm),
        "local1_mm": [d.joint_anchor_mm[i] - d.seated_mm[i] for i in range(3)],
        "axis": list(d.joint_axis_mm), "limits_deg": list(d.joint_limits_deg),
        **({"drive": {k: v for k, v in d.joint_drive.items()
                      if not k.startswith("_")}} if d.joint_drive else {})})
    await asyncio.sleep(2.0)

    verdicts = []
    for arm, task, index, grip, prop in PICKS:
        blob = json.loads((POSES / arm / f"{task}.json").read_text())
        if blob["layout"] != stamp:
            print(f"  {task}: taught against {blob['layout']}, skipping")
            continue
        await index_to(world, cell, cell.index_x(index))
        await settle_tray(world, cell, [(n, pr) for n, pr in
                                        (("Flag", "flag"), ("Pin", "pin"))])
        await asyncio.sleep(1.0)

        clear = POSES / arm / f"{task}.clear.json"
        standoff = (json.loads(clear.read_text())["joints_deg"]
                    if clear.exists() else arm_home(arm))
        await arms[arm].move_to_joint_positions(
            JointPositions(values=standoff), timeout=180)
        await grips[grip].open()
        await arms[arm].move_to_joint_positions(
            JointPositions(values=blob["joints_deg"]), timeout=180)
        await asyncio.sleep(1.0)

        # 1 - what is in front of the cup, from the cup, along the tool axis.
        offset, reach = _cup(grip)
        flange = f"/World/{arm.replace('-', '_')}/wrist_3_link/flange"
        poses = (await world.do_command(
            {"command": "prim_poses", "paths": [flange]}))["prims"][flange]
        at = poses["position_mm"]
        axis = poses["x_axis"]          # the tool runs along the flange's local +x
        cup = [at[i] + axis[i] * offset[0] for i in range(3)]
        ray = (await world.do_command({
            "command": "raycast", "from_mm": cup, "along": list(axis),
            "distance_mm": max(reach * 4.0, 60.0)}))["ray"]
        hit = ray.get("body") or ray.get("collider") or "-"
        gap = ray.get("distance_mm")
        wanted = f"/World/{prop}"
        aimed = "AT THE PART" if wanted in str(hit) else f"at {hit}"
        in_reach = (gap is not None and gap <= reach)
        print(f"  {task}")
        print(f"      cup at {['%.1f' % v for v in cup]} along "
              f"{['%+.3f' % v for v in axis]}")
        print(f"      ray hits {aimed}, {gap if gap is None else round(gap, 1)} mm away; "
              f"cup reaches {reach:.0f} mm -> {'IN REACH' if in_reach else 'TOO FAR'}")

        # 2 - what the gripper says.
        before = (await world.do_command(
            {"command": "prim_poses", "paths": [wanted]}))["prims"][wanted]["position_mm"]
        await grips[grip].grab()
        await asyncio.sleep(0.8)
        said = await grips[grip].do_command({"command": "status"})
        print(f"      after grab: {said}")

        # 3 - does the part come with it?
        #
        # LIFTED, not retreated. Backing out along the stand-off is sideways for a part
        # held by its side wall, and sideways drags the shell against the cradle lip that
        # is there to stop exactly that: measured, the suction attached and the shell moved
        # 0.5 mm, which reads as a failed grab and is a failed TEST. The cycle lifts it,
        # so the test lifts it.
        try:
            await motion.move(
                component_name=arm,
                destination=PoseInFrame(reference_frame="world", pose=Pose(
                    x=blob["flange_mm"][0], y=blob["flange_mm"][1],
                    z=blob["flange_mm"][2] + 80.0,
                    **orientation_for(blob["approach"]))))
        except Exception as exc:                                   # noqa: BLE001
            print(f"      lift refused: {str(exc)[-90:]}", flush=True)
            await arms[arm].move_to_joint_positions(
                JointPositions(values=standoff), timeout=180)
        await asyncio.sleep(1.2)
        after = (await world.do_command(
            {"command": "prim_poses", "paths": [wanted]}))["prims"][wanted]["position_mm"]
        moved = math.dist(before, after)
        held = moved >= HELD_MM
        print(f"      {prop} moved {moved:6.1f} mm with the tool -> "
              f"{'HELD' if held else 'LEFT BEHIND'}\n")
        verdicts.append((task, bool(said.get("gripped")), held, moved))

        await grips[grip].open()
        await asyncio.sleep(1.0)
        await arms[arm].move_to_joint_positions(
            JointPositions(values=arm_home(arm)), timeout=180)
        for name, prop_name in (("Flag", "flag"), ("Pin", "pin")):
            if prop_name == prop:
                await restock_all(world, cell, parts=((name, prop_name),))

    print("  " + "-" * 58)
    for task, said, held, moved in verdicts:
        print(f"  {task:<12} gripper says held: {str(said):<5}   "
              f"part followed: {str(held):<5} ({moved:.1f} mm)")
    bad = [t for t, said, held, _ in verdicts if not (said and held)]
    print(f"\n  {len(verdicts) - len(bad)} of {len(verdicts)} picks actually hold")
    await robot.close()
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
