"""Put the projected flag-hole box on a real frame and look at it.

The one check that matters before generating a labelled set from arithmetic: if the
projection is off, every label is off by the same amount, consistently, and a detector
trained on it is confidently wrong forever. A picture settles it in one look.

The hole's offset is ROTATED by the shell's own orientation before it is added. That is not
pedantry - adding an unrotated offset to a part's origin is a mistake this build has already
made once, and it cost 151 mm and a day of chasing a grip that was never slipping.
"""
from __future__ import annotations

import asyncio
import io
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from assembly.arms import clear_and_park                          # noqa: E402
from assembly.asset import part                                   # noqa: E402
from assembly.build_fragment import chosen                        # noqa: E402
from assembly.film_cycle import clear_of, taught                  # noqa: E402
from assembly.fixture import index_to                             # noqa: E402
from assembly.project import box_for, draw, project               # noqa: E402
from assembly.reach_check import declared, layout_hash, orientation_for  # noqa: E402
from PIL import Image                                             # noqa: E402
from viam.components.arm import Arm                               # noqa: E402
from viam.components.camera import Camera                         # noqa: E402
from viam.components.generic import Generic                       # noqa: E402
from viam.proto.common import Pose, PoseInFrame                   # noqa: E402
from viam.robot.client import RobotClient                         # noqa: E402
from viam.services.motion import MotionClient                     # noqa: E402

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
CAM = "realsense-cam"
CAM_PRIM = "/World/arm_b/wrist_3_link/flange/realsense_cam"
FOV, W, H = 69.0, 1280, 720
HOLE_RADIUS_MM = 9.0          # the grommet's visible ring, not the 4 mm bore


def in_world(origin, axes, offset):
    """A part-frame offset placed in the world, rotated by the part's own orientation."""
    return [origin[i] + sum(axes[k][i] * offset[k] for k in range(3)) for i in range(3)]


async def main() -> int:
    out = Path(sys.argv[1] if len(sys.argv) > 1
               else ".scratch/runs/hole-projection.png").resolve()
    robot = await RobotClient.at_address(
        FQDN, RobotClient.Options.with_api_key(
            api_key=os.environ["VIAM_API_KEY"],
            api_key_id=os.environ["VIAM_API_KEY_ID"]))
    world = Generic.from_robot(robot, "sim-world")
    motion = MotionClient.from_robot(robot, "builtin")
    arms = {n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b", "arm-c")}
    cell, stamp = chosen(), layout_hash()

    await clear_and_park(world, arms, cell)
    await index_to(world, cell, cell.index_x("flag"))
    state = await declared(world, moving="arm-b")
    for step in (clear_of("arm-b", "flag_hole_look", stamp),
                 taught("arm-b", "flag_hole_look", stamp)):
        f, a = step["flange_mm"], step["approach"]
        await motion.move(component_name="arm-b",
                          destination=PoseInFrame(reference_frame="world", pose=Pose(
                              x=f[0], y=f[1], z=f[2], **orientation_for(a))),
                          world_state=state)
    await asyncio.sleep(1.5)

    prims = (await world.do_command({"command": "prim_poses",
                                     "paths": ["/World/shell", CAM_PRIM]}))["prims"]
    shell, cam = prims["/World/shell"], prims[CAM_PRIM]
    axes = [shell["x_axis"], shell["y_axis"], shell["z_axis"]]
    hole = in_world(shell["position_mm"], axes,
                    part("Shell").mm_at("assembly:flagHole:entry"))

    cam_kw = dict(cam_pos_mm=cam["position_mm"], cam_x=cam["x_axis"],
                  cam_y=cam["y_axis"], cam_z=cam["z_axis"],
                  fov_deg=FOV, width=W, height=H)
    got = project(hole, **cam_kw)
    box = box_for(hole, HOLE_RADIUS_MM, **cam_kw)
    print(f"  shell at {[round(c,1) for c in shell['position_mm']]}")
    print(f"  hole   at {[round(c,1) for c in hole]}")
    print(f"  lens   at {[round(c,1) for c in cam['position_mm']]}")
    if got is None:
        print("  the hole is behind the camera")
        await robot.close()
        return 1
    print(f"  projects to pixel {got[0]:.1f},{got[1]:.1f}  range {got[2]:.1f} mm")
    print(f"  box {[round(v,4) for v in box]}  "
          f"({round((box[2]-box[0])*W)} x {round((box[3]-box[1])*H)} px)")

    images, _ = await Camera.from_robot(robot, CAM).get_images()
    frame = Image.open(io.BytesIO(images[0].data)).convert("RGB")
    draw(frame, box, f"flag_hole  {got[2]:.0f} mm")
    out.parent.mkdir(parents=True, exist_ok=True)
    frame.save(out)
    print(f"  wrote {out}")
    await robot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
