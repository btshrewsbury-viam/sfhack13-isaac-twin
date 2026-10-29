"""Look at the hinge bore with the RealSense, and say where it is in the image.

The cell's pin insert has been trusting the carrier to present the pin, and the carrier
cannot: the bore's published slack is 0.20 mm and the tray holds a part to a couple of
millimetres, so the pin jams and a position-controlled arm pushing a jammed pin throws
the shell out of the fixture. Brandon's call is to find the hole with the camera and
correct for whatever misalignment is left, which is also the thing a course about
computer vision ought to be demonstrating - the cell currently points a RealSense at a
flag hole and then does nothing with what it saw.

This is the first half: get the bore in frame, and find it. It writes the frame and the
detection overlay so the result can be LOOKED AT rather than trusted, which is the only
way the projection work in this cell has ever been got right.
"""
from __future__ import annotations

import asyncio
import io
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from assembly.asset import part                                   # noqa: E402
from assembly.arms import clear_and_park, home as arm_home        # noqa: E402
from assembly.build_fragment import chosen                        # noqa: E402
from assembly.creds import options, FQDN                          # noqa: E402
from assembly.fixture import index_to, restock_all                # noqa: E402
from assembly.reach_check import layout_hash, orientation_for     # noqa: E402
from PIL import Image, ImageDraw                                  # noqa: E402
from viam.components.arm import Arm, JointPositions               # noqa: E402
from viam.components.camera import Camera                         # noqa: E402
from viam.components.generic import Generic                       # noqa: E402
from viam.proto.common import Pose, PoseInFrame                   # noqa: E402
from viam.robot.client import RobotClient                         # noqa: E402
from viam.services.motion import MotionClient                     # noqa: E402

OUT = Path(__file__).resolve().parents[2] / ".scratch" / "frames" / "bore"
# How far back along the bore's own axis the lens sits. Far enough that the knuckle is
# wholly in frame, near enough that the bore is more than a handful of pixels: at 69 deg
# over 1280 px, 200 mm puts a millimetre at about 4.6 px.
LOOK_MM = 200.0
# And DROPPED below the bore's axis, looking slightly up at it.
#
# Not a framing preference. A leaf plate sits directly on top of the knuckle (shell frame
# z -0.0026..-0.0002) with the shell floor 0.2 mm above that, so a camera on the axis or
# above it is looking at the plate's edge. From below, the bore's mouth is a shadowed
# disk inside a bright zinc ring with nothing in front of it. 40 mm at 200 gives about
# 11 degrees, which clears the plate without foreshortening the bore into an ellipse the
# centroid would bias.
DROP_MM = 40.0


async def bore_in_world(world) -> tuple[list[float], list[float]]:
    """Where the shell's hinge bore is NOW, and which way it faces.

    Read from the shell's live pose rather than from the index, and ROTATED by the
    shell's own orientation before being added - adding an unrotated offset to a part's
    origin is a mistake this build has made before and it cost a day.
    """
    shell = part("Shell")
    entry = shell.mm_at("assembly:pinBore:entry")
    got = (await world.do_command({"command": "prim_poses",
                                   "paths": ["/World/shell"]}))["prims"]["/World/shell"]
    at = got["position_mm"]
    axes = [got["x_axis"], got["y_axis"], got["z_axis"]]
    here = [at[i] + sum(axes[k][i] * entry[k] for k in range(3)) for i in range(3)]
    # The pin goes in along -Y, so the bore faces +Y and the camera stands off that way.
    facing = [sum(axes[k][i] * v for k, v in enumerate((0.0, 1.0, 0.0)))
              for i in range(3)]
    return here, facing


def find_bore(frame: Image.Image, expect_px: tuple[float, float] | None = None):
    """The darkest round thing near where the bore should be.

    A bore is a hole: whatever is behind it is unlit and further away, so in a rendered
    frame it is the local dark blob. That is a weaker cue than a trained detector and a
    far more honest one to teach from - it needs no dataset, it is three lines of
    arithmetic, and when it fails it fails visibly rather than confidently.
    """
    import numpy as np

    grey = np.asarray(frame.convert("L"), dtype=np.float32)
    h, w = grey.shape
    # Search a window around where the geometry says it should be, so a dark corner of
    # the frame cannot win.
    cx, cy = expect_px if expect_px else (w / 2.0, h / 2.0)
    half = int(min(w, h) * 0.25)
    x0, x1 = max(0, int(cx) - half), min(w, int(cx) + half)
    y0, y1 = max(0, int(cy) - half), min(h, int(cy) + half)
    win = grey[y0:y1, x0:x1]
    if win.size == 0:
        return None
    # Threshold well below the window's own median, so it adapts to exposure.
    thresh = float(np.median(win)) * 0.45
    dark = win < thresh
    if dark.sum() < 20:
        return None
    ys, xs = np.nonzero(dark)
    # Centroid of the darkest connected mass, approximated by the dark pixels' mean -
    # good enough for a round hole on a flat face, and it degrades gracefully.
    return (float(xs.mean()) + x0, float(ys.mean()) + y0, int(dark.sum()))


async def main() -> int:
    robot = await RobotClient.at_address(FQDN, options())
    world = Generic.from_robot(robot, "sim-world")
    arms = {n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b", "arm-c")}
    cam = Camera.from_robot(robot, "realsense-cam")
    motion = MotionClient.from_robot(robot, "builtin")
    cell = chosen()
    print(f"  layout {layout_hash()}", flush=True)

    for which in ("flag", "door", "pin"):
        for j in (f"{which}_seat", f"{which}_clip"):
            await world.do_command({"command": "unjoin", "name": j})
    await world.do_command({"command": "reset"})
    await asyncio.sleep(3.0)
    await clear_and_park(world, arms, cell)
    await restock_all(world, cell)
    await index_to(world, cell, cell.index_x("pin"))
    await asyncio.sleep(1.0)

    here, facing = await bore_in_world(world)
    print(f"  bore at {['%.1f' % v for v in here]} facing "
          f"{['%+.2f' % v for v in facing]}", flush=True)

    # The lens goes LOOK_MM back along the bore's axis. `realsense` carries its lens
    # forward of the flange, so the station's own standoff arithmetic is reused rather
    # than re-derived here.
    from assembly.stations import TOOLS
    tool = TOOLS["realsense"]
    # Stand off along the bore axis, dropped, and look back at the bore.
    eye = [here[i] + facing[i] * LOOK_MM for i in range(3)]
    eye[2] -= DROP_MM
    look = [here[i] - eye[i] for i in range(3)]
    norm = math.sqrt(sum(v * v for v in look)) or 1.0
    look = [v / norm for v in look]
    approach = [-v for v in look]          # the flange stands off against the view
    reach = tool.standoff_mm - tool.lens_mm
    flange = [here[i] + approach[i] * (LOOK_MM + reach) for i in range(3)]
    print(f"  looking from {['%.1f' % v for v in eye]} along "
          f"{['%+.2f' % v for v in look]}", flush=True)
    try:
        await motion.move(component_name="arm-b",
                          destination=PoseInFrame(reference_frame="world", pose=Pose(
                              x=flange[0], y=flange[1], z=flange[2],
                              **orientation_for(approach))))
    except Exception as exc:                                       # noqa: BLE001
        print(f"  could not get the camera there: {str(exc)[-200:]}")
        await robot.close()
        return 1
    await asyncio.sleep(1.2)

    OUT.parent.mkdir(parents=True, exist_ok=True)
    images, _ = await cam.get_images()
    frame = Image.open(io.BytesIO(images[0].data)).convert("RGB")
    frame.save(f"{OUT}.raw.jpg", quality=95)
    found = find_bore(frame)
    draw = ImageDraw.Draw(frame)
    draw.line([(frame.width / 2 - 20, frame.height / 2),
               (frame.width / 2 + 20, frame.height / 2)], fill=(80, 160, 255), width=2)
    draw.line([(frame.width / 2, frame.height / 2 - 20),
               (frame.width / 2, frame.height / 2 + 20)], fill=(80, 160, 255), width=2)
    if found:
        x, y, n = found
        draw.ellipse([x - 26, y - 26, x + 26, y + 26], outline=(255, 70, 70), width=3)
        print(f"  bore found at pixel ({x:.1f}, {y:.1f}) from {n} dark px; "
              f"frame centre is ({frame.width/2:.0f}, {frame.height/2:.0f})", flush=True)
    else:
        print("  NO BORE FOUND - look at the raw frame", flush=True)
    frame.save(f"{OUT}.seen.jpg", quality=95)
    print(f"  wrote {OUT}.raw.jpg and {OUT}.seen.jpg", flush=True)

    await arms["arm-b"].move_to_joint_positions(
        JointPositions(values=arm_home("arm-b")), timeout=180)
    await robot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
