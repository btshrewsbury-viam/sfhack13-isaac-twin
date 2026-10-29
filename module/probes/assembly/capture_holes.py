"""A detection dataset for the flag hole and the pin bore, labelled by projection.

Hand labelling is not just slow here, it is worse: the cell knows exactly where these
features are, so the boxes can be computed from the asset's published positions and the
camera's measured pose. Exact rather than careful, and a hundred of them as cheap as one.

What makes a set like this useful is VARIETY IN THE RIGHT VARIABLE. The detector's job is
to find the feature when the part has arrived slightly wrong, so the part is deliberately
indexed to a spread of positions - the same conveyor scatter the insertion has to survive,
only wider, so the detector is never asked to extrapolate. Scheme and finish vary too,
because paint changes what a hole looks like far more than position does.

Every label is checked before it is written: a box that falls outside the frame, or that is
smaller than a few pixels, is dropped rather than shipped. A dataset is a claim about what
is in each image, and an unverified claim repeated a hundred times is still unverified.
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
from assembly.project import box_for, draw                        # noqa: E402
from assembly.reach_check import declared, layout_hash, orientation_for  # noqa: E402
from assembly.see_hole import in_world                            # noqa: E402
from PIL import Image                                             # noqa: E402
from viam.components.arm import Arm                               # noqa: E402
from viam.components.camera import Camera                         # noqa: E402
from viam.components.generic import Generic                       # noqa: E402
from viam.proto.common import Pose, PoseInFrame                   # noqa: E402
from viam.robot.client import RobotClient                         # noqa: E402
from viam.services.motion import MotionClient                     # noqa: E402

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
OUT = Path(".scratch/datasets/holes").resolve()
CAM, CAM_PRIM = "realsense-cam", "/World/arm_b/wrist_3_link/flange/realsense_cam"
FOV, W, H = 69.0, 1280, 720

# The features this camera can see from the flag-hole station, and how big each looks.
# Radii are of the VISIBLE feature - the grommet's ring, not the bore it surrounds.
FEATURES = {"flag_hole": ("assembly:flagHole:entry", 9.0)}

# Wider than the conveyor's own scatter, deliberately. The belt delivers about 1.8 mm
# standard deviation; a detector trained only on that is extrapolating the first time a
# pallet arrives badly, and the whole point of it is the bad arrivals.
OFFSETS_MM = (-24.0, -12.0, -4.0, 0.0, 4.0, 12.0, 24.0)
MIN_BOX_PX = 12


async def main() -> int:
    robot = await RobotClient.at_address(
        FQDN, RobotClient.Options.with_api_key(
            api_key=os.environ["VIAM_API_KEY"],
            api_key_id=os.environ["VIAM_API_KEY_ID"]))
    world = Generic.from_robot(robot, "sim-world")
    motion = MotionClient.from_robot(robot, "builtin")
    arms = {n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b", "arm-c")}
    camera = Camera.from_robot(robot, CAM)
    cell, stamp = chosen(), layout_hash()
    shell = part("Shell")
    schemes = shell.variant_sets["scheme"]
    finishes = shell.variant_sets["finish"]
    base = cell.index_x("flag")

    OUT.mkdir(parents=True, exist_ok=True)
    lines, n, dropped = [], 0, 0
    for scheme in schemes:
        for finish in finishes:
            await world.do_command({"command": "set_variants", "prop": "shell",
                                    "variants": {"scheme": scheme, "finish": finish}})
            await world.do_command({"command": "reset"})
            await asyncio.sleep(1.8)
            await clear_and_park(world, arms, cell)
            at = None
            for dx in OFFSETS_MM:
                at = await index_to(world, cell, base + dx, at)
                # The arm goes to the SAME taught pose every time. The part moves, the
                # camera does not - which is exactly the situation the detector exists for.
                state = await declared(world, moving="arm-b")
                try:
                    for step in (clear_of("arm-b", "flag_hole_look", stamp),
                                 taught("arm-b", "flag_hole_look", stamp)):
                        f, a = step["flange_mm"], step["approach"]
                        await motion.move(
                            component_name="arm-b",
                            destination=PoseInFrame(reference_frame="world", pose=Pose(
                                x=f[0], y=f[1], z=f[2], **orientation_for(a))),
                            world_state=state)
                except Exception as exc:                           # noqa: BLE001
                    print(f"    {scheme}/{finish} dx{dx:+.0f}: {str(exc)[-90:]}",
                          flush=True)
                    continue
                await asyncio.sleep(1.0)

                prims = (await world.do_command({
                    "command": "prim_poses",
                    "paths": ["/World/shell", CAM_PRIM]}))["prims"]
                sh, cam = prims["/World/shell"], prims[CAM_PRIM]
                axes = [sh["x_axis"], sh["y_axis"], sh["z_axis"]]
                cam_kw = dict(cam_pos_mm=cam["position_mm"], cam_x=cam["x_axis"],
                              cam_y=cam["y_axis"], cam_z=cam["z_axis"],
                              fov_deg=FOV, width=W, height=H)

                boxes = []
                for label, (key, radius) in FEATURES.items():
                    where = in_world(sh["position_mm"], axes, shell.mm_at(key))
                    box = box_for(where, radius, **cam_kw)
                    if box is None or (box[2] - box[0]) * W < MIN_BOX_PX:
                        dropped += 1
                        continue
                    boxes.append({"annotation_label": label,
                                  "x_min_normalized": round(box[0], 6),
                                  "y_min_normalized": round(box[1], 6),
                                  "x_max_normalized": round(box[2], 6),
                                  "y_max_normalized": round(box[3], 6)})
                if not boxes:
                    continue

                images, _ = await camera.get_images()
                frame = Image.open(io.BytesIO(images[0].data)).convert("RGB")
                name = f"{scheme}_{finish}_{dx:+.0f}.jpg"
                frame.save(OUT / name, quality=92)
                lines.append({"image_path": name, "bounding_box_annotations": boxes,
                              "scheme": scheme, "finish": finish, "index_offset_mm": dx})
                n += 1
                if n % 10 == 0:
                    print(f"    {n} frames", flush=True)

    (OUT / "train.jsonl").write_text(
        "\n".join(json.dumps(line) for line in lines) + "\n")
    print(f"\n  {n} frames, {dropped} labels dropped as out of frame or too small")
    print(f"  wrote {OUT / 'train.jsonl'}")

    # A contact sheet of the labels, because a dataset nobody looks at is a dataset nobody
    # has checked. Six spread across the set, boxes drawn.
    picks = lines[:: max(1, len(lines) // 6)][:6]
    if picks:
        ims = []
        for line in picks:
            im = Image.open(OUT / line["image_path"]).convert("RGB")
            for b in line["bounding_box_annotations"]:
                draw(im, (b["x_min_normalized"], b["y_min_normalized"],
                          b["x_max_normalized"], b["y_max_normalized"]),
                     b["annotation_label"])
            ims.append((line, im.resize((440, 248))))
        sheet = Image.new("RGB", (3 * 450 + 10, 2 * 272 + 10), (18, 18, 20))
        from PIL import ImageDraw as _D
        d = _D.Draw(sheet)
        for i, (line, im) in enumerate(ims):
            x, y = 10 + (i % 3) * 450, 10 + (i // 3) * 272
            sheet.paste(im, (x, y))
            d.text((x + 3, y + 250),
                   f"{line['scheme']} {line['finish']} dx{line['index_offset_mm']:+.0f}",
                   fill=(228, 229, 235))
        sheet.save(OUT.parent / "hole-labels.png")
        print(f"  wrote {OUT.parent / 'hole-labels.png'}")
    await robot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
