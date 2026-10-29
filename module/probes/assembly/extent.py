"""The shell's extent, with its own orientation accounted for.

A world-axis box of a rotated part is wider than the part. Now that the fixture holds the
shell square to 0.03 mm the two should agree, so the residual - if any - is geometry rather
than pose, which is the question worth answering.
"""
import asyncio, math, os, sys
sys.path.insert(0, "probes")
from assembly.asset import part
from assembly.build_fragment import chosen
from assembly.fixture import index_to
from viam.robot.client import RobotClient
from viam.components.generic import Generic
FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")

async def main():
    m = await RobotClient.at_address(FQDN, RobotClient.Options.with_api_key(
        api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]))
    w = Generic.from_robot(m, "sim-world"); cell = chosen()
    await index_to(w, cell, cell.index_x("inside"))
    p = (await w.do_command({"command":"prim_poses","paths":["/World/shell"]}))["prims"]["/World/shell"]
    b = (await w.do_command({"command":"prim_bounds","paths":["/World/shell"]}))["bounds"]["/World/shell"]
    o = p["position_mm"]
    lo = [v*1000 - o[i] for i, v in enumerate(b["min"])]
    hi = [v*1000 - o[i] for i, v in enumerate(b["max"])]
    ax = {k: p[k] for k in ("x_axis", "y_axis", "z_axis")}
    yaw = math.degrees(math.atan2(ax["x_axis"][1], ax["x_axis"][0]))
    roll = math.degrees(math.atan2(ax["y_axis"][2], ax["y_axis"][1]))
    pitch = math.degrees(math.atan2(-ax["x_axis"][2], math.hypot(*ax["x_axis"][:2])))
    print(f"  shell orientation: yaw {yaw:+.4f}  pitch {pitch:+.4f}  roll {roll:+.4f} deg")
    print(f"  world-axis box, about the origin:")
    print(f"      x {lo[0]:+8.2f} .. {hi[0]:+8.2f}")
    print(f"      y {lo[1]:+8.2f} .. {hi[1]:+8.2f}")
    print(f"      z {lo[2]:+8.2f} .. {hi[2]:+8.2f}")
    elo, ehi = part("Shell").envelope_mm
    print(f"  asset envelope:")
    print(f"      x {elo[0]:+8.2f} .. {ehi[0]:+8.2f}")
    print(f"      y {elo[1]:+8.2f} .. {ehi[1]:+8.2f}")
    print(f"      z {elo[2]:+8.2f} .. {ehi[2]:+8.2f}")
    # what the measured yaw alone would add to a world-axis box
    th = math.radians(abs(yaw))
    halfL, halfW = (ehi[0] - elo[0]) / 2.0, (ehi[1] - elo[1]) / 2.0
    print(f"  a {abs(yaw):.4f} deg yaw predicts half-y "
          f"{halfW*math.cos(th) + halfL*math.sin(th):.2f} mm "
          f"(flat: {halfW:.2f}); measured {max(abs(lo[1]), abs(hi[1])):.2f}")
    await m.close()
asyncio.run(main())
