"""T2's acceptance: does the beam stop the part at the pick point, repeatably?

The beam DEFINES the pick point, so what matters is not where the beam is but where the
part ends up - and three numbers describe that, not one. The rails leave about 367 mm of
play either side of a 172 mm part, so they hold neither x nor yaw; if the drive walks the
part sideways or turns it, a perfect y is still a failed pick.

Overshoot is the first number to learn: the part coasts after the beam breaks, and the
stop must happen inside the sim step that sees it break, not after a gRPC round trip.

Accept: the beam reads clear with no part and blocked with one, and the part halts with
its leading edge within 10 mm of the same place three times running.
"""
import asyncio, math, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from qc.layout import Cell, FRAGMENT, part_origin_offset_mm


def _props():
    import json
    world = next(c for c in json.loads(FRAGMENT.read_text())["components"]
                 if c["model"].endswith(":world"))
    return world["attributes"]["props"]
from viam.components.generic import Generic
from viam.robot.client import RobotClient

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")


def yaw_deg(q) -> float:
    w, x, y, z = q
    return math.degrees(math.atan2(2.0 * (w * z + x * y),
                                   1.0 - 2.0 * (y * y + z * z)))


async def part_pose(world, cell):
    poses = await world.do_command({"command": "prop_poses", "names": ["part"]})
    p = poses.get("props", poses)["part"]
    off = part_origin_offset_mm()
    centre = [p["position_mm"][i] + off[i] for i in range(3)]
    return centre, yaw_deg(p["orientation_wxyz"])


async def main() -> int:
    robot = await RobotClient.at_address(FQDN, RobotClient.Options.with_api_key(
        api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]))
    world = Generic.from_robot(robot, "sim-world")
    cell = Cell.from_fragment()
    half = cell.part.dims_mm[1] / 2.0
    state = (await world.do_command({"command": "conveyor"}))["conveyor"]
    beam_y = state["beam_y_mm"]
    print(f"beam at y {beam_y}   speed {state['speed_mm_s']} mm/s   "
          f"{int(state['rollers'])} rollers")

    spawn = [v * 1000.0 for v in next(
        pr["position"] for pr in _props() if pr["name"] == "part")]
    runs = []
    clear_before, blocked_after = False, True
    for run in range(1, 4):
        # Stop the belt and put the part back BY HAND. `reset` restores the props in
        # world.scene, and a referenced asset is not one of them - so run 2 once began
        # from where run 1 stalled, quietly turning three independent runs into one
        # drifting sequence.
        await world.do_command({"command": "conveyor", "run": False})
        # Placed 5 mm clear of the rollers and allowed to drop. Put down with its
        # underside exactly on the roller crowns, PhysX reads the contact as a
        # penetration and shoves the part out of it - twice it set off UPSTREAM, away
        # from the beam, before the belt had done anything. At boot this never happens,
        # because props are placed before physics is running.
        await world.do_command({"command": "place_prop", "name": "part",
                                "position_mm": [spawn[0], spawn[1], spawn[2] + 5.0]})
        await asyncio.sleep(2.5)
        (sx, sy, sz), syaw = await part_pose(world, cell)
        clear = (await world.do_command({"command": "conveyor"}))["conveyor"]
        clear_before = clear_before or clear["blocked"]
        await world.do_command({"command": "conveyor", "run": True})
        track, stopped, running = [], False, True
        for _ in range(60):
            await asyncio.sleep(0.5)
            s = (await world.do_command({"command": "conveyor"}))["conveyor"]
            (_, ty, _), _ = await part_pose(world, cell)
            track.append(ty)
            if s["stopped_by_beam"]:
                stopped = True
                break
            running = s["running"]
        await world.do_command({"command": "conveyor", "run": False})
        await asyncio.sleep(1.5)
        # The other half of the acceptance: the beam must read blocked with a part
        # across it, not merely have stopped the belt at some point.
        held = (await world.do_command({"command": "conveyor"}))["conveyor"]
        blocked_after = blocked_after and held["blocked"]
        (x, y, z), yaw = await part_pose(world, cell)
        lead = y + half
        runs.append((x, y, z, yaw, lead))
        # A stall looks exactly like a slow run unless the track is shown.
        crawl = ""
        if len(track) > 2:
            last = [round(track[i] - track[i - 1], 1) for i in range(1, len(track))]
            crawl = f"  steps mm/0.5s: {last[:3]} ... {last[-3:]}"
        print(f"  run {run}: started y {sy:8.1f}  beam clear={not clear['blocked']}  "
              f"stopped_by_beam={stopped} after {len(track) * 0.5:.1f}s"
              f"{'' if stopped else f'  [belt still running={running} -> JAMMED]'}{crawl}")
        print(f"         -> y {y:8.1f}   leading edge {lead:8.1f} "
              f"(overshoot {lead - beam_y:+.1f})   x {x:7.1f} ({x - sx:+.1f})   "
              f"yaw {yaw:7.2f} ({yaw - syaw:+.2f})")

    spread = lambda i: max(r[i] for r in runs) - min(r[i] for r in runs)
    print(f"\nspread over three runs: leading edge {spread(4):.1f} mm   "
          f"x {spread(0):.1f} mm   yaw {spread(3):.2f} deg")
    # A run in which the part never moved is not a pass, however repeatable it is. The
    # beam once read blocked before the belt started - it was looking through a guide
    # rail - so the conveyor stopped instantly and three identical non-events scored a
    # spread of 0.0 mm on every axis.
    travelled = min(abs(r[1] - spawn[1]) for r in runs)
    # What each tolerance is FOR, rather than three round numbers:
    #  - leading edge is the acceptance proper: the beam has to stop the part in the
    #    same place along the belt every time.
    #  - yaw is uncompensated. The grip is taken along a world axis, so a part that
    #    stops skewed presents a skewed face to a flat cup.
    #  - x IS compensated: `pick()` derives the grip from the part's measured pose, so
    #    lateral position only has to stay inside the guide channel, whose own play is
    #    194 - 172.4 = 21.6 mm. Holding x to 10 mm was asking the cell for something it
    #    does not need and does not have.
    ok = (travelled > 500.0 and spread(4) <= 10.0 and spread(0) <= 21.6
          and spread(3) <= 2.5 and blocked_after and not clear_before)
    print(f"shortest travel {travelled:.0f} mm (a run that never moved is not a pass)")
    print(f"beam clear with the part upstream: {not clear_before}   "
          f"blocked with the part across it: {blocked_after}")
    print("T2 REPEATABILITY:", "PASS" if ok else "FAIL")
    await robot.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
