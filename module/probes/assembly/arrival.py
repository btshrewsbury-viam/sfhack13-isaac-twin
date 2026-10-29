"""How repeatably does the belt deliver the part? The number every insertion depends on.

Measured in two frames, because "the part is 4 mm out" has two completely different causes
and only one of them is the fixture's fault:

  shell vs NOMINAL   what an open-loop insertion would be wrong by
  shell vs PALLET    whether the part is creeping in its cradle, or the cradle on the belt

The flag tolerates 1.00 mm of radial error and the pin 0.20 mm, so this measurement decides
whether insertions can be taught once or have to be corrected from a camera every cycle.
"""
import asyncio, os, statistics as st, sys
sys.path.insert(0, "probes")
from assembly.asset import insertions, part
from assembly.build_fragment import chosen
from assembly.fixture import index_to, indexed_mm
from viam.robot.client import RobotClient
from viam.components.generic import Generic
FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
async def main():
    m = await RobotClient.at_address(FQDN, RobotClient.Options.with_api_key(
        api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]))
    w = Generic.from_robot(m, "sim-world"); cell = chosen()
    want = cell.index_x("flag")
    pal_base = indexed_mm(cell, "pallet", 0.0)[0]
    shell_in_pallet = cell.shell_origin_mm[0] - indexed_mm(cell, "pallet", 0.0)[0] \
        - (cell.index_x("inside") - cell.index_x("inside"))
    xs, ys, rel_x, rel_y = [], [], [], []
    for i in range(int(os.environ.get("ARRIVALS", "8"))):
        await index_to(w, cell, cell.index_x("inside"))
        await index_to(w, cell, want)
        r = (await w.do_command({"command": "prim_poses",
                                 "paths": ["/World/shell", "/World/pallet"]}))["prims"]
        p, q = r["/World/shell"]["position_mm"], r["/World/pallet"]["position_mm"]
        xs.append(p[0] - want); ys.append(p[1])
        rel_x.append(p[0] - q[0]); rel_y.append(p[1] - q[1])
        print(f"  arrival {i+1}: shell {p[0]-want:+7.2f},{p[1]:+6.2f} from nominal   "
              f"in the pallet {p[0]-q[0]:+7.2f},{p[1]-q[1]:+6.2f}")

    def spread(name, vals):
        print(f"  {name}: mean {st.mean(vals):+6.2f}  sd {st.pstdev(vals):5.2f}  "
              f"range {min(vals):+6.2f}..{max(vals):+6.2f}  "
              f"spread {max(vals)-min(vals):5.2f} mm")
    print()
    spread("shell x vs nominal ", xs)
    spread("shell y vs nominal ", ys)
    spread("shell x in pallet  ", rel_x)
    spread("shell y in pallet  ", rel_y)
    ins = insertions()
    for n in ("Flag", "Pin"):
        print(f"  {n} radial slack {ins[n].clearance_mm:.2f} mm")
    await m.close()
asyncio.run(main())
