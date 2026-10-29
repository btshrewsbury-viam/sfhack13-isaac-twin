"""Drop the end stops and corner posts from a part that already rides in a channel.

Held ready rather than applied, because it is only needed if the planner refuses the pin's
new coaxial pick - and a change to the tray that is not needed is a change that invalidates
every taught pose for nothing.

The case for it stands on its own, though:

* The pin's END STOPS are two walls 14 mm apart in x, placed from the pin's own x extent
  (+/-4 mm). Their job is to stop the belt - which pushes along x - sliding the part along
  the tray. The pin already has that: `rest_blocks` builds a CHANNEL on its rests, two
  rails either side of the Ø4 shaft, and a channel constrains x more closely than a wall
  14 mm away does. The stops are redundant with the rails for this part, and only this
  part, because it is the only one that rides in a channel.
* The pin's CORNER POSTS constrain nothing at all. They share the stops' own x, so they sit
  wholly outboard of the part's ends and buried inside the stop walls. That is true of the
  flag's as well, and is recorded beside them.

What they do do is sit 7.0 mm from the tool axis of a pick that comes in along +Y, which is
inside the 53 mm envelope the planner is given for the Hand-E. Measured by sweeping the
capsule down the approach against every pallet box: `stop_pin_0`, `stop_pin_1`,
`corner_pin_0_p` and `corner_pin_1_p` at 7.0 mm, the carrier itself at 32.0, and the pin's
own rests and rails at 38. The last three are unavoidable for any tool reaching a part
32 mm above a tray; the first four are not, because they do nothing.
"""
from __future__ import annotations

import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parents[1]

OLD = """    for name in sorted(CARRIER_SLOT_MM):
        dx, y = CARRIER_SLOT_MM[name]
        low, high = part(name).envelope_mm
        lift = CARRIER_REST_MM.get(name, 0.0)"""

NEW = """    for name in sorted(CARRIER_SLOT_MM):
        if name in CARRIER_REST_MM:
            # Already in a channel, and the channel does this job better.
            #
            # `rest_blocks` builds two rails either side of this part on its rests, which
            # constrain the axis the belt pushes along more closely than a wall placed from
            # the part's own extent plus a 3 mm gap. Keeping both only adds geometry - and
            # the geometry it adds sits 7.0 mm from the tool axis of a pick that comes in
            # along the part's own length, inside the 53 mm envelope the planner is given
            # for the Hand-E. Measured by sweeping that capsule down the approach against
            # every box on the pallet.
            continue
        dx, y = CARRIER_SLOT_MM[name]
        low, high = part(name).envelope_mm
        lift = CARRIER_REST_MM.get(name, 0.0)"""


def main() -> int:
    p = REPO / "probes" / "assembly" / "fixture.py"
    s = p.read_text()
    if OLD not in s:
        print("MISS: end_stops loop head")
        return 1
    p.write_text(s.replace(OLD, NEW, 1))
    print("fixture.py: parts that ride in a channel no longer get end stops")
    print("NOTE: regenerate the fragment and re-run the suite; "
          "tests/test_assembly_fragment.py may assert the pin's stops exist.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
