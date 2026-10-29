"""Wire the camera into the pin insert, in one atomic pass.

Held back from the live loop until the cycle was complete, because it changes the layout
fingerprint and so costs a full re-teach, and because a vision loop on top of a cycle that
does not run is two faults to debug at once.

Three edits, and each one is a thing the course needs to be able to claim:

1. `stations.py` gains `pin_bore_look` - the RealSense standing off along the BORE's own
   published axis, exactly as `flag_hole_look` stands off along the flag hole's. The cell
   has pointed a camera at a hole and done nothing with what it saw for its whole life;
   this is the station that acts on it.
2. `film_cycle.py` runs that station before the pin pick, measures the camera's pixel
   response the first time it sees a layout, and stores it beside the taught poses under
   the same fingerprint. At cycle time it is one frame and one short planned move.
3. The pin insert applies that lateral correction on top of the frame correction it
   already makes.

The look happens BEFORE the pick, with an empty gripper, deliberately. At the pre-insert
pose the camera is already aimed near the bore - the station's work point is the pin's
seated position - but the pin itself is then on the tool axis between the lens and the
hole, 110 mm away and pointing at it, and a detector looking for the darkest round thing
near the centre of that frame has an excellent chance of finding the end of the pin.
Nothing moves the shell between the look and the insert: the belt does not index, and both
stations sit at the same index by construction.
"""
from __future__ import annotations

import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parents[1]

DROP_CONST = '''# How far below the bore's axis the camera looks from, as an angle. See the station.\nBORE_LOOK_DROP_DEG = 11.0\n\n\n'''

STATION = '''    ix = cell.index_x("pin")
    # 5a - THE BORE, LOOKED AT. The RealSense stands off along the bore's own published
    #      axis, which is the same arrangement `flag_hole_look` uses on the flag hole.
    #
    #      Before the pick, with an empty gripper. At the pre-insert pose the camera is
    #      already aimed near the bore, but the pin is then on the tool axis between the
    #      lens and the hole and pointing straight at it - and a detector that wants the
    #      darkest round thing near the middle of the frame would have every chance of
    #      finding the pin's own end face. Nothing moves the shell between here and the
    #      insert: the belt does not index, and both stations sit at this same index.
    #      And DROPPED below the bore's axis, looking slightly up at it. Not a framing
    #      preference: a leaf plate sits directly on top of the knuckle - shell frame z
    #      -0.0026 to -0.0002, with the shell floor 0.2 mm above that - so a camera ON the
    #      axis or above it is looking at the plate's edge rather than into the hole. From
    #      below, the bore's mouth is a shadowed disk inside a bright zinc ring with
    #      nothing in front of it, which is exactly what the detector is looking for.
    #
    #      11 degrees, which is the angle `bore_look.py` arrived at (40 mm of drop at a
    #      200 mm stand-off) expressed as an angle rather than an offset, so it holds at
    #      the RealSense's own 320 mm. Shallow enough that the bore does not foreshorten
    #      into an ellipse the centroid would bias.
    bore = shell.mm_at("assembly:pinBore:entry")
    bore_axis = _unit(shell.at("assembly:pinBore:axis"))
    out_of = _unit([-c for c in bore_axis])        # out of the hole, towards the camera
    dropped = _unit([out_of[0], out_of[1],
                     out_of[2] - math.tan(math.radians(BORE_LOOK_DROP_DEG))])
    out.append(Station("assemble", "pin_bore_look", at(ix)(bore),
                       dropped, "realsense", "assembly:pinBore:entry/axis", ix))
'''


FILM_OLD = """        "name": "overview-cam", "namespace": "rdk", "type": "camera",
        "model": "erh:isaac-sim:camera",
        "frame": {"parent": "world", "translation": {"x": 2600, "y": -2400, "z": 2000}},
        "attributes": {"world": "sim-world",
                       "target": _m((cell.index_x("inside"), 0.0,
                                     cell.belt_top_mm + 150.0)),
                       "fov_deg": 52.0, "width": 1280, "height": 720},"""

FILM_NEW = """        "name": "overview-cam", "namespace": "rdk", "type": "camera",
        "model": "erh:isaac-sim:camera",
        # ON THE SIDE THE WORK HAPPENS, and aimed at the middle of it.
        #
        # Both faults were found by watching the film back rather than by reading the
        # config, which is the whole argument for `probes/assembly/review_film.py`.
        #
        # The camera stood at y -2400. arm-b is at y +800 and the carrier's two slots are
        # at y +220 and +290 - so the assemble arm and both parts it fits were BEHIND the
        # mailbox in every frame. arm-b does five of the twelve stations, including both
        # picks and both inserts, and not one of them was visible. A film of an assembly
        # cell that does not show the assembly is not a film of the cell.
        #
        # And it was aimed at x -55, the FIRST index, while the stations run to x 901 - so
        # the cycle happened in the right-hand two thirds of the frame with the left third
        # on empty floor, at a 52 degree field 3.5 m out: a 3.4 m wide shot of a 1 m
        # process.
        #
        # Moved to the assemble side and lifted, aimed at the mid-index and narrowed to
        # 38 degrees, the frame is about 2.6 m across - still the whole 956 mm of belt
        # travel and all three arms, with the work roughly twice the size and on the near
        # side of it. arm-c's handling is then the far side, which is the right trade:
        # it is four stations of carrying, against five of the assembly the course is about.
        "frame": {"parent": "world", "translation": {"x": 2600, "y": 2600, "z": 2600}},
        "attributes": {"world": "sim-world",
                       "target": _m(((cell.index_x("inside")
                                      + cell.index_x("handle")) / 2.0, 0.0,
                                     cell.belt_top_mm + 150.0)),
                       "fov_deg": 38.0, "width": 1280, "height": 720},"""


def film_camera() -> int:
    gen = REPO / "probes" / "assembly" / "build_fragment.py"
    s = gen.read_text()
    if FILM_OLD not in s:
        print("MISS: overview-cam block")
        return 1
    gen.write_text(s.replace(FILM_OLD, FILM_NEW, 1))
    print("build_fragment.py: film camera recentred and narrowed")
    return 0


def main() -> int:
    st = REPO / "probes" / "assembly" / "stations.py"
    s = st.read_text()
    anchor = '''    ix = cell.index_x("pin")
    # 6 - the pin: taken off the tray it arrived on, and driven -Y through the bore.'''
    if anchor not in s:
        print("MISS: stations.py pin block")
        return 1
    s = s.replace(anchor, STATION + anchor.split("\n", 1)[1], 1)
    head = "def stations_for(cell: Cell) -> List[Station]:"
    if head not in s:
        print("MISS: stations_for")
        return 1
    s = s.replace(head, DROP_CONST + head, 1)
    st.write_text(s)
    print("stations.py: pin_bore_look added")
    return film_camera()


if __name__ == "__main__":
    sys.exit(main())
