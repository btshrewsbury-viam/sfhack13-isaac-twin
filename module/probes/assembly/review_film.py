"""Turn a cycle film into something that can actually be reviewed.

Watching a two-minute film back to judge twelve stations is the slow way to miss things,
and it has already cost this cell twice: the overview camera was aimed at one END of the
process for the whole of its life, and nobody noticed that the work happened in the
right-hand two thirds of every frame until a frame was looked at on its own. A film is a
deliverable here, so reviewing it should be a repeatable operation rather than an act of
attention.

So: one frame per station, cropped to where the work is, laid out as a contact sheet with
the station's own caption under it. What this is for is the question "does each station
LOOK like what its caption claims" - which is a different question from "did the numbers
pass", and the only one that catches a gripper closing on air next to a part, or a door
that is shut when it should be flat.

The captions are burned into the film by `film_cycle` along the bottom edge, so the frames
are found by reading that strip rather than by guessing at timings: a station that took
40 s and one that took 4 both get exactly one frame.
"""
from __future__ import annotations

import argparse
import subprocess
from pathlib import Path
from typing import List, Tuple

OUT = Path(__file__).resolve().parents[2] / ".scratch" / "runs"
# How tall the caption strip is in the filmed frame, from `film_cycle`'s own overlay.
CAPTION_PX = 28
# How different two caption strips have to be to count as a new station.
#
# Measured rather than guessed: 6.0 found 5 stations in a film that ran 12. A caption is a
# line of small white text on a black bar, so even a completely different caption changes
# only a few per cent of the strip's pixels and the MEAN difference stays low. 1.0 finds
# every station in that same film; the strip is otherwise perfectly static, so there is no
# noise floor to clear.
CHANGE = 1.0
# How many consecutive samples a caption has to hold to count as a station.
MIN_SAMPLES = 3


def frames(path: Path, every: float = 0.5) -> List[Tuple[float, "object"]]:
    """Every `every` seconds of the film, as PIL images with their timestamps."""
    import io

    from PIL import Image

    got = subprocess.run(
        ["ffmpeg", "-loglevel", "error", "-i", str(path),
         "-vf", f"fps=1/{every}", "-f", "image2pipe", "-vcodec", "mjpeg", "-"],
        capture_output=True, timeout=900)
    if got.returncode != 0:
        raise RuntimeError(f"ffmpeg could not read {path.name}: "
                           f"{got.stderr.decode()[-300:]}")
    blob, out, start = got.stdout, [], 0
    # MJPEG in a pipe is just concatenated JPEGs: split on the SOI marker.
    while True:
        nxt = blob.find(b"\xff\xd8\xff", start + 2)
        chunk = blob[start:] if nxt < 0 else blob[start:nxt]
        if chunk:
            out.append((len(out) * every, Image.open(io.BytesIO(chunk)).convert("RGB")))
        if nxt < 0:
            break
        start = nxt
    return out


def stations(got, every: float = 0.5) -> List[Tuple[float, "object", float]]:
    """One frame per caption, taken from the MIDDLE of each caption's run.

    The middle rather than the first frame, because the first is the moment the caption
    changed - which is the arm still arriving from the previous station, and a picture of
    nothing happening.
    """
    import numpy as np

    runs_every = [every]
    runs, last = [], None
    for at, im in got:
        strip = np.asarray(im.crop((0, im.height - CAPTION_PX, im.width, im.height))
                           .convert("L"), dtype=np.float32)
        if last is None or float(np.abs(strip - last).mean()) > CHANGE:
            runs.append([])
            last = strip
        runs[-1].append((at, im))
    # A run of one or two samples is not a station, it is the caption crossfading or the
    # frame changing under a static caption. A station lasts seconds.
    keep = [run for run in runs if len(run) >= MIN_SAMPLES]
    # The middle frame of each run, and how long that caption held - which is how long the
    # station took. A film that is over budget is over budget somewhere specific, and this
    # is the cheapest way to find out where without instrumenting the cycle again.
    return [(run[len(run) // 2][0], run[len(run) // 2][1],
             len(run) * (runs_every[0] if runs_every else 0.5)) for run in keep]


def sheet(picked, path: Path, *, cols: int = 3, width: int = 620) -> Path:
    from PIL import Image

    tiles = []
    for at, im, _held in picked:
        scale = width / im.width
        tiles.append(im.resize((width, int(im.height * scale)), Image.LANCZOS))
    if not tiles:
        raise RuntimeError("no frames to lay out")
    tw, th = tiles[0].size
    rows = (len(tiles) + cols - 1) // cols
    out = Image.new("RGB", (cols * tw + (cols + 1) * 8, rows * th + (rows + 1) * 8),
                    (17, 17, 19))
    for i, tile in enumerate(tiles):
        r, c = divmod(i, cols)
        out.paste(tile, (8 + c * (tw + 8), 8 + r * (th + 8)))
    out.save(path, quality=92)
    return path


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("film", nargs="?",
                    default=str(OUT / "assembly-teach.mp4"))
    ap.add_argument("--every", type=float, default=0.5)
    args = ap.parse_args()

    film = Path(args.film)
    if not film.exists():
        print(f"  {film} does not exist - nothing to review")
        return 1
    got = frames(film, args.every)
    picked = stations(got, args.every)
    print(f"  {film.name}: {len(got)} samples, {len(picked)} stations", flush=True)
    for at, _im, held in sorted(picked, key=lambda r: -r[2]):
        print(f"    {held:5.1f}s  at {at:6.1f}s", flush=True)
    where = sheet(picked, film.with_suffix(".review.jpg"))
    print(f"  wrote {where}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
