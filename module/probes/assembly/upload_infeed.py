"""Upload the infeed classification set into a Viam cloud dataset.

A classification dataset in Viam is images carrying TAGS: the training script reads a tag
as a class label, so one tag per image - good, damaged, empty - is the whole annotation.
That is the difference from the detection set the arcade's `upload_dataset.py` handles,
which attaches bounding boxes after the upload.

    set -a; . /home/shrews/viam/isaac-arcade/.scratch/secrets/viam-dataset-key.env; set +a
    PYTHONPATH=probes .venv/bin/python -m assembly.upload_infeed \
        --org-id <org> --part-id <part> --dataset-name mailbox-infeed-v1

Progress is appended after each image, so a rerun skips what is already up rather than
duplicating it - an upload that cannot be resumed is an upload that gets abandoned halfway
and restarted from nothing.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import time
from datetime import datetime, timezone
from pathlib import Path

from viam.app.viam_client import ViamClient
from viam.rpc.dial import DialOptions, _dial_app

ROOT = Path(__file__).resolve().parents[2] / ".scratch" / "datasets" / "infeed"
CLASSES = ("good", "damaged", "empty")


async def connect(api_key: str, api_key_id: str) -> ViamClient:
    # viam-sdk 0.78's ViamClient.create_from_dial_options authenticates twice on one
    # channel - once inside _dial_app and again itself - and app.viam.com rejects the
    # second call with "already authenticated; cannot re-authenticate". Keep the first
    # token. (Same workaround as the arcade's upload_dataset.py; it is a client bug, not a
    # credentials problem, and it looks exactly like a credentials problem.)
    options = DialOptions.with_api_key(api_key, api_key_id)
    client = ViamClient()
    client._dial_options = options
    client._channel = await _dial_app("app.viam.com", options)
    client._metadata = client._channel._metadata
    return client


async def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--org-id", required=True)
    ap.add_argument("--part-id", required=True)
    ap.add_argument("--dataset-name", default="mailbox-infeed-v1")
    ap.add_argument("--component-name", default="overhead-cam")
    ap.add_argument("--concurrency", type=int, default=6)
    ap.add_argument("--root", default=str(ROOT))
    args = ap.parse_args()

    root = Path(args.root)
    images = [(c, p) for c in CLASSES for p in sorted((root / c).glob("*.jpg"))]
    if not images:
        raise SystemExit(f"no images under {root}")
    counts = {c: sum(1 for k, _ in images if k == c) for c in CLASSES}
    print(f"  {len(images)} images: " + ", ".join(f"{c} {n}" for c, n in counts.items()))

    progress_path = root / "uploaded.jsonl"
    done = set()
    if progress_path.exists():
        done = {json.loads(line)["image"] for line in progress_path.open()}
    todo = [(c, p) for c, p in images if str(p.relative_to(root)) not in done]

    client = await connect(os.environ["VIAM_API_KEY"], os.environ["VIAM_API_KEY_ID"])
    data = client.data_client
    try:
        existing = [d for d in await data.list_datasets_by_organization_id(args.org_id)
                    if d.name == args.dataset_name]
        dataset_id = (existing[0].id if existing
                      else await data.create_dataset(args.dataset_name, args.org_id))
        print(f"  dataset {args.dataset_name} ({dataset_id}): "
              f"{len(done)} already up, {len(todo)} to go", flush=True)

        progress = progress_path.open("a")
        sem = asyncio.Semaphore(args.concurrency)
        n = {"done": 0}
        t0 = time.monotonic()

        async def one(label: str, path: Path) -> None:
            async with sem:
                now = datetime.now(timezone.utc)   # the upload api requires capture times
                binary_id = await data.binary_data_capture_upload(
                    binary_data=path.read_bytes(), part_id=args.part_id,
                    component_type="camera", component_name=args.component_name,
                    method_name="GetImages", file_extension=".jpg",
                    tags=[label], dataset_ids=[dataset_id], mime_type="image/jpeg",
                    data_request_times=(now, now))
                progress.write(json.dumps({"image": str(path.relative_to(root)),
                                           "label": label,
                                           "binary_data_id": binary_id}) + "\n")
                progress.flush()
                n["done"] += 1
                if n["done"] % 20 == 0:
                    print(f"    {n['done']}/{len(todo)}  "
                          f"{time.monotonic() - t0:.0f}s", flush=True)

        await asyncio.gather(*(one(c, p) for c, p in todo))
        progress.close()
        print(f"  done: {n['done']} uploaded in {time.monotonic() - t0:.0f}s; "
              f"dataset id {dataset_id}", flush=True)
    finally:
        client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
