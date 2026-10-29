"""Submit (and watch) the infeed classifier's training job.

A classification job over the dataset's own TAGS: the three classes the upload put on the
images are the labels the job trains on, so there is one place the class list lives and it
is `upload_infeed.CLASSES`. A job submitted with a label the dataset does not carry trains
on nothing and says so only at the end.

    set -a; . /home/shrews/viam/isaac-arcade/.scratch/secrets/viam-dataset-key.env; set +a
    PYTHONPATH=probes .venv/bin/python -m assembly.train_infeed --watch
"""
from __future__ import annotations

import argparse
import asyncio
import os
import sys
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from assembly.upload_infeed import CLASSES, connect                # noqa: E402
from viam.proto.app.mltraining import ModelType, TrainingStatus    # noqa: E402

ORG = "cd5185ed-7553-464b-84aa-ebf099ecc838"


async def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--org-id", default=ORG)
    ap.add_argument("--dataset-name", default="mailbox-infeed-v1")
    ap.add_argument("--model-name", default="mailbox-infeed-v1")
    ap.add_argument("--job-id", default=None, help="watch an existing job instead")
    ap.add_argument("--watch", action="store_true")
    args = ap.parse_args()

    client = await connect(os.environ["VIAM_API_KEY"], os.environ["VIAM_API_KEY_ID"])
    try:
        job_id = args.job_id
        if job_id is None:
            found = [d for d in
                     await client.data_client.list_datasets_by_organization_id(args.org_id)
                     if d.name == args.dataset_name]
            if not found:
                raise SystemExit(f"no dataset named {args.dataset_name}")
            version = datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")
            job_id = await client.ml_training_client.submit_training_job(
                org_id=args.org_id, dataset_id=found[0].id,
                model_name=args.model_name, model_version=version,
                model_type=ModelType.MODEL_TYPE_SINGLE_LABEL_CLASSIFICATION,
                tags=list(CLASSES))
            print(f"  submitted {job_id} for {args.model_name}:{version} "
                  f"on {list(CLASSES)}", flush=True)
        if not args.watch:
            return 0
        terminal = {TrainingStatus.TRAINING_STATUS_COMPLETED,
                    TrainingStatus.TRAINING_STATUS_FAILED,
                    TrainingStatus.TRAINING_STATUS_CANCELED}
        while True:
            # get_training_job returns the TrainingJobMetadata itself, not a wrapper.
            job = await client.ml_training_client.get_training_job(job_id)
            print(f"  {datetime.now().strftime('%H:%M:%S')} "
                  f"{TrainingStatus.Name(job.status)}", flush=True)
            if job.status in terminal:
                return 0 if job.status == TrainingStatus.TRAINING_STATUS_COMPLETED else 1
            await asyncio.sleep(45)
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
