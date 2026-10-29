"""Where the probes get their api key pair.

Every probe here needs one and none of them should be the place that decides where it
comes from. `VIAM_API_KEY`/`VIAM_API_KEY_ID` in the environment win; failing that, the
file named by `QC_ENV_FILE` is read for the same two names, so a probe can be run from a
plain shell with nothing exported and no key on the command line.

Nothing here prints or returns a key - `options()` hands the pair straight to the SDK -
and no path to one is written down: name the file in `QC_ENV_FILE`.
"""
from __future__ import annotations

import os
from pathlib import Path

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
WANTED = ("VIAM_API_KEY", "VIAM_API_KEY_ID")


def _from_file() -> dict[str, str]:
    named = os.environ.get("QC_ENV_FILE", "")
    if not named:
        return {}
    path = Path(named).expanduser()
    if not path.exists():
        return {}
    found: dict[str, str] = {}
    for line in path.read_text().splitlines():
        line = line.strip().removeprefix("export ").strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        if key.strip() in WANTED:
            found.setdefault(key.strip(), value.strip().strip("'\""))
    return found


def pair() -> tuple[str, str]:
    """The (key_id, key) the SDK needs, for the app client, which takes them separately.

    Returned, never printed: the app client wants `auth_entity` and a `Credentials`
    payload rather than a `RobotClient.Options`, so there has to be one accessor that
    hands the two values over. Everything that talks to a MACHINE should use `options()`
    or `connect()` instead and not touch these at all.
    """
    found = _from_file()
    key = os.environ.get("VIAM_API_KEY") or found.get("VIAM_API_KEY")
    key_id = os.environ.get("VIAM_API_KEY_ID") or found.get("VIAM_API_KEY_ID")
    if not key or not key_id:
        raise SystemExit(
            "no api key pair: export VIAM_API_KEY and VIAM_API_KEY_ID, or point "
            "QC_ENV_FILE at a file holding both")
    return key_id, key


def options():
    """`RobotClient.Options` carrying the api key pair."""
    from viam.robot.client import RobotClient
    key_id, key = pair()
    return RobotClient.Options.with_api_key(api_key=key, api_key_id=key_id)


async def connect(fqdn: str | None = None):
    """A connected machine client for the cell."""
    from viam.robot.client import RobotClient
    return await RobotClient.at_address(fqdn or FQDN, options())
