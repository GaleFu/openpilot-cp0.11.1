#!/usr/bin/env python3
import time
import json
import jwt
from typing import cast
from pathlib import Path

from datetime import datetime, timedelta, UTC
from openpilot.common.api import api_get, get_key_pair
from openpilot.common.params import Params
from openpilot.common.spinner import Spinner
from openpilot.selfdrive.selfdrived.alertmanager import set_offroad_alert
from openpilot.system.hardware import HARDWARE, PC
from openpilot.system.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog


UNREGISTERED_DONGLE_ID = "UnregisteredDevice"
# Keep the custom local label escaped so source encoding cannot be damaged by
# deployment tools that rewrite text files.
# Use an ASCII asterisk instead of U+272A, which is displayed as '?' by
# terminals/fonts that lack the BLACK STAR character.
CUSTOM_DONGLE_ID = "\u9a79\u54e5\u51fa\u54c1 \u5fc5\u5c5e\u7cbe\u54c1"
# /persist can be read-only; keep the user override on /data first.
DATA_DONGLE_ID_OVERRIDE_PATH = Path("/data/dongle_id_override")
PERSIST_DONGLE_ID_OVERRIDE_PATH = Path(Paths.persist_root()) / "comma" / "dongle_id_override"
DONGLE_ID_OVERRIDE_PATHS = (DATA_DONGLE_ID_OVERRIDE_PATH, PERSIST_DONGLE_ID_OVERRIDE_PATH)


def _normalize_dongle_id(value: str | None) -> str | None:
  """Accept the requested local ID or a normal 16-character device ID."""
  if value is None:
    return None
  value = value.strip()
  if value == CUSTOM_DONGLE_ID:
    return value
  if len(value) != 16 or any(c not in "0123456789abcdefABCDEF" for c in value):
    return None
  return value.lower()


def _read_dongle_id_override() -> str | None:
  for override_path in DONGLE_ID_OVERRIDE_PATHS:
    try:
      override = override_path.read_text(encoding="utf-8")
    except (FileNotFoundError, OSError):
      continue
    normalized = _normalize_dongle_id(override)
    if normalized is None:
      cloudlog.warning(f"Ignoring invalid DongleId override at {override_path}")
      continue
    return normalized
  return None


def is_registered_device() -> bool:
  # Keep early callers (for example sentry initialization) consistent with
  # register(), even if Params has not yet been repaired during this boot.
  dongle = _read_dongle_id_override() or Params().get("DongleId")
  return dongle not in (None, UNREGISTERED_DONGLE_ID)


def register(show_spinner=False) -> str | None:
  """
  All devices built since March 2024 come with all
  info stored in /persist/. This is kept around
  only for devices built before then.

  With a backend update to take serial number
  instead of dongle ID to some endpoints, this can be removed
  entirely.
  """
  params = Params()

  # An explicit override is stored outside /data/params so boot-time parameter
  # restore cannot replace a user-selected ID. The factory identity file is
  # left untouched and remains the rollback source.
  dongle_id_override = _read_dongle_id_override()
  dongle_id: str | None = dongle_id_override or params.get("DongleId")
  if dongle_id_override:
    if params.get("DongleId") != dongle_id_override:
      params.put("DongleId", dongle_id_override)
  elif dongle_id is None and Path(Paths.persist_root()+"/comma/dongle_id").is_file():
    # not all devices will have this; added early in comma 3X production (2/28/24)
    with open(Paths.persist_root()+"/comma/dongle_id") as f:
      dongle_id = f.read().strip()

  # Create registration token, in the future, this key will make JWTs directly
  jwt_algo, private_key, public_key = get_key_pair()

  # An explicit local override is authoritative. It is useful for test devices
  # and custom deployments where the normal hardware registration is not the
  # identity that should be used by the running software.
  if not public_key and not dongle_id_override:
    dongle_id = UNREGISTERED_DONGLE_ID
    cloudlog.warning("missing public key")
  elif dongle_id is None:
    if show_spinner:
      spinner = Spinner()
      spinner.update("registering device")

    # Block until we get the imei
    serial = HARDWARE.get_serial()
    start_time = time.monotonic()
    imei1: str | None = None
    imei2: str | None = None
    while imei1 is None and imei2 is None:
      try:
        imei1, imei2 = HARDWARE.get_imei(0), HARDWARE.get_imei(1)
      except Exception:
        cloudlog.exception("Error getting imei, trying again...")
        time.sleep(1)

      if time.monotonic() - start_time > 60 and show_spinner:
        spinner.update(f"registering device - serial: {serial}, IMEI: ({imei1}, {imei2})")

    backoff = 0
    start_time = time.monotonic()
    while True:
      try:
        register_token = jwt.encode({'register': True, 'exp': datetime.now(UTC).replace(tzinfo=None) + timedelta(hours=1)},
                                    cast(str, private_key), algorithm=jwt_algo)
        cloudlog.info("getting pilotauth")
        resp = api_get("v2/pilotauth/", method='POST', timeout=15,
                       imei=imei1, imei2=imei2, serial=serial, public_key=public_key, register_token=register_token)

        if resp.status_code in (402, 403):
          cloudlog.info(f"Unable to register device, got {resp.status_code}")
          dongle_id = UNREGISTERED_DONGLE_ID
        else:
          dongleauth = json.loads(resp.text)
          dongle_id = dongleauth["dongle_id"]
        break
      except Exception:
        cloudlog.exception("failed to authenticate")
        backoff = min(backoff + 1, 15)
        time.sleep(backoff)

      if time.monotonic() - start_time > 60 and show_spinner:
        spinner.update(f"registering device - serial: {serial}, IMEI: ({imei1}, {imei2})")

    if show_spinner:
      spinner.close()

  if dongle_id:
    params.put("DongleId", dongle_id)
    #set_offroad_alert("Offroad_UnofficialHardware", (dongle_id == UNREGISTERED_DONGLE_ID) and not PC)
  return dongle_id


if __name__ == "__main__":
  print(register())
