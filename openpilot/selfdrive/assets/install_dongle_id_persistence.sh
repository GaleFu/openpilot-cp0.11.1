#!/usr/bin/env bash
set -euo pipefail

TARGET="/data/openpilot/openpilot/system/athena/registration.py"
OVERRIDE_DIR="/persist/comma"
OVERRIDE_PATH="${OVERRIDE_DIR}/dongle_id_override"
PARAMS_PATH="/data/params/d/DongleId"

if [[ "$(id -u)" -ne 0 ]]; then
  echo "Error: run as root, for example: sudo $0" >&2
  exit 1
fi
if [[ ! -f "$TARGET" ]]; then
  echo "Error: $TARGET not found. Check the openpilot path first." >&2
  exit 1
fi

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
backup="${TARGET}.pre_dongle_id_persistence.${timestamp}"
cp -p -- "$TARGET" "$backup"

TARGET="$TARGET" OVERRIDE_PATH="$OVERRIDE_PATH" PARAMS_PATH="$PARAMS_PATH" python3 - <<'PY'
import os
from pathlib import Path

target = Path(os.environ["TARGET"])
override_path = Path(os.environ["OVERRIDE_PATH"])
params_path = Path(os.environ["PARAMS_PATH"])
custom_id = "\u9a79\u54e5\u51fa\u54c1\u272a\u5fc5\u5c5e\u7cbe\u54c1"
text = target.read_text(encoding="utf-8")

needs_patch = any(marker not in text for marker in (
  "DONGLE_ID_OVERRIDE_PATH",
  "CUSTOM_DONGLE_ID",
  "if value == CUSTOM_DONGLE_ID:",
))
if needs_patch:
  text = text.replace(
    'UNREGISTERED_DONGLE_ID = "UnregisteredDevice"\n',
    'UNREGISTERED_DONGLE_ID = "UnregisteredDevice"\n'
    'CUSTOM_DONGLE_ID = "\\u9a79\\u54e5\\u51fa\\u54c1\\u272a\\u5fc5\\u5c5e\\u7cbe\\u54c1"\n'
    'DONGLE_ID_OVERRIDE_PATH = Path(Paths.persist_root()) / "comma" / "dongle_id_override"\n\n'
    'def _normalize_dongle_id(value: str | None) -> str | None:\n'
    '  if value is None:\n'
    '    return None\n'
    '  value = value.strip()\n'
    '  if value == CUSTOM_DONGLE_ID:\n'
    '    return value\n'
    '  if len(value) != 16 or any(c not in "0123456789abcdefABCDEF" for c in value):\n'
    '    return None\n'
    '  return value.lower()\n\n'
    'def _read_dongle_id_override() -> str | None:\n'
    '  try:\n'
    '    override = DONGLE_ID_OVERRIDE_PATH.read_text(encoding="utf-8")\n'
    '  except (FileNotFoundError, OSError):\n'
    '    return None\n'
    '  return _normalize_dongle_id(override)\n\n',
  )
  text = text.replace(
    'def is_registered_device() -> bool:\n'
    '  dongle = Params().get("DongleId")\n',
    'def is_registered_device() -> bool:\n'
    '  dongle = _read_dongle_id_override() or Params().get("DongleId")\n',
  )
  text = text.replace(
    '  dongle_id: str | None = params.get("DongleId")\n'
    '  if dongle_id is None and Path(Paths.persist_root()+"/comma/dongle_id").is_file():\n',
    '  dongle_id_override = _read_dongle_id_override()\n'
    '  dongle_id: str | None = dongle_id_override or params.get("DongleId")\n'
    '  if dongle_id_override and params.get("DongleId") != dongle_id_override:\n'
    '    params.put("DongleId", dongle_id_override)\n'
    '  if dongle_id is None and Path(Paths.persist_root()+"/comma/dongle_id").is_file():\n',
  )
  text = text.replace(
    '  if not public_key:\n'
    '    dongle_id = UNREGISTERED_DONGLE_ID\n',
    '  if not public_key and not dongle_id_override:\n'
    '    dongle_id = UNREGISTERED_DONGLE_ID\n',
  )

  required = (
    "DONGLE_ID_OVERRIDE_PATH",
    "CUSTOM_DONGLE_ID",
    "dongle_id_override = _read_dongle_id_override()",
    "if not public_key and not dongle_id_override:",
  )
  missing = [marker for marker in required if marker not in text]
  if missing:
    raise SystemExit("registration.py patch did not apply: " + ", ".join(missing))

tmp = target.with_name(target.name + ".tmp_dongle_id_persistence")
tmp.write_text(text, encoding="utf-8")
os.chmod(tmp, target.stat().st_mode & 0o777)
os.replace(tmp, target)

override_path.parent.mkdir(parents=True, exist_ok=True)
tmp_override = override_path.with_name(override_path.name + ".tmp")
tmp_override.write_text(custom_id, encoding="utf-8")
os.chmod(tmp_override, 0o644)
os.replace(tmp_override, override_path)

# Params paths may be symlinks; write_text follows the link and preserves the
# Params database layout.
params_path.write_text(custom_id, encoding="utf-8")
print(custom_id)
PY

sync
echo "Persistent custom DongleId installed successfully."
echo "registration.py backup: $backup"
echo "override: $OVERRIDE_PATH"
echo "params: $PARAMS_PATH"
