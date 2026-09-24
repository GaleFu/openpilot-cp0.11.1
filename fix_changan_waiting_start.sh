#!/usr/bin/env bash
set -euo pipefail

# Repairs the startup crash reported as: KeyError: <CAR.CHANGAN_Z6>.
# Usage: /data/fix_changan_waiting_start.sh /data/openpilot
repo_dir="${1:-/data/openpilot}"

if [ -f "$repo_dir/opendbc/car/car_helpers.py" ]; then
  car_dir="$repo_dir/opendbc/car"
elif [ -f "$repo_dir/opendbc_repo/opendbc/car/car_helpers.py" ]; then
  car_dir="$repo_dir/opendbc_repo/opendbc/car"
else
  echo "ERROR: opendbc car directory not found under $repo_dir" >&2
  exit 1
fi

values="$car_dir/values.py"
torque_override="$car_dir/torque_data/override.toml"
car_capnp="$car_dir/car.capnp"

if [ ! -f "$values" ]; then
  echo "ERROR: values.py not found: $values" >&2
  exit 1
fi
if [ ! -f "$torque_override" ]; then
  echo "ERROR: torque override not found: $torque_override" >&2
  exit 1
fi
if [ ! -f "$car_capnp" ]; then
  echo "ERROR: car.capnp not found: $car_capnp" >&2
  exit 1
fi

if ! grep -q 'from opendbc.car.changan.values import CAR as CHANGAN' "$values" || \
   ! grep -q '^Platform = .*CHANGAN' "$values"; then
  if [ ! -f "$values.pre_changan_registration" ]; then
    cp -p -- "$values" "$values.pre_changan_registration"
  fi

  if ! grep -q 'from opendbc.car.changan.values import CAR as CHANGAN' "$values"; then
    sed -i '/from opendbc.car.body.values import CAR as BODY/a from opendbc.car.changan.values import CAR as CHANGAN' "$values"
  fi

  if ! grep -q '^Platform = .*CHANGAN' "$values"; then
    sed -i 's/^Platform = /Platform = CHANGAN | /' "$values"
  fi
fi

if ! grep -q '^[[:space:]]*changan @36;' "$car_capnp"; then
  if [ ! -f "$car_capnp.pre_changan_safety_model" ]; then
    cp -p -- "$car_capnp" "$car_capnp.pre_changan_safety_model"
  fi

  python3 - "$car_capnp" <<'PY'
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
safety_match = re.search(r"(  enum SafetyModel \{\n)(.*?)(\n  \})", text, re.DOTALL)
if safety_match is None:
  raise SystemExit("ERROR: SafetyModel enum was not found in car.capnp")

safety_body = safety_match.group(2)
if re.search(r"^\s*changan\s+@", safety_body, re.MULTILINE):
  raise SystemExit("ERROR: changan exists with an unexpected SafetyModel number")
if re.search(r"^\s*\w+\s+@36;", safety_body, re.MULTILINE):
  raise SystemExit("ERROR: SafetyModel number 36 is already occupied")

anchor = "    byd @35;"
if anchor not in safety_body:
  raise SystemExit("ERROR: expected 0111 SafetyModel anchor 'byd @35;' was not found")
safety_body = safety_body.replace(anchor, anchor + "\n    changan @36;", 1)
text = text[:safety_match.start(2)] + safety_body + text[safety_match.end(2):]
path.write_text(text, encoding="utf-8")
PY
fi

if ! grep -q '^"CHANGAN_Z6" = ' "$torque_override" || \
   ! grep -q '^"CHANGAN_Z6_IDD" = ' "$torque_override"; then
  if [ ! -f "$torque_override.pre_changan_torque_params" ]; then
    cp -p -- "$torque_override" "$torque_override.pre_changan_torque_params"
  fi

  python3 - "$torque_override" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
anchor = '"TESLA_MODEL_Y" = [nan, 2.5, nan]'

missing = []
if '"CHANGAN_Z6" = ' not in text:
  missing.append('"CHANGAN_Z6" = [nan, 1.5, nan]')
if '"CHANGAN_Z6_IDD" = ' not in text:
  missing.append('"CHANGAN_Z6_IDD" = [nan, 1.5, nan]')

if missing:
  if anchor not in text:
    raise SystemExit("ERROR: torque override layout is unexpected; no changes applied")
  block = '# Changan Z6 uses angle control; 1.5 m/s^2 is the conservative 099 baseline.\n' + '\n'.join(missing)
  text = text.replace(anchor, anchor + '\n\n' + block, 1)
  path.write_text(text, encoding="utf-8")
PY
fi

python3 -m py_compile "$values"
python3 - "$car_dir" <<'PY'
from pathlib import Path
import sys
import tomllib

car_dir = Path(sys.argv[1])
with (car_dir / "torque_data" / "override.toml").open("rb") as f:
  override = tomllib.load(f)
for name in ("CHANGAN_Z6", "CHANGAN_Z6_IDD"):
  if name not in override:
    raise SystemExit(f"ERROR: missing torque parameter: {name}")
  values = override[name]
  if len(values) != 3 or values[1] != 1.5:
    raise SystemExit(f"ERROR: invalid torque parameter for {name}: {values!r}")
print("PASS: Changan torque parameters are readable")
PY

grep -q 'from opendbc.car.changan.values import CAR as CHANGAN' "$values"
grep -q '^Platform = .*CHANGAN' "$values"
grep -q '^[[:space:]]*byd @35;' "$car_capnp"
grep -q '^[[:space:]]*changan @36;' "$car_capnp"
grep -q '^"CHANGAN_Z6" = \[nan, 1.5, nan\]' "$torque_override"
grep -q '^"CHANGAN_Z6_IDD" = \[nan, 1.5, nan\]' "$torque_override"

echo "PASS: SafetyModel.changan = 36 is installed."
echo "PASS: CHANGAN is registered and both Z6 torque entries are installed."
echo "Backups (when a change was needed):"
echo "  $values.pre_changan_registration"
echo "  $car_capnp.pre_changan_safety_model"
echo "  $torque_override.pre_changan_torque_params"
echo "Run: reboot"
