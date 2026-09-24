#!/usr/bin/env python3
"""Changan Z6 radar/obstacle decoder.

The Z6 publishes dynamic ACC targets, AEB fallback data, and static obstacle
slots on the camera CAN bus.  This module exposes all of them to the normal
openpilot radar fusion.  Native Type/Direction values are retained in the
decoder but are not guessed as a pedestrian/oncoming label without a real
vehicle capture for calibration.
"""

from __future__ import annotations

import math

from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.car.changan.values import DBC


RADAR_BUS = 2
RADAR_PERIOD_S = 0.10
# card calls RadarInterface at roughly 100 Hz while the Z6 object banks are
# transmitted at roughly 10 Hz.  Hold the last valid bank for at most 0.2 s so
# a normal inter-frame gap does not make targets flicker or reset their tracks.
BANK_HOLD_CYCLES = 20
MAX_RANGE_M = 180.0
MAX_LAT_M = 12.0
MAX_REL_SPEED_MPS = 45.0
DYNAMIC_ID_BASE = 100
STATIC_ID_BASE = 1000
AEB_ID = 700


def _finite(value: object, default: float = 0.0) -> float:
  try:
    number = float(value)
  except (TypeError, ValueError):
    return default
  return number if math.isfinite(number) else default


def _int(value: object, default: int = 0) -> int:
  try:
    return int(value)
  except (TypeError, ValueError):
    return default


def _get(mapping: dict[str, float], *names: str, default: float = 0.0) -> float:
  for name in names:
    if name in mapping:
      return mapping[name]
  return default


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    messages = [
      # All object banks are optional.  Some Z6 software versions expose
      # only GW_31A/GW_307; NaN frequency keeps the parser alive without
      # making a missing bank invalidate radar data.
      ("GW_31A", float("nan")),
      ("GW_307", float("nan")),
      ("GW_382", float("nan")),
      ("GW_312", float("nan")),
      ("GW_3D9", float("nan")),
      ("GW_3DB", float("nan")),
    ]
    self.rcp = CANParser(DBC[CP.carFingerprint][Bus.pt], messages, RADAR_BUS)
    self.updated_messages: set[int] = set()
    self._last_position: dict[int, tuple[float, int]] = {}
    self._last_vrel: dict[int, float] = {}
    self._missing_cycles: dict[int, int] = {}
    self._bank_age = {0x31A: 99, 0x307: 99, 0x382: 99, 0x312: 99, 0x3D9: 99, 0x3DB: 99}
    self._cycle = 0
    self._last_update_time_s: float | None = None
    self._seen_since_update: set[int] = set()

  def _ensure_point(self, point_id: int):
    if point_id not in self.pts:
      point = structs.RadarData.RadarPoint()
      point.trackId = point_id
      self.pts[point_id] = point
    return self.pts[point_id]

  def _set_point(self, point_id: int, d_rel: float, y_rel: float, v_rel: float,
                 *, static: bool = False, a_rel: float = float("nan"),
                 yv_rel: float = 0.0, track_state: int = 1) -> bool:
    d_rel = _finite(d_rel, -1.0)
    y_rel = _finite(y_rel, 0.0)
    if not (0.2 < d_rel < MAX_RANGE_M and abs(y_rel) < MAX_LAT_M):
      return False
    if static:
      v_rel = -max(0.0, float(self.v_ego))
      track_state = 2
    else:
      v_rel = max(-MAX_REL_SPEED_MPS, min(MAX_REL_SPEED_MPS, _finite(v_rel)))

    point = self._ensure_point(point_id)
    point.dRel = d_rel
    point.yRel = y_rel
    point.vRel = v_rel
    point.vLead = max(0.0, float(self.v_ego) + v_rel)
    try:
      point.aRel = float(a_rel) if math.isfinite(float(a_rel)) else float("nan")
    except (TypeError, ValueError):
      point.aRel = float("nan")
    point.yvRel = _finite(yv_rel)
    point.measured = True
    point.radarSource = "frontRadar"
    point.trackState = track_state
    self._missing_cycles[point_id] = 0
    return True

  def _estimate_vrel(self, point_id: int, d_rel: float) -> float:
    previous = self._last_position.get(point_id)
    previous_v = self._last_vrel.get(point_id, 0.0)
    self._last_position[point_id] = (d_rel, self._cycle)
    if previous is None:
      return previous_v
    old_d, old_cycle = previous
    dt = max(RADAR_PERIOD_S, (self._cycle - old_cycle) * RADAR_PERIOD_S)
    raw = (d_rel - old_d) / dt
    v_rel = max(-MAX_REL_SPEED_MPS, min(MAX_REL_SPEED_MPS, 0.65 * previous_v + 0.35 * raw))
    self._last_vrel[point_id] = v_rel
    return v_rel

  def _parse_dynamic_bank(self, msg: dict[str, float], slots: range, speed: float = 0.0) -> set[int]:
    seen: set[int] = set()
    for slot in slots:
      prefix = f"ACC_Target{slot}"
      target_id = _int(_get(msg, f"{prefix}ID"))
      detected = _int(_get(msg, f"{prefix}Detection"))
      if target_id <= 0 or detected <= 0:
        continue
      d_rel = _get(msg, f"{prefix}LngRange")
      y_rel = _get(msg, f"{prefix}LatRange")
      point_id = DYNAMIC_ID_BASE + slot * 100 + target_id
      # GW_307 carries one shared relative-speed value for the ACC target
      # slots.  Zero is a valid value (same speed as ego), so do not treat it
      # as "missing" and replace it with a noisy position derivative.
      v_rel = speed if slot in (6, 7) else self._estimate_vrel(point_id, d_rel)
      if self._set_point(point_id, d_rel, y_rel, v_rel):
        seen.add(point_id)
    return seen

  def _parse_static_bank(self, msg: dict[str, float], slots: range) -> set[int]:
    seen: set[int] = set()
    for slot in slots:
      prefix = f"ACC_ObsTarget{slot}"
      target_id = _int(_get(msg, f"{prefix}ID"))
      target_type = _int(_get(msg, f"{prefix}Type"))
      d_rel = _get(msg, f"{prefix}LngRange")
      y_rel = _get(msg, f"{prefix}LatRange")
      # Some Z6 software revisions use type=0 for a valid generic obstacle;
      # the ID/range validity is therefore the authoritative gate here.
      if target_id <= 0:
        continue
      # Include the radar-native object ID in the track key.  Reusing only
      # the slot number can blend two different objects when a slot is
      # reassigned, producing a false velocity spike and a false oncoming
      # classification during a narrow-road pass.
      point_id = STATIC_ID_BASE + slot * 32 + target_id
      if self._set_point(point_id, d_rel, y_rel, 0.0, static=True, track_state=2):
        seen.add(point_id)
    return seen

  def _parse_aeb_target(self) -> set[int]:
    msg = self.rcp.vl["GW_31A"]
    mode = _int(_get(msg, "ACC_AEBTargetmode", "ACC_AEBStatus"))
    d_rel = _get(msg, "ACC_AEBTargetLngRange")
    y_rel = _get(msg, "ACC_AEBTargetLatRange")
    v_rel = _get(msg, "ACC_AEBTargetRelSpeed") / 3.6
    if mode <= 0 and not (0.2 < d_rel < MAX_RANGE_M):
      return set()
    return {AEB_ID} if self._set_point(AEB_ID, d_rel, y_rel, v_rel, track_state=3) else set()

  def _expire_points(self, seen: set[int]) -> None:
    for point_id in list(self.pts):
      if point_id in seen:
        continue
      missing = self._missing_cycles.get(point_id, 0) + 1
      self._missing_cycles[point_id] = missing
      if missing >= 3:
        self.pts.pop(point_id, None)
        self._missing_cycles.pop(point_id, None)
        self._last_position.pop(point_id, None)
        self._last_vrel.pop(point_id, None)

  def update(self, can_strings):
    values = self.rcp.update(can_strings)
    self.updated_messages.update(values)
    self._cycle += 1
    for address in self._bank_age:
      self._bank_age[address] += 1
      if address in values:
        self._bank_age[address] = 0
    trigger_addresses = {0x31A, 0x307, 0x382, 0x312, 0x3D9, 0x3DB}
    if not (self.updated_messages & trigger_addresses):
      return None

    ret = structs.RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True
    seen: set[int] = set()
    if "GW_382" in self.rcp.vl and self._bank_age[0x382] <= BANK_HOLD_CYCLES:
      seen |= self._parse_dynamic_bank(self.rcp.vl["GW_382"], range(1, 6))
    if "GW_307" in self.rcp.vl and self._bank_age[0x307] <= BANK_HOLD_CYCLES:
      shared_speed = _get(self.rcp.vl["GW_307"], "ACC_ACCTargetRelSpd") / 3.6
      seen |= self._parse_dynamic_bank(self.rcp.vl["GW_307"], range(6, 8), shared_speed)
    if "GW_3D9" in self.rcp.vl and self._bank_age[0x3D9] <= BANK_HOLD_CYCLES:
      seen |= self._parse_static_bank(self.rcp.vl["GW_3D9"], range(1, 11))
    if "GW_3DB" in self.rcp.vl and self._bank_age[0x3DB] <= BANK_HOLD_CYCLES:
      seen |= self._parse_static_bank(self.rcp.vl["GW_3DB"], range(11, 21))
    if "GW_31A" in self.rcp.vl and self._bank_age[0x31A] <= BANK_HOLD_CYCLES:
      seen |= self._parse_aeb_target()
    self._expire_points(seen)
    ret.points = list(self.pts.values())
    self.updated_messages.clear()
    return ret
