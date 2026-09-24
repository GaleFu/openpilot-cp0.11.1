from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from .constants import BLINKER_LEFT, BLINKER_RIGHT


def _atc_turn_matches_side(atc_type: str, blinker_state: int) -> bool:
  """Match final and early CarrotMan turn commands to the driver's signal."""
  atc = (atc_type or "").strip().lower()
  side = "left" if blinker_state == BLINKER_LEFT else "right"
  return atc.startswith(f"turn {side}") or atc.startswith(f"atc {side}")


def classify_maneuver_type(blinker_state: int,
                           carstate,
                           side,                 # SideState
                           turn_desire_state: bool,
                           atc_type: str,
                           old_type: str,
                           is_changan_z6: bool = False,
                           x_dist_to_turn: float = 0.0):
  if blinker_state == 0:
    return "none"

  v_kph = carstate.vEgo * CV.MS_TO_KPH
  accel = carstate.aEgo

  nav_turn = (
    is_changan_z6
    and 0.0 < float(x_dist_to_turn) <= 120.0
    and _atc_turn_matches_side(atc_type, blinker_state)
  )
  # The signal-first steering nudge lives in the Changan CarController and is
  # deliberately independent from this model/navigation classification.
  if nav_turn:
    return "turn"

  score_turn = 0
  if v_kph < 30.0:
    score_turn += 1
  elif v_kph < 40.0 and accel < -1.0:
    score_turn += 1

  # 瞒肺 绝绊 edge 咯蜡档 绝栏搁 turn 啊魂
  if v_kph < 40.0 and (not side.lane_available) and (not side.edge_available):
    score_turn += 1

  # 瞒急捞 肋 救 焊捞搁(背瞒肺 殿)
  if v_kph < 40.0 and side.lane_exist_count.counter < int(0.5 / DT_MDL):
    score_turn += 1

  if turn_desire_state:
    score_turn += 1

  if _atc_turn_matches_side(atc_type, blinker_state):
    score_turn += 2
  elif atc_type in ("fork left", "fork right", "atc left", "atc right"):
    score_turn -= 2

  edge_far = side.dist_to_edge_far > 4.0

  if score_turn >= 2:
    if edge_far:
      return "turn"
    return old_type
  return "lane_change"
