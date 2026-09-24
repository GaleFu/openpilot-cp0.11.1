from enum import IntEnum

import math

import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.changan import changancan
from opendbc.car.changan.values import CarControllerParams, ChanganFlags
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase

ACC_CRUISE_STEP = 10

# A low-speed turn signal is the first steering cue: it starts a bounded
# same-side steering nudge before navigation or the driving model sees a turn.
# No navigation, lane-edge, junction, or model-corner gate is used here.
Z6_SIGNAL_ENTRY_MAX_KPH = 20.0
# Once the low-speed cue has fired, keep it through the corner if speed rises
# slightly. This is still a bounded low-speed assist and prevents a 20.1 km/h
# sample from handing control back to the model mid-corner.
Z6_SIGNAL_ENTRY_LATCH_MAX_KPH = 30.0
# First controller update that observes a valid signal starts the cue; no
# intentional time delay is inserted before the same-side steering command.
Z6_SIGNAL_ENTRY_DELAY_FRAMES = 0
Z6_SIGNAL_ENTRY_CONFIRM_FRAMES = Z6_SIGNAL_ENTRY_DELAY_FRAMES
Z6_SIGNAL_ENTRY_MAX_FRAMES = 400     # bounded 4.0 s maximum after the cue fires
Z6_SIGNAL_ENTRY_TARGET_ANGLE = 130.0
Z6_SIGNAL_ENTRY_MIN_CMD_ANGLE = 20.0
Z6_SIGNAL_ENTRY_HANDOFF_ACTUAL_ANGLE = 100.0
Z6_SIGNAL_ENTRY_HANDOFF_MODEL_MIN_ANGLE = 60.0
Z6_SIGNAL_ENTRY_HANDOFF_CONFIRM_FRAMES = 10
# Keep the indicator-selected direction after the stalk self-cancels. This
# prevents a one-frame blinker drop from returning control to an undecided
# model and making the wheel reverse during a tight corner.
Z6_SIGNAL_ENTRY_RELEASE_HOLD_FRAMES = 35
Z6_SIGNAL_ENTRY_HANDOFF_RAMP_FRAMES = 60
Z6_SIGNAL_ENTRY_OPPOSING_RATE = 20.0
# The requested target is 130 degrees. This is only the Python-side target
# window; the per-frame command delta below and Panda/EPS safety checks still
# limit how quickly the wheel can physically move toward it.
Z6_SIGNAL_ENTRY_DIRECT_CMD_ERROR = Z6_SIGNAL_ENTRY_TARGET_ANGLE

# Keep a model-selected large turn from unwinding to straight ahead while the
# vehicle is still in the corner.  This is deliberately restricted to a
# low-speed, large-angle window so ordinary lane changes and gentle bends are
# unaffected.
Z6_MID_TURN_HOLD_MAX_KPH = 35.0
Z6_MID_TURN_HOLD_START_ANGLE = 85.0
Z6_MID_TURN_HOLD_MIN_ANGLE = 35.0
# 调整提示：如果方向盘仍然保持过头，可继续减小此值；
# 如果车辆在到达弯心前就过早回正，再适当增大此值。
Z6_MID_TURN_HOLD_MAX_ANGLE = 175.0
# 调整提示：保持帧数和交接帧数越小，越早把方向控制交还给路径规划；
# 这里的帧数以 50 Hz 的 CAN 转向更新频率计算。
Z6_MID_TURN_HOLD_FRAMES = 40           # 0.8 s at the 50 Hz CAN update
Z6_MID_TURN_HANDOFF_RAMP_FRAMES = 30   # then return to the model smoothly
Z6_MID_TURN_HOLD_OPPOSING_ANGLE = 25.0
Z6_MID_TURN_HOLD_OPPOSING_RATE = 20.0


class IaccEpsState(IntEnum):
  OFF = 0
  STANDBY = 1
  PREARM = 2
  REQUEST_ACTIVE = 3
  ACTIVE = 4
  PAUSE_WAIT_ACK = 5
  PAUSED = 6
  RECOVERY = 7


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_angle_last = 0.
    self.accel_last = 0.
    self.acc_decel_active = False
    self.long_active_last = False
    self.z6_long_activation_timer = 0
    self.z6_long_activation_speed_kph = 0.0
    self.z6_crawl_activation_active = False
    self.z6_low_speed_drive_filtered = 0.0
    self.z6_gap_initialized = False
    self.is_gas = bool(CP.flags & ChanganFlags.GAS)
    self.z6_signal_entry_count = 0
    self.z6_signal_entry_timer = 0
    self.z6_signal_entry_direction = 0
    self.z6_signal_entry_angle = 0.0
    self.z6_signal_entry_active = False
    self.z6_signal_entry_handoff_count = 0
    self.z6_signal_entry_handed_off = False
    self.z6_signal_entry_blending = False
    self.z6_mid_turn_direction = 0
    self.z6_mid_turn_timer = 0
    self.z6_mid_turn_hold_angle = 0.0
    self.z6_mid_turn_handoff = False

    self.steer_override_active = False
    self.steer_override_timer = 0
    self.steer_active_last = False
    self.eps_lateral_accepted_last = False
    self.eps_active_settle_timer = 0
    self.eps_command_authority = 0.

    # This is deliberately separate from CC.enabled/CC.latActive. EPS can drop
    # lateral authority and make controls disengage before the OEM IACC session
    # has had time to recover. Dropping 0x31A at that point prevents recovery.
    self.iacc_eps_state = IaccEpsState.OFF
    self.iacc_state_timer = 0
    self.iacc_recovery_clean_frames = 0
    self.iacc_session_hold_timer = 0

    self.acc_counters_initialized = False
    self.acc_status_counter = 0
    self.acc_cruise_counter = 0
    self.acc_iacc_counter = 0
    self.eps_lateral_status_counter = 0
    self.accel_cdd_enter = getattr(CarControllerParams, "Z6_ACCEL_CDD_ENTER", -0.25) if self.is_gas else \
      getattr(CarControllerParams, "IDD_ACCEL_CDD_ENTER", -0.12)
    self.accel_cdd_exit = getattr(CarControllerParams, "Z6_ACCEL_CDD_EXIT", -0.10) if self.is_gas else \
      getattr(CarControllerParams, "IDD_ACCEL_CDD_EXIT", -0.03)
    self.packer = CANPacker(dbc_names[Bus.pt])

  def _eps_envelope_max_angle(self, v_ego: float) -> float:
    """Largest steering-wheel angle the Z6 EPS accepts on the IACC path at this speed.

    The EPS firmware guards the external angle command with a speed-dependent
    lateral-acceleration budget; exceeding it rejects the IACC session and the
    UI reports steering unavailable. Shape the command to saturate under that
    budget instead of tripping the reject path.
    """
    cap = CarControllerParams.STEER_COMMAND_MAX_ANGLE
    budget = CarControllerParams.EPS_LATERAL_ACCEL_BUDGET
    if v_ego < 0.5 or budget <= 0.:
      return cap
    wheelbase = float(getattr(self.CP, "wheelbase", 0.) or 2.795)
    steer_ratio = float(getattr(self.CP, "steerRatio", 0.) or 14.5)
    road_wheel_rad = math.atan(budget * wheelbase / (v_ego * v_ego))
    return float(min(cap, math.degrees(road_wheel_rad) * steer_ratio))

  def _set_iacc_eps_state(self, state: IaccEpsState) -> bool:
    if state == self.iacc_eps_state:
      return False

    self.iacc_eps_state = state
    self.iacc_state_timer = 0
    if state == IaccEpsState.STANDBY:
      self.iacc_state_timer = CarControllerParams.IACC_STANDBY_FRAMES
    elif state == IaccEpsState.PREARM:
      self.iacc_state_timer = CarControllerParams.STEER_ARMING_FRAMES
    elif state in (IaccEpsState.REQUEST_ACTIVE, IaccEpsState.PAUSE_WAIT_ACK):
      self.iacc_state_timer = CarControllerParams.IACC_EPS_ACK_TIMEOUT_FRAMES
    elif state == IaccEpsState.RECOVERY:
      self.iacc_recovery_clean_frames = 0
      self.iacc_session_hold_timer = max(self.iacc_session_hold_timer,
                                          CarControllerParams.IACC_RECOVERY_SESSION_HOLD_FRAMES)
    return True

  def _update_iacc_eps_state(self, CC, CS, lateral_requested: bool, driver_pause: bool,
                             eps_status: int, eps_active: bool, eps_abort: bool) -> bool:
    # A session remains alive briefly after normal disengagement, and much
    # longer after an EPS reject. This gives EPS a continuous OEM HWA keepalive
    # while it clears its own reject path, without holding the session forever.
    external_session_requested = bool(CC.enabled or lateral_requested or CS.out.cruiseState.available)
    if external_session_requested:
      self.iacc_session_hold_timer = CarControllerParams.IACC_SESSION_OFF_HOLD_FRAMES
    elif self.iacc_session_hold_timer > 0:
      self.iacc_session_hold_timer -= 1
    session_requested = external_session_requested or self.iacc_session_hold_timer > 0

    eps_ready = eps_status == 1 and not eps_abort
    active_requested = lateral_requested and not driver_pause and eps_ready
    state = self.iacc_eps_state

    if state == IaccEpsState.OFF:
      if session_requested:
        return self._set_iacc_eps_state(IaccEpsState.STANDBY)

    elif state == IaccEpsState.STANDBY:
      if not session_requested:
        return self._set_iacc_eps_state(IaccEpsState.OFF)
      if eps_abort:
        return self._set_iacc_eps_state(IaccEpsState.RECOVERY)
      if self.iacc_state_timer > 0:
        self.iacc_state_timer -= 1
      elif active_requested:
        return self._set_iacc_eps_state(IaccEpsState.PREARM)

    elif state == IaccEpsState.PREARM:
      if not session_requested:
        return self._set_iacc_eps_state(IaccEpsState.OFF)
      if eps_abort:
        return self._set_iacc_eps_state(IaccEpsState.RECOVERY)
      if not lateral_requested or driver_pause:
        return self._set_iacc_eps_state(IaccEpsState.PAUSED)
      if not eps_ready:
        return self._set_iacc_eps_state(IaccEpsState.STANDBY)
      if self.iacc_state_timer > 0:
        self.iacc_state_timer -= 1
      else:
        return self._set_iacc_eps_state(IaccEpsState.REQUEST_ACTIVE)

    elif state == IaccEpsState.REQUEST_ACTIVE:
      if eps_abort:
        return self._set_iacc_eps_state(IaccEpsState.RECOVERY)
      if not session_requested:
        return self._set_iacc_eps_state(IaccEpsState.OFF)
      if not lateral_requested or driver_pause:
        return self._set_iacc_eps_state(IaccEpsState.PAUSED)
      if not eps_ready:
        return self._set_iacc_eps_state(IaccEpsState.STANDBY)
      if eps_active:
        return self._set_iacc_eps_state(IaccEpsState.ACTIVE)
      if self.iacc_state_timer > 0:
        self.iacc_state_timer -= 1
      else:
        # A missing EPS acknowledgement is an activation reject. Do not keep
        # reasserting EPS_LatCtrlActive indefinitely.
        return self._set_iacc_eps_state(IaccEpsState.RECOVERY)

    elif state == IaccEpsState.ACTIVE:
      if eps_abort or not eps_ready or not eps_active:
        return self._set_iacc_eps_state(IaccEpsState.RECOVERY)
      if not lateral_requested or driver_pause:
        # Stock behavior drops the 0x1BA active request first, waits for the
        # EPS feedback to go low, then reports HWA paused (mode 4).
        return self._set_iacc_eps_state(IaccEpsState.PAUSE_WAIT_ACK)

    elif state == IaccEpsState.PAUSE_WAIT_ACK:
      if eps_abort:
        return self._set_iacc_eps_state(IaccEpsState.RECOVERY)
      if not eps_active:
        return self._set_iacc_eps_state(IaccEpsState.PAUSED)
      if self.iacc_state_timer > 0:
        self.iacc_state_timer -= 1
      else:
        return self._set_iacc_eps_state(IaccEpsState.RECOVERY)

    elif state == IaccEpsState.PAUSED:
      if not session_requested:
        return self._set_iacc_eps_state(IaccEpsState.OFF)
      if eps_abort:
        return self._set_iacc_eps_state(IaccEpsState.RECOVERY)
      if active_requested:
        return self._set_iacc_eps_state(IaccEpsState.PREARM)

    elif state == IaccEpsState.RECOVERY:
      if not session_requested:
        return self._set_iacc_eps_state(IaccEpsState.OFF)
      if eps_active or not eps_ready:
        self.iacc_recovery_clean_frames = 0
      else:
        self.iacc_recovery_clean_frames += 1
        if self.iacc_recovery_clean_frames >= CarControllerParams.IACC_RECOVERY_CLEAN_FRAMES:
          return self._set_iacc_eps_state(IaccEpsState.STANDBY)

    return False

  def _iacc_hwa_mode(self, eps_active: bool) -> int:
    if self.iacc_eps_state == IaccEpsState.OFF:
      return 0
    if self.iacc_eps_state == IaccEpsState.STANDBY:
      return 2
    if self.iacc_eps_state in (IaccEpsState.PREARM, IaccEpsState.REQUEST_ACTIVE):
      return 1
    if self.iacc_eps_state in (IaccEpsState.ACTIVE, IaccEpsState.PAUSE_WAIT_ACK):
      return 3
    if self.iacc_eps_state == IaccEpsState.PAUSED:
      return 4
    # On a fault, preserve the normal active -> inactive ordering before
    # returning to standby. This avoids reporting mode 2 while EPS still says
    # its lateral controller is active.
    return 3 if eps_active else 2

  def _apply_z6_signal_entry(self, desired_angle: float, actual_angle: float,
                             steering_rate: float, v_ego: float,
                             steer_active: bool, max_command_angle: float,
                             left_blinker: bool, right_blinker: bool,
                             driver_opposing: bool) -> float:
    """Use the low-speed signal itself to steer before the model sees a turn."""
    indicator_direction = (1 if left_blinker and not right_blinker else
                           -1 if right_blinker and not left_blinker else 0)
    signal_direction = indicator_direction
    # The Z6 indicator may self-cancel as the wheel begins the turn. Once the
    # cue has fired, retain its direction until the bounded assist window ends
    # instead of immediately handing control back to an undecided model.
    latched_entry = (self.z6_signal_entry_active and
                     self.z6_signal_entry_timer < Z6_SIGNAL_ENTRY_MAX_FRAMES and
                     self.z6_signal_entry_direction != 0)
    if signal_direction == 0 and latched_entry:
      signal_direction = self.z6_signal_entry_direction
    assist_allowed = (self.is_gas and steer_active and 0.1 < v_ego and
                      v_ego * CV.MS_TO_KPH <= (Z6_SIGNAL_ENTRY_LATCH_MAX_KPH
                                               if latched_entry else Z6_SIGNAL_ENTRY_MAX_KPH) and
                      signal_direction != 0 and not driver_opposing)
    # Do not cancel because the model or EPS measurement is briefly on the
    # opposite side. Before a tight corner is recognized this is expected and
    # would otherwise reset the signal-entry state forever. A real driver
    # counter-steer is represented by driver_opposing and remains an immediate
    # safety exit.
    if not assist_allowed:
      self.z6_signal_entry_active = False
      self.z6_signal_entry_count = 0
      self.z6_signal_entry_timer = 0
      self.z6_signal_entry_direction = 0
      self.z6_signal_entry_angle = 0.0
      self.z6_signal_entry_handoff_count = 0
      self.z6_signal_entry_handed_off = False
      self.z6_signal_entry_blending = False
      return desired_angle

    if self.z6_signal_entry_direction != signal_direction:
      self.z6_signal_entry_count = 0
      self.z6_signal_entry_timer = 0
      self.z6_signal_entry_angle = 0.0
      self.z6_signal_entry_handoff_count = 0
      self.z6_signal_entry_handed_off = False
      self.z6_signal_entry_blending = False
      self.z6_signal_entry_direction = signal_direction

    self.z6_signal_entry_count += 1
    if self.z6_signal_entry_count < Z6_SIGNAL_ENTRY_CONFIRM_FRAMES:
      self.z6_signal_entry_active = False
      self.z6_signal_entry_blending = False
      return desired_angle
    if self.z6_signal_entry_timer >= Z6_SIGNAL_ENTRY_MAX_FRAMES:
      self.z6_signal_entry_active = False
      self.z6_signal_entry_blending = False
      return desired_angle
    self.z6_signal_entry_timer += 1
    self.z6_signal_entry_active = True

    # The cue is deliberately a fixed, same-side 130-degree steering target.
    # The downstream EPS angle/error/rate and Panda safety limits still apply
    # when this target is converted into the actual CAN command.
    target_abs = Z6_SIGNAL_ENTRY_TARGET_ANGLE
    self.z6_signal_entry_angle = target_abs
    # Keep the explicit same-side cue latched for the whole bounded window.
    # Handing back to the model as soon as the wheel reaches a large angle is
    # unsafe on a right-angle corner: the model can still be undecided, causing
    # the command to alternate left/right. The model gets control again only
    # after the indicator is released and the short handoff ramp completes,
    # while a real driver counter-steer still exits immediately above.
    self.z6_signal_entry_handed_off = False
    assisted_angle = signal_direction * target_abs

    # A stalk self-cancel is not a reliable indication that the corner is
    # complete. Hold the same-side target briefly, then blend to the model
    # rather than changing from +130 to a possibly opposite model angle in one
    # frame. This removes the left/right command chatter seen in testing.
    if indicator_direction == 0:
      self.z6_signal_entry_handoff_count += 1
      if self.z6_signal_entry_handoff_count > Z6_SIGNAL_ENTRY_RELEASE_HOLD_FRAMES:
        self.z6_signal_entry_blending = True
        blend = min(1.0, (self.z6_signal_entry_handoff_count -
                          Z6_SIGNAL_ENTRY_RELEASE_HOLD_FRAMES) /
                         max(1, Z6_SIGNAL_ENTRY_HANDOFF_RAMP_FRAMES))
        assisted_angle = ((1.0 - blend) * assisted_angle +
                          blend * desired_angle)
        if blend >= 1.0:
          self.z6_signal_entry_active = False
          self.z6_signal_entry_handed_off = True
          self.z6_signal_entry_blending = False
          return desired_angle
    else:
      self.z6_signal_entry_handoff_count = 0
      self.z6_signal_entry_blending = False
    return float(np.clip(assisted_angle, -max_command_angle, max_command_angle))

  def _apply_z6_mid_turn_hold(self, desired_angle: float, actual_angle: float,
                              v_ego_kph: float, steer_active: bool,
                              driver_opposing: bool,
                              advance_timer: bool = True) -> float:
    """Hold the selected side briefly when a sharp turn is already underway.

    The vision model can reduce its curvature target toward zero in the middle
    of a tight corner before the vehicle has cleared the turn.  That transient
    would unwind the EPS command and make the car run wide.  Once a genuinely
    large same-side target or measured wheel angle is seen, keep a bounded
    same-side floor for a few seconds.  A clear opposite model command,
    driver counter-steer, speed increase, or timeout immediately hands control
    back to the normal planner path.
    """
    def reset() -> float:
      self.z6_mid_turn_direction = 0
      self.z6_mid_turn_timer = 0
      self.z6_mid_turn_hold_angle = 0.0
      self.z6_mid_turn_handoff = False
      return desired_angle

    if (not self.is_gas or not steer_active or driver_opposing or
        v_ego_kph > Z6_MID_TURN_HOLD_MAX_KPH):
      return reset()

    direction = self.z6_mid_turn_direction
    if direction == 0:
      # Prefer the current model target when it is large.  This avoids using a
      # stale measured angle as the direction during a rapid direction change.
      if abs(desired_angle) >= Z6_MID_TURN_HOLD_START_ANGLE:
        direction = 1 if desired_angle > 0.0 else -1
      elif abs(actual_angle) >= Z6_MID_TURN_HOLD_START_ANGLE:
        direction = 1 if actual_angle > 0.0 else -1
      else:
        return desired_angle
      self.z6_mid_turn_direction = direction
      self.z6_mid_turn_timer = 0

    # A strong opposite request is a real handoff (for example, a lane change
    # after a cancelled turn), not the brief near-zero dip this filter targets.
    if desired_angle * direction < -Z6_MID_TURN_HOLD_OPPOSING_ANGLE:
      return reset()

    if advance_timer:
      self.z6_mid_turn_timer += 1
    if self.z6_mid_turn_timer > Z6_MID_TURN_HOLD_FRAMES + Z6_MID_TURN_HANDOFF_RAMP_FRAMES:
      return reset()

    # 调整提示：这里的保持下限已经比原来的大角度保持更柔和。
    # 降低实测方向角比例，会让车辆通过弯心后更早回收方向，
    # 避免方向盘保持过多导致出弯滞后。
    hold_angle = max(
      Z6_MID_TURN_HOLD_MIN_ANGLE,
      min(Z6_MID_TURN_HOLD_MAX_ANGLE, abs(actual_angle) * 0.68),
    )
    # Keep a stable floor based on the command already sent.  This prevents a
    # one-frame measured-angle drop from releasing the wheel in the corner.
    if abs(self.apply_angle_last) > hold_angle:
      hold_angle = min(Z6_MID_TURN_HOLD_MAX_ANGLE,
                       abs(self.apply_angle_last) * 0.62)
    if desired_angle * direction > 0.0:
      hold_angle = max(hold_angle,
                       min(Z6_MID_TURN_HOLD_MAX_ANGLE, abs(desired_angle)))
    self.z6_mid_turn_hold_angle = hold_angle

    if self.z6_mid_turn_timer > Z6_MID_TURN_HOLD_FRAMES:
      self.z6_mid_turn_handoff = True
      blend = min(1.0, (self.z6_mid_turn_timer - Z6_MID_TURN_HOLD_FRAMES) /
                  max(1, Z6_MID_TURN_HANDOFF_RAMP_FRAMES))
      # Do not jump from the held angle to a straight target.  The floor is
      # reduced over the ramp, while any same-side model request is preserved.
      hold_angle *= 1.0 - blend

    if direction > 0:
      held = max(desired_angle, hold_angle)
    else:
      held = min(desired_angle, -hold_angle)
    if self.z6_mid_turn_handoff and blend >= 1.0:
      return reset()
    return float(np.clip(held, -Z6_MID_TURN_HOLD_MAX_ANGLE,
                         Z6_MID_TURN_HOLD_MAX_ANGLE))

  def _apply_z6_low_speed_brake_smoothing(self, accel_cmd: float,
                                          v_ego_kph: float,
                                          standstill: bool) -> float:
    """Release low-speed braking progressively while keeping stronger decel immediate."""
    low_speed_limit = getattr(CarControllerParams, "Z6_LOW_SPEED_BRAKE_KPH", 40.0)
    release_step = getattr(CarControllerParams, "Z6_LOW_SPEED_BRAKE_RELEASE_STEP", 0.04)
    if not (self.is_gas and not standstill and v_ego_kph <= low_speed_limit):
      return accel_cmd

    # A new, stronger brake request must pass through immediately. Only the
    # transition back toward zero is rate limited to prevent brake pulsing.
    if self.accel_last < 0.0 and accel_cmd > self.accel_last:
      return min(accel_cmd, self.accel_last + release_step)
    return accel_cmd

  def _apply_z6_low_speed_drive_smoothing(self, accel_cmd: float,
                                          v_ego_kph: float,
                                          set_speed_kph: float,
                                          standstill: bool) -> float:
    """Smooth positive drive torque below 40 km/h without delaying braking."""
    low_speed_limit = getattr(CarControllerParams, "Z6_LOW_SPEED_BRAKE_KPH", 40.0)
    if not (self.is_gas and not standstill and 0.1 < v_ego_kph <= low_speed_limit):
      self.z6_low_speed_drive_filtered = max(0.0, accel_cmd)
      return accel_cmd
    if accel_cmd <= 0.0:
      self.z6_low_speed_drive_filtered = 0.0
      return accel_cmd
    previous = self.z6_low_speed_drive_filtered
    up_alpha = getattr(CarControllerParams, "Z6_LOW_SPEED_DRIVE_FILTER_UP", 0.12)
    down_alpha = getattr(CarControllerParams, "Z6_LOW_SPEED_DRIVE_FILTER_DOWN", 0.20)
    alpha = up_alpha if accel_cmd >= previous else down_alpha
    target = min(accel_cmd * getattr(CarControllerParams, "Z6_LOW_SPEED_DRIVE_SCALE", 0.80),
                 getattr(CarControllerParams, "Z6_LOW_SPEED_DRIVE_ACCEL_CAP", 0.18))
    if set_speed_kph > 1.0:
      margin = getattr(CarControllerParams, "Z6_LOW_SPEED_SET_SPEED_MARGIN_KPH", 0.5)
      if v_ego_kph >= set_speed_kph + margin:
        target = 0.0
      elif v_ego_kph >= set_speed_kph - margin:
        target = min(target,
                     getattr(CarControllerParams, "Z6_LOW_SPEED_SET_SPEED_ACCEL_CAP", 0.05))
    filtered = previous + alpha * (target - previous)
    self.z6_low_speed_drive_filtered = max(0.0, filtered)
    return self.z6_low_speed_drive_filtered

  def _apply_z6_low_speed_longitudinal_smoothing(self, accel_cmd: float,
                                                 v_ego_kph: float,
                                                 standstill: bool) -> float:
    """Limit low-speed longitudinal jerk and soften the first active frame."""
    low_speed_limit = getattr(CarControllerParams, "Z6_LOW_SPEED_BRAKE_KPH", 40.0)
    if not (self.is_gas and not standstill and 0.1 < v_ego_kph <= low_speed_limit):
      return accel_cmd

    previous = float(self.accel_last)
    up_step = getattr(CarControllerParams, "Z6_LOW_SPEED_ACCEL_UP_STEP", 0.06)
    down_step = getattr(CarControllerParams, "Z6_LOW_SPEED_ACCEL_DOWN_STEP", 0.12)
    immediate_brake = getattr(CarControllerParams, "Z6_LOW_SPEED_IMMEDIATE_BRAKE", -0.50)

    # A newly enabled session starts from a neutral request even if the last
    # longitudinal CAN update happened before CC.longActive was dropped.
    if self.z6_long_activation_timer > 0:
      previous = min(previous, 0.0)
      activation_cap = getattr(CarControllerParams, "Z6_LONG_ACTIVATION_MAX_ACCEL", 0.50)
      accel_cmd = min(accel_cmd, activation_cap)
      # Hold the speed that existed at the activation edge. Until the vehicle
      # is clearly below that anchor, do not let a transient planner request
      # add drive torque and create the observed 25 -> 30 -> 25 overshoot.
      speed_margin = getattr(CarControllerParams, "Z6_LONG_ACTIVATION_SPEED_MARGIN_KPH", 0.5)
      if v_ego_kph >= self.z6_long_activation_speed_kph - speed_margin:
        accel_cmd = min(accel_cmd,
                        getattr(CarControllerParams, "Z6_LONG_ACTIVATION_ACCEL_CAP", 0.12))

    # Crawl-mode stability has already removed drive torque and classified a
    # meaningful brake request. Do not let this older 40 km/h comfort limiter
    # reduce it back into the +/-0.08 neutral band, or a sustained stop request
    # could be swallowed one frame at a time.
    crawl_brake_enter = getattr(CarControllerParams, "Z6_CRAWL_ACTIVATION_BRAKE_ENTER", -0.18)
    if self.z6_crawl_activation_active and accel_cmd <= crawl_brake_enter:
      return accel_cmd

    if accel_cmd > previous:
      # Releasing a brake request remains the slowest transition. Positive
      # drive torque is also rate-limited to prevent a low-speed activation hit.
      release_step = getattr(CarControllerParams, "Z6_LOW_SPEED_BRAKE_RELEASE_STEP", 0.04)
      step = release_step if previous < 0.0 else up_step
      return min(accel_cmd, previous + step)

    if accel_cmd < previous:
      # Preserve rapid stronger braking; only small changes are filtered to
      # stop the planner from toggling the car between drive and brake torque.
      if accel_cmd <= immediate_brake:
        return accel_cmd
      return max(accel_cmd, previous - down_step)

    return accel_cmd

  def _apply_z6_crawl_activation_stability(self, accel_cmd: float,
                                           v_ego_kph: float,
                                           standstill: bool,
                                           driver_brake: bool = False) -> float:
    """Prevent drive/brake hunting after a moving engagement below 20 km/h."""
    if (not self.is_gas or not self.z6_crawl_activation_active or
        self.z6_long_activation_timer <= 0 or standstill or driver_brake or
        v_ego_kph > getattr(CarControllerParams, "Z6_CRAWL_ACTIVATION_MAX_KPH", 20.0)):
      return accel_cmd

    previous = float(self.accel_last)
    neutral_band = getattr(CarControllerParams, "Z6_CRAWL_ACTIVATION_NEUTRAL_BAND", 0.08)
    brake_enter = getattr(CarControllerParams, "Z6_CRAWL_ACTIVATION_BRAKE_ENTER", -0.18)
    immediate_brake = getattr(CarControllerParams, "Z6_LOW_SPEED_IMMEDIATE_BRAKE", -0.50)
    emergency_accel = getattr(CarControllerParams, "Z6_FOLLOW_BRAKE_EMERGENCY_ACCEL", -1.50)

    # Keep the existing low-speed fast path for a substantial brake request;
    # crawl stabilization is only for the small oscillating requests around
    # neutral.
    if accel_cmd <= immediate_brake:
      return accel_cmd

    # The planner's small +/- requests around zero are the oscillating part.
    # Hold the current mode instead of switching between motor torque and CDD.
    if abs(accel_cmd) <= neutral_band:
      if previous < 0.0:
        accel_cmd = min(0.0, previous + getattr(
          CarControllerParams, "Z6_CRAWL_ACTIVATION_DECEL_STEP", 0.04))
      elif previous > 0.0:
        accel_cmd = max(0.0, previous - getattr(
          CarControllerParams, "Z6_CRAWL_ACTIVATION_ACCEL_STEP", 0.02))
      else:
        accel_cmd = 0.0
    elif accel_cmd < 0.0 and previous >= 0.0:
      # Remove drive torque immediately, but do not enter the brake path for a
      # weak one-frame request. A meaningful or urgent brake request proceeds.
      if accel_cmd > brake_enter:
        accel_cmd = 0.0
      elif accel_cmd > emergency_accel:
        accel_cmd = max(accel_cmd, brake_enter)
    elif accel_cmd > 0.0 and previous < 0.0:
      # Release the brake to neutral before asking for drive torque on a later
      # frame. This guarantees there is no direct brake-to-power reversal.
      accel_cmd = min(0.0, previous + getattr(
        CarControllerParams, "Z6_CRAWL_ACTIVATION_DECEL_STEP", 0.04))

    if accel_cmd > previous and previous >= 0.0:
      up_step = getattr(CarControllerParams, "Z6_CRAWL_ACTIVATION_ACCEL_STEP", 0.02)
      accel_cap = getattr(CarControllerParams, "Z6_CRAWL_ACTIVATION_ACCEL_CAP", 0.12)
      return min(accel_cmd, previous + up_step, accel_cap)
    if accel_cmd < previous and previous < 0.0 and accel_cmd > emergency_accel:
      down_step = getattr(CarControllerParams, "Z6_CRAWL_ACTIVATION_DECEL_STEP", 0.04)
      return max(accel_cmd, previous - down_step)
    return accel_cmd

  def _apply_z6_follow_brake_smoothing(self, accel_cmd: float,
                                       v_ego_kph: float,
                                       standstill: bool,
                                       driver_brake: bool = False) -> float:
    """Shape normal follow-car deceleration over the complete speed range.

    Planner acceleration can change several tenths of m/s2 between adjacent
    50 Hz CAN updates.  Passing that step directly to the Z6 ACC brake path
    feels like a hard grab, then a release when the lead estimate moves by a
    frame.  Limit only ordinary requests: a driver pedal, standstill command,
    or a strong deceleration request is handed to the faster bounded ramp
    below instead of being delayed by this ordinary-follow filter.
    """
    low_speed_limit = getattr(CarControllerParams, "Z6_LOW_SPEED_BRAKE_KPH", 40.0)
    # The existing low-speed path below has its own tighter thresholds.  Keep
    # one limiter active at a time so the two filters do not add their delays.
    if not self.is_gas or standstill or driver_brake or v_ego_kph <= low_speed_limit:
      return accel_cmd

    previous = float(self.accel_last)
    emergency_accel = getattr(CarControllerParams,
                              "Z6_FOLLOW_BRAKE_EMERGENCY_ACCEL", -1.80)
    if accel_cmd <= emergency_accel:
      return accel_cmd

    if accel_cmd < 0.0 <= previous:
      # Remove positive drive torque immediately when a braking request
      # arrives, then enter the brake path with a small negative command.  A
      # slow ramp through the previous positive request would delay response
      # to a closing lead vehicle.
      entry_step = getattr(CarControllerParams,
                           "Z6_FOLLOW_BRAKE_ENTRY_STEP", 0.035)
      return max(accel_cmd, -entry_step)

    if accel_cmd < previous and previous < 0.0:
      # Once braking, allow a slightly faster but still bounded ramp deeper
      # into the stop.  This removes the initial brake grab without stretching
      # a normal follow stop over seconds.
      down_step = getattr(CarControllerParams,
                          "Z6_FOLLOW_BRAKE_DOWN_STEP", 0.055)
      return max(accel_cmd, previous - down_step)

    if previous < 0.0 and accel_cmd > previous:
      # Brake release is also ramped.  A tiny planner sign change should not
      # alternate the vehicle between CDD and drive torque paths.
      release_step = getattr(CarControllerParams,
                             "Z6_FOLLOW_BRAKE_RELEASE_STEP", 0.045)
      deadband = getattr(CarControllerParams,
                         "Z6_FOLLOW_BRAKE_RELEASE_DEADBAND", 0.02)
      if accel_cmd - previous <= deadband:
        release_step = min(release_step, deadband)
      return min(accel_cmd, previous + release_step)

    return accel_cmd

  def _apply_z6_rapid_brake_smoothing(self, accel_cmd: float,
                                      v_ego_kph: float,
                                      standstill: bool,
                                      driver_brake: bool = False) -> float:
    """Bound the first brake step during rapid closing and cut-in events.

    The planner can jump directly from drive torque to a large negative
    acceleration when it selects a much closer lead.  Z6 applies that CDD
    transition sharply.  Use the planner request itself as the urgency signal:
    ordinary follow requests retain the existing tuning, strong requests ramp
    faster, and critical requests ramp fastest without a one-frame full step.
    Driver braking and standstill behavior remain unmodified.
    """
    low_speed_limit = getattr(CarControllerParams, "Z6_LOW_SPEED_BRAKE_KPH", 40.0)
    if not self.is_gas or standstill or driver_brake:
      return accel_cmd

    previous = float(self.accel_last)
    trigger = (getattr(CarControllerParams,
                       "Z6_RAPID_BRAKE_LOW_SPEED_TRIGGER_ACCEL", -0.50)
               if v_ego_kph <= low_speed_limit else
               getattr(CarControllerParams, "Z6_RAPID_BRAKE_TRIGGER_ACCEL", -1.50))
    if accel_cmd > trigger or accel_cmd >= previous:
      return accel_cmd

    critical = getattr(CarControllerParams, "Z6_RAPID_BRAKE_CRITICAL_ACCEL", -2.80)
    if accel_cmd <= critical:
      down_step = getattr(CarControllerParams,
                          "Z6_RAPID_BRAKE_CRITICAL_DOWN_STEP", 0.25)
    else:
      down_step = getattr(CarControllerParams, "Z6_RAPID_BRAKE_DOWN_STEP", 0.10)

    if previous >= 0.0:
      # Drop drive torque immediately, then enter CDD with a bounded request.
      entry_step = getattr(CarControllerParams, "Z6_RAPID_BRAKE_ENTRY_STEP", 0.08)
      if accel_cmd <= critical:
        entry_step = max(entry_step, down_step)
      return max(accel_cmd, -entry_step)

    return max(accel_cmd, previous - down_step)

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    if not self.acc_counters_initialized:
      self.acc_status_counter = int(getattr(CS, "acc_status_values", {}).get("ACC_RollingCounter_24E", 0))
      self.acc_cruise_counter = int(getattr(CS, "acc_cruise_status_values", {}).get("ACC_RollingCounter_35E", 0))
      self.acc_iacc_counter = int(getattr(CS, "acc_iacc_status_values", {}).get("ACC_RollingCounter_36D", 0))
      self.eps_lateral_status_counter = int(getattr(CS, "eps_lateral_status_values", {}).get("EPS_RollingCounter_17E", 0))
      self.acc_counters_initialized = True

    driver_torque = abs(getattr(CS.out, "steeringTorque", 0.))
    driver_torsion = abs(getattr(CS, "eps_torsion_bar_torque", 0.))
    driver_steer_angle = abs(getattr(CS.out, "steeringAngleDeg", 0.))
    steer_override_enter = CarControllerParams.STEER_OVERRIDE_TAKEOVER_THRESHOLD_GAS if self.is_gas else \
      CarControllerParams.STEER_OVERRIDE_TAKEOVER_THRESHOLD_IDD
    torsion_override_enter = CarControllerParams.STEER_OVERRIDE_TAKEOVER_TORSION_THRESHOLD
    if self.steer_override_active:
      if driver_torque < CarControllerParams.STEER_OVERRIDE_RELEASE_THRESHOLD and \
         driver_torsion < CarControllerParams.STEER_OVERRIDE_RELEASE_TORSION_THRESHOLD and \
         driver_steer_angle < CarControllerParams.STEER_OVERRIDE_RELEASE_ANGLE:
        self.steer_override_active = False
    elif driver_torque > steer_override_enter or driver_torsion > torsion_override_enter:
      self.steer_override_active = True

    if self.steer_override_active:
      self.steer_override_timer = CarControllerParams.STEER_OVERRIDE_COOLDOWN_FRAMES
    elif self.steer_override_timer > 0:
      self.steer_override_timer -= 1

    lateral_requested = bool(CC.latActive)
    reengage_unsettled = (lateral_requested and not self.steer_active_last and
                          driver_torsion > CarControllerParams.STEER_REENGAGE_TORSION_THRESHOLD and
                          abs(getattr(CS.out, "steeringRateDeg", 0.)) > CarControllerParams.STEER_REENGAGE_RATE_THRESHOLD)
    driver_override_paused = self.steer_override_active or self.steer_override_timer > 0
    driver_pause = driver_override_paused or reengage_unsettled

    # CarState publishes these fields on CS.out. Keep the CS fallback for
    # older adapters, but do not require CC.latActive: the whole purpose of
    # this feature is to start the turn before the model recognizes it.
    left_blinker = bool(getattr(CS.out, "leftBlinker", getattr(CS, "leftBlinker", False)))
    right_blinker = bool(getattr(CS.out, "rightBlinker", getattr(CS, "rightBlinker", False)))
    v_ego_kph = CS.out.vEgo * CV.MS_TO_KPH
    single_signal = left_blinker != right_blinker
    signal_entry_latched = (self.z6_signal_entry_active and
                            self.z6_signal_entry_timer < Z6_SIGNAL_ENTRY_MAX_FRAMES and
                            self.z6_signal_entry_direction != 0)
    signal_entry_requested = (self.is_gas and bool(CC.enabled) and not driver_pause and
                              not bool(CS.out.standstill) and 0.1 < CS.out.vEgo and
                              v_ego_kph <= (Z6_SIGNAL_ENTRY_LATCH_MAX_KPH
                                            if signal_entry_latched else Z6_SIGNAL_ENTRY_MAX_KPH) and
                              (single_signal or signal_entry_latched))

    eps_lateral_status = int(getattr(CS, "eps_lateral_availability_status",
                                     1 if getattr(CS, "eps_lateral_available", True) else 0))
    eps_iacc_abort_reason = int(getattr(CS, "eps_iacc_abort_reason", 0))
    eps_ads_abort_feedback = int(getattr(CS, "eps_ads_abort_feedback", 0))
    eps_apa_abort_feedback = int(getattr(CS, "eps_apa_abort_feedback", 0))
    eps_degraded = eps_lateral_status == 2
    eps_fault = bool(getattr(CS.out, "steerFaultTemporary", False))
    eps_abort = (eps_iacc_abort_reason != 0 or eps_ads_abort_feedback != 0 or
                 eps_apa_abort_feedback != 0 or eps_degraded or eps_fault)
    eps_lateral_accepted = bool(getattr(CS, "eps_lateral_active", False))

    external_session_requested = bool(CC.enabled or lateral_requested or CS.out.cruiseState.available)
    if external_session_requested:
      self.iacc_session_hold_timer = CarControllerParams.IACC_SESSION_OFF_HOLD_FRAMES
    elif self.iacc_session_hold_timer > 0:
      self.iacc_session_hold_timer -= 1
    iacc_session_active = external_session_requested or self.iacc_session_hold_timer > 0

    # Match the supplied legacy Changan lateral behavior: while OP lateral is
    # requested and the driver is not taking over, keep the 0x1BA lateral
    # request asserted. EPS active feedback is logged/bridged, but it no longer
    # gates OP's own steering request after a manual takeover.
    lateral_control_requested = lateral_requested or signal_entry_requested
    # Keep the OEM lateral request active while the signal-only timer is
    # running. This is independent of the model's CC.latActive flag; otherwise
    # the planner can replace the signal cue before or immediately after it
    # fires.
    steer_active = lateral_control_requested and not driver_pause
    signal_entry_start = (signal_entry_requested and self.z6_signal_entry_count == 0 and
                          self.z6_signal_entry_timer == 0)
    if not iacc_session_active:
      self._set_iacc_eps_state(IaccEpsState.OFF)
    elif lateral_control_requested and driver_pause:
      self._set_iacc_eps_state(IaccEpsState.PAUSED)
    elif lateral_control_requested:
      self._set_iacc_eps_state(IaccEpsState.ACTIVE)
    else:
      self._set_iacc_eps_state(IaccEpsState.STANDBY)
    iacc_hwa_mode = self._iacc_hwa_mode(eps_lateral_accepted)

    # In longitudinal mode Panda blocks the stock camera's 0x31A before it
    # reaches the main bus.  The gateway/IP still expects that 10 Hz heartbeat
    # while IACC is disabled.  Only publishing it after an OP session starts
    # leaves a watchdog gap at boot, which is exactly the instrument fault the
    # car reports until IACC is enabled.  Therefore always replace 0x31A;
    # OFF is a valid, stock-like disabled frame (Enable=0, Mode=0), while all
    # unrelated camera-owned fields are retained by create_iacc_status().
    if self.frame % ACC_CRUISE_STEP == 0 or signal_entry_start:
      self.acc_iacc_counter = (self.acc_iacc_counter + 1) & 0xF
      can_sends.append(changancan.create_iacc_status(
        self.packer,
        0,
        self.acc_iacc_counter,
        iacc_session_active,
        iacc_hwa_mode,
        CS.out.steeringPressed,
        getattr(CS, "acc_iacc_status_values", None),
        driver_override_paused if iacc_session_active else False,
        actuators.curvature if iacc_session_active else None,
      ))

    if self.frame % CarControllerParams.STEER_STEP == 0:
      self.eps_lateral_status_counter = (self.eps_lateral_status_counter + 1) & 0xF
      can_sends.append(changancan.create_eps_lateral_control_status(
        self.packer,
        2,
        self.eps_lateral_status_counter,
        getattr(CS, "eps_lateral_status_values", None),
        iacc_session_active,
      ))

      if steer_active:
        if not self.steer_active_last:
          # A signal-only entry is the explicit pre-model cue. Do not hide its
          # first steering command behind the normal 1 s lateral settle ramp.
          self.eps_active_settle_timer = (0 if signal_entry_requested else
                                          CarControllerParams.EPS_ACTIVE_SETTLE_FRAMES)
        max_command_angle = min(CarControllerParams.ANGLE_LIMITS.STEER_ANGLE_MAX,
                                CarControllerParams.STEER_COMMAND_MAX_ANGLE)
        desired_angle = float(np.clip(actuators.steeringAngleDeg + CS.out.steeringAngleOffsetDeg,
                                      -max_command_angle,
                                      max_command_angle))
        actual_angle = float(np.clip(CS.out.steeringAngleDeg,
                                     -CarControllerParams.ANGLE_LIMITS.STEER_ANGLE_MAX,
                                     CarControllerParams.ANGLE_LIMITS.STEER_ANGLE_MAX))
        steering_rate = float(getattr(CS.out, "steeringRateDeg", 0.0))
        driver_opposing = bool(getattr(CS.out, "steeringPressed", False) and
                               ((left_blinker and steering_rate < -Z6_SIGNAL_ENTRY_OPPOSING_RATE) or
                                (right_blinker and steering_rate > Z6_SIGNAL_ENTRY_OPPOSING_RATE) or
                                (self.z6_mid_turn_direction > 0 and
                                 steering_rate < -Z6_MID_TURN_HOLD_OPPOSING_RATE) or
                                (self.z6_mid_turn_direction < 0 and
                                 steering_rate > Z6_MID_TURN_HOLD_OPPOSING_RATE)))
        model_angle = desired_angle
        signal_entry_was_active = self.z6_signal_entry_active
        desired_angle = self._apply_z6_signal_entry(
          desired_angle, actual_angle, steering_rate, CS.out.vEgo,
          steer_active, max_command_angle, left_blinker, right_blinker,
          driver_opposing,
        )
        # Signal entry has its own fixed same-side target.  Once it hands back
        # to the model, retain a bounded mid-turn floor if the corner is still
        # in progress and the model briefly asks for a straight line.
        if self.z6_signal_entry_active:
          self.z6_mid_turn_direction = 0
          self.z6_mid_turn_timer = 0
          self.z6_mid_turn_hold_angle = 0.0
        else:
          desired_angle = self._apply_z6_mid_turn_hold(
            desired_angle,
            actual_angle,
            v_ego_kph,
            steer_active,
            driver_opposing,
          )
        signal_direction = (1 if left_blinker and not right_blinker else
                            -1 if right_blinker and not left_blinker else 0)
        # Reuse the latched entry direction after a stalk self-cancel. Without
        # this, the helper selects +130/-130 but the minimum-angle clamp below
        # would multiply it by zero for the rest of that frame.
        if self.z6_signal_entry_active and signal_direction == 0:
          signal_direction = self.z6_signal_entry_direction
        reference_angle = actual_angle
        if bool(getattr(CS, "eps_steering_angle_ref_valid", False)):
          reference_angle = float(np.clip(getattr(CS, "eps_steering_angle_ref", actual_angle),
                                          actual_angle - CarControllerParams.EPS_REFERENCE_MAX_DELTA,
                                          actual_angle + CarControllerParams.EPS_REFERENCE_MAX_DELTA))
        if self.z6_signal_entry_active and not signal_entry_was_active:
          # The model may have left a large command on apply_angle_last while
          # the signal cue was not yet active. Start the explicit cue
          # from the measured/reference angle instead of walking through that
          # stale model command in the opposite direction.
          self.apply_angle_last = reference_angle
        v_ego_kph = CS.out.vEgo * CV.MS_TO_KPH
        # Keep the 0111 lateral target unchanged. The remaining window/rate
        # limits below only translate it into commands accepted by Z6 EPS.
        max_cmd_error = float(np.interp(v_ego_kph,
                                        CarControllerParams.EPS_CMD_ERROR_BP,
                                        CarControllerParams.EPS_CMD_ERROR_V))
        max_cmd_delta = float(np.interp(v_ego_kph,
                                        CarControllerParams.EPS_CMD_DELTA_BP,
                                        CarControllerParams.EPS_CMD_DELTA_V))
        if self.z6_signal_entry_active:
          # Signal entry is already bounded by speed, target angle, command
          # delta, driver takeover, and Panda safety. Use full authority for
          # this explicit cue so the wheel visibly starts turning immediately.
          max_cmd_error = max(max_cmd_error, Z6_SIGNAL_ENTRY_DIRECT_CMD_ERROR)
          self.eps_command_authority = 1.0
        elif self.eps_active_settle_timer > 0:
          max_cmd_error = min(max_cmd_error, CarControllerParams.EPS_ACTIVE_SETTLE_CMD_ERROR)
          max_cmd_delta = min(max_cmd_delta, CarControllerParams.EPS_ACTIVE_SETTLE_CMD_DELTA)
          authority_step = CarControllerParams.EPS_ACTIVE_SETTLE_AUTHORITY_STEP
          self.eps_active_settle_timer -= 1
        else:
          authority_step = CarControllerParams.EPS_AUTHORITY_STEP
        if not self.z6_signal_entry_active:
          self.eps_command_authority = min(1., self.eps_command_authority + authority_step)
        if self.z6_signal_entry_active:
          # Do not let the model angle re-enter the command after the helper
          # selected the signal target. Enforce a visible same-direction floor
          # even while the measured angle is still close to zero.
          desired_angle = signal_direction * max(
            Z6_SIGNAL_ENTRY_MIN_CMD_ANGLE,
            abs(desired_angle),
          )
          desired_angle = float(np.clip(desired_angle, -max_command_angle, max_command_angle))
        desired_error = float(np.clip(desired_angle - reference_angle,
                                      -max_cmd_error,
                                      max_cmd_error))
        if reference_angle * desired_error > 0. and abs(reference_angle) > CarControllerParams.EPS_HIGH_ANGLE_OUTWARD_START:
          max_outward_error = max(0.,
                                  CarControllerParams.EPS_DYNAMIC_WINDOW_DEFAULT -
                                  abs(reference_angle) -
                                  CarControllerParams.EPS_DYNAMIC_WINDOW_EDGE_MARGIN)
          desired_error = float(np.clip(desired_error, -max_outward_error, max_outward_error))
        desired_angle = reference_angle + self.eps_command_authority * desired_error
        if self.z6_signal_entry_active:
          # Never emit a command on the opposite side while the indicator cue
          # owns the corner. A stale opposite model/reference angle otherwise
          # makes the command cross zero and produces the observed wheel
          # chatter. Driver counter-steer has already been handled by the
          # takeover gate above.
          if signal_direction > 0:
            desired_angle = max(desired_angle, Z6_SIGNAL_ENTRY_MIN_CMD_ANGLE)
          elif signal_direction < 0:
            desired_angle = min(desired_angle, -Z6_SIGNAL_ENTRY_MIN_CMD_ANGLE)
        self.apply_angle_last = float(np.clip(desired_angle,
                                              self.apply_angle_last - max_cmd_delta,
                                              self.apply_angle_last + max_cmd_delta))
        if not self.z6_signal_entry_active and self.z6_mid_turn_direction != 0:
          # Re-apply the hold after EPS authority/rate shaping.  Otherwise the
          # model's brief straight-ahead dip can still leak into the final CAN
          # command while authority is below 1.0 and unwind the wheel.
          final_hold = self._apply_z6_mid_turn_hold(
            self.apply_angle_last,
            actual_angle,
            v_ego_kph,
            steer_active,
            driver_opposing,
            advance_timer=False,
          )
          if self.z6_mid_turn_direction > 0:
            floor = min(final_hold, self.apply_angle_last + max_cmd_delta)
            self.apply_angle_last = max(self.apply_angle_last, floor)
          elif self.z6_mid_turn_direction < 0:
            floor = max(final_hold, self.apply_angle_last - max_cmd_delta)
            self.apply_angle_last = min(self.apply_angle_last, floor)
        if self.z6_signal_entry_active:
          # Keep the actuator command itself on the selected side even when
          # apply_angle_last was inherited from a stale opposite-side model
          # target on the entry frame.
          if signal_direction > 0:
            self.apply_angle_last = max(self.apply_angle_last, 0.0)
          elif signal_direction < 0:
            self.apply_angle_last = min(self.apply_angle_last, 0.0)
        # Saturate the final command inside the EPS speed envelope. A deep
        # angle request therefore decays smoothly toward the budget instead of
        # tripping the EPS reject path and killing the IACC session. Only
        # clamp while active: the standby command already tracks the measured
        # angle and must never jump inward of it.
        envelope_max = self._eps_envelope_max_angle(CS.out.vEgo)
        if abs(self.apply_angle_last) > envelope_max:
          self.apply_angle_last = math.copysign(envelope_max, self.apply_angle_last)
      else:
        self.eps_active_settle_timer = 0
        self.eps_command_authority = 0.
        self.z6_mid_turn_direction = 0
        self.z6_mid_turn_timer = 0
        self.z6_mid_turn_hold_angle = 0.0
        self.z6_mid_turn_handoff = False
        self.z6_signal_entry_count = 0
        self.z6_signal_entry_timer = 0
        self.z6_signal_entry_direction = 0
        self.z6_signal_entry_angle = 0.0
        self.z6_signal_entry_active = False
        self.z6_signal_entry_handoff_count = 0
        self.z6_signal_entry_handed_off = False
        self.apply_angle_last = float(np.clip(CS.out.steeringAngleDeg,
                                              -CarControllerParams.ANGLE_LIMITS.STEER_ANGLE_MAX,
                                              CarControllerParams.ANGLE_LIMITS.STEER_ANGLE_MAX))
      can_sends.append(changancan.create_steering_control(
        self.packer,
        0,
        self.frame // CarControllerParams.STEER_STEP,
        self.apply_angle_last,
        steer_active,
      ))

    if self.CP.openpilotLongitudinalControl:
      long_active = bool(CC.enabled and CC.longActive)
      if long_active and not self.long_active_last:
        activation_speed_kph = CS.out.vEgo * CV.MS_TO_KPH
        activation_max_kph = getattr(CarControllerParams, "Z6_LOW_SPEED_BRAKE_KPH", 40.0)
        moving_low_speed_activation = (self.is_gas and not bool(CS.out.standstill) and
                                       0.1 < activation_speed_kph <= activation_max_kph)
        self.z6_long_activation_timer = (getattr(
          CarControllerParams, "Z6_LONG_ACTIVATION_HOLD_FRAMES", 75,
        ) if moving_low_speed_activation else 0)
        self.z6_crawl_activation_active = (moving_low_speed_activation and
                                           activation_speed_kph <= getattr(
                                             CarControllerParams,
                                             "Z6_CRAWL_ACTIVATION_MAX_KPH", 20.0))
        if self.z6_crawl_activation_active:
          self.z6_long_activation_timer = max(
            self.z6_long_activation_timer,
            getattr(CarControllerParams, "Z6_CRAWL_ACTIVATION_HOLD_FRAMES", 200),
          )
        self.z6_long_activation_speed_kph = activation_speed_kph if moving_low_speed_activation else 0.0
        # Never carry a positive command from a previous active session into
        # a new low-speed activation. Preserve a negative request so a
        # braking vehicle cannot roll forward while the controller re-arms.
        if self.accel_last > 0.0:
          self.accel_last = 0.0
      elif not long_active:
        self.z6_long_activation_timer = 0
        self.z6_long_activation_speed_kph = 0.0
        self.z6_crawl_activation_active = False
        self.accel_last = 0.0
        self.acc_decel_active = False

      if self.frame % CarControllerParams.ACC_CONTROL_STEP == 0:
        self.acc_status_counter = (self.acc_status_counter + 1) & 0xF
        # Preserve the original 0111 longitudinal output. Changan code only
        # translates it to GW_244 and does not add curve coasting/exit boost.
        accel_cmd = actuators.accel if CC.enabled else 0.
        if not long_active:
          # Do not carry a stale brake request across a disabled longitudinal
          # session or into the next activation.
          accel_cmd = 0.0
        if getattr(CS, "changan_standstill_activation", False):
          # Main at standstill arms IACC without an unintended drive-off.
          # A deliberate RES+ press in CarState releases this guard.
          accel_cmd = min(accel_cmd, 0.0)
        v_ego_kph = CS.out.vEgo * CV.MS_TO_KPH
        if long_active:
          standstill = bool(CS.out.standstill)
          accel_cmd = self._apply_z6_crawl_activation_stability(
            accel_cmd,
            v_ego_kph,
            standstill,
            bool(getattr(CS.out, "brakePressed", False)),
          )
          accel_cmd = self._apply_z6_low_speed_brake_smoothing(accel_cmd, v_ego_kph, standstill)
          accel_cmd = self._apply_z6_low_speed_longitudinal_smoothing(accel_cmd, v_ego_kph, standstill)
          driver_brake = bool(getattr(CS.out, "brakePressed", False))
          low_speed_limit = getattr(CarControllerParams, "Z6_LOW_SPEED_BRAKE_KPH", 40.0)
          rapid_trigger = (getattr(CarControllerParams,
                                   "Z6_RAPID_BRAKE_LOW_SPEED_TRIGGER_ACCEL", -0.50)
                           if v_ego_kph <= low_speed_limit else
                           getattr(CarControllerParams,
                                   "Z6_RAPID_BRAKE_TRIGGER_ACCEL", -1.50))
          # Classify from the planner's original request.  Running the
          # ordinary comfort filter first would turn a strong cut-in request
          # into a small -0.05 step and hide its urgency from the rapid ramp.
          rapid_request = (self.is_gas and not standstill and not driver_brake and
                           accel_cmd <= rapid_trigger)
          if rapid_request:
            accel_cmd = self._apply_z6_rapid_brake_smoothing(
              accel_cmd,
              v_ego_kph,
              standstill,
              driver_brake,
            )
          else:
            accel_cmd = self._apply_z6_follow_brake_smoothing(
              accel_cmd,
              v_ego_kph,
              standstill,
              driver_brake,
            )
          try:
            set_speed_kph = float(CC.hudControl.setSpeed) * CV.MS_TO_KPH
          except Exception:
            set_speed_kph = 0.0
          accel_cmd = self._apply_z6_low_speed_drive_smoothing(
            accel_cmd, v_ego_kph, set_speed_kph, standstill)
        else:
          self.z6_low_speed_drive_filtered = 0.0
        self.accel_last = float(np.clip(accel_cmd,
                                        CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
        if self.z6_long_activation_timer > 0:
          self.z6_long_activation_timer -= 1
        if not long_active:
          self.acc_decel_active = False
        else:
          if (self.z6_crawl_activation_active and self.z6_long_activation_timer > 0 and
              not bool(CS.out.standstill)):
            # During a sub-20 km/h activation, keep CDD latched through a
            # wider release band.  This prevents a tiny positive planner
            # sample from switching immediately back to drive torque.
            cdd_enter = getattr(CarControllerParams,
                                "Z6_CRAWL_ACTIVATION_BRAKE_ENTER", -0.18)
            cdd_exit = getattr(CarControllerParams,
                               "Z6_CRAWL_ACTIVATION_CDD_EXIT", 0.12)
          elif self.is_gas and v_ego_kph <= getattr(CarControllerParams, "Z6_LOW_SPEED_BRAKE_KPH", 40.0):
            cdd_enter = getattr(CarControllerParams, "Z6_LOW_SPEED_CDD_ENTER", self.accel_cdd_enter)
            cdd_exit = getattr(CarControllerParams, "Z6_LOW_SPEED_CDD_EXIT", self.accel_cdd_exit)
          else:
            cdd_enter = self.accel_cdd_enter
            cdd_exit = self.accel_cdd_exit
          if self.acc_decel_active:
            self.acc_decel_active = self.accel_last <= cdd_exit
          else:
            self.acc_decel_active = self.accel_last <= cdd_enter

        drive_torque_active = long_active and not self.acc_decel_active
        if drive_torque_active and self.z6_long_activation_timer > 0:
          speed_margin = getattr(CarControllerParams, "Z6_LONG_ACTIVATION_SPEED_MARGIN_KPH", 0.5)
          drive_torque_active = v_ego_kph < self.z6_long_activation_speed_kph - speed_margin

        can_sends.append(changancan.create_longitudinal_control(
          self.packer,
          0,
          self.acc_status_counter,
          self.accel_last,
          CC.enabled,
          CC.enabled and CC.longActive,
          self.acc_decel_active,
          getattr(CS, "acc_status_values", None),
          CS.out.standstill,
          drive_torque_active=drive_torque_active,
        ))

      if self.frame % ACC_CRUISE_STEP == 0:
        self.acc_cruise_counter = (self.acc_cruise_counter + 1) & 0xF
        stock_cruise_values = dict(getattr(CS, "acc_cruise_status_values", {}) or {})
        # Start the Z6 session one step closer than the stock third gap, then
        # leave subsequent values untouched so the physical GAP button can
        # still select any other distance.
        if self.is_gas and not self.z6_gap_initialized:
          stock_cruise_values["ACC_TimeGapSet"] = 2
          self.z6_gap_initialized = True
        stock_set_speed = stock_cruise_values.get("ACC_SetSpeed", 0.)
        if CC.enabled:
          # The set speed remains entirely owned by the original 0111 planner.
          set_speed_kph = max(1., CC.hudControl.setSpeed * CV.MS_TO_KPH)
        elif CS.out.cruiseState.available:
          # IACC main standby displays actual speed, including zero, instead of
          # fabricating a 30 km/h target before controls have enabled.
          set_speed_kph = max(0., CS.out.vEgo * CV.MS_TO_KPH)
        else:
          set_speed_kph = max(30., stock_set_speed)
        can_sends.append(changancan.create_cruise_status(
          self.packer,
          0,
          self.acc_cruise_counter,
          set_speed_kph,
          stock_cruise_values,
          CC.hudControl.leadVisible,
          CC.hudControl.leadDistanceBars,
        ))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    new_actuators.accel = self.accel_last if self.CP.openpilotLongitudinalControl else 0.

    self.steer_active_last = steer_active
    self.long_active_last = bool(CC.enabled and CC.longActive)
    self.eps_lateral_accepted_last = eps_lateral_accepted
    self.frame += 1
    return new_actuators, can_sends
