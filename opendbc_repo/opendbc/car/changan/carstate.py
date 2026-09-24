from opendbc.can import CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.changan.values import DBC, GEAR_MAP, Buttons, CarControllerParams, ChanganFlags
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase

ButtonType = structs.CarState.ButtonEvent.Type

BUTTONS_DICT = {
  Buttons.RES_ACCEL: ButtonType.accelCruise,
  Buttons.SET_DECEL: ButtonType.decelCruise,
  Buttons.CANCEL: ButtonType.cancel,
  Buttons.GAP_DIST: ButtonType.gapAdjustCruise,
  Buttons.IACC: ButtonType.mainCruise,
}

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.is_gas = bool(CP.flags & ChanganFlags.GAS)
    self.has_known_gear_msg = not bool(CP.flags & ChanganFlags.NO_KNOWN_GEAR_MSG)
    self.blindspot_2a4_frames = 0
    self.blindspot_2ad_frames = 0
    self.eps_lateral_available = False
    self.eps_lateral_availability_status = 0
    self.eps_lateral_active = False
    self.eps_ads_abort_feedback = 0
    self.eps_apa_abort_feedback = 0
    self.eps_iacc_abort_reason = 0
    self.prev_cruise_buttons = Buttons.NONE
    self.acc_status_values = {}
    self.acc_cruise_status_values = {}
    self.acc_iacc_status_values = {}
    self.eps_lateral_status_values = {}
    self.op_cruise_available = False
    # At standstill, opening IACC main only arms the 0111 controller. Require a
    # deliberate RES+ press before allowing drive-off torque.
    self.changan_standstill_activation = False
    self.changan_steering_pressed = False
    self.eps_torsion_bar_torque = 0.
    self.eps_actual_motor_torque = 0.
    self.eps_actual_torsion_bar_torque = 0.
    self.eps_max_safety_torsion_bar_torque = 0.
    self.eps_min_safety_torsion_bar_torque = 0.
    self.eps_fault_state = 0
    self.eps_steering_angle_ref = 0.
    self.eps_steering_angle_ref_valid = False
    self.eps_steering_angle_ref_frames = 0

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()
    speed_msg = "GW_187" if self.is_gas else "GW_17A"
    pedal_msg = "GW_196" if self.is_gas else "GW_1A6"
    real_pedal_msg = pedal_msg if self.is_gas else "GW_1C6"

    raw_speed_kph = cp.vl[speed_msg]["ESP_VehicleSpeed"]
    cluster_speed_kph = raw_speed_kph if raw_speed_kph <= 5. else (raw_speed_kph / 0.98) + 2.
    ret.vEgoRaw = raw_speed_kph * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.vEgoCluster = cluster_speed_kph * CV.KPH_TO_MS
    ret.vCluRatio = 1.0
    ret.standstill = ret.vEgoRaw < 0.01

    ret.wheelSpeeds = self.get_wheel_speeds(raw_speed_kph, raw_speed_kph, raw_speed_kph, raw_speed_kph)

    tpms = cp.vl["GW_347"]
    if tpms["TPMS_SystemFailureWarning"] == 0 and tpms["TPMS_SignalStatus"] == 0:
      ret.tpms.fl = tpms["TPMS_LFTyrePressure"] / 100.
      ret.tpms.fr = tpms["TPMS_RFTyrePressure"] / 100.
      ret.tpms.rl = tpms["TPMS_LRTyrePressure"] / 100.
      ret.tpms.rr = tpms["TPMS_RRTyrePressure"] / 100.

    ret.gas = cp.vl[real_pedal_msg]["EMS_RealAccPedal"] / 100.
    ret.gasPressed = ret.gas > 0.01

    ret.brakePressed = cp.vl[pedal_msg]["EMS_BrakePedalStatus"] != 0
    ret.brakeLights = ret.brakePressed or cp.vl["GW_20B"]["ESP_BrakeLightOnRequest"] != 0

    ret.steeringAngleDeg = cp.vl["GW_180"]["SAS_SteeringAngle"]
    ret.steeringRateDeg = cp.vl["GW_180"]["SAS_SteeringAngleSpeed"]
    eps_status = cp.vl["GW_24F"]
    eps_lateral_status = cp.vl["GW_17E"]
    eps_torsion_status = cp.vl["GW_170"]

    self.eps_steering_angle_ref_valid = False

    ret.steeringTorque = eps_status["EPS_SteeringTorque"]
    self.eps_torsion_bar_torque = eps_lateral_status["EPS_MeasuredTorsionBarTorque"]
    steering_pressed_enter = CarControllerParams.STEER_OVERRIDE_TAKEOVER_THRESHOLD_GAS if self.is_gas else \
      CarControllerParams.STEER_OVERRIDE_TAKEOVER_THRESHOLD_IDD
    if self.changan_steering_pressed:
      if abs(ret.steeringTorque) < CarControllerParams.STEER_OVERRIDE_RELEASE_THRESHOLD and \
         abs(self.eps_torsion_bar_torque) < CarControllerParams.STEER_OVERRIDE_RELEASE_TORSION_THRESHOLD and \
         abs(ret.steeringAngleDeg) < CarControllerParams.STEER_OVERRIDE_RELEASE_ANGLE:
        self.changan_steering_pressed = False
    elif abs(ret.steeringTorque) > steering_pressed_enter or \
         abs(self.eps_torsion_bar_torque) > CarControllerParams.STEER_OVERRIDE_TAKEOVER_TORSION_THRESHOLD:
      self.changan_steering_pressed = True
    ret.steeringPressed = self.changan_steering_pressed
    # Z6 iDD logs show this validity bit stays 0 even while steering angle/rate are live and sane.
    # CANParser freshness still feeds ret.canValid in CarInterfaceBase, so don't block engagement here.
    ret.vehicleSensorsInvalid = False

    self.eps_lateral_availability_status = int(eps_lateral_status["EPS_LatCtrlAvailabilityStatus"])
    self.eps_lateral_available = self.eps_lateral_availability_status == 1
    self.eps_lateral_active = eps_lateral_status["EPS_LatCtrlActive"] == 1
    self.eps_ads_abort_feedback = int(eps_lateral_status["EPS_ADS_Abortfeedback"])
    self.eps_apa_abort_feedback = int(eps_status["EPS_APA_Abortfeedback"])
    self.eps_iacc_abort_reason = int(eps_status["EPS_IACC_abortreason"])

    # The EPS firmware uses availability=2 and the three abort feedback paths
    # as IACC reject/degraded states even when EPS_EPSFailed remains clear.
    eps_iacc_reject = (self.eps_lateral_availability_status == 2 or
                       self.eps_ads_abort_feedback != 0 or
                       self.eps_apa_abort_feedback != 0 or
                       self.eps_iacc_abort_reason != 0)
    ret.steerFaultTemporary = eps_status["EPS_EPSFailed"] != 0 or eps_iacc_reject
    self.eps_actual_motor_torque = eps_torsion_status["EPS_ActualMotorTorq"]
    self.eps_actual_torsion_bar_torque = eps_torsion_status["EPS_ActualTorsionBarTorq"]
    self.eps_max_safety_torsion_bar_torque = eps_torsion_status["EPS_MaxSafetyTorsionBarTorq"]
    self.eps_min_safety_torsion_bar_torque = eps_torsion_status["EPS_MinSafetyTorsionBarTorq"]
    self.eps_fault_state = int(eps_torsion_status["EPS_fault_state"])
    self.eps_lateral_status_values = dict(eps_lateral_status)

    self.acc_status_values = dict(cp_cam.vl["GW_244"])
    self.acc_cruise_status_values = dict(cp_cam.vl["GW_307"])
    self.acc_iacc_status_values = dict(cp_cam.vl["GW_31A"])
    acc_mode = int(self.acc_status_values["ACC_ACCMode"])
    iacc_enabled = cp_cam.vl["GW_31A"]["ACC_IACCHWAEnable"] == 1
    acc_set_speed = self.acc_cruise_status_values["ACC_SetSpeed"] * CV.KPH_TO_MS
    if self.CP.openpilotLongitudinalControl:
      ret.cruiseState.available = True
      ret.cruiseState.enabled = False
      ret.cruiseState.speed = acc_set_speed
      ret.cruiseState.speedCluster = acc_set_speed
    else:
      ret.cruiseState.available = acc_mode >= 2 or iacc_enabled
      ret.cruiseState.enabled = acc_mode in (3, 5)
      ret.cruiseState.speed = acc_set_speed
      ret.cruiseState.speedCluster = acc_set_speed
    ret.cruiseState.standstill = ret.standstill and ret.cruiseState.enabled
    ret.pcmCruiseGap = int(self.acc_cruise_status_values["ACC_TimeGapSet"])

    ret.leftBlinker = cp.vl["GW_28B"]["BCM_TurnIndicatorLeft"] != 0
    ret.rightBlinker = cp.vl["GW_28B"]["BCM_TurnIndicatorRight"] != 0

    if len(cp.vl_all["GW_2A4"]["LCDAR_BSD_LCAAlert"]) > 0:
      self.blindspot_2a4_frames = 20
    elif self.blindspot_2a4_frames > 0:
      self.blindspot_2a4_frames -= 1

    if len(cp.vl_all["GW_2AD"]["LCDAL_SEAAlert"]) > 0:
      self.blindspot_2ad_frames = 20
    elif self.blindspot_2ad_frames > 0:
      self.blindspot_2ad_frames -= 1

    has_2a4 = self.blindspot_2a4_frames > 0
    has_2ad = self.blindspot_2ad_frames > 0
    ret.leftBlindspot = (has_2a4 and cp.vl["GW_2A4"]["LCDAR_Left_BSD_LCAAlert"] != 0) or \
                        (has_2ad and cp.vl["GW_2AD"]["LCDAL_SEAAlert"] != 0)
    ret.rightBlindspot = (has_2a4 and cp.vl["GW_2A4"]["LCDAR_BSD_LCAAlert"] != 0) or \
                         (has_2ad and cp.vl["GW_2AD"]["LCDAL_Right_SEAAlert"] != 0)

    ret.doorOpen = bool(cp.vl["GW_28B"]["BCM_DriverDoorStatus"] or
                        cp.vl["GW_298"]["BCM_LeftRearDoorStatus"] or
                        cp.vl["BDC_2D1"]["BCM_PassengerDoorStatus"] or
                        cp.vl["BDC_2D1"]["BCM_RightRearDoorStatus"] or
                        cp.vl["BDC_2D1"]["BCM_TrunkDoorStatus"])
    ret.seatbeltUnlatched = cp.vl["GW_50"]["SRS_DriverBuckleSwitchStatus"] == 1
    ret.parkingBrake = cp.vl["IP_28A"]["IP_HandBrakeSwitch"] != 0

    if self.has_known_gear_msg:
      gear = int(cp.vl["GW_338"]["TCU_GearForDisplay"])
      ret.gearShifter = GEAR_MAP.get(gear, structs.CarState.GearShifter.unknown)
    else:
      # The supplied UNI-V captures do not contain the Z6 0x338 gear frame.
      # Keep the state unknown instead of fabricating a drive/reverse value.
      ret.gearShifter = structs.CarState.GearShifter.unknown

    self.prev_cruise_buttons = self.cruise_buttons
    buttons = cp.vl["GW_28C"]
    # Z6's ERS fields are latched switch states rather than the momentary
    # rocker contacts.  Treating ERS != 0 as a held button prevents the state
    # from returning to NONE, so only the first RES+/SET- edge is emitted.
    accel_button_pressed = buttons["GW_MFS_RESPlus_switch_signal"] == 1
    decel_button_pressed = buttons["GW_MFS_SETReduce_switch_signal"] == 1
    if accel_button_pressed:
      self.cruise_buttons = Buttons.RES_ACCEL
    elif decel_button_pressed:
      self.cruise_buttons = Buttons.SET_DECEL
    elif buttons["GW_MFS_Cancle_switch_signal"] != 0:
      self.cruise_buttons = Buttons.CANCEL
    elif buttons["GW_MFS_DIST_switch_signal"] != 0:
      self.cruise_buttons = Buttons.GAP_DIST
    elif buttons["GW_MFS_IACCenable_switch_signal"] != 0 or buttons["GW_MFS_Crusie_switch_signal"] != 0:
      self.cruise_buttons = Buttons.IACC
    else:
      self.cruise_buttons = Buttons.NONE
    # Keep a normal list until the synthetic second-IACC cancel has been added.
    button_events = create_button_events(self.cruise_buttons, self.prev_cruise_buttons, BUTTONS_DICT)

    if self.CP.openpilotLongitudinalControl:
      main_released = any(b.type == ButtonType.mainCruise and not b.pressed for b in button_events)
      set_released = any(b.type in (ButtonType.accelCruise, ButtonType.decelCruise) and not b.pressed for b in button_events)
      cancel_pressed = any(b.type == ButtonType.cancel and b.pressed for b in button_events)

      if ret.brakePressed or cancel_pressed:
        self.op_cruise_available = False
        self.changan_standstill_activation = False
        ret.activateCruise = -1 if cancel_pressed else 0
      elif main_released:
        if self.op_cruise_available:
          self.op_cruise_available = False
          self.changan_standstill_activation = False
          ret.activateCruise = -1
          button_events.append(structs.CarState.ButtonEvent(pressed=True, type=ButtonType.cancel))
        else:
          self.op_cruise_available = True
          self.changan_standstill_activation = ret.standstill
          ret.activateCruise = 1
      elif set_released:
        if not self.op_cruise_available:
          self.changan_standstill_activation = ret.standstill
        self.op_cruise_available = True

      # RES+ is the explicit drive-off request after a standstill main-button
      # activation. Moving engagements use unmodified 0111 longitudinal logic.
      if accel_button_pressed:
        self.changan_standstill_activation = False

      ret.cruiseState.available = self.op_cruise_available

    ret.buttonEvents = button_events
    ret.stockAeb = cp_cam.vl["GW_244"]["ACC_AEBActive"] != 0
    ret.stockFcw = cp_cam.vl["GW_244"]["ACC_FCWPreWarning"] != 0

    return ret

  def update_button_enable(self, buttonEvents):
    if not self.CP.pcmCruise and self.op_cruise_available:
      for b in buttonEvents:
        if b.type in (ButtonType.mainCruise, ButtonType.accelCruise, ButtonType.decelCruise) and not b.pressed:
          return True
    return False

  @staticmethod
  def get_can_parsers(CP):
    optional = float("nan")
    is_gas = bool(CP.flags & ChanganFlags.GAS)
    speed_msg = "GW_187" if is_gas else "GW_17A"
    pedal_msg = "GW_196" if is_gas else "GW_1A6"
    real_pedal_msg = pedal_msg if is_gas else "GW_1C6"

    messages = [
      (speed_msg, 50),
      ("GW_17E", 100),
      ("GW_170", optional),
      ("GW_180", 100),
      (pedal_msg, 50),
      ("GW_24F", 50),
      ("GW_50", optional),
      ("GW_20B", optional),
      ("GW_28B", optional),
      ("GW_298", optional),
      ("BDC_2D1", optional),
      ("IP_28A", optional),
      ("GW_347", optional),
      ("GW_2A4", optional),
      ("GW_2AD", optional),
      ("GW_28C", optional),
    ]
    if not (CP.flags & ChanganFlags.NO_KNOWN_GEAR_MSG):
      messages.append(("GW_338", optional))
    if real_pedal_msg != pedal_msg:
      messages.append((real_pedal_msg, 50))

    cam_messages = [
      ("GW_244", 50),
      ("GW_307", 10),
      ("GW_31A", 10),
      # Front radar/ACC target slots and the static-obstacle bank are decoded
      # by RadarInterface.  They are optional across Z6 firmware revisions:
      # keeping them out of the required camera-parser set prevents a missing
      # object bank from invalidating carState/CAN engagement.  RadarInterface
      # still subscribes to and parses every frame when it is present.
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], messages, 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, 2),
    }
