import numpy as np

from opendbc.car.crc import CRC8J1850
from opendbc.car.changan.values import CarControllerParams

ACCEL_TO_TRQ_BP = [-3.5, -0.25, -0.15, -0.10, -0.05, 0.0, 0.10, 0.20, 0.30, 0.35, 0.60, 0.80, 1.00, 1.30, 1.60, 2.00, 2.50]
ACCEL_TO_TRQ_V = [-5000., -5000., -5000., -4595., -4565., -4450., -4250., -3930., -3700., -3600., -3450., -3300., -3150., -2900., -2650., -2350., -2050.]


def changan_checksum(data: bytes) -> int:
  crc = 0xFF
  for byte in data:
    crc = CRC8J1850[crc ^ byte]
  return crc ^ 0xFF


def create_steering_control(packer, bus, frame, apply_angle, lat_active):
  values = {
    "ACC_MotorTorqueMaxLimitRequest": CarControllerParams.STEER_TORQUE_LIMIT,
    "ACC_MotorTorqueMinLimitRequest": -CarControllerParams.STEER_TORQUE_LIMIT,
    "EPS_AngleCmd": apply_angle,
    "EPS_LatCtrlActive": lat_active,
    "ACC_RollingCounter_1BA": frame % 0x10,
    "ACC_CRCCheck_1BA": 0,
  }

  dat = packer.make_can_msg("GW_1BA", bus, values)[1]
  values["ACC_CRCCheck_1BA"] = changan_checksum(dat[:7])
  return packer.make_can_msg("GW_1BA", bus, values)


def create_eps_lateral_control_status(packer, bus, frame, stock_values=None, session_active=False):
  values = dict(stock_values or {})
  defaults = {
    "EPS_MeasuredTorsionBarTorque": 0.,
    "EPS_ADS_Abortfeedback": 0,
    "EPS_Pinionang": 0.,
    "EPS_Pinionang_Valid": 0,
    "EPS_Handwheel_Relang": 0.,
    "EPS_MeasuredTorsionBarTorqValid": 0,
    "EPS_RollingCounter_17E": frame % 0x10,
    "EPS_LatCtrlAvailabilityStatus": 0,
    "EPS_LatCtrlActive": 0,
    "EPS_Handwheel_Relang_Valid": 0,
    "EPS_CRCCheck_17E": 0,
  }
  for signal, value in defaults.items():
    values.setdefault(signal, value)

  if session_active:
    values["EPS_MeasuredTorsionBarTorque"] = float(np.clip(
      float(values["EPS_MeasuredTorsionBarTorque"]) + 1.,
      -20.48,
      20.46,
    ))
  values["EPS_RollingCounter_17E"] = frame % 0x10
  values["EPS_CRCCheck_17E"] = 0

  dat = packer.make_can_msg("GW_17E", 0, values)[1]
  values["EPS_CRCCheck_17E"] = changan_checksum(dat[:7])
  return packer.make_can_msg("GW_17E", bus, values)


def accel_to_trq_req(accel):
  trq_req = np.interp(accel, ACCEL_TO_TRQ_BP, ACCEL_TO_TRQ_V)
  return float(np.clip(trq_req, CarControllerParams.ACCEL_TRQ_MIN, CarControllerParams.ACCEL_TRQ_MAX))


def create_longitudinal_control(packer, bus, frame, accel, enabled, long_active, cdd_active,
                                stock_values=None, standstill=False, drive_torque_active=None):
  accel = float(np.clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
  accel_torque = accel_to_trq_req(accel)
  decel_active = long_active and cdd_active
  trq_active = long_active and not decel_active if drive_torque_active is None else bool(drive_torque_active)

  values = dict(stock_values or {})
  defaults = {
    "ACC_ACCTargetAcceleration": 0.,
    "ACC_LDWStatus": 0,
    "ACC_TextInfoForDriver": 0,
    "ACC_DecToStop": 0,
    "ACC_TIAN_1": 1,
    "ACC_CDDActive": 0,
    "ACC_ACCMode": 3 if enabled else 2,
    "ACC_Driveoff_Request": 0,
    "ACC_AEBTargetDeceleration": -0.0005,
    "ACC_AEBActive": 0,
    "ACC_AccTrqReq": -5000.,
    "ACC_TIAN_2": 1,
    "ACC_FCWPreWarning": 0,
    "ACC_FCWLatentWarning": 0,
    "ACC_AWBActive": 0,
    "ACC_AEBCtrlType": 0,
    "ACC_AccTrqReqActive": False,
    "ACC_LatTakeoverReq": 0,
    "ACC_LngTakeOverReq": 0,
    "ACC_HandsOnReq": 0,
    "ACC_RollingCounter_24E": frame % 0x10,
    "ACC_CRCCheck_24E": 0,
    "ACC_RollingCounter_25E": frame % 0x10,
    "ACC_CRCCheck_25E": 0,
  }
  for signal, value in defaults.items():
    values.setdefault(signal, value)

  if enabled:
    values["ACC_ACCTargetAcceleration"] = accel if long_active else 0.
    values["ACC_CDDActive"] = decel_active
    values["ACC_ACCMode"] = 3
    values["ACC_Driveoff_Request"] = long_active and standstill and accel > 0.1
    values["ACC_DecToStop"] = decel_active and standstill
    values["ACC_AEBTargetDeceleration"] = -0.0005
    values["ACC_AEBActive"] = 0
    values["ACC_AccTrqReq"] = accel_torque if trq_active else -5000.
    values["ACC_AccTrqReqActive"] = trq_active
    values["ACC_TIAN_1"] = 1
    values["ACC_TIAN_2"] = 1
    values["ACC_LDWStatus"] = 0
    values["ACC_TextInfoForDriver"] = 0
    values["ACC_FCWPreWarning"] = 0
    values["ACC_FCWLatentWarning"] = 0
    values["ACC_AWBActive"] = 0
    values["ACC_AEBCtrlType"] = 0
    values["ACC_LatTakeoverReq"] = 0
    values["ACC_LngTakeOverReq"] = 0
    values["ACC_HandsOnReq"] = 0
  else:
    values["ACC_ACCTargetAcceleration"] = 0.
    values["ACC_CDDActive"] = 0
    values["ACC_ACCMode"] = 2
    values["ACC_Driveoff_Request"] = 0
    values["ACC_DecToStop"] = 0
    values["ACC_AccTrqReq"] = -5000.
    values["ACC_AccTrqReqActive"] = False

  values["ACC_RollingCounter_24E"] = frame % 0x10
  values["ACC_CRCCheck_24E"] = 0
  values["ACC_RollingCounter_25E"] = frame % 0x10
  values["ACC_CRCCheck_25E"] = 0
  dat = packer.make_can_msg("GW_244", bus, values)[1]
  values["ACC_CRCCheck_24E"] = changan_checksum(dat[:7])
  values["ACC_CRCCheck_25E"] = changan_checksum(dat[8:15])
  return packer.make_can_msg("GW_244", bus, values)


def create_cruise_status(packer, bus, frame, set_speed_kph, stock_values=None, lead_visible=False, lead_distance_bars=0):
  values = dict(stock_values or {})
  set_speed = int(round(np.clip(set_speed_kph, 0., 254.)))

  defaults = {
    "ACC_SetSpeed": set_speed,
    "ACC_ObjValid": 0,
    # Z6 starts at the next-closer second gap.  Later frames preserve the
    # vehicle's current value so the physical GAP button remains usable.
    "ACC_TimeGapSet": 2,
    "ACC_DistanceLevel": 0,
    "ACC_AEBEnable": 0,
    "ACC_FCWSettingStatus": 0,
    "ACC_ACCTargetRelSpd": 0,
    "ACC_FRadarCalibrationStatus": 0,
    "ACC_VehicleStartRemindSts": 0,
    "ACC_IACCProhibitionTime": 0,
    "ACC_CSLSetReq": 0,
    "ACC_CSLAEnableStatus": 1,
    "ACC_RollingCounter_35E": frame % 0x10,
    "ACC_CRCCheck_35E": 0,
    "ACC_RollingCounter_322": frame % 0x10,
    "ACC_CRCCheck_322": 0,
    "ACC_RollingCounter_344": frame % 0x10,
    "ACC_CRCCheck_344": 0,
    "ACC_RollingCounter_35F": frame % 0x10,
    "ACC_CRCCheck_35F": 0,
  }
  for signal, value in defaults.items():
    values.setdefault(signal, value)

  values["ACC_SetSpeed"] = set_speed
  values["ACC_TimeGapSet"] = int(np.clip(values.get("ACC_TimeGapSet", 2), 0, 7))
  values["ACC_DistanceLevel"] = 0
  values["ACC_AEBEnable"] = 1
  values["ACC_FCWSettingStatus"] = 0
  values["ACC_VehicleStartRemindSts"] = 1
  values["ACC_CSLAEnableStatus"] = 1
  values["ACC_RollingCounter_35E"] = frame % 0x10
  values["ACC_CRCCheck_35E"] = 0
  values["ACC_RollingCounter_322"] = frame % 0x10
  values["ACC_CRCCheck_322"] = 0
  values["ACC_RollingCounter_344"] = frame % 0x10
  values["ACC_CRCCheck_344"] = 0
  values["ACC_RollingCounter_35F"] = frame % 0x10
  values["ACC_CRCCheck_35F"] = 0

  dat = packer.make_can_msg("GW_307", bus, values)[1]
  values["ACC_CRCCheck_35E"] = changan_checksum(dat[:7])
  values["ACC_CRCCheck_322"] = changan_checksum(dat[8:15])
  values["ACC_CRCCheck_344"] = changan_checksum(dat[16:23])
  values["ACC_CRCCheck_35F"] = changan_checksum(dat[24:31])
  return packer.make_can_msg("GW_307", bus, values)


def create_iacc_status(packer, bus, frame, session_active, hwa_mode, steering_pressed, stock_values=None,
                       driver_override=False, hwa_curvature=None):
  values = dict(stock_values or {})
  defaults = {
    "ACC_AEBTargetLngRange": 0.,
    "ACC_AEBTargetRelSpeed": 0,
    "ACC_AEBTargetLatRange": 0.,
    "ACC_ELKAlert": 0,
    "ACC_AEBTargetmode": 3,
    "ACC_AEBTextInfo": 0,
    "ACC_AEBStatus": 1,
    "ACC_Voiceinfo": 0,
    "ACC_FRadarFailureStatus": 0,
    "NEW_SIGNAL_1": 1,
    "NEW_SIGNAL_2": 33,
    "ACC_IACCHWAEnable": 0,
    "ACC_HostLaneLeftStatus": 0,
    "ACC_HostLaneRightStatus": 0,
    "ACC_RLaneMarkerType": 1,
    "ACC_LLaneMarkerType": 0,
    "ACC_LaneChangeStatus": 0,
    "ACC_IACCHWAMode": 0,
    "ACC_DriverHandsOffStatus": 1,
    "ACC_IACCHWATextInfoForDriver": 0,
    "ACC_TargetBasedLateralControl": 0,
    "ACC_LLLaneDetection": 0,
    "ACC_RRLaneDetection": 0,
    "ACC_HighBeamControl": 0,
    "ACC_ELKInterventionMode": 0,
    "ACC_ELKMode": 0,
    "ACC_ELKEnableStatus": 0,
    "ACC_LatPathDY": 0.,
    "ACC_LatPathHeadingAngle": 0.,
    "ACC_RollingCounter_36D": frame % 0x10,
    "ACC_CRCCheck_36D": 0,
    "ACC_RollingCounter_30A": frame % 0x10,
    "ACC_CRCCheck_30A": 0,
    "ACC_RollingCounter_30D": frame % 0x10,
    "ACC_CRCCheck_30D": 0,
    "ACC_RollingCounter_367": frame % 0x10,
    "ACC_CRCCheck_367": 0,
  }
  for signal, value in defaults.items():
    values.setdefault(signal, value)

  try:
    stock_target_based_lateral = int(values.get("ACC_TargetBasedLateralControl", 0))
  except (TypeError, ValueError, OverflowError):
    stock_target_based_lateral = 0
  try:
    stock_hands_off = int(values.get("ACC_DriverHandsOffStatus", 1))
  except (TypeError, ValueError, OverflowError):
    stock_hands_off = 1
  try:
    stock_new_signal_1 = int(values.get("NEW_SIGNAL_1", 1))
  except (TypeError, ValueError, OverflowError):
    stock_new_signal_1 = 1

  if stock_new_signal_1 not in range(0x10):
    stock_new_signal_1 = 1
  hwa_mode = int(np.clip(hwa_mode, 0, 4))
  hwa_active = session_active and hwa_mode == 3

  if session_active:
    values["ACC_AEBStatus"] = 1
    values["ACC_AEBTargetmode"] = 3
    values["NEW_SIGNAL_1"] = stock_new_signal_1
    values["NEW_SIGNAL_2"] = 33
    values["ACC_LaneChangeStatus"] = 0
    values["ACC_IACCHWAEnable"] = 1
    values["ACC_IACCHWAMode"] = hwa_mode
    values["ACC_HostLaneLeftStatus"] = 4 if hwa_active else 0
    values["ACC_HostLaneRightStatus"] = 4 if hwa_active else 0
    values["ACC_TargetBasedLateralControl"] = stock_target_based_lateral if (hwa_active and stock_target_based_lateral in (2, 3)) else (2 if hwa_active else 0)
    if hwa_active:
      values["ACC_DriverHandsOffStatus"] = stock_hands_off if stock_hands_off in (0, 1) else 0
      if CarControllerParams.IACC_CURVATURE_CONTROL and hwa_curvature is not None:
        road_curvature = float(np.clip(-hwa_curvature, -0.03, 0.03))
        values["ACC_RoadCurvature"] = road_curvature
        values["ACC_RoadCurvatureNear"] = road_curvature
        values["ACC_RoadCurvatureFar"] = road_curvature
    elif hwa_mode == 4:
      values["ACC_DriverHandsOffStatus"] = 3 if (driver_override or steering_pressed) else 1
    else:
      values["ACC_DriverHandsOffStatus"] = stock_hands_off if stock_hands_off in (0, 1) else (0 if steering_pressed else 1)
    values["ACC_AEBTextInfo"] = 0
    values["ACC_IACCHWATextInfoForDriver"] = 0
    values["ACC_ELKAlert"] = 0
  else:
    values["ACC_IACCHWAEnable"] = 0
    values["ACC_IACCHWAMode"] = 0
    values["ACC_TargetBasedLateralControl"] = 0
    values["ACC_HostLaneLeftStatus"] = 0
    values["ACC_HostLaneRightStatus"] = 0
    values["ACC_DriverHandsOffStatus"] = 1

  values["ACC_RollingCounter_36D"] = frame % 0x10
  values["ACC_CRCCheck_36D"] = 0
  values["ACC_RollingCounter_30A"] = frame % 0x10
  values["ACC_CRCCheck_30A"] = 0
  values["ACC_RollingCounter_30D"] = frame % 0x10
  values["ACC_CRCCheck_30D"] = 0
  values["ACC_RollingCounter_367"] = frame % 0x10
  values["ACC_CRCCheck_367"] = 0

  dat = packer.make_can_msg("GW_31A", bus, values)[1]
  values["ACC_CRCCheck_36D"] = changan_checksum(dat[:7])
  values["ACC_CRCCheck_30A"] = changan_checksum(dat[8:15])
  values["ACC_CRCCheck_30D"] = changan_checksum(dat[16:23])
  values["ACC_CRCCheck_367"] = changan_checksum(dat[24:31])
  return packer.make_can_msg("GW_31A", bus, values)
