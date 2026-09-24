from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import AngleSteeringLimits, Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, structs
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts, SupportType
from opendbc.car.fw_query_definitions import FwQueryConfig
from opendbc.car.structs import CarParams

Ecu = CarParams.Ecu


@dataclass
class ChanganCarDocs(CarDocs):
  package: str = "\u539f\u8f66\u8f85\u52a9\u9a7e\u9a76"
  car_parts: CarParts = field(default_factory=lambda: CarParts.common([CarHarness.obd_ii]))
  support_type: SupportType = SupportType.COMMUNITY
  support_link: str = "#community"


@dataclass
class ChanganPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: "changan"})


class ChanganFlags(IntFlag):
  GAS = 1
  NO_KNOWN_GEAR_MSG = 2
  NO_OPENPILOT_LONGITUDINAL = 4


class CAR(Platforms):
  CHANGAN_Z6_IDD = ChanganPlatformConfig(
    [ChanganCarDocs("Changan Z6 Idd")],
    CarSpecs(mass=1760., wheelbase=2.795, steerRatio=14.5, centerToFrontRatio=0.44),
  )
  CHANGAN_Z6 = ChanganPlatformConfig(
    [ChanganCarDocs("Changan Z6")],
    CarSpecs(mass=1550., wheelbase=2.795, steerRatio=14.5, centerToFrontRatio=0.44),
    flags=ChanganFlags.GAS,
  )


class CarControllerParams:
  ANGLE_LIMITS = AngleSteeringLimits(
    720.,  # deg, matches EPS_AngleCmd DBC range
    ([0., 5., 15.], [40., 7., 1.2]),
    ([0., 5., 15.], [45., 30., 3.5]),
  )
  # Legacy desktop Changan controller kept the OP angle close to the live
  # steering angle and only widened the window at low speed. Keep that behavior
  # here so driver takeover and big-angle steering do not make EPS reject IACC.
  EPS_CMD_ERROR_BP = [0., 15., 30., 45., 60., 75., 90., 105., 120., 135., 150., 180.]
  EPS_CMD_ERROR_V = [15.0, 15.0, 12.0, 9.0, 7.0, 5.5, 4.0, 3.5, 3.0, 2.7, 2.5, 2.5]
  EPS_CMD_DELTA_BP = [0., 15., 30., 45., 60., 75., 90., 105., 120., 135., 150., 180.]
  EPS_CMD_DELTA_V = [1.4, 1.4, 1.1, 0.9, 0.7, 0.55, 0.45, 0.38, 0.32, 0.27, 0.22, 0.22]
  EPS_REFERENCE_MAX_DELTA = 5.0
  EPS_REFERENCE_HOLD_FRAMES = 10
  EPS_HIGH_ANGLE_OUTWARD_START = 400.0
  EPS_DYNAMIC_WINDOW_DEFAULT = 430.0
  EPS_DYNAMIC_WINDOW_EDGE_MARGIN = 0.5
  EPS_ACTIVE_SETTLE_FRAMES = 100
  EPS_ACTIVE_SETTLE_CMD_ERROR = 1.5
  EPS_ACTIVE_SETTLE_CMD_DELTA = 0.8
  EPS_AUTHORITY_STEP = 0.02
  EPS_ACTIVE_SETTLE_AUTHORITY_STEP = 0.01
  STEER_COMMAND_MAX_ANGLE = 170.0

  STEER_STEP = 1
  # Proven Z6 EPS authority; must match Panda safety raw limits 1174/874.
  STEER_TORQUE_LIMIT = 3.0
  STEER_THRESHOLD = 1.0
  STEER_OVERRIDE_TAKEOVER_THRESHOLD_IDD = 3.0
  STEER_OVERRIDE_TAKEOVER_THRESHOLD_GAS = 8.0
  STEER_OVERRIDE_TAKEOVER_TORSION_THRESHOLD = 6.5
  STEER_OVERRIDE_RELEASE_THRESHOLD = 1.5
  STEER_OVERRIDE_RELEASE_TORSION_THRESHOLD = 1.0
  STEER_OVERRIDE_RELEASE_ANGLE = 90.0
  # IACC/EPS session timing is at 100 Hz. These windows come from stock logs
  # and the EPS reject-state analysis; keep 0x31A alive while EPS recovers.
  IACC_STANDBY_FRAMES = 10
  STEER_ARMING_FRAMES = 120
  IACC_EPS_ACK_TIMEOUT_FRAMES = 100
  IACC_RECOVERY_CLEAN_FRAMES = 120
  IACC_SESSION_OFF_HOLD_FRAMES = 100
  IACC_RECOVERY_SESSION_HOLD_FRAMES = 1000
  STEER_OVERRIDE_COOLDOWN_FRAMES = 20  # short handoff pause; angle hysteresis prevents high-angle re-entry
  STEER_REENGAGE_TORSION_THRESHOLD = 1.8
  STEER_REENGAGE_RATE_THRESHOLD = 25.0
  IACC_CURVATURE_CONTROL = False

  ACC_CONTROL_STEP = 2
  ACCEL_MIN = -3.5
  IDD_ACCEL_CDD_ENTER = -0.12
  IDD_ACCEL_CDD_EXIT = -0.03
  Z6_ACCEL_CDD_ENTER = -0.25
  Z6_ACCEL_CDD_EXIT = -0.10
  # Z6 gas logs keep torque active through small negative accel and only cut
  # into CDD on stronger decel. Start conservatively and keep a deadband.
  # Below this speed, keep the CDD brake path latched through the release
  # deadband and smooth small longitudinal changes. This removes the low-speed
  # brake/torque on-off pulse without weakening stronger decel.
  Z6_LOW_SPEED_BRAKE_KPH = 40.0
  Z6_LOW_SPEED_BRAKE_RELEASE_STEP = 0.04
  Z6_LOW_SPEED_ACCEL_UP_STEP = 0.06
  Z6_LOW_SPEED_ACCEL_DOWN_STEP = 0.08
  # Continuous low-pass for positive drive torque. Unlike a zero deadband,
  # this does not cut power and then surge; it simply removes fast oscillation.
  Z6_LOW_SPEED_DRIVE_FILTER_UP = 0.06
  Z6_LOW_SPEED_DRIVE_FILTER_DOWN = 0.10
  # Full low-speed range (3-40 km/h): reduce positive drive torque strongly
  # enough to prevent the repeated overshoot/correction cycle.
  Z6_LOW_SPEED_DRIVE_SCALE = 0.40
  Z6_LOW_SPEED_DRIVE_ACCEL_CAP = 0.05
  Z6_LOW_SPEED_SET_SPEED_MARGIN_KPH = 0.4
  Z6_LOW_SPEED_SET_SPEED_ACCEL_CAP = 0.03
  # Preserve the proven low-speed fast path for meaningful braking requests.
  Z6_LOW_SPEED_IMMEDIATE_BRAKE = -0.50
  Z6_LONG_ACTIVATION_MAX_ACCEL = 0.50
  Z6_LONG_ACTIVATION_RAMP_FRAMES = 25
  Z6_LONG_ACTIVATION_HOLD_FRAMES = 75
  Z6_LONG_ACTIVATION_SPEED_MARGIN_KPH = 0.5
  Z6_LONG_ACTIVATION_ACCEL_CAP = 0.0
  # Extra stabilization for a moving engagement below 20 km/h.  Around zero,
  # Z6's drive-torque and CDD brake paths have visibly different response;
  # suppress small sign changes during the first seconds so they cannot
  # alternate and rock the vehicle.
  Z6_CRAWL_ACTIVATION_MAX_KPH = 20.0
  Z6_CRAWL_ACTIVATION_HOLD_FRAMES = 200
  Z6_CRAWL_ACTIVATION_ACCEL_CAP = 0.12
  Z6_CRAWL_ACTIVATION_ACCEL_STEP = 0.02
  Z6_CRAWL_ACTIVATION_DECEL_STEP = 0.04
  Z6_CRAWL_ACTIVATION_NEUTRAL_BAND = 0.08
  Z6_CRAWL_ACTIVATION_BRAKE_ENTER = -0.18
  Z6_CRAWL_ACTIVATION_CDD_EXIT = 0.12
  Z6_LOW_SPEED_CDD_ENTER = -0.05
  Z6_LOW_SPEED_CDD_EXIT = 0.05
  # All-speed follow-braking shaping.  The controller runs the longitudinal
  # CAN update at 50 Hz (ACC_CONTROL_STEP=2 at 100 Hz), so these steps limit
  # jerk without making a normal lead-car response unacceptably slow.
  # Very strong requests remain an immediate path below the emergency limit.
  Z6_FOLLOW_BRAKE_ENTRY_STEP = 0.05
  Z6_FOLLOW_BRAKE_DOWN_STEP = 0.08
  Z6_FOLLOW_BRAKE_RELEASE_STEP = 0.06
  Z6_FOLLOW_BRAKE_EMERGENCY_ACCEL = -1.50
  Z6_FOLLOW_BRAKE_RELEASE_DEADBAND = 0.02
  # Neutral CAN cycles inserted when changing from positive drive torque to
  # ordinary braking.  This prevents residual throttle torque from overlapping
  # CDD and avoids the audible brake-pump kick on the C3X/Z6.
  Z6_THROTTLE_BRAKE_HANDOFF_FRAMES = 2
  # Requests at or below this acceleration are treated as critical and bypass
  # the comfort handoff to preserve emergency stopping response.
  Z6_THROTTLE_BRAKE_HANDOFF_EMERGENCY_ACCEL = -2.80
  # The normal follow filter intentionally yields to large planner requests.
  # Catch that handoff with a faster, bounded ramp so a newly selected close
  # lead or cut-in cannot turn one 50 Hz update into a full brake step.  The
  # critical branch still reaches maximum braking quickly, but progressively.
  Z6_RAPID_BRAKE_TRIGGER_ACCEL = -1.50
  Z6_RAPID_BRAKE_LOW_SPEED_TRIGGER_ACCEL = -0.50
  Z6_RAPID_BRAKE_ENTRY_STEP = 0.20
  Z6_RAPID_BRAKE_DOWN_STEP = 0.16
  Z6_RAPID_BRAKE_CRITICAL_ACCEL = -2.80
  Z6_RAPID_BRAKE_CRITICAL_DOWN_STEP = 0.25
  ACCEL_MAX = 2.5
  ACCEL_TRQ_MIN = -5000.0
  ACCEL_TRQ_NEUTRAL = -4500.0
  ACCEL_TRQ_MAX = -2050.0

  def __init__(self, CP):
    pass


class ChanganSafetyFlags(IntFlag):
  LONGITUDINAL = 1


class Buttons:
  NONE = 0
  RES_ACCEL = 1
  SET_DECEL = 2
  CANCEL = 3
  GAP_DIST = 4
  IACC = 5


GEAR_MAP = {
  7: structs.CarState.GearShifter.park,
  8: structs.CarState.GearShifter.reverse,
  9: structs.CarState.GearShifter.neutral,
  10: structs.CarState.GearShifter.drive,
  11: structs.CarState.GearShifter.sport,
}


DBC = CAR.create_dbc_map()

# Changan has no FW fingerprinting yet; manual selection remains available as a fallback.
FW_QUERY_CONFIG = FwQueryConfig(requests=[])


if __name__ == "__main__":
  cars = []
  for platform in CAR:
    for doc in platform.config.car_docs:
      cars.append(doc.name)
  cars.sort()
  for c in cars:
    print(c)
