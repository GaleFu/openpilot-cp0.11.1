#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerPanda


class TestChanganSafety(common.PandaCarSafetyTest, common.AngleSteeringSafetyTest):
  TX_MSGS = [[0x1BA, 0], [0x17E, 2], [0x244, 0], [0x307, 0], [0x31A, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x1BA,)}
  FWD_BLACKLISTED_ADDRS = {2: [0x1BA]}

  GAS_PRESSED_THRESHOLD = 0

  STEER_ANGLE_MAX = 720
  DEG_TO_CAN = 10

  ANGLE_RATE_BP = [0., 5., 15.]
  ANGLE_RATE_UP = [5., .8, .15]
  ANGLE_RATE_DOWN = [5., 3.5, .4]

  def setUp(self):
    self.packer = CANPackerPanda("changan")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.changan, 1)
    self.safety.init_tests()

  def _angle_cmd_msg(self, angle: float, enabled: bool, torque_limit: float = 4.0):
    values = {
      "ACC_MotorTorqueMaxLimitRequest": torque_limit,
      "ACC_MotorTorqueMinLimitRequest": -torque_limit,
      "EPS_AngleCmd": angle,
      "EPS_LatCtrlActive": enabled,
      "ACC_RollingCounter_1BA": 0,
      "ACC_CRCCheck_1BA": 0,
    }
    return self.packer.make_can_msg_panda("GW_1BA", 0, values)

  def test_eps_torque_limit_boundary(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._angle_cmd_msg(0., True, 4.0)))
    self.assertFalse(self._tx(self._angle_cmd_msg(0., True, 4.02)))
    # The previously proven 3.0 authority must still be accepted.
    self.assertTrue(self._tx(self._angle_cmd_msg(0., True, 3.0)))

  def _angle_meas_msg(self, angle: float):
    values = {"SAS_SteeringAngle": angle, "SAS_SteeringAngleValid": 0}
    return self.packer.make_can_msg_panda("GW_180", 0, values)

  def _speed_msg(self, speed):
    values = {"ESP_VehicleSpeed": speed * 3.6, "ESP_VehicleSpeedValid": 0}
    return self.packer.make_can_msg_panda("GW_17A", 0, values)

  def _user_brake_msg(self, brake):
    values = {"EMS_BrakePedalStatus": 1 if brake else 0}
    return self.packer.make_can_msg_panda("GW_1A6", 0, values)

  def _user_gas_msg(self, gas):
    values = {"EMS_AccPedal": gas}
    return self.packer.make_can_msg_panda("GW_1A6", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"ACC_ACCMode": 3 if enable else 1}
    return self.packer.make_can_msg_panda("GW_244", 2, values)

  def _cruise_buttons_msg(self, *, iacc=False, cancel=False, resume=False, set_button=False):
    values = {
      "GW_MFS_IACCenable_switch_signal": 1 if iacc else 0,
      "GW_MFS_Cancle_switch_signal": 1 if cancel else 0,
      "GW_MFS_RESPlus_switch_signal": 1 if resume else 0,
      "GW_MFS_SETReduce_switch_signal": 1 if set_button else 0,
    }
    return self.packer.make_can_msg_panda("GW_28C", 0, values)

  def test_iacc_main_button_toggles_controls_allowed(self):
    self.safety.set_controls_allowed(False)

    self.assertTrue(self._rx(self._cruise_buttons_msg(iacc=True)))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self._cruise_buttons_msg()))
    self.assertTrue(self.safety.get_controls_allowed())

    self.assertTrue(self._rx(self._cruise_buttons_msg(iacc=True)))
    self.assertTrue(self._rx(self._cruise_buttons_msg()))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_set_resume_reengage_and_cancel_clear_state(self):
    self.safety.set_controls_allowed(False)
    for name in ("resume", "set_button"):
      self.assertTrue(self._rx(self._cruise_buttons_msg(**{name: True})))
      self.assertTrue(self._rx(self._cruise_buttons_msg()))
      self.assertTrue(self.safety.get_controls_allowed())
      self.assertTrue(self._rx(self._cruise_buttons_msg(cancel=True)))
      self.assertFalse(self.safety.get_controls_allowed())

    self.assertTrue(self._rx(self._cruise_buttons_msg(iacc=True)))
    self.assertTrue(self._rx(self._cruise_buttons_msg()))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self._cruise_buttons_msg(cancel=True)))
    self.assertFalse(self.safety.get_controls_allowed())

    self.assertTrue(self._rx(self._cruise_buttons_msg(resume=True)))
    self.assertTrue(self._rx(self._cruise_buttons_msg()))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_brake_clears_iacc_main_state(self):
    self.assertTrue(self._rx(self._cruise_buttons_msg(iacc=True)))
    self.assertTrue(self._rx(self._cruise_buttons_msg()))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self._user_brake_msg(True)))
    self.assertFalse(self.safety.get_controls_allowed())

    # After braking, one IACC release must enable again instead of toggling an
    # obsolete pre-brake main state off.
    self.assertTrue(self._rx(self._cruise_buttons_msg(iacc=True)))
    self.assertTrue(self._rx(self._cruise_buttons_msg()))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_longitudinal_frames_registered(self):
    allowed = {(addr, bus) for addr, bus in self.TX_MSGS}
    self.assertIn((0x1BA, 0), allowed)
    self.assertIn((0x17E, 2), allowed)
    self.assertIn((0x244, 0), allowed)
    self.assertIn((0x307, 0), allowed)
    self.assertIn((0x31A, 0), allowed)


if __name__ == "__main__":
  unittest.main()
