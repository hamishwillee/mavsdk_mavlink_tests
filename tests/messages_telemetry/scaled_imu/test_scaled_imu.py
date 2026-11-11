"""
SCALED_IMU (message id 26) — https://mavlink.io/en/messages/common.html#SCALED_IMU

The first IMU's scaled readings. The checks are shared with the other
SCALED_IMU messages: see imu_checks.py and README.md in this directory.
"""

import pytest

from tests import report
from tests.messages_telemetry.emission import Emission
from tests.messages_telemetry.scaled_imu import imu_checks

_MSG_NAME = "SCALED_IMU"
_MSG_ID = 26
REQUIRED_MESSAGES = {"ATTITUDE": "the attitude the accel/mag are checked against"}
report.declare_params(report.MESSAGE_PROTOCOL, _MSG_NAME, imu_checks.FIELDS)

pytestmark = pytest.mark.timeout(180)

EMISSION = Emission(_MSG_NAME, _MSG_ID, window_extra=("ATTITUDE", "GLOBAL_POSITION_INT"))
EMISSION.define_tests(globals(), "scaled_imu")
imu_checks.define_field_tests(globals(), "scaled_imu", EMISSION)
