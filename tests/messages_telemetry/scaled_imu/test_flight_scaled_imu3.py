"""
SCALED_IMU3 in flight — gyro, acceleration and magnetometer checks that need
rotation and acceleration (imu_checks.py, README.md in this directory; the
shared flight is in ../flight.py).
"""

from tests import report
from tests.flight_helpers import require_real_stack  # noqa: F401 — autouse: flight tests need a real stack
from tests.messages_telemetry import flight as F
from tests.messages_telemetry.scaled_imu import imu_checks

_MSG_NAME = "SCALED_IMU3"
_MSG_ID = 129
report.declare_params(report.MESSAGE_PROTOCOL, _MSG_NAME, imu_checks.FIELDS)

pytestmark = F.PYTESTMARK

imu_checks.define_flight_tests(globals(), "scaled_imu3", _MSG_NAME)
