"""
GPS_RAW_INT (message id 24) — https://mavlink.io/en/messages/common.html#GPS_RAW_INT

The (first) GPS receiver's raw output. Checks shared with GPS2_RAW: see
gps_checks.py and README.md in this directory.
"""

import pytest

from tests import report
from tests.messages_telemetry.emission import Emission
from tests.messages_telemetry.gps_raw_int import gps_checks

_MSG_NAME = "GPS_RAW_INT"
_MSG_ID = 24
REQUIRED_MESSAGES = {"GPS_RAW_INT": "the message under test",
                     "GLOBAL_POSITION_INT": "the estimate the GPS position is compared against"}
report.declare_params(report.MESSAGE_PROTOCOL, _MSG_NAME, gps_checks.FIELDS)

pytestmark = pytest.mark.timeout(180)

EMISSION = Emission(_MSG_NAME, _MSG_ID, window_extra=("GLOBAL_POSITION_INT",))
EMISSION.define_tests(globals(), "gps_raw_int")
gps_checks.define_field_tests(globals(), "gps_raw_int", EMISSION)
