"""
SCALED_PRESSURE (message id 29) — https://mavlink.io/en/messages/common.html#SCALED_PRESSURE

The first barometer (and differential-pressure sensor). The checks are shared
with the other SCALED_PRESSURE messages: see pressure_checks.py and README.md
in this directory.
"""

import pytest

from tests import report
from tests.messages_telemetry.emission import Emission
from tests.messages_telemetry.scaled_pressure import pressure_checks

_MSG_NAME = "SCALED_PRESSURE"
_MSG_ID = 29
REQUIRED_MESSAGES = {"GLOBAL_POSITION_INT": "the altitude press_abs is checked against"}
report.declare_params(report.MESSAGE_PROTOCOL, _MSG_NAME, pressure_checks.FIELDS)

pytestmark = pytest.mark.timeout(180)

EMISSION = Emission(_MSG_NAME, _MSG_ID, window_extra=("GLOBAL_POSITION_INT",))
EMISSION.define_tests(globals(), "scaled_pressure")
pressure_checks.define_field_tests(globals(), "scaled_pressure", EMISSION)
