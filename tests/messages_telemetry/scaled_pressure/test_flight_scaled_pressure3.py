"""
SCALED_PRESSURE3 in flight — press_abs at two cruise altitudes and press_diff
with airflow (pressure_checks.py, README.md in this directory; the shared
flight is in ../flight.py).
"""

from tests import report
from tests.flight_helpers import require_real_stack  # noqa: F401 — autouse: flight tests need a real stack
from tests.messages_telemetry import flight as F
from tests.messages_telemetry.scaled_pressure import pressure_checks

_MSG_NAME = "SCALED_PRESSURE3"
_MSG_ID = 143
report.declare_params(report.MESSAGE_PROTOCOL, _MSG_NAME, pressure_checks.FIELDS)

pytestmark = F.PYTESTMARK

pressure_checks.define_flight_tests(globals(), "scaled_pressure3", _MSG_NAME)
