"""
GPS_RAW_INT in flight — vel, cog and alt checks that need the vehicle to move
(gps_checks.py, README.md in this directory; the shared flight is in ../flight.py).

GPS2_RAW has no flight module: neither SITL has a second receiver unless
test_gps2_raw.py's setup adds one, and that setup isn't combined with the flight.
"""

from tests import report
from tests.flight_helpers import require_real_stack  # noqa: F401 — autouse: flight tests need a real stack
from tests.messages_telemetry import flight as F
from tests.messages_telemetry.gps_raw_int import gps_checks

_MSG_NAME = "GPS_RAW_INT"
_MSG_ID = 24
report.declare_params(report.MESSAGE_PROTOCOL, _MSG_NAME, gps_checks.FIELDS)

pytestmark = F.PYTESTMARK

gps_checks.define_flight_tests(globals(), "gps_raw_int", _MSG_NAME)
