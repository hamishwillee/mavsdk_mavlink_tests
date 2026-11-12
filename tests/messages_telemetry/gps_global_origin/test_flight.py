"""
GPS_GLOBAL_ORIGIN in flight (README.md in this directory; the shared flight is in ../flight.py).

The XML says the message is "emitted whenever a new GPS-Local position mapping
is requested or set" — so it may change, and the flight characterises whether
it does from arming to the end of the flight (including landing elsewhere and
arming there). LOCAL_POSITION_NED's flight test uses the origin current at each
sample, so a change can't make that check fail.
"""

from __future__ import annotations

from tests import report
from tests.flight_helpers import require_real_stack  # noqa: F401 — autouse: flight tests need a real stack
from tests.messages_telemetry import flight as F
from tests.messages_telemetry.conftest import distinct, record_detail, skip_untestable

_MSG_NAME = "GPS_GLOBAL_ORIGIN"
_MSG_ID = 49
_FIELDS = ["latitude", "longitude", "altitude"]
report.declare_params(report.MESSAGE_PROTOCOL, _MSG_NAME, [*_FIELDS, "time_usec"])

pytestmark = F.PYTESTMARK


async def test_gps_global_origin_obs_changes_in_flight(gcs_system, request, restart_flight_stack):
    """Characterise whether the origin changes from arming to the end of the flight."""
    fl = await F.get_flight(gcs_system, restart_flight_stack)
    t0, _ = fl.need_moving(request, [])
    samples = fl.s(_MSG_NAME, t0)
    if len(samples) < 3:
        skip_untestable(request, [], "too few GPS_GLOBAL_ORIGIN in flight", f"{len(samples)} received")
    shown = "; ".join(f"{f} {distinct([s.fields[f] for s in samples], 3)}" for f in _FIELDS)
    values = {tuple(s.fields[f] for f in _FIELDS) for s in samples}
    record_detail(request, f"{len(samples)} from arming to the end: "
                           f"{'constant' if len(values) == 1 else f'{len(values)} different origins'} ({shown}); {fl.label}")
