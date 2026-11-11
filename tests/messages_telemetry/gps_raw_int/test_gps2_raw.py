"""
GPS2_RAW (message id 124) — https://mavlink.io/en/messages/common.html#GPS2_RAW

The second GPS receiver. Same fields as GPS_RAW_INT plus dgps_numch/dgps_age
(the extensions are in a different order). Checks shared with GPS_RAW_INT: see
gps_checks.py and README.md in this directory.

Neither SITL has a second receiver by default, so `_second_gps` (autouse)
gives it one for this module — a stack-keyed setup step (root CLAUDE.md Tier 2
pattern #11), keyed on the vehicle's own HEARTBEAT: ArduPilot GPS2_TYPE=1 +
SIM_GPS2_ENABLE=1, PX4 SENS_GNSS1_OFFX=1 m (sensor_gps_sim then publishes a
second receiver). Both persist, so the module's last test puts them back.
Set GPS2_SIM_SETUP=0 to test the SITL as it is.
"""

import asyncio
import logging
import os
import time

import pytest

from tests import report
from tests.message_watcher import MAV_AUTOPILOT_ARDUPILOTMEGA, ensure_harness_streams
from tests.messages_telemetry.conftest import MessageSampler, request_message, vehicle_heartbeat
from tests.messages_telemetry.emission import Emission
from tests.messages_telemetry.gps_raw_int import gps_checks

_MSG_NAME = "GPS2_RAW"
_MSG_ID = 124
REQUIRED_MESSAGES = {"GLOBAL_POSITION_INT": "the estimate the GPS position is compared against"}
report.declare_params(report.MESSAGE_PROTOCOL, _MSG_NAME, [*gps_checks.FIELDS, *gps_checks.GPS2_EXTRA])

pytestmark = pytest.mark.timeout(300)  # the first and last tests include a SITL restart

EMISSION = Emission(_MSG_NAME, _MSG_ID, window_extra=("GLOBAL_POSITION_INT",))
EMISSION.define_tests(globals(), "gps2_raw")
gps_checks.define_field_tests(globals(), "gps2_raw", EMISSION, gps2=True)


# ---------------------------------------------------------------------------
# Stack-keyed setup: a second simulated GPS for this module only
# ---------------------------------------------------------------------------

log = logging.getLogger(__name__)
MAV_AUTOPILOT_PX4 = 12
# autopilot -> [(param, value while testing, value to restore, is_float)]
SECOND_GPS_PARAMS = {
    MAV_AUTOPILOT_ARDUPILOTMEGA: [("GPS2_TYPE", 1, 0, False), ("SIM_GPS2_ENABLE", 1, 0, False)],
    MAV_AUTOPILOT_PX4: [("SENS_GNSS1_OFFX", 1.0, 0.0, True)],
}
GPS2_READY_TIMEOUT_S = 120.0
_setup: dict = {"done": False, "changed": None}


async def _gps2_arrives(system, timeout_s: float = 3.0) -> bool:
    async with MessageSampler(system, _MSG_NAME) as s:
        await request_message(system, _MSG_ID)
        await asyncio.sleep(timeout_s)
    return bool(s.samples(_MSG_NAME))


async def _set_params(system, values) -> None:
    for name, value, is_float in values:
        if is_float:
            await system.param.set_param_float(name, float(value))
        else:
            await system.param.set_param_int(name, int(value))
        log.info("GPS2 setup: %s = %s", name, value)


@pytest.fixture(autouse=True)
async def _second_gps(request, gcs_system, restart_flight_stack):
    if not report.MOCK_MODE and os.environ.get("GPS2_SIM_SETUP", "1") != "0" and not _setup["done"]:
        _setup["done"] = True
        if not await _gps2_arrives(gcs_system):
            hb = await vehicle_heartbeat(gcs_system)
            params = SECOND_GPS_PARAMS.get(hb.get("autopilot"))
            try:
                if params:
                    await _set_params(gcs_system, [(n, v, f) for n, v, _, f in params])
            except Exception as exc:  # noqa: BLE001 — e.g. a build without the parameter
                log.warning("GPS2 setup: can't add a second simulated GPS on this build (%r); testing as is", exc)
                _setup["unavailable"] = f"can't add a second simulated GPS on this build ({exc})"
                params = None
            if params:
                _setup["changed"] = params
                restart_flight_stack()
                await ensure_harness_streams(gcs_system)
                deadline = time.monotonic() + GPS2_READY_TIMEOUT_S
                while time.monotonic() < deadline and not await _gps2_arrives(gcs_system, 2.0):
                    pass
                log.info("GPS2 setup: restarted with a second simulated GPS (%s), %.0fs to first GPS2_RAW",
                         [p[0] for p in params], GPS2_READY_TIMEOUT_S - (deadline - time.monotonic()))
    yield
    module_items = [i for i in request.session.items if i.module is request.node.module]
    if _setup["changed"] and module_items and request.node is module_items[-1]:
        try:
            await _set_params(gcs_system, [(n, r, f) for n, _, r, f in _setup["changed"]])
            restart_flight_stack()
            await ensure_harness_streams(gcs_system)
            log.info("GPS2 setup: restored %s and restarted", [p[0] for p in _setup["changed"]])
        finally:
            _setup["changed"] = None
