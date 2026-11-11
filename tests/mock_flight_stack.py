"""
Mock flight stack for paired-mode protocol compliance testing.

The MockFlightStack class implements the complete MAVLink mission protocol
(upload, download, clear, capability reporting) using ``mavlink_direct`` on the
drone-side MAVSDK System object.  All four protocol roles run as concurrent
asyncio tasks behind a single-subscription-per-message-type queue.

Usage::

    stack = MockFlightStack()
    task = asyncio.create_task(stack.run(drone_system))
    # … run tests …
    task.cancel()
    await task

Default behaviour:
- Accept all frames, store items exactly as received, serve unchanged on download.
- Advertise MAV_PROTOCOL_CAPABILITY_MISSION_INT (bit 2, value 4).
- No delays, no failure injection.
- Stream SYSTEM_TIME (1 Hz), ATTITUDE (4 Hz) and ATTITUDE_QUATERNION (4 Hz),
  for one fixed, non-trivial attitude (MOCK_ATTITUDE_RAD), honouring
  MAV_CMD_SET_MESSAGE_INTERVAL / MAV_CMD_REQUEST_MESSAGE for them, and answer
  REQUEST_MESSAGE(MESSAGE_INTERVAL) — enough for tests/messages_telemetry/ to
  exercise its own logic in paired mode.
"""

import asyncio
import json
import logging
import math
import time

from tests.mavsdk_compat import SystemShim as System, open_system, open_paired_drone
from mavsdk.plugins.mavlink_direct import MavlinkMessage

log = logging.getLogger(__name__)

# MAV_MISSION_RESULT values (common.xml)
_ACCEPTED = 0
_UNSUPPORTED_FRAME = 2

# MAV_RESULT values (MAVLink master common.xml)
MAV_RESULT_ACCEPTED             = 0
MAV_RESULT_TEMPORARILY_REJECTED = 1
MAV_RESULT_DENIED               = 2
MAV_RESULT_UNSUPPORTED          = 3
MAV_RESULT_FAILED               = 4
MAV_RESULT_IN_PROGRESS          = 5
MAV_RESULT_CANCELLED            = 6  # defined in MAVLink master; absent from pymavlink 2.4.49
MAV_RESULT_COMMAND_LONG_ONLY    = 7
MAV_RESULT_COMMAND_INT_ONLY     = 8
MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME = 9
MAV_RESULT_NOT_IN_CONTROL       = 10  # wip in MAVLink master common.xml

# Default: MAV_PROTOCOL_CAPABILITY_MISSION_INT (bit 2)
DEFAULT_CAPABILITY_BITS: int = 4

# MAVLink COMMAND_INT lat/lon sentinel and valid ranges
_INT32_MAX = 0x7FFF_FFFF       # "use current position" sentinel
_MAX_LAT_INT = 900_000_000     # ±90° × 1e7
_MAX_LON_INT = 1_800_000_000   # ±180° × 1e7

# MAVLink identity of the GCS peer (MAVSDK GROUND_STATION — see tests/mavsdk_compat.GCS_SYSID)
from tests.mavsdk_compat import GCS_COMPID as _GCS_COMPID, GCS_SYSID as _GCS_SYSID  # MAVSDK GROUND_STATION identity

def _f(val, default: float = 0.0) -> float:
    """Convert a JSON field value to float, treating None (JSON null) as NaN."""
    if val is None:
        return float("nan")
    return float(val)


# MAVLink message ids the mock can stream: id -> (name, default interval in µs).
# Default rates are arbitrary but distinct from anything a test requests, so a
# test can tell "back to default" from "still at the requested rate".
MSG_ID_SYSTEM_TIME = 2
MSG_ID_ATTITUDE = 30
MSG_ID_ATTITUDE_QUATERNION = 31
MSG_ID_MESSAGE_INTERVAL = 244
DEFAULT_STREAM_INTERVALS_US: dict[int, int] = {
    MSG_ID_SYSTEM_TIME: 1_000_000,          # 1 Hz
    MSG_ID_ATTITUDE: 250_000,               # 4 Hz
    MSG_ID_ATTITUDE_QUATERNION: 250_000,    # 4 Hz
}
# The mock's attitude: roll, pitch, yaw (rad). Non-trivial, so a test that
# converts ATTITUDE_QUATERNION to Euler angles exercises every component's sign.
MOCK_ATTITUDE_RAD = (math.radians(2.0), math.radians(-3.0), math.radians(30.0))


def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> list[float]:
    """ZYX intrinsic (aeronautical) Euler angles -> [w, x, y, z]."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]
_MAV_CMD_SET_MESSAGE_INTERVAL = 511
_MAV_CMD_REQUEST_MESSAGE = 512


# Message types the mock needs to subscribe to
_MSG_TYPES = (
    "MISSION_COUNT",        # GCS → drone: initiates upload
    "MISSION_REQUEST_LIST", # GCS → drone: initiates download
    "MISSION_REQUEST_INT",  # GCS → drone: requests one item during download
    "MISSION_ITEM_INT",     # GCS → drone: delivers one item during upload
    "MISSION_CLEAR_ALL",    # GCS → drone: clear stored missions
    "COMMAND_LONG",         # GCS → drone: capability request + direct commands
    "COMMAND_INT",          # GCS → drone: direct command with location params
)


class MockFlightStack:
    """
    Configurable mock MAVLink flight stack for mission protocol testing.

    Parameters
    ----------
    capability_bits : int
        Value returned in AUTOPILOT_VERSION.capabilities.
        Default 4 (MAV_PROTOCOL_CAPABILITY_MISSION_INT).
    item_request_delay_s : float
        Seconds to wait before sending each MISSION_REQUEST_INT during upload.
    item_response_delay_s : float
        Seconds to wait before sending each MISSION_ITEM_INT during download.
    drop_responses : dict[str, int] | None
        ``{message_name: N}`` — silently skip the first N outgoing mission
        protocol messages of that type.  Example: ``{"MISSION_REQUEST_INT": 2}``.
    rejected_frames : set[int] | None
        Frame values to reject with MISSION_ACK(UNSUPPORTED_FRAME) on upload.
    command_results : dict[int, int] | None
        ``{command_id: MAV_RESULT}`` — result to return in COMMAND_ACK for each
        MAV_CMD.  Commands not listed return MAV_RESULT_ACCEPTED (0) by default.
    drop_command_acks : dict[int, int] | None
        ``{command_id: N}`` — silently drop the first N COMMAND_ACKs for that
        command, forcing the GCS to retransmit.
    command_in_progress : dict[int, list[int]] | None
        ``{command_id: [p0, p1, ...]}`` — emit one IN_PROGRESS ACK per entry
        (with the given progress value) before sending the final result ACK.
    duplicate_command_acks : dict[int, int] | None
        ``{command_id: N}`` — send N extra copies of the terminal COMMAND_ACK
        for that command (beyond the normal single ACK), simulating a
        double-ACK protocol bug.  Consumed after the first send (resets to 0),
        so it fires once per test.  IN_PROGRESS ACKs (if configured via
        ``command_in_progress``) are unaffected — only the terminal result ACK
        is duplicated.
    require_valid_location_cmds : set[int] | None
        Command IDs that require a real GNSS coordinate (no sentinel allowed).
        For these commands the INT32_MAX "use current position" sentinel is
        treated as invalid (DENIED), and NaN altitude is also DENIED.
        By default the mock treats INT32_MAX as always valid; this overrides
        that behaviour for the listed commands.
    emit_gps_global_origin : bool
        When True, send GPS_GLOBAL_ORIGIN (id=49) after COMMAND_ACK(ACCEPTED)
        for any command in ``require_valid_location_cmds``.  Default True.
    stream_intervals_us : dict[int, int] | None
        Default interval (µs) per streamable message id
        (``DEFAULT_STREAM_INTERVALS_US`` when omitted; 0 = not streamed by
        default). SET_MESSAGE_INTERVAL changes it (-1 disables, 0 restores
        the default) and REQUEST_MESSAGE sends one copy. Other message ids
        are ACKed as before but have no effect.
    """

    def __init__(
        self,
        capability_bits: int = DEFAULT_CAPABILITY_BITS,
        item_request_delay_s: float = 0.0,
        item_response_delay_s: float = 0.0,
        drop_responses: dict | None = None,
        rejected_frames: set | None = None,
        command_results: dict | None = None,
        drop_command_acks: dict | None = None,
        command_in_progress: dict | None = None,
        duplicate_command_acks: dict | None = None,
        require_valid_location_cmds: set | None = None,
        emit_gps_global_origin: bool = True,
        stream_intervals_us: dict | None = None,
    ) -> None:
        self.capability_bits = capability_bits
        self.item_request_delay_s = item_request_delay_s
        self.item_response_delay_s = item_response_delay_s
        self._drop_remaining: dict[str, int] = dict(drop_responses or {})
        self.rejected_frames: set[int] = set(rejected_frames or set())
        self._command_results: dict[int, int] = dict(command_results or {})
        self._drop_ack_remaining: dict[int, int] = dict(drop_command_acks or {})
        self._command_in_progress: dict[int, list[int]] = dict(command_in_progress or {})
        self._duplicate_ack_remaining: dict[int, int] = dict(duplicate_command_acks or {})
        self._require_valid_location_cmds: frozenset[int] = frozenset(require_valid_location_cmds or set())
        self.emit_gps_global_origin: bool = emit_gps_global_origin
        # Records every received COMMAND_INT and non-capability COMMAND_LONG
        self.received_commands: list[dict] = []
        # mission_type (int) → list of item field dicts
        self._store: dict[int, list[dict]] = {}
        self._queues: dict[str, asyncio.Queue] = {}
        # Streamed messages: default and current interval (µs) per message id.
        self._default_intervals: dict[int, int] = dict(
            DEFAULT_STREAM_INTERVALS_US if stream_intervals_us is None else stream_intervals_us
        )
        self._intervals: dict[int, int] = dict(self._default_intervals)
        self._interval_changed: dict[int, asyncio.Event] = {}
        self._boot_monotonic = time.monotonic()

    @property
    def store(self) -> dict[int, list[dict]]:
        """Snapshot of the current mission store (read-only view)."""
        return {k: list(v) for k, v in self._store.items()}

    # ------------------------------------------------------------------ run --

    async def run(self, system: System) -> None:
        """
        Start all protocol handlers.  Runs until the task is cancelled.

        Creates one queue per message type, one subscriber task per type, and
        four protocol handler tasks.  All share the same event loop.
        """
        self._queues = {t: asyncio.Queue() for t in _MSG_TYPES}

        all_tasks = [
            asyncio.create_task(self._subscribe(system, t, self._queues[t]))
            for t in _MSG_TYPES
        ] + [
            asyncio.create_task(self._upload_loop(system)),
            asyncio.create_task(self._download_loop(system)),
            asyncio.create_task(self._clear_loop(system)),
            asyncio.create_task(self._command_long_dispatch_loop(system)),
            asyncio.create_task(self._command_int_loop(system)),
        ] + [
            asyncio.create_task(self._stream_loop(system, msg_id))
            for msg_id in self._default_intervals
        ]
        try:
            await asyncio.gather(*all_tasks)
        except asyncio.CancelledError:
            pass
        finally:
            for t in all_tasks:
                t.cancel()
            await asyncio.gather(*all_tasks, return_exceptions=True)

    # --------------------------------------------------------- subscribers ---

    async def _subscribe(
        self, system: System, msg_type: str, queue: asyncio.Queue
    ) -> None:
        try:
            async for msg in system.mavlink_direct.message(msg_type):
                await queue.put(msg)
        except asyncio.CancelledError:
            pass
        except Exception as exc:
            log.warning("Subscription error for %s: %s", msg_type, exc)

    # --------------------------------------------------- receive helpers ----

    async def _recv(self, msg_type: str, timeout: float | None = None):
        q = self._queues[msg_type]
        if timeout is not None:
            return await asyncio.wait_for(q.get(), timeout=timeout)
        return await q.get()

    async def _recv_matching(self, msg_type: str, predicate, timeout: float | None = None):
        """
        Wait for the first message of *msg_type* whose parsed fields satisfy
        *predicate*.  Non-matching messages are discarded (protocol ordering
        guarantees seq-ordered delivery in normal operation).
        """
        async def _inner():
            while True:
                msg = await self._queues[msg_type].get()
                if predicate(json.loads(msg.fields_json)):
                    return msg
                log.debug("Discarding out-of-order %s message", msg_type)

        if timeout is not None:
            try:
                return await asyncio.wait_for(_inner(), timeout=timeout)
            except asyncio.TimeoutError:
                return None
        return await _inner()

    # --------------------------------------------------- send helpers ------

    def _should_drop(self, msg_name: str) -> bool:
        n = self._drop_remaining.get(msg_name, 0)
        if n > 0:
            self._drop_remaining[msg_name] = n - 1
            log.debug("Injecting drop of %s (%d remaining after this)", msg_name, n - 1)
            return True
        return False

    async def _send(
        self,
        system: System,
        message_name: str,
        fields: dict,
    ) -> None:
        if self._should_drop(message_name):
            return
        await system.mavlink_direct.send_message(MavlinkMessage(
            message_name=message_name,
            system_id=0,
            component_id=0,
            target_system_id=_GCS_SYSID,
            target_component_id=_GCS_COMPID,
            fields_json=json.dumps(fields),
        ))

    async def _send_gps_global_origin(
        self, system: System, lat_e7: int, lon_e7: int, alt_m: float
    ) -> None:
        """Send GPS_GLOBAL_ORIGIN (message id=49) echoing the commanded coordinates."""
        await self._send(system, "GPS_GLOBAL_ORIGIN", {
            "latitude": lat_e7,
            "longitude": lon_e7,
            "altitude": int(alt_m * 1000),  # m → mm (message field is int32, units mm)
            "time_usec": 0,
        })
        log.debug("GPS_GLOBAL_ORIGIN sent: lat=%d lon=%d alt_mm=%d", lat_e7, lon_e7, int(alt_m * 1000))

    async def _send_ack(self, system: System, mission_type: int, result: int) -> None:
        await self._send(system, "MISSION_ACK", {
            "target_system": _GCS_SYSID,
            "target_component": _GCS_COMPID,
            "type": result,
            "mission_type": mission_type,
        })

    # --------------------------------------------------------- upload loop --

    async def _upload_loop(self, system: System) -> None:
        while True:
            try:
                msg = await self._recv("MISSION_COUNT")
                await self._handle_upload(system, msg)
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                log.warning("Upload handler error: %s", exc, exc_info=True)

    async def _handle_upload(self, system: System, count_msg) -> None:
        fields = json.loads(count_msg.fields_json)
        count = int(fields["count"])
        mission_type = int(fields["mission_type"])
        log.debug("Upload: mission_type=%d count=%d", mission_type, count)

        items: list[dict] = []
        for seq in range(count):
            if self.item_request_delay_s > 0:
                await asyncio.sleep(self.item_request_delay_s)

            await self._send(system, "MISSION_REQUEST_INT", {
                "target_system": _GCS_SYSID,
                "target_component": _GCS_COMPID,
                "seq": seq,
                "mission_type": mission_type,
            })

            item_msg = await self._recv_matching(
                "MISSION_ITEM_INT",
                lambda f, s=seq: int(f.get("seq", -1)) == s,
                timeout=30.0,
            )
            if item_msg is None:
                log.warning("Timeout waiting for MISSION_ITEM_INT(seq=%d)", seq)
                return

            item = json.loads(item_msg.fields_json)
            frame = int(item.get("frame", 0))
            if frame in self.rejected_frames:
                log.debug("Rejecting frame=%d at seq=%d", frame, seq)
                await self._send_ack(system, mission_type, _UNSUPPORTED_FRAME)
                return

            items.append(item)

        self._store[mission_type] = items
        await self._send_ack(system, mission_type, _ACCEPTED)
        log.debug("Upload done: mission_type=%d items=%d", mission_type, len(items))

    # ------------------------------------------------------- download loop --

    async def _download_loop(self, system: System) -> None:
        while True:
            try:
                msg = await self._recv("MISSION_REQUEST_LIST")
                await self._handle_download(system, msg)
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                log.warning("Download handler error: %s", exc, exc_info=True)

    async def _handle_download(self, system: System, req_list_msg) -> None:
        fields = json.loads(req_list_msg.fields_json)
        mission_type = int(fields["mission_type"])
        items = self._store.get(mission_type, [])
        log.debug("Download: mission_type=%d count=%d", mission_type, len(items))

        await self._send(system, "MISSION_COUNT", {
            "target_system": _GCS_SYSID,
            "target_component": _GCS_COMPID,
            "count": len(items),
            "mission_type": mission_type,
        })

        for seq in range(len(items)):
            req_msg = await self._recv_matching(
                "MISSION_REQUEST_INT",
                lambda f, s=seq: int(f.get("seq", -1)) == s,
                timeout=30.0,
            )
            if req_msg is None:
                log.warning("Timeout waiting for MISSION_REQUEST_INT(seq=%d)", seq)
                return

            if self.item_response_delay_s > 0:
                await asyncio.sleep(self.item_response_delay_s)

            item = items[seq]
            await self._send(system, "MISSION_ITEM_INT", {
                "target_system": _GCS_SYSID,
                "target_component": _GCS_COMPID,
                "seq": seq,
                "frame": item.get("frame", 0),
                "command": item.get("command", 0),
                "current": item.get("current", 0),
                "autocontinue": item.get("autocontinue", 1),
                "param1": item.get("param1", 0.0),
                "param2": item.get("param2", 0.0),
                "param3": item.get("param3", 0.0),
                "param4": item.get("param4", 0.0),
                "x": item.get("x", 0),
                "y": item.get("y", 0),
                "z": item.get("z", 0.0),
                "mission_type": mission_type,
            })

        log.debug("Download done: mission_type=%d", mission_type)

    # ---------------------------------------------------------- clear loop --

    async def _clear_loop(self, system: System) -> None:
        while True:
            try:
                msg = await self._recv("MISSION_CLEAR_ALL")
                fields = json.loads(msg.fields_json)
                mission_type = int(fields["mission_type"])
                if mission_type == 255:
                    self._store.clear()
                    log.debug("Clear: all mission types")
                else:
                    self._store.pop(mission_type, None)
                    log.debug("Clear: mission_type=%d", mission_type)
                await self._send_ack(system, mission_type, _ACCEPTED)
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                log.warning("Clear handler error: %s", exc, exc_info=True)

    # --------------------------------------------- command_long dispatch ----

    async def _command_long_dispatch_loop(self, system: System) -> None:
        while True:
            try:
                msg = await self._recv("COMMAND_LONG")
                fields = json.loads(msg.fields_json)
                cmd = int(fields.get("command", 0))
                param1 = _f(fields.get("param1"))  # _f handles None (JSON null) → NaN
                # MAV_CMD_REQUEST_MESSAGE (512) with param1=148 (AUTOPILOT_VERSION)
                # NaN param1 (math.isnan) correctly falls to the else branch
                if cmd == 512 and not math.isnan(param1) and abs(param1 - 148.0) < 0.5:
                    await self._send(system, "AUTOPILOT_VERSION", {
                        "capabilities": self.capability_bits,
                        "flight_sw_version": 0,
                        "middleware_sw_version": 0,
                        "os_sw_version": 0,
                        "board_version": 0,
                        "vendor_id": 0,
                        "product_id": 0,
                        "uid": 0,
                        "flight_custom_version": [0, 0, 0, 0, 0, 0, 0, 0],
                        "middleware_custom_version": [0, 0, 0, 0, 0, 0, 0, 0],
                        "os_custom_version": [0, 0, 0, 0, 0, 0, 0, 0],
                    })
                    log.debug("Capability response: bits=0x%x", self.capability_bits)
                else:
                    confirmation = int(fields.get("confirmation", 0))
                    record = {
                        "type": "COMMAND_LONG",
                        "command": cmd,
                        "confirmation": confirmation,
                        "param1": _f(fields.get("param1")),
                        "param2": _f(fields.get("param2")),
                        "param3": _f(fields.get("param3")),
                        "param4": _f(fields.get("param4")),
                        "param5": _f(fields.get("param5")),
                        "param6": _f(fields.get("param6")),
                        "param7": _f(fields.get("param7")),
                    }
                    self.received_commands.append(record)

                    if cmd in self._require_valid_location_cmds:
                        # In COMMAND_LONG, param5/6 are float degrees; param7 is float metres.
                        # Reject: NaN for any coordinate, or lat/lon outside valid range.
                        # float(INT32_MAX) ≈ 2.1e9° is far outside range and caught here.
                        p5, p6, p7 = record["param5"], record["param6"], record["param7"]
                        coord_invalid = (
                            math.isnan(p5) or math.isnan(p6) or math.isnan(p7)
                            or abs(p5) > 90.0 or abs(p6) > 180.0
                        )
                        if coord_invalid:
                            log.debug(
                                "COMMAND_LONG cmd=%d: invalid coordinates (p5=%s p6=%s p7=%s) → DENIED",
                                cmd, p5, p6, p7,
                            )
                            await self._send(system, "COMMAND_ACK", {
                                "command": cmd,
                                "result": MAV_RESULT_DENIED,
                                "progress": 255,
                                "result_param2": 0,
                                "target_system": _GCS_SYSID,
                                "target_component": _GCS_COMPID,
                            })
                        else:
                            result = self._command_results.get(cmd, MAV_RESULT_ACCEPTED)
                            await self._send_command_ack(system, cmd)
                            if self.emit_gps_global_origin and result == MAV_RESULT_ACCEPTED:
                                lat_e7 = int(p5 * 1e7)
                                lon_e7 = int(p6 * 1e7)
                                await self._send_gps_global_origin(system, lat_e7, lon_e7, p7)
                    else:
                        await self._send_command_ack(system, cmd)
                        await self._apply_message_command(system, cmd, record)

                    log.debug("COMMAND_LONG cmd=%d confirmation=%d", cmd, confirmation)
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                log.warning("Command long dispatch error: %s", exc, exc_info=True)

    # ------------------------------------------------------- command_int loop --

    async def _command_int_loop(self, system: System) -> None:
        while True:
            try:
                msg = await self._recv("COMMAND_INT")
                fields = json.loads(msg.fields_json)
                cmd = int(fields.get("command", 0))
                x = int(fields.get("x", 0))
                y = int(fields.get("y", 0))
                z = _f(fields.get("z"))
                record = {
                    "type": "COMMAND_INT",
                    "command": cmd,
                    "frame": int(fields.get("frame", 0)),
                    "param1": _f(fields.get("param1")),
                    "param2": _f(fields.get("param2")),
                    "param3": _f(fields.get("param3")),
                    "param4": _f(fields.get("param4")),
                    "x": x,
                    "y": y,
                    "z": z,
                }
                self.received_commands.append(record)

                denied = False

                if cmd in self._require_valid_location_cmds:
                    # Command requires a real GNSS coordinate — INT32_MAX sentinel and NaN alt invalid.
                    if x == _INT32_MAX or y == _INT32_MAX:
                        log.debug("COMMAND_INT cmd=%d: INT32_MAX sentinel invalid for this command → DENIED", cmd)
                        denied = True
                    elif math.isnan(z):
                        log.debug("COMMAND_INT cmd=%d: NaN altitude invalid for this command → DENIED", cmd)
                        denied = True

                if not denied:
                    # INT32_MAX is the "use current position" sentinel — valid for other commands.
                    # Any other value outside the physical lat/lon range is DENIED.
                    lat_invalid = x != _INT32_MAX and abs(x) > _MAX_LAT_INT
                    lon_invalid = y != _INT32_MAX and abs(y) > _MAX_LON_INT
                    if lat_invalid or lon_invalid:
                        log.debug("COMMAND_INT cmd=%d: out-of-range lat/lon (x=%d, y=%d) → DENIED", cmd, x, y)
                        denied = True

                if denied:
                    await self._send(system, "COMMAND_ACK", {
                        "command": cmd,
                        "result": MAV_RESULT_DENIED,
                        "progress": 255,
                        "result_param2": 0,
                        "target_system": _GCS_SYSID,
                        "target_component": _GCS_COMPID,
                    })
                else:
                    result = self._command_results.get(cmd, MAV_RESULT_ACCEPTED)
                    await self._send_command_ack(system, cmd)
                    if (
                        self.emit_gps_global_origin
                        and cmd in self._require_valid_location_cmds
                        and result == MAV_RESULT_ACCEPTED
                    ):
                        await self._send_gps_global_origin(system, x, y, z)

                log.debug("COMMAND_INT cmd=%d frame=%d", cmd, record["frame"])
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                log.warning("Command int handler error: %s", exc, exc_info=True)

    async def _send_command_ack(self, system: System, cmd: int) -> None:
        """Send COMMAND_ACK for cmd, honouring drop and in-progress configuration."""
        n_drop = self._drop_ack_remaining.get(cmd, 0)
        if n_drop > 0:
            self._drop_ack_remaining[cmd] = n_drop - 1
            log.debug("Dropping COMMAND_ACK for cmd=%d (%d remaining)", cmd, n_drop - 1)
            return

        progress_values = self._command_in_progress.get(cmd, [])
        for progress in progress_values:
            await self._send(system, "COMMAND_ACK", {
                "command": cmd,
                "result": MAV_RESULT_IN_PROGRESS,
                "progress": int(progress),
                "result_param2": 0,
                "target_system": _GCS_SYSID,
                "target_component": _GCS_COMPID,
            })
            await asyncio.sleep(0.05)

        result = self._command_results.get(cmd, MAV_RESULT_ACCEPTED)
        ack_fields = {
            "command": cmd,
            "result": result,
            "progress": 255,
            "result_param2": 0,
            "target_system": _GCS_SYSID,
            "target_component": _GCS_COMPID,
        }
        await self._send(system, "COMMAND_ACK", ack_fields)
        log.debug("COMMAND_ACK cmd=%d result=%d", cmd, result)

        n_dup = self._duplicate_ack_remaining.get(cmd, 0)
        if n_dup > 0:
            self._duplicate_ack_remaining[cmd] = 0
            for _ in range(n_dup):
                await self._send(system, "COMMAND_ACK", ack_fields)
                log.debug("COMMAND_ACK cmd=%d result=%d (injected duplicate)", cmd, result)

    # ------------------------------------------------------- message streams --

    def _boot_ms(self) -> int:
        return int((time.monotonic() - self._boot_monotonic) * 1000)

    def _message_fields(self, msg_id: int) -> tuple[str, dict] | None:
        """(name, fields) for one fresh copy of a streamable message, or None."""
        if msg_id == MSG_ID_SYSTEM_TIME:
            return "SYSTEM_TIME", {
                "time_unix_usec": int(time.time() * 1e6),
                "time_boot_ms": self._boot_ms(),
            }
        if msg_id == MSG_ID_ATTITUDE:
            roll, pitch, yaw = MOCK_ATTITUDE_RAD
            return "ATTITUDE", {
                "time_boot_ms": self._boot_ms(),
                "roll": roll, "pitch": pitch, "yaw": yaw,
                "rollspeed": 0.0, "pitchspeed": 0.0, "yawspeed": 0.0,
            }
        if msg_id == MSG_ID_ATTITUDE_QUATERNION:
            q = _euler_to_quaternion(*MOCK_ATTITUDE_RAD)
            return "ATTITUDE_QUATERNION", {
                "time_boot_ms": self._boot_ms(),
                "q1": q[0], "q2": q[1], "q3": q[2], "q4": q[3],
                "rollspeed": 0.0, "pitchspeed": 0.0, "yawspeed": 0.0,
                "repr_offset_q": [1.0, 0.0, 0.0, 0.0],
            }
        return None

    async def _emit(self, system: System, msg_id: int) -> None:
        built = self._message_fields(msg_id)
        if built is not None:
            await self._send(system, *built)

    async def _stream_loop(self, system: System, msg_id: int) -> None:
        """Send msg_id at its current interval; wakes early when its interval changes."""
        changed = self._interval_changed.setdefault(msg_id, asyncio.Event())
        while True:
            try:
                interval_us = self._intervals.get(msg_id, 0)
                if interval_us > 0:
                    await self._emit(system, msg_id)
                    timeout = interval_us / 1e6
                else:
                    timeout = None  # not streamed: wait for a SET_MESSAGE_INTERVAL
                changed.clear()
                try:
                    await asyncio.wait_for(changed.wait(), timeout)
                except asyncio.TimeoutError:
                    pass
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                log.warning("Stream loop error (msg %d): %s", msg_id, exc, exc_info=True)
                await asyncio.sleep(0.5)

    def _interval_us(self, msg_id: int) -> int:
        """MESSAGE_INTERVAL.interval_us for msg_id: -1 disabled, 0 not available, >0 interval."""
        if msg_id not in self._default_intervals:
            return 0
        current = self._intervals.get(msg_id, 0)
        return current if current > 0 else -1

    async def _apply_message_command(self, system: System, cmd: int, record: dict) -> None:
        """Act on an ACCEPTED SET_MESSAGE_INTERVAL / REQUEST_MESSAGE for a message the mock streams."""
        if self._command_results.get(cmd, MAV_RESULT_ACCEPTED) != MAV_RESULT_ACCEPTED:
            return
        p1, p2 = record["param1"], record["param2"]
        if math.isnan(p1):
            return
        msg_id = int(p1)
        if cmd == _MAV_CMD_SET_MESSAGE_INTERVAL and msg_id in self._default_intervals and not math.isnan(p2):
            interval = int(p2)
            self._intervals[msg_id] = (
                self._default_intervals[msg_id] if interval == 0 else (0 if interval < 0 else interval)
            )
            if msg_id in self._interval_changed:
                self._interval_changed[msg_id].set()
            log.debug("Stream interval msg=%d -> %d us", msg_id, self._intervals[msg_id])
        elif cmd == _MAV_CMD_REQUEST_MESSAGE:
            if msg_id == MSG_ID_MESSAGE_INTERVAL and not math.isnan(p2):
                await self._send(system, "MESSAGE_INTERVAL", {
                    "message_id": int(p2), "interval_us": self._interval_us(int(p2)),
                })
            else:
                await self._emit(system, msg_id)
