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
"""

import asyncio
import json
import logging

from mavsdk import System
from mavsdk.mavlink_direct import MavlinkMessage

log = logging.getLogger(__name__)

# MAV_MISSION_RESULT values (common.xml)
_ACCEPTED = 0
_UNSUPPORTED_FRAME = 2

# Default: MAV_PROTOCOL_CAPABILITY_MISSION_INT (bit 2)
DEFAULT_CAPABILITY_BITS: int = 4

# MAVLink identity of the GCS peer (sysid=255, compid=1 in paired loopback)
_GCS_SYSID = 255
_GCS_COMPID = 1

# Message types the mock needs to subscribe to
_MSG_TYPES = (
    "MISSION_COUNT",        # GCS → drone: initiates upload
    "MISSION_REQUEST_LIST", # GCS → drone: initiates download
    "MISSION_REQUEST_INT",  # GCS → drone: requests one item during download
    "MISSION_ITEM_INT",     # GCS → drone: delivers one item during upload
    "MISSION_CLEAR_ALL",    # GCS → drone: clear stored missions
    "COMMAND_LONG",         # GCS → drone: capability request (cmd 512, p1=148)
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
        Simulates a slow autopilot requesting items from the GCS.
    item_response_delay_s : float
        Seconds to wait before sending each MISSION_ITEM_INT during download.
        Simulates a slow autopilot serving items to the GCS.
    drop_responses : dict[str, int] | None
        ``{message_name: N}`` — silently skip the first N outgoing messages of
        that type to force GCS retries.  Example: ``{"MISSION_REQUEST_INT": 2}``.
    rejected_frames : set[int] | None
        Frame values to reject with MISSION_ACK(UNSUPPORTED_FRAME) when seen
        in an upload.  The rejection happens after the offending item arrives.
    """

    def __init__(
        self,
        capability_bits: int = DEFAULT_CAPABILITY_BITS,
        item_request_delay_s: float = 0.0,
        item_response_delay_s: float = 0.0,
        drop_responses: dict | None = None,
        rejected_frames: set | None = None,
    ) -> None:
        self.capability_bits = capability_bits
        self.item_request_delay_s = item_request_delay_s
        self.item_response_delay_s = item_response_delay_s
        self._drop_remaining: dict[str, int] = dict(drop_responses or {})
        self.rejected_frames: set[int] = set(rejected_frames or set())
        # mission_type (int) → list of item field dicts
        self._store: dict[int, list[dict]] = {}
        self._queues: dict[str, asyncio.Queue] = {}

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
            asyncio.create_task(self._capability_loop(system)),
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

    # ----------------------------------------------------- capability loop --

    async def _capability_loop(self, system: System) -> None:
        while True:
            try:
                msg = await self._recv("COMMAND_LONG")
                fields = json.loads(msg.fields_json)
                cmd = int(fields.get("command", 0))
                param1 = float(fields.get("param1", 0.0))
                # MAV_CMD_REQUEST_MESSAGE (512) with param1=148 (AUTOPILOT_VERSION)
                if cmd == 512 and abs(param1 - 148.0) < 0.5:
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
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                log.warning("Capability handler error: %s", exc, exc_info=True)
