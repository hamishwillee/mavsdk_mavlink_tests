"""
Thin compatibility layer between this suite and MAVSDK-Python 4.x (the native
C++ binding, `mavsdk` >= 4).

MAVSDK 4 replaced the gRPC wrapper (a `mavsdk_server` subprocess per System,
plugins as attributes, async-generator streams named after the data) with an
in-process binding: one `Mavsdk` instance per MAVLink identity, plugins built
explicitly per System (`TelemetryAsync(system)`), and streams exposed as
`subscribe_<name>()` async generators. This module keeps the suite's existing
call style (`system.telemetry.position()`, `system.mavlink_direct.message(...)`,
`system.mission_raw.upload_mission(...)`) working on top of that, so test
bodies didn't need rewriting — only the harness fixtures that created
connections changed.

- `Endpoint`: one `Mavsdk` instance with one MAVLink identity (sysid/compid)
  and one connection URL. Session-lifetime — v4 subscriptions capture the
  running event loop when each subscription starts, so one Endpoint serves
  every test's (function- or class-scoped) event loop.
- `SystemShim`: what fixtures hand to tests in place of the old
  `mavsdk.System`. Plugin attributes resolve to v4 plugin instances (cached per
  underlying System, so every shim shares one plugin instance), with the old
  stream names mapped onto `subscribe_*`.
"""

from __future__ import annotations

import asyncio
import logging
import time
from pathlib import Path
from types import SimpleNamespace

from mavsdk import logging as _mavsdk_logging
from mavsdk.asyncio import ComponentType, ConnectionResult, Configuration, Mavsdk
from mavsdk.asyncio.plugins.action import ActionAsync
from mavsdk.asyncio.plugins.info import InfoAsync
from mavsdk.asyncio.plugins.mavlink_direct import MavlinkDirectAsync
from mavsdk.asyncio.plugins.mission import MissionAsync
from mavsdk.asyncio.plugins.mission_raw import MissionRawAsync
from mavsdk.asyncio.plugins.mission_raw_server import MissionRawServerAsync
from mavsdk.asyncio.plugins.param import ParamAsync
from mavsdk.asyncio.plugins.telemetry import TelemetryAsync

log = logging.getLogger(__name__)

# The GCS identity: MAVSDK's own GROUND_STATION configuration (sysid 245,
# compid 190). It has to be a real ground-station config, not
# create_manual(255, 1) like the gRPC-era harness used: with a compid-1
# (autopilot-class) GCS, MAVSDK 4 never reports a PX4 vehicle armable — every
# health flag OK but is_armable False for 40 s+ — while a GROUND_STATION GCS
# sees it armable within ~1 s (PX4 v1.17 gz VTOL, 2026-09-29). Raw messages the
# suite builds itself (mavlink_direct) use these ids too.
GCS_SYSID, GCS_COMPID = 245, 190


# ---------------------------------------------------------------------------
# Native MAVSDK log output -> its own file under logs/, never stdout
# ---------------------------------------------------------------------------
# The native library logs to stdout by default (discovery, warnings like
# "Received ack for not-existing command"), which would land in pytest's
# output. Same convention as the SITL consoles and the old mavsdk_server
# output: its own file under logs/.

_log_fh = None


def route_mavsdk_logs(logs_dir: Path = Path("logs")) -> Path:
    """Send all native MAVSDK log output to logs/mavsdk_<timestamp>.log. Idempotent."""
    global _log_fh
    if _log_fh is not None:
        return Path(_log_fh.name)
    logs_dir.mkdir(exist_ok=True)
    path = logs_dir / f"mavsdk_{time.strftime('%Y%m%d_%H%M%S')}.log"
    _log_fh = open(path, "a", buffering=1)

    def _to_file(level, message, file, line) -> bool:
        _log_fh.write(f"{time.strftime('%H:%M:%S')} {getattr(level, 'name', level)} {message} ({file}:{line})\n")
        return True  # handled — suppress the library's own stdout print

    _mavsdk_logging.log_subscribe(_to_file)
    return path


# ---------------------------------------------------------------------------
# Plugin shims
# ---------------------------------------------------------------------------

class _PluginShim:
    """
    Old-API view of a v4 async plugin. An attribute `x` resolves to the
    plugin's `subscribe_x` when that exists (old streams were named after the
    data: `telemetry.position()`, `mission_raw.mission_progress()`), else to
    the plugin's own `x` (one-shot calls are unchanged: `upload_mission`,
    `arm`, `get_version`, ...). `extra` maps any name that doesn't follow that
    rule.
    """

    def __init__(self, plugin, extra: dict | None = None):
        self._plugin = plugin
        self._extra = extra or {}

    def __getattr__(self, name):
        if name in self._extra:
            return getattr(self._plugin, self._extra[name])
        sub = getattr(self._plugin, f"subscribe_{name}", None)
        if sub is not None:
            return sub
        return getattr(self._plugin, name)


class _CoreShim:
    """Old `system.core.connection_state()`: yields objects with `.is_connected`, current state first."""

    def __init__(self, system):
        self._system = system

    async def connection_state(self):
        yield SimpleNamespace(is_connected=await self._system.is_connected())
        async for connected in self._system.is_connected_state():
            yield SimpleNamespace(is_connected=connected)


# One plugin instance per (System, plugin class), shared by every shim for that
# System. Several native instances of e.g. MissionRawServer on one component
# each run their own protocol state machine: one completes a transfer (and
# ACKs the GCS) while the one a test subscribed to times out — observed
# 2026-09-29 before Endpoint started caching its System/server component
# (v4 returns a NEW wrapper object on every first_autopilot()/get_systems()/
# server_component() call, so keying on id() of a fresh wrapper each test
# created a fresh native plugin each test).
_PLUGINS: dict[tuple[int, type], object] = {}


def _plugin(owner, cls):
    key = (id(owner), cls)
    if key not in _PLUGINS:
        _PLUGINS[key] = cls(owner)
    return _PLUGINS[key]


class SystemShim:
    """
    Stand-in for the old gRPC `mavsdk.System`. `system` is the v4 asyncio
    System (the remote peer); `server_component` is this endpoint's own
    component, needed only for server plugins (`mission_raw_server`).
    """

    def __init__(self, system, server_component=None):
        self._system = system
        self._server_component = server_component
        # Bumped by Endpoint.reopen() (e.g. after a flight-stack restart):
        # subscriptions made before it are on the destroyed connection, and
        # anything requested from the vehicle (message rates) was forgotten.
        # tests/message_watcher.ensure_harness_streams() keys on it.
        self.generation = 0

    async def connect(self, *args, **kwargs) -> None:
        """No-op — kept so old `await system.connect()` call sites still work."""

    @property
    def raw(self):
        """The underlying v4 asyncio System, for anything this shim doesn't cover."""
        return self._system

    @property
    def core(self):
        return _CoreShim(self._system)

    @property
    def telemetry(self):
        return _PluginShim(_plugin(self._system, TelemetryAsync))

    @property
    def action(self):
        return _PluginShim(_plugin(self._system, ActionAsync))

    @property
    def mission(self):
        return _PluginShim(_plugin(self._system, MissionAsync))

    @property
    def mission_raw(self):
        return _PluginShim(_plugin(self._system, MissionRawAsync))

    @property
    def mavlink_direct(self):
        # Old: mavlink_direct.message(name) stream.
        return _PluginShim(_plugin(self._system, MavlinkDirectAsync), {"message": "subscribe_message"})

    @property
    def param(self):
        return _PluginShim(_plugin(self._system, ParamAsync))

    @property
    def info(self):
        return _PluginShim(_plugin(self._system, InfoAsync))

    @property
    def mission_raw_server(self):
        if self._server_component is None:
            raise AttributeError("mission_raw_server needs a server-side (drone) endpoint")
        # Note: incoming_mission() now yields (MissionRawServerResult, MissionPlan)
        # tuples — the old gRPC API's "plan discarded on SUCCESS" bug is gone.
        return _PluginShim(_plugin(self._server_component, MissionRawServerAsync))


# ---------------------------------------------------------------------------
# Endpoints
# ---------------------------------------------------------------------------

class Endpoint:
    """
    One MAVSDK instance with one MAVLink identity and one connection. The GCS
    side talks to an autopilot (`autopilot()`); the paired mock drone side
    talks to the GCS it sees (`peer()`).
    """

    def __init__(self, name: str, sysid: int, compid: int, url: str):
        """(sysid, compid) == (GCS_SYSID, GCS_COMPID) means "the GCS": a real
        GROUND_STATION configuration (see GCS_SYSID). Anything else is created
        manually (the paired mock drone: 1/1)."""
        self.name, self.sysid, self.compid, self.url = name, sysid, compid, url
        self._shim: SystemShim | None = None  # cached: see _PLUGINS
        self._open()

    def _open(self) -> None:
        route_mavsdk_logs()
        if (self.sysid, self.compid) == (GCS_SYSID, GCS_COMPID):
            config = Configuration.create_with_component_type(ComponentType.GROUND_STATION)
        else:
            config = Configuration.create_manual(self.sysid, self.compid, True)
        self.mavsdk = Mavsdk(config)
        # The sync call on the wrapped instance: this runs from session-scoped
        # (non-async) fixtures, and adding a connection doesn't block.
        result = self.mavsdk._mavsdk.add_any_connection(self.url)
        if result != ConnectionResult.SUCCESS:
            raise RuntimeError(f"{self.name}: add_any_connection({self.url}) failed: {result}")
        log.info("MAVSDK endpoint %s up (sysid=%d compid=%d, %s)", self.name, self.sysid, self.compid, self.url)

    def reopen(self, timeout_s: float = 30.0) -> None:
        """
        Tear down and recreate the connection (e.g. after the flight stack was
        restarted), and rebind the existing SystemShim — if any — to the new
        connection's System in place. Tests hold that shim object for their
        whole duration, so replacing it would leave them calling into the
        destroyed instance ("system handle is null", seen 2026-09-30 when a
        mid-test restart was followed by the same test's cleanup).
        """
        shim = self._shim
        self.close()  # also drops the shim's cached plugins
        self._open()
        if shim is None:
            return
        # Synchronous discovery: this runs from the synchronous restart fixture.
        from mavsdk.asyncio.system import System as _AsyncSystem
        system = self.mavsdk._mavsdk.first_autopilot(timeout_s)
        if system is None:
            raise RuntimeError(f"{self.name}: no autopilot rediscovered within {timeout_s}s after reopen")
        shim._system = _AsyncSystem(system)
        shim.generation += 1
        self._shim = shim

    def close(self) -> None:
        if self._shim is not None:
            for owner in (self._shim._system, self._shim._server_component):
                for key in [k for k in _PLUGINS if k[0] == id(owner)]:
                    _PLUGINS.pop(key)
            self._shim = None
        self.mavsdk.destroy()

    async def autopilot(self, timeout_s: float = 30.0) -> SystemShim:
        """The first autopilot this endpoint has discovered (GCS side)."""
        if self._shim is None:
            system = await self.mavsdk.first_autopilot(timeout_s=timeout_s)
            if system is None:
                raise asyncio.TimeoutError(f"{self.name}: no autopilot discovered within {timeout_s}s ({self.url})")
            self._shim = SystemShim(system)
        return self._shim

    async def peer(self, timeout_s: float = 30.0) -> SystemShim:
        """The first remote system this endpoint has discovered (paired mock drone side: the GCS)."""
        if self._shim is not None:
            return self._shim
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout_s
        while True:
            systems = await self.mavsdk.get_systems()
            if systems:
                break
            if loop.time() >= deadline:
                raise asyncio.TimeoutError(f"{self.name}: no peer discovered within {timeout_s}s ({self.url})")
            await asyncio.sleep(0.1)
        self._shim = SystemShim(systems[0], await self.mavsdk.server_component())
        return self._shim


# Registry of the session's endpoints, for code that needs one without having
# it passed as a fixture (e.g. class-scoped fixtures that used to connect to a
# fixed gRPC port constant).
ENDPOINTS: dict[str, Endpoint] = {}


async def open_system(endpoint: Endpoint, timeout_s: float = 30.0) -> SystemShim:
    """A System for this endpoint: its autopilot (GCS side) or its peer (drone side)."""
    if endpoint.name.endswith("drone"):
        return await endpoint.peer(timeout_s)
    return await endpoint.autopilot(timeout_s)


async def open_paired_drone(timeout_s: float = 30.0) -> SystemShim:
    """The paired mock drone's view of the GCS (replaces the old DRONE_GRPC_PORT connection)."""
    return await open_system(ENDPOINTS["paired_drone"], timeout_s)
