# MAVLink Protocol Tests

Python-based protocol tests for the MAVLink common message set using [MAVSDK-Python](https://github.com/mavlink/MAVSDK-Python) as the transport library.

The test suite validates both the **client (GCS)** and **server (drone)** sides of each MAVLink service, and runs both sides against each other using a built-in mock drone — no external simulator required.

## Requirements

- Python 3.10+
- `mavsdk >= 2.0.0` (includes `mavsdk_server` binary)
- `pytest >= 8.0.0`
- `pytest-asyncio >= 0.23.0`

Install dependencies:

```bash
pip install -r requirements.txt
```

## Supported dialects

The default dialect is `common.xml`.
MAVSDK-Python includes common.xml support out of the box.
To load a custom dialect that includes common.xml, pass its XML to `mavlink_direct.load_custom_xml()` inside your test fixture.

## Running the tests

### Mock mode (no external drone needed)

```bash
pytest tests/
```

Starts two local `mavsdk_server` processes over loopback UDP and runs `MockFlightStack` as the drone-side handler.
All tests run without any external simulator (the skips are Tier 2 execution tests and stack-specific probes that require a real flight stack).

**Known scaling limitation**: at the current test count (~345, up from 242 tests when this suite was smaller), a single `pytest tests/` session accumulates enough function-scoped `System`/gRPC channels over its ~13-minute run that connections in the last few files reliably start erroring out (confirmed: the exact same tests pass cleanly when their file/subtree is run on its own — this is a resource-accumulation artifact of one very long combined session, not a code defect). Root `CLAUDE.md`'s design decision #3 already documents the same class of issue for `test_frame_types.py`'s 65 tests alone. Until the client-test fixtures are revisited for this scale (tracked as future work, not yet done), verify cleanly by running the two top-level subtrees separately:

```bash
pytest tests/command/   # expect 110 passed, 58 skipped, 22 xfailed (verified 2026-09-14 — nav_takeoff Tier 2 grew 5 new characterisation/honoured tests; occasional single-test flakiness in do_set_global_origin's GPS_GLOBAL_ORIGIN dedup check — unrelated, pre-existing, a genuinely intermittent timing race, not a regression; re-run in isolation if seen)
pytest tests/mission/   # expect 153 passed, 16 skipped, 3 xfailed (verified 2026-09-14 — nav_takeoff Tier 2 grew 4 new characterisation tests)
```

### Against a real drone or simulator

```bash
# PX4 SITL (UDP port 14540 — start PX4 with PX4_SIM_MODEL=sihsim_quadx first)
pytest tests/ --drone-address=udp://:14540

# ArduCopter SITL (TCP port 5760 — requires --home-lat/lon for the SITL home position)
pytest tests/ \
  --drone-address=tcp://127.0.0.1:5760 \
  --connection-timeout=60 \
  --home-lat=37.6234 --home-lon=-122.0811 --home-alt=0

# Let the suite start/stop ArduCopter SITL automatically
pytest tests/ \
  --drone-address=tcp://127.0.0.1:5760 \
  --connection-timeout=60 \
  --ardupilot-sitl=~/ardu_sitl/arducopter \
  --home-lat=37.6234 --home-lon=-122.0811 --home-alt=0

# Serial connection
pytest tests/ --drone-address=serial:///dev/ttyUSB0:57600

# Increase connection timeout for slow links (default 30 s)
pytest tests/ --drone-address=udp://:14540 --connection-timeout=60
```

### Running a subset

```bash
# Frame-type support matrix only (21 frames × 3 mission types)
pytest tests/mission/test_frame_types.py -v

# Protocol conformance tests only
pytest tests/mission/test_protocol_conformance.py -v

# Client tests only
pytest tests/mission/test_mission_client.py --drone-address=udp://:14540

# Server tests only (always use mock, ignore --drone-address)
pytest tests/mission/test_mission_server.py

# NAV_TAKEOFF tests only (Tier 1 + Tier 2)
pytest tests/mission/nav_takeoff/ -v --log-cli-level=INFO
pytest tests/mission/nav_takeoff/ --drone-address=udp://:14540 -v --log-cli-level=INFO

# DO_REPOSITION tests only (Tier 1 — rejected as a mission item on every stack; see tests/mission/do_reposition/README.md)
pytest tests/mission/do_reposition/ -v --log-cli-level=INFO

# CONDITION_GATE tests only (Tier 1 + Tier 2 — <wip/> command needing the raw mavlink_direct
# transport; see tests/mission/condition_gate/README.md)
pytest tests/mission/condition_gate/ -v --log-cli-level=INFO
```

### Verbose output with protocol logging

```bash
pytest tests/ -v --log-cli-level=INFO
```

## Mission plan files

Plans are stored as JSON in `tests/mission/plans/`.
Each file contains:

- `mission_type`: integer matching `MAV_MISSION_TYPE` (0=mission, 1=fence, 2=rally)
- `items`: list of MISSION_ITEM_INT fields

Fields with leading underscores (`_comment`, `_description`, etc.) are documentation-only and ignored by the loader.

| File | Contents |
|------|----------|
| `simple_mission.json` | 4-item flight plan: takeoff → 2 waypoints → RTL |
| `simple_geofence.json` | Fence return point + 4-vertex inclusion polygon |
| `simple_rally.json` | 2 rally (emergency landing) points |

Coordinates are centred on **47.3977 N, 8.5456 E** (Zurich, Switzerland) at low altitude; they are safe to use against any simulator.

## Protocol timeouts

These values are defined by the MAVLink mission protocol specification and applied internally by MAVSDK:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `TIMEOUT_INITIAL_RESPONSE` | 1500 ms | Wait for MISSION_COUNT or first MISSION_REQUEST_INT |
| `TIMEOUT_ITEM_RESPONSE` | 250 ms | Per-item wait between MISSION_REQUEST_INT / MISSION_ITEM_INT |
| `MAX_RETRIES` | 5 | Retransmissions before cancellation |

Worst-case transfer time for *N* items: `(N + 1) × MAX_RETRIES × max(TIMEOUT_INITIAL_RESPONSE, TIMEOUT_ITEM_RESPONSE)`.

## Project structure

```
.
├── conftest.py                    # CLI options (--drone-address, --connection-timeout, etc.)
├── pytest.ini                     # asyncio_mode=auto, markers
├── requirements.txt
├── README.md
├── CLAUDE.md                      # Implementation notes for Claude
└── tests/
    ├── conftest.py                # GCS/drone System fixtures; MockFlightStack integration
    ├── mock_flight_stack.py       # MAVLink drone simulator (upload/download/clear/capability)
    └── mission/
        ├── conftest.py            # Plan loaders, comparison helpers, home-slot detection
        ├── plans/
        │   ├── simple_mission.json
        │   ├── simple_geofence.json
        │   └── simple_rally.json
        ├── test_mission_client.py # GCS-side tests (upload, download, capability)
        ├── test_mission_server.py # Drone-side tests (receive, clear, paired)
        ├── test_frame_types.py    # MAV_FRAME support matrix (21 frames × 3 mission types)
        ├── test_protocol_conformance.py  # Normative spec-conformance tests
        ├── nav_takeoff/
        │   ├── test_protocol.py           # NAV_TAKEOFF param acceptance (Tier 1, Tier1MissionTestBase)
        │   └── test_flight.py             # NAV_TAKEOFF execution tests (Tier 2, requires real stack)
        ├── do_reposition/
        │   └── test_protocol.py           # DO_REPOSITION mission-item acceptance (Tier 1, Tier1MissionTestBase — UNSUPPORTED everywhere; no Tier 2, see README)
        └── condition_gate/
            ├── test_protocol.py           # CONDITION_GATE mission-item acceptance (Tier 1 — raw transport, <wip/> command)
            └── test_flight.py             # CONDITION_GATE execution tests (Tier 2, PX4 only, requires real stack)
```

## Adding new tests

1. Create a new subdirectory under `tests/` (e.g. `tests/heartbeat/`).
2. Add `__init__.py` and a `conftest.py` for any service-specific fixtures.
3. Name client tests `test_<service>_client.py` and server tests `test_<service>_server.py`.
4. Update `CLAUDE.md` with any protocol behaviour notes specific to the new service.

## Known autopilot behaviour

Results below are from the current test suite run against each stack.

### PX4 (mainline, SIH simulator)

| Test group | Result |
|------------|--------|
| Capability | PASS |
| Flight mission upload/download | PASS |
| Flight mission roundtrip | XFAIL — PX4 converts frame on storage |
| Geofence upload/download | PASS |
| Geofence roundtrip | XFAIL — PX4 converts frame=0→5 on storage |
| Rally upload/download | PASS |
| Rally roundtrip | XFAIL — PX4 converts frame=0→5 on storage |
| Clear mission | PASS |
| Protocol conformance (no home slot required) | PASS |
| Frame support — flight/rally | All accepted frames preserve altitude category |
| Frame support — geofence | FAIL for frames 3 and 6 — PX4 stores all geofence items as GLOBAL_INT, losing relative-alt |
| NAV_TAKEOFF param acceptance | See MAV_CMD table below |
| DO_REPOSITION mission-item acceptance | UNSUPPORTED (rejected outright, spec-aligned) — see MAV_CMD table below |

**Accepted frames (flight and rally):** GLOBAL (0), GLOBAL_RELATIVE_ALT (3), GLOBAL_INT (5), GLOBAL_RELATIVE_ALT_INT (6); others REJECTED.
MAV_FRAME_MISSION accepted for DO commands only.
Geofence shares the same accepted set but frames 3 and 6 are stored incorrectly (see above).

### ArduCopter (V4.8.0-dev, SITL)

| Test group | Result |
|------------|--------|
| Capability | PASS |
| Flight mission upload/download | PASS |
| Flight mission roundtrip | XFAIL — ArduCopter converts all frames to GLOBAL on storage |
| Geofence upload/download | PASS |
| Geofence roundtrip | PASS — ArduCopter preserves frame |
| Rally upload/download | PASS |
| Rally roundtrip | PASS — ArduCopter preserves frame |
| Clear mission | FAIL — ArduCopter retains home waypoint after clear |
| Protocol conformance (no home slot required) | FAIL — ArduCopter requires home at seq=0 (spec violation) |
| MAV_FRAME_MISSION DO_CHANGE_SPEED param1 | FAIL — ArduCopter zeroes param1 (spec violation) |
| Frame support — flight/geofence | See frame table; frames 3/6/10/11 change altitude category → FAIL |
| Frame support — rally | All accepted frames preserve altitude category |
| NAV_TAKEOFF param acceptance | See MAV_CMD table below |
| DO_REPOSITION mission-item acceptance | UNSUPPORTED (rejected outright, spec-aligned) — see MAV_CMD table below |

**Home-slot requirement:** ArduCopter reserves seq=0 for the home position.
The test suite auto-detects this and prepends a home item (using `--home-lat`/`--home-lon`/`--home-alt`) so frame tests can probe real frame support rather than hitting the home-slot rejection.

### MAV_CMD protocol acceptance (Tier 1)

Round-trip evidence is asymmetric: a param that **is** preserved on download was stored correctly for that specific test value, but does not confirm the autopilot acts on it during execution.
A param that is **not** preserved was silently altered; the stack should have NACKed instead.
Full per-param tables, source verification, and Tier 2 results live in each command's own `README.md`/`CLAUDE.md` — this is a bottom-line index only.

| Command | PX4 | ArduCopter/ArduPlane | Details |
|---------|-----|----------------------|---------|
| `MAV_CMD_NAV_TAKEOFF` (22) | Stores Yaw + location; never stores Pitch/Flags (v1.17.0, confirmed 2026-09-14) — a newer 1.18.0-beta dev build (2026-09-13) instead actively rejects non-default Pitch/Flags/unused values, so this appears to be a version-dependent validation change, not settled behaviour. MC takes off correctly via mission upload (v1.17.0); **fixed-wing does not** — accepts the mission but never climbs within 90 s, a genuine spec-compliance FAIL, not yet root-caused (v1.17.0, 2026-09-14). VTOL untested — blocked by a sandbox resource issue, see root `CLAUDE.md` item #10 | Stores Pitch + location only; rejects NaN in any param but Yaw and rejects the `INT32_MAX` location sentinel (spec violations) | [`nav_takeoff/README.md`](tests/mission/nav_takeoff/README.md) |
| `MAV_CMD_DO_REPOSITION` (192) | `UNSUPPORTED` — rejected outright as a mission item, on every vehicle type | Same | [`do_reposition/README.md`](tests/mission/do_reposition/README.md) |
| `MAV_CMD_CONDITION_GATE` (4501) | Accepted (`<wip/>` tag needs the raw `mavlink_direct` transport — `mission_raw` blocks it client-side); never stores Geometry/UseAltitude; Tier 2 confirms mavlink-devguide PR #761's crossing-point claims | `UNSUPPORTED` — not implemented ([ardupilot#13778](https://github.com/ArduPilot/ardupilot/issues/13778)) | [`condition_gate/README.md`](tests/mission/condition_gate/README.md) |

DO_REPOSITION's rejection is spec-aligned (the spec directs guided-only commands like it to COMMAND_INT, not missions — see `tests/command/do_reposition/`), so it has no Tier 2 mission-flight test.
Both PX4 and ArduPilot's mission-command recognition switches are frame/vehicle-type independent — results generalise across multicopter/fixed-wing/VTOL within each firmware family unless a table says otherwise.

### ArduPlane / QuadPlane (V4.8.0-dev, SITL)

ArduPlane (fixed-wing) and QuadPlane (VTOL) use the same `arduplane` binary; results are identical for both vehicle types.

| Test group | Result |
|------------|--------|
| NAV_TAKEOFF param acceptance | See MAV_CMD table above |
| DO_REPOSITION mission-item acceptance | See MAV_CMD table above — UNSUPPORTED, identical to ArduCopter |

ArduPlane does **not** require a home item at seq=0 (unlike ArduCopter).
NAV_TAKEOFF storage behaviour mirrors ArduCopter: param1 (Pitch) preserved; params 3 and 4 zeroed; param2 NaN rejected (spec violation).

### Mock (MockFlightStack, no external drone)

The mock accepts every command and frame, stores items exactly as received, and serves them unchanged on download — use it to verify protocol-level interactions without a real autopilot. See "Known scaling limitation" above for current pass/skip/xfail counts (skips are Tier 2/stack-specific probes needing a real flight stack).

## Spec violations

Confirmed deviations from the MAVLink mission protocol spec are flagged inline (bold or an explicit FAIL/✗ marker) in the frame tables above and in each command's own results table — see `tests/mission/README.md`'s frame tables and each `tests/mission/<command>/README.md` for the full list with the failing test names. Highlights: PX4's geofence altitude-reference bug (frames 3/6 → `GLOBAL_INT`, losing relative-alt); ArduCopter's home-slot requirement, non-empty `clear_mission()`, and altitude-reference loss on frames 3/6/10/11; both stacks' NAV_TAKEOFF param-storage/NaN/`INT32_MAX` gaps (see the table above).
