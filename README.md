# MAVLink Protocol Tests

Python-based protocol tests for the MAVLink common message set using [MAVSDK-Python](https://github.com/mavlink/MAVSDK-Python) as the transport library.

The test suite validates both the **client (GCS)** and **server (drone)** sides of each MAVLink service, and can run the two sides against each other using a local mock drone.

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

The default dialect is `common.xml`.  MAVSDK-Python includes common.xml support out of the box.  To load a custom dialect that includes common.xml, pass its XML to `mavlink_direct.load_custom_xml()` inside your test fixture.

## Running the tests

### Against a real drone or simulator (standalone client tests)

```bash
# PX4 SITL on UDP port 14540 (default)
pytest tests/mission/test_mission_client.py --drone-address=udp://:14540

# ArduPilot SITL
pytest tests/mission/test_mission_client.py --drone-address=udp://:14550

# Serial connection
pytest tests/mission/test_mission_client.py --drone-address=serial:///dev/ttyUSB0:57600

# Increase connection timeout for slow links (default 30 s)
pytest tests/mission/ --drone-address=udp://:14540 --connection-timeout=60
```

### Server and paired tests (no external drone needed)

```bash
# Server tests + the paired GCS message-format test
pytest tests/mission/test_mission_server.py tests/mission/test_mission_client.py::TestDeprecatedMessageHandling
```

These tests start two local `mavsdk_server` processes over loopback UDP and exercise both sides within each test.  No external simulator is required.

> **Note:** Running `pytest tests/mission/` (all tests) without `--drone-address` will cause the 10 client-only tests to fail because the mock drone has no mission protocol handler unless a `mission_raw_server` subscription is active.  Client tests always require `--drone-address`.

### Running only one side

```bash
# Client (GCS) tests only — requires --drone-address
pytest tests/mission/test_mission_client.py --drone-address=udp://:14540

# Server (drone) tests only
pytest tests/mission/test_mission_server.py
```

### Skipping slow tests

```bash
pytest tests/mission/ -m "not slow"
```

### Verbose output with protocol logging

```bash
pytest tests/mission/ -v --log-cli-level=DEBUG
```

## Mission plan files

Plans are stored as JSON in `tests/mission/plans/`.  Each file contains:

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
├── conftest.py                    # CLI options (--drone-address, --connection-timeout)
├── pytest.ini                     # asyncio_mode=auto, markers
├── requirements.txt
├── README.md
├── CLAUDE.md                      # Implementation notes for Claude
└── tests/
    ├── conftest.py                # GCS and drone System fixtures
    └── mission/
        ├── conftest.py            # Plan loaders, item-comparison helpers
        ├── plans/
        │   ├── simple_mission.json
        │   ├── simple_geofence.json
        │   └── simple_rally.json
        ├── test_mission_client.py # GCS-side tests (upload, download, capability)
        └── test_mission_server.py # Drone-side tests (receive, clear, paired)
```

## Adding new tests

1. Create a new subdirectory under `tests/` (e.g. `tests/heartbeat/`).
2. Add `__init__.py` and a `conftest.py` for any service-specific fixtures.
3. Name client tests `test_<service>_client.py` and server tests `test_<service>_server.py`.
4. Update `CLAUDE.md` with any protocol behaviour notes specific to the new service.

## Known implementation differences

| Autopilot | Notes |
|-----------|-------|
| PX4 | Full flight-plan and geofence support; rally points not supported; cancellation works for download only |
| ArduPilot | Non-atomic uploads (partial state on failure); item 0 is home position; float rounding on storage; cannot clear mission in Auto mode |
| QGroundControl (server) | Reference implementation for all three mission types |
