# Baseline Takeoff Tests

Minimum verified takeoff sequences for each supported (stack, vehicle-type) combination.
Run these before the higher-level execution tests in `tests/command/nav_takeoff/test_flight.py`.

## Tests

| Test | Stack | Vehicle | Command used | Result (2026-06-02) |
|------|-------|---------|--------------|---------------------|
| `test_ardupilot_mc_takeoff_baseline` | ArduCopter 4.8.0-dev | Multicopter | NAV_TAKEOFF (22) COMMAND_LONG | **PASS** — 17.1 m |
| `test_px4_mc_takeoff_baseline` | PX4 1.18.0 | Multicopter | NAV_TAKEOFF (22) COMMAND_INT | **PASS** — 17.0 m |
| `test_px4_vtol_takeoff_baseline` | PX4 1.18.0 | VTOL | NAV_VTOL_TAKEOFF (84) preferred, NAV_TAKEOFF fallback | **PASS** — 17.0 m (MC hover) |
| `test_ardupilot_quadplane_takeoff_baseline` | ArduPlane 4.8.0-dev | QuadPlane | NAV_TAKEOFF (22) COMMAND_LONG in GUIDED (15) | **PASS** — 17.3 m |

## Command selection for VTOL frames

### PX4 VTOL — NAV_VTOL_TAKEOFF (84) preferred

`MAV_CMD_NAV_VTOL_TAKEOFF (84)` is directly supported as a COMMAND_INT on PX4 VTOL.
It executes a full VTOL sequence via `vtol_takeoff.cpp`:

1. **TAKEOFF_HOVER**: climb to takeoff altitude in MC mode
2. **ALIGN_HEADING**: rotate to face the loiter destination
3. **TRANSITION**: transition to fixed-wing flight
4. **CLIMB**: continue climbing in FW mode to `loiter_altitude + LOITER_ALT_OFFSET`

The test passes once the initial MC hover phase reaches the airborne threshold (5 m), without requiring the full FW transition.
The full sequence may take 60–120 s.

If `NAV_VTOL_TAKEOFF` returns `UNSUPPORTED` (e.g. the frame is not VTOL-capable), the test falls back to `NAV_TAKEOFF (22)`.

### ArduPlane QuadPlane — NAV_VTOL_TAKEOFF is mission-only

On ArduPlane QuadPlane, `NAV_VTOL_TAKEOFF (84)` is handled in `commands_logic.cpp` during AUTO mode mission execution only.
Sending it as a direct COMMAND_INT returns FAILED.
The correct sequence for a direct autonomous VTOL takeoff is:

```
DO_SET_MODE GUIDED (mode=15)  →  arm  →  COMMAND_LONG NAV_TAKEOFF (22) p7=alt
```

ArduPlane QuadPlane's GUIDED mode (custom_mode=15) activates the VTOL position controller: `quadplane.cpp:4039` sets `guided_takeoff=true` on NAV_TAKEOFF receipt, and `in_vtol_mode()` returns true, triggering the quadplane attitude controller.
The vehicle climbs vertically using its VTOL motors.

The quadplane autotest (quadplane.py) uses the same sequence: `takeoff(height, mode="GUIDED")` → change_mode("GUIDED") → user_takeoff().

QHOVER (18) and QLOITER (19) are NOT suitable for autonomous MAVLink takeoff because their throttle comes from `get_pilot_desired_climb_rate_ms()` (RC channel).

## ArduCopter / ArduPlane mode restriction

With param3=0 (default, `must_navigate=True`), only GUIDED (4 / 15), LOITER (5), and POSHOLD (16) accept NAV_TAKEOFF.
But only GUIDED uses `_AutoTakeoff::run()` (autonomous).
See `tests/command/baseline_takeoff/test_baseline.py` module docstring for the full table.

## Running

```bash
# ArduCopter MC
pytest tests/command/baseline_takeoff/ \
    --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \
    --ardupilot-sitl=~/ardu_sitl/arducopter \
    --home-lat=37.6234 --home-lon=-122.0811 --home-alt=0 \
    --vehicle-type=quadcopter --autopilot=ardupilot -v --log-cli-level=INFO

# PX4 MC
pytest tests/command/baseline_takeoff/ \
    --drone-address=udp://:14540 --connection-timeout=60 \
    --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx \
    --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO

# PX4 VTOL
pytest tests/command/baseline_takeoff/ \
    --drone-address=udp://:14540 --connection-timeout=60 \
    --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_standard_vtol \
    --vehicle-type=vtol --autopilot=px4 -v --log-cli-level=INFO

# ArduPlane QuadPlane
pytest tests/command/baseline_takeoff/ \
    --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \
    --ardupilot-sitl=~/ardu_sitl/arduplane --ardupilot-model=quadplane \
    --vehicle-type=quadplane --autopilot=ardupilot -v --log-cli-level=INFO
```
