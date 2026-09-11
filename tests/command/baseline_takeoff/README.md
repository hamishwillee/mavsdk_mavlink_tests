# Baseline Takeoff Tests

Minimum verified takeoff sequence for each supported (stack, vehicle-type) combination. Run before the higher-level tests in `tests/command/nav_takeoff/test_flight.py`.

## Tests

| Test | Stack | Vehicle | Command used | Result (2026-06-02) |
|------|-------|---------|--------------|---------------------|
| `test_ardupilot_mc_takeoff_baseline` | ArduCopter 4.8.0-dev | Multicopter | NAV_TAKEOFF (22) COMMAND_LONG | **PASS** — 17.1 m |
| `test_px4_mc_takeoff_baseline` | PX4 1.18.0 | Multicopter | NAV_TAKEOFF (22) COMMAND_INT | **PASS** — 17.0 m |
| `test_px4_vtol_takeoff_baseline` | PX4 1.18.0 | VTOL | NAV_VTOL_TAKEOFF (84) preferred, NAV_TAKEOFF fallback | **PASS** — 17.0 m (MC hover) |
| `test_ardupilot_quadplane_takeoff_baseline` | ArduPlane 4.8.0-dev | QuadPlane | NAV_TAKEOFF (22) COMMAND_LONG in GUIDED (15) | **PASS** — 17.3 m |

## Command selection for VTOL frames

**PX4 VTOL — NAV_VTOL_TAKEOFF (84) preferred.** Directly supported as COMMAND_INT; `vtol_takeoff.cpp` runs TAKEOFF_HOVER (climb in MC mode) → ALIGN_HEADING → TRANSITION (to FW) → CLIMB (in FW mode, to `loiter_altitude + LOITER_ALT_OFFSET`). The test only requires reaching the airborne threshold (5 m) during the MC hover phase — the full sequence can take 60–120s. Falls back to NAV_TAKEOFF (22) if NAV_VTOL_TAKEOFF returns UNSUPPORTED (non-VTOL-capable frame).

**ArduPlane QuadPlane — NAV_VTOL_TAKEOFF is mission-only.** `commands_logic.cpp` only handles it during AUTO mission execution; a direct COMMAND_INT returns FAILED. Correct sequence: `DO_SET_MODE GUIDED (15) → arm → COMMAND_LONG NAV_TAKEOFF (22) p7=alt`. GUIDED (custom_mode=15) sets `guided_takeoff=true` on NAV_TAKEOFF (`quadplane.cpp:4039`) and `in_vtol_mode()` returns true, engaging the VTOL attitude controller for a vertical climb — the same sequence `quadplane.py`'s autotest uses (`takeoff(mode="GUIDED")` → `change_mode("GUIDED")` → `user_takeoff()`). QHOVER (18)/QLOITER (19) don't work for autonomous MAVLink takeoff — their throttle comes from the RC channel (`get_pilot_desired_climb_rate_ms()`).

## ArduCopter / ArduPlane mode restriction

See root `tests/command/CLAUDE.md` § ArduCopter mode restriction for NAV_TAKEOFF.

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
