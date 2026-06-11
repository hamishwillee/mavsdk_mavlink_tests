# SITL setup

## ArduCopter SITL

```bash
# Download pre-built binary (one-time)
mkdir -p ~/ardu_sitl/sitl_working
curl -o ~/ardu_sitl/arducopter \
  https://firmware.ardupilot.org/Copter/latest/SITL_x86_64_linux_gnu/arducopter
chmod +x ~/ardu_sitl/arducopter

# Start SITL — must run from sitl_working; parm file path is exact (no shortcut)
cd ~/ardu_sitl/sitl_working
~/ardu_sitl/arducopter -S -I0 --model + \
  --home=37.6234,-122.0811,0,270 \
  --defaults ~/github/ArduPilot/ardupilot/Tools/autotest/default_params/copter.parm
```

The parm file path is exact — do not abbreviate or symlink it.
ArduCopter silently accepts the TCP connection then panics if the parm file is missing or wrong.

## PX4 SITL

PX4 SITL is managed automatically by the `--px4-sitl` fixture.
No manual download needed beyond cloning PX4-Autopilot and running the standard build (`make px4_sitl_default`).

See `CLAUDE.md` for pitfall notes (two-space readiness string, 64 KB log read limit).
