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

### PX4's pre-built SIH container image, without Docker

PX4 publishes its SIH simulator as a container, `px4io/px4-sitl` (`latest` tracks `main`; release tags such as `v1.18.0-rc1`).
The image is just PX4's `px4` .deb (`/opt/px4`) on Ubuntu 24.04, so the same binary runs natively on a matching host.
`scripts/fetch_px4_sitl_image.py` pulls the image from Docker Hub, unpacks `/opt/px4` and adds the `build/px4_sitl_default` layout `--px4-sitl` expects:

```bash
python3 scripts/fetch_px4_sitl_image.py                  # px4io/px4-sitl:latest → ~/px4_sitl_images/latest
python3 scripts/fetch_px4_sitl_image.py --tag v1.18.0-rc1

pytest tests/command/nav_land/ --drone-address=udp://:14540 \
  --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_quadx --vehicle-type=quadcopter --autopilot=px4
pytest tests/mission/nav_vtol_land/ --drone-address=udp://:14540 \
  --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_standard_vtol --vehicle-type=vtol --autopilot=px4
```

Nothing in the harness changes: it starts, restarts and stops this PX4 exactly as it does a source build.
The image's exact build is in `<dest>/image.json`, and in every report's header (version and git hash).
Unlike `docker run --rm`, parameters persist between runs (`<dest>/root/opt/px4/fs/`); delete that directory for a factory-fresh start.
With Docker available, `docker run --rm --network host -e PX4_SIM_MODEL=sihsim_quadx px4io/px4-sitl:latest` and `--drone-address=udp://:14540` *without* `--px4-sitl` runs the same tests against the real container, but the harness then can't restart the stack (Tier 2 pattern #8), so flight tests that rely on a restart for cleanup aren't supported that way.
