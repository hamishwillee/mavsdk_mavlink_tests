"""
Reproducer: a NAV_VTOL_TAKEOFF sent while DISARMED with an altitude below the vehicle
makes PX4 accept the next valid NAV_VTOL_TAKEOFF but never execute it (stays in HOLD).

Each variant boots a fresh PX4 SIH multicopter, waits for position, optionally sends
one NAV_VTOL_TAKEOFF (COMMAND_INT) while disarmed, then arms, sends a valid one to
home + 30 m (frame 5, AMSL) and reports the max altitude and flight modes over 40 s.

  A  no disarmed send                                   -> climbs
  B  disarmed send, valid (real lat/lon, home + 30 m)   -> climbs
  C  disarmed send, INT32_MAX lat/lon, frame 6, z=30    -> stays in HOLD
  D  disarmed send, INT32_MAX lat/lon, valid altitude   -> climbs
  E  disarmed send, real lat/lon, frame 6, z=30         -> stays in HOLD
(PX4 main 7cb65787b3, 2026-10-01, 2 runs each.) z=30 in frame 6 is read by PX4 as
30 m AMSL — far below the vehicle — because it ignores the COMMAND_INT frame for z.

Usage (from the repo root, with no other PX4 SITL running):
  PX4_DIR=~/github/px4/PX4-Autopilot-main .venv/bin/python scripts/px4_disarmed_vtol_takeoff_ab.py A E
Clears the build's saved parameters/dataman before each boot.
"""
import asyncio, os, shutil, subprocess, sys, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from tests.mavsdk_compat import Endpoint, GCS_SYSID, GCS_COMPID
from tests.command.conftest import probe_command_int
from tests.flight_helpers import _wait_armable, _get_home_position, _get_heading

PX4 = os.path.expanduser(os.environ.get("PX4_DIR", "~/github/px4/PX4-Autopilot"))
B = f"{PX4}/build/px4_sitl_default"
S = os.path.abspath("logs")
os.makedirs(S, exist_ok=True)


def boot(tag):
    env = os.environ.copy(); env["PX4_SIM_MODEL"] = "sihsim_quadx"; env["PATH"] = f"{B}/bin:" + env["PATH"]
    for f in ("parameters.bson", "parameters_backup.bson", "dataman"):
        p = f"{B}/{f}"
        if os.path.exists(p):
            os.remove(p)
    fh = open(f"{S}/ab_px4_{tag}.log", "w")
    return subprocess.Popen([f"{B}/bin/px4", "-s", f"{B}/etc/init.d-posix/rcS", "-w", B], cwd=PX4,
                            stdout=fh, stderr=fh, env=env)


async def run(variant):
    ep = Endpoint("gcs", GCS_SYSID, GCS_COMPID, "udpin://0.0.0.0:14540")
    try:
        system = await ep.autopilot(60)
        home = await _get_home_position(system, timeout_s=90)
        await _get_heading(system, timeout_s=120)  # position available
        lat, lon, amsl = int(home.latitude_deg * 1e7), int(home.longitude_deg * 1e7), home.absolute_altitude_m
        cmd = dict(frame=5, param1=0.0, param2=0.0, param3=0.0, param4=None, x=lat, y=lon, z=amsl + 30.0)
        if variant == "C":
            # exactly the removed disarmed support probe: INT32_MAX lat/lon, frame 6, z=30
            ack = await probe_command_int(system, 84, frame=6, param4=None, x=0x7FFFFFFF, y=0x7FFFFFFF, z=30.0)
            print(f"  disarmed probe (INT32_MAX, frame 6, z=30): ack={None if ack is None else ack['result']}", flush=True)
            await asyncio.sleep(3)
        if variant == "D":  # INT32_MAX lat/lon only (valid absolute altitude)
            ack = await probe_command_int(system, 84, frame=5, param4=None, x=0x7FFFFFFF, y=0x7FFFFFFF, z=amsl + 30.0)
            print(f"  disarmed probe (INT32_MAX lat/lon, valid z): ack={None if ack is None else ack['result']}", flush=True)
            await asyncio.sleep(3)
        if variant == "E":  # real lat/lon, frame 6, z=30 (below ground as AMSL)
            ack = await probe_command_int(system, 84, frame=6, param4=None, x=lat, y=lon, z=30.0)
            print(f"  disarmed probe (real lat/lon, frame 6, z=30): ack={None if ack is None else ack['result']}", flush=True)
            await asyncio.sleep(3)
        if variant == "B":
            ack = await probe_command_int(system, 84, **cmd)
            print(f"  disarmed send: ack={None if ack is None else ack['result']}", flush=True)
            await asyncio.sleep(3)
        await _wait_armable(system)
        await system.action.arm()
        await asyncio.sleep(0.5)
        ack = await probe_command_int(system, 84, **cmd)
        modes, max_alt = set(), 0.0
        t0 = time.monotonic()

        async def watch_mode():
            async for fm in system.telemetry.flight_mode():
                modes.add(fm.name)
        task = asyncio.create_task(watch_mode())
        async for pos in system.telemetry.position():
            max_alt = max(max_alt, pos.relative_altitude_m)
            if time.monotonic() - t0 > 40:
                break
        task.cancel()
        print(f"  armed send: ack={None if ack is None else ack['result']}  max_alt={max_alt:.1f} m  modes={sorted(modes)}", flush=True)
    finally:
        ep.close()


for variant in sys.argv[1:] or ("A", "B", "A", "B"):
    print(f"variant {variant}:", flush=True)
    p = boot(variant)
    try:
        time.sleep(8)
        asyncio.run(run(variant))
    finally:
        p.kill(); p.wait(); time.sleep(3)
