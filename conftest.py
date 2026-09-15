"""Root conftest: command-line options shared across all test modules."""
import pytest


def pytest_addoption(parser):
    parser.addoption(
        "--drone-address",
        action="store",
        default=None,
        help=(
            "MAVLink connection URL for the drone under test. "
            "If omitted the tests start their own paired mock server. "
            "Examples: udp://:14540  tcp://127.0.0.1:5760"
        ),
    )
    parser.addoption(
        "--connection-timeout",
        action="store",
        default=30,
        type=int,
        help="Seconds to wait for the drone to become reachable (default: 30).",
    )
    parser.addoption(
        "--ardupilot-sitl",
        action="store",
        default=None,
        metavar="PATH",
        help=(
            "Path to any ArduPilot SITL binary (arducopter, arduplane, ardurover, …). "
            "When set, the test suite starts the SITL automatically (killing any "
            "existing instance first) and stops it at session end. "
            "Use alongside --drone-address=tcp://127.0.0.1:5760."
        ),
    )
    parser.addoption(
        "--ardupilot-model",
        action="store",
        default=None,
        metavar="MODEL",
        help=(
            "ArduPilot SITL model string, e.g. '+', 'plane', 'quadplane', 'rover'. "
            "Defaults are auto-detected from the binary name and --vehicle-type when omitted."
        ),
    )
    parser.addoption(
        "--px4-sitl",
        action="store",
        default=None,
        metavar="DIR",
        help=(
            "Path to the PX4-Autopilot repository root (must contain "
            "build/px4_sitl_default/bin/px4). When set, the test suite starts PX4 "
            "SIH automatically and stops it at session end. "
            "Use alongside --drone-address=udp://:14540."
        ),
    )
    parser.addoption(
        "--px4-model",
        action="store",
        default="sihsim_quadx",
        metavar="MODEL",
        help=(
            "PX4_SIM_MODEL value for the SIH simulator "
            "(default: sihsim_quadx). Examples: sihsim_airplane, sihsim_rover_ackermann."
        ),
    )
    parser.addoption(
        "--home-lat",
        action="store",
        default=47.3977,
        type=float,
        help="Home latitude in degrees for home-slot prepend (default: 47.3977, PX4 SIH Zurich).",
    )
    parser.addoption(
        "--home-lon",
        action="store",
        default=8.5456,
        type=float,
        help="Home longitude in degrees for home-slot prepend (default: 8.5456, PX4 SIH Zurich).",
    )
    parser.addoption(
        "--home-alt",
        action="store",
        default=0.0,
        type=float,
        help="Home altitude in metres AMSL for home-slot prepend (default: 0.0).",
    )
    parser.addoption(
        "--vehicle-type",
        action="store",
        default=None,
        metavar="TYPE",
        help=(
            "Vehicle type label for log file naming, e.g. quadcopter, fixed_wing, quadplane, vtol. "
            "When omitted the probe attempts MAV_TYPE from HEARTBEAT (may show UNKNOWN)."
        ),
    )
    parser.addoption(
        "--autopilot",
        action="store",
        default=None,
        metavar="NAME",
        help=(
            "Autopilot stack label for log file naming, e.g. ardupilot, px4. "
            "When omitted the probe attempts to detect from AUTOPILOT_VERSION."
        ),
    )
    parser.addoption(
        "--sitl-instance",
        action="store",
        default=0,
        type=int,
        metavar="N",
        help=(
            "SITL instance number [0..N], for running multiple concurrent PX4/ArduPilot "
            "SITL processes (each pytest invocation targets one instance). Passed through "
            "as PX4's `-i N` / ArduPilot's `-I N`, which each stack uses to derive a "
            "disjoint port set and isolated runtime-state directory. Also offsets this "
            "harness's own local gRPC ports (mavsdk_server) by instance so concurrent "
            "pytest processes don't collide. --drone-address must be set to match "
            "instance N's resulting port yourself (PX4: 14540+N; ArduPilot: 5760+10N) — "
            "see root CLAUDE.md's CI section for worked examples and the full formula "
            "table. Default: 0 (single-instance, byte-identical to pre-multi-instance "
            "behaviour — log/working-dir paths are unchanged at N=0)."
        ),
    )
    parser.addoption(
        "--mavlink-definitions-dir",
        action="store",
        default="mavlink/message_definitions/v1.0",
        metavar="DIR",
        help=(
            "Path to directory containing common.xml and standard.xml "
            "(default: mavlink/message_definitions/v1.0 — the bundled git submodule). "
            "Used by command survey and protocol tests."
        ),
    )
