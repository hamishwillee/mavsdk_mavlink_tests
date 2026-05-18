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
            "Examples: udp://:14540  serial:///dev/ttyUSB0:57600"
        ),
    )
    parser.addoption(
        "--connection-timeout",
        action="store",
        default=30,
        type=int,
        help="Seconds to wait for the drone to become reachable (default: 30).",
    )
