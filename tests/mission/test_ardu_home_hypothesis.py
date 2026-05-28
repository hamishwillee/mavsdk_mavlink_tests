"""
Hypothesis test: does ArduCopter accept MAV_FRAME_GLOBAL (frame=0) for flight
mission items when a home waypoint is explicitly included at seq=0?

Background
----------
test_frame_types.py uploads single-item missions (seq=0, frame=0, cmd=16) to
probe frame support.  ArduCopter rejects these with TOO_MANY_MISSION_ITEMS.

Hypothesis: ArduCopter reserves seq=0 for the home position.  Real GCS
applications (QGroundControl, Mission Planner) always include the home position
as seq=0 before any user items.  If the upload includes a home item at seq=0,
ArduCopter may accept frame=0 items at seq=1+.

Plan: ardu_home_mission.json
-----------------------------
  seq=0 — home at ArduCopter SITL home (37.6234N, 122.0811W, 0m AMSL, frame=0)
  seq=1 — test waypoint ~111m north (37.6244N, 122.0811W, 50m AMSL, frame=0)

Outcomes
--------
  Upload accepted, seq=1 frame=0 preserved  → hypothesis confirmed
  Upload accepted, seq=1 frame=5 (GLOBAL_INT) → partially confirmed (INT upgrade)
  Upload rejected TOO_MANY_MISSION_ITEMS     → hypothesis wrong; test skips
  Upload rejected with different error       → new information; test skips

Running against ArduCopter SITL::

    pytest tests/mission/test_ardu_home_hypothesis.py \\
        --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 -v -s

Running in paired mode (sanity check — mock accepts everything)::

    pytest tests/mission/test_ardu_home_hypothesis.py -v -s
"""

import asyncio
import logging

import pytest

from mavsdk.mission_raw import MissionRawError

from .conftest import clear_all_mission_types, load_plan

log = logging.getLogger(__name__)

TRANSFER_TIMEOUT_S = 30


class TestArduHomeFirstHypothesis:
    """Probe whether home-first upload fixes MAV_FRAME_GLOBAL acceptance on ArduCopter."""

    async def test_frame0_accepted_with_home_prepended(self, gcs_system):
        """
        Upload [home@seq=0 (frame=0), waypoint@seq=1 (frame=0)].

        Pass:  upload accepted and seq=1 returns frame=0 or frame=5 (INT upgrade).
        Skip:  upload rejected — hypothesis not confirmed; reason logged.
        Fail:  upload accepted but download is inconsistent.
        """
        items = load_plan("ardu_home_mission.json")
        mr = gcs_system.mission_raw

        try:
            try:
                async with asyncio.timeout(TRANSFER_TIMEOUT_S):
                    await mr.upload_mission(items)
            except MissionRawError as exc:
                reason = str(exc).split(":")[0].strip()
                log.info(
                    "frame=0 with home@seq=0: upload REJECTED (%s) — hypothesis NOT confirmed",
                    reason,
                )
                pytest.skip(
                    f"Upload rejected ({reason}) even with home item at seq=0.  "
                    "The TOO_MANY_MISSION_ITEMS error is not caused by a missing home item."
                )

            try:
                async with asyncio.timeout(TRANSFER_TIMEOUT_S):
                    downloaded = await mr.download_mission()
            except MissionRawError as exc:
                pytest.fail(
                    f"Upload succeeded but download failed: {exc}.  "
                    "Autopilot accepted the mission but cannot serve it back."
                )

            wp = next((d for d in downloaded if d.seq == 1), None)
            if wp is None:
                pytest.fail(
                    f"Upload accepted but seq=1 waypoint missing from download "
                    f"(got {len(downloaded)} item(s), seqs={[d.seq for d in downloaded]})."
                )

            log.info(
                "frame=0 with home@seq=0: ACCEPTED.  seq=1 downloaded as "
                "frame=%d (z=%.3f).  Hypothesis %s.",
                wp.frame,
                wp.z,
                "CONFIRMED" if wp.frame == 0
                else "CONFIRMED (INT-upgraded to GLOBAL_INT)" if wp.frame == 5
                else f"UNEXPECTED — frame changed to {wp.frame}",
            )

            assert wp.frame in (0, 5), (
                f"seq=1 downloaded as frame={wp.frame} — expected 0 (MAV_FRAME_GLOBAL, "
                f"preserved) or 5 (MAV_FRAME_GLOBAL_INT, acceptable INT upgrade).  "
                f"Got an unexpected frame."
            )

        finally:
            await clear_all_mission_types(gcs_system)
