# MAV_CMD_NAV_TAKEOFF COMMAND_INT SUMMARY (PX4 - Fixed Wing)

**Firmware:** 4.8.0 **Date:** 2026-06-02 21:33

[✓ Supported | ✘ Unsupported| ? Unknown]

Flight summary ....

| Param | Label     | Description                                                                                                             | Values |
| ----- | --------- | ----------------------------------------------------------------------------------------------------------------------- | ------ |
| 1     | Abort Alt | [✓ / ✘ / ? ] Minimum target altitude if landing is aborted (0 = undefined/use system default)                           |        |
| 2     | Land Mode | [✓ / ✘ / ? ] `PRECISION_LAND_MODE` enum: 0=DISABLED, 1=OPPORTUNISTIC, 2=REQUIRED                                        | 0/1/2  |
| 3     | —         | Empty                                                                                                                   |        |
| 4     | Yaw Angle | [✓ / ✘ / ? ] Desired yaw angle. NaN = use current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home) |        |
| 5     | Latitude  | [✓ / ✘ / ? ]                                                                                                            |        |
| 6     | Longitude | [✓ / ✘ / ? ]                                                                                                            |        |
| 7     | Altitude  | [✓ / ✘ / ? ] **Landing altitude (ground level in current frame)**                                                       |        |

- NaN used for default/unsupported
- 0 Use for default/unsupported

**Preconditions:**

-
