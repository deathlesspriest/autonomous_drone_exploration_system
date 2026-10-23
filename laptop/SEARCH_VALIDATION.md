# Offline validation record

Date: 2026-10-23. Upstream baseline: `c4caf911d626c95af28cfb619afcdbd06da15082`.

- Added mission and adapter tests: **39 passed**.
- Full laptop suite: **149 passed, 1 failed**. The same failure was reproduced from isolated, unmodified baseline files. The failure is the unchanged upstream `test_parse_telemetry_includes_px4_status_fields`: its assertion accesses `TelemetryPacket.px4_link_ok`, which is absent in the baseline parser. The new runner obtains armed/mode/landed information directly from MAVLink instead of depending on those missing custom-packet fields.
- The example route at 1.0 m and 0.3 m/s has 19 lanes, approximately 73.1 m travel and a 398 s route estimate. This excludes takeoff/landing and is not a measured endurance result. Geometry uses illustrative 120 mm tags and 20 mm camera offset.
- Synthetic end-to-end test: required three tags, return and landing complete. This validates software sequencing only.
- No real UAV was connected, armed, flown or reflashed. Live communication, optical calibration, tracking accuracy, actual endurance, real landing behavior and actual `.ulg` analysis remain to be verified with hardware.

The existing `main/`, firmware configuration and original laptop control scripts are unchanged. New scripts use the original protocol functions and host detector.
