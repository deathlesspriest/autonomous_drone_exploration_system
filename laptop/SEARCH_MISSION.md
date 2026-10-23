# CDE1302 autonomous search

This addition implements the course mission on the laptop: take off, scan a configurable rectangle, identify and locate unique AprilTags, return to the captured launch position, verify landing, and save records for post-flight analysis. It uses the existing ESP32 trajectory and MAVLink bridge interfaces. No firmware files or PX4 parameters are changed.

Based on upstream commit `c4caf911d626c95af28cfb619afcdbd06da15082`. It has been checked with offline tests and a synthetic mission, **not flown on your aircraft**. Successful software simulation does not establish real detection coverage, localization accuracy, or flight performance.

## Start without the drone

Run these from the repository's top directory. Use Python 3.10 or later and the C compiler already needed by the repository's host AprilTag detector.

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r laptop/requirements-search.txt
python laptop/search_mission.py --plan --example --out search_runs/example-plan
python laptop/search_mission.py --simulate --example --out search_runs/example-simulation
python laptop/analyze_search.py search_runs/example-simulation
```

Each output folder must be new. Open `plan/preview.html` for the route, and `flight_analysis.png` for the synthetic mission plot. The simulation uses a simple kinematic vehicle and synthetic target observations; it does not simulate PX4, radio communication, image recognition, or real flight dynamics.

The example assumes a 0.12 m black tag square, 0.02 m forward camera offset, zero arena heading, and an illustrative outer-boundary offset. These are **not measurements of your UAV or arena**. `--example` is rejected for connected modes.

## Configure the real test

Copy `laptop/search_configs/mission.json` to a separate JSON file. Edit this before the test; target count, rectangle dimensions, and search offsets are all configurable. `null` means a required measurement is missing. `geometry_verified` must remain false until the following settings and frame checks have been verified.

| Setting | What to enter or check |
|---|---|
| `drone_id` | Your actual ID, using the existing network setup: UAV `192.168.1.(200+ID)`, laptop `192.168.1.(100+ID)`. |
| `expected_tags` | Number announced before the test; supplied arena has three. |
| `allowed_tag_ids` | Leave null if IDs are unknown. Otherwise list eligible IDs, excluding navigation/background tags. The stock family is tag16h5, IDs 0–21. |
| `search_area` | Rectangle minimum x/y and dimensions in the arena frame below. |
| `flight_y_min_m` | Measured outer flight boundary's minimum y relative to launch center. This offset is not explicitly dimensioned in the arena PDF. |
| `arena_heading_deg` | Angle of arena +x in PX4 local x/y: `degrees(atan2(delta_local_y, delta_local_x))`. Zero only when arena +x matches local +x. |
| `camera.tag_size_m` | Side length of the **outer black square**, in metres; not A4 paper size or the inner code region. The repository includes 120 mm tags on A4, but your print scale must be measured. |
| `camera.position_body_m` | Camera optical-center offset from the vehicle position reference: forward, right, down, in metres. A centered camera near the nose still needs a forward offset. Verify the default zero vertical/lateral values. |
| `camera.image_rotation_deg` | Orientation around the optical axis. At zero, image right maps to vehicle right and image down maps to vehicle rear for a level vehicle. Determine using the stationary checks below. |
| `camera.fx/fy/cx/cy` | Copied from the repository for its 320×240 stream. Validate against your camera; recalibrate if inaccurate. No lens-distortion correction is added here. |
| `camera.measured_reliable_radius_m` | Optional validated tag-center detection radius about the camera's vertical projection, at the selected height, tag size, lighting, tilt and speed. If null, use a conservative geometric estimate. |
| `vehicle_radius_m` | Measure the complete horizontal envelope including propeller guards; initial assumption is 0.15 m. |
| `tracking_error_m`, `arrival_radius_m` | Planner error allowances, initially 0.03 + 0.06 m. Their sum bounds the runtime segment deviation threshold. Validate from flight records. |

Arena frame: origin at the center of the 0.9×0.9 m launch square, +x to the right of the supplied arena drawing (towards the search region), +y down the drawing. The supplied dimensions yield outer x from −0.45 to 4.95 m and search x from 0.9 to 4.5 m, y from −1.8 to 1.35 m. The outer region is 5.4×4.5 m and the search region is 3.6×3.15 m. Confirm the launch-center convention and actual markings before use. The preview plots positive y upward; it is a coordinate plot, not a copy of the arena drawing.

Do not assume that tags appear only at the illustrative dots. The planner covers the whole rectangle. It also rejects geometry where launch, search coverage, or the route violate the inset flight boundary.

The initial search height is **1.0 m**, reference speed **0.3 m/s**, acceleration **0.3 m/s²**, overlap **30%**, and deadline **540 s** with a **60 s return reserve**. These are adjustable initial values, not measured performance claims. The existing takeoff still targets local z = −0.5 m; a trajectory then climbs to the configured survey height. A 0.5 m survey produced a long route beyond the estimated endurance. The runner currently limits configured height to 2 m, speed to 0.3 m/s and acceleration to 0.5 m/s². Actual controller velocity can differ from reference velocity.

Plan again after changing measurements:

```sh
python laptop/search_mission.py --plan --config my-mission.json --out search_runs/measured-plan
```

## Validate with the drone available

1. Keep the working firmware and network configuration. Close QGroundControl, camera viewers and other mission scripts while this runner owns UDP ports 14550, 5005 and 5009. Do not run competing control programs.
2. With the aircraft on the floor and disarmed, run the readback check. Replace `22` with your ID:

   ```sh
   python laptop/search_mission.py --monitor --drone-id 22 --seconds 15 --out search_runs/readback
   ```

   This requests existing MAVLink telemetry streams; it sends no arm, takeoff, mode, parameter, or navigation commands. Confirm position, attitude, heartbeat, extended landed state and ESP telemetry all arrive. If extended landed state is absent, live preflight will refuse to start; inspect bridge/PX4 compatibility rather than substituting a timer for landing proof.
3. Establish arena heading from two known locations while disarmed and level, or from a verified local-coordinate survey. With the measured configuration filled, use `python laptop/search_mission.py --inspect-tags --config my-mission.json --seconds 120 --out search_runs/tag-check` for disarmed camera/local-frame checks. Begin on the launch floor; once `INSPECTION_STARTED` appears, raise the disarmed aircraft on a suitable support to the measured camera height. Use the lab's propeller-removal procedure for handling. This mode sends only telemetry requests and camera-stream commands, and permits `geometry_verified=false`. Verify PX4 height agrees with the actual support height. Establish camera orientation with a known tag forward/right of the camera: printed `TAG_DETECTION` local direction must agree with the measured displacement. Check tag scale, offsets and positioning at several points and yaw angles. The old `tag_stream.py` body-pose display assumes the original 45° mount; do not use its body coordinates as validation of this new 90° transform.
4. Measure reliable detection footprint at the planned camera-to-floor distance, with tags at its center and boundary. Include edge placements, motion blur, illumination, and allowed tilt. If using `measured_reliable_radius_m`, it must describe where the **tag center** is repeatedly detected, including the full printed tag fitting in the image. Re-measure when altitude or tag size changes.
5. Validate a short search rectangle and one known tag before the full area. Inspect the actual trajectory, tracking error, tag error and landing location. Keep the normal lab RC/emergency procedure available. The course requires emergency action and examiner notification after failure; automatic mission restart is not implemented.
6. Once those checks support the configured bounds, set `geometry_verified` to true. Do not change the setting just to bypass missing measurements.

The aircraft must begin landed and disarmed with local floor z near zero: the stock firmware's initial takeoff uses absolute local z = −0.5. The runner captures actual PX4 launch x/y/z, then rotates and translates arena coordinates. It does not assume launch x/y equals zero or rely on the stock waypoint runner's map offset. **Do not pass the preview's `waypoints.txt` to `simple_waypoint_mission.py`.**

## Fly when ready

```sh
python laptop/search_mission.py --fly --config my-mission.json --out search_runs/trial-01
```

Only `--fly` can issue flight commands. It checks fresh position, attitude, heartbeat, landed state, ESP telemetry and usable camera frames before asking you to type `SEARCH-ID`. This is the final runtime takeoff confirmation. Target coordinates are never entered manually during the mission.

It then:

1. Requests stock takeoff and verifies armed Offboard operation and stable height.
2. Climbs to survey height and flies the selected lawnmower route using short uploaded, acceleration-limited trajectories. Each segment starts and ends at rest; the existing trajectory interface holds heading.
3. Processes host camera images with the repository's detector, records ID, camera-relative translation/rotation, local position, quality and timestamps, and confirms distinct tags using multiple frames and spatial consistency.
4. Checks target count and time before each new segment. It returns at a segment boundary after enough tags are confirmed; otherwise it finishes the scan or returns early to preserve the time reserve. Reserve calculation includes the proposed next segment, return and landing. It is a time budget, not a battery-state estimate.
5. Returns to the captured launch position, sends land and requires fresh on-ground state plus disarmed status. It also checks the landing center lies within a conservative circle inside the launch square.

`complete` requires the requested unique tags, return and confirmed landing. `partial` means returned and landed but fewer tags were confirmed. `aborted` means a required state failed; the program attempts the existing land command and does not claim that command was received or executed. Missing camera/telemetry, a frame reset, tracking violation or timeout stops the mission. **No new onboard Wi-Fi-loss action is installed**: after link loss the stock onboard behavior remains, and laptop commands may not arrive. There is no automatic reconnection/resume. The current firmware's disabled collision-avoidance path is not replaced; the rectangular route assumes a clear flight area.

## Localization and route design

Local position is `vehicle_position + R_local_body × (camera_offset_body + R_body_camera × tag_translation_camera)`. The full measured roll, pitch and yaw are used. Tag coordinates are in the PX4 local NED frame, as required by the project, not in the arena or camera frame. Multi-frame medians reduce isolated noise; they cannot remove calibration errors or estimator drift. Five observations spanning at least 0.6 s are initially required, with spatial rejection and spread checks. Widely inconsistent observations stay unconfirmed.

The planner obtains a conservative circular camera footprint from the calibrated nearest image edge, tilt allowance, lower permitted height, usable fraction, camera offset and full-tag extent. A measured tag-center detection radius can replace that optical estimate. It erodes coverage by tracking and arrival allowances, then uses an inscribed square and overlapping lanes. It compares both sweep directions and route entry/order variants using acceleration-limited travel time and segment overhead. This is a useful bounded search strategy, not a proof of globally minimum flight time. Geometric coverage does not guarantee successful decoding.

Camera capture timing is approximate: laptop receive time minus ESP frame age and configured network delay. PX4 sample time is aligned using boot timestamps and an observed host offset; nearby pose samples are interpolated. Delayed, missing or unmatched frames are rejected. Network asymmetry and clock estimation can still bias positions, so validate localization against independent measured tag coordinates. A 0.1 s timing error at 0.3 m/s alone corresponds to approximately 0.03 m translation error.

## Logs, PX4 analysis and the report

Every run saves configuration and plan; connected missions add launch coordinates, timestamped `events.jsonl`, tag results and mission status. Key takeoff, tag and landing events are also printed for the examiner. Preserve the complete run directory.

After the flight, close this program, download the matching PX4 `.ulg` through the usual QGroundControl/log-download procedure, and run:

```sh
python laptop/analyze_search.py search_runs/trial-01 --ulog path/to/flight.ulg
```

This overlays the local trajectory, launch, confirmed landing, and confirmed tag IDs/positions. It matches PX4 boot-time ranges, checks position agreement and rejects logs containing position resets. Select the correct flight log; these checks alone are not a unique flight identity. Optional `--references surveyed-tags.csv` computes horizontal errors and RMSE using columns `id,x,y` in the **same PX4 local frame**. References are used after the flight only. Without `--ulog`, the plot is explicitly labeled telemetry-only and does not satisfy the required PX4-log analysis. Synthetic data is also clearly labeled and cannot be combined with a real ULog.

The course report still requires your actual results and verification: maximum 10 pages including cover/appendix, single column/single spacing, Times New Roman at least 12 pt, code repository link, and submission within 48 hours after demonstration. Publish your reviewed modified code to your own GitHub repository; this package does not publish to the upstream author's repository.

For the AI declaration, record that Codex assisted in document/repository inspection, route and coordinate design, software generation, automated tests and synthetic analysis. Representative prompts include your requests to implement a configurable downward-camera search path and fulfill the supplied project tasks. Include limitations identified during review: A4 is not tag size; the stock takeoff is 0.5 m; stock waypoint coordinates are not automatically a captured launch frame; geometric coverage is not measured detection reliability; and simulated landing is not evidence of physical landing. Add your own flight-log checks, measured errors, failures and revisions after testing. Do not present this offline verification as a completed flight experiment.

## Files and tests

`search_core.py` contains configuration, planning, transforms and mission sequencing. `search_geometry.py` contains footprint/rectangle planning and preview export. `search_io.py` implements telemetry, vision, trajectory and landing adapters. `search_mission.py` is the entry point. `analyze_search.py` produces the post-flight figure and metrics.

```sh
python -m pip install pytest
python -m pytest laptop/tests/test_search_mission.py laptop/tests/test_search_io.py -q
```

The added tests cover geometric coverage, speed/acceleration, coordinate transforms, tag deduplication, mission failure/return behavior, real MAVLink decoding, stock trajectory packet structure, stale inputs, and landing confirmation. See `SEARCH_VALIDATION.md` for the recorded test results and the unrelated upstream test failure.
