METHOD 1 - MINIMAL WAYPOINT INTEGRATION

Copy these additions into the original repository on simple_waypoint_method.
Keep all original GitHub files. Merge laptop/ and waypoints/ folders.
No test files, dry-run option, trajectory system or automatic route planner included.

Files:
run_sweep_with_tags.py                 One-command entry point
laptop/mini_project_mission.py         Camera + waypoint mission integration
laptop/mini_project_support.py         Coordinates and required records
waypoints/method1_test.txt             Small integration route ending at (0,0)

The original simple_waypoint_mission.py is not launched as a separate process.
The new script reuses the same START/GOTO/LAND protocol and waypoint sequence,
with a concurrent camera loop, records, and abort-on-timeout handling.
Existing detector, communication, camera assembly and drawing helpers are imported.
Comments mark project additions and changes from the supplied GitHub behaviour.

INSTALL NOW
Extract this ZIP outside the repository, then copy its contents into the root.
Commit on simple_waypoint_method, then Push origin. No drone is required to copy.

LAB EXECUTION (only after setup, mounting and route checks)
conda activate cde1302-laptop
python run_sweep_with_tags.py --drone-id 2

Replace 2 with the assigned ID. Camera starts automatically. Click the camera
window and press M when READY and lab checks are complete. This replaces the
Lab 5 MISSION-ID console prompt. Do not run tag_stream.py or another flight script
at the same time. Q/ESC requests abort/in-place landing; link loss may prevent it.
Default takeoff wait is 10 s, waypoint timeout 40 s, arrival radius 0.25 m.

CAMERA SETTINGS - MEASURE BEFORE USING POSITIONS
The conversion now assumes a straight-down (90 degree) camera, with image top
pointing toward drone forward and image right toward drone right. A straight-down
lens alone does not establish image orientation. Check a known tag on the ground
before flying. --camera-rotation-deg adjusts optical-axis mounting rotation;
positive means image top points toward body right, viewed from above.

Defaults inherited/assumed, NOT measurements of your aircraft:
--tag-size-m 0.12             Outer black square side length, not A4 paper size
--camera-offset 0.02 0 0      Lens offset from vehicle centre, forward/right/down (m)
--camera-rotation-deg 0       Image top = forward

Example if the measured mount differs:
python run_sweep_with_tags.py --drone-id 2 --tag-size-m 0.12 --camera-offset 0.02 0 0 --camera-rotation-deg 90
Do not use these example values without checking them. Intrinsics remain those
of the original 320x240 detector; they also require physical validation.

The downward transform updates only the imported module's in-memory mounting
constants before detector creation, so both its t_body/R_body properties and
video overlay agree. Original apriltag_host.py on disk stays unchanged.
Local x/y uses yaw-only rotation: it assumes approximately level flight, NOT full
roll/pitch compensation. Timing uses nearest telemetry and an estimated image
capture time. Camera calibration, tilt, timing and estimator drift affect accuracy.

ROUTE AND HOME
The supplied three-point route is for integration testing, NOT assessment-area
coverage. Its coordinates require a verified local origin at launch (0,0), axes
and clear space. The program sends zero firmware map offset; this does NOT reset
PX4's origin. It requires position near zero before start. Later supply a validated
search file using --waypoints-file waypoints/YOUR_ROUTE.txt, ending at (0,0).
The old six-pass sweep was not certified for a downward camera at 0.5 m; it is not
included as a ready-to-fly search route. Measure coverage before choosing spacing.

RECORDS
mission_logs/mission_DATE_TIME/ contains events.csv, events.jsonl, detections.csv
and tag_summary.csv. Required events are also printed. Relative orientation and
position are saved; each tag gets a median local x/y report. A median is not an
accuracy guarantee or a count of independently confirmed targets.
Preserve matching PX4 .ulg logs separately for the assignment report.

COMPLETION LIMIT
Normal landing is requested only after fresh telemetry verifies home proximity.
Timeouts abort instead of skipping goals. Abort landing is in place and does NOT
claim successful return. Takeoff uses the Lab 5 timed wait, not altitude feedback.
The script records LAND commands but cannot verify touchdown/disarm from its
custom telemetry. Verify those in QGroundControl and observation, as in Lab 5.
Program exit is not the assessment completion time. No hardware test performed.

Development verification: existing 21 offline logic checks passed, along with
checks of downward-camera axes, offset and mounting rotation. Checks are not
included in this deliverable or the mission workflow.
