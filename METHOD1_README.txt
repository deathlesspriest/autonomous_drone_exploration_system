CDE1302 METHOD 1 - WAYPOINT MISSION

The current operating guide is instructions_waypoint_and_trajectory.txt.
Use its Method 1 section. The root run_sweep_with_tags.py launcher is not needed.

On simple_trajectory_method:
python laptop/mini_project_mission.py --drone-id 2 --waypoints-file waypoints/mini_project_waypoint_test.txt

On simple_waypoint_method:
python laptop/mini_project_mission.py --drone-id 2 --waypoints-file waypoints/cde1302_sweep_1.txt

Both listed route files contain the full six-pass draft. The smaller
waypoints/method1_test.txt is only a short route. Confirm camera coverage and
local axes before the assessment; the full draft has not been flight-validated.

The integrated program starts the camera. Click its window and press M when
READY and flight checks are complete. Outputs go to mission_logs/mission_DATE_TIME/.
Open tag_summary.csv for final estimated local tag coordinates.

Keep the original camera, detector C sources, protocol and communication files.
Do not run another flight program or camera viewer at the same time.
See the full guide for landing verification and required post-flight report work.
