#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
usage:
  run_three_car_unloaded_pretest.sh \
    --confirm-area-clear \
    [--confirm-circle-area-clear] \
    --confirm-wheels-on-floor \
    --confirm-unloaded-40cm-fixture

This command performs topology and camera-observation checks, resets all three
odometers, records a bag and runs the selected bounded unloaded cooperative
validation. Run the 0.30 m straight gate before the 1.00 m S gate.
Circle mode additionally requires --confirm-circle-area-clear and reserves the
0.20 m straight entry, 0.40 m curvature ramp and full clockwise R=0.5 m circle.
EOF
}

confirm_area=false
confirm_circle_area=false
confirm_floor=false
confirm_fixture=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --confirm-area-clear) confirm_area=true ;;
    --confirm-circle-area-clear) confirm_circle_area=true ;;
    --confirm-wheels-on-floor) confirm_floor=true ;;
    --confirm-unloaded-40cm-fixture) confirm_fixture=true ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; usage; exit 2 ;;
  esac
  shift
done
if [[ "$confirm_area" != true || "$confirm_floor" != true ||
      "$confirm_fixture" != true ]]; then
  echo "ERROR: all three physical confirmation flags are required" >&2
  usage
  exit 2
fi

path_mode="${AGV_THREE_CAR_PATH_MODE:-s}"
case "$path_mode" in
  s)
    estimator_path_file="path_s_curve.yaml"
    motion_launch="three_car_unloaded_bounded_pretest.launch"
    run_prefix="three_car_cooperative_s_1m"
    manifest_path_description="continuous_S_1.00m"
    run_speed="0.05"
    validation_profile="camera_fused_s_1p00"
    ;;
  straight)
    estimator_path_file="path_straight_1m.yaml"
    motion_launch="three_car_unloaded_straight_pretest.launch"
    run_prefix="three_car_cooperative_straight_0p30m"
    manifest_path_description="continuous_straight_0.30m"
    run_speed="0.03"
    validation_profile="camera_fused_straight_0p30"
    ;;
  circle)
    if [[ "$confirm_circle_area" != true ]]; then
      echo "ERROR: circle mode requires --confirm-circle-area-clear" >&2
      usage
      exit 2
    fi
    estimator_path_file="path_circle_r0p5_cw_smooth.yaml"
    motion_launch="three_car_unloaded_circle_pretest.launch"
    run_prefix="three_car_cooperative_circle_r0p5_cw_smooth"
    manifest_path_description="straight_0.20m_ramp_0.40m_clockwise_circle_R0.50m_full"
    run_speed="0.05"
    validation_profile="camera_fused_circle_r0p5_cw_smooth"
    ;;
  *)
    echo "ERROR: unsupported AGV_THREE_CAR_PATH_MODE=${path_mode}" >&2
    exit 2
    ;;
esac

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd "${script_dir}/../../.." && pwd)"
cd "$workspace"

if [[ "$(hostname)" != "robot1" ]]; then
  echo "ERROR: this orchestration command must run on Robot1" >&2
  exit 3
fi
if [[ ! -f devel/setup.bash ]]; then
  echo "ERROR: ${workspace}/devel/setup.bash is missing" >&2
  exit 4
fi
if [[ -n "$(git status --porcelain --untracked-files=normal)" ]]; then
  echo "WARNING: working tree contains uncommitted changes; continuing by request" >&2
  git status --short >&2
fi

source /opt/ros/noetic/setup.bash
source devel/setup.bash
source src/multi_agv_bringup/scripts/setup_ros_network.sh \
  192.168.6.101 192.168.6.101
estimator_path_config="${workspace}/src/multi_agv_bringup/config/${estimator_path_file}"

local_sha="$(git rev-parse HEAD)"
for index in 1 2 3; do
  node="/agv${index}/chassis_controller"
  if ! rosnode list | grep -Fqx "$node"; then
    echo "ERROR: ${node} is not running" >&2
    exit 6
  fi
  deployed_sha="$(rosparam get "/agv${index}/deployment/git_sha" 2>/dev/null || true)"
  deployed_host="$(rosparam get "/agv${index}/deployment/hostname" 2>/dev/null || true)"
  if [[ "$deployed_sha" != "$local_sha" ]]; then
    echo "ERROR: agv${index} Git SHA ${deployed_sha:-<missing>} != ${local_sha}" >&2
    echo "Start every chassis with start_three_car_chassis.sh." >&2
    exit 7
  fi
  if [[ "$deployed_host" != "robot${index}" ]]; then
    echo "ERROR: agv${index} deployment hostname is ${deployed_host:-<missing>}" >&2
    exit 8
  fi
done

expected_track=(0.135484339 0.139284482 0.136802843)
expected_left_scale=(1.129384768 1.091612663 1.134911374)
expected_right_scale=(1.115373526 1.102973507 1.137011300)
numeric_equal() {
  awk -v actual="$1" -v expected="$2" \
    'BEGIN {difference=actual-expected; if (difference<0) difference=-difference;
            exit !(difference <= 0.0000005)}'
}
for index in 1 2 3; do
  array_index=$((index - 1))
  for parameter in wheel_separation wheel_feedback_scale_left \
                   wheel_feedback_scale_right; do
    case "$parameter" in
      wheel_separation) expected="${expected_track[$array_index]}" ;;
      wheel_feedback_scale_left) expected="${expected_left_scale[$array_index]}" ;;
      wheel_feedback_scale_right) expected="${expected_right_scale[$array_index]}" ;;
    esac
    actual="$(rosparam get "/agv${index}/chassis_controller/${parameter}" 2>/dev/null || true)"
    if [[ -z "$actual" ]] || ! numeric_equal "$actual" "$expected"; then
      echo "ERROR: agv${index} ${parameter}=${actual:-<missing>}, expected ${expected}" >&2
      echo "Restart that chassis from the current three-car configuration." >&2
      exit 8
    fi
  done
done

for node in /pose_provider /camera_odom_fusion; do
  if ! rosnode list | grep -Fqx "$node"; then
    echo "ERROR: camera observation node is missing: ${node}" >&2
    echo "Run ./start_camera_pose_fusion_only.sh on Robot1." >&2
    exit 9
  fi
done
alive_value="$(timeout 4 rostopic echo -n 1 /vision/aruco/alive 2>/dev/null |
  awk '/data:/ {print $2; exit}' || true)"
if [[ "$alive_value" != "True" ]]; then
  echo "ERROR: Windows ArUco UDP stream is not alive" >&2
  exit 9
fi
for index in 1 2 3; do
  if ! timeout 4 rostopic echo -n 1 \
       "/pose_provider/agv${index}/base_pose_raw" >/dev/null 2>&1; then
    echo "ERROR: no fresh camera base pose for agv${index}" >&2
    exit 9
  fi
done

for forbidden in /multi_agv_controller /experiment_supervisor \
                 /three_car_unloaded_bounded_pretest /path_state_estimator; do
  if rosnode list | grep -Fqx "$forbidden"; then
    echo "ERROR: conflicting central node is already running: ${forbidden}" >&2
    exit 9
  fi
done

estimator_pid=""
bag_pid=""
motion_pid=""
cleanup() {
  for pid in "$motion_pid" "$estimator_pid" "$bag_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
  for pid in "$motion_pid" "$estimator_pid" "$bag_pid"; do
    if [[ -n "$pid" ]]; then
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM HUP

if [[ ! -f "$estimator_path_config" ]]; then
  echo "ERROR: path configuration is missing: ${estimator_path_config}" >&2
  exit 10
fi
roslaunch multi_agv_bringup camera_fused_virtual_load_state_estimator.launch \
  path_config:="$estimator_path_config" &
estimator_pid="$!"
deadline=$((SECONDS + 15))
until rostopic info /multi_agv/cooperative_state 2>/dev/null |
      grep -q '^Publishers:'; do
  if ! kill -0 "$estimator_pid" 2>/dev/null; then
    wait "$estimator_pid"
  fi
  if (( SECONDS >= deadline )); then
    echo "ERROR: cooperative state estimator did not start" >&2
    exit 10
  fi
  sleep 0.2
done

rosrun multi_agv_bringup check_three_car_readonly_gate.py \
  --observe-seconds 5 \
  --require-fused-cooperative-state
"${script_dir}/require_three_car_motion_gate.sh" 2 10 fused

run_stamp="$(date +%Y%m%d_%H%M%S)"
run_id="${run_prefix}_${run_stamp}"
run_dir="${AGV_BAG_DIR:-${workspace}/experiment_data/three_car_cooperative_validation}/${run_stamp}_${path_mode}"
mkdir -p "$run_dir"
bag_path="${run_dir}/${run_id}.bag"
manifest_path="${run_dir}/${run_id}_manifest.txt"
params_path="${run_dir}/${run_id}_params.yaml"
analysis_path="${run_dir}/${run_id}_analysis.json"

{
  echo "run_id=${run_id}"
  echo "git_sha=${local_sha}"
  echo "git_branch=$(git branch --show-current)"
  echo "robot1_ip=192.168.6.101"
  echo "robot2_ip=192.168.6.102"
  echo "robot3_ip=192.168.6.103"
  echo "started_at=$(date --iso-8601=seconds)"
  echo "fixture=unloaded_equilateral_0.40m"
  echo "path=${manifest_path_description}"
  echo "speed=${run_speed}m/s"
  echo "validation_profile=${validation_profile}"
  echo "control_state=three_camera_fused_base_poses"
  echo "fusion_propagation=wheel_translation_plus_corrected_imu_yaw"
  echo "camera_role=absolute_position_and_heading_closed_loop_authority"
  echo "raw_camera_role=same-sensor_observation_channel"
  echo "cooperative_frame=three_car_path_auto_anchored"
  echo "virtual_load=rigid_fit_from_three_fused_support_centres"
  echo "load_tag_required=false"
  echo "tracker_parameters=see_post_launch_params_snapshot"
  echo "parameter_snapshot_timing=after_motion_node_parameter_load_before_start_delay"
  echo "post_gate_odometry_reset=false"
} > "$manifest_path"

rosbag record -O "$bag_path" \
  /agv1/chassis_command /agv2/chassis_command /agv3/chassis_command \
  /agv1/chassis_feedback /agv2/chassis_feedback /agv3/chassis_feedback \
  /agv1/capability_report /agv2/capability_report /agv3/capability_report \
  /agv1/imu /agv2/imu /agv3/imu \
  /agv1/odom /agv2/odom /agv3/odom \
  /vision/aruco/alive /vision/aruco/calibration_epoch \
  /vision/aruco/diagnostics \
  /camera/world/agv1_tag_pose /camera/world/agv2_tag_pose \
  /camera/world/agv3_tag_pose \
  /camera/world/agv1_confidence /camera/world/agv2_confidence \
  /camera/world/agv3_confidence \
  /pose_provider/agv1/base_pose_raw /pose_provider/agv2/base_pose_raw \
  /pose_provider/agv3/base_pose_raw \
  /pose_provider/agv1/base_pose_filtered \
  /pose_provider/agv2/base_pose_filtered \
  /pose_provider/agv3/base_pose_filtered \
  /pose_provider/agv1/base_pose_fused \
  /pose_provider/agv2/base_pose_fused \
  /pose_provider/agv3/base_pose_fused \
  /multi_agv/cooperative_state \
  /multi_agv/bounded_pretest/path_reference \
  /multi_agv/bounded_pretest/controller_state &
bag_pid="$!"
sleep 2
if ! kill -0 "$bag_pid" 2>/dev/null; then
  wait "$bag_pid"
fi

result_parameter="/multi_agv/three_car_unloaded_pretest_result_code"
rosparam set "$result_parameter" -1
# Remove parameters retained by an earlier completed run. This makes the
# post-launch snapshot authoritative rather than mixing old and new fields.
rosparam delete /three_car_unloaded_bounded_pretest 2>/dev/null || true
roslaunch multi_agv_bringup "$motion_launch" \
  platform_transport_type:=serial \
  enable_commands:=true \
  confirm_readonly_gate_passed:=true \
  confirm_test_area_clear:=true \
  confirm_wheels_on_floor:=true \
  confirm_unloaded_40cm_fixture:=true &
motion_pid="$!"
parameter_namespace="/three_car_unloaded_bounded_pretest/three_car_unloaded_bounded_pretest/tracker"
parameter_deadline=$((SECONDS + 4))
until rosnode list 2>/dev/null | \
        grep -Fqx /three_car_unloaded_bounded_pretest && \
      rosparam get \
        "${parameter_namespace}/curvature_preview_seconds_negative" \
        >/dev/null 2>&1; do
  if ! kill -0 "$motion_pid" 2>/dev/null; then
    wait "$motion_pid"
  fi
  if (( SECONDS >= parameter_deadline )); then
    echo "ERROR: motion node parameters were not loaded before snapshot" >&2
    exit 10
  fi
  sleep 0.05
done
rosparam dump "$params_path"
set +e
wait "$motion_pid"
roslaunch_status="$?"
set -e
motion_pid=""
reported_status="$(rosparam get "$result_parameter" 2>/dev/null || true)"
if [[ "$reported_status" =~ ^[0-9]+$ ]] &&
   (( reported_status >= 0 )); then
  motion_status="$reported_status"
elif (( roslaunch_status != 0 )); then
  motion_status="$roslaunch_status"
else
  echo "ERROR: motion node exited without a valid result code" >&2
  motion_status=125
fi

kill -INT "$bag_pid"
wait "$bag_pid" || true
bag_pid=""

echo "finished_at=$(date --iso-8601=seconds)" >> "$manifest_path"
echo "motion_exit_status=${motion_status}" >> "$manifest_path"
set +e
rosrun multi_agv_bringup analyze_three_car_cooperative_bag.py \
  "$bag_path" --mode "$path_mode" --output "$analysis_path"
analysis_status="$?"
set -e
echo "analysis_exit_status=${analysis_status}" >> "$manifest_path"
echo
if [[ "$motion_status" -eq 0 ]]; then
  echo "THREE-CAR PRETEST COMPLETED"
else
  echo "THREE-CAR PRETEST FAILED OR ABORTED (status=${motion_status})" >&2
fi
echo "bag=${bag_path}"
echo "manifest=${manifest_path}"
echo "params=${params_path}"
echo "analysis=${analysis_path}"
echo "Results: ${run_dir}"
exit "$motion_status"
