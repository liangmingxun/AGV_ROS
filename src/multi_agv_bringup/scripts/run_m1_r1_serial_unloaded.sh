#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
usage: run_m1_r1_serial_unloaded.sh \
  --operator NAME --pair-block ID \
  --confirm-area-clear --confirm-wheels-on-floor \
  --confirm-unloaded-30cm-fixture

Runs the selected M1+R1, M2a+R1 or M2b+M2b serial entry on Robot1. Method
selection changes only the formal upper/lower configuration; the selected
path, planar execution runtime, chassis limiter and recording chain remain
shared. Each method uses its own physical-authorization overlay. The three
chassis, vision bridge and camera fusion must already be running. Any missing
recorder heartbeat makes the algorithm publish zero commands.
EOF
}

operator=""
pair_block=""
formal_upper_mode="${FORMAL_UPPER_MODE:-M1}"
enable_robot2_derating="${FORMAL_ENABLE_ROBOT2_DERATING:-false}"
if [[ "$formal_upper_mode" != M1 && "$formal_upper_mode" != M2a &&
      "$formal_upper_mode" != M2b ]]; then
  echo "ERROR: unsupported formal upper mode: ${formal_upper_mode}" >&2
  exit 2
fi
if [[ "$enable_robot2_derating" != true &&
      "$enable_robot2_derating" != false ]]; then
  echo "ERROR: invalid Robot2 derating selection" >&2
  exit 2
fi
confirm_area=false
confirm_floor=false
confirm_fixture=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --operator) operator="$2"; shift 2 ;;
    --pair-block) pair_block="$2"; shift 2 ;;
    --confirm-area-clear) confirm_area=true; shift ;;
    --confirm-wheels-on-floor) confirm_floor=true; shift ;;
    --confirm-unloaded-30cm-fixture) confirm_fixture=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done
if [[ -z "$operator" || -z "$pair_block" ||
      "$confirm_area" != true || "$confirm_floor" != true ||
      "$confirm_fixture" != true ]]; then
  echo "ERROR: operator, pair block and all confirmations are required" >&2
  usage
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd "${script_dir}/../../.." && pwd)"
cd "$workspace"
runtime_config="${FORMAL_RUNTIME_CONFIG:-src/multi_agv_bringup/config/formal_serial_m1_r1_runtime.yaml}"
path_config="${FORMAL_PATH_CONFIG:-src/multi_agv_bringup/config/path_s_curve_terminal_straight.yaml}"
path_version="${FORMAL_PATH_VERSION:-s_curve_terminal_straight_v1}"
evaluation_config="${FORMAL_EVALUATION_CONFIG:-src/multi_agv_bringup/config/formal_evaluation_window.yaml}"
derating_authorization_config="${FORMAL_DERATING_AUTHORIZATION_CONFIG:-src/multi_agv_bringup/config/formal_exp2a_derating_authorization.yaml}"
case "$formal_upper_mode" in
  M2a)
    upper_config="${FORMAL_UPPER_CONFIG:-src/multi_agv_bringup/config/exp2a_M2a_serial_008.yaml}"
    lower_config="src/multi_agv_bringup/config/exp3_R1.yaml"
    authorization_config="${FORMAL_EXECUTION_AUTHORIZATION_CONFIG:-src/multi_agv_bringup/config/formal_serial_m2a_r1_authorization.yaml}"
    method_id="M2a_R1"
    expected_lower_mode="R1"
    experiment_id="${FORMAL_EXPERIMENT_ID:-exp2a_m2a_r1_robot2_derating_serial}"
    run_prefix="${FORMAL_RUN_PREFIX:-m2a_r1_serial}"
    ;;
  M2b)
    upper_config="src/multi_agv_bringup/config/exp2b_M2b.yaml"
    lower_config="src/multi_agv_bringup/config/exp2b_M2b.yaml"
    authorization_config="${FORMAL_EXECUTION_AUTHORIZATION_CONFIG:-src/multi_agv_bringup/config/formal_serial_m2b_authorization.yaml}"
    method_id="M2b_M2b"
    expected_lower_mode="M2b"
    experiment_id="${FORMAL_EXPERIMENT_ID:-exp2b_m2b_unloaded_serial}"
    run_prefix="${FORMAL_RUN_PREFIX:-m2b_serial}"
    ;;
  M1)
    upper_config="${FORMAL_UPPER_CONFIG:-src/multi_agv_bringup/config/exp2a_M1_serial_008.yaml}"
    lower_config="src/multi_agv_bringup/config/exp3_R1.yaml"
    authorization_config="${FORMAL_EXECUTION_AUTHORIZATION_CONFIG:-src/multi_agv_bringup/config/formal_serial_m1_r1_authorization.yaml}"
    method_id="M1_R1"
    expected_lower_mode="R1"
    experiment_id="${FORMAL_EXPERIMENT_ID:-exp2a_m1_r1_unloaded_serial}"
    run_prefix="${FORMAL_RUN_PREFIX:-m1_r1_serial}"
    ;;
esac
for required_config in "$runtime_config" "$path_config" \
                       "$evaluation_config" "$upper_config" \
                       "$lower_config" "$authorization_config" \
                       "$derating_authorization_config"; do
  if [[ ! -f "$required_config" ]]; then
    echo "ERROR: required formal configuration is missing: ${required_config}" >&2
    exit 3
  fi
done
if [[ -z "$path_version" || ! "$path_version" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "ERROR: path version is missing or invalid: ${path_version:-<missing>}" >&2
  exit 3
fi
configured_upper_mode="$(awk '
  /^formal_upper:/ {in_upper=1; next}
  /^formal_lower:/ {in_upper=0}
  in_upper && /^[[:space:]]*mode:/ {print $2; exit}
' "$upper_config")"
configured_lower_mode="$(awk '
  /^formal_lower:/ {in_lower=1; next}
  in_lower && /^[[:space:]]*mode:/ {print $2; exit}
' "$lower_config")"
if [[ "$configured_upper_mode" != "$formal_upper_mode" ||
      "$configured_lower_mode" != "$expected_lower_mode" ]]; then
  echo "ERROR: selected method/configuration mismatch: requested=${formal_upper_mode}+${expected_lower_mode}, configured=${configured_upper_mode:-missing}+${configured_lower_mode:-missing}" >&2
  exit 3
fi
if [[ "$formal_upper_mode" == M2a ]]; then
  m2a_capability_policy="$(awk '
    /^formal_upper:/ {in_upper=1; next}
    /^formal_lower:/ {in_upper=0}
    in_upper && /^[[:space:]]*capability_policy:/ {print $2; exit}
  ' "$upper_config")"
  if [[ "$m2a_capability_policy" != log_only_never_used_by_control ]]; then
    echo "ERROR: M2a must record runtime capability without using it for control" >&2
    exit 3
  fi
fi
runtime_status="$(awk '/^[[:space:]]*configuration_status:/ {print $2; exit}' \
  "$runtime_config")"
serial_authorized="$(awk '/^[[:space:]]*serial_execution_authorized:/ {print $2; exit}' \
  "$runtime_config")"
method_upper_authorized="$(awk '
  /^formal_upper:/ {in_upper=1; next}
  /^formal_lower:/ {in_upper=0}
  in_upper && /^[[:space:]]*hardware_execution_authorized:/ {print $2; exit}
' "$authorization_config")"
method_lower_authorized="$(awk '
  /^formal_lower:/ {in_lower=1; next}
  in_lower && /^[[:space:]]*hardware_execution_authorized:/ {print $2; exit}
' "$authorization_config")"
authorized_path_config="$(awk '
  /^authorization_scope:/ {in_scope=1; next}
  in_scope && /^[[:space:]]*path_config:/ {print $2; exit}
' "$authorization_config")"
authorized_upper_config="$(awk '
  /^authorization_scope:/ {in_scope=1; next}
  in_scope && /^[[:space:]]*upper_config:/ {print $2; exit}
' "$authorization_config")"
authorized_runtime_config="$(awk '
  /^authorization_scope:/ {in_scope=1; next}
  in_scope && /^[[:space:]]*runtime_config:/ {print $2; exit}
' "$authorization_config")"
authorized_evaluation_config="$(awk '
  /^authorization_scope:/ {in_scope=1; next}
  in_scope && /^[[:space:]]*evaluation_config:/ {print $2; exit}
' "$authorization_config")"
authorized_target_progress="$(awk '
  /^authorization_scope:/ {in_scope=1; next}
  in_scope && /^[[:space:]]*target_progress:/ {print $2; exit}
' "$authorization_config")"
authorized_robot2_derating="$(awk '
  /^authorization_scope:/ {in_scope=1; next}
  in_scope && /^[[:space:]]*robot2_derating:/ {print $2; exit}
' "$authorization_config")"
derating_hardware_authorized="$(awk '
  /^exp2a_derating_pretest:/ {in_derating=1; next}
  in_derating && /^[[:space:]]*hardware_execution_authorized:/ {print $2; exit}
' "$derating_authorization_config")"
derating_activation_progress="$(awk '
  /^exp2a_derating_pretest:/ {in_window=1; next}
  in_window && /^[[:space:]]*activation_progress:/ {print $2; exit}
' "$evaluation_config")"
derating_restoration_progress="$(awk '
  /^exp2a_derating_pretest:/ {in_window=1; next}
  in_window && /^[[:space:]]*restoration_progress:/ {print $2; exit}
' "$evaluation_config")"
derating_speed_ratio_left="$(awk '
  /^exp2a_derating_pretest:/ {in_window=1; next}
  in_window && /^[[:space:]]*target_speed_ratio_left:/ {print $2; exit}
' "$evaluation_config")"
derating_speed_ratio_right="$(awk '
  /^exp2a_derating_pretest:/ {in_window=1; next}
  in_window && /^[[:space:]]*target_speed_ratio_right:/ {print $2; exit}
' "$evaluation_config")"
nominal_common_velocity="$(awk '
  /^[[:space:]]*leader:/ {in_leader=1; next}
  in_leader && /^[[:space:]]*velocity:/ {print $2; exit}
' "$runtime_config")"
target_progress="$(awk '
  /^[[:space:]]*target_progress:/ {print $2; exit}
' "$runtime_config")"
target_progress="${FORMAL_TARGET_PROGRESS:-$target_progress}"
run_timeout_seconds="${FORMAL_RUN_TIMEOUT_SECONDS:-50}"
if ! awk -v value="$nominal_common_velocity" \
    'BEGIN {exit !(value ~ /^[0-9]+([.][0-9]+)?$/ && value > 0.0)}'; then
  echo "ERROR: runtime leader.velocity is missing or invalid: " \
       "${nominal_common_velocity:-<missing>}" >&2
  exit 3
fi
if ! awk -v value="$target_progress" \
    'BEGIN {exit !(value ~ /^[0-9]+([.][0-9]+)?$/ && value > 1.0)}'; then
  echo "ERROR: runtime target_progress is missing or invalid: " \
       "${target_progress:-<missing>}" >&2
  exit 3
fi
if [[ ! "$run_timeout_seconds" =~ ^[0-9]+$ ]] ||
   (( run_timeout_seconds < 10 || run_timeout_seconds > 300 )); then
  echo "ERROR: run timeout must be an integer from 10 to 300 seconds" >&2
  exit 3
fi
if [[ "$runtime_status" == NEEDS_MANUAL_CONFIRMATION* ||
      "$serial_authorized" != true ]]; then
  echo "ERROR: NEEDS_MANUAL_CONFIRMATION: confirm the 0.16 m/s chassis-applied " \
       "limit and approve 0.18 m/s as the pre-limit demand abort threshold" >&2
  exit 3
fi
if [[ "$method_upper_authorized" != true ||
      "$method_lower_authorized" != true ]]; then
  echo "ERROR: ${method_id} has not received independent physical execution authorization" >&2
  echo "Authorization remains fail-closed in ${authorization_config}" >&2
  exit 3
fi
if [[ "$enable_robot2_derating" == true &&
      "$derating_hardware_authorized" != true ]]; then
  echo "ERROR: Robot2 physical derating is not authorized" >&2
  echo "Authorize the reviewed Robot2 derating scope in ${derating_authorization_config}" >&2
  exit 3
fi
if [[ "$enable_robot2_derating" == true ]]; then
  if ! awk -v start="$derating_activation_progress" \
          -v restore="$derating_restoration_progress" \
          -v target="$target_progress" \
          -v left="$derating_speed_ratio_left" \
          -v right="$derating_speed_ratio_right" 'BEGIN {
        exit !(start >= 0.0 && start < restore && restore < target &&
               left > 0.0 && left < 1.0 && right == left)
      }'; then
    echo "ERROR: invalid or asymmetric Robot2 derating window in ${evaluation_config}" >&2
    exit 3
  fi
fi
if [[ -n "$authorized_path_config" ]] &&
   [[ -n "$authorized_upper_config" &&
      "$upper_config" != "$authorized_upper_config" ||
      "$path_config" != "$authorized_path_config" ||
      "$runtime_config" != "$authorized_runtime_config" ||
      "$evaluation_config" != "$authorized_evaluation_config" ||
      "$target_progress" != "$authorized_target_progress" ||
      "$enable_robot2_derating" != "$authorized_robot2_derating" ]]; then
  echo "ERROR: ${method_id} execution request is outside its authorized scope" >&2
  echo "Authorized scope: upper=${authorized_upper_config:-method-default}, path=${authorized_path_config}, runtime=${authorized_runtime_config}, target=${authorized_target_progress}, Robot2_derating=${authorized_robot2_derating}" >&2
  exit 3
fi
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source src/multi_agv_bringup/scripts/setup_ros_network.sh \
  192.168.6.101 192.168.6.101

if [[ "$(hostname)" != robot1 ]]; then
  echo "ERROR: formal serial orchestration must run on Robot1" >&2
  exit 3
fi
dirty="$(git status --porcelain --untracked-files=normal |
  awk '$2 !~ /^experiment_data\// {print}' || true)"
if [[ -n "$dirty" ]]; then
  echo "WARNING: Robot1 has uncommitted central software changes; " \
       "the recorder will preserve git_dirty and git_status metadata" >&2
  echo "$dirty" >&2
fi
local_sha="$(git rev-parse HEAD)"
for index in 1 2 3; do
  node="/agv${index}/chassis_controller"
  if ! rosnode list | grep -Fqx "$node"; then
    echo "ERROR: missing ${node}" >&2; exit 4
  fi
  transport="$(rosparam get "${node}/transport_type" 2>/dev/null || true)"
  deployed_sha="$(rosparam get "/agv${index}/deployment/git_sha" 2>/dev/null || true)"
  if [[ "$transport" != serial ]]; then
    echo "ERROR: agv${index} is not a serial deployment" >&2
    exit 4
  fi
  available_limit="$(rosparam get \
    "${node}/formal_available_wheel_limit" 2>/dev/null || true)"
  if ! awk -v value="$available_limit" \
      'BEGIN {exit !(value >= 0.159999 && value <= 0.160001)}'; then
    echo "ERROR: agv${index} wheel capability is ${available_limit:-missing}; " \
         "restart all chassis with the confirmed 0.16 m/s envelope" >&2
    exit 4
  fi
  if [[ -z "$deployed_sha" ]] ||
     ! git cat-file -e "${deployed_sha}^{commit}" 2>/dev/null; then
    echo "ERROR: agv${index} deployment Git SHA is missing or unavailable" >&2
    exit 4
  fi
  if [[ "$index" == 1 ]]; then
    chassis_launch_file="src/multi_agv_bringup/launch/car1_master.launch"
  else
    chassis_launch_file="src/multi_agv_bringup/launch/car${index}_client.launch"
  fi
  chassis_paths=(
    src/agv_msgs
    src/common
    src/chassis_controller
    src/multi_agv_bringup/config/common_platform.yaml
    "src/multi_agv_bringup/config/agv${index}_chassis.yaml"
    src/multi_agv_bringup/launch/chassis_single.launch
    "$chassis_launch_file"
    src/multi_agv_bringup/scripts/start_three_car_chassis.sh
    src/multi_agv_bringup/scripts/check_local_clock_sync.sh
    src/multi_agv_bringup/scripts/setup_ros_network.sh
  )
  if ! git diff --quiet "$deployed_sha" "$local_sha" -- "${chassis_paths[@]}"; then
    echo "ERROR: agv${index} chassis deployment is incompatible with ${local_sha}" >&2
    echo "Synchronize and restart agv${index}; relevant changes:" >&2
    git diff --name-only "$deployed_sha" "$local_sha" -- \
      "${chassis_paths[@]}" >&2
    exit 4
  fi
  chassis_worktree_changes="$(
    git status --porcelain --untracked-files=all -- "${chassis_paths[@]}"
  )"
  if [[ -n "$chassis_worktree_changes" ]]; then
    echo "ERROR: agv${index} has uncommitted chassis-relevant changes" >&2
    echo "Commit, synchronize and restart the affected chassis before running:" >&2
    echo "$chassis_worktree_changes" >&2
    exit 4
  fi
  if [[ "$deployed_sha" != "$local_sha" ]]; then
    echo "agv${index}: accepting chassis-compatible deployment ${deployed_sha}"
  fi
done
for node in /pose_provider /camera_odom_fusion; do
  rosnode list | grep -Fqx "$node" || {
    echo "ERROR: missing camera fusion node ${node}" >&2; exit 5; }
done
for forbidden in /formal_algorithm /formal_fake_algorithm \
                 /three_car_unloaded_bounded_pretest /multi_agv_controller \
                 /path_state_estimator /experiment_recorder; do
  if rosnode list | grep -Fqx "$forbidden"; then
    echo "ERROR: conflicting node is running: ${forbidden}" >&2
    exit 5
  fi
done
alive="$(timeout 3 rostopic echo -n 1 /vision/aruco/alive 2>/dev/null |
  awk '/data:/ {print $2; exit}' || true)"
[[ "$alive" == True ]] || { echo "ERROR: ArUco stream is not alive" >&2; exit 5; }

echo "EXPERIMENT_METHOD=${method_id} upper=${configured_upper_mode} lower=${configured_lower_mode}"
echo "SHARED_CONDITION=R0.7_CW_SMOOTH_EXIT speed=${nominal_common_velocity}m/s target=${target_progress}m"
if [[ "$enable_robot2_derating" == true ]]; then
  echo "ROBOT2_DERATING=enabled ratio=${derating_speed_ratio_left} activation=${derating_activation_progress}m restoration=${derating_restoration_progress}m"
  if [[ "$formal_upper_mode" == M2a ]]; then
    echo "M2A_CAPABILITY_POLICY=${m2a_capability_policy}; fixed upper boundary remains independent of runtime capability"
  fi
else
  echo "ROBOT2_DERATING=disabled"
fi

estimator_pid=""; algorithm_pid=""; evaluation_pid=""; recorder_pid=""
cleanup() {
  for pid in "$recorder_pid" "$evaluation_pid" "$algorithm_pid" "$estimator_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
  for pid in "$recorder_pid" "$evaluation_pid" "$algorithm_pid" "$estimator_pid"; do
    if [[ -n "$pid" ]]; then wait "$pid" 2>/dev/null || true; fi
  done
}
trap cleanup EXIT INT TERM HUP

stop_runtime_nodes() {
  for pid in "$evaluation_pid" "$algorithm_pid" "$estimator_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
  for pid in "$evaluation_pid" "$algorithm_pid" "$estimator_pid"; do
    if [[ -n "$pid" ]]; then wait "$pid" 2>/dev/null || true; fi
  done
  evaluation_pid=""
  algorithm_pid=""
  estimator_pid=""
}

roslaunch multi_agv_bringup camera_fused_virtual_load_state_estimator.launch \
  path_config:="${workspace}/${path_config}" \
  runtime_config:="${workspace}/${runtime_config}" &
estimator_pid="$!"
deadline=$((SECONDS + 15))
until rosnode list 2>/dev/null | grep -Fqx /path_state_estimator; do
  if ! kill -0 "$estimator_pid" 2>/dev/null; then wait "$estimator_pid"; fi
  (( SECONDS < deadline )) || { echo "ERROR: state estimator timeout" >&2; exit 6; }
  sleep 0.2
done
rosrun multi_agv_bringup check_three_car_readonly_gate.py \
  --observe-seconds 5 --require-fused-cooperative-state

run_id="${run_prefix}_$(date +%Y%m%d_%H%M%S)"
roslaunch multi_agv_bringup formal_serial_m1_r1.launch \
  upper_config:="${workspace}/${upper_config}" \
  lower_config:="${workspace}/${lower_config}" \
  authorization_config:="${workspace}/${authorization_config}" \
  path_config:="${workspace}/${path_config}" \
  runtime_config:="${workspace}/${runtime_config}" \
  experiment_id:="$experiment_id" \
  target_progress:="$target_progress" \
  enable_commands:=true \
  confirm_test_area_clear:=true \
  confirm_wheels_on_floor:=true \
  confirm_unloaded_30cm_fixture:=true &
algorithm_pid="$!"
roslaunch multi_agv_bringup formal_evaluation_window.launch \
  platform_transport_type:=serial \
  evaluation_config:="${workspace}/${evaluation_config}" \
  authorization_config:="${workspace}/${derating_authorization_config}" \
  derating_publication_authorized:="$enable_robot2_derating" \
  confirm_robot2_local_derating:="$enable_robot2_derating" \
  run_id:="$run_id" method_id:="$method_id" \
  experiment_id:="$experiment_id" block_id:="$pair_block" &
evaluation_pid="$!"
deadline=$((SECONDS + 10))
until rosnode list 2>/dev/null | grep -Fqx /formal_evaluation_supervisor; do
  if ! kill -0 "$evaluation_pid" 2>/dev/null; then wait "$evaluation_pid"; fi
  (( SECONDS < deadline )) || { echo "ERROR: evaluation supervisor timeout" >&2; exit 6; }
  sleep 0.2
done

output_root="${workspace}/experiment_data/formal_serial_unloaded"
roslaunch multi_agv_bringup experiment.launch \
  arming_authorized:=true \
  output_root:="$output_root" run_id:="$run_id" \
  experiment_id:="$experiment_id" method_id:="$method_id" \
  pair_block_id:="$pair_block" payload_state:=unloaded \
  localization_source:=camera_imu_wheel_fused operator:="$operator" \
  camera_mode:=true virtual_load_from_robots:=true \
  require_windows_sender_manifest:=false \
  localization_config:="${workspace}/src/multi_agv_bringup/config/localization_camera_three_car_closed_loop.yaml" \
  runtime_config:="${workspace}/${runtime_config}" \
  upper_config:="${workspace}/${upper_config}" \
  lower_config:="${workspace}/${lower_config}" \
  execution_authorization_config:="${workspace}/${authorization_config}" \
  derating_authorization_config:="${workspace}/${derating_authorization_config}" \
  evaluation_config:="${workspace}/${evaluation_config}" \
  path_config:="${workspace}/${path_config}" \
  path_version:="$path_version" \
  nominal_common_velocity:="$nominal_common_velocity" \
  evaluation_target:="$target_progress" &
recorder_pid="$!"

deadline=$((SECONDS + run_timeout_seconds))
reached=false
while (( SECONDS < deadline )); do
  for pid in "$algorithm_pid" "$evaluation_pid" "$recorder_pid"; do
    if ! kill -0 "$pid" 2>/dev/null; then wait "$pid"; fi
  done
  progress="$(timeout 2 rostopic echo -n 1 /multi_agv/path_reference 2>/dev/null |
    awk '/load_path_progress_reference:/ {print $2; exit}' || true)"
  if [[ -n "$progress" ]] && awk -v value="$progress" \
      -v target="$target_progress" \
      'BEGIN {exit !(value >= target - 0.000001)}'; then
    reached=true; break
  fi
  sleep 0.2
done
if [[ "$reached" != true ]]; then
  echo "ERROR: ${method_id} did not reach the bounded target" >&2
  exit 7
fi

python3 src/multi_agv_bringup/scripts/wait_three_car_wheel_stop.py \
  --timeout 6.0 --speed-threshold 0.01 --stable-seconds 0.10 || {
    echo "ERROR: wheel stop was not confirmed" >&2
    exit 8
  }

rosservice call /experiment_recorder/stop >/dev/null
wait "$recorder_pid" || true
recorder_pid=""
run_dir="${output_root}/${run_id}"
stop_runtime_nodes
echo "PHYSICAL_TASK_STATUS=PASSED run_dir=${run_dir}"
if rosrun multi_agv_analysis process_experiment_run.py "$run_dir" \
    "$(rospack find multi_agv_analysis)/config/validation_defaults.yaml"; then
  echo "POSTPROCESS_STATUS=PASSED"
  echo "SUMMARY: ${run_dir}/summary_metrics.json"
  echo "VALIDATION: ${run_dir}/validation.json"
  echo "RAW FIGURES: ${run_dir}/plots_raw"
  echo "SMOOTHED FIGURES: ${run_dir}/plots_smoothed_0p8s"
else
  postprocess_result="$?"
  echo "POSTPROCESS_STATUS=FAILED step=bag_csv_validation_metrics_or_plots" >&2
  echo "The physical task passed. Its bag and run directory are retained:" >&2
  echo "${run_dir}" >&2
  echo "Offline retry:" >&2
  echo "rosrun multi_agv_analysis process_experiment_run.py '${run_dir}' '$(rospack find multi_agv_analysis)/config/validation_defaults.yaml'" >&2
  exit $((20 + postprocess_result))
fi
echo "${method_id} SERIAL RUN AND AUTOMATIC FIGURES COMPLETE: ${run_dir}"
