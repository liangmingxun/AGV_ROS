cmake_minimum_required(VERSION 3.0.2)
project(multi_agv_analysis)

find_package(catkin REQUIRED COMPONENTS
  agv_msgs
  diagnostic_msgs
  rosbag
  rosgraph
  rospy
  std_msgs
  std_srvs
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/bag_to_csv.py
  scripts/calibration_workspace.py
  scripts/camera_quality_report.py
  scripts/compute_metrics.py
  scripts/aggregate_experiment.py
  scripts/experiment_batch.py
  scripts/fleet_health_check.py
  scripts/manage_stage_evidence.py
  scripts/record_experiment.py
  scripts/software_watchdog.py
  scripts/validate_run.py
  scripts/verify_config_registry.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_metrics.py)
  catkin_add_nosetests(test/test_tooling.py)
endif()
