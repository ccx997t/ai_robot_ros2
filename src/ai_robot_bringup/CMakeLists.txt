cmake_minimum_required(VERSION 3.8)
project(ai_robot_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch maps DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_pytest_test(m4_launch_contract_test test/test_m4_launch_contract.py)
  ament_add_pytest_test(m5_mapping_contract_test test/test_m5_mapping_contract.py)
  ament_add_pytest_test(m5_map_artifact_test test/test_m5_map_artifact.py)
  ament_add_pytest_test(
    m5_localization_contract_test test/test_m5_localization_contract.py)
  add_launch_test(
    test/test_bringup_launch.py
    TIMEOUT 30
    ENV "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs"
  )
  add_launch_test(
    test/test_m4_motion_launch.py
    TIMEOUT 120
    ENV
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs"
      "ROS_DOMAIN_ID=83"
      "IGN_PARTITION=ai_robot_m4_motion_test"
  )
  add_launch_test(
    test/test_m4_fault_safety_launch.py
    TIMEOUT 45
    ENV
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs"
      "ROS_DOMAIN_ID=84"
  )
  add_launch_test(
    test/test_m5_mapping_launch.py
    TIMEOUT 120
    ENV
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs"
      "ROS_DOMAIN_ID=85"
      "IGN_PARTITION=ai_robot_m5_mapping_test"
  )
  add_launch_test(
    test/test_m5_map_reload_launch.py
    TIMEOUT 45
    ENV
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs"
      "ROS_DOMAIN_ID=86"
  )
  add_launch_test(
    test/test_m5_localization_launch.py
    TIMEOUT 120
    ENV
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs"
      "ROS_DOMAIN_ID=87"
      "IGN_PARTITION=ai_robot_m5_localization_test"
  )
  set_tests_properties(
    test_test_bringup_launch.py
    test_test_m4_fault_safety_launch.py
    test_test_m4_motion_launch.py
    test_test_m5_mapping_launch.py
    test_test_m5_map_reload_launch.py
    test_test_m5_localization_launch.py
    PROPERTIES RUN_SERIAL TRUE
  )
endif()

ament_package()
