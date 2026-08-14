cmake_minimum_required(VERSION 3.8)
project(ai_robot_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_pytest_test(m4_launch_contract_test test/test_m4_launch_contract.py)
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
  set_tests_properties(
    test_test_bringup_launch.py
    test_test_m4_motion_launch.py
    PROPERTIES RUN_SERIAL TRUE
  )
endif()

ament_package()
