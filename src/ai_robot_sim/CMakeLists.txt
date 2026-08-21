cmake_minimum_required(VERSION 3.8)
project(ai_robot_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch models worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_pytest_test(sim_config_test test/test_sim_config.py)
  add_launch_test(
    test/test_sim_base_launch.py
    TIMEOUT 120
    ENV
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs"
      "ROS_DOMAIN_ID=81"
      "IGN_PARTITION=ai_robot_m2_test"
  )
  add_launch_test(
    test/test_sensor_interfaces_launch.py
    TIMEOUT 120
    ENV
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs"
      "ROS_DOMAIN_ID=82"
      "IGN_PARTITION=ai_robot_m3_sensor_test"
  )
  set_tests_properties(
    test_test_sim_base_launch.py
    test_test_sensor_interfaces_launch.py
    PROPERTIES RUN_SERIAL TRUE
  )
endif()

ament_package()
