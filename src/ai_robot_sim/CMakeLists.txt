cmake_minimum_required(VERSION 3.8)
project(ai_robot_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_pytest_test(sim_config_test test/test_sim_config.py)
  add_launch_test(
    test/test_sim_base_launch.py
    TIMEOUT 120
    ENV "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs"
  )
endif()

ament_package()
