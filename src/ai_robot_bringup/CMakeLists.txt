cmake_minimum_required(VERSION 3.8)
project(ai_robot_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(
    test/test_bringup_launch.py
    TIMEOUT 30
    ENV "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs"
  )
endif()

ament_package()
