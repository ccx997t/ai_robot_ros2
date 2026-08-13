cmake_minimum_required(VERSION 3.8)
project(ai_robot_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(sim_config_test test/test_sim_config.py)
endif()

ament_package()
