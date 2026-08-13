cmake_minimum_required(VERSION 3.8)
project(ai_robot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(description_test test/test_description.py)
endif()

ament_package()
