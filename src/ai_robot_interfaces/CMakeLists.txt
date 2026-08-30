cmake_minimum_required(VERSION 3.8)
project(ai_robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteRobotTask.action"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(task_action_contract_test test/test_task_action_contract.py)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
