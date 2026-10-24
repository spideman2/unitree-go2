cmake_minimum_required(VERSION 3.16)
project(unitree_go2 VERSION 0.1.0 LANGUAGES C CXX)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
# Import motor's driver dependencies before the SDK aggregate adds its targets.
find_package(motor REQUIRED)
find_package(Components REQUIRED)
add_library(go2_motor INTERFACE)
if(MOTOR_WHOLE_ARCHIVE)
  # The SDK motor library is static; retain its constructor-based driver registry.
  target_link_libraries(go2_motor INTERFACE
    -Wl,--whole-archive motor::motor -Wl,--no-whole-archive)
else()
  target_link_libraries(go2_motor INTERFACE motor::motor)
endif()
option(BUILD_TESTING "Build Go2 tests" OFF)
include(CTest)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetMode.srv"
  "srv/Calibration.srv")

add_library(go2_control STATIC
  src/hardware.cpp src/controller.cpp
  src/policy.cpp src/policy_adapter.cpp)
target_include_directories(go2_control PUBLIC src)
target_link_libraries(go2_control PUBLIC
  components::rl Eigen3::Eigen go2_motor components::imu Threads::Threads yaml-cpp)
target_compile_options(go2_control PRIVATE -Wall -Wextra -Wpedantic)
add_executable(go2_node src/main.cpp src/node.cpp)
target_link_libraries(go2_node go2_control nlohmann_json::nlohmann_json components::led)
ament_target_dependencies(go2_node rclcpp rcl_interfaces ament_index_cpp geometry_msgs sensor_msgs std_msgs std_srvs)
rosidl_get_typesupport_target(typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(go2_node ${typesupport})
add_executable(sbus_teleop_node src/sbus_teleop_node.cpp)
target_link_libraries(sbus_teleop_node components::rc_receiver ${typesupport})
target_compile_options(sbus_teleop_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(sbus_teleop_node rclcpp rcl_interfaces geometry_msgs std_msgs std_srvs)
add_executable(motor_brake_capture tool/calibration/motor_brake_capture.cpp)
target_link_libraries(motor_brake_capture go2_motor Threads::Threads)
target_compile_options(motor_brake_capture PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS go2_node sbus_teleop_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS motor_brake_capture DESTINATION lib/${PROJECT_NAME}/tool/calibration)
install(DIRECTORY config launch resources DESTINATION share/${PROJECT_NAME}
        PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
install(DIRECTORY clients/web DESTINATION share/${PROJECT_NAME})
install(DIRECTORY policy DESTINATION share/${PROJECT_NAME} OPTIONAL
        FILES_MATCHING PATTERN "*.onnx")
install(PROGRAMS scripts/web_teleop_server.py scripts/go2_network.py
        scripts/download_models_go2.py
        scripts/start-go2.sh DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS tool/calibration/calibrate_robot.py DESTINATION lib/${PROJECT_NAME}/tool/calibration)
install(FILES tool/calibration/calibration_pose_xyz.yaml
        tool/calibration/CALIBRATION.md
        DESTINATION share/${PROJECT_NAME}/tool/calibration)
install(DIRECTORY tool/calibration/assets DESTINATION share/${PROJECT_NAME}/tool/calibration)
install(FILES README.md LICENSE NOTICE
        DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  set(GO2_TEST_MODEL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/policy" CACHE PATH "Go2 test model assets")
  add_executable(sbus_event_test test/sbus_event_test.cpp)
  target_link_libraries(sbus_event_test components::rc_receiver ${typesupport})
  target_compile_options(sbus_event_test PRIVATE -UNDEBUG)
  ament_target_dependencies(sbus_event_test rclcpp rcl_interfaces geometry_msgs std_msgs std_srvs)
  add_test(NAME go2_sbus_event COMMAND sbus_event_test)
  set_tests_properties(go2_sbus_event PROPERTIES TIMEOUT 15 ENVIRONMENT "ROS_DOMAIN_ID=197;ROS_LOCALHOST_ONLY=1")
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME go2_web_teleop COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/web_teleop_test.py)
  set_tests_properties(go2_web_teleop PROPERTIES TIMEOUT 15)
  add_test(NAME go2_calibration COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/calibration_test.py --capture-tool $<TARGET_FILE:motor_brake_capture>)
  set_tests_properties(go2_calibration PROPERTIES TIMEOUT 30)
  add_test(NAME go2_board_profiles COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/board_profiles_test.py ${CMAKE_CURRENT_SOURCE_DIR})
  add_library(no_hardware_io SHARED test/no_hardware_io.c)
  target_link_libraries(no_hardware_io dl)
  foreach(test_name policy controller hardware skill)
    add_executable(${test_name}_test test/${test_name}_test.cpp)
    target_link_libraries(${test_name}_test PRIVATE go2_control)
    target_compile_options(${test_name}_test PRIVATE -UNDEBUG)
    target_compile_definitions(${test_name}_test PRIVATE
      GO2_TEST_APP="${CMAKE_CURRENT_SOURCE_DIR}"
      GO2_TEST_MODEL_DIR="${GO2_TEST_MODEL_DIR}"
      GO2_TEST_POLICY_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/policies.yaml")
    add_test(NAME go2_${test_name} COMMAND ${test_name}_test "${GO2_TEST_MODEL_DIR}")
    set_tests_properties(go2_${test_name} PROPERTIES TIMEOUT 30)
  endforeach()
  set_tests_properties(go2_hardware PROPERTIES ENVIRONMENT "LD_PRELOAD=$<TARGET_FILE:no_hardware_io>")
endif()
ament_export_dependencies(rosidl_default_runtime)
ament_package()
