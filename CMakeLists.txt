cmake_minimum_required(VERSION 3.8)
project(bxi_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/mit_protocol.cpp
  src/socketcan_bus.cpp
  src/bxi_actuator.cpp
  src/bus_streamer.cpp
  src/bxi_system_interface.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

pluginlib_export_plugin_description_file(hardware_interface bxi_hardware.xml)

# 実機の前提確認ツール（1軸）
add_executable(bxi_probe tools/bxi_probe.cpp)
target_link_libraries(bxi_probe ${PROJECT_NAME})

# vcan 専用の模擬モーター（スケジューリング条件の比較用）
add_executable(bxi_vcan_motor_emulator tools/bxi_vcan_motor_emulator.cpp)
target_link_libraries(bxi_vcan_motor_emulator ${PROJECT_NAME})

add_executable(sine_command_cpp hil/single_joint/sine_command.cpp)
ament_target_dependencies(sine_command_cpp rclcpp sensor_msgs std_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_mit_protocol test/test_mit_protocol.cpp)
  target_link_libraries(test_mit_protocol ${PROJECT_NAME})
  set_target_properties(test_mit_protocol PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}")
  ament_add_gtest(test_can_frame test/test_can_frame.cpp)
  target_include_directories(test_can_frame PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_bxi_actuator test/test_bxi_actuator.cpp)
  target_link_libraries(test_bxi_actuator ${PROJECT_NAME})
  set_target_properties(test_bxi_actuator PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}")
  ament_add_gtest(test_bus_streamer test/test_bus_streamer.cpp)
  target_link_libraries(test_bus_streamer ${PROJECT_NAME})
  set_target_properties(test_bus_streamer PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}")
  ament_add_gtest(test_activation_rollback test/test_activation_rollback.cpp)
  target_link_libraries(test_activation_rollback ${PROJECT_NAME})
  target_compile_definitions(test_activation_rollback PRIVATE
    BXI_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  set_target_properties(test_activation_rollback PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}")
  ament_add_gtest(test_vcan_lifecycle test/test_vcan_lifecycle.cpp)
  target_link_libraries(test_vcan_lifecycle ${PROJECT_NAME})
  set_target_properties(test_vcan_lifecycle PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}")
  ament_add_gtest(test_vcan_stream_system test/test_vcan_stream_system.cpp TIMEOUT 120)
  target_link_libraries(test_vcan_stream_system ${PROJECT_NAME})
  target_compile_definitions(test_vcan_stream_system PRIVATE
    BXI_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  set_target_properties(test_vcan_stream_system PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}")
  ament_add_pytest_test(test_validate_config test/test_validate_config.py)
  ament_add_pytest_test(test_analyze_stream_timing test/test_analyze_stream_timing.py)
  ament_add_pytest_test(test_compare_stream_scheduling test/test_compare_stream_scheduling.py)
  ament_add_pytest_test(test_sine_trajectory test/test_sine_trajectory.py)
endif()

install(
  DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(PROGRAMS scripts/validate_config.py DESTINATION lib/${PROJECT_NAME})
# 1軸の実機試験（ベンチ）用の設定・launch・指令スクリプト
install(DIRECTORY hil/ DESTINATION share/${PROJECT_NAME}/hil
  PATTERN "*.py" EXCLUDE PATTERN "*.cpp" EXCLUDE PATTERN "__pycache__" EXCLUDE)
install(FILES hil/single_joint/single_joint_hil.launch.py
  DESTINATION share/${PROJECT_NAME}/hil/single_joint)
install(PROGRAMS hil/single_joint/ramp_command.py hil/single_joint/sine_command.py
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS bxi_probe bxi_vcan_motor_emulator sine_command_cpp
  DESTINATION lib/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
