cmake_minimum_required(VERSION 3.8)
project(bxi_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)

# The hardware library keeps the file name libbxi_hardware.so (bxi_hardware.xml), but its CMake
# target has another name: rosidl needs the target named after the package, because the type
# support is loaded at run time as lib<package>__rosidl_typesupport_<rmw>.so.
set(BXI_HW_LIB bxi_hardware_hw)
add_library(${BXI_HW_LIB} SHARED
  src/mit_protocol.cpp
  src/socketcan_bus.cpp
  src/bxi_actuator.cpp
  src/bus_streamer.cpp
  src/stream_trace_io.cpp
  src/bxi_system_interface.cpp
  src/pair_system_interface.cpp
  src/hil_trajectory.cpp
)
target_include_directories(${BXI_HW_LIB} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${BXI_HW_LIB}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(${BXI_HW_LIB} yaml-cpp)
set_target_properties(${BXI_HW_LIB} PROPERTIES OUTPUT_NAME bxi_hardware)

pluginlib_export_plugin_description_file(hardware_interface bxi_hardware.xml)

# 1軸の HIL 用の軌道 controller（RT 軌道 controller の計画）とその型
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HilTrajectoryStatus.msg"
  "srv/StartHilTrajectory.srv"
)
rosidl_get_typesupport_target(bxi_interfaces_cpp ${PROJECT_NAME} rosidl_typesupport_cpp)
add_library(bxi_hil_controllers SHARED src/hil_trajectory_controller.cpp)
target_include_directories(bxi_hil_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bxi_hil_controllers
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs
)
target_link_libraries(bxi_hil_controllers ${BXI_HW_LIB} "${bxi_interfaces_cpp}")
pluginlib_export_plugin_description_file(controller_interface bxi_hil_controllers.xml)

# G N-axis plugin. Keep the legacy plugin, interfaces and tests independent.
add_library(bxi_n_axis_controllers SHARED src/n_axis_trajectory_controller.cpp)
target_include_directories(bxi_n_axis_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(bxi_n_axis_controllers controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle)
target_link_libraries(bxi_n_axis_controllers ${BXI_HW_LIB})
pluginlib_export_plugin_description_file(controller_interface bxi_n_axis_controllers.xml)

# 試験専用の controller（vcan の故障経路：controller_manager の更新の停止）。本番の controller とは
# 別のライブラリにする。BXI_ALLOW_TEST_FAULTS=1 のときだけ設定できる。
add_library(bxi_hil_test_controllers SHARED src/hil_update_stall_controller.cpp)
target_include_directories(bxi_hil_test_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bxi_hil_test_controllers
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_srvs
)
pluginlib_export_plugin_description_file(controller_interface bxi_hil_test_controllers.xml)

# 実機の前提確認ツール（1軸）
add_executable(bxi_probe tools/bxi_probe.cpp)
target_link_libraries(bxi_probe ${BXI_HW_LIB})

# vcan 専用の模擬モーター（スケジューリング条件の比較用）
add_executable(bxi_vcan_motor_emulator tools/bxi_vcan_motor_emulator.cpp)
target_link_libraries(bxi_vcan_motor_emulator ${BXI_HW_LIB})

add_executable(sine_command_cpp hil/single_joint/sine_command.cpp)
ament_target_dependencies(sine_command_cpp rclcpp sensor_msgs std_msgs)
target_link_libraries(sine_command_cpp ${BXI_HW_LIB})

if(BUILD_TESTING)
  add_library(bxi_hil_trace_test SHARED src/hil_trace_benchmark_system.cpp)
  target_link_libraries(bxi_hil_trace_test ${BXI_HW_LIB} yaml-cpp)
  ament_target_dependencies(bxi_hil_trace_test hardware_interface pluginlib rclcpp rclcpp_lifecycle)
  pluginlib_export_plugin_description_file(hardware_interface bxi_hil_trace_test.xml)
  install(TARGETS bxi_hil_trace_test LIBRARY DESTINATION lib)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_hil_trace_benchmark_guard test/test_hil_trace_benchmark_guard.cpp)
  target_link_libraries(test_hil_trace_benchmark_guard ${BXI_HW_LIB})
  ament_target_dependencies(test_hil_trace_benchmark_guard pluginlib hardware_interface)
  target_compile_definitions(test_hil_trace_benchmark_guard PRIVATE BXI_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  ament_add_gtest(test_mit_protocol test/test_mit_protocol.cpp)
  target_link_libraries(test_mit_protocol ${BXI_HW_LIB})
  set_target_properties(test_mit_protocol PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}")
  ament_add_gtest(test_can_frame test/test_can_frame.cpp)
  target_include_directories(test_can_frame PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_bxi_actuator test/test_bxi_actuator.cpp)
  target_link_libraries(test_bxi_actuator ${BXI_HW_LIB})
  set_target_properties(test_bxi_actuator PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}")
  ament_add_gtest(test_stream_trace_io test/test_stream_trace_io.cpp)
  target_link_libraries(test_stream_trace_io ${BXI_HW_LIB} yaml-cpp)
  set_target_properties(test_stream_trace_io PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}")
  ament_add_gtest(test_pair_system test/test_pair_system.cpp)
  target_link_libraries(test_pair_system ${BXI_HW_LIB})
  ament_target_dependencies(test_pair_system hardware_interface rclcpp rclcpp_lifecycle)
  target_compile_definitions(test_pair_system PRIVATE BXI_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

  ament_add_gtest(test_pair_commit test/test_pair_commit.cpp)
  target_link_libraries(test_pair_commit ${BXI_HW_LIB})

  ament_add_gtest(test_atomic_snapshot test/test_atomic_snapshot.cpp)
  target_link_libraries(test_atomic_snapshot ${BXI_HW_LIB})
  ament_add_gtest(test_bus_streamer test/test_bus_streamer.cpp)
  target_link_libraries(test_bus_streamer ${BXI_HW_LIB})
  set_target_properties(test_bus_streamer PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}")
  ament_add_gtest(test_activation_rollback test/test_activation_rollback.cpp)
  target_link_libraries(test_activation_rollback ${BXI_HW_LIB})
  target_compile_definitions(test_activation_rollback PRIVATE
    BXI_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  set_target_properties(test_activation_rollback PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}")
  ament_add_gtest(test_vcan_lifecycle test/test_vcan_lifecycle.cpp)
  target_link_libraries(test_vcan_lifecycle ${BXI_HW_LIB})
  set_target_properties(test_vcan_lifecycle PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}")
  ament_add_gtest(test_vcan_stream_system test/test_vcan_stream_system.cpp TIMEOUT 120)
  target_link_libraries(test_vcan_stream_system ${BXI_HW_LIB})
  target_compile_definitions(test_vcan_stream_system PRIVATE
    BXI_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  set_target_properties(test_vcan_stream_system PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}")
  ament_add_pytest_test(test_validate_config test/test_validate_config.py)
  ament_add_pytest_test(test_write_stream_tools test/test_write_stream_tools.py)
  ament_add_pytest_test(test_analyze_stream_timing test/test_analyze_stream_timing.py)
  ament_add_pytest_test(test_compare_stream_scheduling test/test_compare_stream_scheduling.py)
  ament_add_pytest_test(test_sine_trajectory test/test_sine_trajectory.py)
  ament_add_pytest_test(test_trajectory_reference test/test_trajectory_reference.py)
  ament_add_pytest_test(test_hil_trajectory_command test/test_hil_trajectory_command.py TIMEOUT 45)
  ament_add_gtest(test_n_axis_trajectory_controller test/test_n_axis_trajectory_controller.cpp)
  target_link_libraries(test_n_axis_trajectory_controller bxi_n_axis_controllers)
  ament_target_dependencies(test_n_axis_trajectory_controller controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle)
  set_target_properties(test_n_axis_trajectory_controller PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}")

  ament_add_gtest(test_hil_trajectory_controller test/test_hil_trajectory_controller.cpp)
  target_link_libraries(test_hil_trajectory_controller bxi_hil_controllers)
  set_target_properties(test_hil_trajectory_controller PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}")
  ament_add_gtest(test_hil_update_stall_controller test/test_hil_update_stall_controller.cpp)
  target_link_libraries(test_hil_update_stall_controller bxi_hil_test_controllers)
  set_target_properties(test_hil_update_stall_controller PROPERTIES
    BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}")
  ament_add_gtest(test_hil_trajectory test/test_hil_trajectory.cpp)
  target_link_libraries(test_hil_trajectory ${BXI_HW_LIB})
  target_compile_definitions(test_hil_trajectory PRIVATE
    BXI_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  set_target_properties(test_hil_trajectory PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}")
endif()

install(
  DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(PROGRAMS scripts/validate_config.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/analyze_write_stream.py scripts/make_write_stream_config.py
  DESTINATION lib/${PROJECT_NAME})
# 1軸の実機試験（ベンチ）用の設定・launch・指令スクリプト
install(DIRECTORY hil/ DESTINATION share/${PROJECT_NAME}/hil
  PATTERN "*.py" EXCLUDE PATTERN "*.cpp" EXCLUDE PATTERN "__pycache__" EXCLUDE)
install(FILES hil/single_joint/single_joint_hil.launch.py
  DESTINATION share/${PROJECT_NAME}/hil/single_joint)
install(PROGRAMS hil/single_joint/ramp_command.py hil/single_joint/sine_command.py
  hil/single_joint/hil_trajectory.py
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS bxi_probe bxi_vcan_motor_emulator sine_command_cpp
  DESTINATION lib/${PROJECT_NAME})
install(
  TARGETS ${BXI_HW_LIB} bxi_hil_controllers bxi_hil_test_controllers bxi_n_axis_controllers
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rosidl_default_runtime
  std_msgs
  std_srvs
)

ament_package()
