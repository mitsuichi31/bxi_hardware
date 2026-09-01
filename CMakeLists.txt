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
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/mit_protocol.cpp
  src/socketcan_bus.cpp
  src/bxi_actuator.cpp
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_mit_protocol test/test_mit_protocol.cpp)
  target_link_libraries(test_mit_protocol ${PROJECT_NAME})
  set_target_properties(test_mit_protocol PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}")
  ament_add_gtest(test_bxi_actuator test/test_bxi_actuator.cpp)
  target_link_libraries(test_bxi_actuator ${PROJECT_NAME})
  set_target_properties(test_bxi_actuator PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}")
  ament_add_gtest(test_vcan_lifecycle test/test_vcan_lifecycle.cpp)
  target_link_libraries(test_vcan_lifecycle ${PROJECT_NAME})
  set_target_properties(test_vcan_lifecycle PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}")
  ament_add_pytest_test(test_validate_config test/test_validate_config.py)
endif()

install(
  DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(PROGRAMS scripts/validate_config.py DESTINATION lib/${PROJECT_NAME})
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
