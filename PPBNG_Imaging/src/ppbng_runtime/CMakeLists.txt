cmake_minimum_required(VERSION 3.16)
project(ppbng_runtime LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ppbng_interfaces REQUIRED)
find_package(ppbng_storage REQUIRED)
find_package(std_srvs REQUIRED)

add_library(hardware_safety_gate src/hardware_safety_gate.cpp)
target_compile_features(hardware_safety_gate PUBLIC cxx_std_17)
target_include_directories(hardware_safety_gate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(production_workflow src/production_workflow.cpp)
target_compile_features(production_workflow PUBLIC cxx_std_17)
target_include_directories(production_workflow PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(storage_capacity_guard src/storage_capacity_guard.cpp)
target_compile_features(storage_capacity_guard PUBLIC cxx_std_17)
target_include_directories(storage_capacity_guard PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(production_acquisition_manager_node
  src/production_acquisition_manager_node.cpp)
target_compile_features(production_acquisition_manager_node PRIVATE cxx_std_17)
target_link_libraries(production_acquisition_manager_node
  hardware_safety_gate production_workflow storage_capacity_guard)
ament_target_dependencies(production_acquisition_manager_node
  rclcpp ppbng_interfaces ppbng_storage std_srvs)
if(MSVC)
  target_compile_options(production_acquisition_manager_node PRIVATE /W4 /permissive-)
else()
  target_compile_options(production_acquisition_manager_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  target_compile_options(hardware_safety_gate PRIVATE /W4 /permissive-)
else()
  target_compile_options(hardware_safety_gate PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(production_control_bridge_node src/production_control_bridge_node.cpp)
target_compile_features(production_control_bridge_node PRIVATE cxx_std_17)
ament_target_dependencies(production_control_bridge_node rclcpp ppbng_interfaces std_srvs)
if(MSVC)
  target_compile_options(production_control_bridge_node PRIVATE /W4 /permissive-)
  target_link_libraries(production_control_bridge_node ws2_32)
else()
  target_compile_options(production_control_bridge_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS production_acquisition_manager_node production_control_bridge_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS hardware_safety_gate production_workflow storage_capacity_guard
  EXPORT hardware_safety_gateTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_operator_warning_status test/test_operator_warning_status.cpp)
  target_link_libraries(test_operator_warning_status production_workflow)
  ament_add_gtest(test_hardware_safety_gate test/test_hardware_safety_gate.cpp)
  target_link_libraries(test_hardware_safety_gate hardware_safety_gate)
  ament_add_gtest(test_production_workflow test/test_production_workflow.cpp)
  target_link_libraries(test_production_workflow production_workflow)
  ament_add_gtest(test_storage_capacity_guard test/test_storage_capacity_guard.cpp)
  target_link_libraries(test_storage_capacity_guard storage_capacity_guard)
endif()

ament_package()
