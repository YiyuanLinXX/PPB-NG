cmake_minimum_required(VERSION 3.16)
project(ppbng_rsm400 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ppbng_interfaces REQUIRED)
find_package(ppbng_storage REQUIRED)

add_library(${PROJECT_NAME} STATIC
  src/mcp2_protocol.cpp
  src/command_transport.cpp
  src/win32_serial_transport.cpp
  src/production_activation_gate.cpp
  src/readiness_gate.cpp
  src/session_binding.cpp
  src/telemetry_log_writer.cpp
)

add_executable(rsm400_production_node src/rsm400_production_node.cpp)
target_compile_features(rsm400_production_node PRIVATE cxx_std_17)
target_link_libraries(rsm400_production_node ${PROJECT_NAME})
ament_target_dependencies(rsm400_production_node rclcpp std_srvs ppbng_interfaces ppbng_storage)
ament_target_dependencies(${PROJECT_NAME} ppbng_storage)
if(MSVC)
  target_compile_options(rsm400_production_node PRIVATE /W4 /permissive-)
else()
  target_compile_options(rsm400_production_node PRIVATE -Wall -Wextra -Wpedantic)
endif()
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(MSVC)
  target_compile_options(${PROJECT_NAME} PRIVATE /W4 /permissive-)
else()
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS rsm400_production_node DESTINATION lib/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mcp2_protocol test/test_mcp2_protocol.cpp)
  target_link_libraries(test_mcp2_protocol ${PROJECT_NAME})
  ament_add_gtest(test_command_transport test/test_command_transport.cpp)
  target_link_libraries(test_command_transport ${PROJECT_NAME})
  ament_add_gtest(test_production_activation_gate test/test_production_activation_gate.cpp)
  target_link_libraries(test_production_activation_gate ${PROJECT_NAME})
  ament_add_gtest(test_rsm_storage test/test_rsm_storage.cpp)
  target_link_libraries(test_rsm_storage ${PROJECT_NAME})
  ament_add_gtest(test_readiness_gate test/test_readiness_gate.cpp)
  target_link_libraries(test_readiness_gate ${PROJECT_NAME})
endif()

ament_package()
