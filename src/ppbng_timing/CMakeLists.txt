cmake_minimum_required(VERSION 3.16)
project(ppbng_timing LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ppbng_interfaces REQUIRED)

add_library(${PROJECT_NAME} STATIC
  src/protocol.cpp
  src/controller_model.cpp
  src/host_client.cpp
  src/windows_controller_serial.cpp
  src/production_control.cpp
  src/timing_event_writer.cpp
)
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

add_library(ppbng_timing_firmware STATIC firmware/src/firmware_core.cpp)
target_compile_features(ppbng_timing_firmware PUBLIC cxx_std_17)
target_include_directories(ppbng_timing_firmware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/firmware/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(ppbng_timing_firmware PUBLIC ${PROJECT_NAME})
if(MSVC)
  target_compile_options(ppbng_timing_firmware PRIVATE /W4 /permissive-)
else()
  target_compile_options(ppbng_timing_firmware PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(timing_production_node src/timing_production_node.cpp)
target_compile_features(timing_production_node PRIVATE cxx_std_17)
target_link_libraries(timing_production_node ${PROJECT_NAME})
ament_target_dependencies(timing_production_node rclcpp std_srvs ppbng_interfaces)
install(TARGETS timing_production_node DESTINATION lib/${PROJECT_NAME})

add_executable(uno_r4_ascii_trigger_node src/uno_r4_ascii_trigger_node.cpp)
target_compile_features(uno_r4_ascii_trigger_node PRIVATE cxx_std_17)
target_link_libraries(uno_r4_ascii_trigger_node ${PROJECT_NAME})
ament_target_dependencies(uno_r4_ascii_trigger_node rclcpp std_srvs ppbng_interfaces)
install(TARGETS uno_r4_ascii_trigger_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY firmware/include/ DESTINATION include)
install(DIRECTORY firmware_protocol_docs/ DESTINATION share/${PROJECT_NAME}/firmware_protocol_docs)
install(FILES firmware/README.md DESTINATION share/${PROJECT_NAME}/firmware)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ppbng_timing_firmware
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_timing_protocol test/test_timing_protocol.cpp)
  target_link_libraries(test_timing_protocol ${PROJECT_NAME})
  ament_add_gtest(test_host_client test/test_host_client.cpp)
  target_link_libraries(test_host_client ${PROJECT_NAME})
  ament_add_gtest(test_production_control test/test_production_control.cpp)
  target_link_libraries(test_production_control ${PROJECT_NAME})
  ament_add_gtest(test_firmware_core firmware/test/test_firmware_core.cpp)
  target_link_libraries(test_firmware_core ppbng_timing_firmware ${PROJECT_NAME})
  ament_add_gtest(test_timing_event_writer test/test_timing_event_writer.cpp)
  target_link_libraries(test_timing_event_writer ${PROJECT_NAME})
endif()

ament_package()
