cmake_minimum_required(VERSION 3.16)
project(ppbng_gnss LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ppbng_interfaces REQUIRED)
find_package(ppbng_storage REQUIRED)

add_library(${PROJECT_NAME} STATIC
  src/um982_protocol.cpp
  src/um982_receiver.cpp
  src/production_activation_gate.cpp
  src/gnss_log_writer.cpp)
target_sources(${PROJECT_NAME} PRIVATE src/session_binding.cpp)
ament_target_dependencies(${PROJECT_NAME} ppbng_storage)
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

add_executable(um982_production_node src/um982_production_node.cpp)
target_compile_features(um982_production_node PRIVATE cxx_std_17)
target_link_libraries(um982_production_node ${PROJECT_NAME})
ament_target_dependencies(um982_production_node rclcpp std_msgs std_srvs ppbng_interfaces)
if(MSVC)
  target_compile_options(um982_production_node PRIVATE /W4 /permissive-)
else()
  target_compile_options(um982_production_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(DIRECTORY include/ DESTINATION include)
install(FILES README_PRODUCTION_NODE.md DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS um982_production_node DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_um982_protocol test/test_um982_protocol.cpp)
  target_link_libraries(test_um982_protocol ${PROJECT_NAME})
  ament_add_gtest(test_um982_receiver test/test_um982_receiver.cpp)
  target_link_libraries(test_um982_receiver ${PROJECT_NAME})
  ament_add_gtest(test_production_activation_gate test/test_production_activation_gate.cpp)
  target_link_libraries(test_production_activation_gate ${PROJECT_NAME})
  ament_add_gtest(test_gnss_log_writer test/test_gnss_log_writer.cpp)
  target_link_libraries(test_gnss_log_writer ${PROJECT_NAME})
  ament_add_gtest(test_session_binding test/test_session_binding.cpp)
  target_link_libraries(test_session_binding ${PROJECT_NAME})
  ament_add_gtest(test_fault_policy test/test_fault_policy.cpp)
  target_link_libraries(test_fault_policy ${PROJECT_NAME})

  ament_add_gtest(test_bounded_recovery test/test_bounded_recovery.cpp)
  target_link_libraries(test_bounded_recovery ${PROJECT_NAME})
endif()

ament_package()
