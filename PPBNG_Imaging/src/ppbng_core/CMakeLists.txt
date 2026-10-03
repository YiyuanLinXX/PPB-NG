cmake_minimum_required(VERSION 3.16)
project(ppbng_core LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ppbng_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

add_library(${PROJECT_NAME}
  src/frame_trigger_association.cpp
  src/frame_context_association.cpp
  src/frame_context_writer.cpp
  src/gnss_association.cpp
  src/gga_utc_resolver.cpp
  src/pps_time_mapper.cpp
  src/rsm_attitude_association.cpp
  src/time_authority_correlator.cpp
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

add_executable(time_authority_node src/time_authority_node.cpp)
target_compile_features(time_authority_node PRIVATE cxx_std_17)
target_link_libraries(time_authority_node ${PROJECT_NAME})
ament_target_dependencies(time_authority_node rclcpp ppbng_interfaces)

add_executable(frame_context_node src/frame_context_node.cpp)
target_compile_features(frame_context_node PRIVATE cxx_std_17)
target_link_libraries(frame_context_node ${PROJECT_NAME})
ament_target_dependencies(frame_context_node rclcpp ppbng_interfaces std_srvs)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS time_authority_node frame_context_node DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pps_time_mapper test/test_pps_time_mapper.cpp)
  target_link_libraries(test_pps_time_mapper ${PROJECT_NAME})

  ament_add_gtest(test_gnss_association test/test_gnss_association.cpp)
  target_link_libraries(test_gnss_association ${PROJECT_NAME})

  ament_add_gtest(test_rsm_attitude_association test/test_rsm_attitude_association.cpp)
  target_link_libraries(test_rsm_attitude_association ${PROJECT_NAME})

  ament_add_gtest(test_frame_trigger_association test/test_frame_trigger_association.cpp)
  target_link_libraries(test_frame_trigger_association ${PROJECT_NAME})

  ament_add_gtest(test_time_authority_correlator test/test_time_authority_correlator.cpp)
  target_link_libraries(test_time_authority_correlator ${PROJECT_NAME})

  ament_add_gtest(test_frame_context_association test/test_frame_context_association.cpp)
  target_link_libraries(test_frame_context_association ${PROJECT_NAME})

  ament_add_gtest(test_gga_utc_resolver test/test_gga_utc_resolver.cpp)
  target_link_libraries(test_gga_utc_resolver ${PROJECT_NAME})

  ament_add_gtest(test_frame_context_writer test/test_frame_context_writer.cpp)
  target_link_libraries(test_frame_context_writer ${PROJECT_NAME})
endif()

ament_package()
