cmake_minimum_required(VERSION 3.16)
project(ppbng_hsi LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ppbng_interfaces REQUIRED)
find_package(ppbng_storage REQUIRED)

option(PPBNG_HSI_ENABLE_SPECSENSOR
  "Compile and link the hardware SpecSensor backend (never enabled by default)" OFF)
set(PPBNG_SPECSENSOR_ROOT
  "C:/Program Files (x86)/Specim/SDKs/SpecSensor/2020_519"
  CACHE PATH "Installed SpecSensor SDK root")

add_library(${PROJECT_NAME}
  src/hsi_format.cpp
  src/mock_hsi_adapter.cpp
  src/dual_hsi_coordinator.cpp
  src/specsensor_adapter.cpp
  src/production_activation_gate.cpp
  src/envi_segment_writer.cpp
  src/envi_part_verifier.cpp
  src/dataset_context_verifier.cpp
  src/frame_evidence_verifier.cpp
  src/session_binding.cpp
  src/pending_trigger_matcher.cpp
  src/sample_stamp_mapper.cpp
)
ament_target_dependencies(${PROJECT_NAME} ppbng_storage ppbng_interfaces)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(${PROJECT_NAME} PRIVATE
  PPBNG_HSI_ENABLE_SPECSENSOR=$<BOOL:${PPBNG_HSI_ENABLE_SPECSENSOR}>)

if(PPBNG_HSI_ENABLE_SPECSENSOR)
  if(NOT WIN32)
    message(FATAL_ERROR "The installed SpecSensor backend is Windows-only")
  endif()
  set(_specsensor_include "${PPBNG_SPECSENSOR_ROOT}/include")
  set(_specsensor_library "${PPBNG_SPECSENSOR_ROOT}/bin/x64/SpecSensor.lib")
  if(NOT EXISTS "${_specsensor_include}/SI_sensor.h")
    message(FATAL_ERROR "SpecSensor headers not found under ${_specsensor_include}")
  endif()
  if(NOT EXISTS "${_specsensor_library}")
    message(FATAL_ERROR "SpecSensor x64 import library not found: ${_specsensor_library}")
  endif()
  target_include_directories(${PROJECT_NAME} PRIVATE "${_specsensor_include}")
  target_link_libraries(${PROJECT_NAME} "${_specsensor_library}")
endif()

add_library(hsi_production_node_impl STATIC src/hsi_production_node.cpp)
target_compile_features(hsi_production_node_impl PRIVATE cxx_std_17)
target_link_libraries(hsi_production_node_impl ${PROJECT_NAME})
ament_target_dependencies(hsi_production_node_impl rclcpp std_srvs ppbng_interfaces)

add_executable(hsi_production_node src/hsi_production_node_main.cpp)
target_compile_features(hsi_production_node PRIVATE cxx_std_17)
target_link_libraries(hsi_production_node hsi_production_node_impl)
ament_target_dependencies(hsi_production_node rclcpp std_srvs ppbng_interfaces)

add_executable(ppbng_verify_dataset tools/verify_hsi_dataset_main.cpp)
target_compile_features(ppbng_verify_dataset PRIVATE cxx_std_17)
target_link_libraries(ppbng_verify_dataset ${PROJECT_NAME})
if(MSVC)
  target_compile_options(hsi_production_node_impl PRIVATE /W4 /permissive-)
  target_compile_options(hsi_production_node PRIVATE /W4 /permissive-)
else()
  target_compile_options(hsi_production_node_impl PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(hsi_production_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  target_compile_options(${PROJECT_NAME} PRIVATE /W4 /permissive-)
else()
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README_SPECSENSOR.md DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS hsi_production_node ppbng_verify_dataset
  DESTINATION lib/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(ppbng_storage ppbng_interfaces)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_hsi_contracts test/test_hsi_contracts.cpp)
  target_link_libraries(test_hsi_contracts ${PROJECT_NAME})
  target_compile_definitions(test_hsi_contracts PRIVATE
    PPBNG_HSI_ENABLE_SPECSENSOR=$<BOOL:${PPBNG_HSI_ENABLE_SPECSENSOR}>)
  ament_add_gtest(test_envi_segment_writer test/test_envi_segment_writer.cpp)
  target_link_libraries(test_envi_segment_writer ${PROJECT_NAME})
  ament_add_gtest(test_production_coordination test/test_production_coordination.cpp)
  target_link_libraries(test_production_coordination ${PROJECT_NAME})
  ament_add_gtest(test_fault_policy test/test_fault_policy.cpp)
  target_link_libraries(test_fault_policy ${PROJECT_NAME})

  ament_add_gtest(test_bounded_recovery test/test_bounded_recovery.cpp)
  target_link_libraries(test_bounded_recovery ${PROJECT_NAME})
  ament_add_gtest(test_sample_stamp_mapper test/test_sample_stamp_mapper.cpp)
  target_link_libraries(test_sample_stamp_mapper ${PROJECT_NAME})
  ament_target_dependencies(test_sample_stamp_mapper ppbng_interfaces)
  ament_add_gtest(test_dataset_context_verifier test/test_dataset_context_verifier.cpp)
  target_link_libraries(test_dataset_context_verifier ${PROJECT_NAME})
endif()

ament_package()
