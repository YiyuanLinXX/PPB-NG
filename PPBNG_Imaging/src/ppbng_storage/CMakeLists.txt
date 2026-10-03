cmake_minimum_required(VERSION 3.16)
project(ppbng_storage LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME}
  src/association_verifier.cpp
  src/dataset_session.cpp
  src/durable_write_qualification.cpp
  src/json_verifier.cpp
  src/manifest_verifier.cpp
  src/segment_io.cpp
  src/segment_verifier.cpp
  src/session_manifest.cpp
  src/thermal_payload_layout.cpp
)

if(WIN32)
  add_executable(ppbng_storage_durable_write_qualification
    tools/durable_write_qualification_main.cpp)
  target_compile_features(ppbng_storage_durable_write_qualification PRIVATE cxx_std_17)
  target_link_libraries(ppbng_storage_durable_write_qualification ${PROJECT_NAME})
  if(MSVC)
    target_compile_options(ppbng_storage_durable_write_qualification PRIVATE /W4 /permissive-)
  endif()
  install(TARGETS ppbng_storage_durable_write_qualification DESTINATION lib/${PROJECT_NAME})
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
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_thermal_payload_layout test/test_thermal_payload_layout.cpp)
  target_link_libraries(test_thermal_payload_layout ${PROJECT_NAME})

  ament_add_gtest(test_dataset_session test/test_dataset_session.cpp)
  target_link_libraries(test_dataset_session ${PROJECT_NAME})

  ament_add_gtest(test_segment_io test/test_segment_io.cpp)
  target_link_libraries(test_segment_io ${PROJECT_NAME})

  ament_add_gtest(test_segment_verifier test/test_segment_verifier.cpp)
  target_link_libraries(test_segment_verifier ${PROJECT_NAME})

  ament_add_gtest(test_json_verifier test/test_json_verifier.cpp)
  target_link_libraries(test_json_verifier ${PROJECT_NAME})

  ament_add_gtest(test_manifest_verifier test/test_manifest_verifier.cpp)
  target_link_libraries(test_manifest_verifier ${PROJECT_NAME})

  ament_add_gtest(test_association_verifier test/test_association_verifier.cpp)
  target_link_libraries(test_association_verifier ${PROJECT_NAME})

  ament_add_gtest(test_session_manifest test/test_session_manifest.cpp)
  target_link_libraries(test_session_manifest ${PROJECT_NAME})

  ament_add_gtest(test_durable_write_qualification test/test_durable_write_qualification.cpp)
  target_link_libraries(test_durable_write_qualification ${PROJECT_NAME})
endif()

ament_package()
