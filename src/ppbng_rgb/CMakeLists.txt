cmake_minimum_required(VERSION 3.16)
project(ppbng_rgb LANGUAGES CXX)
find_package(ament_cmake REQUIRED)

option(PPBNG_RGB_ENABLE_SPINNAKER "Build the hardware-capable Spinnaker RGB backend" OFF)
set(PPBNG_SPINNAKER_ROOT "C:/Program Files/Teledyne/Spinnaker" CACHE PATH "Spinnaker SDK root")

add_library(${PROJECT_NAME} src/rgb_adapter.cpp src/mock_rgb_backend.cpp src/runtime_contract.cpp src/sequence_association.cpp src/recovery_supervisor.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
if(MSVC)
  target_compile_options(${PROJECT_NAME} PRIVATE /W4 /permissive-)
else()
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

if(PPBNG_RGB_ENABLE_SPINNAKER)
  find_path(PPBNG_SPINNAKER_INCLUDE_DIR Spinnaker.h
    PATHS "${PPBNG_SPINNAKER_ROOT}/include" NO_DEFAULT_PATH REQUIRED)
  find_library(PPBNG_SPINNAKER_LIBRARY_RELEASE Spinnaker_v140
    PATHS "${PPBNG_SPINNAKER_ROOT}/lib64/vs2015" NO_DEFAULT_PATH REQUIRED)
  find_library(PPBNG_SPINNAKER_LIBRARY_DEBUG Spinnakerd_v140
    PATHS "${PPBNG_SPINNAKER_ROOT}/lib64/vs2015" NO_DEFAULT_PATH REQUIRED)
  add_library(Spinnaker::Spinnaker UNKNOWN IMPORTED)
  set_target_properties(Spinnaker::Spinnaker PROPERTIES
    IMPORTED_CONFIGURATIONS "DEBUG;RELEASE"
    IMPORTED_LOCATION_DEBUG "${PPBNG_SPINNAKER_LIBRARY_DEBUG}"
    IMPORTED_LOCATION_RELEASE "${PPBNG_SPINNAKER_LIBRARY_RELEASE}"
    MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE
    MAP_IMPORTED_CONFIG_MINSIZEREL RELEASE
    INTERFACE_INCLUDE_DIRECTORIES "${PPBNG_SPINNAKER_INCLUDE_DIR}")
  add_library(${PROJECT_NAME}_spinnaker SHARED src/spinnaker_rgb_backend.cpp)
  set_target_properties(${PROJECT_NAME}_spinnaker PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
  target_compile_features(${PROJECT_NAME}_spinnaker PUBLIC cxx_std_17)
  target_include_directories(${PROJECT_NAME}_spinnaker PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
  target_link_libraries(${PROJECT_NAME}_spinnaker PUBLIC ${PROJECT_NAME} PRIVATE Spinnaker::Spinnaker)
  target_compile_definitions(${PROJECT_NAME}_spinnaker PUBLIC PPBNG_RGB_HAS_SPINNAKER_BACKEND=1)
  if(MSVC)
    target_compile_options(${PROJECT_NAME}_spinnaker PRIVATE /W4 /permissive-)
  else()
    target_compile_options(${PROJECT_NAME}_spinnaker PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  install(TARGETS ${PROJECT_NAME}_spinnaker EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
else()
  add_library(${PROJECT_NAME}_spinnaker SHARED src/spinnaker_rgb_backend_stub.cpp)
  set_target_properties(${PROJECT_NAME}_spinnaker PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
  target_compile_features(${PROJECT_NAME}_spinnaker PUBLIC cxx_std_17)
  target_include_directories(${PROJECT_NAME}_spinnaker PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
  target_link_libraries(${PROJECT_NAME}_spinnaker PUBLIC ${PROJECT_NAME})
  target_compile_definitions(${PROJECT_NAME}_spinnaker PUBLIC PPBNG_RGB_HAS_SPINNAKER_BACKEND=0)
  install(TARGETS ${PROJECT_NAME}_spinnaker EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
endif()

find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ppbng_interfaces REQUIRED)
find_package(ppbng_storage REQUIRED)
add_executable(ppbng_rgb_camera_node src/rgb_camera_node.cpp)
target_compile_features(ppbng_rgb_camera_node PRIVATE cxx_std_17)
target_link_libraries(ppbng_rgb_camera_node ${PROJECT_NAME}_spinnaker)
ament_target_dependencies(ppbng_rgb_camera_node rclcpp std_srvs ppbng_interfaces ppbng_storage)
install(TARGETS ppbng_rgb_camera_node RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rgb_adapter test/test_rgb_adapter.cpp)
  target_link_libraries(test_rgb_adapter ${PROJECT_NAME})
  if(NOT PPBNG_RGB_ENABLE_SPINNAKER)
    ament_add_gtest(test_spinnaker_rgb_backend_stub test/test_spinnaker_rgb_backend_stub.cpp)
    target_link_libraries(test_spinnaker_rgb_backend_stub ${PROJECT_NAME}_spinnaker)
  endif()
endif()
ament_package()
