cmake_minimum_required(VERSION 3.16)
project(ppbng_sim LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(ppbng_core REQUIRED)
find_package(ppbng_orchestrator REQUIRED)
find_package(ppbng_timing REQUIRED)
find_package(ppbng_hsi REQUIRED)
find_package(ppbng_rgb REQUIRED)
find_package(ppbng_thermal REQUIRED)
find_package(ppbng_storage REQUIRED)

add_library(${PROJECT_NAME} src/integration_scenario.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  ppbng_core ppbng_orchestrator ppbng_timing ppbng_hsi
  ppbng_rgb ppbng_thermal ppbng_storage)

add_executable(ppbng_sim_soak src/soak_main.cpp)
target_compile_features(ppbng_sim_soak PRIVATE cxx_std_17)
target_link_libraries(ppbng_sim_soak ${PROJECT_NAME})
ament_target_dependencies(ppbng_sim_soak
  ppbng_core ppbng_orchestrator ppbng_timing ppbng_hsi
  ppbng_rgb ppbng_thermal ppbng_storage)

if(MSVC)
  target_compile_options(${PROJECT_NAME} PRIVATE /W4 /permissive-)
else()
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(DIRECTORY include/ DESTINATION include)
install(FILES CONTRACT_GAPS.md DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} ppbng_sim_soak
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  ppbng_core ppbng_orchestrator ppbng_timing ppbng_hsi
  ppbng_rgb ppbng_thermal ppbng_storage)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_integration_scenario test/test_integration_scenario.cpp)
  target_link_libraries(test_integration_scenario ${PROJECT_NAME})
  ament_target_dependencies(test_integration_scenario
    ppbng_core ppbng_orchestrator ppbng_timing ppbng_hsi
    ppbng_rgb ppbng_thermal ppbng_storage)
endif()

ament_package()
