cmake_minimum_required(VERSION 3.8)
project(cho_hardware_openarm_mit_real)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(cho_openarm_mit_core REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(OpenArmCAN QUIET)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

# `extern/openarm_can` is upstream CMake, not an ament package.  Keeping this
# package configurable without its installed export lets simulation-only CI
# build the rest of the workspace; setting this ON turns a missing real vendor
# dependency into a clear configure failure.
option(CHO_OPENARM_MIT_REAL_REQUIRE_VENDOR "Require the installed OpenArmCAN vendor package" OFF)
if(NOT OpenArmCAN_FOUND)
  if(CHO_OPENARM_MIT_REAL_REQUIRE_VENDOR)
    message(FATAL_ERROR "OpenArmCAN was not found. Build/install extern/openarm_can first; see cho_hardware_openarm_mit_real/README.md.")
  endif()
  message(WARNING "OpenArmCAN is unavailable: cho_hardware_openarm_mit_real plugin is not built.")
  ament_package()
  return()
endif()

add_library(${PROJECT_NAME} SHARED src/openarm_mit_real_system.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(${PROJECT_NAME} OpenArmCAN::openarm_can)
ament_target_dependencies(${PROJECT_NAME}
  cho_openarm_mit_core hardware_interface pluginlib rclcpp rclcpp_lifecycle)
pluginlib_export_plugin_description_file(hardware_interface hardware.xml)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES hardware.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_openarm_mit_real_gates test/test_openarm_mit_real_gates.cpp)
  target_link_libraries(test_openarm_mit_real_gates ${PROJECT_NAME})
  target_include_directories(test_openarm_mit_real_gates PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_openarm_mit_real_gates hardware_interface)
  target_compile_definitions(test_openarm_mit_real_gates PRIVATE
    OPENARM_SAFETY_PROFILE_SOURCE="${CMAKE_CURRENT_SOURCE_DIR}/../../cho_description/cho_description_openarm/config/mit_safety_profiles_v1.yaml")
endif()

ament_export_dependencies(cho_openarm_mit_core hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
