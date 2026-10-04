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

# `extern/openarm_can` is an upstream plain-CMake package (build_type cmake);
# package.xml declares it so colcon orders the two. It stays QUIET here so a
# workspace without the submodule (simulation-only CI, which does not build this
# package) still configures; setting this ON turns a missing vendor package into
# a clear configure failure instead of a skipped plugin.
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
  # Build-file and manifest lint only: the C++ predates any formatter here.
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_lint_cmake()
  ament_xmllint()
  find_package(cho_description_openarm REQUIRED)
  # The description's installed share directory (symlinked to the source
  # under --symlink-install), not a path back into the source tree.
  set(OPENARM_DESCRIPTION_SHARE "${cho_description_openarm_DIR}/..")
  ament_add_gtest(test_openarm_mit_real_gates test/test_openarm_mit_real_gates.cpp)
  target_link_libraries(test_openarm_mit_real_gates ${PROJECT_NAME})
  target_include_directories(test_openarm_mit_real_gates PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_openarm_mit_real_gates hardware_interface)
  target_compile_definitions(test_openarm_mit_real_gates PRIVATE
    OPENARM_SAFETY_PROFILE_SOURCE="${OPENARM_DESCRIPTION_SHARE}/config/mit_safety_profiles_v1.yaml")
endif()

ament_export_dependencies(cho_openarm_mit_core hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
