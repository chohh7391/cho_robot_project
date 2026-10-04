cmake_minimum_required(VERSION 3.8)
project(cho_openarm_mit_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED src/mit_protocol.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} yaml-cpp)
ament_target_dependencies(${PROJECT_NAME} trajectory_msgs)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES DESIGN.md DESTINATION share/${PROJECT_NAME})

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
  ament_add_gtest(test_safety_profile test/test_safety_profile.cpp)
  target_link_libraries(test_safety_profile ${PROJECT_NAME})
  ament_add_gtest(test_arm_consumer test/test_arm_consumer.cpp)
  target_link_libraries(test_arm_consumer ${PROJECT_NAME})
  target_compile_definitions(test_safety_profile PRIVATE
    OPENARM_SAFETY_PROFILE_SOURCE="${OPENARM_DESCRIPTION_SHARE}/config/mit_safety_profiles_v1.yaml")
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(trajectory_msgs yaml-cpp)
ament_package()
