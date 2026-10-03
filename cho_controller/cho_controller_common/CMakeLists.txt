cmake_minimum_required(VERSION 3.5)
project(cho_controller_common)

add_compile_options(-Wall -Wextra -Wpedantic -std=c++17 -Ofast)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pinocchio REQUIRED)
find_package(eiquadprog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ruckig REQUIRED)
# Used only by the header-only trajectory/motion_limits_params.hpp, which the
# controller packages compile; exported below so they find it.
find_package(joint_limits REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

file(GLOB_RECURSE PROJECT_HEADERS include/${PROJECT_NAME}/*.hpp include/${PROJECT_NAME}/*.hh)
file(GLOB_RECURSE PROJECT_SOURCES src/*.cpp)

# Ruckig's templates rely on isnan()/isinf(), which -Ofast folds away; the
# one file that instantiates them opts out. -fno-finite-math-only beats -Ofast
# whatever the order (measured for cho_vla_core), and test_point_to_point fails
# if this line goes.
set_source_files_properties(src/trajectory/point_to_point.cpp
  PROPERTIES COMPILE_OPTIONS -fno-finite-math-only)

# Build shared library
add_library(${PROJECT_NAME} SHARED
  ${PROJECT_SOURCES}
  ${PROJECT_HEADERS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pinocchio
  eiquadprog
  Eigen3
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  eiquadprog::eiquadprog
  # pinocchio 3+/4.0: link the modern imported target explicitly.
  # ament_target_dependencies(pinocchio) does not propagate the target's
  # interface properties, so we would otherwise miss:
  #  - BOOST_MPL_LIMIT_LIST_SIZE=30 compile defs (JointCollectionDefault now
  #    has >20 boost::variant types), and
  #  - the split-out pinocchio_parsers lib holding the URDF parser symbols
  #    (pinocchio::urdf::buildModel), which caused an undefined-symbol crash
  #    at runtime.
  # pinocchio::pinocchio bundles default + parsers + collision (and headers).
  pinocchio::pinocchio
)
# No installed header includes Ruckig, but this file links with the plain
# signature, which makes every link transitive: ruckig is exported below so a
# consumer's find_package(cho_controller_common) defines ruckig::ruckig.
target_link_libraries(${PROJECT_NAME} ruckig::ruckig)

# Install
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  EXPORT export_${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  ruckig
  joint_limits
  pinocchio
  eiquadprog
  Eigen3
)
ament_export_targets(export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_point_to_point test/test_point_to_point.cpp)
  set_source_files_properties(test/test_point_to_point.cpp PROPERTIES COMPILE_OPTIONS -fno-finite-math-only)
  # The private planner header lives in src/.
  target_include_directories(test_point_to_point PRIVATE src)
  target_link_libraries(test_point_to_point ${PROJECT_NAME} pinocchio::pinocchio)
endif()

ament_package()
