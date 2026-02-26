cmake_minimum_required(VERSION 3.5)
project(cho_controller_common)

add_compile_options(-Wall -Wextra -Wpedantic -std=c++17 -Ofast)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pinocchio REQUIRED)
find_package(eiquadprog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cho_trajectory_smoother REQUIRED)

file(GLOB_RECURSE PROJECT_HEADERS include/${PROJECT_NAME}/*.hpp include/${PROJECT_NAME}/*.hh)
file(GLOB_RECURSE PROJECT_SOURCES src/*.cpp)

# Source and header files
file(GLOB_RECURSE PROJECT_HEADERS include/${PROJECT_NAME}/*.hpp include/${PROJECT_NAME}/*.hh)
file(GLOB_RECURSE PROJECT_SOURCES src/*.cpp)

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
  cho_trajectory_smoother
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  eiquadprog::eiquadprog
)

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
  pinocchio
  eiquadprog
  Eigen3
  cho_trajectory_smoother
)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
