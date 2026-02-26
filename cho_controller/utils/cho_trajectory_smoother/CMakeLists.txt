cmake_minimum_required(VERSION 3.8)
project(cho_trajectory_smoother)

# Build settings
add_compile_options(-Wall -Wextra -Wpedantic -std=c++17 -O3)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(eigenpy REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Sources and headers
set(SOURCES
  src/Path.cpp
  src/Trajectory.cpp
)

set(HEADERS
  include/cho_trajectory_smoother/Path.h
  include/cho_trajectory_smoother/Trajectory.h
)

# Build the shared library
add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} PUBLIC)
ament_target_dependencies(${PROJECT_NAME}
  Eigen3
  Boost
  eigenpy
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Export package
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(Eigen3 Boost eigenpy)

ament_package()
