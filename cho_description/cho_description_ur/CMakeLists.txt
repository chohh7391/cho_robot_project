cmake_minimum_required(VERSION 3.5)
project(cho_description_ur)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes rviz urdf test xml
  DESTINATION share/${PROJECT_NAME}
)

# USD assets for Isaac Sim. Generated (and large), so they are not tracked in
# git -- see cho_description_franka/usd/README.md for the conversion command.
# Guarded so a fresh clone that has not run the converter yet still builds.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/usd")
  install(DIRECTORY
    usd
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(ur_urdf_xacro test/test_ur_urdf_xacro.py)
  ament_add_pytest_test(view_ur_launch test/test_view_ur_launch.py)
endif()

ament_package()
