# Build-time stand-in for isaac_ros_common's version-stamping resources.
#
# WHY THIS EXISTS
# ---------------
# Two of the cuMotion packages reach into isaac_ros_common at build time:
#
#   isaac_ros_cumotion_interfaces/CMakeLists.txt
#       ament_index_get_resource(... isaac_ros_common_cmake_path isaac_ros_common)
#       include(".../isaac_ros_common-version-info.cmake")
#       generate_version_info(${PROJECT_NAME})
#
#   isaac_ros_cumotion_python_utils/setup.py
#       get_resource('isaac_ros_common_scripts_path', 'isaac_ros_common')
#       -> loads isaac_ros_common-version-info.py, uses GenerateVersionInfoCommand
#
# Both are version stamping and nothing else. But upstream isaac_ros_common
# cannot be built here: its CMakeLists has an unconditional
# `find_package(vpi REQUIRED)` for a library (NVIDIA Vision Programming
# Interface) that only its own `vpi_utilities` target uses, that this project
# never uses, and that is not installed on this machine:
#
#   Could not find a package configuration file provided by "vpi"
#
# So this package registers those two resource paths UNDER THE PACKAGE NAME
# `isaac_ros_common` and ships no-op implementations of the two symbols. Nothing
# upstream is patched (todo/CUROBO_MOVEIT_TODO.md D6), and the only thing lost is
# a version string embedded in two packages we do not ship.
#
# THE ALTERNATIVE, if you would rather stay on the supported upstream path:
# install NVIDIA VPI (`libnvvpi3` from the Isaac ROS apt repository), remove
# extern/isaac_ros_common/isaac_ros_common/COLCON_IGNORE, and delete this
# package. Nothing else references it.
cmake_minimum_required(VERSION 3.8)
project(cho_moveit_curobo_deps)

find_package(ament_cmake REQUIRED)

# isaac_ros_cumotion_interfaces resolves the cmake helper through this resource.
ament_index_register_resource(isaac_ros_common_cmake_path
  PACKAGE_NAME isaac_ros_common
  CONTENT "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/cmake"
)
install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})

# isaac_ros_cumotion_python_utils' setup.py resolves the python helper through
# this one, and expects to find the file directly inside the reported directory.
ament_index_register_resource(isaac_ros_common_scripts_path
  PACKAGE_NAME isaac_ros_common
  CONTENT "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/scripts"
)
install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

ament_package()
