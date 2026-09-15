cmake_minimum_required(VERSION 3.8)
project(cho_camera_calibration)

find_package(ament_cmake REQUIRED)

# config/ is where a finished calibration lands, so that it can be handed to a
# driver as package://cho_camera_calibration/config/<file>.yaml rather than a path
# under someone's home directory. It starts out holding nothing but .gitkeep.
install(DIRECTORY targets config scripts
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_flake8()
  ament_lint_cmake()
  # D212 (summary on the first line) and D213 (on the second) are mutually
  # exclusive; ament ignores D212, this repo writes D212. Same choice as
  # cho_object_pose/test/test_pep257.py.
  ament_pep257(--add-ignore D213)
  ament_xmllint()
endif()

ament_package()
