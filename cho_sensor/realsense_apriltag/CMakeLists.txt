cmake_minimum_required(VERSION 3.8)
project(realsense_apriltag)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_flake8()
  ament_lint_cmake()
  # D213 wants a multi-line docstring's summary on the second line. This
  # repository puts it on the first (that is D212, which ament's own
  # convention ignores), so the two rules are mutually exclusive and this
  # picks the side the rest of the code is on. Same call as
  # cho_object_pose/test/test_pep257.py makes.
  ament_pep257(--add-ignore D213)
  ament_xmllint()
endif()

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()
