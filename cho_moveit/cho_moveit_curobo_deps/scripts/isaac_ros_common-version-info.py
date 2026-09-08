# No-op stand-in for isaac_ros_common's version-stamping build_py command.
#
# Upstream's GenerateVersionInfoCommand writes a version file during build_py.
# Nothing in this project reads it. See ../CMakeLists.txt for why this exists.
#
# The name and base class must match upstream: isaac_ros_cumotion_python_utils'
# setup.py loads this file by path and uses GenerateVersionInfoCommand as its
# `build_py` cmdclass.

from setuptools.command.build_py import build_py


class GenerateVersionInfoCommand(build_py):
    """Plain build_py: run the normal build with no version stamping."""

    pass
