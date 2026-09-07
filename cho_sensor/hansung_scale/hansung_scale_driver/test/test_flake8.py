"""ament_flake8 over the package sources."""
import os

import pytest

from ament_flake8.main import main_with_errors

# Paths are resolved from this file rather than from the working directory:
# `colcon test` runs pytest with the build directory as cwd, where a
# --symlink-install tree only partly mirrors the sources.
PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TARGETS = [os.path.join(PACKAGE_ROOT, name)
           for name in ('hansung_scale_driver', 'launch', 'test', 'setup.py')]


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=TARGETS)
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + '\n'.join(errors)
