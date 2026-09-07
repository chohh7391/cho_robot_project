"""ament_pep257 docstring conventions over the package sources."""
import os

import pytest

from ament_pep257.main import main

# Resolved from this file rather than from the working directory: `colcon
# test` runs pytest with the build directory as cwd, where a
# --symlink-install tree only partly mirrors the sources.
PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TARGETS = [os.path.join(PACKAGE_ROOT, name)
           for name in ('hansung_scale_driver', 'launch', 'test', 'setup.py')]


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    assert os.path.isdir(TARGETS[0]), f'package sources not found at {TARGETS[0]}'
    rc = main(argv=list(TARGETS))
    assert rc == 0, 'Found code style errors / warnings'
