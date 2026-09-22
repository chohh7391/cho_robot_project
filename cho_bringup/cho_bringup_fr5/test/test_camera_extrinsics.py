"""Unit tests for the measured camera extrinsics.

The extrinsic is the one number in the perception stack that nothing at
runtime checks: a wrong one produces a confident, wrong object pose. A typo
cannot be caught later, so it is caught here.
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_LAUNCH = os.path.join(os.path.dirname(_HERE), 'launch', 'camera_extrinsics.launch.py')

_spec = importlib.util.spec_from_file_location('camera_extrinsics_launch', _LAUNCH)
_module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_module)
load_transforms = _module.load_transforms


def _shipped():
    return os.path.join(os.path.dirname(_HERE), 'config', 'real', 'camera_extrinsics.yaml')


def _write(tmp_path, entry):
    path = tmp_path / 'extrinsics.yaml'
    import yaml
    path.write_text(yaml.safe_dump({'transforms': [entry]}), encoding='utf-8')
    return str(path)


def _entry(**overrides):
    entry = {'name': 'side_1', 'parent_frame': 'base_link', 'child_frame': 'side_1_mount',
             'xyz': [0.1, 0.2, 0.3], 'quaternion': [0.0, 0.0, 0.0, 1.0]}
    entry.update(overrides)
    return entry


# ------------------------------------------------------------- the real file

def test_the_bench_ships_all_three_cameras():
    transforms = load_transforms(_shipped())
    assert {entry['name'] for entry in transforms} == {'wrist', 'side_1', 'side_2'}


def test_the_wrist_hangs_off_the_arm_and_the_side_camera_off_the_base():
    by_name = {entry['name']: entry for entry in load_transforms(_shipped())}
    # Eye-in-hand vs eye-to-hand, and the difference is not cosmetic: the wrist
    # entry needs the robot's TF to resolve and the side camera's does not.
    assert by_name['wrist']['parent_frame'] == 'wrist3_link'
    assert by_name['side_1']['parent_frame'] == 'base_link'
    assert by_name['side_2']['parent_frame'] == 'base_link'


def test_every_child_is_a_driver_root_not_an_optical_frame():
    # Both drivers publish their own internal chain. Parenting anything below
    # the root would give one frame two parents -- no error, just transforms
    # that resolve through whichever arrived last.
    for entry in load_transforms(_shipped()):
        assert 'optical' not in entry['child_frame'], entry['name']


def test_the_installed_copy_is_the_one_that_was_checked():
    # config/ is installed wholesale, so a file added to the source tree and
    # not to the install is a stale extrinsic that still launches.
    installed = os.path.join(get_package_share_directory('cho_bringup_fr5'),
                             'config', 'real', 'camera_extrinsics.yaml')
    assert load_transforms(installed) == load_transforms(_shipped())


# ------------------------------------------------------------- the validation

def test_a_quaternion_that_is_not_unit_length_is_rejected(tmp_path):
    # It scales as well as rotates, which reads downstream as a camera with the
    # wrong intrinsics rather than as a bad transform.
    with pytest.raises(ValueError, match='not 1'):
        load_transforms(_write(tmp_path, _entry(quaternion=[0.0, 0.0, 0.0, 0.9])))


def test_w_first_order_is_caught_when_it_is_not_unit(tmp_path):
    with pytest.raises(ValueError, match='x, y, z, w'):
        load_transforms(_write(tmp_path, _entry(quaternion=[1.0, 0.5, 0.0, 0.0])))


def test_a_frame_parented_to_itself_is_rejected(tmp_path):
    with pytest.raises(ValueError, match='itself'):
        load_transforms(_write(tmp_path, _entry(child_frame='base_link')))


def test_two_transforms_into_one_child_frame_are_rejected(tmp_path):
    import yaml
    path = tmp_path / 'extrinsics.yaml'
    path.write_text(yaml.safe_dump({'transforms': [
        _entry(name='a'), _entry(name='b', parent_frame='wrist3_link')]}),
        encoding='utf-8')
    with pytest.raises(ValueError, match='same child frame'):
        load_transforms(str(path))


def test_a_short_vector_is_rejected(tmp_path):
    for key, value in (('xyz', [0.1, 0.2]), ('quaternion', [0.0, 0.0, 1.0])):
        with pytest.raises(ValueError, match=key):
            load_transforms(_write(tmp_path, _entry(**{key: value})))


def test_an_empty_table_is_rejected(tmp_path):
    import yaml
    path = tmp_path / 'extrinsics.yaml'
    path.write_text(yaml.safe_dump({'transforms': []}), encoding='utf-8')
    with pytest.raises(ValueError, match='non-empty'):
        load_transforms(str(path))
