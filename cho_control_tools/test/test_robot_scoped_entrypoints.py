"""Import-boundary tests for robot-workspace-friendly action clients."""

import importlib
import sys
from pathlib import Path

import pytest


@pytest.mark.parametrize('module_name,robot_type,allow_arm', [
    ('fr5.action_client', 'fr5', False),
    ('franka.action_client', 'franka', False),
    ('openarm.action_client', 'openarm', True),
    ('ur5e.action_client', 'ur5e', False),
])
def test_robot_entrypoint_binds_only_its_robot_loader(
        monkeypatch, module_name, robot_type, allow_arm):
    module = importlib.import_module(f'cho_control_tools.clients.{module_name}')
    calls = []

    def fake_run(bound_robot, argv, **kwargs):
        calls.append((bound_robot, argv, kwargs))
        return 0

    monkeypatch.setattr(module, 'run_robot_client', fake_run)
    assert module.main(['--help']) == 0
    assert calls[0][0] == robot_type
    if allow_arm:
        assert calls[0][2]['allow_arm'] is True
    else:
        assert 'allow_arm' not in calls[0][2]

    loader = calls[0][2]['robot_config_loader']
    with pytest.raises(ValueError, match=f'fixed to {robot_type}'):
        loader('some_other_robot')


def test_robot_entrypoint_import_does_not_import_common_ros_shell_or_registry(monkeypatch):
    # A source checkout normally has both packages importable. Remove cached
    # modules so this checks import-time dependencies rather than test order.
    for name in list(sys.modules):
        if name.startswith('cho_control_tools.clients.fr5'):
            sys.modules.pop(name)
    sys.modules.pop('cho_robot_config', None)
    sys.modules.pop('cho_control_tools.clients.action_client', None)

    imported = importlib.import_module('cho_control_tools.clients.fr5.action_client')
    assert imported.main is not None
    assert 'cho_robot_config' not in sys.modules
    assert 'cho_control_tools.clients.action_client' not in sys.modules


def test_fr5_metadata_loader_runs_without_registry_or_other_robot_metadata(monkeypatch):
    metadata_modules = [
        'cho_robot_config',
        'cho_robot_config.registry',
        'cho_control_tools.clients.fr5.metadata',
        'cho_control_tools.clients.franka.metadata',
        'cho_control_tools.clients.openarm.metadata',
        'cho_control_tools.clients.ur5e.metadata',
    ]
    for name in metadata_modules:
        sys.modules.pop(name, None)
    # A non-module sentinel makes an accidental central-registry import fail
    # immediately instead of silently resolving from this source workspace.
    monkeypatch.setitem(sys.modules, 'cho_robot_config', None)

    from cho_control_tools.clients._robot_metadata import config_loader_for

    config = config_loader_for('fr5')('fr5')
    assert config['robot_type'] == 'fr5'
    assert config['poses']['home']['1'][2] == -1.5707963268
    assert 'cho_control_tools.clients.fr5.metadata' in sys.modules
    assert 'cho_control_tools.clients.franka.metadata' not in sys.modules
    assert 'cho_control_tools.clients.openarm.metadata' not in sys.modules
    assert 'cho_control_tools.clients.ur5e.metadata' not in sys.modules


def test_injected_shell_loaders_do_not_fall_back_to_registry(monkeypatch):
    from cho_control_tools.clients import action_client

    monkeypatch.setitem(sys.modules, 'cho_robot_config', None)
    shell = object.__new__(action_client.ControlSuiteShell)
    shell.robot_type = 'fr5'
    shell.arm = 'single'
    shell._robot_config_loader = lambda robot, arm: {
        'robot_type': robot, 'arm': arm, 'actions': {'preferences': {}}}
    shell._home_pose_policy_loader = lambda _config, _selector: {
        'enabled': True, 'reason': 'local'}

    assert shell._config()['robot_type'] == 'fr5'
    assert shell._home_pose_policy_loader({}, '0')['reason'] == 'local'


@pytest.mark.parametrize('robot_type,profile', [
    ('fr5', 'single'), ('franka', 'single'), ('openarm', 'single'),
    ('openarm', 'left'), ('openarm', 'right'), ('openarm', 'both'),
    ('ur5e', 'single'),
])
def test_bundled_metadata_tracks_the_registry_action_client_contract(robot_type, profile):
    """Detect drift when the full project registry is available in CI.

    The production operator entrypoints do not import this registry. A
    robot-only workspace can therefore omit it; this test is an optional
    source-tree consistency gate rather than a runtime dependency.
    """
    try:
        from cho_robot_config import load_robot_config
    except ImportError:
        pytest.skip('central registry is not installed in this robot workspace')

    from cho_control_tools.clients._robot_metadata import config_loader_for

    bundled = config_loader_for(robot_type)(robot_type, profile)
    canonical = load_robot_config(robot_type, profile)
    assert bundled['supports_task'] == canonical['supports_task']
    assert bundled['actions'] == canonical['actions']
    assert bundled['poses'] == {
        key: canonical['poses'][key]
        for key in ('home', 'reach', 'home_safety') if key in canonical['poses']
    }
    assert bundled['motions'] == canonical['motions']
    assert bundled['controllers']['moveit_trajectory'] == (
        canonical['controllers']['moveit_trajectory'])
    assert bundled['moveit'].get('controllers', [
        bundled['controllers']['moveit_trajectory']]) == canonical['moveit'].get(
            'controllers', [canonical['controllers']['moveit_trajectory']])


def test_console_scripts_point_to_robot_scoped_modules():
    setup = (importlib.import_module('pathlib').Path(__file__).parents[1] / 'setup.py').read_text()
    for robot_type in ('fr5', 'franka', 'openarm', 'ur5e'):
        assert (
            f'{robot_type}_action_client = '
            f'cho_control_tools.clients.{robot_type}.action_client:main' in setup)


def test_robot_clients_have_no_flat_compatibility_modules():
    clients_dir = Path(__file__).parents[1] / 'cho_control_tools' / 'clients'
    assert not (clients_dir / 'robot_action_client.py').exists()
    for robot_type in ('fr5', 'franka', 'openarm', 'ur5e'):
        assert (clients_dir / robot_type / 'action_client.py').is_file()
        assert (clients_dir / robot_type / 'metadata.py').is_file()
        assert not (clients_dir / f'{robot_type}_action_client.py').exists()
        assert not (clients_dir / f'{robot_type}_metadata.py').exists()
