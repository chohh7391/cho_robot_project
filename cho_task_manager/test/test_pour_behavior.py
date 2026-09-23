"""
What a pour goal a tree builds must and must not be allowed to say.

The control law is C++ and is tested against a synthetic vessel in
cho_controller_fr5. What is checked here is the part that fails silently on the
tree side: a goal assembled from the wrong pieces is accepted by the action
server's type but is refused, or worse honoured, by the controller -- and by
then a vessel is in the gripper.
"""

import py_trees
import pytest

from cho_interfaces.action import Pour

from cho_task_manager.behaviors.action.pour import MATERIALS, PourActionBehavior
from cho_task_manager.utils.blackboard import TASK_NAMESPACE

CONTROLLER = 'pouring_controller'


def behaviour(**kwargs):
    kwargs.setdefault('target_grams', 50.0)
    kwargs.setdefault('container_grams', 139.15)
    return PourActionBehavior('pour', CONTROLLER, **kwargs)


def test_it_targets_the_controllers_own_action_endpoint():
    assert behaviour().action_name == f'/controller_action_server/{CONTROLLER}'


def test_material_names_map_to_the_actions_constants():
    # The tree says 'granular' because that is legible in a review; the wire
    # carries a number. If these ever drift, a granular pour silently runs a
    # liquid's profile.
    assert MATERIALS['liquid'] == Pour.Goal.MATERIAL_LIQUID
    assert MATERIALS['granular'] == Pour.Goal.MATERIAL_GRANULAR


def test_the_container_weight_is_not_optional():
    # The indicator cannot be tared over RS232, so this is the pour's only zero.
    # Letting it default would turn the controller's pre-tilt check -- the thing
    # that catches a missing vessel, the wrong vessel, and residue from the last
    # pour -- into a coin toss.
    with pytest.raises(ValueError) as excinfo:
        PourActionBehavior('pour', CONTROLLER, target_grams=50.0)
    assert 'container_grams' in str(excinfo.value)


def test_exactly_one_form_of_each_amount():
    with pytest.raises(ValueError):
        behaviour(target_grams_key='grams')
    with pytest.raises(ValueError):
        behaviour(container_grams_key='vessel')
    with pytest.raises(ValueError):
        PourActionBehavior('pour', CONTROLLER, container_grams=139.15)


def test_an_unknown_material_is_refused_at_tree_build_time():
    with pytest.raises(ValueError) as excinfo:
        behaviour(material='powder')
    assert 'granular' in str(excinfo.value)


@pytest.mark.parametrize('flow_index', [-0.1, 1.1, 1000.0])
def test_a_flow_index_outside_its_range_is_refused(flow_index):
    # Out of range almost always means a physical quantity was passed --
    # centipoise, a percentage -- and silently clamping it to 1.0 would pour
    # honey on water's profile.
    with pytest.raises(ValueError) as excinfo:
        behaviour(flow_index=flow_index)
    assert 'flow_index' in str(excinfo.value)


@pytest.mark.parametrize('flow_index', [0.0, 0.5, 1.0])
def test_the_whole_declared_range_is_accepted(flow_index):
    assert behaviour(flow_index=flow_index).flow_index == flow_index


def test_bounds_left_at_zero_stay_zero_for_the_controller_to_fill():
    # Same convention GripperActionBehavior uses: 0 means "use the controller's
    # configured value", and the behaviour must not invent one.
    pour = behaviour()
    assert pour.tolerance == 0.0
    assert pour.max_tilt == 0.0
    assert pour.max_tilt_rate == 0.0
    assert pour.pour_timeout == 0.0


def test_the_behaviour_timeout_outlasts_a_pour_that_settles_between_pulses():
    # A pour legitimately holds still for seconds at a time: the reading has to
    # settle after every stop, and every trim pulse costs another of those. A
    # 30 s default would abandon a perfectly healthy granular pour.
    assert behaviour().timeout_sec >= 120.0


def test_blackboard_keys_are_registered_for_reading():
    pour = behaviour(target_grams=None, target_grams_key='dose_grams',
                     container_grams=None, container_grams_key='vessel_grams')
    keys = {key.split('/')[-1] for key in pour.blackboard.remappings.values()}
    assert {'dose_grams', 'vessel_grams'} <= keys
    assert pour.blackboard_namespace == TASK_NAMESPACE


def test_an_unwritten_key_sends_no_goal_and_fails():
    py_trees.blackboard.Blackboard.clear()
    pour = behaviour(target_grams=None, target_grams_key='dose_grams')
    # No setup(), so there is no node; the point is that initialise() must not
    # reach send_action_goal() with a key nobody wrote. A tree that poured from
    # a missing number would pour an arbitrary amount.
    pour.node = _SilentNode()
    pour.initialise()
    assert pour.send_goal_future is None
    assert pour.update() == py_trees.common.Status.FAILURE


class _SilentNode:
    """Just enough node for a behaviour that is about to log and give up."""

    def get_logger(self):
        return self

    def error(self, _message):
        pass

    def warn(self, _message):
        pass

    def info(self, _message):
        pass


def test_a_finished_run_leaves_nothing_for_the_next_one_to_read():
    # The next run must not see this run's goal. BaseActionBehavior.terminate()
    # is what clears it, and py_trees calls terminate() on every transition to
    # SUCCESS or FAILURE -- so a re-entered pour whose key has since gone
    # unwritten fails, instead of reporting the previous pour as its own. This
    # pins that: a subclass overriding terminate() without super() breaks it.
    pour = behaviour()
    pour.send_goal_future = object()
    pour.get_result_future = object()
    pour.goal_handle = object()
    pour.stop(py_trees.common.Status.SUCCESS)
    assert pour.send_goal_future is None
    assert pour.get_result_future is None
    assert pour.goal_handle is None
