"""Pour a measured amount into a vessel whose weight the tree already knows."""
import py_trees

from cho_interfaces.action import Pour
from cho_task_manager.behaviors.action.base_action_behavior import BaseActionBehavior
from cho_task_manager.utils.blackboard import TASK_NAMESPACE, read_if_set
from cho_task_manager.utils.controller_names import controller_action_name

#: Written the way a tree reads, mapped to the action's constants here. A tree
#: that says `material='granular'` is legible in a review; one that says
#: `material=1` is not.
MATERIALS = {
    'liquid': Pour.Goal.MATERIAL_LIQUID,
    'granular': Pour.Goal.MATERIAL_GRANULAR,
}

#: Longer than the other action behaviours by default. A pour legitimately
#: spends seconds holding still -- the reading has to settle after every stop,
#: which took 1.0 s per stop on the FR5 cell's scale -- and a trim pulse costs
#: another cycle of that. The controller has its own `timeout` for the pour
#: proper; this is only the behaviour giving up on the goal ever finishing.
DEFAULT_TIMEOUT_SEC = 300.0


class PourActionBehavior(BaseActionBehavior):
    """
    Tilt a held vessel until the scale says enough has come out.

    ``container_grams`` is not an optional convenience. The indicator is
    read-only -- it cannot be tared over RS232 -- so this is the pour's only
    zero, and the controller checks the scale against it before the first tilt
    and refuses the goal when they disagree. That check is what catches a vessel
    that is not on the pan, the wrong vessel, and one still holding what an
    earlier pour left in it; on the real cell the same flask read 139.15 g dry
    and 149.02 g with the last pour's water in it.

    Either amount can come off the blackboard instead of being fixed when the
    tree is built, resolved in ``initialise()`` immediately before the goal is
    sent -- so a recipe step can decide how much, and a weighing step can decide
    what the vessel weighs, without the tree being rebuilt.
    """

    def __init__(
        self,
        name: str,
        controller_name: str,
        target_grams: float = None,
        container_grams: float = None,
        material: str = 'liquid',
        flow_index: float = 0.0,
        tolerance: float = 0.0,
        max_tilt: float = 0.0,
        max_tilt_rate: float = 0.0,
        pour_timeout: float = 0.0,
        target_grams_key: str = None,
        container_grams_key: str = None,
        blackboard_namespace: str = TASK_NAMESPACE,
        timeout_sec: float = DEFAULT_TIMEOUT_SEC,
    ):
        super().__init__(
            name, Pour, controller_action_name(controller_name), timeout_sec=timeout_sec)

        if (target_grams is None) == (target_grams_key is None):
            raise ValueError(
                f"[{name}] give exactly one of target_grams (a literal, fixed when the tree "
                'is built) or target_grams_key (read off the blackboard when the goal is sent)')
        if (container_grams is None) == (container_grams_key is None):
            raise ValueError(
                f"[{name}] give exactly one of container_grams or container_grams_key. It is "
                'not optional either way: the scale cannot be tared, so this is the only zero '
                'the pour has, and the controller refuses a goal whose vessel does not match it')
        if material not in MATERIALS:
            raise ValueError(
                f"[{name}] material must be one of {sorted(MATERIALS)}, got {material!r}. The "
                'two do not share a flow law: a liquid can be regulated, a granular medium '
                'arrives in avalanches and can only be metered')
        if not 0.0 <= flow_index <= 1.0:
            raise ValueError(
                f"[{name}] flow_index is a 0..1 position between this material class's two "
                f'configured endpoints, not a physical unit; got {flow_index}')

        self.target_grams = target_grams
        self.container_grams = container_grams
        self.material = material
        self.flow_index = flow_index
        # Every bound left at 0 falls back to the controller's configured value,
        # the same convention GripperActionBehavior uses.
        self.tolerance = tolerance
        self.max_tilt = max_tilt
        self.max_tilt_rate = max_tilt_rate
        self.pour_timeout = pour_timeout
        self.target_grams_key = target_grams_key
        self.container_grams_key = container_grams_key
        self.blackboard_namespace = blackboard_namespace

        self.blackboard = self.attach_blackboard_client(namespace=blackboard_namespace)
        for key in (target_grams_key, container_grams_key):
            if key:
                self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)

    def _resolve(self, literal, key):
        if key is None:
            return literal
        # py_trees raises KeyError for a registered but unwritten key, and
        # getattr() does not absorb it -- that KeyError escapes update() and
        # takes the node down.
        return read_if_set(self.blackboard, key)

    def initialise(self):
        target = self._resolve(self.target_grams, self.target_grams_key)
        container = self._resolve(self.container_grams, self.container_grams_key)

        if target is None or container is None:
            # No goal is sent, so update() reports FAILURE on the next tick --
            # the same shape TaskSpaceActionBehavior uses for an unwritten key.
            missing = self.target_grams_key if target is None else self.container_grams_key
            self.node.get_logger().error(
                f'[{self.name}] blackboard key {missing!r} was never written; nothing is poured '
                'from a number that does not exist')
            return

        goal = Pour.Goal()
        goal.target_grams = float(target)
        goal.container_grams = float(container)
        goal.material = MATERIALS[self.material]
        goal.flow_index = float(self.flow_index)
        goal.tolerance = float(self.tolerance)
        goal.max_tilt = float(self.max_tilt)
        goal.max_tilt_rate = float(self.max_tilt_rate)
        goal.timeout = float(self.pour_timeout)
        self.send_action_goal(goal)
