"""Blackboard conventions for passing motion targets between behaviours.

A task tree's motion targets used to be baked in when the tree was built:
``TaskSpaceActionBehavior(target_pose=make_pose(...))`` fixed the pose at
import time, so a target computed at runtime -- a detected grasp pose, a pose
latched off a topic -- could not be expressed at all.

The action leaves now accept a blackboard *key* instead of a literal and read
it in ``initialise()``, which py_trees calls immediately before the goal is
sent. Producer and consumer have to agree on the namespace, so the namespaces
live here rather than being spelled out at each call site.
"""

import py_trees

# Where a task's runtime motion targets live. Namespaced so a key as generic
# as 'grasp_pose' cannot collide with an unrelated behaviour's variable.
TASK_NAMESPACE = '/task'

# The MIT tuning task's own namespace, kept separate on purpose: its keys are
# measurements for a report, not targets for a motion.
MIT_TUNING_NAMESPACE = '/mit_tuning'


def read_client(name, keys, namespace=TASK_NAMESPACE):
    """A blackboard client with read access to *keys* (None entries ignored)."""
    board = py_trees.blackboard.Client(name=name, namespace=namespace)
    for key in keys:
        if key:
            board.register_key(key=key, access=py_trees.common.Access.READ)
    return board


def write_client(name, keys, namespace=TASK_NAMESPACE):
    """A blackboard client with write access to *keys* (None entries ignored)."""
    board = py_trees.blackboard.Client(name=name, namespace=namespace)
    for key in keys:
        if key:
            board.register_key(key=key, access=py_trees.common.Access.WRITE)
    return board


def read_if_set(board, key, default=None):
    """Read *key* if it has been written, else *default*.

    ``getattr(board, key, default)`` does NOT do this: py_trees raises KeyError
    for a registered-but-unwritten key, and getattr's default only absorbs
    AttributeError. An unguarded read of a key whose producer has not run
    therefore escapes update(), propagates through the tick and takes the node
    down, instead of failing the one behaviour that needed it.
    """
    if not key or not board.exists(key):
        return default
    return getattr(board, key)


class TargetKey:
    """A motion target read off the blackboard at goal-send time.

    Bundles the key, the message type the action goal needs, and the client, so
    an action leaf resolving a target is one call and every leaf reports a
    missing or wrong-typed target the same way.
    """

    def __init__(self, behaviour_name, key, expected_type, namespace=TASK_NAMESPACE):
        self.key = key
        self.expected_type = expected_type
        self.namespace = namespace
        self.board = read_client(behaviour_name, [key], namespace)

    @property
    def path(self):
        """The full blackboard path, for log messages."""
        return f'{self.namespace}/{self.key}'

    def read(self, behaviour):
        """The target, or None after logging why it cannot be used.

        Returning None keeps the failure inside the behaviour: the caller skips
        send_action_goal(), so update() reports FAILURE on the next tick the
        same way an unavailable action server does.
        """
        logger = behaviour.node.get_logger()
        if not self.board.exists(self.key):
            logger.error(
                f'[{behaviour.name}] blackboard {self.path} is not set; the '
                'behaviour that writes it has to run, and succeed, first.')
            return None
        value = getattr(self.board, self.key)
        if not isinstance(value, self.expected_type):
            logger.error(
                f'[{behaviour.name}] blackboard {self.path} holds '
                f'{type(value).__name__}, but the goal needs '
                f'{self.expected_type.__name__}.')
            return None
        return value
