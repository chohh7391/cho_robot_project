"""Leave the arm held when a mission fails, instead of walking away from it.

A py_trees ``Sequence`` that fails propagates straight to the root, the task
manager node sees a terminal status and shuts down -- with whichever controller
the mission was last driving still active, and nothing in the tree having tried
to put the arm somewhere safe.

:func:`guarded_mission` puts a ``Selector`` between the mission and the root:
the mission runs, and only if it fails does the abort branch run. The abort
deliberately does **not** move the arm. A motion commanded under exactly the
conditions that just produced a failure is not a recovery. It switches to the
control mode's hold controller and then verifies the switch actually took,
because the exclusive switch path is BEST_EFFORT: activating a controller the
bringup never loaded leaves ``result.ok`` true and nothing holding the arm.
"""

import py_trees

from cho_task_manager.behaviors.service import (
    ListControllersServiceBehavior,
    SwitchControllerServiceBehavior,
)
from cho_task_manager.utils.controller_names import (
    hold_controllers,
    resolve_control_mode,
)


def safe_abort_subtree(robot_config, control_mode=None, name='Safe_Abort'):
    """Switch the arm onto its hold controller and prove the switch took.

    ``control_mode`` is the mode the calling task is written for; the
    operator's ``control_mode`` parameter overrides it. Raises ValueError at
    build time when the robot declares no hold controller for the resolved
    mode -- one message now beats an unheld arm later.
    """
    mode = resolve_control_mode(robot_config, control_mode)
    holds = hold_controllers(robot_config, mode)

    seq = py_trees.composites.Sequence(name=name, memory=True)
    seq.add_children([
        SwitchControllerServiceBehavior(
            name=f'Abort_Switch_To_Hold_{mode}',
            activate=holds,
            robot_config=robot_config,
        ),
        # The switch alone does not prove anything: see the module docstring.
        ListControllersServiceBehavior(
            name='Abort_Verify_Hold_Active',
            require_active=holds,
        ),
    ])
    return seq


def watched_mission(mission, monitor, name='Mission_Under_Watch'):
    """Run *mission* alongside *monitor*, so a trip preempts the motion.

    ``SuccessOnSelected([mission])`` puts the success condition on the mission
    alone -- the monitor is a watchdog and never succeeds. Either child failing
    fails the Parallel, and that is what makes this a Parallel rather than a
    decorator: py_trees then invalidates the sibling branch, so the running
    action leaf gets terminate(INVALID) and BaseActionBehavior cancels its goal
    there. Without the cancel the motion would run to completion while the tree
    had already moved on.
    """
    return py_trees.composites.Parallel(
        name=name,
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected(children=[mission]),
        # Monitor first, so a trip is seen on the tick it happens.
        children=[monitor, mission],
    )


def guarded_mission(
    mission,
    robot_config=None,
    control_mode=None,
    name='OneShot_Root',
    abort=True,
    monitor=None,
):
    """Wrap *mission* in the standard root: safe abort on failure, then OneShot.

    ``abort=False`` reproduces the bare ``OneShot`` root exactly, for a bringup
    that has no hold controller to switch to (the OpenArm MIT prototype spawns
    only the selected MIT controller and owns its own SAFE stop).

    ``monitor`` adds a watchdog branch beside the mission (see
    :func:`watched_mission`). A trip fails the mission branch, which the abort
    then handles like any other failure -- so the arm ends up held, not just
    stopped.

    The abort branch is wrapped so that the root still reports FAILURE:
    ``FailureIsSuccess`` makes the abort best-effort, and ``Inverter`` turns its
    result back into FAILURE. Without the ``Inverter`` a successful abort would
    be reported to the operator as a successful mission.
    """
    if monitor is not None:
        mission = watched_mission(mission, monitor)

    root_child = mission
    if abort:
        guard = py_trees.composites.Selector(
            name='Mission_Or_Safe_Abort',
            # Resume at the abort branch rather than re-ticking the mission,
            # which would re-send the motion goal that just failed.
            memory=True,
        )
        guard.add_children([
            mission,
            py_trees.decorators.Inverter(
                name='Report_Mission_Failure',
                child=py_trees.decorators.FailureIsSuccess(
                    name='Abort_Best_Effort',
                    child=safe_abort_subtree(robot_config, control_mode),
                ),
            ),
        ])
        root_child = guard

    return py_trees.decorators.OneShot(
        child=root_child,
        name=name,
        policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION,
    )
