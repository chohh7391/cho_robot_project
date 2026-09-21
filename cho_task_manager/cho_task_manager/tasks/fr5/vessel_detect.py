"""Latch the beaker's and the flask's detected poses. The arm does not move.

This tree commands nothing. It switches no controller, sends no action goal and
needs no bringup running -- the only thing it does is subscribe to what
``cho_object_pose`` publishes and prove a task tree can latch it. That is the
point: the offsets, the tag ids and the camera extrinsics are all unverified
until someone looks, and the way to look is not to drive an arm at a number
nobody has checked against a ruler.

What it adds over ``ros2 topic echo`` is the consumer's half of the contract,
which is where a perception setup actually fails:

* the pose has to arrive in the robot's own base frame -- ``PoseTargetBehavior``
  transforms nothing, so a pose published in a camera frame FAILS here instead
  of being driven to later;
* it has to arrive within the timeout, from a cold start, which is what a
  detection gate that never opens looks like;
* it has to land on the blackboard under the key a motion leaf would read.

So a pass means a motion task would have worked, and a failure says which of
those three it was.

Run it with no robot at all::

    TABLE=$(ros2 pkg prefix --share cho_task_manager)/config/perception/vessel_detect.yaml

    # one camera; give it the serial -- two D435s live on this bench
    ros2 launch cho_realsense d435.launch.py serial_no:=_332322072253

    # the detector gets the SAME table: it is what says which tag ids to look
    # for and what edge length they are printed at
    ros2 launch cho_object_pose apriltag.launch.py objects_config:=$TABLE
        image_topic:=/camera/camera/infra1/image_rect_raw

    # where the camera is. base_link comes into existence here, so this is what
    # makes the lookup resolve without a robot running.
    ros2 run tf2_ros static_transform_publisher X Y Z YAW PITCH ROLL base_link camera_link

    ros2 launch cho_task_manager run_task_manager.launch.py task:=vessel_detect
        robot_type:=fr5 object_pose_config:=$TABLE

(the wrapped lines are one command each)

Add ``object_pose_cameras_config:=`` once more than one camera is plugged in;
without it the pose node runs the single-camera path, which is one detector on
``/detections`` and unprefixed ``tag_<id>`` frames.

Seeing it is usually faster than reading it -- ``object_pose.launch.py
rviz:=true`` draws each vessel at its detected pose, sized from the table's
``shape``, so a sideways offset that points past the glass is visible rather
than deduced.

Driving to what this finds is a separate tree. ``perceived_replay`` is the one
that exists, and it uses the detection as a GATE on a recorded trajectory
rather than as a motion target.
"""

import py_trees

from cho_task_manager.behaviors.topic import PoseTargetBehavior
from cho_task_manager.subtrees import guarded_mission
from cho_task_manager.tasks.fr5.common import VESSELS
from cho_task_manager.utils.controller_names import load_robot_config

#: Generous on purpose. cho_object_pose publishes nothing until it has a full
#: agreement window, and on a cold start that also waits on the TF buffer
#: filling and the camera's exposure settling.
DETECT_TIMEOUT_SEC = 20.0


def create_fr5_vessel_detect_tree(robot_config=None) -> py_trees.behaviour.Behaviour:
    """Wait for each vessel's pose, in the robot's base frame, and latch it."""
    robot_config = robot_config or load_robot_config('fr5')

    # The frame an absolute task-space goal would be interpreted in, taken from
    # the same registry cho_object_pose reads, so the producer's frame_id and
    # the consumer's expectation cannot drift. Deliberately arm_base_link and
    # not base_frame: the latter is a MoveIt notion and is not in the TF tree.
    base_frame = robot_config['arm_base_link']

    mission = py_trees.composites.Sequence(
        name='FR5_Vessel_Detect_Sequence', memory=True)
    mission.add_children([
        PoseTargetBehavior(
            name='Detect_%s' % vessel.name.capitalize(),
            record_as=vessel.key,
            topic=vessel.topic,
            required_frame=base_frame,
            timeout_sec=DETECT_TIMEOUT_SEC,
        )
        for vessel in VESSELS
    ])

    # abort=False, and it is not a shortcut: the safe abort exists to put an arm
    # back on a hold controller, and this tree never took the arm off one. There
    # is no controller to switch to, no bringup required, and switching one here
    # would be this tree's only act of claiming the robot.
    return guarded_mission(mission, robot_config, abort=False,
                           name='FR5_Vessel_Detect_Root')


__all__ = ['create_fr5_vessel_detect_tree', 'DETECT_TIMEOUT_SEC']
