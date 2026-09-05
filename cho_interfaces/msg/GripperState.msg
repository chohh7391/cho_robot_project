# Published by cho_controller_gripper/GripperController on ~/state.
#
# `width` is the physical opening the fingers present to the object, in metres,
# regardless of whether the underlying joint is prismatic or revolute: the
# controller maps the joint into this unit so a task can be written once and run
# on any gripper.

std_msgs/Header header

float64 width          # measured opening [m], 0 = fully closed
float64 target_width   # opening the controller is currently commanding [m]
float64 speed          # measured opening rate [m/s], positive while opening
float64 effort         # measured joint effort, 0 when the hardware exports none

bool moving            # the commanded width has not yet reached its target
# The measured width stopped following the command. On a closing move this is
# what an object in the fingers looks like; on an opening move it is a fault.
bool stalled
# The last grasp goal ended stalled inside its width tolerance, i.e. the
# controller believes an object of the expected size is held. Stays true until
# the next goal.
bool grasped
