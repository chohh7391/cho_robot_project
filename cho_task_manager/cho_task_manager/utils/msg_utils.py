from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from typing import List


def make_pose(position: List, orientation: List = [1.0, 0.0, 0.0, 0.0]):
    """
    position: [x, y, z]
    orientation: [qx, qy, qz, qw]
    """
    p = Pose()
    p.position.x, p.position.y, p.position.z = position[0], position[1], position[2]
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = orientation[0], orientation[1], orientation[2], orientation[3]
    return p
    

def make_down_pose(height):
    p = Pose()
    p.position.x, p.position.y, p.position.z = 0.0, 0.0, float(height)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = 0.0, 0.0, 0.0, 1.0
    return p

def make_up_pose(height):
    p = Pose()
    p.position.x, p.position.y, p.position.z = 0.0, 0.0, float(-height)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = 0.0, 0.0, 0.0, 1.0
    return p

def make_joint_state(position):
    js = JointState()
    js.position = [float(p) for p in position]
    return js