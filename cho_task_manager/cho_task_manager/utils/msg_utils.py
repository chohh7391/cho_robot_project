from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState


def make_pose(x, y, z, qx=1.0, qy=0.0, qz=0.0, qw=0.0):
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = float(qx), float(qy), float(qz), float(qw)
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

def make_joint_state(positions):
    js = JointState()
    js.position = [float(p) for p in positions]
    return js