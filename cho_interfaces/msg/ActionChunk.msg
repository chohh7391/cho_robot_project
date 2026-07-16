std_msgs/Header header

string action_space # joint, task
bool relative
string rotation_type # euler, axis_angle, rotation6d, quaternion

int32 chunk_size

# Time between consecutive waypoints in this chunk [s]. Set this when the policy's
# action horizon differs from its inference period. <= 0 (default) falls back to
# (1 / inference_frequency) / chunk_size, i.e. the chunk spans one inference period.
float64 control_dt

float64[] arm_actions
float32[] gripper_actions