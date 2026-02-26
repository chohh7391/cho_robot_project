std_msgs/Header header

string action_space # joint, task
bool relative
string rotation_type # euler, axis_angle, rotation6d, quaternion

int32 chunk_size
float64[] arm_action
float32[] gripper_action