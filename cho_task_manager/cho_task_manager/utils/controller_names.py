from enum import Enum

class ControllerNames(str, Enum):
    # Joint Space Controllers
    JOINT_IMPEDANCE = 'joint_space_impedance_controller'
    JOINT_QP = 'joint_space_qp_controller'
    
    # Task Space Controllers
    OPERATIONAL_SPACE = 'operational_space_controller'
    TASK_QP = 'task_space_qp_controller'
    
    # Others
    GRAVITY_COMPENSATION = 'gravity_compensation_controller'
    GRIPPER = 'gripper_controller'

    def __str__(self):
        return self.value