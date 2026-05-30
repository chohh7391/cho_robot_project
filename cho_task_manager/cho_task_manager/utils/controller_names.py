from enum import Enum


ACTION_SERVER_NAMESPACE = '/controller_action_server'
CONTROLLER_MANAGER_NAMESPACE = '/controller_manager'

SWITCH_CONTROLLER_SERVICE = f'{CONTROLLER_MANAGER_NAMESPACE}/switch_controller'
LIST_CONTROLLERS_SERVICE = f'{CONTROLLER_MANAGER_NAMESPACE}/list_controllers'

class ControllerNames(str, Enum):
    # Joint Space Controllers
    JOINT_IMPEDANCE = 'joint_space_impedance_controller'
    JOINT_QP = 'joint_space_qp_controller'
    
    # Task Space Controllers
    IK = 'task_space_ik_controller'
    OPERATIONAL_SPACE = 'operational_space_controller'
    TASK_IMPEDANCE = 'task_space_impedance_controller'
    TASK_QP = 'task_space_qp_controller'

    # VLA
    VLA = 'vla_controller'
    
    # Others
    GRAVITY_COMPENSATION = 'gravity_compensation_controller'
    GRIPPER = 'gripper_controller'

    def __str__(self):
        return self.value


def controller_name_value(controller):
    if isinstance(controller, ControllerNames):
        return controller.value
    return str(controller)


def controller_action_name(controller):
    return f'{ACTION_SERVER_NAMESPACE}/{controller_name_value(controller)}'


def valid_controller_action_names():
    return [controller_action_name(controller) for controller in ControllerNames]


def vla_completion_service_name():
    return f'{controller_action_name(ControllerNames.VLA)}/notify_completion'
