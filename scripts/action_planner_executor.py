from ament_index_python.packages import get_package_share_directory

from rclpy.action import ActionClient
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, TransitionEvent

class ActionPlannerExecutor():
    def __init__(self, node, node_name):
        self._node = node
        self.node_name = node_name
        self._node._action_node_client = self.create_client(ChangeState, f'/{node_name}/change_state')
        self._transition_label_to_id = {
            'configure': Transition.TRANSITION_CONFIGURE,
            'cleanup': Transition.TRANSITION_CLEANUP,
            'activate': Transition.TRANSITION_ACTIVATE,
            'deactivate': Transition.TRANSITION_DEACTIVATE,
        }
        
    def call_change_state(self, transition, callback):
        if not self._action_node_client.service_is_ready():
            self._node.get_logger().error(f'Could not change state of node {self.node_name}. Service not available.')
            return
        
        # Create a request
        request = ChangeState.Request()
        request.transition.id = self._transition_label_to_id[transition]
        request.transition.label = transition

        future = self._action_node_client.call_async(request)
        future.add_done_callback(callback)


    