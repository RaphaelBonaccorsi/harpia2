from rclpy.action import ActionClient
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

class ActionPlannerExecutor:
    """
    Executor responsible for managing the lifecycle state transitions of a ROS 2 node.

    Parameters
    ----------
    node : rclpy.node.Node
        Parent node responsible for creating clients and logging.
    node_name : str
        Name of the target node whose lifecycle state will be managed.
    memory : object
        Singleton memory instance shared between components.

    Attributes
    ----------
    _parent_node : rclpy.node.Node
        Reference to the parent node.
    node_name : str
        Target node name for lifecycle management.
    _memory_singleton : object
        Shared memory instance.
    _action_node_client : rclpy.client.Client
        Client to call the ChangeState service for lifecycle transitions.
    _transition_label_to_id : dict
        Dictionary mapping transition labels to their respective transition IDs.
    """

    def __init__(self, node, node_name, memory):
        self._parent_node = node
        self.node_name = node_name
        self._memory_singleton = memory

        # Create a client to call the ChangeState service for the target node
        self._action_node_client = self._parent_node.create_client(
            ChangeState,
            f'/{node_name}/change_state'
        )

        # Mapping between human-readable transition labels and their corresponding IDs
        self._transition_label_to_id = {
            'configure': Transition.TRANSITION_CONFIGURE,
            'cleanup': Transition.TRANSITION_CLEANUP,
            'activate': Transition.TRANSITION_ACTIVATE,
            'deactivate': Transition.TRANSITION_DEACTIVATE,
        }
        

    def call_change_state(self, transition, callback):
        """
        Request a lifecycle state transition for the managed node.

        Parameters
        ----------
        transition : str
            The label of the desired transition. Must be a valid key in `_transition_label_to_id`.
            Examples: 'configure', 'activate', 'deactivate', 'cleanup'.
        callback : callable
            Callback function to be executed when the service call is completed.

        Returns
        -------
        None
        """
        if not self._action_node_client.service_is_ready():
            self._parent_node.get_logger().error(
                f'Could not change state of node {self.node_name}. Service not available.'
            )
            return

        # Create and populate the ChangeState request
        request = ChangeState.Request()
        request.transition.id = self._transition_label_to_id[transition]
        request.transition.label = transition

        # Send the request asynchronously and register the callback
        future = self._action_node_client.call_async(request)
        future.add_done_callback(callback)
