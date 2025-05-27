from rclpy.action import ActionClient
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from enum import Enum

class ActionState(Enum):
    PENDING = 1
    IN_PROGRESS = 2
    COMPLETED = 3

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

    def __init__(self, node, memory):
        self._parent_node = node
        # self.node_name = node_name
        self.memory = memory
        self.get_logger().info("ActionPlannerExecutor initialized")

        self.current_plan = None

        # # Create a client to call the ChangeState service for the target node
        # self._action_node_client = self._parent_node.create_client(
        #     ChangeState,
        #     f'/{node_name}/change_state'
        # )

        # # Mapping between human-readable transition labels and their corresponding IDs
        # self._transition_label_to_id = {
        #     'configure': Transition.TRANSITION_CONFIGURE,
        #     'cleanup': Transition.TRANSITION_CLEANUP,
        #     'activate': Transition.TRANSITION_ACTIVATE,
        #     'deactivate': Transition.TRANSITION_DEACTIVATE,
        # }
        

    # def call_change_state(self, transition, callback):
    #     """
    #     Request a lifecycle state transition for the managed node.

    #     Parameters
    #     ----------
    #     transition : str
    #         The label of the desired transition. Must be a valid key in `_transition_label_to_id`.
    #         Examples: 'configure', 'activate', 'deactivate', 'cleanup'.
    #     callback : callable
    #         Callback function to be executed when the service call is completed.

    #     Returns
    #     -------
    #     None
    #     """
    #     if not self._action_node_client.service_is_ready():
    #         self._parent_node.get_logger().error(
    #             f'Could not change state of node {self.node_name}. Service not available.'
    #         )
    #         return

    #     # Create and populate the ChangeState request
    #     request = ChangeState.Request()
    #     request.transition.id = self._transition_label_to_id[transition]
    #     request.transition.label = transition

    #     # Send the request asynchronously and register the callback
    #     future = self._action_node_client.call_async(request)
    #     future.add_done_callback(callback)

    def get_logger(self):
        # if self._parent_node.get_logger() is None:
        #     raise ValueError("Logger not initialized, use the function .init()")
        return self._parent_node.get_logger()
    
    def print_action(self, action):
        self.get_logger().info(f"action: {action['name']} {action['args']} ({action['state']})")

    def execute_plan(self, plan, on_success=None, on_failure=None):
        self.on_plan_success = on_success
        self.on_plan_failure = on_failure
        self.get_logger().info("plan:\n"+'\n'.join([ f"{action[0]} {' '.join(action[1])}" for action in plan]))

        if self.current_plan is not None:
            self.get_logger().warn("plan already in progress")
            return
        
        self.current_plan = []

        for action in plan:
            if self.memory._get_action_by_name(action[0]) is None:
                self.get_logger().error(f"action {action[0]} from plan not found on domain")
                return
            
            new_action = {}
            new_action['name'] = action[0]
            new_action['args'] = action[1]
            new_action['state'] = ActionState.PENDING

            self.current_plan.append(new_action)

        [self.print_action(action) for action in self.current_plan]
        self.check_if_can_start_action()
        [self.print_action(action) for action in self.current_plan]

        self.check_actions_condition_timer = self._parent_node.create_timer(1, self.check_in_progress_action_conditions)

    def check_if_can_start_action(self):
        if self.current_plan is None:
            self.get_logger().error("no plan in progress")
            return
        
        for action in self.current_plan:

            if action['state'] == ActionState.PENDING:
                if self.memory.check_conditions_action(action['name'], action['args'], "at start"):
                    self.get_logger().info(f"action {action['name']} can be started")
                    self.start_action(action)
                    continue
                else:
                    self.get_logger().info(f"action {action['name']} can NOT be started")
                    break
            
    def start_action(self, action):        
        action['state'] = ActionState.IN_PROGRESS
        self.memory.apply_effects(action['name'], action['args'], ["at start"])
        self.get_logger().info(f"action {action['name']} started")

    def check_in_progress_action_conditions(self):
        if self.current_plan is None:
            self.get_logger().error("no plan in progress")
            return
        
        for action in self.current_plan:
            if action['state'] == ActionState.IN_PROGRESS:
                if not self.memory.check_conditions_action(action['name'], action['args'], "over all"):
                    self.get_logger().error(f"action {action['name']} do not meet over all conditions")
                    self.handle_plan_failure()

    def handle_plan_failure(self):
        self.get_logger().error("Plan failed")
        if self.on_plan_failure is not None:
            self.on_plan_failure()
        self.current_plan = None

        self.destroy_timer(self.check_actions_condition_timer)

    def handle_plan_success(self):
        self.get_logger().info("Plan succeeded")
        if self.on_plan_success is not None:
            self.on_plan_success()
        self.current_plan = None

        self.destroy_timer(self.check_actions_condition_timer)