#!/usr/bin/env python3

import rclpy
from rclpy.parameter import Parameter, ParameterType
from harpia_msgs.srv import GeneratePath
from geometry_msgs.msg import PoseStamped
from harpia_msgs.action import MoveTo
from rclpy.action import ActionClient
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor


class go_to(LifecycleNode):

    def __init__(self):
        super().__init__('go_to')
        self.logger = self.get_logger()
        
    def on_configure(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.logger.info('Configuring...')
        self.is_action_running = False
        self.waypoints = []  # Initialize waypoints as an empty list
        self.current_waypoint_index = 0  # Initialize the index
        self.cli = self.create_client(GeneratePath, 'path_planner/generate_path')

        self.action_client = ActionClient(self, MoveTo, '/drone/move_to_waypoint')

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.logger.info('Activating...')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('path planner service not available, waiting again...')
        
        
        
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.logger.info('Deactivating...')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.logger.info('Cleaning up...')
        self.destroy_client(self.update_parameters_client)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.logger.info('Shutting down...')
        self.destroy_client(self.update_parameters_client)
        return TransitionCallbackReturn.SUCCESS
    

def main(args=None):
    """
    Main function to initialize the go_to node and handle spinning.
    """
    rclpy.init(args=args)
    go_to = go_to()
    executor = MultiThreadedExecutor()
    executor.add_node(go_to)
    try:
        executor.spin()
    except KeyboardInterrupt:
        go_to.get_logger().info('KeyboardInterrupt, shutting down.\n')
    go_to.destroy_node()
    rclpy.shutdown()