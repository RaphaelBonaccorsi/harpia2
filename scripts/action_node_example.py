#!/usr/bin/env python3

import os, sys, rclpy
from ament_index_python.packages import get_package_share_directory
package_share_path = get_package_share_directory("route_executor2")
scripts_path = os.path.join(package_share_path, 'scripts')
sys.path.append(scripts_path)

from action_executor_base import ActionExecutorBase
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, LifecycleState

class ActionNodeExample(ActionExecutorBase):

    def __init__(self):
        super().__init__("action_node_example")
        self.get_logger().info("ActionNodeExample initialized")

        self.index = 0

    def on_configure_extension(self):
        self.get_logger().info("Configuring... (example)")
        return TransitionCallbackReturn.SUCCESS
    

    def new_goal(self, goal_request):
        self.get_logger().info("New goal received")
        self.index = 0
        return True

    def execute_goal(self, goal_handle):
        self.get_logger().info("Executing goal")
        self.index += 1

        if self.index >= 60:
            self.get_logger().info("Goal completed")
            return True, 1.0
        
        self.get_logger().info(f"Goal status {self.index}/60")
        return False, self.index/60
            

    def cancel_goal(self, goal_handle):
        self.get_logger().info("Canceling goal")

def main(args=None):
    rclpy.init(args=args)
    node = ActionNodeExample()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()