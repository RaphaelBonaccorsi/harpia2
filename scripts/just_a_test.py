#!/usr/bin/env python3

"""just_a_test.py

Lifecycle demo node that sends an ActionCaller goal to `/action/action_node_example`,
logs feedback, and cancels it after 10 s.

Key fixes:
* Import `Transition` from **lifecycle_msgs.msg**, not `rclpy.lifecycle`.
* Use `cancel_goal_async()` from the **goal handle**.
* Corrected indentation of `_on_cancel_timer`.
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from harpia_msgs.action import ActionCaller
from harpia_msgs.srv import StrInOut
import json
from harpia_msgs.action import ExecutePlan


class JustATest(LifecycleNode):
    def __init__(self):
        super().__init__('just_a_test')
        self._action_client = None
        self._goal_handle = None
        self._cancel_timer = None

    # ───────── Lifecycle hooks ──────────
    def on_configure(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring node…')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
      
        # Create a client for the update_parameters service

        self._update_parameters_client = self.create_client(
            StrInOut,
            'plansys_interface/update_parameters'
        )

        # Wait for the service to be available
        if not self._update_parameters_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('update_parameters service not available!')
            return TransitionCallbackReturn.ERROR

        # Example JSON command to add an instance
        req = StrInOut.Request()
        req.message = json.dumps([
        {
            "type": "add_instances",
            "values": ["region_1 region", "region_2 region", "base_1 base"],
        },
        {
            "type": "add_predicates",
            "values": ["picture_goal region_1", "picture_goal region_2", "at base_1"],
        },
        {
            "type": "add_functions",
            "values": [
                "(distance base_1 base_1) 0.0",
                "(distance base_1 region_1) 126.27384891047109",
                "(distance base_1 region_2) 94.05380248989793",
                "(distance region_1 base_1) 126.27384891047109",
                "(distance region_1 region_1) 0.0",
                "(distance region_1 region_2) 129.32460292272947",
                "(distance region_2 base_1) 94.05380248989793",
                "(distance region_2 region_1) 129.32460292272947",
                "(distance region_2 region_2) 0.0",
                "(battery_capacity) 100",
                # "(discharge_rate_battery) 0.1",
                # "(velocity) 7",
                "(input_capacity) 1",
                "(battery_amount) 100",
                "(input_amount) 1",
                "(mission_length) 0.0",
            ],
        },
        {
            "type": "set_goals",
            "values": ["taken_image region_1", "taken_image region_2", "at base_1"],
        },
            {
                "type": "log_pddl",
                "values": []
            },
        ])

        self.get_logger().info(f"Requesting update_parameters with: {req.message}")
        future = self._update_parameters_client.call_async(req)

        def _on_update_parameters_response(fut):
            try:
                resp = fut.result()
                if resp.success:
                    self.get_logger().info(f"update_parameters succeeded: {resp.message}")
                else:
                    self.get_logger().error(f"update_parameters failed: {resp.message}")
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")

        future.add_done_callback(_on_update_parameters_response)


        # Create an action client for ActionCaller        
        # After update_parameters finishes, send a goal to /action_planner/execute_plan


        self._execute_plan_client = ActionClient(self, ExecutePlan, '/action_planner/execute_plan')

        def _send_execute_plan_goal():
            if not self._execute_plan_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('ExecutePlan action server not available!')
                return

            goal_msg = ExecutePlan.Goal()
            # '{}' means default/empty goal, so no fields set

            self.get_logger().info('Sending ExecutePlan goal...')
            send_goal_future = self._execute_plan_client.send_goal_async(goal_msg)

            def _goal_response_callback(fut):
                goal_handle = fut.result()
                if not goal_handle.accepted:
                    self.get_logger().error('ExecutePlan goal rejected')
                    return
                self.get_logger().info('ExecutePlan goal accepted')

            send_goal_future.add_done_callback(_goal_response_callback)

        # Add to the update_parameters callback
        def _on_update_parameters_response(fut):
            try:
                resp = fut.result()
                if resp.success:
                    self.get_logger().info(f"update_parameters succeeded: {resp.message}")
                    _send_execute_plan_goal()
                else:
                    self.get_logger().error(f"update_parameters failed: {resp.message}")
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")

        future.add_done_callback(_on_update_parameters_response)

        return TransitionCallbackReturn.SUCCESS


# ───────── main ──────────


def main(args=None):
    """
    Main function to initialize the action_planner node and handle spinning.
    """
    rclpy.init(args=args)
    action_planner = JustATest()
    executor = MultiThreadedExecutor()
    executor.add_node(action_planner)
    try:
        executor.spin()
    except KeyboardInterrupt:
        action_planner.get_logger().info('KeyboardInterrupt, shutting down.\n')
    action_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
