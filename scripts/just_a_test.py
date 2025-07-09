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
from action_msgs.srv import CancelGoal
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
        self.get_logger().info('Configuring node...')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:


        # Update parameters service client
        self._update_parameters_client = self.create_client(
            StrInOut,
            'plansys_interface/update_parameters')
        
        if not self._update_parameters_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('update_parameters service not available!')
            return TransitionCallbackReturn.ERROR
        

        self._execute_plan_client = ActionClient(self, ExecutePlan, '/action_planner/execute_plan')

        if not self._execute_plan_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('ExecutePlan action server not available!')
            return TransitionCallbackReturn.ERROR
        

        self.get_logger().info('starting timer...')
        self.get_logger().info(f"\n\n\n\n\n\n")
        self.some_timer = self.create_timer(5, self.call_update_parameters)

        return TransitionCallbackReturn.SUCCESS

    def call_update_parameters(self):
        
        self.destroy_timer(self.some_timer)

        self.get_logger().info('@@ Calling update_parameters service...')

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

        # self.get_logger().info(f"Requesting update_parameters with: {req.message}")
        future = self._update_parameters_client.call_async(req)

        def _on_update_parameters_response(fut):
            try:
                resp = fut.result()
                if resp.success:
                    self.get_logger().info(f"update_parameters succeeded: {resp.message}")
                    self.get_logger().info(f"starting timer")
                    self.get_logger().info(f"\n\n\n\n\n\n")
                    self.some_timer = self.create_timer(5, self.call_execute_plan)
                else:
                    self.get_logger().error(f"update_parameters failed: {resp.message}")
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")

        future.add_done_callback(_on_update_parameters_response)

    def call_execute_plan(self):
        self.destroy_timer(self.some_timer)

        goal_msg = ExecutePlan.Goal()
        # '{}' means default/empty goal, so no fields set

        self.get_logger().info('@@ Sending ExecutePlan goal...')
        send_goal_future = self._execute_plan_client.send_goal_async(goal_msg)

        def _goal_response_callback(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().error('ExecutePlan goal rejected')
                return
            self.get_logger().info('ExecutePlan goal accepted')

            self.some_timer = self.create_timer(
                2.0,
                lambda: self.call_cancel_goal(goal_handle)
            )

            # self.get_logger().info(f"\n\n\n\n\n\n")

        send_goal_future.add_done_callback(_goal_response_callback)
    
    def call_cancel_goal(self, goal_handle):
        self.destroy_timer(self.some_timer)

        self.get_logger().info('@@ Cancelling goal')
        cancel_future = goal_handle.cancel_goal_async()
        def _on_cancel_done(fut):
            self.get_logger().info('on_cancel_done called')
            resp = fut.result()
            if resp.return_code != CancelGoal.Response.ERROR_NONE:
                self.get_logger().error(f"The goal cancelation request returned with error code: {resp.return_code}")
                return
            
            if any(g.goal_id == goal_handle.goal_id for g in resp.goals_canceling): # if the current goal is in the list of goals being canceled
                self.get_logger().info("The goal cancelation request was accepted.")
            else:
                self.get_logger().info("The goal cancelation request was rejected.")
            
        cancel_future.add_done_callback(_on_cancel_done)


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
