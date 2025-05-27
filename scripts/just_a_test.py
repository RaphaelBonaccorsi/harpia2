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
        self.get_logger().info('Activating node…')

        # Create the Action Client
        self._action_client = ActionClient(self, ActionCaller, '/action/action_node_example')
        self.get_logger().info('Waiting for action server…')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available – shutting down')
            return TransitionCallbackReturn.FAILURE
        self.get_logger().info('Action server available')

        # Send the goal once activated
        self.send_goal()
        return TransitionCallbackReturn.SUCCESS

    # ───────── Action handling ──────────
    def send_goal(self):
        goal_msg = ActionCaller.Goal(action_name='pick', parameters=['box1', 'shelf2'])
        self.get_logger().info(f"Sending goal: {goal_msg.action_name} {goal_msg.parameters}")

        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        future.add_done_callback(self.goal_response_callback)

        # Start timer to cancel after 10 s
        self._cancel_timer = self.create_timer(10.0, self._on_cancel_timer)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by server')
            return

        self._goal_handle = goal_handle
        self.get_logger().info('Goal accepted – waiting for result…')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, fb_msg):
        self.get_logger().info(f"Feedback: {fb_msg.feedback.status:.2f}")

    def result_callback(self, future):
        # Stop cancel timer
        if self._cancel_timer:
            self._cancel_timer.cancel()

        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Goal succeeded: success={result.success}')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal was canceled by client')
        else:
            self.get_logger().error(f'Goal ended with status code: {status}')

    # ───────── Cancel timer callback ──────────
    def _on_cancel_timer(self):
        self.get_logger().info('10 s elapsed – sending cancel request')
        if self._goal_handle:
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda f: self.get_logger().info('Cancel request sent'))
        # One‑shot timer – stop it
        if self._cancel_timer:
            self._cancel_timer.cancel()


# ───────── main ──────────

def main(args=None):
    rclpy.init(args=args)
    node = JustATest()
    executor = MultiThreadedExecutor()
    executor.add_node(node)


    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt – shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
