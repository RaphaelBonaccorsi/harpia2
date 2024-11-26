#!/usr/bin/env python3

import rclpy
from rclpy.parameter import Parameter, ParameterType
from harpia_msgs.srv import GeneratePath
from geometry_msgs.msg import PoseStamped
from harpia_msgs.action import MoveTo
from rclpy.action import ActionClient
from plansys2_support_py.ActionExecutorClient import ActionExecutorClient


class move(ActionExecutorClient):

    def __init__(self):
        super().__init__('move', 0.2)
        self.is_new_action = True
        self.cli = self.create_client(GeneratePath, 'path_planner/generate_path')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...')
        self.action_client = ActionClient(self, MoveTo, '/drone/move_to_waypoint')

    # DO NOT USE THIS ↓↓↓ FUNCTIONS TO IMPLEMENT THE ACTION 
    def finish(self, success, completion, status):
        super().finish(success, completion, status)
        self.handle_end_of_action(success, completion, status)
        self.is_new_action = True

    def do_work(self):
        if self.is_new_action:
            self.is_new_action = False
            self.handle_start_of_action()
        self.handle_action_loop()
    # DO NOT USE THIS ↑↑↑ FUNCTIONS TO IMPLEMENT THE ACTION 
    
    def handle_start_of_action(self):
        self.waypoints = []
        self.get_logger().info('Starting action')
        self.progress_ = 0.0
        self.get_logger().info(f"Current action arguments: {self.current_arguments}")
        self.waypoints = self.send_path_planner(self.current_arguments[1], self.current_arguments[2]) # Synchronous Call, possible deadlock !!!
        self.get_logger().info('Received waypoints from path planner')

        self.current_waypoint_index = 0
        self.send_goal(self.waypoints)
    
    def send_goal(self, waypoints):
        """
        Sends a waypoint as a goal to the action server.

        Parameters
        ----------
        waypoints : PoseStamped[]
            Target waypoints messages.
        """
        for waypoint in waypoints:
            goal_msg = MoveTo.Goal()
            goal_msg.destination = waypoint

            self.action_client.wait_for_server()
            self.get_logger().info(f"Sending waypoint goal: x={waypoint.pose.position.x}, y={waypoint.pose.position.y}, z={waypoint.pose.position.z}")

            send_goal_future = self.action_client.send_goal_async(
                goal_msg, feedback_callback=self.feedback_callback
            )
            send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """
        Receives feedback from the action server.

        Parameters
        ----------
        feedback_msg : MoveTo.FeedbackMessage
            Feedback message containing the distance to the waypoint.
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback received: Distance to waypoint: {feedback.distance:.2f}m")

    def goal_response_callback(self, future):
        """
        Handles the response from the action server after sending a goal.

        Parameters
        ----------
        future : Future
            Future containing the goal handle.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """
        Processes the result of a goal after completion.

        Parameters
        ----------
        future : Future
            Future containing the result of the goal.
        """
        result = future.result().result
        if result:
            self.get_logger().info('Waypoint reached successfully')
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                next_waypoint = self.waypoints[self.current_waypoint_index]
                self.send_goal(next_waypoint)
            else:
                self.get_logger().info('All waypoints reached')
        else:
            self.get_logger().info('Failed to reach waypoint')

    def send_path_planner(self, origin, destination):
        self.req = GeneratePath.Request()
        self.req.origin = origin
        self.req.destination = destination
        return self.cli.call(self.req)

    def handle_action_loop(self):
        self.progress_ = self.current_waypoint_index / len(self.waypoints)
        self.get_logger().info(f'moveing ... {self.progress_:.2f}')
        
        if self.progress_ < 1.0:
            self.send_feedback(self.progress_, 'move running')
        else:
            self.finish(True, 1.0, 'move completed')

    def handle_end_of_action(self, success, completion, status):
        if success:
            self.get_logger().info("Action finished successfully.")
        else:
            self.get_logger().info(f"Action finished with error: {status}")

def main(args=None):
    rclpy.init(args=args)

    node = move()
    node.set_parameters([Parameter(name='action_name', value='move')])

    node.trigger_configure()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
