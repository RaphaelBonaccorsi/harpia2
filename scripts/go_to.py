#!/usr/bin/env python3

import rclpy
from rclpy.parameter import Parameter, ParameterType
from harpia_msgs.srv import GeneratePath
from geometry_msgs.msg import PoseStamped
from harpia_msgs.action import MoveTo
from rclpy.action import ActionClient
from plansys2_support_py.ActionExecutorClient import ActionExecutorClient


class go_to(ActionExecutorClient):
 
    def __init__(self):
        super().__init__('go_to', 0.2)
        self.is_action_running = False
        self.waypoints = []  # Initialize waypoints as an empty list
        self.current_waypoint_index = 0  # Initialize the index
        self.cli = self.create_client(GeneratePath, 'path_planner/generate_path')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('path planner service not available, waiting again...')
        self.action_client = ActionClient(self, MoveTo, '/drone/move_to_waypoint')

    # DO NOT USE THIS ↓↓↓ FUNCTIONS TO IMPLEMENT THE ACTION 
    def finish(self, success, completion, status):
        super().finish(success, completion, status)
        self.handle_end_of_action(success, completion, status)
        self.is_action_running = False

    def do_work(self):
        if not self.is_action_running:
            self.is_action_running = True
            self.handle_start_of_action()
        self.handle_action_loop()

    def on_deactivate(self, state):
        self.is_action_running = False
        return super().on_deactivate(state)
    # DO NOT USE THIS ↑↑↑ FUNCTIONS TO IMPLEMENT THE ACTION 
    
    def handle_start_of_action(self):
        self.waypoints = []
        pass # self.get_logger().info('Starting action') # NOT_ESSENTIAL_PRINT
        self.progress_ = 0.0
        pass # self.get_logger().info(f"Current action arguments: {self.current_arguments}") # NOT_ESSENTIAL_PRINT
        self.send_path_planner(self.current_arguments[0], self.current_arguments[1])  # Now asynchronous

    # Action server go_to
    def send_goal(self, waypoint):
        """
        Sends a waypoint as a goal to the action server.

        Parameters
        ----------
        waypoint : PoseStamped
            Target waypoints messages.
        """

        if not self.is_action_running:
            pass # self.get_logger().info('Mission stoped, dont send waypoint') # NOT_ESSENTIAL_PRINT
            return


        pass # self.get_logger().info("sending waypoint") # NOT_ESSENTIAL_PRINT
        goal_msg = MoveTo.Goal()
        goal_msg.destination = waypoint

        pass # self.get_logger().info("wait...") # NOT_ESSENTIAL_PRINT
        self.action_client.wait_for_server()
        pass # self.get_logger().info(f"Sending waypoint goal: x={waypoint.pose.position.x}, y={waypoint.pose.position.y}, z={waypoint.pose.position.z}") # NOT_ESSENTIAL_PRINT

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
            self.get_logger().error('Goal rejected')
            return

        pass # self.get_logger().info('Goal accepted') # NOT_ESSENTIAL_PRINT
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
            pass # self.get_logger().info('Waypoint reached successfully') # NOT_ESSENTIAL_PRINT
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                next_waypoint = self.waypoints[self.current_waypoint_index]
                self.send_goal(next_waypoint)
            else:
                pass # self.get_logger().info('All waypoints reached') # NOT_ESSENTIAL_PRINT
        else:
            self.get_logger().error('Failed to reach waypoint')
    

    # Path planner
    def send_path_planner(self, origin, destination):
        """
        Sends a request to the path planner asynchronously.

        Parameters
        ----------
        origin : str
            The starting point of the path.
        destination : str
            The destination point of the path.
        """

        if not self.is_action_running:
            pass # self.get_logger().info('Mission stoped, dont call path_planner') # NOT_ESSENTIAL_PRINT
            return
        
        self.req = GeneratePath.Request()
        self.req.origin = origin
        self.req.destination = destination

        # Send the request asynchronously
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.path_planner_response_callback)

    def path_planner_response_callback(self, future):
        """
        Handles the response from the path planner service.

        Parameters
        ----------
        future : Future
            Future containing the service response.
        """
        try:
            response = future.result()
            pass # self.get_logger().info('response number of waypoints: ' + str(len(response.waypoints))) # NOT_ESSENTIAL_PRINT
            # if response.success: # this dont work
            if len(response.waypoints) > 0:
                pass # self.get_logger().info('Received waypoints from path planner:') # NOT_ESSENTIAL_PRINT
                for waypoint in response.waypoints:
                    pass # self.get_logger().info(f"x: {waypoint.pose.position.x:20.15f} y: {waypoint.pose.position.y:20.15f}") # NOT_ESSENTIAL_PRINT

                self.waypoints = response.waypoints  # Assuming `waypoints` is part of the response
                self.current_waypoint_index = 0  # Reset index when waypoints are received
                self.send_goal(self.waypoints[0])
            else:
                self.get_logger().error('Failed to generate path')
                self.finish(False, 0.0, 'Failed to generate path')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self.finish(False, 0.0, 'Service call exception')
            
    def handle_action_loop(self):
        if not self.waypoints:
            # Skip processing if waypoints are not yet available
            pass # self.get_logger().info('Waiting for waypoints from the path planner...') # NOT_ESSENTIAL_PRINT
            return

        self.progress_ = self.current_waypoint_index / len(self.waypoints)

        if self.progress_ < 1.0:
            self.send_feedback(self.progress_, 'move running')
        else:
            self.finish(True, 1.0, 'move completed')

    def handle_end_of_action(self, success, completion, status):
        if success:
            pass # self.get_logger().info("Action finished successfully.") # NOT_ESSENTIAL_PRINT
        else:
            self.get_logger().error(f"Action finished with error: {status}")

def main(args=None):
    rclpy.init(args=args)

    node = go_to()
    node.set_parameters([Parameter(name='action_name', value='go_to')])

    node.trigger_configure()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()