#!/usr/bin/env python3

"""
RouteExecutor Node

This node controls a drone using MAVROS, allowing it to navigate to specified waypoints
via ROS 2 actions. It handles offboard mode activation, drone arming, setpoint publishing,
and monitors the drone's position to provide feedback on distance to the target waypoint.

Classes
-------
RouteExecutor
    A node for executing drone movements to specified waypoints using ROS 2 actions.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from math import sqrt
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from harpia_msgs.action import MoveTo
import time


class RouteExecutor(Node):
    """
    A node that manages drone waypoint navigation using MAVROS and ROS 2 actions.

    Attributes
    ----------
    publisher_ : rclpy.publisher.Publisher
        Publisher for setpoint positions (`PoseStamped` messages).
    arming_client : rclpy.client.Client
        Client for the `CommandBool` service to arm the drone.
    set_mode_client : rclpy.client.Client
        Client for the `SetMode` service to change the flight mode.
    position_subscriber : rclpy.subscription.Subscription
        Subscriber to the drone's current position topic (`/mavros/local_position/pose`).
    setpoint_timer : rclpy.timer.Timer
        Timer to periodically publish the current setpoint.
    _action_server : rclpy.action.ActionServer
        Action server for handling waypoint movement requests.
    waypoint : PoseStamped
        The current target waypoint for the drone.
    current_position : PoseStamped
        The current position of the drone.
    """

    def __init__(self):
        """
        Initializes the RouteExecutor node, sets up publishers, service clients,
        subscriptions, timers, and the action server.
        """
        super().__init__('route_executor')

        # Publisher and service clients
        self.publisher_ = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Wait for services to become available
        pass # self.get_logger().info('Waiting for arming and set mode services...') # NOT_ESSENTIAL_PRINT
        self.arming_client.wait_for_service(timeout_sec=10.0)
        self.set_mode_client.wait_for_service(timeout_sec=10.0)

        # Initialize variables
        self.waypoint = PoseStamped()
        self.current_position = PoseStamped()

        # Define QoS profile for the position subscription
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to the drone's current position
        self.position_subscriber = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.position_callback, qos_profile
        )
        self._timer = None  # Timer para gerenciar a execução assíncrona da ação
        # Set up the action server
        self._action_server = ActionServer(
            self,
            MoveTo,
            '/drone/move_to_waypoint',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Timer for publishing setpoints
        self.setpoint_timer = self.create_timer(0.05, self.publish_current_setpoint)
        pass # self.get_logger().info('Started publishing setpoints at 20Hz...') # NOT_ESSENTIAL_PRINT

        # Activate offboard mode and arm the drone
        
        self.set_offboard_mode()
        time.sleep(2)
        self.arm()

    def goal_callback(self, goal_request):
        """
        Handles goal requests from clients.

        Parameters
        ----------
        goal_request : MoveTo.Goal
            The goal request containing the target waypoint.

        Returns
        -------
        GoalResponse
            Response indicating whether the goal is accepted or rejected.
        """
        pass # self.get_logger().info(f'Receiving move request to: {goal_request.destination.pose}') # NOT_ESSENTIAL_PRINT
        self.waypoint = goal_request.destination
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Handles cancellation requests for ongoing actions.

        Parameters
        ----------
        goal_handle : rclpy.action.GoalHandle
            The handle for the action being canceled.

        Returns
        -------
        CancelResponse
            Response indicating whether the cancellation is accepted.
        """
        pass # self.get_logger().info('Cancelling goal') # NOT_ESSENTIAL_PRINT
        self._cancel_requested = True
        return CancelResponse.ACCEPT


    async def execute_callback(self, goal_handle):
        """
        Executes the action of moving the drone to a specified waypoint.

        Parameters
        ----------
        goal_handle : rclpy.action.GoalHandle
            The handle for the action being executed.

        Returns
        -------
        MoveTo.Result
            The result of the action, indicating success or failure.
        """
        pass # self.get_logger().info('Executing movement action') # NOT_ESSENTIAL_PRINT

        feedback_msg = MoveTo.Feedback()
        result = MoveTo.Result()
        self._cancel_requested = False  # Flag para gerenciar cancelamento

        # Future para monitorar a conclusão da meta
        goal_future = rclpy.task.Future()

        def timer_callback():
            nonlocal goal_future

            # Atualizar feedback com a distância
            feedback_msg.distance = float(self.get_distance(self.waypoint))
            goal_handle.publish_feedback(feedback_msg)

            # Checar se o objetivo foi alcançado
            if self.has_reached_waypoint(self.waypoint):
                self._timer.cancel()  # Cancelar o timer
                goal_handle.succeed()
                result.success = True
                pass # self.get_logger().info('Movement completed successfully') # NOT_ESSENTIAL_PRINT
                goal_future.set_result(result)

            # Checar se o cancelamento foi solicitado
            elif self._cancel_requested:
                self._timer.cancel()  # Cancelar o timer
                goal_handle.canceled()
                result.success = False
                pass # self.get_logger().info('Action canceled') # NOT_ESSENTIAL_PRINT
                goal_future.set_result(result)

            # Publicar o setpoint atual
            else:
                self.publish_current_setpoint()

        # Configurar o timer para executar periodicamente
        self._timer = self.create_timer(0.05, timer_callback)

        # Aguardar a conclusão da meta
        await goal_future

        return result




    def position_callback(self, msg):
        """
        Updates the drone's current position.

        Parameters
        ----------
        msg : PoseStamped
            The current position of the drone.
        """
        self.current_position = msg

    def get_current_position(self):
        """
        Retrieves the drone's current position.

        Returns
        -------
        geometry_msgs.msg.Point
            The current position of the drone.
        """
        return self.current_position.pose.position

    def publish_current_setpoint(self):
        """
        Publishes the current setpoint for the drone to follow.
        """
        msg = PoseStamped()
        waypoint = self.waypoint

        msg.pose.position.x = waypoint.pose.position.x
        msg.pose.position.y = waypoint.pose.position.y
        msg.pose.position.z = waypoint.pose.position.z
        msg.pose.orientation.w = 1.0

        self.publisher_.publish(msg)

    def set_offboard_mode(self):
        """
        Sets the drone's mode to OFFBOARD.
        """
        pass # self.get_logger().info('Setting mode to OFFBOARD') # NOT_ESSENTIAL_PRINT
        set_mode_request = SetMode.Request()
        set_mode_request.custom_mode = 'OFFBOARD'
        set_mode_future = self.set_mode_client.call_async(set_mode_request)
        rclpy.spin_until_future_complete(self, set_mode_future)

        if set_mode_future.result() is not None and set_mode_future.result().mode_sent:
            pass # self.get_logger().info('Offboard mode set successfully') # NOT_ESSENTIAL_PRINT
        else:
            self.get_logger().error('Failed to set Offboard mode')

    def arm(self):
        """
        Arms the drone.
        """
        pass # self.get_logger().info('Arming the drone...') # NOT_ESSENTIAL_PRINT
        arm_request = CommandBool.Request()
        arm_request.value = True
        arm_future = self.arming_client.call_async(arm_request)
        rclpy.spin_until_future_complete(self, arm_future)

        if arm_future.result() is not None and arm_future.result().success:
            pass # self.get_logger().info('Drone armed successfully') # NOT_ESSENTIAL_PRINT
        else:
            self.get_logger().error('Failed to arm the drone')

    def has_reached_waypoint(self, waypoint, threshold=1.0):
        """
        Checks if the drone has reached the target waypoint.

        Parameters
        ----------
        waypoint : PoseStamped
            The target waypoint.
        threshold : float, optional
            Distance threshold to consider the waypoint reached (default is 1.0).

        Returns
        -------
        bool
            True if the drone is within the threshold distance to the waypoint, False otherwise.
        """
        distance = self.get_distance(waypoint)
        return distance < threshold

    def get_distance(self, waypoint):
        """
        Calculates the distance between the current position and a target waypoint.

        Parameters
        ----------
        waypoint : PoseStamped
            The target waypoint.

        Returns
        -------
        float
            The distance to the waypoint.
        """
        current_position = self.get_current_position()
        distance = sqrt(
            (waypoint.pose.position.x - current_position.x) ** 2 +
            (waypoint.pose.position.y - current_position.y) ** 2 +
            (waypoint.pose.position.z - current_position.z) ** 2
        )
        return distance


def main(args=None):
    """
    Entry point for the RouteExecutor node.

    Parameters
    ----------
    args : list, optional
        Command-line arguments passed to the node (default is None).
    """
    rclpy.init(args=args)
    node = RouteExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
