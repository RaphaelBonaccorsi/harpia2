#!/usr/bin/env python3
import os
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Int32
from math import sqrt


class RouteExecutor(Node):
    def __init__(self):
        super().__init__('route_executor')

        # Create a publisher to the /mavros/setpoint_position/local topic for position control
        self.publisher_ = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        
        # Create service clients for arming and setting mode
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Wait for services to be available
        self.get_logger().info('Waiting for arming and set mode services...')
        self.arming_client.wait_for_service(timeout_sec=10.0)
        self.set_mode_client.wait_for_service(timeout_sec=10.0)

        # Subscriber to listen for waypoints in PoseStamped format
        self.waypoint_subscriber = self.create_subscription(
            PoseStamped, '/drone/waypoint', self.waypoint_callback, 10
        )

        # Define QoS profile with reliable reliability to match MAVROS
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber to get the current position of the drone with updated QoS
        self.position_subscriber = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.position_callback, qos_profile
        )

        # Variable to store the current position of the drone
        self.current_position = PoseStamped()

        # Waypoints list to store the received waypoints
        self.waypoints = []
        self.current_waypoint_index = 0

        # Start publishing continuous setpoints at 20Hz
        self.setpoint_timer = self.create_timer(0.05, self.publish_current_setpoint)
        self.get_logger().info('Started publishing setpoints at 20Hz...')

        # Switch to Offboard mode and arm the drone
        time.sleep(2)
        self.set_offboard_mode()
        self.arm()

    def position_callback(self, msg):
        """Callback to update the drone's current position."""
        self.current_position = msg

    def get_current_position(self):
        """Returns the current position of the drone."""
        return self.current_position.pose.position

    def publish_current_setpoint(self):
        """Publish the next setpoint or (0,0,10) if no waypoints are received."""
        msg = PoseStamped()

        if len(self.waypoints) > 0 and self.current_waypoint_index < len(self.waypoints):
            # Get the current waypoint
            waypoint = self.waypoints[self.current_waypoint_index]
            msg.pose.position.x = waypoint.pose.position.x
            msg.pose.position.y = waypoint.pose.position.y
            msg.pose.position.z = waypoint.pose.position.z
            msg.pose.orientation.w = 1.0

            # Check if the drone has reached the waypoint
            if self.has_reached_waypoint(waypoint):
                self.get_logger().info(f"Reached waypoint ({waypoint.pose.position.x}, {waypoint.pose.position.y}, {waypoint.pose.position.z})")
                if self.current_waypoint_index < len(self.waypoints) - 1:
                    self.current_waypoint_index += 1
                else:
                    self.get_logger().info("All waypoints reached.")
        else:
            # Send (0, 0, 10) when no waypoint is available
            msg.pose.position.x = 0.0
            msg.pose.position.y = 0.0
            msg.pose.position.z = 10.0
            msg.pose.orientation.w = 1.0

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published setpoint: ({msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z})")

    def set_offboard_mode(self):
        """Sets the drone's mode to OFFBOARD."""
        self.get_logger().info('Setting mode to OFFBOARD')
        set_mode_request = SetMode.Request()
        set_mode_request.custom_mode = 'OFFBOARD'
        set_mode_future = self.set_mode_client.call_async(set_mode_request)
        rclpy.spin_until_future_complete(self, set_mode_future)

        if set_mode_future.result() is not None and set_mode_future.result().mode_sent:
            self.get_logger().info('Offboard mode set successfully')
        else:
            self.get_logger().error('Failed to set Offboard mode')

    def arm(self):
        """Arms the drone."""
        self.get_logger().info('Arming the drone...')
        arm_request = CommandBool.Request()
        arm_request.value = True
        arm_future = self.arming_client.call_async(arm_request)
        rclpy.spin_until_future_complete(self, arm_future)

        if arm_future.result() is not None and arm_future.result().success:
            self.get_logger().info('Drone armed successfully')
        else:
            self.get_logger().error('Failed to arm the drone')

    def waypoint_callback(self, msg):
        """Callback to receive waypoints and store them."""
        self.get_logger().info(f"Received waypoint: ({msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z})")
        self.waypoints.append(msg)

    def has_reached_waypoint(self, waypoint, threshold=1.0):
        """Check if the drone has reached the current waypoint."""
        current_position = self.get_current_position()
        distance = sqrt(
            (waypoint.pose.position.x - current_position.x) ** 2 +
            (waypoint.pose.position.y - current_position.y) ** 2 +
            (waypoint.pose.position.z - current_position.z) ** 2
        )
        return distance < threshold


def main(args=None):
    rclpy.init(args=args)
    node = RouteExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
