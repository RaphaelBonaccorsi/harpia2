#!/usr/bin/env python3

import rclpy
import math
import json
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
import time, os, sys
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import NavSatFix

class DataServer(Node):

    def __init__(self):

        super().__init__('data_server')
        package_share_dir = get_package_share_directory('route_executor2')
        self.map_file = f"{package_share_dir}/data/map.json"
        self.hardware_file = f"{package_share_dir}/data/hardware.json"
        self.all_missions_file = f"{package_share_dir}/data/all_missions.json"
        self.mission_index = self.declare_parameter('mission_index', 1).value
        self.get_logger().info(f"@@ mission index: {self.mission_index}")
        self.home_lat = None
        self.home_long = None
        # Declaring callbackgroups for async callbacks
        files_cb = ReentrantCallbackGroup()
        position_cb = MutuallyExclusiveCallbackGroup()
        gps_cb = MutuallyExclusiveCallbackGroup()
        # Define QoS profile for the position subscription
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        # Subscribe to the drone's current position
        self.position_subscriber = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.position_callback, qos_profile, callback_group=position_cb
        )
        # Subscribe to the drone's current gps position
        self.gps_subscriber = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.gps_callback, sensor_qos, callback_group=gps_cb
        )

        self.map_srv = self.create_service(Trigger, 'data_server/map', self.map_callback, callback_group=files_cb)
        self.mission_srv = self.create_service(Trigger, 'data_server/mission', self.mission_callback, callback_group=files_cb)
        self.hardware_srv = self.create_service(Trigger, 'data_server/hardware', self.hardware_callback, callback_group=files_cb)
        self.position_srv = self.create_service(Trigger, 'data_server/drone_position', self.position_service_callback, callback_group=position_cb)
        self.home_srv = self.create_service(Trigger, 'data_server/home_position', self.home_service_callback, callback_group=gps_cb)
        self.gps_pos_srv = self.create_service(Trigger, 'data_server/gps_position', self.gps_pos_service_callback, callback_group=gps_cb)

        self.get_logger().info("Data server service is ready.")

        ## self.action_client = ActionClient(self, MoveTo, '/drone/move_to_waypoint')
    
    def gps_pos_service_callback(self, request, response):
        response.message = f'{self.drone_latitude} {self.drone_longitude} {self.drone_altitude}'
        return response

    def home_service_callback(self, request, response):
        response.message = f'{self.home_lat} {self.home_long}'
        return response

    def gps_callback(self, msg):
        self.drone_latitude = msg.latitude
        self.drone_longitude = msg.longitude
        self.drone_altitude = msg.altitude
        if self.home_lat is None:
            # Saves the home latitude and longitude
            self.home_lat = self.drone_latitude
            self.home_long = self.drone_longitude


    def position_service_callback(self, request, response):
        response.message = f'{self.current_position.x} {self.current_position.y} {self.current_position.z}'
        return response
    
    def position_callback(self, msg):
        """
        Updates the drone's current position.

        Parameters
        ----------
        msg : PoseStamped
            The current position of the drone.
        """
        self.current_position = msg.pose.position

    def map_callback(self, request, response):
        self.get_logger().info(f"Received request to send map")
        with open(self.map_file, 'r') as file:
            response.message = file.read()
        return response

    def mission_callback(self, request, response):
        self.get_logger().info(f"Received request to send mission")
        with open(self.all_missions_file, 'r') as file:
            missions = json.load(file)
            response.message = json.dumps(missions[self.mission_index])
        return response

    def hardware_callback(self, request, response):
        self.get_logger().info(f"Received request to send hardware")
        with open(self.hardware_file, 'r') as file:
            response.message = file.read()
        return response

def main(args=None):
    """
    Main function to initialize the data_server node and handle spinning.
    """
    rclpy.init(args=args)
    data_server = DataServer()
    executor = MultiThreadedExecutor()
    executor.add_node(data_server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        data_server.get_logger().info('KeyboardInterrupt, shutting down.\n')
    data_server.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()