#!/usr/bin/env python3

import rclpy
import math
import json
from rclpy.node import Node
from harpia_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
import time, os, sys

class DataServer(Node):

    def __init__(self):

        super().__init__('data_server')
        try:
            package_share_dir = get_package_share_directory('route_executor2')
            self.map_file = f"{package_share_dir}/data/map.json"
            # self.map.read_route_from_json("/home/artur/rafael/route_executor2/data/map.json")
        except:
            self.get_logger().error("map.json not found, check the path and permissions.")
        """ Services declaration """
        self.map_srv = self.create_service(GetMap, 'data_server/map', self.map_callback)
        #self.mission_srv = self.create_service(getMission, 'data_server/mission', self.mission_callback)
        #self.hardware_srv = self.create_service(getHardware, 'data_server/hardware', self.hardware_callback)
        self.get_logger().info("Data server service is ready.")

        ## self.action_client = ActionClient(self, MoveTo, '/drone/move_to_waypoint')
    
    def map_callback(self, request, response):
        self.get_logger().info(f"Received request to send map")
        with open(self.map_file, 'r') as file:
            response.map_file = file.read()
        return response

    #def mission_callback(self, request, response):

    #def hardware_callback(self, request, response):

def main(args=None):
    """
    Main function to initialize the data_server node and handle spinning.
    """
    rclpy.init(args=args)
    data_server = DataServer()
    rclpy.spin(data_server)
    data_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()