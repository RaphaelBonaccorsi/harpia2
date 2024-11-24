#!/usr/bin/env python3

import rclpy
import math
import json
from rclpy.node import Node
from harpia_msgs.srv import GeneratePath
from geometry_msgs.msg import PoseStamped
from harpia_msgs.action import MoveTo
from rclpy.action import ActionClient


class CoordinateConverter:
    """Coordinate conversion utilities."""

    @staticmethod
    def geo_to_cart(geo_point, geo_home):
        """
        Converts geographic coordinates to Cartesian coordinates.

        Parameters
        ----------
        geo_point : tuple
            A tuple representing the latitude and longitude of the point to convert.
        geo_home : tuple
            A tuple representing the latitude and longitude of the home reference point.

        Returns
        -------
        tuple
            Cartesian coordinates (x, y) relative to the home reference point.
        """
        def calc_y(lat, home_lat):
            return (lat - home_lat) * 111320.0

        def calc_x(longi, home_long, home_lat):
            return (longi - home_long) * (111320.0 * math.cos(home_lat * math.pi / 180))

        x = calc_x(geo_point[1], geo_home[1], geo_home[0])  # Longitude
        y = calc_y(geo_point[0], geo_home[0])  # Latitude

        return x, y


class Map:
    """Map data handling with conversion and parsing capabilities."""

    def __init__(self, home_lat, home_lon):
        """
        Initializes the map with the given home coordinates.

        Parameters
        ----------
        home_lat : float
            Latitude of the home position.
        home_lon : float
            Longitude of the home position.
        """
        self.home_lat = home_lat
        self.home_lon = home_lon
        self.converter = CoordinateConverter()
        self.bases = []
        self.rois = []
        self.nfz = []

    def read_route_from_json(self, map_file):
        """
        Reads route data from a JSON file and populates bases, ROIs, and NFZs.

        Parameters
        ----------
        map_file : str
            Path to the JSON file containing route data.
        """
        with open(map_file, 'r') as file:
            data = json.load(file)

        for base in data.get('bases', []):
            center = base.get('center')
            geo_points = base.get('geo_points', [])
            if center and geo_points:
                enu_center_x, enu_center_y = self.converter.geo_to_cart((center[1], center[0]), (self.home_lat, self.home_lon))
                enu_geo_points = [
                    self.converter.geo_to_cart((gp[1], gp[0]), (self.home_lat, self.home_lon)) for gp in geo_points
                ]
                self.bases.append({
                    'id': base.get('id'),
                    'name': base.get('name'),
                    'center': (enu_center_x, enu_center_y),
                    'geo_points': enu_geo_points
                })

        for roi in data.get('roi', []):
            center = roi.get('center')
            geo_points = roi.get('geo_points', [])
            if center and geo_points:
                enu_center_x, enu_center_y = self.converter.geo_to_cart((center[1], center[0]), (self.home_lat, self.home_lon))
                enu_geo_points = [
                    self.converter.geo_to_cart((gp[1], gp[0]), (self.home_lat, self.home_lon)) for gp in geo_points
                ]
                self.rois.append({
                    'id': roi.get('id'),
                    'name': roi.get('name'),
                    'center': (enu_center_x, enu_center_y),
                    'geo_points': enu_geo_points
                })

        for nfz in data.get('nfz', []):
            center = nfz.get('center')
            geo_points = nfz.get('geo_points', [])
            if center and geo_points:
                enu_center_x, enu_center_y = self.converter.geo_to_cart((center[1], center[0]), (self.home_lat, self.home_lon))
                enu_geo_points = [
                    self.converter.geo_to_cart((gp[1], gp[0]), (self.home_lat, self.home_lon)) for gp in geo_points
                ]
                self.nfz.append({
                    'id': nfz.get('id'),
                    'name': nfz.get('name'),
                    'center': (enu_center_x, enu_center_y),
                    'geo_points': enu_geo_points
                })


class RRT:
    """Rapidly-exploring Random Tree (RRT) for path planning."""

    def __init__(self, start, goal, map_limits, step_size=1.0, max_iters=5000):
        """
        Initializes the RRT with start and goal points and map constraints.

        Parameters
        ----------
        start : tuple
            Starting point for the path (x, y).
        goal : tuple
            Goal point for the path (x, y).
        map_limits : list
            Boundaries of the map [(x_min, y_min), (x_max, y_max)].
        step_size : float, optional
            Step size for RRT expansion (default is 1.0).
        max_iters : int, optional
            Maximum iterations for RRT expansion (default is 5000).
        """
        self.start = start
        self.goal = goal
        self.map_limits = map_limits
        self.step_size = step_size
        self.max_iters = max_iters

    def build(self):
        """
        Constructs a direct path between start and goal.

        Returns
        -------
        list
            List of points forming the path between start and goal.
        """
        path = self.generate_straight_path(self.start, self.goal)
        return path

    def generate_straight_path(self, start, goal, num_points=10):
        """
        Generates a straight path between start and goal with interpolated points.

        Parameters
        ----------
        start : tuple
            Starting point (x, y).
        goal : tuple
            Goal point (x, y).
        num_points : int, optional
            Number of interpolated points (default is 10).

        Returns
        -------
        list
            List of points along the straight path.
        """
        path = []
        for i in range(num_points + 1):
            t = i / num_points
            x = (1 - t) * start[0] + t * goal[0]
            y = (1 - t) * start[1] + t * goal[1]
            path.append((x, y))
        return path


class PathPlanner(Node):
    """Path planning node for generating and executing paths."""

    def __init__(self):
        """Initializes the path planner node, map, and service."""
        super().__init__('path_planner')

        self.home_lat = -22.001333
        self.home_lon = -47.934152

        self.map = Map(self.home_lat, self.home_lon)
        self.get_logger().info("json map start")
        self.map.read_route_from_json("/home/artur/rafael/route_executor2/data/map.json")
        self.get_logger().info("json map finish")

        self.srv = self.create_service(GeneratePath, 'path_planner/generate_path', self.generate_path_callback)
        self.get_logger().info("Path planner service is ready to generate paths.")

        self.action_client = ActionClient(self, MoveTo, '/drone/move_to_waypoint')
        self.waypoints = []
        self.current_waypoint_index = 0

    def find_location_by_name(self, name):
        """
        Searches for a location by name in bases, ROIs, or NFZs.

        Parameters
        ----------
        name : str
            Name of the location to find.

        Returns
        -------
        tuple or None
            Coordinates (x, y) of the location, or None if not found.
        """
        for base in self.map.bases:
            if base['name'] == name:
                return base['center']

        for roi in self.map.rois:
            if roi['name'] == name:
                return roi['center']

        for nfz in self.map.nfz:
            if nfz['name'] == name:
                return nfz['center']

        return None

    def generate_path_callback(self, request, response):
        """
        Handles path generation requests and sets up waypoints.

        Parameters
        ----------
        request : GeneratePath.Request
            Service request containing origin and destination names.
        response : GeneratePath.Response
            Service response to be returned after path generation.

        Returns
        -------
        GeneratePath.Response
            Response object after processing.
        """
        self.get_logger().info(f"Received request to generate path from {request.origin} to {request.destination}")

        start = self.find_location_by_name(request.origin)
        goal = self.find_location_by_name(request.destination)

        if start is None or goal is None:
            self.get_logger().error("Invalid origin or destination name.")
            response.success = False
            return response

        map_limits = [(-10000, -10000), (10000, 10000)]
        rrt = RRT(start, goal, map_limits)
        path = rrt.build()

        self.waypoints = [
            self.create_waypoint_message(waypoint) for waypoint in path
        ]
        
        self.current_waypoint_index = 0
        if self.waypoints:
            self.send_goal(self.waypoints[self.current_waypoint_index])

        self.get_logger().info("Path generation complete.")
        response.success = True
        return response

    def create_waypoint_message(self, waypoint):
        """
        Creates a PoseStamped message for a given waypoint.

        Parameters
        ----------
        waypoint : tuple
            (x, y) coordinates of the waypoint.

        Returns
        -------
        PoseStamped
            Message representing the waypoint.
        """
        wp_msg = PoseStamped()
        wp_msg.pose.position.x = waypoint[0]
        wp_msg.pose.position.y = waypoint[1]
        wp_msg.pose.position.z = 10.0
        return wp_msg

    def send_goal(self, waypoint):
        """
        Sends a waypoint as a goal to the action server.

        Parameters
        ----------
        waypoint : PoseStamped
            Target waypoint message.
        """
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


def main(args=None):
    """
    Main function to initialize the path planner node and handle spinning.
    """
    rclpy.init(args=args)
    path_planner = PathPlanner()
    rclpy.spin(path_planner)
    path_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
