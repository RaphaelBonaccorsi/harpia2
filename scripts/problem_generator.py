#!/usr/bin/env python3
import rclpy
from std_srvs.srv import Trigger
import json
import math
import time

from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn

class ProblemGenerator(LifecycleNode):
    def __init__(self):
        super().__init__('problem_generator')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring node...")

        self.data = {
            "map": None,
            "hardware": None,
            "mission": None
        }
        self.current_region = None

        self.map_cli = self.create_client(Trigger, 'data_server/map')
        self.hardware_cli = self.create_client(Trigger, 'data_server/hardware')
        self.mission_cli = self.create_client(Trigger, 'data_server/mission')
        self.gps_cli = self.create_client(Trigger, 'data_server/gps_position')

        self.getData()

        self.get_current_position()

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating node...")

        if self.data["map"] is None or self.data["hardware"] is None or self.data["mission"] is None:
            self.get_logger().info("Can't activate node: map, hardware or mission data is missing")
            return TransitionCallbackReturn.FAILURE
        
        if self.current_region is None:
            self.get_logger().info("Can't activate node: current region is missing")
            return TransitionCallbackReturn.FAILURE

        self.srv = self.create_service(Trigger, 'problem_generator/get_problem', self.get_problem_callback)

        return TransitionCallbackReturn.SUCCESS

    def getData(self):

        def handle_response(future, attr_name):
            if not future.result():
                self.get_logger().error(f"Failed to retrieve {attr_name} data")
                return
            
            self.get_logger().info(f"Received {attr_name} data successfully")
            self.data[attr_name] = json.loads(future.result().message)


        # Request map data
        map_req = Trigger.Request()
        future = self.map_cli.call_async(map_req)
        future.add_done_callback(lambda f: handle_response(f, "map"))

        # Request hardware status
        hardware_req = Trigger.Request()
        future = self.hardware_cli.call_async(hardware_req)
        future.add_done_callback(lambda f: handle_response(f, "hardware"))

        # Request mission data
        mission_req = Trigger.Request()
        future = self.mission_cli.call_async(mission_req)
        future.add_done_callback(lambda f: handle_response(f, "mission"))
    
    def get_problem_callback(self, request, response):


        if self.data['map'] is None or self.data['hardware'] is None or self.data['mission'] is None:
            response.success = False
            self.get_logger().error("map, hardware or mission data is missing")
            raise ValueError("map, hardware or mission data is missing")
            # return response


        response.success = True
        response.message = self.generate_json_problem()


        return response
    
    def generate_json_problem(self):

        instances, predicates, functions, goals = self.generate_problem_parameters()
        problem = json.dumps({
            "instances": [ f'{name} {key}' for key in instances.keys() for name in instances[key] ],
            "predicates": predicates,
            "functions": functions,
            "goals": goals
        })

        return problem
    
    # def generate_pddl_problem(self):

    #     instances, predicates, functions, goals = self.generate_problem_parameters()

    #     objects = ""
    #     init = ""
    #     goal = ""

    #     for key, value in instances.items():
    #         objects += f"  {' '.join(value)} - {key}\n"

    #     init += "  " + "\n  ".join(predicates) + "\n"
    #     init += "  " + "\n  ".join(functions) + "\n"

    #     goal += "  (and\n    " + "\n    ".join(goals) + "\n  )"

    #     return f"( define ( problem problem_1 )\n( :domain harpia )\n( :objects\n{objects}\n)\n( :init\n{init}\n)\n( :goal\n{goal}\n)\n)"
    
    def generate_problem_parameters(self):

        result = self.current_region
        if result is None:
            self.get_logger().info("Failed to get position")
            return
        lat, lon, alt, inside_region = result
        
        if inside_region == "":
            raise ValueError("Drone is not inside any region")

        EARTH_RADIUS_M = 6371000  # Raio da Terra em metros
        def haversine(reg1, reg2):
            lat1 = reg1['center'][1]
            lon1 = reg1['center'][0]
            lat2 = reg2['center'][1]
            lon2 = reg2['center'][0]
           
            # Converte graus para radianos
            lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

            # Diferenças das coordenadas
            dlat = lat2 - lat1
            dlon = lon2 - lon1

            # Fórmula de Haversine
            a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

            # Distância em metros
            return EARTH_RADIUS_M * c

        
        regions_list = []
        instances = {
            'region': [],
            'base': [],
        }
        predicates = []
        functions = []
        goals = []


        for base in self.data["map"]["bases"]:
            instances['base'].append(base["name"])
            regions_list.append(base)

        for reg in self.data["map"]["roi"]:
            instances['region'].append(reg["name"])
            regions_list.append(reg)

        
        for region1 in regions_list:
            for region2 in regions_list:
                if region1['name'] == region2['name']:
                    functions.append(f"(distance {region1['name']} {region1['name']}) 0.0")
                else:
                    functions.append(f"(distance {region1['name']} {region2['name']}) {haversine(region1, region2)}")

        for mission_command in self.data['mission']['mission_execution']:
            command = mission_command["command"]
            area = mission_command["instructions"]["area"]

            if command == "take_picture":
                predicates.append(f"picture_goal {area}")
                goals.append(f"taken_image {area}")
            
            elif command == "end":
                goals.append(f"at {area}")
            
            else:
                self.get_logger().error(f"Comando '{command}' desconhecido")
            

        functions.append(f"(battery_capacity) {self.data['hardware']['battery-capacity']}")
        # functions.append(f"(= (discharge_rate_battery) {self.data['hardware']['discharge-rate-battery']})")
        functions.append(f"(discharge_rate_battery) {0.1}")
        functions.append(f"(velocity) {self.data['hardware']['efficient_velocity']}")
        functions.append(f"(input_capacity) {self.data['hardware']['input-capacity']}")

        self.get_logger().warn("Using assumed values for inital state")
        functions.append(f"(battery_amount) {self.data['hardware']['battery-capacity']}")
        functions.append(f"(input_amount) {self.data['hardware']['input-capacity']}")
        predicates.append(f"at {inside_region}")
        
        functions.append(f"(mission_length) 0.0")

        return instances, predicates, functions, goals
    
    def get_current_position(self, strm=""):
        
        if not self.gps_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('GPS service is not available!')
            return None


        def gps_callback(furure):
            
            if future.result() is None:
                self.get_logger().error('Service call failed')
                return None

            if not future.result().success:
                self.get_logger().error('Service could not return valid response')
                return None
            lat, lon, alt, inside_regions = future.result().message.split(" ")
            inside_region = inside_regions.split(',')[0]

            self.current_region = lat, lon, alt, inside_region

        gps_req = Trigger.Request()
        future = self.gps_cli.call_async(gps_req)
        future.add_done_callback(gps_callback)
        


def main(args=None):
    rclpy.init(args=args)
    node = ProblemGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
