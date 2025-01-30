#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import json
import math

class ProblemGenerator(Node):
    def __init__(self):
        super().__init__('problem_generator')
        self.srv = self.create_service(Trigger, 'problem_generator/get_problem', self.service_callback)
        self.get_logger().info("Service 'problem_generator/get_problem' is ready.")


        self.map_cli = self.create_client(Trigger, 'data_server/map')
        self.hardware_cli = self.create_client(Trigger, 'data_server/hardware')
        self.mission_cli = self.create_client(Trigger, 'data_server/mission')

        while not (self.map_cli.wait_for_service(timeout_sec=1.0) and 
                   self.hardware_cli.wait_for_service(timeout_sec=1.0) and 
                   self.mission_cli.wait_for_service(timeout_sec=1.0)):
            self.get_logger().info('Still waiting for services...')


        self.map_req = Trigger.Request()
        self.hardware_req = Trigger.Request()
        self.mission_req = Trigger.Request()

        self.data = {
            "map": None,
            "hardware": None,
            "mission": None,
        }

        self.getData()

    def getData(self):

        def handle_response(future, attr_name):
            if not future.result():
                self.get_logger().error(f"Failed to retrieve {attr_name} data")
                return
            
            self.get_logger().info(f"Received {attr_name} data successfully")
            self.data[attr_name] = json.loads(future.result().message)

        # Request map data
        future = self.map_cli.call_async(self.map_req)
        future.add_done_callback(lambda f: handle_response(f, "map"))

        # Request hardware status
        future = self.hardware_cli.call_async(self.hardware_req)
        future.add_done_callback(lambda f: handle_response(f, "hardware"))

        # Request mission data
        future = self.mission_cli.call_async(self.mission_req)
        future.add_done_callback(lambda f: handle_response(f, "mission"))

    
    def service_callback(self, request, response):

        if self.data['map'] is None or self.data['hardware'] is None or self.data['mission'] is None:
            response.success = True
            self.get_logger().error("map, hardware or mission data is missing")
            return response

        response.success = True

        response.message = self.generate_problem()

        return response
    
    def generate_problem(self):

        instances, predicates, functions, goals = self.generate_problem_parameters()

        objects = ""
        init = ""
        goal = ""

        for key, value in instances.items():
            objects += f"  {' '.join(value)} - {key}\n"

        init += "  " + "\n  ".join(predicates) + "\n"
        init += "  " + "\n  ".join(functions) + "\n"

        goal += "  (and\n    " + "\n    ".join(goals) + "\n  )"

        return f"( define ( problem problem_1 )\n( :domain harpia )\n( :objects\n{objects}\n)\n( :init\n{init}\n)\n( :goal\n{goal}\n)\n)"

    
    def generate_problem_parameters(self):

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
                    functions.append(f"(= (distance {region1['name']} {region1['name']}) 0.0)")
                else:
                    functions.append(f"(= (distance {region1['name']} {region2['name']}) {haversine(region1, region2)})")

        for mission_command in self.data['mission']['mission_execution']:
            command = mission_command["command"]
            area = mission_command["instructions"]["area"]

            if command == "take_picture":
                predicates.append("(picture_goal "+area+")")
                goals.append("(taken_image "+area+")")
            
            elif command == "end":
                goals.append("(at "+area+")")
            
            else:
                self.get_logger().error(f"Comando '{command}' desconhecido")
            

        functions.append(f"(= (battery_capacity) {self.data['hardware']['battery-capacity']})")
        # functions.append(f"(= (discharge_rate_battery) {self.data['hardware']['discharge-rate-battery']})")
        functions.append(f"(= (discharge_rate_battery) {0.1})")
        functions.append(f"(= (velocity) {self.data['hardware']['efficient_velocity']})")
        functions.append(f"(= (input_capacity) {self.data['hardware']['input-capacity']})")

        self.get_logger().warn("Using assumed values for inital state")
        functions.append(f"(= (battery_amount) {self.data['hardware']['battery-capacity']})")
        functions.append(f"(= (input_amount) {self.data['hardware']['input-capacity']})")
        predicates.append(f"(at base_2)")
        
        functions.append(f"(= (mission_length) 0.0)")

        return instances, predicates, functions, goals
        


def main(args=None):
    rclpy.init(args=args)
    node = ProblemGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
