#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
         
class ActionPlanner(LifecycleNode):

    def __init__(self):
        super().__init__('action_planner')

        self.declare_parameter("pddl_domain", "")
        self.domain_file = self.get_parameter("pddl_domain").get_parameter_value().string_value


    def on_configure(self, state: State) -> TransitionCallbackReturn:
        try:
            self.get_logger().info("Configuring node...")
            self.memory = ActionPlannerMemory()
            if not self.memory.init(self.domain_file, self.get_logger()):
                return TransitionCallbackReturn.ERROR
        
            return TransitionCallbackReturn.SUCCESS
        
        except Exception as e:
            self.get_logger().error(f"Error during configuration: {e}")
            return TransitionCallbackReturn.ERROR

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating node...")

        try:

            self.memory.add_instance("region", "region_1")
            self.memory.add_instance("region", "region_2")
            self.memory.add_instance("base", "base_1")

            self.memory.add_predicate("picture_goal", ["region_1"])
            self.memory.add_predicate("picture_goal", ["region_2"])
            self.memory.add_predicate("at", ["base_1"])

            self.memory.set_function("distance", ["base_1", "base_1"], 0.0000000000)
            self.memory.set_function("distance", ["base_1", "region_1"], 126.2738489105)
            self.memory.set_function("distance", ["base_1", "region_2"], 94.0538024899)
            self.memory.set_function("distance", ["region_1", "base_1"], 126.2738489105)
            self.memory.set_function("distance", ["region_1", "region_1"], 0.0000000000)
            self.memory.set_function("distance", ["region_1", "region_2"], 129.3246029227)
            self.memory.set_function("distance", ["region_2", "base_1"], 94.0538024899)
            self.memory.set_function("distance", ["region_2", "region_1"], 129.3246029227)
            self.memory.set_function("distance", ["region_2", "region_2"], 0.0000000000)
            self.memory.set_function("battery_capacity", [], 100.0000000000)
            self.memory.set_function("input_capacity", [], 1.0000000000)
            self.memory.set_function("battery_amount", [], 100.0000000000)
            self.memory.set_function("input_amount", [], 1.0000000000)
            self.memory.set_function("mission_length", [], 0.0000000000)

            self.memory.add_goal("taken_image", ["region_1"] )
            self.memory.add_goal("taken_image", ["region_2"] )
            self.memory.add_goal("at", ["base_1"] )

            domain, problem = self.memory.get_pddl()
            self.get_logger().info(f"Domain: {domain}")
            self.get_logger().info(f"Problem: {problem}")


        except Exception as e:
            self.get_logger().error(f"Error during activation: {e}")
            return TransitionCallbackReturn.ERROR
        
        return TransitionCallbackReturn.SUCCESS

def main(args=None):
    """
    Main function to initialize the action_planner node and handle spinning.
    """
    rclpy.init(args=args)
    action_planner = ActionPlanner()
    executor = MultiThreadedExecutor()
    executor.add_node(action_planner)
    try:
        executor.spin()
    except KeyboardInterrupt:
        action_planner.get_logger().info('KeyboardInterrupt, shutting down.\n')
    action_planner.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()