#!/usr/bin/env python3

import rclpy, os, sys, subprocess, re
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
         
from ament_index_python.packages import get_package_share_directory
package_share_path = get_package_share_directory("route_executor2")
scripts_path = os.path.join(package_share_path, 'scripts')
sys.path.append(scripts_path)
from action_planner_memory import ActionPlannerMemory
from action_planner_executor import ActionPlannerExecutor
from harpia_msgs.action import ExecutePlan

class ActionPlanner(LifecycleNode):

    def __init__(self):
        super().__init__('action_planner')

        self.declare_parameter("pddl_domain", "")
        self.domain_file = self.get_parameter("pddl_domain").get_parameter_value().string_value

        self.solver = "TFD"
        # self.solver = "OPTIC"

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        try:
            self.get_logger().info("Configuring node...")
            self.memory = ActionPlannerMemory()
            if not self.memory.init(self.domain_file, self.get_logger()):
                return TransitionCallbackReturn.ERROR
            
            self.plan_executor = ActionPlannerExecutor(self, self.memory)
        
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
            self.memory.add_predicate("at", ["region_1"])

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

            plan = self.generate_plan_custom_solver()

            self.get_logger().info(f"Generated plan:")
            for action in plan:
                self.get_logger().info(f"{action[0]} "+" ".join(action[1]))
                        
            self.plan_executor.execute_plan(plan)
            
        except Exception as e:
            self.get_logger().error(f"Error during activation: {e}")
            return TransitionCallbackReturn.ERROR
        
        return TransitionCallbackReturn.SUCCESS
    
    def generate_plan_custom_solver(self) -> bool:
        """
        Write domain/problem PDDL to files under <package_share>/solver,
        call the external generate_plan.sh with the chosen solver, parse
        the output into a plan list, and send it to plan_executor.
        """

        solver_base = get_package_share_directory("route_executor2")
        solver_dir = os.path.join(solver_base, "solver")
        domain_path = os.path.join(solver_dir, "domain.pddl")
        problem_path = os.path.join(solver_dir, "problem.pddl")

        domain_pddl_text, problem_pddl_text = self.memory.get_pddl()

        try:
            with open(domain_path, "w") as f:
                f.write(domain_pddl_text)
        except Exception as e:
            self.get_logger().error(f"Failed to write domain file: {e}")
            return False

        try:
            with open(problem_path, "w") as f:
                f.write(problem_pddl_text)
        except Exception as e:
            self.get_logger().error(f"Failed to write problem file: {e}")
            return False

        solver_subdir = os.path.join(solver_dir, self.solver)
        generate_script = os.path.join(solver_subdir, "generate_plan.sh")
        cmd = [generate_script, domain_path, problem_path]

        self.get_logger().info(f"Calling external solver: {' '.join(cmd)}")
        try:
            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=False  # We’ll check returncode ourselves
            )
        except FileNotFoundError:
            self.get_logger().error(f"Solver script not found: {generate_script}")
            return False
        except Exception as e:
            self.get_logger().error(f"Error when running solver: {e}")
            return False

        if result.returncode != 0:
            self.get_logger().error(f"Solver returned non-zero exit code {result.returncode}:\n{result.stderr}")
            return False

        regex_pattern = r"^(\d+\.\d+):\s\(([^)]+)\)\s\[(\d+\.\d+)\]$"
        plan = []

        for line in result.stdout.splitlines():
            self.get_logger().debug(f"Parsing line: {line.strip()}")
            match = re.match(regex_pattern, line.strip())

            if match:
                start_time = float(match.group(1))
                action_parameters = match.group(2).split()
                action_name = action_parameters.pop(0)
                duration = float(match.group(3))

                plan.append((action_name, action_parameters))

        return plan

    # (…existing methods, e.g. on_deactivate(), on_cleanup(), etc.…)

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