#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from harpia_msgs.srv import StrInOut
from harpia_msgs.action import ExecutePlan
import json

class MissionController(LifecycleNode):
    def __init__(self):
        super().__init__('mission_controller')
        self.update_parameters_client = None
        self.logger = self.get_logger()
        
    def on_configure(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.logger.info('Configuring...')

        self.isMissionRunning = False
        self.isWaitingCancelation = False
        self.isWaitingToReplan = False
        self.whatToRunWhenReplan = None

        # Create service client but don't activate it yet
        self.update_parameters_client = self.create_client(StrInOut, 'plansys_interface/update_parameters')
        self.get_problem_client = self.create_client(Trigger, 'problem_generator/get_problem')
        self.goal_handle = None
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.logger.info('Activating...')
        
        # Ensure service is available
        if not self.update_parameters_client.wait_for_service(timeout_sec=5.0):
            self.logger.error('Service not available, activation failed!')
            return TransitionCallbackReturn.ERROR
        if not self.get_problem_client.wait_for_service(timeout_sec=5.0):
            self.logger.error('get_problem service not available, activation failed!')
            return TransitionCallbackReturn.ERROR

        self.execute_plan_action_client = ActionClient(self, ExecutePlan, 'plansys_interface/execute_plan')

        self.get_problem()

        # def replan_test_timer_callback():
        #     if self.replan_timer is not None:
        #         self.replan_timer.cancel()
        #     self.get_logger().info("Let's test replan")

        #     def replan_test():
        #         self.get_logger().info("replan worked?")

        #     self.request_replan(replan_test)
            
        # self.replan_timer = self.create_timer(10, replan_test_timer_callback)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.logger.info('Deactivating...')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.logger.info('Cleaning up...')
        self.destroy_client(self.update_parameters_client)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.logger.info('Shutting down...')
        self.destroy_client(self.update_parameters_client)
        return TransitionCallbackReturn.SUCCESS
    
    def send_parameters_update(self, updates, callback_func):

        # Create and send request
        request = StrInOut.Request()
        request.message = json.dumps(updates)
            
        future = self.update_parameters_client.call_async(request)
        future.add_done_callback(callback_func)

    def send_problem(self, problem):

        updates = [
            {
                'type': 'addInstances',
                'values': problem['instances']
            },
            {
                'type': 'addPredicates',
                'values': problem['predicates']
            },
            {
                'type': 'addFunctions',
                'values': problem['functions']
            },
            {
                'type': 'setGoals',
                'values': problem['goals']
            }
        ]

        def service_callback(future):
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.logger.info(f'Service call successful')
                    self.requestPlanExecution()
                else:
                    self.logger.error(f'Service call failed: {response.message}')
                    return TransitionCallbackReturn.ERROR
            else:
                self.logger.error('Service call timed out!')
                return TransitionCallbackReturn.ERROR
            
        self.send_parameters_update(updates, service_callback)

    def get_problem(self):
        request = Trigger.Request()

        def service_callback(future):
            if future.result() is None:
                self.logger.error('Service call to get problem timed out!')
                return
            
            response = future.result()
            if not response.success:
                self.logger.error(f'Failed to retrieve problem')
                return
            
            self.logger.info(f'Problem retrieved successfully')

            problem = json.loads(response.message)
            self.logger.info(f'Problem: {problem}')

            self.send_problem(problem)

        future = self.get_problem_client.call_async(request)
        future.add_done_callback(service_callback)

    def requestPlanExecution(self):

        if self.isMissionRunning:
            self.get_logger().error('Error: Cant start a plan, another is running')
            return
        
        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        self.execute_plan_action_client.wait_for_server()

        # Create goal
        goal_msg = ExecutePlan.Goal()

        # Send goal and register callbacks
        self._send_goal_future = self.execute_plan_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
            

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.isMissionRunning = True

        # Get result
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Final result: {result.success}')
        self.isMissionRunning = False

        if self.isWaitingCancelation:
            self.isWaitingCancelation = False
            self.get_logger().info('Mission canceled.')

            if self.isWaitingToReplan:
                self.isWaitingToReplan = False
                self.get_logger().info('Now replanning...')
                if self.whatToRunWhenReplan is None:
                    self.get_logger().error('No replan function defined')
                else:
                    self.whatToRunWhenReplan()
                    self.whatToRunWhenReplan = None

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: Step {feedback.step}/{feedback.nofsteps}')

        # if feedback.step == 2:
        #     self.cancel_goal()

    def request_cancel_goal(self):
        if not self.goal_handle:
            self.get_logger().error('No goal to cancel')
            return
        
        self.get_logger().info('Requesting goal cancellation...')


        def cancel_response_callback(future):
            cancel_response = future.result()
            if cancel_response.return_code == 0:  # GOAL_TERMINAL_STATE
                self.get_logger().info('Cancellation succeeded')
                self.isWaitingCancelation = True
            else:
                self.get_logger().warn(f'Cancellation failed with code: {cancel_response.return_code}')
                if self.isWaitingToReplan:
                    self.get_logger().error("could not replan, cancelation failed")
                    self.isWaitingToReplan = False
                    self.whatToRunWhenReplan = None

        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(cancel_response_callback)

    def request_replan(self, whatToRunWhenReplan):
        if self.isWaitingToReplan:
            self.get_logger().error('Already waiting to replan')
            return
        
        self.get_logger().info('Replanning...')
        self.isWaitingToReplan = True
        self.whatToRunWhenReplan = whatToRunWhenReplan
        self.request_cancel_goal()

def main(args=None):
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    node = MissionController()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()