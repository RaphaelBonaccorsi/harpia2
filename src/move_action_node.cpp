#include <memory>
#include <algorithm>
#include <iostream>
#include <regex>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "harpia_msgs/srv/generate_path.hpp" // Custom service

using namespace std::chrono_literals;

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction()
  : plansys2::ActionExecutorClient("move", 100ms)
  {
    progress_ = 0.0;
  }
private:
  void do_work(){
    // Call the path planning service when the node is activated
    auto arguments = get_arguments();
    if (arguments.size() >= 3) {
      std::string origin_waypoint = arguments[1];
      std::string destination_waypoint = arguments[2];

      if (origin_waypoint.empty() || destination_waypoint.empty()) {
        RCLCPP_ERROR(get_logger(), "Invalid waypoint IDs.");
        return;
      }

      if (call_generate_path_service(origin_waypoint, destination_waypoint)) {
        RCLCPP_INFO(get_logger(), "Path generated successfully.");
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to generate path.");
        finish(false, 0.0, "Failed to generate path");
        return;
      }
    }
  }

  // Function to call the '/path_planner/generate_path' service
  bool call_generate_path_service(std::string origin, std::string destination)
  {
    auto client = this->create_client<harpia_msgs::srv::GeneratePath>("/path_planner/generate_path");

    // Aguarda o serviço estar disponível
    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service /path_planner/generate_path. Exiting.");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    auto request = std::make_shared<harpia_msgs::srv::GeneratePath::Request>();
    request->origin = origin;
    request->destination = destination;

    // Envia a requisição e espera a resposta
    auto future_result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service /path_planner/generate_path.");
      finish(false, 0.0, "Service call failed.");
      return false;
    }

    auto response = future_result.get();
    if (!response->success) {  // Assumindo que o serviço tem um campo "success"
      RCLCPP_ERROR(this->get_logger(), "Service responded with failure.");
      finish(false, 0.0, "Failed to generate path.");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Path generated successfully.");
    finish(true, 1.0, "Path generated successfully.");
    return true;
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "move"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  // Use spin with NodeBaseInterface
  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}