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

      int origin_id = extract_waypoint_id(origin_waypoint);
      int destination_id = extract_waypoint_id(destination_waypoint);

      if (origin_id != -1 && destination_id != -1) {
        RCLCPP_INFO(get_logger(), "Origin ID: %d, Destination ID: %d", origin_id, destination_id);

        if (call_generate_path_service(origin_id, destination_id)) {
          RCLCPP_INFO(get_logger(), "Path generated successfully.");
        } else {
          RCLCPP_ERROR(get_logger(), "Failed to generate path.");
          finish(false, 0.0, "Failed to generate path");
          return;
        }
      } else {
        RCLCPP_ERROR(get_logger(), "Invalid waypoint IDs.");
        return;
      }
    }
  }
  // Function to extract the integer ID from a waypoint string
  int extract_waypoint_id(const std::string & waypoint)
  {
    std::regex waypoint_regex("waypoint_(\\d+)");
    std::smatch match;
    if (std::regex_search(waypoint, match, waypoint_regex)) {
      return std::stoi(match[1].str());
    }
    return -1; // Return -1 if no ID is found
  }

  // Function to call the '/path_planner/generate_path' service
  bool call_generate_path_service(int origin_id, int destination_id)
  {
    auto client = this->create_client<harpia_msgs::srv::GeneratePath>("/path_planner/generate_path");

    // Aguarda o serviço estar disponível
    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    auto request = std::make_shared<harpia_msgs::srv::GeneratePath::Request>();
    request->origin_id = origin_id;
    request->destination_id = destination_id;

    // Envia a requisição e espera a resposta
    auto future = client->async_send_request(request);

    // Aguardando a resposta da chamada de serviço

    auto response = future.get();
    if (response) {
      RCLCPP_INFO(this->get_logger(), "Received path with %ld waypoints.", response->waypoints.size());
      finish(true, 1.0, "Path generated successfully.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to generate path.");
      finish(false, 0.0, "Failed to generate path");
    }


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
