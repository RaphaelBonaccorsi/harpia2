#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class take_image : public plansys2::ActionExecutorClient
{
public:
  take_image()
  : plansys2::ActionExecutorClient("take_image", 100ms)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    // // Retrieve the action parameters
    // const auto & args = get_arguments();
    // if (args.size() < 3) {
    //   RCLCPP_ERROR(get_logger(), "Not enough parameters for the 'take_image' action");
    //   finish(false, 0.0, "Failed due to missing parameters");
    //   return;
    // }

    // // Access the parameters
    // std::string drone = args[0];       // ?d (drone)
    // std::string from_waypoint = args[1]; // ?from (start waypoint)
    // std::string to_waypoint = args[2];   // ?to (destination waypoint)

    if (progress_ < 1.0) {
      progress_ += 0.2;

      // Send frequent feedback with updated progress
      // std::string feedback_message = "(Msg 1) Moving " + drone + " from " + from_waypoint + " to " + to_waypoint + ", progress: " + std::to_string(progress_ * 100.0) + "%";
      std::string feedback_message = "take_image progress: " + std::to_string(progress_ * 100.0) + "%";
      send_feedback(progress_, feedback_message);

      // Update console output
      RCLCPP_INFO(this->get_logger(), "%s", feedback_message.c_str());

    } else {
      finish(true, 1.0, "take_image completed");
      progress_ = 0.0;
      RCLCPP_INFO(this->get_logger(), "take_image action completed successfully.");
    }
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<take_image>();

  node->set_parameter(rclcpp::Parameter("action_name", "take_image"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}