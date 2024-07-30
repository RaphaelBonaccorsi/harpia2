#ifndef RP_HARPIA_EXECUTOR_HPP
#define RP_HARPIA_EXECUTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <plansys2_msgs/msg/action_dispatch.hpp>
#include <memory>

namespace plansy2 {

class RPHarpiaExecutor : public rclcpp::Node
{
public:
    /* constructor */
    RPHarpiaExecutor(const rclcpp::NodeOptions & options);

private:
    /* callback function for action dispatch */
    void concreteCallback(const plansys2_msgs::msg::ActionDispatch::SharedPtr msg);

    /* subscriber for action dispatch topic */
    rclcpp::Subscription<plansys2_msgs::msg::ActionDispatch>::SharedPtr action_dispatch_subscriber_;
};

}  // namespace plansy2

#endif  // RP_HARPIA_EXECUTOR_HPP