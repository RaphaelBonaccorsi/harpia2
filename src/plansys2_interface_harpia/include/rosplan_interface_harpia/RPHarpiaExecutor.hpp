#ifndef HARPIA_EXECUTOR_HPP_
#define HARPIA_EXECUTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <plansys2_executor/ActionExecutorClient.hpp>
#include <plansys2_msgs/msg/action_execution_info.hpp>
#include <interfaces/srv/mission_fault_mitigation.hpp>
#include <mavros_msgs/srv/waypoint_push.hpp>
#include <mavros_msgs/srv/waypoint_clear.hpp>

namespace plansys2
{

class RPHarpiaExecutor : public plansys2::ActionExecutorClient
{
public:
    RPHarpiaExecutor();

    void do_work() override;

private:
    rclcpp::Client<interfaces::srv::MissionFaultMitigation>::SharedPtr mission_fault_client_;
    rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedPtr waypoint_push_client_;
    rclcpp::Client<mavros_msgs::srv::WaypointClear>::SharedPtr waypoint_clear_client_;

    // Adicione outros membros privados conforme necessário
};

} // namespace plansys2

#endif // HARPIA_EXECUTOR_HPP_
