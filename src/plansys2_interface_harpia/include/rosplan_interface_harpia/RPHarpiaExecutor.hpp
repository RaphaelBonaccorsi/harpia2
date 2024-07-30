#ifndef RP_HARPIA_EXECUTOR
#define RP_HARPIA_EXECUTOR

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
    private:
        rclcpp::Client<interfaces::srv::MissionFaultMitigation>::SharedPtr mission_fault_client_;
        rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedPtr waypoint_push_client_;
        rclcpp::Client<mavros_msgs::srv::WaypointClear>::SharedPtr waypoint_clear_client_;

    public:
        /* construtor */
        RPHarpiaExecutor();

        /* método para executar o trabalho */
        void do_work() override;
    };

} // namespace plansys2

#endif // RP_HARPIA_EXECUTOR
