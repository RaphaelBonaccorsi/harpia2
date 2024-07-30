#ifndef HARPIA_EXECUTOR
#define HARPIA_EXECUTOR

#include <rclcpp/rclcpp.hpp>
#include <plansys2_executor/ActionExecutorClient.hpp>
#include <plansys2_msgs/msg/action_execution_info.hpp>

namespace plansys2
{

    class RPHarpiaExecutor : public plansys2::ActionExecutorClient
    {
    public:
        RPHarpiaExecutor::RPHarpiaExecutor()
            : plansys2::ActionExecutorClient("rpharpia_executor", std::chrono::seconds(1))
        {
            mission_fault_client_ = this->create_client<interfaces::srv::MissionFaultMitigation>("harpia/mission_fault_mitigation");
            waypoint_push_client_ = this->create_client<mavros_msgs::srv::WaypointPush>("mavros/mission/push");
            waypoint_clear_client_ = this->create_client<mavros_msgs::srv::WaypointClear>("mavros/mission/clear");

            // Declaração de parâmetros
            this->declare_parameter<double>("action_duration", 2.0);

            // Inicialização do feedback
            auto feedback = std::make_shared<plansys2_msgs::msg::ActionExecutionFeedback>();
            feedback->progress = 0.0;
            send_feedback(feedback);
        }

        void do_work() override
        {
            // Crie um feedback
            auto feedback = std::make_shared<plansys2_msgs::msg::ActionExecutionInfo>();
            feedback->status = plansys2_msgs::msg::ActionExecutionInfo::EXECUTING;
            feedback->completion = 0.5;

            // Envie o feedback usando os valores apropriados
            send_feedback(feedback->completion, "Executing");

            // Adicione sua lógica de execução aqui

            // Finalize a ação com sucesso
            finish(true, 1.0, "Action completed successfully");
        }
    };

} // namespace plansys2

#endif // HARPIA_EXECUTOR