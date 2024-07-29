#ifndef HARPIA_EXECUTOR
#define HARPIA_EXECUTOR

#include <rclcpp/rclcpp.hpp>
#include <plansys2_executor/ActionExecutorClient.hpp>
#include <plansys2_msgs/msg/action_execution_info.hpp>  // Atualizado para a mensagem correta

namespace plansys2 {
    class RPHarpiaExecutor : public plansys2::RPActionInterface {
    public:
        RPHarpiaExecutor()
        : plansys2::RPActionInterface("rpharpia_executor")
        {
            this->declare_parameter<double>("action_duration", 2.0);
        }

        void do_work()
        {
            auto feedback = std::make_shared<plansys2_msgs::msg::ActionExecutionInfo>(); // Use a mensagem correta
            feedback->status = plansys2_msgs::msg::ActionExecutionInfo::EXECUTING;
            feedback->completion = 0.5;
            send_feedback(feedback);

            // Adicione sua lógica de execução aqui

            finish(true, 1.0, "Action completed successfully");
        }
    };
}

#endif
