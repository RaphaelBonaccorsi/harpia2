#ifndef HARPIA_EXECUTOR
#define HARPIA_EXECUTOR

#include <rclcpp/rclcpp.hpp>
#include <plansys2_executor/ActionExecutorClient.hpp>
#include <plansys2_msgs/msg/action_execution_feedback.hpp>  // Atualizado para a mensagem correta

namespace plansys2 {

class RPHarpiaExecutor : public plansys2::ActionExecutorClient {
public:
    RPHarpiaExecutor()
    : plansys2::ActionExecutorClient("rpharpia_executor")
    {
        this->declare_parameter<double>("action_duration", 2.0);
    }

    void do_work() override
    {
        auto feedback = std::make_shared<plansys2_msgs::msg::ActionExecutionFeedback>();
        feedback->progress = 0.5; // Use the appropriate field for feedback
        send_feedback(feedback);

        // Add your execution logic here

        finish(true, 1.0, "Action completed successfully");
    }
};

} // namespace plansys2

#endif // HARPIA_EXECUTOR


#endif
