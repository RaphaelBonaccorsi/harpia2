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
        RPHarpiaExecutor()
            : plansys2::ActionExecutorClient("rpharpia_executor", std::chrono::seconds(1))
        {
            this->declare_parameter<double>("action_duration", 2.0);
        }

        void do_work() override
        {
            // Create a feedback
            auto feedback = std::make_shared<plansys2_msgs::msg::ActionExecutionInfo>();
            feedback->status = plansys2_msgs::msg::ActionExecutionInfo::EXECUTING;
            feedback->completion = 0.5;

            // Send feedback using appropriate values
            send_feedback(feedback->completion, "Executing");

            // Add your execution logic here

            // Finish the action successfully
            finish(true, 1.0, "Action completed successfully");
        }
    };

} // namespace plansys2

#endif // HARPIA_EXECUTOR