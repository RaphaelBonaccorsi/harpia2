#ifndef HARPIA_EXECUTOR
#define HARPIA_EXECUTOR

#include <rclcpp/rclcpp.hpp>
#include <plansys2_executor/ActionExecutorClient.hpp>

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

        void do_work() override;
    };

} // namespace plansys2

#endif // HARPIA_EXECUTOR