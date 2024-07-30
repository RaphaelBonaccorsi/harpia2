#ifndef HARPIA_EXECUTOR
#define HARPIA_EXECUTOR

#ifndef RP_HARPIA_EXECUTOR
#define RP_HARPIA_EXECUTOR

#include <rclcpp/rclcpp.hpp>
#include <plansys2_executor/ActionExecutorClient.hpp>
#include <plansys2_msgs/msg/action_execution_info.hpp>

namespace plansys2
{

    class RPHarpiaExecutor : public plansys2::ActionExecutorClient
    {
    private:
        // Aqui você pode adicionar membros privados, se necessário

    public:
        /* construtor */
        RPHarpiaExecutor();

        /* método para executar o trabalho */
        void do_work() override;
    };

} // namespace plansys2

#endif // RP_HARPIA_EXECUTOR
