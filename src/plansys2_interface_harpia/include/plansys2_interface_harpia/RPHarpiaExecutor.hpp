#ifndef HARPIA_EXECUTOR
#define HARPIA_EXECUTOR

#include <rclcpp/rclcpp.hpp>
#include <plansys2_executor/ActionExecutorClient.hpp>
#include <plansys2_msgs/msg/action_execution_feedback.hpp>

namespace plansy2{
	class HarpiaExecutor : public plansys2::RPActionInterface{
	public:
	HarpiaExecutor()
	: plansys2::RPActionInterface("rpharpia_executor")
	{
		this->declare_parameter<double>("action_duration", 2.0);
	}

	void do_work()
	{
		auto feedback = std::make_shared<plansys2_msgs::msg::ActionExecutionFeedback>();
		feedback->progress = 0.5;
		send_feedback(feedback);

		// Adicione sua lógica de execução aqui

		finish(true, 1.0, "Action completed successfully");
	}
	};
}
#endif
