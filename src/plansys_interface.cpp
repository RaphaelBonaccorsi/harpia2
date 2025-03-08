#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
// #include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "harpia_msgs/srv/str_in_out.hpp"
#include <nlohmann/json.hpp>

#include "lifecycle_msgs/srv/get_state.hpp"     // For GetState service
#include "lifecycle_msgs/msg/state.hpp"         // For lifecycle state definitions
#include <chrono>                               // For std::chrono::seconds (missing in your list!)
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "harpia_msgs/action/execute_plan.hpp"  
#include "plansys2_msgs/msg/action_execution_info.hpp"
// #include "plansys2_msgs/msg/action_execution_info_array.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace std;

using ExecutePlan = harpia_msgs::action::ExecutePlan;
using GoalHandleMyAction = rclcpp_action::ServerGoalHandle<ExecutePlan>;

// what plansys_interface should do
// add instances, functions, predicates
// rem instances, functions, predicates
// set goal
// generate and execute plan

// service 'plansys_interface/update_parameters' to add/rem instances, functions, predicates and set goal
// action 'plansys_interface/execute_plan' to generate and execute plan

class PlansysInterface : public rclcpp_lifecycle::LifecycleNode {
public:
    PlansysInterface() : rclcpp_lifecycle::LifecycleNode("plansys_interface") {
        domain_client_ = std::make_shared<plansys2::DomainExpertClient>();
        problem_client_ = std::make_shared<plansys2::ProblemExpertClient>();
        planner_client_ = std::make_shared<plansys2::PlannerClient>();
        executor_client_ = std::make_shared<plansys2::ExecutorClient>();


        // solver = "TFD";
        solver = "OPTIC";
    }

    // Transition callback for state 'configuring'
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "Configuring...");
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // Transition callback for state 'activating'
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "Activating...");

        update_parameter_service = this->create_service<harpia_msgs::srv::StrInOut>("plansys_interface/update_parameters",std::bind(&PlansysInterface::update_parameters_callback, this, _1, _2));
        // wait_for_problem_expert_availability();


        action_server_ = rclcpp_action::create_server<ExecutePlan>(
            this,
            "plansys_interface/execute_plan",
            std::bind(&PlansysInterface::handle_goal, this, _1, _2),
            std::bind(&PlansysInterface::handle_cancel, this, _1),
            std::bind(&PlansysInterface::handle_accepted, this, _1)
        );
        
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // Transition callback for state 'deactivating'
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "Deactivating...");
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // Transition callback for state 'cleaning up'
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "Cleaning up...");
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // Transition callback for state 'shutting down'
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "Shutting down...");
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:
    
    std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
    std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
    std::shared_ptr<plansys2::PlannerClient> planner_client_;
    std::shared_ptr<plansys2::ExecutorClient> executor_client_;

    string solver;

    rclcpp::Service<harpia_msgs::srv::StrInOut>::SharedPtr update_parameter_service;
    rclcpp_action::Server<ExecutePlan>::SharedPtr action_server_;

    void update_parameters_callback(
        const std::shared_ptr<harpia_msgs::srv::StrInOut::Request> request,
        std::shared_ptr<harpia_msgs::srv::StrInOut::Response> response) {
        (void)request;  // Suppress unused parameter warning (Trigger has no request fields)

        RCLCPP_INFO(this->get_logger(), "Trigger service called!, strin: %s", request->message.c_str());

        nlohmann::json json_commands;
        try
        {
            json_commands = nlohmann::json::parse(request->message);
            RCLCPP_INFO(this->get_logger(), "Parsed JSON: %s", json_commands.dump().c_str());
        }
        catch (const nlohmann::json::parse_error &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON: %s", e.what());
            response->success = false;
            response->message = "Failed to parse JSON";
            return;
        }

        response->message = "";

        for (auto& command : json_commands) {
            string type = command["type"].get<string>();
            auto& values = command["values"];
            
            if(type == "addInstances")
            {
                if(!add_instances(values))
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to add instances");
                    response->success = false;
                    return;
                }
            }
            else if(type == "addPredicates")
            {
                if(!addPredicates(values))
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to add predicates");
                    response->success = false;
                    return;
                }
            }
            else if(type == "addFunctions")
            {
                if(!addFunctions(values))
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to add functions");
                    response->success = false;
                    return;
                }
            }
            else if(type == "setGoals")
            {
                if(!setGoals(values))
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to set goals");
                    response->success = false;
                    return;
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Unknown command type: %s", type.c_str());
                response->success = false;
            }
        }

        // // Print the domain and problemD
        // auto domain = domain_client_->getDomain();
        // auto problem = problem_client_->getProblem();

        // RCLCPP_INFO(this->get_logger(), "Domain: %s", domain.c_str());
        // RCLCPP_INFO(this->get_logger(), "Problem: %s", problem.c_str());
        
        response->success = true;
    }

    bool add_instances(nlohmann::json& instances)
    {
        RCLCPP_INFO(this->get_logger(), "Adding instances...");

        for (auto& instance : instances) {
            string str = instance.get<string>();
            try
            {
                string instance_name = str.substr(0, str.find(' '));
                string instance_type = str.substr(str.find(' ') + 1);
                RCLCPP_INFO(this->get_logger(), "Adding instance %s of type %s", instance_name.c_str(), instance_type.c_str());

                problem_client_->addInstance(plansys2::Instance(instance_name, instance_type));
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to add instance: %s", e.what());
                return false;
            }
        }

        return true;
    }

    bool addPredicates(nlohmann::json& predicates)
    {
        RCLCPP_INFO(this->get_logger(), "Adding predicates...");

        for (auto& predicate : predicates) {
            string str = predicate.get<string>();
            try
            {
                RCLCPP_INFO(this->get_logger(), "Adding predicate %s", str.c_str());

                problem_client_->addPredicate(plansys2::Predicate("("+str+")"));
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to add predicate: %s", e.what());
                return false;
            }
        }

        return true;
    }

    bool addFunctions(nlohmann::json& functions)
    {
        RCLCPP_INFO(this->get_logger(), "Adding functions...");

        for (auto& function : functions) {
            string str = function.get<string>();
            try
            {
                RCLCPP_INFO(this->get_logger(), "Adding function %s", str.c_str());
                problem_client_->addFunction(plansys2::Function("(= "+str+")"));
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to add function: %s", e.what());
                return false;
            }
        }

        return true;
    }

    bool setGoals(nlohmann::json& goals)
    {
        RCLCPP_INFO(this->get_logger(), "Setting goals...");

        string goal_str = "(and";
        for (auto& goal : goals) {
            string str = goal.get<string>();
            try
            {
                RCLCPP_INFO(this->get_logger(), "Adding goal %s", str.c_str());
                goal_str += " ("+str+")";
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to add goal: %s", e.what());
                return false;
            }
        }

        goal_str += ")";
        problem_client_->setGoal(plansys2::Goal(goal_str));

        return true;
    }


    void wait_for_problem_expert_availability()
    {
        auto client = this->create_client<lifecycle_msgs::srv::GetState>("/problem_expert/get_state");

        while (!client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrompido enquanto esperava pelo serviço /problem_expert/get_state");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Esperando o serviço /problem_expert/get_state ficar disponível...");
        }

        // Requisição para checar o estado do Problem Expert Node
        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

        while (rclcpp::ok()) {
            // Checar o estado atual
            auto future_result = client->async_send_request(request);
            
            // Esperar pela resposta do serviço
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) !=
                rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(this->get_logger(), "Falha ao chamar o serviço /problem_expert/get_state");
                continue;
            }

            auto response = future_result.get();
            if (response->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                RCLCPP_INFO(this->get_logger(), "Problem Expert está ativo.");
                break;
            } else {
                RCLCPP_INFO(this->get_logger(), "Problem Expert ainda não está ativo, estado atual: %s", response->current_state.label.c_str());
            }

            rclcpp::sleep_for(std::chrono::seconds(1));
        }
    }

    std::string executeCommand(const std::string &command)
    {
        std::string result;
        char buffer[128];
        FILE *pipe = popen(command.c_str(), "r"); // Open a pipe to run the command
        if (!pipe)
        {
            throw std::runtime_error("popen() failed!");
        }
        try
        {
            while (fgets(buffer, sizeof(buffer), pipe) != nullptr)
            {
                result += buffer; // Capture the output
            }
        }
        catch (...)
        {
            pclose(pipe);
            throw;
        }
        pclose(pipe);
        return result;
    }

    int generate_plan_custom_solver() {
        plansys2_msgs::msg::Plan gen_plan;

        // Helper lambda to add actions to the plan
        auto logger = this->get_logger();
        auto add_action = [&gen_plan, &logger](std::string action_name, double start_time) {
            RCLCPP_INFO(logger, "adding: |%s|%f|", action_name.c_str(), start_time);
            plansys2_msgs::msg::PlanItem plan_item;
            plan_item.action = action_name;
            plan_item.time = start_time;
            gen_plan.items.push_back(plan_item);
        };

        std::string solver_path = ament_index_cpp::get_package_share_directory("route_executor2") + "/solver";
        std::string domain_file_path = solver_path + "/domain.pddl";
        std::string problem_file_path = solver_path + "/problem.pddl";

        auto write_to_file = [](std::string file_path, std::string content) -> void {
            std::ofstream file(file_path);
            file << content;
            file.close();
        };

        write_to_file(domain_file_path, domain_client_->getDomain());
        write_to_file(problem_file_path, problem_client_->getProblem());
        std::string output_folder = ament_index_cpp::get_package_share_directory("route_executor2") + "/../../../../output";
        write_to_file(output_folder + "/domain.pddl", domain_client_->getDomain());
        write_to_file(output_folder + "/problem.pddl", problem_client_->getProblem());

        std::string command = solver_path + "/" + solver + "/generate_plan.sh " + domain_file_path + " " + problem_file_path;
        std::string result = executeCommand(command);

        std::stringstream ss(result);
        std::string line;

        while (std::getline(ss, line)) {
            size_t colon_pos = line.find(":");
            size_t open_paren = line.find('(');
            size_t close_paren = line.find(')');
            size_t open_bracket = line.find('[');
            size_t close_bracket = line.find(']');

            if (colon_pos == std::string::npos || open_paren == std::string::npos || close_paren == std::string::npos || open_bracket == std::string::npos || close_bracket == std::string::npos) {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse line: %s", line.c_str());
                continue;
            }

            float start_time = std::stof(line.substr(0, colon_pos));
            std::string action_name = "(" + line.substr(open_paren + 1, close_paren - open_paren - 1) + ")";
            float duration = std::stof(line.substr(open_bracket + 1, close_bracket - open_bracket - 1));

            add_action(action_name, start_time);
        }

        std::ofstream output_file(output_folder + "/plan.txt");
        output_file << "Plan created with solver " << solver << "\n";
        // Log the created plan
        RCLCPP_INFO(this->get_logger(), "Plan created:");
        for (const auto &action : gen_plan.items) {
            RCLCPP_INFO(this->get_logger(), "Action: %s, Start Time: %.2f", action.action.c_str(), action.time);
            output_file << "Action: " << action.action << ", Start Time: " << action.time << "\n";
        }
        output_file.close();

        // Send the generated plan for execution
        if (!executor_client_->start_plan_execution(gen_plan)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send the generated plan for execution.");
            return 0;
        }

        RCLCPP_INFO(this->get_logger(), "plan sent for execution.");
        return true;
    }



    // Handle an incoming goal request.
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecutePlan::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        
        if(!generate_plan_custom_solver())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to generate plan");
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(this->get_logger(), "Goal accepted");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMyAction> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMyAction> goal_handle)
    {
        // std::thread{std::bind(&PlansysInterface::execute, this, _1), goal_handle}.detach();
        std::thread([this, goal_handle]() { this->execute(goal_handle); }).detach();
    }

    // The main execution callback where work is performed.
    // This function should be called in another thread, so sleeps in this function will not freeze node execution
    void execute(const std::shared_ptr<GoalHandleMyAction> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        // const int total_steps = 1000;

        // Check if goal is still valid
        if (!goal_handle->is_executing()) {
            RCLCPP_INFO(get_logger(), "Goal rejected before execution");
            return;
        }

        int last_feedback_step = 0;
        int last_feedback_nofsteps = 0;

        bool is_canceling = false;
        string status_when_canceling = "";

        // for (int step = 0; step < total_steps; ++step) {
        while (1)
        {
            auto result = std::make_shared<ExecutePlan::Result>();
            // Check if a cancel request has been received.
            if (goal_handle->is_canceling())
            {
                if(!is_canceling)
                {
                    is_canceling = true;
                    // handle start of plan cancelation:

                    auto executor_feedback = executor_client_->getFeedBack();
                    int n_of_tasks = executor_feedback.action_execution_status.size();
                    int executing_tasks = 0;

                    status_when_canceling = "";
                    for (int i = 0; i < n_of_tasks; i++)
                    {
                        if(executor_feedback.action_execution_status[i].status == plansys2_msgs::msg::ActionExecutionInfo::EXECUTING)
                        {
                            executing_tasks++;
                        }
                        // RCLCPP_INFO(this->get_logger(), "Action: %s, Status: %d", executor_feedback.action_execution_status[i].action.c_str(), executor_feedback.action_execution_status[i].status);
                        status_when_canceling += to_string(executor_feedback.action_execution_status[i].status);
                    }

                    if(executing_tasks == 1)
                    {
                        RCLCPP_INFO(this->get_logger(), "Waiting the last action to finish to trigger cancelation...");
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "ERROR, should not have %d executing task when cancelling plan, canceling plan now", executing_tasks);
                        executor_client_->cancel_plan_execution();
                    }
                }
            }
            
            if (!executor_client_->execute_and_check_plan()) // check if plan finished
            {
                auto executor_result_opt = executor_client_->getResult();
                if (!executor_result_opt.has_value()) // if no result available
                {
                    auto my_result = std::make_shared<harpia_msgs::action::ExecutePlan::Result>();
                    my_result->success = false;
                    RCLCPP_ERROR(this->get_logger(), "No result available from executor_client");
                    goal_handle->abort(my_result);
                    return;
                }
                
                auto my_result = std::make_shared<harpia_msgs::action::ExecutePlan::Result>();
                my_result->success = executor_result_opt->success;

                if (executor_result_opt->success)
                {
                    RCLCPP_INFO(this->get_logger(), "Plan finished successfully");
                    goal_handle->succeed(my_result);
                    return;
                }
                else if(is_canceling)
                {
                    RCLCPP_INFO(this->get_logger(), "Plan cancelled successfully");
                    goal_handle->abort(my_result);
                    return;
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Plan failed");
                    goal_handle->abort(my_result);
                    return;
                }

            }
            
            // give feedback from executor feedback
            give_action_feedback_and_check_cancelation(goal_handle, last_feedback_step, last_feedback_nofsteps, status_when_canceling, is_canceling);

            std::this_thread::sleep_for(100ms);
        }
    }

    void give_action_feedback_and_check_cancelation(const std::shared_ptr<GoalHandleMyAction> goal_handle, int &last_feedback_step, int &last_feedback_nofsteps, string &status_when_canceling, bool is_canceling)
    {
        auto executor_feedback = executor_client_->getFeedBack();
        int n_of_tasks = executor_feedback.action_execution_status.size();
        int completed_tasks = 0;

        string status = "";

        for (int i = 0; i < n_of_tasks; i++)
        {
            if(executor_feedback.action_execution_status[i].status == plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED)
            {
                completed_tasks++;
            }
            status += to_string(executor_feedback.action_execution_status[i].status);
            // RCLCPP_INFO(this->get_logger(), "Action: %s, Status: %d", executor_feedback.action_execution_status[i].action.c_str(), executor_feedback.action_execution_status[i].status);
        }

        // Populate and publish feedback.
        auto feedback = std::make_shared<ExecutePlan::Feedback>();
        feedback->step = completed_tasks-1;
        feedback->nofsteps = n_of_tasks-1;

        if(last_feedback_step != feedback->step || last_feedback_nofsteps != feedback->nofsteps)
        {
            // if one of the two feedback parameters is different than last time, publish feedback

            last_feedback_step = feedback->step;
            last_feedback_nofsteps = feedback->nofsteps;
            // RCLCPP_INFO(this->get_logger(), "Publishing feedback: step %d of %d", feedback->step, feedback->nofsteps);
            goal_handle->publish_feedback(feedback);
        }

        if(is_canceling)
        {
            if(status != status_when_canceling)
            {
                // if current status then when started canceling, the action that was running at that time should be done
                
                RCLCPP_INFO(this->get_logger(), "The action that was running stoped, canceling plan now");
                executor_client_->cancel_plan_execution();
            }
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlansysInterface>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}