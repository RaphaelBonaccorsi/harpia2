#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "plansys2_executor/ExecutorClient.hpp" // Added for PlanSys2 Executor

class InterfacePlansys2 : public rclcpp::Node
{
public:
  InterfacePlansys2(const std::string &problem_file)
  : Node("interface_plansys2")
  {
    // Inicializar os clientes do PlanSys2 para trabalhar com PDDL
    domain_client_ = std::make_shared<plansys2::DomainExpertClient>();
    problem_client_ = std::make_shared<plansys2::ProblemExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>(); // Added Executor Client

    // Inicializar o publisher para o tópico de planos
    plan_publisher_ = this->create_publisher<plansys2_msgs::msg::Plan>("plansys2_interface/plan", 10);

    // Verificar a disponibilidade do Problem Expert antes de continuar
    wait_for_problem_expert_availability();

    // Carregar apenas o arquivo de problema PDDL
    load_pddl_files(problem_file);

    // Gerar o plano
    generate_plan();
  }

private:
  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_; // Added Executor Client
  rclcpp::Publisher<plansys2_msgs::msg::Plan>::SharedPtr plan_publisher_;  // Publisher para o plano

  // Função para aguardar a disponibilidade dos serviços do Problem Expert
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

  // Função para carregar o arquivo de problema PDDL
  void load_pddl_files(const std::string & problem_file)
  {
    // Carregar o problema
    std::ifstream problem_stream(problem_file);
    if (!problem_stream.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Erro ao abrir o arquivo de problema PDDL.");
      return;
    }

    std::string problem_content((std::istreambuf_iterator<char>(problem_stream)),
                                 std::istreambuf_iterator<char>());

    // Enviar o problema ao PlanSys2
    if (!problem_client_->addProblem(problem_content)) {
      RCLCPP_ERROR(this->get_logger(), "Erro ao carregar o problema PDDL.");
    }
  }

  // Função para gerar o plano
  void generate_plan()
  {
    auto domain = domain_client_->getDomain();
    auto problem = problem_client_->getProblem();

    // Verificar se o domínio e o problema estão configurados corretamente
    if (problem.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Problema PDDL não está configurado corretamente.");
      return;
    }

    if (domain.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Domínio PDDL não está configurado corretamente.");
      return;
    }

    // Gerar o plano
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
      RCLCPP_ERROR(this->get_logger(), "Erro ao gerar o plano.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Plano gerado com sucesso.");

    // Ler, exibir e publicar o plano gerado
    read_print_and_publish_plan(plan.value());

    // Iniciar a execução do plano
    if (executor_client_->start_plan_execution(plan.value())) {
      RCLCPP_INFO(this->get_logger(), "Plano enviado para execução.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Falha ao enviar o plano para execução.");
    }

    // Verificar se a execução foi bem-sucedida
    if (!executor_client_->execute_and_check_plan()) {
      RCLCPP_ERROR(this->get_logger(), "Falha durante a execução do plano.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Plano executado com sucesso.");
    }
  }

  // Função para ler, exibir e publicar o plano gerado
  void read_print_and_publish_plan(const plansys2_msgs::msg::Plan & plan)
  {
    RCLCPP_INFO(this->get_logger(), "Plano:");
    for (const auto & action : plan.items) {
      RCLCPP_INFO(this->get_logger(), "Ação: %s, Tempo inicial: %.2f", action.action.c_str(), action.time);
    }

    // Publicar o plano no tópico "plansys2_interface/plan"
    // plan_publisher_->publish(plan);  // Commented out to avoid redundant dispatch
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InterfacePlansys2>("/home/harpia/route_executor2/pddl/problem.pddl");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
