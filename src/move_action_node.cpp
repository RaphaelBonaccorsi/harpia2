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
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"

class WaypointNavigator : public rclcpp::Node
{
public:
  WaypointNavigator()
  : Node("waypoint_navigator")
  {
    // Criar o publisher para enviar os índices dos waypoints
    waypoint_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/drone/waypoint_index", 10);

    // Inicializar os clientes do PlanSys2 para trabalhar com PDDL
    domain_client_ = std::make_shared<plansys2::DomainExpertClient>();
    problem_client_ = std::make_shared<plansys2::ProblemExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();

    // Verificar a disponibilidade do Problem Expert antes de continuar
    wait_for_problem_expert_availability();

    // Carregar o problema PDDL (domínio assume-se corretamente configurado via arquivo de launch)
    load_pddl_files("/home/harpia/route_executor2/pddl/problem.pddl");

    // Gerar o plano
    generate_plan();
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr waypoint_publisher_;
  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;

  std::vector<int> waypoints_; // Armazena os índices dos waypoints no plano gerado

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

  // Função para carregar os arquivos PDDL
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
    std::chrono::nanoseconds duration{10000000000};
    rclcpp::sleep_for(duration);
    
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

    // Processar as ações no plano e extrair os índices dos waypoints
    for (const auto & action : plan.value().items) {
      int waypoint_index = extract_waypoint_index(action.action);
      if (waypoint_index != -1) {
        waypoints_.push_back(waypoint_index);
      }
    }

    // Publicar os waypoints
    publish_waypoints();
  }

  // Função para extrair o índice do waypoint a partir da ação no plano
  int extract_waypoint_index(const std::string & action)
  {
    // Supondo que o plano contenha ações do tipo (move drone wp1 wp2)
    std::size_t wp_pos = action.find("wp");
    if (wp_pos != std::string::npos) {
      return std::stoi(action.substr(wp_pos + 2)); // Extrair o número do waypoint
    }
    return -1;
  }

  // Função para publicar os waypoints
  void publish_waypoints()
  {
    for (int waypoint_index : waypoints_) {
      std_msgs::msg::Int32 msg;
      msg.data = waypoint_index;

      RCLCPP_INFO(this->get_logger(), "Publicando waypoint: %d", waypoint_index);
      waypoint_publisher_->publish(msg);

      // Simular algum tempo entre cada waypoint (para teste)
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointNavigator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
};
