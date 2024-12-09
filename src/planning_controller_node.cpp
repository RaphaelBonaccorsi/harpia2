#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>

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
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>

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

    // // Carregar apenas o arquivo de problema PDDL
    // load_pddl_files(problem_file);


    add_mission_problem();
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

  const double EARTH_RADIUS_M = 6371.0*1000;
  double to_radians(double degree) {
    return degree * M_PI / 180.0;
  }
  double haversine(double lat1, double lon1, double lat2, double lon2) {
    // Converte graus para radianos
    lat1 = to_radians(lat1);
    lon1 = to_radians(lon1);
    lat2 = to_radians(lat2);
    lon2 = to_radians(lon2);

    // Diferenças das coordenadas
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    // Fórmula de Haversine
    double a = sin(dlat / 2) * sin(dlat / 2) + cos(lat1) * cos(lat2) * sin(dlon / 2) * sin(dlon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    // Distância em quilômetros
    return EARTH_RADIUS_M * c;
  }

  std::string get_package_path()
  {
    try
    {
      std::string package_path = ament_index_cpp::get_package_share_directory("route_executor2")+"/../../../../";
      return package_path;
    }
    catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Erro: %s", e.what());
      return "";
    }
  }

  nlohmann::json read_json(std::string json_path)
  {
    try {

      // Abrir o arquivo JSON
      std::ifstream json_file(json_path);
      if (!json_file.is_open()) {
          RCLCPP_ERROR(this->get_logger(), "Erro ao abrir o arquivo JSON.");
          return nlohmann::json{};
      }

      // Carregar o JSON
      nlohmann::json json_data;
      json_file >> json_data;
      json_file.close();
      return json_data;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Erro: %s", e.what());
      return nlohmann::json{};
    }
  }
  
  nlohmann::json region_list;
  void clear_region_list()
  {
    region_list = nlohmann::json::array();
  }
  void add_to_region_list(const nlohmann::json &region)
  {
    region_list.push_back(region);
  }

  void add_mission_problem()
  {
    int missao_index = 1;
    int mapa_index = 1;
    int hardware_index = 1;


    std::string package_path = get_package_path();
    if(package_path == "") return;


    nlohmann::json json_mapa = read_json(package_path+"json/mapa.json");

    if (!(json_mapa.is_array() && json_mapa.size() > mapa_index))
    {
      RCLCPP_ERROR(this->get_logger(), "Mapa %d não existe", mapa_index);
      return;
    }

    clear_region_list();

    const auto& mapa = json_mapa[mapa_index];
    for (const auto& base : mapa["bases"])
    {
      problem_client_->addInstance(plansys2::Instance(base["name"], "base"));
      add_to_region_list(base);
    }
    for (const auto& roi : mapa["roi"])
    {
      problem_client_->addInstance(plansys2::Instance(roi["name"], "region"));
      add_to_region_list(roi);
    }

    for (const auto& region1 : region_list)
    {
      for (const auto& region2 : region_list)
      { 
        if(region1["name"].get<std::string>() != region2["name"].get<std::string>())
        {
          std::string name1 = region1["name"].get<std::string>();
          std::string name2 = region2["name"].get<std::string>();
          double lat1 = region1["center"][0].get<double>();
          double lon1 = region1["center"][1].get<double>();
          double lat2 = region2["center"][0].get<double>();
          double lon2 = region2["center"][1].get<double>();
          std::string distance_str = std::to_string(haversine(lat1, lon1, lat2, lon2));
          // RCLCPP_INFO(this->get_logger(), "%20s ~ %20s: %s", name1.c_str(), name2.c_str(), distance_str.c_str());
          problem_client_->addFunction(plansys2::Function("(= (distance "+name1+" "+name2+") "+distance_str+")"));
          problem_client_->addFunction(plansys2::Function("(= (distance "+name2+" "+name1+") "+distance_str+")"));
        }
      }
    }



    nlohmann::json json_missao = read_json(package_path+"json/missao.json");
    if(json_missao.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "JSON está vazio");
      return;
    }
    if (!(json_missao.is_array() && json_missao.size() > missao_index))
    {
      RCLCPP_ERROR(this->get_logger(), "Missão %d não existe", missao_index);
      return;
    }

    const auto& mission = json_missao[missao_index];

    // Iterando sobre "mission_execution" do segundo objeto
    if (!(mission.contains("mission_execution") && mission["mission_execution"].is_array()))
    {
      RCLCPP_ERROR(this->get_logger(), "\"mission_execution\" não encontrado ou não é um array.");
      return;
    }

    std::string goal_str = "(and";
    for (const auto& mission : mission["mission_execution"])
    {
      std::string command = mission["command"];
      std::string area = mission["instructions"]["area"];

      // RCLCPP_INFO(this->get_logger(), "Comando: %s, Área: %s", command.c_str(), area.c_str());
      if(command == "take_picture")
      {
        problem_client_->addPredicate(plansys2::Predicate("(picture_goal "+area+")"));
        goal_str += " (taken_image "+area+")";
      }
      else if(command == "end")
      {
        goal_str += " (at "+area+")";
      }
      else {
        RCLCPP_WARN(this->get_logger(), "Comando '%s' desconhecido", command.c_str());
      }
    }

    nlohmann::json json_hardware = read_json(package_path+"json/hardware.json");
    if(json_hardware.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "JSON está vazio");
      return;
    }
    if (!(json_hardware.is_array() && json_hardware.size() > hardware_index))
    {
      RCLCPP_ERROR(this->get_logger(), "Missão %d não existe", hardware_index);
      return;
    }

    const auto& hardware = json_hardware[hardware_index];

    float discharge_rate_battery = hardware["discharge-rate-battery"].get<float>();
    float recharge_rate_battery = hardware["recharge-rate-battery"].get<float>();
    float battery_capacity = hardware["battery-capacity"].get<float>();
    float efficient_velocity = hardware["efficient_velocity"].get<float>();
    float input_capacity = hardware["input-capacity"].get<float>();
    discharge_rate_battery = 0.1;

    problem_client_->addFunction(plansys2::Function("(= (battery_capacity) "+std::to_string(battery_capacity)+")"));
    problem_client_->addFunction(plansys2::Function("(= (discharge_rate_battery) "+std::to_string(discharge_rate_battery)+")"));
    problem_client_->addFunction(plansys2::Function("(= (velocity) "+std::to_string(efficient_velocity)+")"));
    problem_client_->addFunction(plansys2::Function("(= (input_capacity) "+std::to_string(input_capacity)+")"));

    problem_client_->addFunction(plansys2::Function("(= (battery_amount) "+std::to_string(battery_capacity)+")"));
    problem_client_->addFunction(plansys2::Function("(= (input_amount) "+std::to_string(input_capacity)+")"));
    problem_client_->addPredicate(plansys2::Predicate("(at base_2)"));
    
    problem_client_->addFunction(plansys2::Function("(= (mission_length) 0.0)")); 

    goal_str += ")";
    problem_client_->setGoal(plansys2::Goal(goal_str));
  }

  // Função para carregar o arquivo de problema PDDL
  void load_pddl_files(const std::string & problem_file)
  {

    std::string package_directory = ament_index_cpp::get_package_share_directory("route_executor2");
    std::string problem_file2 = package_directory + problem_file;
    RCLCPP_INFO(this->get_logger(), "caminho para pddl: %s", problem_file2.c_str());
    // Carregar o problema
    std::ifstream problem_stream(problem_file2);
    if (!problem_stream.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Erro ao abrir o arquivo de problema PDDL.");
      return;
    }

    std::string problem_content((std::istreambuf_iterator<char>(problem_stream)),
                                 std::istreambuf_iterator<char>());

    // Enviar o problema ao PlanSys2
    if (!problem_client_->addProblem(problem_content)) {
      RCLCPP_ERROR(this->get_logger(), "Erro ao carregar o problema PDDL: '%s'", problem_file2.c_str());
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
    ////////////////////////

    // // Assuming `plan` is a valid std::optional<plansys2_msgs::msg::Plan>
    // auto &plan_items = plan.value().items;

    // // Log the initial plan
    // RCLCPP_INFO(this->get_logger(), "Initial Plan:");
    // for (const auto &action : plan_items) {
    //   RCLCPP_INFO(this->get_logger(), "Action: %s, Start Time: %.2f", action.action.c_str(), action.time);
    // }

    // auto logger = this->get_logger()

    // // Example: Remove an action by some condition (e.g., action name matches "some_action")
    // plan_items.erase(
    //   std::remove_if(plan_items.begin(), plan_items.end(),
    //                 [](const auto &action) {
    //     bool result = action.action == "take_image";
    //     RCLCPP_INFO(logger, "Comparing (%d): %s", result, action.action.c_str());
    //     return result;
    //                   }),
    //   plan_items.end());

    // // Log the modified plan
    // RCLCPP_INFO(this->get_logger(), "Modified Plan:");
    // for (const auto &action : plan_items) {
    //   RCLCPP_INFO(this->get_logger(), "Action: %s, Start Time: %.2f", action.action.c_str(), action.time);
    // }

    /////////////////////

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
  auto node = std::make_shared<InterfacePlansys2>("/pddl/problem.pddl");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}