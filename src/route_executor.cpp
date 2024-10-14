#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "std_msgs/msg/int32.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class RouteExecutor : public rclcpp::Node {
public:
    RouteExecutor()
        : Node("route_executor"), home_lat_(-22.001333), home_lon_(-47.934152), home_alt_(0.0), current_waypoint_index_(-1), first_waypoint_received_(false), waypoint_threshold_(1.0) {
        
        // Inicializar publisher para enviar setpoints ao drone
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", 10);

        // Inicializar clientes para os serviços de armar o drone e definir modo
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

        // Assinatura para receber o índice do waypoint
        waypoint_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            "/drone/waypoint_index", rclcpp::QoS(10), std::bind(&RouteExecutor::waypointCallback, this, _1));

        // Assinatura para monitorar o estado do drone
        state_subscriber_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", rclcpp::QoS(10), std::bind(&RouteExecutor::stateCallback, this, _1));

        // Timer para publicação constante de setpoints a 20Hz
        setpoint_timer_ = this->create_wall_timer(50ms, std::bind(&RouteExecutor::publishCurrentSetpoint, this));

        // Timer para tentar armar o drone a cada 5 segundos, caso ele não esteja armado
        arm_timer_ = this->create_wall_timer(5s, std::bind(&RouteExecutor::arm, this));

        // Definir modo Offboard após um pequeno atraso para garantir que os setpoints estão sendo publicados
        std::this_thread::sleep_for(2s);
        setOffboardMode();
    }

private:
    double home_lat_, home_lon_, home_alt_;
    int current_waypoint_index_;
    std::pair<double, double> current_position_{0.0, 0.0}; // Posição inicial do drone
    bool first_waypoint_received_;  // Variável para verificar se o primeiro waypoint foi recebido
    mavros_msgs::msg::State current_state_;  // Variável para armazenar o estado atual do drone
    double waypoint_threshold_;  // Distância mínima para considerar que o waypoint foi atingido

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr waypoint_subscriber_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscriber_;
    rclcpp::TimerBase::SharedPtr setpoint_timer_;
    rclcpp::TimerBase::SharedPtr arm_timer_;

    std::vector<std::pair<double, double>> waypoints_;  // Lista de waypoints
    geometry_msgs::msg::PoseStamped current_setpoint_;  // Setpoint atual publicado

    void setOffboardMode() {
        if (current_state_.mode != "OFFBOARD") {
            RCLCPP_INFO(this->get_logger(), "Definindo modo OFFBOARD");
            auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            request->custom_mode = "OFFBOARD";

            // Enviar a requisição para definir o modo OFFBOARD
            auto result = set_mode_client_->async_send_request(request);
            
            if (result.valid()) {

                if (result.get()->mode_sent) {
                    RCLCPP_INFO(this->get_logger(), "Modo Offboard definido com sucesso");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Falha ao definir modo Offboard");
                }
            }
        }
    }

    void arm() {
        if (!current_state_.armed) {
            RCLCPP_INFO(this->get_logger(), "Tentando armar o drone...");
            auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            request->value = true;

            // Enviar a requisição para armar o drone
            auto result = arming_client_->async_send_request(request);

            if (result.valid()) {
                if (result.get()->success) {
                    RCLCPP_INFO(this->get_logger(), "Drone armado com sucesso");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Falha ao armar o drone");
                }
            }
        }
    }

    void publishCurrentSetpoint() {
        if (current_waypoint_index_ <= 0 || static_cast<size_t>(current_waypoint_index_) >= waypoints_.size()) {
            // Se nenhum waypoint for recebido, publicar em (0, 0, 10)
            current_setpoint_.pose.position.x = 0.0;
            current_setpoint_.pose.position.y = 0.0;
            current_setpoint_.pose.position.z = 10.0;
            current_setpoint_.pose.orientation.w = 1.0;
            RCLCPP_INFO_ONCE(this->get_logger(), "Publicando setpoint inicial em (0, 0, 10) até receber waypoints.");
        } else {
            // Publicar os waypoints reais
            auto [x, y] = waypoints_[current_waypoint_index_];
            current_setpoint_.pose.position.x = x;
            current_setpoint_.pose.position.y = y;
            current_setpoint_.pose.position.z = 10.0;
            current_setpoint_.pose.orientation.w = 1.0;

            RCLCPP_INFO(this->get_logger(), "Publicando: Movendo para (%f, %f, 10.0)", x, y);

            // Verificar se o drone alcançou o waypoint
            if (hasReachedWaypoint(x, y)) {
                RCLCPP_INFO(this->get_logger(), "Alcançou o waypoint (%f, %f)", x, y);
                if (current_waypoint_index_ < static_cast<int>(waypoints_.size()) - 1) {
                    current_waypoint_index_++;
                } else {
                    RCLCPP_INFO(this->get_logger(), "Todos os waypoints foram alcançados.");
                }
            }
        }

        publisher_->publish(current_setpoint_);
    }

    bool hasReachedWaypoint(double x, double y) {
        double distance = std::sqrt(std::pow(x - current_position_.first, 2) + std::pow(y - current_position_.second, 2));
        return distance < waypoint_threshold_;
    }

    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg) {
        current_state_ = *msg;  // Atualiza o estado do drone
        RCLCPP_INFO(this->get_logger(), "Estado atual: %s, Armado: %d", current_state_.mode.c_str(), current_state_.armed);
    }

    void waypointCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Índice do waypoint recebido: %d", msg->data);
        current_waypoint_index_ = msg->data;
        first_waypoint_received_ = true;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RouteExecutor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
