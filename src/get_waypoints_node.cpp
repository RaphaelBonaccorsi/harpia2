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
#include "std_msgs/msg/int32.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

// Classe para conversão de coordenadas
class CoordinateConverter {
public:
    static std::pair<double, double> geoToCart(const std::pair<double, double>& geo_point, const std::pair<double, double>& geo_home) {
        double x = calcX(geo_point.second, geo_home.second, geo_home.first); // Longitude
        double y = calcY(geo_point.first, geo_home.first);                   // Latitude
        return std::make_pair(x, y);
    }

private:
    static double calcY(double lat, double home_lat) {
        return (lat - home_lat) * 111320.0;
    }

    static double calcX(double longi, double home_long, double home_lat) {
        return (longi - home_long) * (111320.0 * std::cos(home_lat * M_PI / 180.0));
    }
};

// Classe para carregar e converter waypoints
class Map {
public:
    Map(double home_lat, double home_lon) : home_lat_(home_lat), home_lon_(home_lon) {}

    std::vector<std::pair<double, double>> readRouteFromJson(const std::string& map_file) {
        std::ifstream file(map_file);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open JSON file.");
        }

        // Simulação de leitura do JSON (deve usar uma biblioteca como nlohmann::json)
        std::vector<std::pair<double, double>> route;
        // Exemplo de dados fictícios (substitua pela lógica de leitura real do arquivo JSON)
        route.push_back(CoordinateConverter::geoToCart({-22.000, -47.930}, {home_lat_, home_lon_}));
        route.push_back(CoordinateConverter::geoToCart({-22.001, -47.932}, {home_lat_, home_lon_}));

        return route;
    }

private:
    double home_lat_;
    double home_lon_;
};

// Classe principal para controle do drone
class RouteExecutor : public rclcpp::Node {
public:
    RouteExecutor()
        : Node("route_executor"), home_lat_(-22.001333), home_lon_(-47.934152), home_alt_(0.0), current_waypoint_index_(-1) {
        
        // Inicializar objeto Map para manusear waypoints
        map_ = std::make_shared<Map>(home_lat_, home_lon_);

        // Inicializar publisher para enviar setpoints ao drone
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", 10);

        // Inicializar clientes para os serviços de armar o drone e definir modo
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

        // Esperar pela disponibilidade dos serviços
        RCLCPP_INFO(this->get_logger(), "Esperando pelos serviços de armar e definir modo...");
        arming_client_->wait_for_service(10s);
        set_mode_client_->wait_for_service(10s);

        // Assinatura para receber o índice do waypoint
        waypoint_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
    "/drone/waypoint_index", rclcpp::QoS(10), std::bind(&RouteExecutor::waypointCallback, this, _1));


        // Obter o diretório do pacote e definir o caminho para o arquivo JSON
        auto package_share = ament_index_cpp::get_package_share_directory("route_executor");
        std::string map_file = package_share + "/data/map.json";

        // Ler waypoints do arquivo JSON
        route_ = map_->readRouteFromJson(map_file);

        // Iniciar a publicação dos setpoints
        startPublishingSetpoints();

        // Definir modo Offboard e armar o drone
        std::this_thread::sleep_for(10s);
        setOffboardMode();
        arm();
    }

private:
    double home_lat_, home_lon_, home_alt_;
    std::vector<std::pair<double, double>> route_;
    int current_waypoint_index_;
    std::vector<std::pair<double, double>> waypoints_;
    std::pair<double, double> current_position_{0.0, 0.0}; // Posição inicial do drone

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr waypoint_subscriber_;

    std::shared_ptr<Map> map_;

    void startPublishingSetpoints() {
        this->create_wall_timer(50ms, std::bind(&RouteExecutor::publishCurrentSetpoint, this));
        RCLCPP_INFO(this->get_logger(), "Publicando setpoints a 20 Hz...");
    }

    void setOffboardMode() {
        RCLCPP_INFO(this->get_logger(), "Definindo modo OFFBOARD");
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = "OFFBOARD";
        auto result = set_mode_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) == rclcpp::FutureReturnCode::SUCCESS) {
            if (result.get()->mode_sent) {
                RCLCPP_INFO(this->get_logger(), "Modo Offboard definido com sucesso");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Falha ao definir modo Offboard");
            }
        }
    }

    void arm() {
        RCLCPP_INFO(this->get_logger(), "Armando o drone...");
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;
        auto result = arming_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) == rclcpp::FutureReturnCode::SUCCESS) {
            if (result.get()->success) {
                RCLCPP_INFO(this->get_logger(), "Drone armado com sucesso");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Falha ao armar o drone");
            }
        }
    }

    void publishCurrentSetpoint() {
        if (current_waypoint_index_ < 0 || static_cast<size_t>(current_waypoint_index_) >= waypoints_.size()) {
            return;
        }

        auto [x, y] = waypoints_[current_waypoint_index_];

        geometry_msgs::msg::PoseStamped msg;
        msg.pose.position.x = x;
        msg.pose.position.y = y;
        msg.pose.position.z = 10.0;
        msg.pose.orientation.w = 1.0;
        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Publicando: Movendo para (%f, %f, 10.0)", x, y);

        if (hasReachedWaypoint(x, y)) {
            RCLCPP_INFO(this->get_logger(), "Alcançou o waypoint (%f, %f)", x, y);
            if (current_waypoint_index_ < static_cast<int>(waypoints_.size()) - 1) {
                current_waypoint_index_++;
            } else {
                RCLCPP_INFO(this->get_logger(), "Todos os waypoints foram alcançados.");
            }
        }
    }

    bool hasReachedWaypoint(double x, double y, double threshold = 1.0) {
        double distance = std::sqrt(std::pow(x - current_position_.first, 2) + std::pow(y - current_position_.second, 2));
        return distance < threshold;
    }

    void waypointCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Índice do waypoint recebido: %d", msg->data);
        current_waypoint_index_ = msg->data;

        if (current_waypoint_index_ < static_cast<int>(route_.size())) {
            waypoints_.push_back(route_[current_waypoint_index_]);
            RCLCPP_INFO(this->get_logger(), "Waypoint %d armazenado: (%f, %f)", current_waypoint_index_, waypoints_.back().first, waypoints_.back().second);
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RouteExecutor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
