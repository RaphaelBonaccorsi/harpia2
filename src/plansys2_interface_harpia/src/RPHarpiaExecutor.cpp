#include "rosplan_interface_harpia/RPHarpiaExecutor.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "mavros_msgs/msg/waypoint_list.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/srv/waypoint_push.hpp"
#include "mavros_msgs/msg/waypoint.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/extended_state.hpp"
#include "mavros_msgs/srv/waypoint_clear.hpp"
#include "mavros_msgs/srv/waypoint_set_current.hpp"
#include "interfaces/msg/mission.hpp"
#include "interfaces/msg/change_mission.hpp"
#include "interfaces/msg/uav.hpp"
#include "interfaces/msg/map.hpp"
#include "interfaces/msg/goal.hpp"
#include "interfaces/msg/region_point.hpp"
#include "interfaces/action/mission_planner.hpp"
#include "interfaces/srv/mission_fault_mitigation.hpp"
#include "interfaces/srv/path_planning.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/msg/goal_status.hpp"
#include <rclcpp/rclcpp.hpp>
#include <plansys2_executor/ActionExecutorClient.hpp>
#include "mavros_msgs/msg/waypoint_reached.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include <plansys2_msgs/msg/action_execution.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/executor.hpp"

#include <chrono>
#include <thread>

#include <signal.h>
#include <math.h>

#include <fstream>
#include <iomanip>

#include <cstdlib>
using namespace std;
std::string homepath = getenv("HOME");
rclcpp::Node::SharedPtr node;
//
// change GeoPoint to geographic_msgs/geopoint
struct GeoPoint{
	string  name;
	double longitude;
	double latitude;
	double altitude;
};

class Drone
{
public:
    geographic_msgs::msg::GeoPoint position;
    mavros_msgs::msg::State current_state;
    mavros_msgs::msg::ExtendedState ex_current_state;

    void chatterCallback_GPS(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void chatterCallback_currentState(const mavros_msgs::msg::State::SharedPtr msg);
    void chatterCallback_currentStateExtended(const mavros_msgs::msg::ExtendedState::SharedPtr msg);
};

void Drone::chatterCallback_GPS(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    position.longitude = msg->longitude;
    position.latitude = msg->latitude;
    position.altitude = msg->altitude;
}


void Drone::chatterCallback_currentState(const mavros_msgs::msg::State::SharedPtr msg)
{
    current_state = *msg;
}

void Drone::chatterCallback_currentStateExtended(const mavros_msgs::msg::ExtendedState::SharedPtr msg)
{
    ex_current_state = *msg;
}




// detach route from mission
class Mission
{
public:
    int WPqtd;
    int currentWP;
    int IDGoal;
    bool Ended = true;
    bool Cancelled = false;
    interfaces::msg::Mission hMission;

    void send_mission();
    void chatterCallback_wpqtd(const mavros_msgs::msg::WaypointList::SharedPtr msg);
    void chatterCallback_current(const mavros_msgs::msg::WaypointReached::SharedPtr msg);
    void chatterCallback_harpiaMission(const interfaces::msg::Mission::SharedPtr msg);
    void chatterCallback_IDGoal(const interfaces::action::MissionPlanner::Goal::SharedPtr msg);
    void chatterCallback_cancelGoal(const interfaces::msg::ChangeMission::SharedPtr msg);


    Mission();
};

Mission::Mission() { Ended = true; }

void Mission::chatterCallback_wpqtd(const mavros_msgs::msg::WaypointList::SharedPtr msg)
{
    WPqtd = msg->waypoints.size() - 1;
}

void Mission::chatterCallback_IDGoal(const interfaces::action::MissionPlanner::Goal::SharedPtr msg)
{
    if (!msg->mission.goals.empty()) {
        IDGoal = std::stoi(msg->mission.goals[0].region);
        std::string action = msg->mission.goals[0].action;

        // Faça algo com `action` e `region` se necessário
    }
}




void Mission::chatterCallback_current(const mavros_msgs::msg::WaypointReached::SharedPtr msg)
{
    if (currentWP != msg->wp_seq)
    {
        currentWP = msg->wp_seq;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waypoint: %i", msg->wp_seq + 1);
    }
    Ended = (WPqtd == msg->wp_seq);
}

void Mission::chatterCallback_cancelGoal(const interfaces::msg::ChangeMission::SharedPtr msg)
{
    Cancelled = (msg->op != 0);
}

void Mission::chatterCallback_harpiaMission(const interfaces::msg::Mission::SharedPtr msg)
{
    hMission.uav = msg->uav;
    hMission.map = msg->map;
    hMission.goals = msg->goals;
}


Mission mission;
Drone drone;


void land(const std::shared_ptr<Drone> &drone, const rclcpp::Node::SharedPtr &node)
{
    auto land_client = node->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");
    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    request->altitude = 0;
    request->latitude = drone->position.latitude;
    request->longitude = drone->position.longitude;
    request->min_pitch = 0;
    request->yaw = 0;

    if (land_client->wait_for_service(std::chrono::seconds(10)))
    {
        auto result = land_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(node->get_logger(), "srv_land send ok %d", result.get()->success);
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Failed Land");
        }
    }
}



void set_loiter(const rclcpp::Node::SharedPtr &node)
{
    // Cria um cliente para o serviço /mavros/set_mode
    auto client = node->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    
    // Cria um pedido para o serviço
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->base_mode = 0;
    request->custom_mode = "AUTO.LOITER";
    
    // Espera até o cliente estar disponível
    if (!client->wait_for_service(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(node->get_logger(), "Service /mavros/set_mode not available");
        return;
    }
    
    // Cria um pedido de serviço
    auto future = client->async_send_request(request);

    try
    {
        // Espera pelo resultado do serviço
        auto result = future.get();
        if (result->mode_sent)
        {
            RCLCPP_INFO(node->get_logger(), "LOITER mode set successfully");
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to set LOITER mode");
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Service call failed: %s", e.what());
    }
}



void set_auto(const rclcpp::Node::SharedPtr &node)
{
    // Cria um cliente para o serviço /mavros/set_mode
    auto client = node->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    
    // Cria um pedido para o serviço
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->base_mode = 0;
    request->custom_mode = "AUTO.MISSION";
    
    // Espera até o cliente estar disponível
    if (!client->wait_for_service(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(node->get_logger(), "Service /mavros/set_mode not available");
        return;
    }
    
    // Cria um futuro para a chamada de serviço
    auto future = client->async_send_request(request);

    try
    {
        // Espera pelo resultado do serviço
        auto result = future.get();
        if (result->mode_sent)
        {
            RCLCPP_INFO(node->get_logger(), "AUTO mode set successfully");
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to set AUTO mode");
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Service call failed: %s", e.what());
    }
}



void arm(rclcpp::Node::SharedPtr &node)
{
    auto arming_client = node->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = true;

    if (arming_client->wait_for_service(std::chrono::seconds(10)))
    {
        auto result = arming_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(node->get_logger(), "ARM send ok %d", result.get()->success);
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Failed arming or disarming");
        }
    }
}


void takeoff(const std::shared_ptr<rclcpp::Node> &node, const Drone &drone)
{
    // Cria um cliente para o serviço /mavros/cmd/takeoff
    auto takeoff_client = node->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    
    // Cria um pedido para o serviço
    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    request->altitude = 15;
    request->latitude = drone.position.latitude;
    request->longitude = drone.position.longitude;
    request->min_pitch = 0;
    request->yaw = 0;
    
    // Espera até o cliente estar disponível
    if (!takeoff_client->wait_for_service(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(node->get_logger(), "Service /mavros/cmd/takeoff not available");
        return;
    }
    
    // Cria um futuro para a chamada de serviço
    auto future = takeoff_client->async_send_request(request);

    try
    {
        // Espera pelo resultado do serviço
        auto result = future.get();
        if (result->success)
        {
            RCLCPP_INFO(node->get_logger(), "srv_takeoff send ok %d", result->success);
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Failed Takeoff");
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Service call failed: %s", e.what());
    }
}


interfaces::msg::RegionPoint getGeoPoint(const GeoPoint &geo, const interfaces::msg::Map &mapa, const rclcpp::Logger &logger)
{
    int qtd_regions = mapa.roi.size();
    
    // Procurando nas regiões de interesse (ROI)
    for (int i = 0; i < qtd_regions; ++i)
    {
        if (mapa.roi[i].name == geo.name)
        {
            RCLCPP_INFO(logger, "%s", mapa.roi[i].name.c_str());
            return mapa.roi[i].center;
        }
    }
    
    // Procurando nas bases
    qtd_regions = mapa.bases.size();
    for (int i = 0; i < qtd_regions; ++i)
    {
        if (mapa.bases[i].name == geo.name)
        {
            RCLCPP_INFO(logger, "%s", mapa.bases[i].name.c_str());
            return mapa.bases[i].center;
        }
    }
    
    // Retorno padrão em caso de não encontrar
    interfaces::msg::RegionPoint null;
    return null;
}
/*
int getRadius(string region) // parte do código nao elterada
{
	string command = "python3 ~/drone_arch/drone_ws/src/ROSPlan/src/rosplan/rosplan_planning_system/src/ActionInterface/getRadius.py "+region+" >> ~/drone_arch/Data/out.txt";
	system(command.c_str());
	// cout << result;
	string line;
  	ifstream myfile ((homepath + "/drone_arch/Data/out.txt").c_str());
  	if (myfile.is_open())
 	{
 		cout << "file opened" << endl;
 		getline (myfile,line);
 		int radius = stod(line);
    	myfile.clear();
    	myfile.close();

    	return radius;
  	}
  	else
  		cout << "Unable to open file";
  		return 10.0;
}*/

mavros_msgs::msg::WaypointList calcRoute(
    const interfaces::msg::RegionPoint &from, 
    const interfaces::msg::RegionPoint &to,
    const std::string &name_from, 
    const std::string &name_to,
    const interfaces::msg::Map &map,
    const rclcpp::Node::SharedPtr &node)
{
    auto client = node->create_client<interfaces::srv::PathPlanning>("harpia/path_planning");

    auto request = std::make_shared<interfaces::srv::PathPlanning::Request>();
    request->r_from = from;
    request->r_to = to;
    request->name_from = name_from;
    request->name_to = name_to;
    request->op = 0;
    request->map = map;

    auto future_result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, future_result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future_result.get();
        return response->waypoints;
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service harpia/path_planning");
        mavros_msgs::msg::WaypointList null;
        return null;
    }
}

mavros_msgs::msg::WaypointList calcRoute_pulverize(
    const interfaces::msg::RegionPoint &at,
    const interfaces::msg::Map &map,
    const rclcpp::Node::SharedPtr &node)
{
    auto client = node->create_client<interfaces::srv::PathPlanning>("harpia/path_planning");

    auto request = std::make_shared<interfaces::srv::PathPlanning::Request>();
    request->r_from = at;
    request->r_to = at;
    request->op = 1;
    request->name_from = "at";
    request->name_to = "pulverize_region";
    request->map = map;

    auto future_result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, future_result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future_result.get();
        return response->waypoints;
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service harpia/path_planning");
        mavros_msgs::msg::WaypointList null;
        return null;
    }
}

 mavros_msgs::msg::WaypointList calcRoute_picture(
    const interfaces::msg::RegionPoint &at,
    const interfaces::msg::Map &map,
    const rclcpp::Node::SharedPtr &node)
{
    auto client = node->create_client<interfaces::srv::PathPlanning>("harpia/path_planning");

    auto request = std::make_shared<interfaces::srv::PathPlanning::Request>();
    request->r_from = at;
    request->r_to = at;
    request->op = 2;
    request->name_from = "at";
    request->name_to = "take_picture";
    request->map = map;

    auto future_result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, future_result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future_result.get();
        return response->waypoints;
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service harpia/path_planning");
        mavros_msgs::msg::WaypointList null;
        return null;
    }
}


int sendWPFile(
    const mavros_msgs::msg::WaypointList &mission_wp,
    const rclcpp::Node::SharedPtr &node)
{
    auto wp_push_client = node->create_client<mavros_msgs::srv::WaypointPush>("mavros/mission/push");
    auto wp_clear_client = node->create_client<mavros_msgs::srv::WaypointClear>("mavros/mission/clear");

    auto wp_clear_request = std::make_shared<mavros_msgs::srv::WaypointClear::Request>();
    auto wp_push_request = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();

    // Clear existing waypoints
    if (wp_clear_client->wait_for_service(std::chrono::seconds(10)))
    {
        auto wp_clear_future = wp_clear_client->async_send_request(wp_clear_request);
        if (rclcpp::spin_until_future_complete(node, wp_clear_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(node->get_logger(), "Waypoint list was cleared");
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Waypoint list couldn't be cleared");
            return 0;
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Service 'mavros/mission/clear' not available");
        return 0;
    }

    // Prepare and send new waypoints
    wp_push_request->start_index = 0;
    wp_push_request->waypoints = mission_wp.waypoints;

    if (wp_push_client->wait_for_service(std::chrono::seconds(10)))
    {
        auto wp_push_future = wp_push_client->async_send_request(wp_push_request);
        if (rclcpp::spin_until_future_complete(node, wp_push_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto wp_push_response = wp_push_future.get();
            RCLCPP_INFO(node->get_logger(), "Success: %d", wp_push_response->success);
            // Assume 'homepath' is defined elsewhere or passed to the function
            // remove((homepath + "/drone_arch/Data/route.txt").c_str());
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Waypoint couldn't be sent");
            // Assume 'homepath' is defined elsewhere or passed to the function
            // remove((homepath + "/drone_arch/Data/route.txt").c_str());
            return 0;
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Service 'mavros/mission/push' not available");
        return 0;
    }

    return 1;
}

 void reset_mission(const rclcpp::Node::SharedPtr &node)
{
    auto set_current_client = node->create_client<mavros_msgs::srv::WaypointSetCurrent>("mavros/mission/set_current");

    auto set_current_request = std::make_shared<mavros_msgs::srv::WaypointSetCurrent::Request>();
    set_current_request->wp_seq = 0;

    if (set_current_client->wait_for_service(std::chrono::seconds(10)))
    {
        auto future_result = set_current_client->async_send_request(set_current_request);
        if (rclcpp::spin_until_future_complete(node, future_result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future_result.get();
            RCLCPP_INFO(node->get_logger(), "Reset Mission");
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Reset couldn't be done");
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Service 'mavros/mission/set_current' not available");
    }
}


void callRoute(const std::string &from_name, const std::string &to_name)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending WP file for route %s _ %s.wp", from_name.c_str(), to_name.c_str());
    
    std::string command = "ros2 run mavros mavwp load ~/drone_arch/Data/Rotas/wp/" + from_name + "_" + to_name + ".wp";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", command.c_str());
    
    int result = std::system(command.c_str());
    if (result != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to execute command");
    }
}


geometry_msgs::msg::Point convert_goe_to_cart(geographic_msgs::msg::GeoPoint p, geographic_msgs::msg::GeoPoint home)
{
    geometry_msgs::msg::Point point;
    double pi = 2 * acos(0.0);
    point.x = (p.longitude - home.longitude) * (6400000.0 * (cos(home.latitude * pi / 180) * 2 * pi / 360));
    point.y = (p.latitude - home.latitude) * (10000000.0 / 90);

    return point;
}

// Função para converter de interfaces::msg::GeoPoint para geographic_msgs::msg::GeoPoint
geographic_msgs::msg::GeoPoint convert_to_geographic(const interfaces::msg::GeoPoint& geo_point)
{
    geographic_msgs::msg::GeoPoint geo;
    geo.latitude = geo_point.latitude;
    geo.longitude = geo_point.longitude;
    geo.altitude = geo_point.altitude;
    return geo;
}

interfaces::msg::Point convert_to_interfaces_point(const geometry_msgs::msg::Point& geo_point)
{
    interfaces::msg::Point point;
    point.x = geo_point.x;
    point.y = geo_point.y;
    point.z = geo_point.z;
    return point;
}

// Função para criar RegionPoint
interfaces::msg::RegionPoint create_RegionPoint(const geographic_msgs::msg::GeoPoint& geo, const interfaces::msg::Map& map)
{
    interfaces::msg::RegionPoint region_point;

    // Definindo a região com valores diretamente
    region_point.geo.latitude = geo.latitude;
    region_point.geo.longitude = geo.longitude;
    region_point.geo.altitude = geo.altitude;

    // Converter map.geo_home para o tipo correto
    geographic_msgs::msg::GeoPoint geo_home_converted = convert_to_geographic(map.geo_home);

    // Usar a conversão correta
    geometry_msgs::msg::Point cartesian_point = convert_goe_to_cart(geo, geo_home_converted);

    // Converter o resultado para o tipo esperado
    region_point.cartesian = convert_to_interfaces_point(cartesian_point);

    return region_point;
}


/*--------------------------------------------*/
struct SignalHandlerParams
{
    rclcpp::Node::SharedPtr node;
};

SignalHandlerParams params;

void mySigintHandler(int)
{
    // Usar params.node que foi inicializado anteriormente
    if (params.node) {
        set_loiter(params.node);
    }
    rclcpp::shutdown();
}

/*--------------------------------------------*/
/*--------------------------------------------*/
namespace plansys2
{

    RPHarpiaExecutor::RPHarpiaExecutor()
       : plansys2::ActionExecutorClient("rpharpia_executor", std::chrono::seconds(1))
    {
        mission_fault_client_ = this->create_client<interfaces::srv::MissionFaultMitigation>("harpia/mission_fault_mitigation");
        waypoint_push_client_ = this->create_client<mavros_msgs::srv::WaypointPush>("mavros/mission/push");
        waypoint_clear_client_ = this->create_client<mavros_msgs::srv::WaypointClear>("mavros/mission/clear");

        // Declaração de parâmetros
        this->declare_parameter<double>("action_duration", 2.0);

        // Inicialização do feedback
        auto feedback = std::make_shared<plansys2_msgs::msg::ActionExecutionFeedback>();
        feedback->progress = 0.0;
        send_feedback(feedback);
    }

    void RPHarpiaExecutor::do_work(){
        auto msg = get_goal();

        auto client = mission_fault_client_;
        auto request = std::make_shared<interfaces::srv::MissionFaultMitigation::Request>();
        // Configure a solicitação do serviço usando msg->parameters, conforme necessário

        if (client->wait_for_service(std::chrono::seconds(10)))
        {
            auto result = client->async_send_request(request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = result.get();
                int replan = response->replan;

                if (replan != 1)
                {
                    // Implementação da ação
                    std::string str = msg->action;
                    std::string str1 = "go_to";
                    auto found = str.find(str1);
                    RCLCPP_INFO(this->get_logger(), "%s", msg->action.c_str());

                    if (found != std::string::npos)
                    {
                        // Implementar a lógica da ação
                        mission.Ended = false;
                        GeoPoint from, to;
                        interfaces::msg::RegionPoint r_from, r_to;
                        mavros_msgs::msg::WaypointList route;

                        // Obter coordenadas
                        from.name = msg->arguments[0];
                        to.name = msg->arguments[1];
                        RCLCPP_INFO(this->get_logger(), "go_to %s -> %s", from.name.c_str(), to.name.c_str());

                        from.latitude = drone.position.latitude;
                        from.longitude = drone.position.longitude;
                        from.altitude = 15;

                        r_from = create_RegionPoint(from, mission.hMission.map);
                        r_to = getGeoPoint(to, mission.hMission.map);

                        RCLCPP_INFO(this->get_logger(), "GEO GeoPoint %f %f %f -> %f %f %f", r_from.geo.latitude, r_from.geo.longitude, r_from.geo.altitude, r_to.geo.latitude, r_to.geo.longitude, r_to.geo.altitude);

                        // Calcular rota
                        route = calcRoute(r_from, r_to, from.name, to.name, mission.hMission.map);

                        // Verificar se está voando
                        while (!drone.current_state.armed && drone.ex_current_state.landed_state != 2)
                        {
                            set_loiter();
                            arm();
                            takeoff(drone);
                        }
                        std::this_thread::sleep_for(std::chrono::seconds(10));

                        // Enviar rota
                        if (!sendWPFile(route))
                            callRoute(from, to);
                        std::this_thread::sleep_for(std::chrono::seconds(20));

                        set_auto();

                        while (!mission.Ended)
                        {
                            std::this_thread::sleep_for(std::chrono::seconds(10));
                        }

                        set_loiter();
                        std::this_thread::sleep_for(std::chrono::seconds(10));

                        RCLCPP_INFO(this->get_logger(), "KCL: (%s) HarpiaExecutor Action completing.", msg->action.c_str());
                    }
                    // Continue com outros casos de `msg->action` como "pulverize_region", "take_image", etc.
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "NEED TO REPLAN");
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to call service harpia/mission_fault_mitigation");
            }
        }

        finish(true, 1.0, "Action completed successfully");
    }

} // namespace plansys2

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    params.node = std::make_shared<rclcpp::Node>("my_node");

    // Configurar o manipulador de sinal
    signal(SIGINT, mySigintHandler);

    rclcpp::spin(params.node);


    // Create the main node
    auto node = std::make_shared<rclcpp::Node>("plansys2_interface_harpia");

    // Create instances of Drone and Mission
    auto drone = std::make_shared<Drone>();
    auto mission = std::make_shared<Mission>();

    // Create subscriptions
    auto gps_sub = node->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/mavros/global_position/global", 1,
        [drone](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
            drone->chatterCallback_GPS(msg);
        });

    auto state_sub = node->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", 1,
        [drone](const mavros_msgs::msg::State::SharedPtr msg) {
            drone->chatterCallback_currentState(msg);
        });

    auto state_ext_sub = node->create_subscription<mavros_msgs::msg::ExtendedState>(
        "/mavros/extended_state", 1,
        [drone](const mavros_msgs::msg::ExtendedState::SharedPtr msg) {
            drone->chatterCallback_currentStateExtended(msg);
        });

    // Spin the node
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}