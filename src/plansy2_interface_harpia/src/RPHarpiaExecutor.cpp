//#include "rosplan_interface_harpia/RPHarpiaExecutor.hpp"
#include "rclcpp/rclcpp.hpp"
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
#include "interfaces/srv/change_mission.hpp"
#include "interfaces/msg/uav.hpp"
#include "interfaces/msg/map.hpp"
#include "interfaces/msg/goal.hpp"
#include "interfaces/msg/region_point.hpp"
#include "interfaces/action/mission_planner.hpp"
#include "interfaces/srv/mission_fault_mitigation.hpp"
#include "interfaces/srv/path_planning.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/msg/goal_status.hpp"


#include <signal.h>
#include<math.h>

#include <fstream>
#include<iomanip>

#include <cstdlib>
using namespace std;
std::string homepath = getenv("HOME");
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
    GeoPoint position;
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
    void chatterCallback_cancelGoal(const interfaces::srv::ChangeMission::Request::SharedPtr msg);

    Mission();
};

Mission::Mission() { Ended = true; }

void Mission::chatterCallback_wpqtd(const mavros_msgs::msg::WaypointList::SharedPtr msg)
{
    WPqtd = msg->waypoints.size() - 1;
}

void Mission::chatterCallback_IDGoal(const interfaces::action::MissionPlanner::Goal::SharedPtr msg)
{
    IDGoal = std::stoi(msg->goal_id.id);
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

void Mission::chatterCallback_cancelGoal(const interfaces::srv::ChangeMission::Request::SharedPtr msg)
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


void land(std::shared_ptr<Drone> drone, rclcpp::Node::SharedPtr node)
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
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::executor::FutureReturnCode::SUCCESS)
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



void arm(rclcpp::Node::SharedPtr node)
{
    auto arming_client = node->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = true;

    if (arming_client->wait_for_service(std::chrono::seconds(10)))
    {
        auto result = arming_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::executor::FutureReturnCode::SUCCESS)
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
}

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


geometry_msgs::Point convert_goe_to_cart(geographic_msgs::GeoPoint p, geographic_msgs::GeoPoint home)
{
	geometry_msgs::Point point;
	double pi = 2*acos(0.0);
	point.x = (p.longitude - home.longitude) * (6400000.0 * (cos(home.latitude * pi / 180) * 2 * pi / 360));
	point.y =(p.latitude - home.latitude)  * (10000000.0 / 90);

	return point;
}

interfaces::RegionPoint create_RegionPoint(GeoPoint point, interfaces::Map map)
{
	interfaces::RegionPoint region_point;

	region_point.geo.latitude = point.latitude;
	region_point.geo.longitude = point.longitude;
	region_point.geo.altitude = point.altitude;

	region_point.cartesian = convert_goe_to_cart(region_point.geo, map.geo_home);


	return region_point;
}

/*--------------------------------------------*/
void mySigintHandler(int sig)
{
	// Do some custom action.
	set_loiter();
	// All the default sigint handler does is call shutdown()
	rclcpp::shutdown();
}
/*--------------------------------------------*/
/*--------------------------------------------*/


namespace plansy2 { //veroficar essa parte Karine

class RPHarpiaExecutor : public rclcpp::Node
{
public:
    RPHarpiaExecutor()
    : Node("rpharpia_executor")
    {
        // Inicialize os serviços e assinaturas aqui
        action_dispatch_sub_ = this->create_subscription<rosplan_dispatch_msgs::msg::ActionDispatch>(
            "action_dispatch", 10,
            std::bind(&RPHarpiaExecutor::concreteCallback, this, std::placeholders::_1)
        );

        mission_fault_client_ = this->create_client<interfaces::srv::MissionFaultMitigation>("harpia/mission_fault_mitigation");
        waypoint_push_client_ = this->create_client<mavros_msgs::srv::WaypointPush>("mavros/mission/push");
        waypoint_clear_client_ = this->create_client<mavros_msgs::srv::WaypointClear>("mavros/mission/clear");
    }

private:
    void concreteCallback(const rosplan_dispatch_msgs::msg::ActionDispatch::SharedPtr msg)
    {
        auto client = mission_fault_client_;
        auto request = std::make_shared<interfaces::srv::MissionFaultMitigation::Request>();
        request->uav = mission.hMission.uav;
        request->action_id = msg->action_id;

        if (client->wait_for_service(std::chrono::seconds(10))) {
            auto result = client->async_send_request(request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
                auto response = result.get();
                int replan = response->replan;

                if (replan != 1) {
                    // Action implementation
                    std::string str = msg->name;
                    std::string str1 = "go_to";
                    auto found = str.find(str1);
                    RCLCPP_INFO(this->get_logger(), "%s", msg->name.c_str());

                    if (found != std::string::npos) {
                        // Implement action logic
                        mission.Ended = false;
                        GeoPoint from, to;
                        interfaces::msg::RegionPoint r_from, r_to;
                        mavros_msgs::msg::WaypointList route;

                        // Get coordinates
                        from.name = msg->parameters[0].value;
                        to.name = msg->parameters[1].value;
                        RCLCPP_INFO(this->get_logger(), "go_to %s -> %s", from.name.c_str(), to.name.c_str());

                        from.latitude = drone.position.latitude;
                        from.longitude = drone.position.longitude;
                        from.altitude = 15;

                        r_from = create_RegionPoint(from, mission.hMission.map);
                        r_to = getGeoPoint(to, mission.hMission.map);

                        RCLCPP_INFO(this->get_logger(), "GEO GeoPoint %f %f %f -> %f %f %f", r_from.geo.latitude, r_from.geo.longitude, r_from.geo.altitude, r_to.geo.latitude, r_to.geo.longitude, r_to.geo.altitude);

                        // Calc route
                        route = calcRoute(r_from, r_to, from.name, to.name, mission.hMission.map);

                        // Check if flying
                        while (!drone.current_state.armed && drone.ex_current_state.landed_state != 2) {
                            set_loiter();
                            arm();
                            takeoff(drone);
                        }
                        std::this_thread::sleep_for(std::chrono::seconds(10));

                        // Send route
                        if (!sendWPFile(route))
                            callRoute(from, to);
                        std::this_thread::sleep_for(std::chrono::seconds(20));

                        set_auto();

                        while (!mission.Ended) {
                            std::this_thread::sleep_for(std::chrono::seconds(10));
                        }

                        set_loiter();
                        std::this_thread::sleep_for(std::chrono::seconds(10));

                        RCLCPP_INFO(this->get_logger(), "KCL: (%s) HarpiaExecutor Action completing.", msg->name.c_str());
                    }
                    // Continue com outros casos de `msg->name` como "pulverize_region", "take_image", etc.
                }
                else {
                    RCLCPP_INFO(this->get_logger(), "NEED TO REPLAN");
                }
            }
            else {
                RCLCPP_ERROR(this->get_logger(), "Failed to call service harpia/mission_fault_mitigation");
            }
        }
    } 
}
}


/*-------------*/
/* Main method */
/*-------------*/



int main(int argc, char **argv)
{
    // Inicialize o ROS2
    rclcpp::init(argc, argv);

    // Crie o nó principal
    auto node = std::make_shared<rclcpp::Node>("rosplan_interface_harpia");

    // Crie o drone e a missão
    auto drone = std::make_shared<Drone>(node);
    auto mission = std::make_shared<Mission>(node);

    // Crie assinaturas
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

    auto global_sub = node->create_subscription<mavros_msgs::msg::WaypointList>(
        "/mavros/mission/waypoints", 1,
        [mission](const mavros_msgs::msg::WaypointList::SharedPtr msg) {
            mission->chatterCallback_wpqtd(msg);
        });

    auto current_sub = node->create_subscription<mavros_msgs::msg::WaypointReached>(
        "/mavros/mission/reached", 1,
        [mission](const mavros_msgs::msg::WaypointReached::SharedPtr msg) {
            mission->chatterCallback_current(msg);
        });

    auto harpia_mission_sub = node->create_subscription<interfaces::msg::Mission>(
        "/harpia/mission", 1,
        [mission](const interfaces::msg::Mission::SharedPtr msg) {
            mission->chatterCallback_harpiaMission(msg);
        });

    auto harpia_goalId_sub = node->create_subscription<interfaces::msg::MissionGoalManager>(
        "/harpia/mission_goal_manager/goal", 1,
        [mission](const interfaces::msg::MissionGoalManager::SharedPtr msg) {
            mission->chatterCallback_IDGoal(msg);
        });

    auto harpia_goalCancel_sub = node->create_subscription<interfaces::msg::ChangeMission>(
        "/harpia/ChangeMission", 1,
        [mission](const interfaces::msg::ChangeMission::SharedPtr msg) {
            mission->chatterCallback_cancelGoal(msg);
        });

    // Crie o executor e adicione o nó
    auto rpti = std::make_shared<plansy2::RPHarpiaExecutor>(node);

    // Executar o loop principal do ROS2
    rpti->runActionInterface();

    // Inicie o spinning
    rclcpp::spin(node);

    // Encerrar o ROS2
    rclcpp::shutdown();

    return 0;
}

