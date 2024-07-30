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

        // Handle `action` and `region` if necessary
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
    // Create a client for the /mavros/set_mode service
    auto client = node->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    
    // Create a request for the service
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->base_mode = 0;
    request->custom_mode = "AUTO.LOITER";
    
    // Wait until the client is available
    if (!client->wait_for_service(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(node->get_logger(), "Service /mavros/set_mode not available");
        return;
    }
    
    // Create a future for the service call
    auto future = client->async_send_request(request);

    try
    {
        // Wait for the service result
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
    // Create a client for the /mavros/set_mode service
    auto client = node->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    
    // Create a request for the service
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->base_mode = 0;
    request->custom_mode = "AUTO";
    
    // Wait until the client is available
    if (!client->wait_for_service(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(node->get_logger(), "Service /mavros/set_mode not available");
        return;
    }
    
    // Create a future for the service call
    auto future = client->async_send_request(request);

    try
    {
        // Wait for the service result
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



void set_waypoint_current(const int waypoint, const rclcpp::Node::SharedPtr &node)
{
    // Create a client for the /mavros/mission/set_current service
    auto client = node->create_client<mavros_msgs::srv::WaypointSetCurrent>("/mavros/mission/set_current");
    
    // Create a request for the service
    auto request = std::make_shared<mavros_msgs::srv::WaypointSetCurrent::Request>();
    request->wp_seq = waypoint;
    
    // Wait until the client is available
    if (!client->wait_for_service(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(node->get_logger(), "Service /mavros/mission/set_current not available");
        return;
    }
    
    // Create a future for the service call
    auto future = client->async_send_request(request);

    try
    {
        // Wait for the service result
        auto result = future.get();
        if (result->success)
        {
            RCLCPP_INFO(node->get_logger(), "Waypoint %d set successfully", waypoint);
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to set waypoint %d", waypoint);
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Service call failed: %s", e.what());
    }
}


void send_waypoints(const interfaces::msg::Mission &mission, const rclcpp::Node::SharedPtr &node)
{
    auto client = node->create_client<mavros_msgs::srv::WaypointPush>("/mavros/mission/push");
    
    auto request = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();
    request->start_index = 0;
    
    for (const auto &goal : mission.goals)
    {
        mavros_msgs::msg::Waypoint waypoint;
        waypoint.frame = mavros_msgs::msg::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint.command = mavros_msgs::msg::CommandCode::NAV_WAYPOINT;
        waypoint.is_current = false;
        waypoint.autocontinue = true;
        waypoint.param1 = 0;
        waypoint.param2 = 0;
        waypoint.param3 = 0;
        waypoint.param4 = 0;
        waypoint.x_lat = goal.point.latitude;
        waypoint.y_long = goal.point.longitude;
        waypoint.z_alt = goal.point.altitude;
        
        request->waypoints.push_back(waypoint);
    }
    
    if (!client->wait_for_service(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(node->get_logger(), "Service /mavros/mission/push not available");
        return;
    }
    
    auto future = client->async_send_request(request);

    try
    {
        auto result = future.get();
        if (result->success)
        {
            RCLCPP_INFO(node->get_logger(), "Waypoints sent successfully");
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to send waypoints");
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Service call failed: %s", e.what());
    }
}


void takeoff(const std::shared_ptr<Drone> &drone, const rclcpp::Node::SharedPtr &node)
{
    auto takeoff_client = node->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    request->altitude = 5; 
    request->latitude = drone->position.latitude;
    request->longitude = drone->position.longitude;
    request->min_pitch = 0;
    request->yaw = 0;

    if (takeoff_client->wait_for_service(std::chrono::seconds(10)))
    {
        auto result = takeoff_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(node->get_logger(), "srv_takeoff send ok %d", result.get()->success);
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Failed Takeoff");
        }
    }
}

bool run_waypoints(const rclcpp::Node::SharedPtr &node)
{
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    auto client = node->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
    auto result = client->async_send_request(request);
    auto response = result.get();

    if (!response->success)
    {
        RCLCPP_INFO(node->get_logger(), "Failed arming");
        return false;
    }

    takeoff(drone_ptr, node);

    set_auto(node);

    return true;
}

void run_mission(const std::shared_ptr<Drone> &drone, const std::shared_ptr<Mission> &mission, const rclcpp::Node::SharedPtr &node)
{
    // Upload waypoints
    send_waypoints(mission->hMission, node);

    // Arm the drone
    auto arm_client = node->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_request->value = true;

    if (!arm_client->wait_for_service(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(node->get_logger(), "Service /mavros/cmd/arming not available");
        return;
    }

    auto arm_future = arm_client->async_send_request(arm_request);
    if (rclcpp::spin_until_future_complete(node, arm_future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Arming command sent successfully");
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to send arming command");
        return;
    }

    // Takeoff
    takeoff(drone, node);

    // Set auto mode
    set_auto(node);

    // Check if the mission is cancelled or ended
    while (!mission->Ended && !mission->Cancelled)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if (mission->Cancelled)
    {
        set_loiter(node);
        RCLCPP_INFO(node->get_logger(), "Mission cancelled, loiter mode set");
    }

    if (mission->Ended)
    {
        land(drone, node);
        RCLCPP_INFO(node->get_logger(), "Mission ended, landing");
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rpharpia_executor");
    
    // Create Drone and Mission instances
    auto drone = std::make_shared<Drone>();
    auto mission = std::make_shared<Mission>();
    
    // Subscribe to topics
    auto subscription_GPS = node->create_subscription<sensor_msgs::msg::NavSatFix>("/mavros/global_position/global", 10, std::bind(&Drone::chatterCallback_GPS, drone, std::placeholders::_1));
    auto subscription_currentState = node->create_subscription<mavros_msgs::msg::State>("/mavros/state", 10, std::bind(&Drone::chatterCallback_currentState, drone, std::placeholders::_1));
    auto subscription_currentStateExtended = node->create_subscription<mavros_msgs::msg::ExtendedState>("/mavros/extended_state", 10, std::bind(&Drone::chatterCallback_currentStateExtended, drone, std::placeholders::_1));
    auto subscription_wpqtd = node->create_subscription<mavros_msgs::msg::WaypointList>("/mavros/mission/waypoints", 10, std::bind(&Mission::chatterCallback_wpqtd, mission, std::placeholders::_1));
    auto subscription_current = node->create_subscription<mavros_msgs::msg::WaypointReached>("/mavros/mission/reached", 10, std::bind(&Mission::chatterCallback_current, mission, std::placeholders::_1));
    auto subscription_harpiaMission = node->create_subscription<interfaces::msg::Mission>("/harpia/mission", 10, std::bind(&Mission::chatterCallback_harpiaMission, mission, std::placeholders::_1));
    auto subscription_IDGoal = node->create_subscription<interfaces::msg::IDGoal>("/harpia/IDGoal", 10, std::bind(&Mission::chatterCallback_IDGoal, mission, std::placeholders::_1));
    
    // Run mission
    run_mission(drone, mission, node);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

