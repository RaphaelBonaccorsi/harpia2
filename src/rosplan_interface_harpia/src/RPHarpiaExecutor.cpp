#include <iostream>
#include "rosplan_interface_harpia/RPHarpiaExecutor.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Imu.h"
#include "mavros_msgs/WaypointList.h"
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <bits/stdc++.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <harpia_msgs/Mission.h>
#include <harpia_msgs/ChangeMission.h>
#include <harpia_msgs/UAV.h>
#include <harpia_msgs/Map.h>
#include <harpia_msgs/Goal.h>
#include <harpia_msgs/RegionPoint.h>
#include <harpia_msgs/MissionPlannerAction.h>
#include <harpia_msgs/MissionFaultMitigation.h>
#include <harpia_msgs/RegionPoint.h>
#include <harpia_msgs/PathPlanning.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "harpia_msgs/MissionPlannerActionGoal.h"
#include "actionlib_msgs/GoalID.h"

#include <signal.h>
#include <math.h>

#include <fstream>
#include<iomanip>

#include <cstdlib>


#include <string>
#include <cstdlib>
#include <geographic_msgs/msg/geo_point.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <harpia_msgs/msg/mission.hpp>
#include <mavros_msgs/msg/waypoint_list.hpp>
#include <mavros_msgs/msg/waypoint_reached.hpp>
#include <harpia_msgs/msg/mission_planner_action_goal.hpp>
#include <harpia_msgs/msg/change_mission.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <harpia_msgs/msg/region_point.hpp>
#include <harpia_msgs/msg/map.hpp>
#include <cstring>  // For std::strcmp
#include <fstream>
#include <sstream>
#include <iostream>
#include <cstdlib>  // For std::system
#include <harpia_msgs/srv/path_planning.hpp>



using namespace std;

std::string homepath = getenv("HOME");

/**
 * @brief Struct representing a geographic point with additional metadata.
 * 
 * This struct is used to represent a geographical point with name, longitude, latitude, and altitude.
 */
struct GeoPoint
{
    std::string name;       ///< Name or identifier for the geographic point
    double longitude;       ///< Longitude of the geographic point
    double latitude;        ///< Latitude of the geographic point
    double altitude;        ///< Altitude of the geographic point
};

/**
 * @brief Class representing a drone with its current state and position.
 * 
 * This class provides methods to handle callbacks related to the drone's GPS data, 
 * current state, and extended state. It also stores the current position and state of the drone.
 */
class Drone
{
public:
    GeoPoint position;                       ///< Current geographic position of the drone
    mavros_msgs::msg::State current_state;   ///< Current state of the drone
    mavros_msgs::msg::ExtendedState ex_current_state; ///< Extended state of the drone

    /**
     * @brief Callback function for handling GPS data.
     * 
     * This function updates the drone's position based on the received GPS data.
     * 
     * @param msg A pointer to the GPS data message of type sensor_msgs::msg::NavSatFix.
     */
    void chatterCallback_GPS(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    /**
     * @brief Callback function for handling the current state of the drone.
     * 
     * This function updates the drone's current state based on the received state data.
     * 
     * @param msg A pointer to the state data message of type mavros_msgs::msg::State.
     */
    void chatterCallback_currentState(const mavros_msgs::msg::State::SharedPtr msg);

    /**
     * @brief Callback function for handling the extended state of the drone.
     * 
     * This function updates the drone's extended state based on the received extended state data.
     * 
     * @param msg A pointer to the extended state data message of type mavros_msgs::msg::ExtendedState.
     */
    void chatterCallback_currentStateExtended(const mavros_msgs::msg::ExtendedState::SharedPtr msg);
};

/**
 * @brief Callback function for handling GPS data.
 * 
 * This method updates the drone's position attributes based on the received GPS data.
 * It assigns the longitude, latitude, and altitude from the GPS message to the respective 
 * fields in the `position` attribute of the `Drone` class.
 * 
 * @param msg A shared pointer to the GPS data message of type sensor_msgs::msg::NavSatFix.
 */
void Drone::chatterCallback_GPS(const sensor_msgs::msg::NavSatFix::SharedPtr msg){
    position.longitude = msg->longitude;
    position.latitude = msg->latitude;
    position.altitude = msg->altitude;
}

/**
 * @brief Callback function for handling the current state of the drone.
 * 
 * This method updates the `current_state` attribute of the `Drone` class based on the 
 * received state data message. The state message contains information about the drone's 
 * current status.
 * 
 * @param msg A shared pointer to the state data message of type mavros_msgs::msg::State.
 */
void Drone::chatterCallback_currentState(const mavros_msgs::msg::State::SharedPtr msg){
    current_state = *msg;
}

/**
 * @brief Callback function for handling the extended state of the drone.
 * 
 * This method updates the `ex_current_state` attribute of the `Drone` class based on the 
 * received extended state data message. The extended state message provides additional 
 * information about the drone's status.
 * 
 * @param msg A shared pointer to the extended state data message of type mavros_msgs::msg::ExtendedState.
 */
void Drone::chatterCallback_currentStateExtended(const mavros_msgs::msg::ExtendedState::SharedPtr msg){
    ex_current_state = *msg;
}

/**
 * @brief Represents a mission with waypoints and goal management.
 * 
 * This class encapsulates the information and operations related to a mission. It maintains
 * the state of the mission, including the current waypoint, goal ID, and status flags. It also
 * provides methods for handling mission-related messages and managing waypoints.
 */
class Mission{
public:
    int WPqtd;              ///< Number of waypoints in the mission.
    int currentWP;          ///< Index of the current waypoint.
    int IDGoal;             ///< ID of the goal.
    bool Ended = true;     ///< Flag indicating whether the mission has ended.
    bool Cancelled = false; ///< Flag indicating whether the mission has been cancelled.
    harpia_msgs::msg::Mission hMission; ///< Mission data.
    // decision_support::newMission missionWP; ///< Uncomment if needed for new mission support.

    /**
     * @brief Sends the current mission data.
     * 
     * This method is responsible for sending the mission data to relevant subscribers or services.
     */
    void send_mission();

    /**
     * @brief Callback function for handling waypoint list messages.
     * 
     * This method updates the waypoint count based on the received waypoint list message.
     * 
     * @param msg A shared pointer to the waypoint list message of type mavros_msgs::msg::WaypointList.
     */
    void chatterCallback_wpqtd(const mavros_msgs::msg::WaypointList::SharedPtr msg);

    /**
     * @brief Callback function for handling waypoint reached messages.
     * 
     * This method processes the waypoint reached message and updates the mission status.
     * 
     * @param msg A shared pointer to the waypoint reached message of type mavros_msgs::msg::WaypointReached.
     */
    void chatterCallback_current(const mavros_msgs::msg::WaypointReached::SharedPtr msg);

    /**
     * @brief Callback function for handling Harpia mission messages.
     * 
     * This method updates the mission data based on the received Harpia mission message.
     * 
     * @param msg A shared pointer to the Harpia mission message of type harpia_msgs::msg::Mission.
     */
    void chatterCallback_harpiaMission(const harpia_msgs::msg::Mission::SharedPtr msg);

    /**
     * @brief Callback function for handling mission planner action goal messages.
     * 
     * This method updates the goal ID based on the received action goal message.
     * 
     * @param msg A shared pointer to the mission planner action goal message of type harpia_msgs::msg::MissionPlannerActionGoal.
     */
    void chatterCallback_IDGoal(const harpia_msgs::msg::MissionPlannerActionGoal::SharedPtr msg);

    /**
     * @brief Callback function for handling change mission messages.
     * 
     * This method processes the change mission message and updates the mission cancellation status.
     * 
     * @param msg A shared pointer to the change mission message of type harpia_msgs::msg::ChangeMission.
     */
    void chatterCallback_cancelGoal(const harpia_msgs::msg::ChangeMission::SharedPtr msg);

    /**
     * @brief Default constructor for the Mission class.
     * 
     * Initializes the mission status to ended.
     */
    Mission();
};

// Implementation of the Mission constructor
Mission::Mission(){
    Ended = true;
}

/**
 * @brief Callback function for handling waypoint list messages.
 * 
 * This method updates the waypoint count based on the received waypoint list message.
 * 
 * @param msg A shared pointer to the waypoint list message of type mavros_msgs::msg::WaypointList.
 */
void Mission::chatterCallback_wpqtd(const mavros_msgs::msg::WaypointList::SharedPtr msg){
    WPqtd = msg->waypoints.size() - 1;
}

/**
 * @brief Callback function for handling mission planner action goal messages.
 * 
 * This method updates the goal ID based on the received action goal message.
 * 
 * @param msg A shared pointer to the mission planner action goal message of type harpia_msgs::msg::MissionPlannerActionGoal.
 */
void Mission::chatterCallback_IDGoal(const harpia_msgs::msg::MissionPlannerActionGoal::SharedPtr msg){
    IDGoal = std::stoi(msg->goal_id.id);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "id Goal: %i", IDGoal);
}

/**
 * @brief Callback function for handling waypoint reached messages.
 * 
 * This method processes the waypoint reached message and updates the current waypoint and mission status.
 * 
 * @param msg A shared pointer to the waypoint reached message of type mavros_msgs::msg::WaypointReached.
 */
void Mission::chatterCallback_current(const mavros_msgs::msg::WaypointReached::SharedPtr msg){
    if (currentWP != msg->wp_seq)
    {
        currentWP = msg->wp_seq;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waypoint: %i", msg->wp_seq + 1);
    }
    if (WPqtd == msg->wp_seq)
    {
        Ended = true;
    }
    else
    {
        Ended = false;
    }
}

/**
 * @brief Callback function for handling change mission messages.
 * 
 * This method updates the mission cancellation status based on the received change mission message.
 * 
 * @param msg A shared pointer to the change mission message of type harpia_msgs::msg::ChangeMission.
 */
void Mission::chatterCallback_cancelGoal(const harpia_msgs::msg::ChangeMission::SharedPtr msg){
    Cancelled = (msg->op != 0);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", msg->goals[0]);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%li", msg->op);

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "msg size: %li", sizeof(msg));
}

/**
 * @brief Callback function for handling mission messages.
 * 
 * This method updates the internal mission data based on the received mission message.
 * 
 * @param msg A shared pointer to the mission message of type harpia_msgs::msg::Mission.
 */
void Mission::chatterCallback_harpiaMission(const harpia_msgs::msg::Mission::SharedPtr msg)
{
    hMission.uav = msg->uav;
    hMission.map = msg->map;
    hMission.goals = msg->goals;
}

// Global instances of Mission and Drone classes
Mission mission;
Drone drone;

/**
 * @brief Land the drone at its current position.
 * 
 * This function sends a land command to the drone using the MAVROS CommandTOL service.
 * 
 * @param drone A reference to the Drone object containing current position data.
 */
void land(Drone &drone)
{
    auto node = rclcpp::Node::make_shared("drone_land_node");
    auto land_client = node->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");

    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    request->altitude = 0;
    request->latitude = drone.position.latitude;
    request->longitude = drone.position.longitude;
    request->min_pitch = 0;
    request->yaw = 0;

    if (land_client->wait_for_service(std::chrono::seconds(10)))
    {
        auto result_future = land_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result_future.get();
            RCLCPP_INFO(node->get_logger(), "Land service response: %d", response->success);
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to call land service.");
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Land service not available.");
    }
}

/**
 * @brief Set the drone to loiter mode.
 * 
 * This function sends a set mode command to the drone using the MAVROS SetMode service.
 */
void set_loiter()
{
    auto node = rclcpp::Node::make_shared("drone_set_loiter_node");
    auto mode_client = node->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->base_mode = 0;
    request->custom_mode = "AUTO.LOITER";

    if (mode_client->wait_for_service(std::chrono::seconds(10)))
    {
        auto result_future = mode_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(node->get_logger(), "Set mode to LOITER.");
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to set mode.");
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Set mode service not available.");
    }
}

/**
 * @brief Set the drone to auto mission mode.
 * 
 * This function sends a set mode command to the drone using the MAVROS SetMode service to switch to "AUTO.MISSION" mode.
 */
void set_auto()
{
    auto node = rclcpp::Node::make_shared("drone_set_auto_node");
    auto mode_client = node->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->base_mode = 0;
    request->custom_mode = "AUTO.MISSION";

    if (mode_client->wait_for_service(std::chrono::seconds(10)))
    {
        auto result_future = mode_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(node->get_logger(), "Set mode to AUTO.");
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to set mode.");
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Set mode service not available.");
    }
}

/**
 * @brief Arm the drone.
 * 
 * This function sends a command to arm the drone using the MAVROS CommandBool service.
 */
void arm()
{
    auto node = rclcpp::Node::make_shared("drone_arm_node");
    auto arming_client = node->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = true;

    if (arming_client->wait_for_service(std::chrono::seconds(10)))
    {
        auto result_future = arming_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result_future.get();
            RCLCPP_INFO(node->get_logger(), "Arming command response: %d", response->success);
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to arm the drone.");
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Arming service not available.");
    }
}

/**
 * @brief Command the drone to take off.
 * 
 * This function sends a takeoff command to the drone using the MAVROS CommandTOL service.
 * 
 * @param drone The Drone object containing the current position of the drone.
 */
void takeoff(const Drone& drone)
{
    auto node = rclcpp::Node::make_shared("drone_takeoff_node");
    auto takeoff_client = node->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");

    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    request->altitude = 15;
    request->latitude = drone.position.latitude;
    request->longitude = drone.position.longitude;
    request->min_pitch = 0;
    request->yaw = 0;

    if (takeoff_client->wait_for_service(std::chrono::seconds(10)))
    {
        auto result_future = takeoff_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result_future.get();
            RCLCPP_INFO(node->get_logger(), "Takeoff command response: %d", response->success);
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to send takeoff command.");
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Takeoff service not available.");
    }
}

/**
 * @brief Retrieve the geographical point from the map based on the given GeoPoint name.
 * 
 * This function searches the provided map for a region or base that matches the name in the given GeoPoint.
 * 
 * @param geo The GeoPoint containing the name to search for.
 * @param mapa The map to search within.
 * 
 * @return The RegionPoint corresponding to the name in the map, or a null RegionPoint if not found.
 */
harpia_msgs::msg::RegionPoint getGeoPoint(const GeoPoint& geo, const harpia_msgs::msg::Map& mapa)
{
    int qtd_regions = mapa.roi.size();
    for (int i = 0; i < qtd_regions; ++i)
    {
        if (std::strcmp(mapa.roi[i].name.c_str(), geo.name.c_str()) == 0)
        {
            RCLCPP_INFO(rclcpp::get_logger("getGeoPoint"), "%s", mapa.roi[i].name.c_str());
            return mapa.roi[i].center;
        }
    }

    qtd_regions = mapa.bases.size();
    for (int i = 0; i < qtd_regions; ++i)
    {
        if (std::strcmp(mapa.bases[i].name.c_str(), geo.name.c_str()) == 0)
        {
            RCLCPP_INFO(rclcpp::get_logger("getGeoPoint"), "%s", mapa.bases[i].name.c_str());
            return mapa.bases[i].center;
        }
    }

    // Return a null RegionPoint if not found
    harpia_msgs::msg::RegionPoint null;
    return null;
}

/**
 * @brief Retrieve the radius of a given region by executing a Python script.
 * 
 * This function executes a Python script to get the radius of a specified region and reads the result
 * from a file. The file path is constructed using the home directory path.
 * 
 * @param region The name of the region for which to retrieve the radius.
 * 
 * @return The radius of the region as an integer. Returns a default value of 10 if the file cannot be opened.
 */
int getRadius(const std::string& region)
{
    std::string command = "python3 ~/drone_arch/drone_ws/src/ROSPlan/src/rosplan/rosplan_planning_system/src/ActionInterface/getRadius.py " + region + " >> ~/drone_arch/Data/out.txt";
    std::system(command.c_str());

    std::string line;
    std::ifstream myfile((std::getenv("HOME") + std::string("/drone_arch/Data/out.txt")).c_str());
    if (myfile.is_open())
    {
        std::getline(myfile, line);
        myfile.close();
        
        try
        {
            return std::stoi(line);
        }
        catch (const std::invalid_argument& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("getRadius"), "Failed to convert line to integer: %s", e.what());
            return 10; // Return a default value in case of error
        }
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("getRadius"), "Unable to open file");
        return 10; // Return a default value if file cannot be opened
    }
}

/**
 * @brief Calculate the route between two geographic points using a path planning service.
 * 
 * This function calls a path planning service to calculate a route between the specified points. The route
 * is represented as a list of waypoints.
 * 
 * @param from The starting point of the route.
 * @param to The destination point of the route.
 * @param name_from The name associated with the starting point.
 * @param name_to The name associated with the destination point.
 * @param map The map used for planning the route.
 * 
 * @return A list of waypoints representing the calculated route. Returns an empty list if the service call fails.
 */
mavros_msgs::msg::WaypointList calcRoute(const harpia_msgs::msg::RegionPoint& from, const harpia_msgs::msg::RegionPoint& to,
                                         const std::string& name_from, const std::string& name_to, const harpia_msgs::msg::Map& map)
{
    auto node = rclcpp::Node::make_shared("route_calculation_node");
    auto client = node->create_client<harpia_msgs::srv::PathPlanning>("harpia/path_planning");

    auto request = std::make_shared<harpia_msgs::srv::PathPlanning::Request>();
    request->r_from = from;
    request->r_to = to;
    request->name_from = name_from;
    request->name_to = name_to;
    request->op = 0;
    request->map = map;

    if (client->wait_for_service(std::chrono::seconds(10)))
    {
        auto result_future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            return result_future.get()->waypoints;
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to call service harpia/path_planning");
            mavros_msgs::msg::WaypointList null;
            return null;
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Service harpia/path_planning not available.");
        mavros_msgs::msg::WaypointList null;
        return null;
    }
}

/**
 * @brief Calculate the route to pulverize a region from a given point using a path planning service.
 * 
 * This function requests a route from a specified point to a "pulverize_region" using the path planning service.
 * The operation type is set to 1 for pulverizing.
 * 
 * @param at The point from which the route starts and ends.
 * @param map The map used for planning the route.
 * 
 * @return A list of waypoints representing the calculated route. Returns an empty list if the service call fails.
 */
mavros_msgs::msg::WaypointList calcRoute_pulverize(const harpia_msgs::msg::RegionPoint& at, const harpia_msgs::msg::Map& map)
{
    auto node = rclcpp::Node::make_shared("route_calculation_node");
    auto client = node->create_client<harpia_msgs::srv::PathPlanning>("harpia/path_planning");

    auto request = std::make_shared<harpia_msgs::srv::PathPlanning::Request>();
    request->r_from = at;
    request->r_to = at;
    request->op = 1; // Operation type for pulverize
    request->name_from = "at";
    request->name_to = "pulverize_region";
    request->map = map;

    if (client->wait_for_service(std::chrono::seconds(10)))
    {
        auto result_future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            return result_future.get()->waypoints;
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to call service harpia/path_planning");
            mavros_msgs::msg::WaypointList null;
            return null;
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Service harpia/path_planning not available.");
        mavros_msgs::msg::WaypointList null;
        return null;
    }
}

/**
 * @brief Calculate the route to take a picture from a given point using a path planning service.
 * 
 * This function requests a route from a specified point to a "take_picture" region using the path planning service.
 * The operation type is set to 2 for taking a picture.
 * 
 * @param at The point from which the route starts and ends.
 * @param map The map used for planning the route.
 * 
 * @return A list of waypoints representing the calculated route. Returns an empty list if the service call fails.
 */
mavros_msgs::msg::WaypointList calcRoute_picture(const harpia_msgs::msg::RegionPoint& at, const harpia_msgs::msg::Map& map)
{
    auto node = rclcpp::Node::make_shared("route_calculation_node");
    auto client = node->create_client<harpia_msgs::srv::PathPlanning>("harpia/path_planning");

    auto request = std::make_shared<harpia_msgs::srv::PathPlanning::Request>();
    request->r_from = at;
    request->r_to = at;
    request->op = 2; // Operation type for taking a picture
    request->name_from = "at";
    request->name_to = "take_picture";
    request->map = map;

    if (client->wait_for_service(std::chrono::seconds(10)))
    {
        auto result_future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            return result_future.get()->waypoints;
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to call service harpia/path_planning");
            mavros_msgs::msg::WaypointList null;
            return null;
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Service harpia/path_planning not available.");
        mavros_msgs::msg::WaypointList null;
        return null;
    }
}

int sendWPFile(mavros_msgs::WaypointList mission_wp)
{
	ros::NodeHandle p;
	GeoPoint geo;
	string line;
	int wp_count = 1 ;
	//mavros_msgs::Waypoint* mission_wp = NULL;
	mavros_msgs::WaypointPush wp_push_srv;
	mavros_msgs::WaypointClear wp_clear_srv;


	ros::ServiceClient wp_srv_client = p.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
	ros::ServiceClient wp_clear_client = p.serviceClient<mavros_msgs::WaypointClear>("/mavros/mission/clear");


 	wp_clear_srv.request = {};

  	if (wp_clear_client.call(wp_clear_srv))
	{
	    ROS_INFO("Waypoint list was cleared");
	}
	else
	{
	    ROS_ERROR("Waypoint list couldn't been cleared");
	}

  	wp_push_srv.request.start_index = 0;
  	wp_count = mission_wp.waypoints.size();
  	cout << wp_count << endl;
  	for(int n=0; n<wp_count; n++)
  		wp_push_srv.request.waypoints.push_back(mission_wp.waypoints[n]);

  	cout << wp_srv_client.call(wp_push_srv) << endl;

  	if(wp_srv_client.call(wp_push_srv))
  	{
 	   ROS_INFO("Success:%d", (bool)wp_push_srv.response.success);
 	   remove((homepath + "/drone_arch/Data/route.txt").c_str());
 	}
 	else
 	{
  	  ROS_ERROR("Waypoint couldn't been sent");
  	  ROS_ERROR("Success:%d", (bool)wp_push_srv.response.success);
 	  remove((homepath + "/drone_arch/Data/route.txt").c_str());
  	  return 0;
  	}


  	return 1;
}

void reset_mission()
{
	ros::NodeHandle nh;
	ros::ServiceClient set_current_client = nh.serviceClient<mavros_msgs::WaypointSetCurrent>("mavros/mission/set_current");
	mavros_msgs::WaypointSetCurrent set_current_srv;

	set_current_srv.request.wp_seq = 0;

  	if (set_current_client.call(set_current_srv))
	{
	    ROS_INFO("Reset Mission");
	}
	else
	{
	    ROS_ERROR("Reset couldn't been done");
	}

}

void callRoute(GeoPoint from, GeoPoint to)
{
	ROS_INFO("Sending WP file for route %s _ %s.wp", from.name.c_str(), to.name.c_str());
	string command = "rosrun mavros mavwp load ~/drone_arch/Data/Rotas/wp/"+from.name+"_"+to.name+".wp";
	ROS_INFO("%s", command.c_str());
	system(command.c_str());
}

geometry_msgs::Point convert_goe_to_cart(geographic_msgs::GeoPoint p, geographic_msgs::GeoPoint home)
{
	geometry_msgs::Point point;
	double pi = 2*acos(0.0);
	point.x = (p.longitude - home.longitude) * (6400000.0 * (cos(home.latitude * pi / 180) * 2 * pi / 360));
	point.y =(p.latitude - home.latitude)  * (10000000.0 / 90);

	return point;
}

harpia_msgs::RegionPoint create_RegionPoint(GeoPoint point, harpia_msgs::Map map)
{
	harpia_msgs::RegionPoint region_point;

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
	ros::shutdown();
}
/*--------------------------------------------*/


namespace KCL_rosplan {
	RPHarpiaExecutor::RPHarpiaExecutor(ros::NodeHandle &nh) {}

	/* action dispatch callback */
	bool RPHarpiaExecutor::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		ros::NodeHandle n;
	 	ros::ServiceClient client = n.serviceClient<harpia_msgs::MissionFaultMitigation>("harpia/mission_fault_mitigation");
	  	harpia_msgs::MissionFaultMitigation srv;
	  	srv.request.uav = mission.hMission.uav;
	  	srv.request.action_id = msg->action_id;


	  	int replan, cancelled;

	  	if (client.call(srv) && msg->action_id != 0)
	  	{
	    	replan = srv.response.replan;
	  	}
	  	else
	  	{
	    	ROS_INFO("BN not called");
	    	replan = 0;
	  	}
	 //  	if(mission.Cancelled)
		// {
		// 	ROS_ERROR("Preempted %s", msg->name.c_str());
		// 	set_loiter();
		// 	mission.Cancelled = false;
		// 	return false;
		// }
	  	if(replan != 1)
		{
            // There was no need for replan.

            // The action implementation goes here.
			string str = msg->name.c_str();
			string str1 = "go_to";
			size_t found = str.find(str1);
			ROS_INFO("%s", msg->name.c_str());
	    	if (found != string::npos)
			{
                // We are on the way and there is no need for replan.

				mission.Ended = false;
				GeoPoint from, to;
				harpia_msgs::RegionPoint r_from, r_to;
				mavros_msgs::WaypointList route;

				//get coordinates
				from.name = msg->parameters[0].value.c_str();
				to.name = msg->parameters[1].value.c_str();
				ROS_INFO("go_to %s -> %s", from.name.c_str(), to.name.c_str());

				//getGeoPoint(&from);
				from.latitude = drone.position.latitude;
				from.longitude = drone.position.longitude;
				from.altitude = 15;

				r_from = create_RegionPoint(from, mission.hMission.map);
				r_to = getGeoPoint(to, mission.hMission.map);



				ROS_INFO("GEO GeoPoint %f %f %f -> %f %f %f", r_from.geo.latitude, r_from.geo.longitude, r_from.geo.altitude, r_to.geo.latitude, r_to.geo.longitude, r_to.geo.altitude);


				//calc route
				route = calcRoute(r_from, r_to, from.name, to.name, mission.hMission.map);

				//is flying?
					//is armed
				while(!drone.current_state.armed && drone.ex_current_state.landed_state != 2)
				{
					set_loiter();
					arm();
					takeoff(drone);
				}
				ros::Duration(10).sleep();

				//send route
				if(!sendWPFile(route))
					callRoute(from, to);
				ros::Duration(20).sleep();


				set_auto();

				//while not at the end, wait
				while(!mission.Ended){
					ros::Duration(10).sleep();
				}

				set_loiter();
				ros::Duration(10).sleep();

				ROS_INFO("KCL: (%s) HarpiaExecutor Action completing.", msg->name.c_str());
				// return true;

			}
			else if (strcmp(msg->name.c_str(), "pulverize_region") == 0)
			{
				// string region = msg->parameters[1].value.c_str();

				// ROS_INFO("pulverize_region %s", region.c_str());

				ROS_INFO("pulverize_region");
				mavros_msgs::WaypointList route;
				// string region = msg->parameters[1].value.c_str();
				mission.Ended = false;

				//get coordinates
				//int radius = getRadius(region);

				//calc route
				GeoPoint at;
				at.latitude = drone.position.latitude;
				at.longitude = drone.position.longitude;
				at.altitude = 15;
				harpia_msgs::RegionPoint r_at;
				r_at = create_RegionPoint(at, mission.hMission.map);
				route = calcRoute_pulverize(r_at, mission.hMission.map);


				//is flying?
					//is armed
				while(!drone.current_state.armed && drone.ex_current_state.landed_state != 2)
				{
					set_loiter();
					arm();
					takeoff(drone);
				}
				ros::Duration(20).sleep();

				//send route
				if(!sendWPFile(route))
					ROS_ERROR("Error call route pulverize_region");
				ros::Duration(20).sleep();

				//change to auto
				set_auto();

				while(!mission.Ended){
					ros::Duration(10).sleep();
				}
				//while not at the end, wait
				set_loiter();
				ros::Duration(10).sleep();

				// complete the action
				ROS_INFO("KCL: (%s) HarpiaExecutor Action completing.", msg->name.c_str());
				// return true;
			}
			else if (strcmp(msg->name.c_str(), "take_image") == 0)
			{
				ROS_INFO("take_image");
				string region = msg->parameters[0].value.c_str();
				mission.Ended = false;
				mavros_msgs::WaypointList route;

				//get coordinates
				//int radius = getRadius(region);

				//calc route
				GeoPoint at;
				at.latitude = drone.position.latitude;
				at.longitude = drone.position.longitude;
				at.altitude = 15;
				harpia_msgs::RegionPoint r_at;
				r_at = create_RegionPoint(at, mission.hMission.map);
				route = calcRoute_picture(r_at, mission.hMission.map);

				//is flying?
					//is armed
				while(!drone.current_state.armed && drone.ex_current_state.landed_state != 2)
				{
					set_loiter();
					arm();
					takeoff(drone);
				}
				ros::Duration(20).sleep();

				//send route
				if(!sendWPFile(route))
					ROS_ERROR("Error call route pulverize_region");
				ros::Duration(20).sleep();

				//change to auto
				set_auto();

				//while not at the end, wait
				while(!mission.Ended){
					ros::Duration(10).sleep();
				}

				set_loiter();
				ros::Duration(10).sleep();
				// complete the action
				ROS_INFO("KCL: (%s) HarpiaExecutor Action completing.", msg->name.c_str());
				// return true;
			}
			else if (strcmp(msg->name.c_str(), "recharge_input") == 0)
			{
				ROS_INFO("recharge_input");

				if(drone.current_state.mode != "AUTO.LAND")
					land(drone);

				while(drone.ex_current_state.landed_state != 1)
				{
					ROS_INFO("landing... %d", drone.ex_current_state.landed_state);
					ros::Duration(10).sleep();
				}


				ros::Duration(msg->duration).sleep();
				// complete the action
				ROS_INFO("KCL: (%s) HarpiaExecutor Action completing.", msg->name.c_str());
				// return true;
			}
			else if (strcmp(msg->name.c_str(), "clean_camera") == 0)
			{
				ROS_INFO("clean_camera");
				if(drone.current_state.mode != "AUTO.LAND")
					land(drone);

				while(drone.ex_current_state.landed_state != 1)
				{
					ROS_INFO("landing... %d", drone.ex_current_state.landed_state);
					ros::Duration(10).sleep();
				}
				ros::Duration(msg->duration).sleep();
				// complete the action
				ROS_INFO("KCL: (%s) HarpiaExecutor Action completing.", msg->name.c_str());
				// return true;
			}
			else if (strcmp(msg->name.c_str(), "recharge_battery") == 0)
			{
				ROS_INFO("recharge_battery");
				if(drone.current_state.mode != "AUTO.LAND")
					land(drone);

				while(drone.ex_current_state.landed_state != 1)
				{
					ROS_INFO("landing... %d", drone.ex_current_state.landed_state);
					ros::Duration(10).sleep();
				}
				ros::Duration(msg->duration).sleep();
				// complete the action
				ROS_INFO("KCL: (%s) HarpiaExecutor Action completing.", msg->name.c_str());
				// return true;
			}
			else if (strcmp(msg->name.c_str(), "has_all_goals_achived") == 0)
			{
				ROS_INFO("has-all-goals-achived");

				ros::Duration(1).sleep();
				// complete the action
				ROS_INFO("KCL: (%s) HarpiaExecutor Action completing.", msg->name.c_str());
				// return true;
			}
			else if (strcmp(msg->name.c_str(), "need_battery") == 0)
			{
				ROS_INFO("need-battery");

				ros::Duration(msg->duration).sleep();
				// complete the action
				ROS_INFO("KCL: (%s) HarpiaExecutor Action completing.", msg->name.c_str());
				// return true;
			}
			else if (strcmp(msg->name.c_str(), "need_input") == 0)
			{
				ROS_INFO("need-input");

				ros::Duration(msg->duration).sleep();
				// complete the action
				ROS_INFO("KCL: (%s) HarpiaExecutor Action completing.", msg->name.c_str());
				// return true;
			}

			if(mission.Cancelled)
			{
				ROS_ERROR("Preempted %s", msg->name.c_str());
				set_loiter();
				mission.Cancelled = false;
				return false;
			}

			return true;
		}
		else
		{
			ROS_INFO("NEED TO REPLAN");
			return false;
		}
		// verify if has cancelled current mission

		

	}
} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_interface_harpia", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    signal(SIGINT, mySigintHandler);

    ros::Subscriber GPS 				= nh.subscribe("/mavros/global_position/global", 		1, &Drone::chatterCallback_GPS, 				&drone);
    ros::Subscriber state 	 			= nh.subscribe("/mavros/state", 				 		1, &Drone::chatterCallback_currentState, 		&drone);
    ros::Subscriber state_ext 			= nh.subscribe("/mavros/extended_state", 		 		1, &Drone::chatterCallback_currentStateExtended,&drone);
    ros::Subscriber global 				= nh.subscribe("/mavros/mission/waypoints", 	 		1, &Mission::chatterCallback_wpqtd, 			&mission);
    ros::Subscriber current 			= nh.subscribe("/mavros/mission/reached", 		 		1, &Mission::chatterCallback_current, 			&mission);
    ros::Subscriber harpia_mission 		= nh.subscribe("/harpia/mission", 				 		1, &Mission::chatterCallback_harpiaMission, 	&mission);
    ros::Subscriber harpia_goalId 		= nh.subscribe("/harpia/mission_goal_manager/goal", 	1, &Mission::chatterCallback_IDGoal, 			&mission);
    ros::Subscriber harpia_goalCancel 	= nh.subscribe("/harpia/ChangeMission", 				1, &Mission::chatterCallback_cancelGoal, 		&mission);

    // create PDDL action subscriber
    KCL_rosplan::RPHarpiaExecutor rpti(nh);

    rpti.runActionInterface();

    return 0;
}
