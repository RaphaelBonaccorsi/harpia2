#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <mavros_msgs/msg/WaypointList.hpp>
#include <mavros_msgs/srv/CommandBool.hpp>
#include <mavros_msgs/msg/VfrHud.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <bits/stdc++.h>
// Update or replace harpia_msgs with ROS 2 equivalent
#include <interfaces/msg/drone_pose.hpp>
#include "action_msgs/msg/goal_id.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <signal.h>
#include <cmath>

#include <fstream>
#include <iomanip>

#include <cstdlib>
using namespace std;
std::string homepath = getenv("HOME");

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
		interfaces::DronePose pose;
		void chatterCallback_localPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void chatterCallback_imu(const sensor_msgs::Imu::ConstPtr& msg);
		void chatterCallback_vfr_hud(const mavros_msgs::VfrHud::ConstPtr& msg);
		void chatterCallback_golbalp(const nav_msgs::Odometry::ConstPtr& msg);
		void chatterCallback_compass(const std_msgs::Float64::ConstPtr& msg);
};



void Drone::chatterCallback_localPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // do conversions
	float linearposx=msg->pose.position.x;
   	float linearposy=msg->pose.position.y;
   	double quatx= msg->pose.orientation.x;
   	double quaty= msg->pose.orientation.y;
   	double quatz= msg->pose.orientation.z;
   	double quatw= msg->pose.orientation.w;

    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    pose.roll = roll;
    pose.pitch = pitch;
    pose.yaw = yaw;
}

void Drone::chatterCallback_imu(const sensor_msgs::Imu::ConstPtr& msg)
{
    pose.rollRate = msg->angular_velocity.x;
    pose.pitchRate = msg->angular_velocity.y;
    pose.yawRate = msg->angular_velocity.z;
}

void Drone::chatterCallback_vfr_hud(const mavros_msgs::VfrHud::ConstPtr& msg)
{
    pose.groundSpeed = msg->groundspeed;
    pose.climbRate   = msg->climb;
    pose.throttle    = msg->throttle;
}

void Drone::chatterCallback_golbalp(const nav_msgs::Odometry::ConstPtr& msg)
{
    pose.altRelative = msg->pose.pose.position.z;
}

void Drone::chatterCallback_compass(const std_msgs::Float64::ConstPtr& msg)
{
    pose.heading = msg->data;
}


/*-------------*/
/* Main method */
/*-------------*/
class Drone
{
public:
    Drone()
    {
        // Initialize publishers and subscribers
        pub_ = this->create_publisher<interfaces::msg::DronePose>("pose", 10);

        gps_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", 1, std::bind(&Drone::chatterCallbackLocalPose, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data", 1, std::bind(&Drone::chatterCallbackImu, this, std::placeholders::_1));

        vfr_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/mavros/vfr_hud", 1, std::bind(&Drone::chatterCallbackVfrHud, this, std::placeholders::_1));

        gpos_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/mavros/global_position/local", 1, std::bind(&Drone::chatterCallbackGlobalPos, this, std::placeholders::_1));

        hdg_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/mavros/global_position/compass_hdg", 1, std::bind(&Drone::chatterCallbackCompass, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&Drone::timerCallback, this));
    }

private:
    void chatterCallbackLocalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Process local pose data
    }

    void chatterCallbackImu(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Process IMU data
    }

    void chatterCallbackVfrHud(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Process VFR HUD data
    }

    void chatterCallbackGlobalPos(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Process global position data
    }

    void chatterCallbackCompass(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // Process compass heading data
    }

    void timerCallback()
    {
        // Publish the drone pose
        pub_->publish(pose_);
    }

    rclcpp::Publisher<interfaces::msg::DronePose>::SharedPtr pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vfr_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gpos_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr hdg_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    interfaces::msg::DronePose pose_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Drone>();

    RCLCPP_INFO(node->get_logger(), "ROS 2 node started");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
