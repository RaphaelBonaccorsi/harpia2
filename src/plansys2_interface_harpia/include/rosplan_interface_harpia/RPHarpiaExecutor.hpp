#ifndef RPHARPIAEXECUTOR_H
#define RPHARPIAEXECUTOR_H

#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/waypoint_list.hpp>
#include <mavros_msgs/msg/waypoint_reached.hpp>
#include <mavros_msgs/srv/waypoint_set_current.hpp>
#include <mavros_msgs/srv/waypoint_push.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "interfaces/msg/mission.hpp"

#include "drone.hpp"
#include "mission.hpp"

namespace plansys2 {

class RPHarpiaExecutor : public rclcpp::Node {
public:
    RPHarpiaExecutor();
    void do_work() override;

private:
    // Declarar os membros privados aqui
};

}

#endif // RPHARPIAEXECUTOR_H
