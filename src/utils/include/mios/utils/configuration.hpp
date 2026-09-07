#pragma once
#include <string>
#include <iostream>

namespace mios{

struct MiosConfiguration {
    std::string verbosity;
    std::string robot_ip;
    std::string database_name;

    unsigned    robot_configuration;
    unsigned    database_port;
    unsigned    websocket_port;
    unsigned    udp_port;
    unsigned    rpc_port;
    bool        use_desk;
    // Keep legacy behavior by default. The ROS-owned Core can defer this
    // blocking connection until its explicitly enabled scheduler initializes.
    bool        defer_database_connection{false};
    // A ROS controller claims an interface before a legacy MIOS task starts,
    // which places Franka in MOVE mode even while the commanded velocity is
    // zero. Keep the legacy IDLE-only task precheck unless a ROS operator
    // explicitly commissions that ownership hand-off.
    bool        allow_controller_owned_move_mode{false};
    // Define how this struct should be printed
    friend std::ostream& operator<<(std::ostream& os, const MiosConfiguration& c) {
        os << "MiosConfiguration: {\n"
           << "  verbosity:           " << c.verbosity << "\n"
           << "  robot_ip:            " << c.robot_ip << "\n"
           << "  robot_configuration: " << c.robot_configuration << "\n"
           << "  use_desk:            " << c.use_desk << "\n"
           << "  database_name:       " << c.database_name << "\n"
           << "  database_port:       " << c.database_port << "\n"
           << "  defer_database_connection: " << c.defer_database_connection << "\n"
           << "  allow_controller_owned_move_mode: "
           << c.allow_controller_owned_move_mode << "\n"
           << "  websocket_port:      " << c.websocket_port << "\n"
           << "  rpc_port:            " << c.rpc_port << "\n"
           << "  udp_port:            " << c.udp_port << "\n";
        return os;
    }
};

}
