#include "mios/panda/panda_body.hpp"
#include "mios/memory/memory.hpp"
#include "mios/control/franka_adapter.hpp"
#include "mirmi_cpp_utils/network/network.hpp"
#include "mirmi_cpp_utils/conversion/conversion.hpp"
#include "conv_vel2pose/conv_vel2pose_wrapper.hpp"

#include "spdlog/spdlog.h"

#include "franka/exception.h"
#include "pybind11/pybind11.h"
#include "pybind11/embed.h"
#include "pybind11_json/pybind11_json.hpp"
#include "pybind11/stl.h"

#include <thread>

namespace mios {

PandaBody::PandaBody(Memory *memory,const MiosContext &context):
    m_panda_arm(nullptr),
    m_panda_model(nullptr),
    m_panda_hand(nullptr),
    m_has_arm(false),
    m_hand(PandaHandNone),
    m_arm_connected(false),
    m_hand_connected(false),
    m_context(context),
    m_memory(memory){
        spdlog::trace("PandaBody::PandaBody");
        m_robot_state.O_T_EE={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
}

bool PandaBody::initialize(){
    spdlog::trace("PandaBody::initialize()");

    switch(m_context.config.robot_configuration){
        case 0:     
            m_has_arm=true;
            m_hand=PandaHand::PandaHandDefault;    // will be overwritten by ensure_robot_ready (deskapi)  
            break;
        case 1:     
            m_has_arm=true;
            m_hand=PandaHand::PandaHandNone;      // will be overwritten by ensure_robot_ready (deskapi)  
            break;
        case 2:     
            m_has_arm=true;
            m_hand=PandaHand::PandaHandSofthand2;      
            break;
        case 3:    
            m_has_arm=false;
            m_hand=PandaHand::PandaHandNone;     
            break;
        default:
            spdlog::error("Robot configuration " + std::to_string(m_context.config.robot_configuration) + " does not exist.");
            return false;
    }
    std::optional<std::string> ip;
    if(m_has_arm){
    //request control and activate FCI
        spdlog::debug("PandaBody::initialize()");
        ip = PandaBody::ping_robot(m_context.config.robot_ip);     
    }

    // m_context.config.robot_ip = ip.value_or("127.0.0.1");
    if(!connect_to_robot(m_context.config.robot_ip)){
        return false;
    }
    if(!connect_to_gripper(m_context.config.robot_ip)){
        return false;
    }
    load_gripper_configuration();

    franka::RobotState state;
    if(!get_robot_state(state)){
        spdlog::error("Could not acquire initial state from robot.");
        return false;
    }
    if(fabs(state.F_T_NE[14]-0.1034)>1e-4 && m_hand==PandaHand::PandaHandDefault){
        spdlog::warn("MIOS is configured with the default Franka Hand as end effector, but the robots end effector configuration (see Desk) is not consistent with this. However, this may be fine if a custom configuration has been selected.");
    }
    if(!set_robot_parameters()){
        spdlog::warn("Could not set robot parameters in current mode.");
    }
    return true;
}


std::optional<std::string> PandaBody::ping_robot(const std::optional<std::string> &last_ip){
    spdlog::trace("PandaBody::ping_robot("+last_ip.value()+")");
    std::optional<std::string> new_ip={};
    unsigned count=0;
    spdlog::debug("PandaBody: ping_robot("+last_ip.value_or("127.0.0.1")+")");
    //check given IP:
    while(!new_ip.has_value()){
        if(last_ip.has_value()){
            if(m_context.shutdown_signal){
                break;
            }
            if(is_service_reachable(last_ip.value().c_str(), 1337, 5)==false ){
                if(count%10000==0){
                    spdlog::warn("IP was set to "+last_ip.value()+" but no device has been found. Searching for new connection...");
                }  
            }else{
                if(is_robot(last_ip.value_or("127.0.0.1"))){ // 
                    new_ip=last_ip;
                    return new_ip;
                }
            }
        }
        count++;
    }
    // // search for robot IP:
    // if(!new_ip.has_value()){
    //     std::map<std::string,std::string> ifaces = mirmi_utils::get_subnets();
    //     std::string address;
    //     for(const auto& i : ifaces){
    //         if(i.first=="lo" || i.first=="docker0" || i.first=="tap0" || i.first=="flannel.1" || i.first.substr(0,3)=="enx" || i.first.substr(0,3)=="wlp" || i.first.substr(0,2)=="br" || i.first.substr(0,4)=="enp4"){
    //             continue;
    //         }
    //         for(unsigned j=2;j<255;j++){
    //             address=i.second +std::to_string(j);
    //             if(!mirmi_utils::ping(address.c_str())){
    //                 spdlog::debug("Cannot find device at "+address+" on interface "+i.first+".");
    //                 continue;
    //             }else{
    //                 spdlog::info("Found device at ip "+address+" at interface "+i.first+".");
    //                 if(is_robot(address)){  // 
    //                     new_ip = address;
    //                     return new_ip;
    //                 }
    //             }
    //         }
    //     }
    // }
    spdlog::warn("PandaBody::ping_robot: Cannot find Robot");
    return new_ip;

}

bool PandaBody::is_service_reachable(const std::string& address, int port, int timeout_sec) {
    struct addrinfo hints{}, *res, *ptr;
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;     // Allow IPv4 or IPv6
    hints.ai_socktype = SOCK_STREAM; // TCP

    // 1. Resolve Hostname
    if (getaddrinfo(address.c_str(), std::to_string(port).c_str(), &hints, &res) != 0) {
        return false;
    }

    bool is_connected = false;

    // 2. Iterate through ALL resolved addresses (Crucial for K8s)
    for (ptr = res; ptr != nullptr; ptr = ptr->ai_next) {
        int sockfd = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
        if (sockfd < 0) continue;

        // 3. Set Non-Blocking (FIXED SYNTAX)
        int flags = fcntl(sockfd, F_GETFL, 0);
        if (flags == -1 || fcntl(sockfd, F_SETFL, flags | O_NONBLOCK) == -1) {
            close(sockfd);
            continue;
        }

        // 4. Attempt Connection
        int connect_result = connect(sockfd, ptr->ai_addr, ptr->ai_addrlen);

        if (connect_result == 0) {
            is_connected = true; // Connected immediately
        } 
        else if (errno == EINPROGRESS) {
            // Connection in progress, wait using poll
            struct pollfd pfd{};
            pfd.fd = sockfd;
            pfd.events = POLLOUT;

            // poll expects milliseconds
            int poll_res = poll(&pfd, 1, timeout_sec * 1000); 

            if (poll_res > 0) {
                // Poll succeeded, check socket error state
                int so_error = 0;
                socklen_t len = sizeof(so_error);
                if (getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &so_error, &len) == 0) {
                    if (so_error == 0) {
                        is_connected = true;
                    }
                }
            }
        }

        close(sockfd); 
        
        if (is_connected) {
            break; // Stop trying other IPs if we found a working one
        }
    }

    freeaddrinfo(res);
    return is_connected;
}

void PandaBody::load_gripper_configuration(){
    spdlog::trace("PandaBody::load_gripper_configuration()");
    if(m_hand==PandaHandDefault){
        m_memory->get_parameters()->frames.F_T_EE<<0.7071,-0.7071,0,0,0.7071,0.7071,0,0,0,0,1,0,0,0,0.1034,1;
        m_memory->get_parameters()->frames.F_T_EE.transposeInPlace();
    }
    if(m_hand==PandaHandSofthand2){
        m_memory->get_parameters()->frames.F_T_EE.setIdentity();
    }
}

bool PandaBody::connect_to_robot(const std::optional<std::string> &ip){
    spdlog::trace("PandaBody::connect_to_robot()");
    if(!m_has_arm){
        m_arm_connected=false;
        return true;
    }
    if(!ip.has_value()){
        m_arm_connected=false;
        return false;
    }
    try{
        m_panda_arm = std::make_unique<franka::Robot>(ip.value());
        m_panda_model = std::make_unique<franka::Model>(m_panda_arm->loadModel());
        m_arm_connected=true;
        return true;
    }catch(const franka::NetworkException& e){
        spdlog::debug(e.what());
        spdlog::error("Cannot connect to robot at IP "+ip.value());
        return false;
    }catch(const franka::IncompatibleVersionException& e){
        spdlog::debug(e.what());
        spdlog::error("Panda: Detected incompatible version on robot at IP "+ip.value());
        return false;
    }catch(const franka::ModelException& e){
        spdlog::debug(e.what());
        spdlog::error("Panda: Model could not be loaded.");
        return false;
    }
}

bool PandaBody::connect_to_gripper(const std::optional<std::string> &ip){
    spdlog::trace("PandaBody::connect_to_gripper()");
    if(m_hand==PandaHandNone){
        m_hand_connected=false;
        return true;
    }
    if(m_hand==PandaHandDefault){
        if(!ip.has_value()){
            m_hand_connected=false;
            return false;
        }
        try{
            m_panda_hand = std::make_unique<franka::Gripper>(ip.value());
            m_hand_connected=true;
            return true;
        }catch(const franka::NetworkException& e){
            spdlog::debug(e.what());
            spdlog::error("Can not connect to gripper at IP " + ip.value());
            return false;
        }catch(const franka::IncompatibleVersionException& e){
            spdlog::debug(e.what());
            spdlog::error("Panda: Detected incompatible version on robot at IP "+ip.value());
            return false;
        }
    }
    if(m_hand==PandaHandSofthand2){
        m_softhand2 = std::make_unique<Softhand2>();
        if(!m_softhand2->initialize()){
            return false;
        }else{
            m_hand_connected=true;
            return true;
        }
    }
    return false;
}

void PandaBody::disconnect_from_robot(){
    spdlog::trace("PandaBody::disconnect_from_robot()");
    m_arm_connected=false;
    m_panda_model.release();
    m_panda_arm.release();
}

void PandaBody::disconnect_from_gripper(){
    spdlog::trace("PandaBody::disconnect_from_gripper()");
    m_hand_connected=false;
    if(m_hand==PandaHandDefault){
        m_panda_hand.release();
    }
    if(m_hand==PandaHandSofthand2){
        m_softhand2.release();
    }
}

ControlReturnType PandaBody::recover(){
    spdlog::trace("PandaBody::recover()");
    if(!m_arm_connected){
        return {false,"None",""};
    }
    try{
        m_panda_arm->automaticErrorRecovery();
        return {false,"None",""};
    }catch(const franka::NetworkException& e){
        spdlog::debug(e.what());
        return {true,"NetworkException",""};
    }catch(const franka::CommandException& e){
        spdlog::debug(e.what());
        return {true,"CommandException",""};
    }
}

bool PandaBody::pre_run_checks() const{
    spdlog::trace("PandaBody::pre_run_checks()");
    franka::RobotState state;
    if(!get_robot_state(state)){
        return false;
    }
    if(state.robot_mode!=franka::RobotMode::kIdle){
        return false;
    }
    return true;
}

bool PandaBody::is_robot(const std::string &ip){
    spdlog::debug("PandaBody::is_robot("+ip+")" );
    bool result = false;
    while(!result && m_context.config.use_desk && !m_context.shutdown_signal){
        result = ensure_robot_ready();
    }
    m_memory->set_default_parameters();
    return true;
}

// currently not used:
std::optional<std::string> PandaBody::find_robot(){
    spdlog::trace("PandaBody::find_robot()");
    std::optional<std::string> robot_address={};
    std::string robot_iface="none";

    std::map<std::string,std::string> ifaces = mirmi_utils::get_subnets();
    for(const auto& i : ifaces){
        if(i.first=="lo" || i.first=="docker0" || i.first=="tap0" || i.first=="flannel.1" || i.first.substr(0,3)=="enx" || i.first.substr(0,3)=="wlp" || i.first.substr(0,2)=="br"){
            continue;
        }
        for(unsigned j=1;j<255;j++){
            std::string address=i.second+std::to_string(j);
            if(!mirmi_utils::ping(address.c_str())){
                continue;
            }else{
                if(is_robot(address)){ 
                    robot_address=address;
                    break;
                }
            }
        }
        if(robot_address.has_value()){
            robot_iface=i.first;
            break;
        }
    }

    if(!robot_address.has_value()){
        spdlog::error("No connected robot found. Make sure that master controller and this computer share the same network and that the connection is properly configured.");
    }else{
        spdlog::info("Found robot at ip "+robot_address.value()+" at interface "+robot_iface+".");
    }
    return robot_address;
}

ControlReturnType PandaBody::control(std::function<franka::Torques (const franka::RobotState&,franka::Duration)> controller_callback){
    spdlog::trace("PandaBody::control(Torques)");
    if(m_arm_connected){
        try{
            m_panda_arm->control(controller_callback);
            return {false,"None",""};
        }catch(const franka::ControlException& e){
            spdlog::debug(e.what());
            return {true,"ControlException",e.what()};
        }catch(const franka::InvalidOperationException& e){
            spdlog::debug(e.what());
            return {true,"InvalidOperationException",e.what()};
        }catch(const franka::NetworkException& e){
            spdlog::debug(e.what());
            return {true,"NetworkException",e.what()};
        }catch(const franka::RealtimeException& e){
            spdlog::debug(e.what());
            return {true,"RealtimeException",e.what()};
        }catch(const std::invalid_argument& e){
            spdlog::debug(e.what());
            return {true,"invalid_argument",e.what()};
        }
    }else{
        dummy_control(controller_callback);
        return {false,"None",""};
    }
}

ControlReturnType PandaBody::control(std::function<franka::CartesianVelocities (const franka::RobotState &,franka::Duration)> controller_callback){
    spdlog::trace("PandaBody::control(CartesianVelocities)");
    if(m_arm_connected){
        try{
            m_panda_arm->control(controller_callback);
            return {false,"None",""};
        }catch(const franka::ControlException& e){
            spdlog::debug(e.what());
            return {true,"ControlException",e.what()};
        }catch(const franka::InvalidOperationException& e){
            spdlog::debug(e.what());
            return {true,"InvalidOperationException",e.what()};
        }catch(const franka::NetworkException& e){
            spdlog::debug(e.what());
            return {true,"NetworkException",e.what()};
        }catch(const franka::RealtimeException& e){
            spdlog::debug(e.what());
            return {true,"RealtimeException",e.what()};
        }catch(const std::invalid_argument& e){
            spdlog::debug(e.what());
            return {true,"invalid_argument",e.what()};
        }
    }else{
        dummy_control(controller_callback);
        return {false,"None",""};
    }
}

ControlReturnType PandaBody::control(std::function<franka::JointVelocities (const franka::RobotState &,franka::Duration)> controller_callback){
    spdlog::trace("PandaBody::control(JointVelocities)");
    if(m_arm_connected){
        try{
            m_panda_arm->control(controller_callback);
            return {false,"None",""};
        }catch(const franka::ControlException& e){
            spdlog::debug(e.what());
            return {true,"ControlException",e.what()};
        }catch(const franka::InvalidOperationException& e){
            spdlog::debug(e.what());
            return {true,"InvalidOperationException",e.what()};
        }catch(const franka::NetworkException& e){
            spdlog::debug(e.what());
            return {true,"NetworkException",e.what()};
        }catch(const franka::RealtimeException& e){
            spdlog::debug(e.what());
            return {true,"RealtimeException",e.what()};
        }catch(const std::invalid_argument& e){
            spdlog::debug(e.what());
            return {true,"invalid_argument",e.what()};
        }
    }else{
        dummy_control(controller_callback);
        return {false,"None",""};
    }
}

ControlReturnType PandaBody::control(std::function<franka::CartesianPose (const franka::RobotState &,franka::Duration)> controller_callback){
    spdlog::trace("PandaBody::control(CartesianPose)");
    if(m_arm_connected){
        try{
            m_panda_arm->control(controller_callback);
            return {false,"None",""};
        }catch(const franka::ControlException& e){
            spdlog::debug(e.what());
            return {true,"ControlException",e.what()};
        }catch(const franka::InvalidOperationException& e){
            spdlog::debug(e.what());
            return {true,"InvalidOperationException",e.what()};
        }catch(const franka::NetworkException& e){
            spdlog::debug(e.what());
            return {true,"NetworkException",e.what()};
        }catch(const franka::RealtimeException& e){
            spdlog::debug(e.what());
            return {true,"RealtimeException",e.what()};
        }catch(const std::invalid_argument& e){
            spdlog::debug(e.what());
            return {true,"invalid_argument",e.what()};
        }
    }else{
        dummy_control(controller_callback);
        return {false,"None",""};
    }
}

ControlReturnType PandaBody::control(std::function<franka::JointPositions (const franka::RobotState &,franka::Duration)> controller_callback){
    spdlog::trace("PandaBody::control(JointPositions)");
    if(m_arm_connected){
        try{
            m_panda_arm->control(controller_callback);
            return {false,"None",""};
        }catch(const franka::ControlException& e){
            spdlog::debug(e.what());
            return {true,"ControlException",e.what()};
        }catch(const franka::InvalidOperationException& e){
            spdlog::debug(e.what());
            return {true,"InvalidOperationException",e.what()};
        }catch(const franka::NetworkException& e){
            spdlog::debug(e.what());
            return {true,"NetworkException",e.what()};
        }catch(const franka::RealtimeException& e){
            spdlog::debug(e.what());
            return {true,"RealtimeException",e.what()};
        }catch(const std::invalid_argument& e){
            spdlog::debug(e.what());
            return {true,"invalid_argument",e.what()};
        }
    }else{
        dummy_control(controller_callback);
        return {false,"None",""};
    }
}

ControlReturnType PandaBody::control(control::CommandMode mode, ControlCallback controller_callback) {
    const auto period_seconds = [](const franka::Duration& duration) { return duration.toSec(); };
    switch (mode) {
        case control::CommandMode::kTorque:
        {
            std::function<franka::Torques(const franka::RobotState&, franka::Duration)> bridge =
                [this, controller_callback, period_seconds](const franka::RobotState& state,
                                                            franka::Duration duration) {
                    const control::RobotModel model =
                        m_panda_model ? control::from_franka_model(*m_panda_model, state)
                                      : control::RobotModel{};
                    const control::GripperState gripper_state =
                        control::from_franka_state(m_gripper_state);
                    return control::to_franka_torques(controller_callback(
                        control::from_franka_state(state), model, gripper_state,
                        period_seconds(duration)));
                };
            return control(std::move(bridge));
        }
        case control::CommandMode::kJointVelocity:
        {
            std::function<franka::JointVelocities(const franka::RobotState&, franka::Duration)>
                bridge = [this, controller_callback, period_seconds](const franka::RobotState& state,
                                                                      franka::Duration duration) {
                    const control::RobotModel model =
                        m_panda_model ? control::from_franka_model(*m_panda_model, state)
                                      : control::RobotModel{};
                    const control::GripperState gripper_state =
                        control::from_franka_state(m_gripper_state);
                    return control::to_franka_joint_velocities(controller_callback(
                        control::from_franka_state(state), model, gripper_state,
                        period_seconds(duration)));
                };
            return control(std::move(bridge));
        }
        case control::CommandMode::kCartesianVelocity:
        {
            std::function<franka::CartesianVelocities(const franka::RobotState&, franka::Duration)>
                bridge = [this, controller_callback, period_seconds](const franka::RobotState& state,
                                                                      franka::Duration duration) {
                    const control::RobotModel model =
                        m_panda_model ? control::from_franka_model(*m_panda_model, state)
                                      : control::RobotModel{};
                    const control::GripperState gripper_state =
                        control::from_franka_state(m_gripper_state);
                    return control::to_franka_cartesian_velocities(controller_callback(
                        control::from_franka_state(state), model, gripper_state,
                        period_seconds(duration)));
                };
            return control(std::move(bridge));
        }
        default:
            spdlog::error("PandaBody::control: command mode is not supported by the legacy adapter");
            return {true, "UnsupportedCommandMode", ""};
    }
}

void PandaBody::dummy_control(std::function<franka::Torques (const franka::RobotState &,franka::Duration)> controller_callback){
    spdlog::trace("PandaBody::dummy_control(Torques)");
    franka::Torques tau_J={0,0,0,0,0,0,0};

    m_robot_state.K_F_ext_hat_K={0,0,0,0,0,0};
    m_robot_state.O_F_ext_hat_K={0,0,0,0,0,0};
    m_robot_state.tau_ext_hat_filtered={0,0,0,0,0,0,0};
    m_robot_state.dq={0,0,0,0,0,0,0};
    std::chrono::high_resolution_clock::time_point t_0;
    franka::Duration duration(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-t_0));
    while(!tau_J.motion_finished){
        auto t_s_start = std::chrono::system_clock::now();
        tau_J=controller_callback(m_robot_state,duration);
        auto t_s_end = std::chrono::system_clock::now();
        double t=std::chrono::duration_cast<std::chrono::microseconds>(t_s_end-t_s_start).count();
        duration=franka::Duration(t);
        std::this_thread::sleep_for(std::chrono::microseconds(1000-static_cast<int>(t)));
    }
}

void PandaBody::dummy_control(std::function<franka::CartesianVelocities (const franka::RobotState &,franka::Duration)> controller_callback){
    spdlog::trace("PandaBody::dummy_control(CartesianVelocities)");
    franka::CartesianVelocities dX_d={0,0,0,0,0,0};
    m_robot_state.K_F_ext_hat_K={0,0,0,0,0,0};
    m_robot_state.O_F_ext_hat_K={0,0,0,0,0,0};
    m_robot_state.tau_ext_hat_filtered={0,0,0,0,0,0,0};
    m_robot_state.dq={0,0,0,0,0,0,0};
    std::chrono::high_resolution_clock::time_point t_0;
    franka::Duration duration(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-t_0));
    conv_vel2pose::conv_vel2pose conv;
    conv.u.TF_T_EE=Eigen::Matrix<double,4,4>(m_robot_state.O_T_EE.data());
    conv.u.TF_dX_d.setZero();
    conv.initialize();
    while(!dX_d.motion_finished){
        auto t_s_start = std::chrono::system_clock::now();
        dX_d=controller_callback(m_robot_state,duration);
        conv.u.TF_T_EE=Eigen::Matrix<double,4,4>(m_robot_state.O_T_EE.data());
        conv.u.TF_dX_d=Eigen::Matrix<double,6,1>(dX_d.O_dP_EE.data());
        conv.step();
        m_robot_state.O_T_EE=mirmi_utils::convert_to_array<double,4,4>(conv.y.TF_T_EE_d);
        auto t_s_end = std::chrono::system_clock::now();
        double t=std::chrono::duration_cast<std::chrono::microseconds>(t_s_end-t_s_start).count();
        duration=franka::Duration(t);
        std::this_thread::sleep_for(std::chrono::microseconds(1000-static_cast<int>(t)));
    }
    conv.terminate();
}

void PandaBody::dummy_control(std::function<franka::JointVelocities (const franka::RobotState &,franka::Duration)> controller_callback){
    spdlog::trace("PandaBody::dummy_control(JointVelocities)");
    franka::JointVelocities dq_d={0,0,0,0,0,0,0};
    m_robot_state.K_F_ext_hat_K={0,0,0,0,0,0};
    m_robot_state.O_F_ext_hat_K={0,0,0,0,0,0};
    m_robot_state.tau_ext_hat_filtered={0,0,0,0,0,0,0};
    m_robot_state.dq={0,0,0,0,0,0,0};
    std::chrono::high_resolution_clock::time_point t_0;
    franka::Duration duration(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-t_0));
    while(!dq_d.motion_finished){
        auto t_s_start = std::chrono::system_clock::now();
        dq_d=controller_callback(m_robot_state,duration);
        for(unsigned i=0;i<7;i++){
            m_robot_state.q[i]+=dq_d.dq[i]*0.001;
        }
        m_robot_state.dq=dq_d.dq;
        auto t_s_end = std::chrono::system_clock::now();
        double t=std::chrono::duration_cast<std::chrono::microseconds>(t_s_end-t_s_start).count();
        duration=franka::Duration(t);
        std::this_thread::sleep_for(std::chrono::microseconds(1000-static_cast<int>(t)));
    }
}

void PandaBody::dummy_control(std::function<franka::CartesianPose (const franka::RobotState &,franka::Duration)> controller_callback){
    spdlog::trace("PandaBody::dummy_control(CartesianPose)");
    franka::CartesianPose O_T_EE_d={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
    m_robot_state.K_F_ext_hat_K={0,0,0,0,0,0};
    m_robot_state.O_F_ext_hat_K={0,0,0,0,0,0};
    m_robot_state.tau_ext_hat_filtered={0,0,0,0,0,0,0};
    m_robot_state.dq={0,0,0,0,0,0,0};
    std::chrono::high_resolution_clock::time_point t_0;
    franka::Duration duration(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-t_0));
    while(!O_T_EE_d.motion_finished){
        auto t_s_start = std::chrono::system_clock::now();
        O_T_EE_d=controller_callback(m_robot_state,duration);
        m_robot_state.O_T_EE=O_T_EE_d.O_T_EE;
        auto t_s_end = std::chrono::system_clock::now();
        double t=std::chrono::duration_cast<std::chrono::microseconds>(t_s_end-t_s_start).count();
        duration=franka::Duration(t);
        std::this_thread::sleep_for(std::chrono::microseconds(1000-static_cast<int>(t)));
    }
}

void PandaBody::dummy_control(std::function<franka::JointPositions (const franka::RobotState &,franka::Duration)> controller_callback){
    spdlog::trace("PandaBody::dummy_control(JointPositions)");
    franka::JointPositions q_d={0,0,0,0,0,0,0};
    franka::RobotState robot_state;
    robot_state.K_F_ext_hat_K={0,0,0,0,0,0};
    robot_state.O_F_ext_hat_K={0,0,0,0,0,0};
    robot_state.tau_ext_hat_filtered={0,0,0,0,0,0,0};
    robot_state.dq={0,0,0,0,0,0,0};
    std::chrono::high_resolution_clock::time_point t_0;
    franka::Duration duration(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-t_0));
    while(!q_d.motion_finished){
        auto t_s_start = std::chrono::system_clock::now();
        q_d=controller_callback(robot_state,duration);
        robot_state.q=q_d.q;
        auto t_s_end = std::chrono::system_clock::now();
        double t=std::chrono::duration_cast<std::chrono::microseconds>(t_s_end-t_s_start).count();
        duration=franka::Duration(t);
        std::this_thread::sleep_for(std::chrono::microseconds(1000-static_cast<int>(t)));
    }
}

bool PandaBody::set_robot_parameters(){
    spdlog::trace("PandaBody::set_robot_parameters");
    if(!m_arm_connected){
        return true;
    }
    ControlParameters control= m_memory->read_parameters()->control;
    UserParameters user= m_memory->read_parameters()->user;
    FramesParameters frames =m_memory->read_parameters()->frames;

    std::array<double,3> load_com = mirmi_utils::convert_to_array<double,3,1>(user.load_com);
    std::array<double,9> load_I = mirmi_utils::convert_to_array<double,3,3>(user.load_I);
    std::array<double,7> tau_ext_contact = mirmi_utils::convert_to_array<double,7,1>(user.tau_ext_contact);
    std::array<double,7> tau_ext_max = mirmi_utils::convert_to_array<double,7,1>(user.tau_ext_max);
    std::array<double,6> F_ext_K_contact = {user.F_ext_contact(0),user.F_ext_contact(0),user.F_ext_contact(0),user.F_ext_contact(1),user.F_ext_contact(1),user.F_ext_contact(1)};
    std::array<double,6> F_ext_K_max = {user.F_ext_max(0),user.F_ext_max(0),user.F_ext_max(0),user.F_ext_max(1),user.F_ext_max(1),user.F_ext_max(1)};
    std::array<double,16> EE_T_K = mirmi_utils::convert_to_array<double,4,4>(frames.EE_T_K);
    std::array<double,7> K_theta = mirmi_utils::convert_to_array<double,7,1>(control.joint_imp.K_theta);
    std::array<double,6> K_x = mirmi_utils::convert_to_array<double,6,1>(control.cart_imp.K_x);
    std::array<double,16> NE_T_TCP = mirmi_utils::convert_to_array<double,4,4>(frames.EE_T_TCP);

    try{
        m_panda_arm->setLoad(user.load_m,load_com,load_I);
        m_panda_arm->setEE(NE_T_TCP);
        m_panda_arm->setCollisionBehavior(tau_ext_contact,tau_ext_max,F_ext_K_contact,F_ext_K_max);
        m_panda_arm->setK(EE_T_K);
        m_panda_arm->setCartesianImpedance(K_x);
        m_panda_arm->setJointImpedance(K_theta);
    }catch(franka::CommandException& e){
        spdlog::debug(e.what());
        return false;
    }catch(franka::NetworkException& e){
        spdlog::debug(e.what());
        return false;
    }
    return true;
}

bool PandaBody::set_load(double load_m, std::array<double, 3> load_com, std::array<double, 9> load_I){
    spdlog::trace("PandaBody::set_load");
    try{
        m_panda_arm->setLoad(load_m,load_com,load_I);
        return true;
    }catch(franka::CommandException& e){
        spdlog::debug(e.what());
        return false;
    }catch(franka::NetworkException& e){
        spdlog::debug(e.what());
        return false;
    }
}

bool PandaBody::set_ee(std::array<double, 16> F_T_EE){
    spdlog::trace("PandaBody::set_ee");
    try{
        m_panda_arm->setEE(F_T_EE);
        return true;
    }catch(franka::CommandException& e){
        spdlog::debug(e.what());
        return false;
    }catch(franka::NetworkException& e){
        spdlog::debug(e.what());
        return false;
    }
}

Eigen::Matrix4d PandaBody::dhTransformationMatrix(double theta, double d, double a, double alpha) const {
    Eigen::Matrix4d T;

    T << cos(theta),                 -sin(theta),                 0,               a,
         sin(theta) * cos(alpha),    cos(theta) * cos(alpha),    -sin(alpha),     -d * sin(alpha),
         sin(theta) * sin(alpha),    cos(theta) * sin(alpha),     cos(alpha),      d * cos(alpha),
         0,                          0,                           0,               1;

    return T;
}

std::array<double, 16> PandaBody::forward_kinematics(franka::RobotState& state) const{
    spdlog::trace("PandaBody::forward_kinematics");
    Eigen::Matrix<double, 8, 1>d = m_memory->get_parameters()->user.DH_d;
    Eigen::Matrix<double, 8, 1> a = m_memory->get_parameters()->user.DH_a;
    Eigen::Matrix<double, 8, 1> alpha = m_memory->get_parameters()->user.DH_alpha;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    for (int i = 0; i < 7; ++i) {
        Eigen::Matrix4d deltaT = dhTransformationMatrix(state.q[i], d[i], a[i], alpha[i]);
        T = T * deltaT;
    }

    Eigen::Matrix4d deltaT = dhTransformationMatrix(-M_PI / 4, d[7], a[7], alpha[7]);
    T = T * deltaT;
    
    std::array<double, 16> O_T_EE;

    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            O_T_EE[row * 4 + col] = T(row, col);
        }
    }

    return O_T_EE;

}

// std::array<double, 16> PandaBody::get_WF_T_EE(franka::RobotState& state) const{
//     Eigen::Matrix4d WF_T_O;
//     Eigen::Matrix4d O_T_EE;
//     Eigen::Matrix4d WF_T_EE;
//     WF_T_O = m_memory->get_parameters()->frames.WF_T_O;
//     O_T_EE = Eigen::Map<Eigen::Matrix4d>(state.O_T_EE.data());

//     WF_T_EE=WF_T_O*O_T_EE;

//     std::array<double, 16> _WF_T_EE;

//     for (int row = 0; row < 4; ++row) {
//         for (int col = 0; col < 4; ++col) {
//             _WF_T_EE[row * 4 + col] = WF_T_EE(row, col);
//         }
//     }

//     return _WF_T_EE;
// }

// std::array<double, 16> PandaBody::get_TF_T_EE(franka::RobotState& state) const{
//     Eigen::Matrix4d WF_T_TF;
//     Eigen::Matrix4d WF_T_EE;
//     Eigen::Matrix4d TF_T_EE;
//     WF_T_TF<<m_memory->get_parameters()->frames.WF_T_TF;
//     O_T_EE = Eigen::Map<Eigen::Matrix4d>(state.WF_T_EE.data());

//     TF_T_EE=WF_T_TF.inverse()*WF_T_EE;

//     std::array<double, 16> _TF_T_EE;

//     for (int row = 0; row < 4; ++row) {
//         for (int col = 0; col < 4; ++col) {
//             _TF_T_EE[row * 4 + col] = TF_T_EE(row, col);
//         }
//     }
//     return _TF_T_EE;
// }


bool PandaBody::get_robot_state(franka::RobotState &state) const{
    spdlog::trace("PandaBody::get_robot_state");
    if(m_arm_connected){
        try{
            state=m_panda_arm->readOnce();

            //use clibrated kinematics
            //order matters!
            //state.O_T_EE=forward_kinematics(state);
            //state.WF_T_EE=get_WF_T_EE(state);
            //state.TF_T_EE=get_TF_T_EE(state);
            return true;
        }catch(const franka::InvalidOperationException& e){
            spdlog::debug(e.what());
            return false;
        }catch(const franka::NetworkException& e){
            spdlog::debug(e.what());
            return false;
        }
    }else{
        get_default_robot_state(state);
        //use clibrated kinematics
        //state.O_T_EE=PandaBody::forward_kinematics(state);
        return true;
    }
}

bool PandaBody::get_gripper_state(franka::GripperState &state) const{
//    spdlog::trace("PandaBody::get_gripper_state");
    if(m_hand_connected && m_hand==PandaHandDefault){
        try{
            state=m_panda_hand->readOnce();
            return true;
        }catch(const franka::InvalidOperationException& e){
            spdlog::debug(e.what());
            return false;
        }catch(const franka::NetworkException& e){
            spdlog::debug(e.what());
            return false;
        }
    }else{
        get_default_gripper_state(state);
        return true;
    }
}

const std::unique_ptr<franka::Model>& PandaBody::get_panda_model() const{
    return m_panda_model;
}

bool PandaBody::get_robot_snapshot(control::RobotState& robot_state,
                                   control::RobotModel& robot_model,
                                   control::GripperState& gripper_state) const {
    franka::RobotState native_robot_state;
    franka::GripperState native_gripper_state;
    if (!get_robot_state(native_robot_state) || !get_gripper_state(native_gripper_state)) {
        return false;
    }
    robot_state = control::from_franka_state(native_robot_state);
    gripper_state = control::from_franka_state(native_gripper_state);
    if (m_panda_model) {
        robot_model = control::from_franka_model(*m_panda_model, native_robot_state);
    } else {
        // Preserve the legacy no-arm/dummy configuration. It has no dynamics
        // model, but it can still provide a deterministic neutral percept.
        robot_model = {};
    }
    return true;
}

bool PandaBody::grasp(double width, double speed, double force, double epsilon_inner, double epsilon_outer) const{
    spdlog::trace("PandaBody::grasp");
    std::scoped_lock<std::mutex> lock(m_mtx_hand_active);
    if(!m_hand_connected){
        spdlog::error("No gripper connected.");
        return false;
    }
    if(m_hand==PandaHandDefault){
        try{
            franka::GripperState state = m_panda_hand->readOnce();
            double max_width=state.max_width;
            double current_width=state.width;

            if(width<0 || width>max_width){
                spdlog::error("Gripper cannot reach width of "+std::to_string(width)+". Must be between 0 and "+std::to_string(max_width)+".");
                return false;
            }
            /*
            if(width>=current_width){
                spdlog::error("Grasping to a width larger than the current width is not possible.");
                spdlog::debug("Current width is " + std::to_string(current_width) + ", desired width is " + std::to_string(width));
                return false;
            }
            */

            if(state.is_grasped){
                spdlog::error("I am already grasping something.");
                return false;
            }
            return this->m_panda_hand->grasp(width,speed,force,epsilon_inner,epsilon_outer);
        }catch(franka::CommandException& e){
            spdlog::debug(e.what());
            return false;
        }catch(franka::NetworkException& e){
            spdlog::debug(e.what());
            return false;
        }catch(franka::InvalidOperationException& e){
            spdlog::debug(e.what());
            return false;
        }
    }
    if(m_hand==PandaHandSofthand2){
        return m_softhand2->move(width);
    }
    return false;
}

bool PandaBody::move_to_finger_position(double width, double speed) const{
    spdlog::trace("PandaBody::move_to_finger_position");
    std::scoped_lock<std::mutex> lock(m_mtx_hand_active);
    if(!m_hand_connected){
        return false;
    }
    if(m_hand==PandaHandDefault){

        try{
            franka::GripperState state = m_panda_hand->readOnce();
            double max_width=state.max_width;

            if(width<0 || width>max_width){
                spdlog::warn("Gripper cannot reach width of "+std::to_string(width)+". Must be between 0 and "+std::to_string(max_width)+".");
                if(width<0){
                    width=0;
                }
                if(width>max_width){
                    width=max_width;
                }
            }
            return this->m_panda_hand->move(width,speed);
        }catch(franka::CommandException& e){
            spdlog::debug(e.what());
            return false;
        }catch(franka::NetworkException& e){
            spdlog::debug(e.what());
            return false;
        }catch(franka::InvalidOperationException& e){
            spdlog::debug(e.what());
            return false;
        }
    }
    if(m_hand==PandaHandSofthand2){
        return m_softhand2->move(width);
    }
    return false;
}

bool PandaBody::stop_gripper(){
    spdlog::trace("PandaBody::stop_gripper");
//    std::scoped_lock<std::mutex> lock(m_mtx_hand_active);
    if(!m_hand_connected){
        return false;
    }
    if(m_hand==PandaHandDefault){
        try{
            return this->m_panda_hand->stop();
        }catch(franka::CommandException& e){
            spdlog::debug(e.what());
            return false;
        }catch(franka::NetworkException& e){
            spdlog::debug(e.what());
            return false;
        }
    }
    if(m_hand==PandaHandSofthand2){
        return true;
    }
    return false;
}

bool PandaBody::home_gripper() const{
    spdlog::trace("PandaBody::home_gripper");
    std::scoped_lock<std::mutex> lock(m_mtx_hand_active);
    if(!m_hand_connected){
        return false;
    }
    if(m_hand==PandaHandDefault){

        try{
            return this->m_panda_hand->homing();
        }catch(franka::CommandException& e){
            spdlog::debug(e.what());
            return false;
        }catch(franka::NetworkException& e){
            spdlog::debug(e.what());
            return false;
        }
    }
    if(m_hand==PandaHandSofthand2){
        return true;
    }
    return false;
}

bool PandaBody::is_hand_active(){
    if(m_mtx_hand_active.try_lock()){
        m_mtx_hand_active.unlock();
        return false;
    }else{
        return true;
    }
}

void PandaBody::get_default_robot_state(franka::RobotState &state) const{
    spdlog::trace("PandaBody::get_default_robot_state");
    state=m_robot_state;
    state.robot_mode=franka::RobotMode::kIdle;
}

void PandaBody::get_default_gripper_state(franka::GripperState &state) const{
    spdlog::trace("PandaBody::get_default_gripper_state");
    state=m_gripper_state;
}


// deskapi functions:

bool PandaBody::activate_fci(){
    spdlog::trace("PandaBody::activate_fci()");
    if(!m_context.config.use_desk){
        spdlog::error("DESK features are not activated.");
        return false;
    }
    bool result;
    try{
        pybind11::module deskapi = pybind11::module::import("deskapi");
        pybind11::object py_result = deskapi.attr("activate_fci")();
        py::tuple result_tuple = py_result.cast<py::tuple>();
        result = result_tuple[0].cast<bool>();
        nlohmann::json status_json = result_tuple[1].cast<nlohmann::json>();
        if(!result){
            spdlog::error("Cannot activate FCI through Desk client: "+status_json.dump());
        }
    }catch(const pybind11::error_already_set& e){
        spdlog::debug(e.what());
        spdlog::warn("Cannot activate FCI, error when calling the python desk client.");
        return false;
    }
    if(result){
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return true;
}

bool PandaBody::deactivate_fci(){
    spdlog::trace("PandaBody::deactivate_fci()");
    if(!m_context.config.use_desk){
        spdlog::error("DESK features are not activated.");
        return false;
    }
    bool result;
    try{
        pybind11::module deskapi = pybind11::module::import("deskapi");
        pybind11::object py_result = deskapi.attr("deactivate_fci")();
        py::tuple result_tuple = py_result.cast<py::tuple>();
        result = result_tuple[0].cast<bool>();
        nlohmann::json status_json = result_tuple[1].cast<nlohmann::json>();
        if(!result){
            spdlog::error("Cannot deactivate FCI through Desk client: "+status_json.dump());
        }

    }catch(const pybind11::error_already_set& e){
        spdlog::debug(e.what());
        spdlog::warn("Cannot deactivate FCI, error when calling the python desk client.");
        return false;
    }
    if(result){
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return true;
}

bool PandaBody::shutdown_robot(){
    spdlog::trace("PandaBody::shutdown_robot");
    if(!m_context.config.use_desk){
        spdlog::error("DESK features are not activated.");
        return false;
    }
    bool result;
    try{
        pybind11::module deskapi = pybind11::module::import("deskapi");
        pybind11::object py_result = deskapi.attr("shutdown_system")();
        py::tuple result_tuple = py_result.cast<py::tuple>();
        result = result_tuple[0].cast<bool>();
        nlohmann::json status_json = result_tuple[1].cast<nlohmann::json>();
        if(!result){
            spdlog::error("Cannot shutdown robot through Desk client: "+status_json.dump());
        }
    }catch(const pybind11::error_already_set& e){
        spdlog::debug(e.what());
        spdlog::warn("Cannot shutdown, error when calling the python desk client.");
        result=false;
    }
    if(result){
        spdlog::info("Shutting down Robot... Wait until re-initialising.");
        std::this_thread::sleep_for(std::chrono::seconds(5));
        disconnect_from_gripper();
        disconnect_from_robot();
        std::this_thread::sleep_for(std::chrono::seconds(5));
        result=false;
        while(!this->initialize()){
            spdlog::warn("Robot initialization failed. Wait and retry...");
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }   
    }
    return result;
}

bool PandaBody::reboot_robot(){
    spdlog::trace("PandaBody::reboot_robot");
    if(!m_context.config.use_desk){
        spdlog::error("DESK features are not activated.");
        return false;
    }
    //deactivate_fci();
    bool result;
    try{
        pybind11::module deskapi = pybind11::module::import("deskapi");
        pybind11::object py_result = deskapi.attr("reboot_system")();
        py::tuple result_tuple = py_result.cast<py::tuple>();
        result = result_tuple[0].cast<bool>();
        nlohmann::json status_json = result_tuple[1].cast<nlohmann::json>();
        if(!result){
            spdlog::error("Cannot reboot robot through Desk client: "+status_json.dump());
        }
    }catch(const pybind11::error_already_set& e){
        spdlog::debug(e.what());
        spdlog::warn("Cannot reboot, error when calling the python desk client.");
        result=false;
    }
    if(result){
        spdlog::info("Rebooting Robot... Wait until re-initialising.");
        std::this_thread::sleep_for(std::chrono::seconds(5));
        disconnect_from_gripper();
        disconnect_from_robot();
        std::this_thread::sleep_for(std::chrono::seconds(5));
        result=false;
        while(!this->initialize()){
            spdlog::warn("Robot initialization failed. Wait and retry...");
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }

    }
    return result;
}

bool PandaBody::unlock_brakes(){  // make sure it is waiting until all brakes are unlocked
    spdlog::trace("PandaBody::unlock_brakes");
    if(!m_context.config.use_desk){
        spdlog::error("DESK features are not activated.");
        return false;
    }
    bool result;
    try{
        pybind11::module deskapi = pybind11::module::import("deskapi");
        pybind11::object py_result = deskapi.attr("unlock_joints")();
        py::tuple result_tuple = py_result.cast<py::tuple>();
        result = result_tuple[0].cast<bool>();
        nlohmann::json status_json = result_tuple[1].cast<nlohmann::json>();
        if(!result){
            spdlog::error("Cannot deactivate FCI through Desk client: "+status_json.dump());
        }
    }catch(const pybind11::error_already_set& e){
        spdlog::debug(e.what());
        spdlog::warn("Cannot unlock brakes, error when calling the python desk client.");
        result=false;
    }
    if(result){
        std::this_thread::sleep_for(std::chrono::seconds(30));
    }
    return result;
}

bool PandaBody::lock_brakes(){  // make sure it is waiting until all brakes are locked
    spdlog::trace("PandaBody::lock_brakes");
    if(!m_context.config.use_desk){
        spdlog::error("DESK features are not activated.");
        return false;
    }
    bool result;
    try{
        pybind11::module deskapi = pybind11::module::import("deskapi");
        pybind11::object py_result = deskapi.attr("lock_joints")();
        py::tuple result_tuple = py_result.cast<py::tuple>();
        result = result_tuple[0].cast<bool>();
        nlohmann::json status_json = result_tuple[1].cast<nlohmann::json>();
        if(!result){
            spdlog::error("Cannot lock brakes through Desk client: "+status_json.dump());
        }
        else{
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
    }catch(const pybind11::error_already_set& e){
        spdlog::debug(e.what());
        spdlog::warn("Cannot lock brakes, error when calling the python desk client.");
        result=false;
    }
    return result;
}

bool PandaBody::ensure_robot_ready(){  // put this in interface
    spdlog::trace("PandaBody::ensure_robot_ready");
    if(!m_context.config.use_desk){
        spdlog::error("DESK features are not activated.");
        return false;
    }
    bool result;
    try{
        pybind11::module deskapi = pybind11::module::import("keep_alive");
        pybind11::object py_result = deskapi.attr("ensure_robot_ready")();
        py::tuple result_tuple = py_result.cast<py::tuple>();
        result = result_tuple[0].cast<bool>();
        nlohmann::json status_json = result_tuple[1].cast<nlohmann::json>();
        if(!result){
            spdlog::error("Wait after error at deskapi: "+status_json.dump());
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
        else if(status_json.contains("status")){
            if(status_json["status"] == "Off"){
                m_hand=PandaHandNone;
            }
            else if(status_json["status"] == "On"){
                m_hand=PandaHandDefault; 
            }
            else{
                m_hand=PandaHandNone;
            }
        }
    }catch(const pybind11::error_already_set& e){
        spdlog::debug(e.what());
        spdlog::warn("Cannot ensure robot is ready, error when calling the python desk client.");
        result=false;
    }
    return result;
}


bool PandaBody::programming(){  // put this in interface
    spdlog::trace("PandaBody::programming");
    if(!m_context.config.use_desk){
        spdlog::error("DESK features are not activated.");
        return false;
    }
    bool result;
    try{
        pybind11::module deskapi = pybind11::module::import("deskapi");
        pybind11::object py_result = deskapi.attr("switch_to_programming")();
        py::tuple result_tuple = py_result.cast<py::tuple>();
        result = result_tuple[0].cast<bool>();
        nlohmann::json status_json = result_tuple[1].cast<nlohmann::json>();
        if(!result){
            spdlog::error("Cannot deactivate FCI through Desk client: "+status_json.dump());
        }
    }catch(const pybind11::error_already_set& e){
        spdlog::debug(e.what());
        spdlog::warn("Cannot lock brakes, error when calling the python desk client.");
        result=false;
    }
    if(result){
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return result;
}

bool PandaBody::execution(){  // put this in interface
    spdlog::trace("PandaBody::execution");
    if(!m_context.config.use_desk){
        spdlog::error("DESK features are not activated.");
        return false;
    }
    bool result;
    try{
        pybind11::module deskapi = pybind11::module::import("deskapi");
        pybind11::object py_result = deskapi.attr("switch_to_execution")();
        py::tuple result_tuple = py_result.cast<py::tuple>();
        result = result_tuple[0].cast<bool>();
        nlohmann::json status_json = result_tuple[1].cast<nlohmann::json>();
        if(!result){
            spdlog::error("Cannot deactivate FCI through Desk client: "+status_json.dump());
        }
    }catch(const pybind11::error_already_set& e){
        spdlog::debug(e.what());
        spdlog::warn("Cannot lock brakes, error when calling the python desk client.");
        result=false;
    }
    if(result){
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return result;
}

bool PandaBody::release_control(){
    spdlog::trace("PandaBody::release_control");
    if(!m_context.config.use_desk){
        spdlog::error("DESK features are not activated.");
        return false;
    }
    bool result;
    try{
        pybind11::module deskapi = pybind11::module::import("deskapi");
        pybind11::object py_result = deskapi.attr("release_control")();
        py::tuple result_tuple = py_result.cast<py::tuple>();
        result = result_tuple[0].cast<bool>();
        nlohmann::json status_json = result_tuple[1].cast<nlohmann::json>();
        if(!result){
            spdlog::error("Cannot release control through Desk client: "+status_json.dump());
        }
    }catch(const pybind11::error_already_set& e){
        spdlog::debug(e.what());
        spdlog::warn("Cannot release control, error when calling the python desk client.");
        result=false;
    }
    if(result){
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return result;
}
bool PandaBody::take_control(){
    spdlog::trace("PandaBody::take_control");
    if(!m_context.config.use_desk){
        spdlog::error("DESK features are not activated.");
        return false;
    }
    bool result;
    try{
        pybind11::module deskapi = pybind11::module::import("deskapi");
        pybind11::object py_result = deskapi.attr("take_control")();
        py::tuple result_tuple = py_result.cast<py::tuple>();
        result = result_tuple[0].cast<bool>();
        nlohmann::json status_json = result_tuple[1].cast<nlohmann::json>();
        if(!result){
            spdlog::error("Cannot take control through Desk client: "+status_json.dump());
        }
    }catch(const pybind11::error_already_set& e){
        spdlog::debug(e.what());
        spdlog::warn("Cannot ake control, error when calling the python desk client.");
        result=false;
    }
    if(result){
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return result;
}

}
