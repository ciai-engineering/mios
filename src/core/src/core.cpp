#include "mios/core/core.hpp"

#include "mios/panda/robot_backend.hpp"
#ifdef MIOS_HAS_DIRECT_PANDA_BACKEND
#include "mios/panda/panda_body.hpp"
#endif

#include "mirmi_cpp_utils/math/math.hpp"
#include "mirmi_cpp_utils/conversion/conversion.hpp"
#include "mirmi_cpp_utils/json/json.hpp"
#include "mirmi_cpp_utils/system/system.hpp"
#include "mios/utils/exceptions.hpp"
#include "mios/skill/skill.hpp"

#include "mios/controller_pipeline/cart_torque_pipeline.hpp"
#include "mios/controller_pipeline/joint_torque_pipeline.hpp"
#include "mios/controller_pipeline/cart_velocity_pipeline.hpp"
#include "mios/controller_pipeline/joint_velocity_pipeline.hpp"
#include "mios/controller_pipeline/joint_position_pipeline.hpp"
#include "mios/control/robot_parameter_translation.hpp"
#include "mios/safety_stage_1/velocity_walls.hpp"
#include "mios/safety_stage_2/virtual_cube.hpp"
#include "mios/safety_stage_2/virtual_joint_walls.hpp"
#include "mios/safety_stage_2/cartesian_velocity_damping.hpp"

#include <functional>

#include "spdlog/spdlog.h"

#include <thread>

namespace mios {

#ifdef MIOS_HAS_DIRECT_PANDA_BACKEND
Core::Core(const MiosContext &context):
    m_memory(context),
    m_skill_engine(SkillEngine(this)),
    m_robot_backend(std::make_unique<PandaBody>(&m_memory, context)),
    m_portal(Portal("0.0.0.0",context.config.websocket_port,"mios/core",
                    "0.0.0.0",context.config.rpc_port,
                    context.config.udp_port)),
    m_task_engine(TaskEngine(this)),
    m_command_interface(CommandInterface(this,&m_task_engine,&m_portal,&m_memory)),
    m_telemetry(TelemetryUDP(this,&m_portal)),
    m_is_ready(false),
    m_context(context),
    m_blend_skill(false),
    m_hand_grace_period(0){
        spdlog::trace("Core::Core(direct PandaBody)");
}
#endif

Core::Core(const MiosContext &context, std::unique_ptr<RobotBackend> robot_backend):
    m_memory(context),
    m_skill_engine(SkillEngine(this)),
    m_robot_backend(std::move(robot_backend)),
    m_portal(Portal("0.0.0.0",context.config.websocket_port,"mios/core", //websocket
                    "0.0.0.0",m_context.config.rpc_port,                 //rpc
                    context.config.udp_port)),                           //udp
    m_task_engine(TaskEngine(this)),
    m_command_interface(CommandInterface(this,&m_task_engine,&m_portal,&m_memory)),//m_ros_node(this,&m_task_engine,&m_portal,&m_memory),
    m_telemetry(TelemetryUDP(this,&m_portal)),
    m_is_ready(false),
    m_context(context),
    m_blend_skill(false),
    m_hand_grace_period(0){
        spdlog::trace("Core::Core()");
}

Core::~Core(){
    spdlog::trace("Core::~Core()");
    terminate();
}

bool Core::initialize(){
    spdlog::trace("Core::initialize()");
    if(!m_robot_backend){
        spdlog::error("No robot backend was supplied.");
        return false;
    }
    spdlog::info("Initializing memory...");
    if(!m_memory.initialize(&m_skill_library)){
        spdlog::error("Could not initialize memory.");
        return false;
    }
    m_robot_backend->set_robot_parameter_provider([this] {
        return std::optional<control::RobotParameters>(
            control::make_robot_parameters(*m_memory.read_parameters()));
    });
    spdlog::info("Initializing robot...");
    if(!m_robot_backend->initialize()){
        spdlog::error("Could not initialize robot.");
        return false;
    }
    spdlog::info("Updating database...");
    if(!m_memory.update_database()){
        spdlog::error("Could not update datebase.");
        return false;
    }

    spdlog::info("Acquiring initial percept...");
    if(!refresh_percept({})){
        spdlog::error("Could not acquire initial percept.");
        return false;
    }
    spdlog::info("Initializing interfaces...");
    if(!m_portal.initialize()){
        spdlog::error("Could not initialize portal.");
        return false;
    }
    //m_ros_node.start();

    m_is_ready=true;
    return true;
}

void Core::start(){
    spdlog::trace("Core::start()");
    spdlog::info("Starting task engine...");
    m_task_engine.life_cycle();
}

void Core::terminate(){
    spdlog::trace("Core::terminate()");
    m_task_engine.stop();
    if(m_control_executor){
        m_control_executor->terminate();
    }
    if(m_robot_backend){
        m_robot_backend->disconnect_from_robot();
        m_robot_backend->disconnect_from_gripper();
    }
}

Memory* Core::get_memory(){
    return &m_memory;
}

SkillEngine* Core::get_skill_engine(){
    return &m_skill_engine;
}

Portal* Core::get_portal(){
    return &m_portal;
}

TaskEngine* Core::get_task_engine(){
    return &m_task_engine;
}

CommandInterface* Core::get_command_interface(){
    return &m_command_interface;
}
/*
RosNode* Core::get_ros_node(){
    return &m_ros_node;
}
*/
LearningModule* Core::get_learning_module(){
    return &m_learning_module;
}

TelemetryUDP* Core::get_telemetry(){
    return &m_telemetry;
}

ControlReturnType Core::execute_skill(){
    spdlog::trace("Core:execute_skill()");

    if(!m_robot_backend){
        return {true, "NoRobotBackend", "A ROS 2 robot backend must be supplied."};
    }

    if(!m_robot_backend->pre_run_checks()){
        if(ControlReturnType result=m_robot_backend->recover();result.exception){
            return result;
        }
    }
    ControlReturnType result(false,"None","");

    refresh_percept(m_memory.read_parameters()->frames.O_R_T);
    std::scoped_lock<std::mutex> busy_lock(m_mtx_is_busy);
    m_percept.update_controller();
    m_robot_backend->set_robot_parameters();

    spdlog::trace("CORE:execute_skill.start_control_cycle");
    const auto run_control = [this](const control::CommandMode command_mode) {
        return m_robot_backend->control(
            command_mode,
            [this, command_mode](const control::RobotState& robot_state,
                                 const control::RobotModel& robot_model,
                                 const control::GripperState& gripper_state, double) {
                return control_base_cycle(robot_state, robot_model, gripper_state, command_mode);
            });
    };

    switch(m_memory.read_parameters()->control.control_mode){
        case ControlMode::mCartTorque:
            if(!configure_control_executor(std::make_unique<CartTorqueControllerPipeline>(),
                                           control::CommandMode::kTorque, true)){
                return {true, "ControlExecutorInitializationFailed", "cartesian torque"};
            }
            result=run_control(control::CommandMode::kTorque);
            break;
        case ControlMode::mJointTorque:
            if(!configure_control_executor(std::make_unique<JointTorqueControllerPipeline>(),
                                           control::CommandMode::kTorque, false)){
                return {true, "ControlExecutorInitializationFailed", "joint torque"};
            }
            result=run_control(control::CommandMode::kTorque);
            break;
        case ControlMode::mCartVelocity:
            if(!configure_control_executor(std::make_unique<CartVelocityControllerPipeline>(),
                                           control::CommandMode::kCartesianVelocity, false)){
                return {true, "ControlExecutorInitializationFailed", "cartesian velocity"};
            }
            result=run_control(control::CommandMode::kCartesianVelocity);
            break;
        case ControlMode::mJointVelocity:
            if(!configure_control_executor(std::make_unique<JointVelocityControllerPipeline>(),
                                           control::CommandMode::kJointVelocity, false)){
                return {true, "ControlExecutorInitializationFailed", "joint velocity"};
            }
            result=run_control(control::CommandMode::kJointVelocity);
            break;
        case ControlMode::mJointPosition:
            if(!configure_control_executor(std::make_unique<JointPositionControllerPipeline>(),
                                           control::CommandMode::kJointPosition, false)){
                return {true, "ControlExecutorInitializationFailed", "joint position"};
            }
            result=run_control(control::CommandMode::kJointPosition);
            break;
        case ControlMode::mNoControl:
            spdlog::error("No control mode has been selected.");
            break;
    }

    m_blend_skill=false;
    //    m_robot_backend.stop_gripper();
    return result;
}

bool Core::configure_control_executor(std::unique_ptr<ControllerPipeline> pipeline,
                                      const control::CommandMode command_mode,
                                      const bool add_cartesian_velocity_damping){
    auto executor = std::make_unique<MiosAlgorithmExecutor>(std::move(pipeline), command_mode);
    if(command_mode == control::CommandMode::kTorque){
        executor->add_safety_stage_1(std::make_unique<VelocityWallsSafetyModule>());
        executor->add_safety_stage_2(std::make_unique<VirtualCubeSafetyModule>());
        executor->add_safety_stage_2(std::make_unique<VirtualJointWallsSafetyModule>());
        if(add_cartesian_velocity_damping){
            executor->add_safety_stage_2(
                std::make_unique<CartesianVelocityDampingSafetyModule>());
        }
    }
    const Parameters* parameters = m_memory.read_parameters();
    control::ControlRuntimeConfig config;
    config.control = parameters->control;
    config.safety = parameters->safety;
    config.limits = parameters->limits;
    config.frames = parameters->frames;
    if(!executor->initialize(m_percept, config)){
        return false;
    }
    if(m_control_executor){
        m_control_executor->terminate();
    }
    m_control_executor = std::move(executor);
    return true;
}

void Core::post_execution(){
    spdlog::trace("Core::post_execution()");
    if(m_control_executor){
        m_control_executor->terminate();
        m_control_executor.reset();
    }
    if(!m_memory.update_database()){
        spdlog::warn("Could not update datebase.");
    }
}

void Core::handle_gripper(Actuator* cmd){
    if(m_percept.internal_model.hand_activity_state==HandActivityState::hsIdle && cmd->get_gripper_request()!=GripperRequest::None){
        m_percept.internal_model.hand_activity_state=HandActivityState::hsBusy;
        if(cmd->get_gripper_request()==GripperRequest::Grasp){
            std::thread gripper(&Core::grasp,this,cmd->gripper_width,cmd->gripper_speed,cmd->gripper_force,0.1,0.1,cmd->gripper_object);
            gripper.detach();
        }
        if(cmd->get_gripper_request()==GripperRequest::Move){
            std::thread gripper(&Core::move_gripper,this,cmd->gripper_width,cmd->gripper_speed);
            gripper.detach();
        }
        cmd->accecpt_gripper_request();
    }
    if(m_percept.internal_model.hand_activity_state==HandActivityState::hsFinished){
        if(m_hand_grace_period==0){
            m_hand_grace_period++;
        }else{
            m_hand_grace_period=0;
            m_percept.internal_model.hand_activity_state=HandActivityState::hsIdle;
        }
    }
}

control::ArmCommand Core::control_base_cycle(const control::RobotState& robot_state,
                                             const control::RobotModel& robot_model,
                                             const control::GripperState& gripper_state,
                                             control::CommandMode command_mode){
    if(m_context.shutdown_signal){
        terminate();
    }
    bool exception=false;
    if(m_skill_engine.is_running_queue() && m_blend_skill){
        if(!m_skill_engine.blend_skill_stage_1()){
            spdlog::error("First stage of skill blending failed.");
            exception=true;
        }
    }
    m_percept.update(robot_state, robot_model, gripper_state,
                     m_memory.read_parameters()->frames.O_R_T);
    m_memory.internal_update(m_percept);
    if(m_skill_engine.is_running_queue() && m_blend_skill){
        if(!m_skill_engine.blend_skill_stage_2()){
            spdlog::error("Second stage of skill blending failed.");
            exception=true;
        }
        m_blend_skill=false;
    }

    Actuator* cmd=m_skill_engine.get_next_command(m_percept);
    if(!cmd->is_valid()){
        exception=true;
    }
    if(exception){
        cmd->stop();
    }

    handle_gripper(cmd);


    m_memory.get_parameters()->frames.O_R_T=cmd->O_R_T;
    if(!m_control_executor){
        spdlog::error("No MIOS algorithm executor is configured.");
        cmd->stop();
        control::ArmCommand safe_command;
        safe_command.mode = command_mode;
        safe_command.motion_finished = true;
        return safe_command;
    }
    const MiosAlgorithmExecutor::CycleResult cycle = m_control_executor->step(m_percept, *cmd);
    if(!cycle.valid){
        spdlog::error("Invalid command from MIOS algorithm executor.");
        cmd->stop();
        return cycle.command;
    }
    control::ArmCommand robot_command = cycle.command;

    if(m_memory.get_parameters()->skill->log_data){
        m_skill_engine.log_data(m_percept);
    }
    

    if(cmd->is_stopped()){
        if(m_skill_engine.is_running_queue()){
            m_blend_skill=true;
            if(m_skill_engine.is_last_skill()){
                spdlog::trace("Core::control_base_cycle.stopped");
                robot_command.motion_finished=true;
            }
        }else{
            spdlog::trace("Core::control_base_cycle.stopped");
            robot_command.motion_finished=true;
        }
    }
    if (robot_command.mode != command_mode) {
        spdlog::error("Controller pipeline returned a command in the wrong mode.");
        control::ArmCommand safe_command;
        safe_command.mode = command_mode;
        safe_command.motion_finished = true;
        return safe_command;
    }
    return robot_command;
}

bool Core::grasp_object(const std::string &name,double speed){
    spdlog::trace("Core::grasp_object()");
    const Object* object=m_memory.get_object(name);
    if(object->name=="NullObject"){
        spdlog::error("Cannot find object "+name+" in knowledge base.");
        return false;
    }
    if(!refresh_percept({})){
        spdlog::error("Could not refresh my perception. Discrepancy between real world and believe state is possible.");
        return false;
    }
    if(m_percept.robot_mode==control::RobotMode::kUserStopped){
        spdlog::error("Action is not permitted while in user mode.");
        return false;
    }
    if(m_robot_backend->grasp(object->grasp_width,speed,object->grasp_force,0.005,0.005)){
        m_memory.get_live_context()->grasped_object=object;
        m_memory.internal_update(m_percept);
        m_memory.get_parameters()->user.load_m=object->mass;
        m_memory.get_parameters()->user.load_com=(m_memory.read_parameters()->frames.F_T_EE*mirmi_utils::invert_transformation_matrix(object->OB_T_gp)).block<3,1>(0,3);
        m_memory.get_parameters()->user.load_I=object->OB_I;
        m_memory.get_parameters()->frames.EE_T_TCP=mirmi_utils::invert_transformation_matrix(object->OB_T_gp)*object->OB_T_TCP;
        if(!m_robot_backend->set_robot_parameters()){
            return false;
        }
        if(!m_memory.update_database()){
            spdlog::warn("Could not update datebase.");
        }
        return true;
    }else{
        return false;
    }
}

bool Core::home_gripper(){
    spdlog::trace("Core::home_gripper()");
    if(!refresh_percept({})){
        spdlog::error("Could not refresh my perception. Discrepancy between real world and believe state is possible.");
        return false;
    }
    if(m_percept.robot_mode==control::RobotMode::kUserStopped){
        spdlog::error("Action is not permitted while in user mode.");
        return false;
    }
    return m_robot_backend->home_gripper();
}

bool Core::grasp(double width, double speed, double force,double epsilon_inner,double epsilon_outer,std::string object_name){
    spdlog::trace("Core::grasp()");
    if(m_percept.robot_mode==control::RobotMode::kUserStopped){
        spdlog::error("Action is not permitted while in user mode.");
        return false;
    }
    m_percept.internal_model.hand_activity_state=HandActivityState::hsBusy;
    bool result = m_robot_backend->grasp(width,speed,force,epsilon_inner,epsilon_outer);
    const Object* object=m_memory.get_object(object_name);
    // A raw gripper command deliberately has no named object yet.  This is
    // the normal teaching path before teach_object() records the grasp pose.
    if(object_name != "NullObject" && object->name=="NullObject"){
        spdlog::warn("Cannot find object "+object_name+" in knowledge base.");
    }
    m_memory.get_live_context()->grasped_object=object;
    m_memory.internal_update(m_percept);
    if(!m_memory.update_database()){
        spdlog::warn("Could not update datebase.");
    }
    m_memory.get_parameters()->user.load_m=object->mass;
    m_memory.get_parameters()->user.load_com=(m_memory.read_parameters()->frames.F_T_EE*mirmi_utils::invert_transformation_matrix(object->OB_T_gp)).block<3,1>(0,3);
    m_memory.get_parameters()->user.load_I=object->OB_I;
    m_memory.get_parameters()->frames.EE_T_TCP=mirmi_utils::invert_transformation_matrix(object->OB_T_gp)*object->OB_T_TCP;
    m_percept.internal_model.hand_activity_state=HandActivityState::hsFinished;
    return result;
}

bool Core::move_gripper(double width, double speed){
    spdlog::trace("Core::move_gripper()");
    if(m_percept.robot_mode==control::RobotMode::kUserStopped){
        spdlog::error("Action is not permitted while in user mode.");
        return false;
    }
    m_percept.internal_model.hand_activity_state=HandActivityState::hsBusy;
    bool result = m_robot_backend->move_to_finger_position(width,speed);
    const Object* object=m_memory.get_object("NullObject");
    m_memory.get_live_context()->grasped_object=object;
    m_memory.internal_update(m_percept);
    if(!m_memory.update_database()){
        spdlog::warn("Could not update datebase.");
    }
    m_memory.get_parameters()->user.load_m=object->mass;
    m_memory.get_parameters()->user.load_com=(m_memory.read_parameters()->frames.F_T_EE*mirmi_utils::invert_transformation_matrix(object->OB_T_gp)).block<3,1>(0,3);
    m_memory.get_parameters()->user.load_I=object->OB_I;
    m_memory.get_parameters()->frames.EE_T_TCP=mirmi_utils::invert_transformation_matrix(object->OB_T_gp)*object->OB_T_TCP;
    m_percept.internal_model.hand_activity_state=HandActivityState::hsFinished;
    return result;
}

bool Core::is_grasping(){
    spdlog::trace("Core::is_grasping()");
    refresh_percept({});
    return m_percept.proprioception.is_grasping;
}

bool Core::set_grasped_object(const std::string &name){
    spdlog::trace("Core::set_grasped_object()");
    const Object* object=m_memory.get_object(name);
    if(object->name=="NullObject"){
        spdlog::error("Cannot find object "+name+" in knowledge base.");
        return false;
    }
    if(!refresh_percept({})){
        spdlog::warn("Could not refresh my perception. Discrepancy between real world and believe state is possible.");
    }
    if(m_percept.robot_mode==control::RobotMode::kUserStopped){
        spdlog::error("Action is not permitted while in user mode.");
        return false;
    }
    m_memory.get_live_context()->grasped_object=object;
    m_memory.internal_update(m_percept);
    if(!m_memory.update_database()){
        spdlog::warn("Could not update datebase.");
    }
    m_memory.get_parameters()->user.load_m=object->mass;
    m_memory.get_parameters()->user.load_com=(m_memory.read_parameters()->frames.F_T_EE*mirmi_utils::invert_transformation_matrix(object->OB_T_gp)).block<3,1>(0,3);
    m_memory.get_parameters()->user.load_I=object->OB_I;
    m_memory.get_parameters()->frames.EE_T_TCP=mirmi_utils::invert_transformation_matrix(object->OB_T_gp)*object->OB_T_TCP;
    return m_robot_backend->set_robot_parameters();
}

bool Core::release_object(std::optional<double> width, double speed){
    spdlog::trace("Core::release_object()");
    const Object* object=m_memory.get_live_context()->grasped_object;
    if(object->name=="NullObject" && !is_grasping()){
        spdlog::error("I am not grasping anything.");
        return false;
    }
    if(!refresh_percept({})){
        spdlog::error("Could not refresh my perception. Discrepancy between real world and believe state is possible.");
        return false;
    }
    if(m_percept.robot_mode==control::RobotMode::kUserStopped){
        spdlog::error("Action is not permitted while in user mode.");
        return false;
    }
    if(!m_memory.update_database()){
        spdlog::warn("Could not update datebase.");
    }
    object=m_memory.get_object("NullObject");
    if(m_robot_backend->move_to_finger_position(width.value_or(m_percept.internal_model.max_finger_width),speed)){
        m_memory.get_live_context()->grasped_object=object;
        m_memory.internal_update(m_percept);
        m_memory.get_parameters()->user.load_m=object->mass;
        m_memory.get_parameters()->user.load_com=(m_memory.read_parameters()->frames.F_T_EE*mirmi_utils::invert_transformation_matrix(object->OB_T_gp)).block<3,1>(0,3);
        m_memory.get_parameters()->user.load_I=object->OB_I;
        m_memory.get_parameters()->frames.EE_T_TCP=mirmi_utils::invert_transformation_matrix(object->OB_T_gp)*object->OB_T_TCP;
        m_robot_backend->set_robot_parameters();
        return true;
    }else{
        return false;
    }
}

bool Core::refresh_percept(std::optional<Eigen::Matrix<double,3,3> > O_R_TF, bool wait){
    control::RobotState robot_state;
    control::RobotModel robot_model;
    control::GripperState gripper_state;
    if(is_busy()){
        return true;
    }
    bool read_successful=false;
    int count=0;
    if(wait){
        while(!read_successful){
            if(is_busy()){
                return true;
            }
            read_successful=true;
            if(!m_robot_backend->get_robot_snapshot(robot_state, robot_model, gripper_state)){
                spdlog::debug("Core::refresh_percept.failed_to_acquire_robot_snapshot");
                read_successful=false;
            }
            if(!read_successful){
                spdlog::debug("Waiting for valid perception...");
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }else{
                break;
            }
            if(count>6){
                count = 0;
                spdlog::debug("reconnecting to Robot and Gripper");
                m_robot_backend->connect_to_robot(m_context.config.robot_ip);
                m_robot_backend->connect_to_gripper(m_context.config.robot_ip);
            }
            count++;
        }
    }else{
        if(!m_robot_backend->get_robot_snapshot(robot_state, robot_model, gripper_state)){
            spdlog::debug("Core::refresh_percept.failed_to_acquire_robot_snapshot");
            spdlog::debug("reconnecting to Robot");
            m_robot_backend->connect_to_robot(m_context.config.robot_ip);
            m_robot_backend->connect_to_gripper(m_context.config.robot_ip);
            if(!m_robot_backend->get_robot_snapshot(robot_state, robot_model, gripper_state)){
                return false;
            }
        }

    }
    m_percept.update(robot_state, robot_model, gripper_state, O_R_TF);
    if(m_control_executor){
        m_control_executor->update_percept(m_percept.controller);
    }
    m_memory.internal_update(m_percept);
    return true;
}

bool Core::unlock_body(){
    spdlog::trace("Core::unlock_body()");
    return m_robot_backend->unlock_brakes();

}

bool Core::lock_body(){
    spdlog::trace("Core::lock_body()");
    return m_robot_backend->lock_brakes();
}

bool Core::shutdown_body(){
    spdlog::trace("Core::shutdown_body()");
    return m_robot_backend->shutdown_robot();
}

bool Core::reboot_body(){
    spdlog::trace("Core::reboot_body()");
    return m_robot_backend->reboot_robot();
}

bool Core::pack_body(){
    spdlog::info("Core::pack_body(): This function is not implemented by the configured backend.");
    return false;
}

bool Core::start_desk_task(const std::string &task){
    spdlog::info("Core::start_desk_task(): This function is not implemented by the configured backend.");
    return false;
}

bool Core::stop_desk_task(){
    spdlog::info("Core::stop_desk_task(): This function is not implemented by the configured backend.");
    return false;
}

bool Core::recover_body(){
    spdlog::trace("Core::recover_body()");
    return !m_robot_backend->recover().exception;
}

const Percept* Core::get_percept() const{
    return &m_percept;
}

bool Core::is_ready() const{
    return m_is_ready;
}

bool Core::is_busy(){
    if(m_mtx_is_busy.try_lock()){
        m_mtx_is_busy.unlock();
        return false;
    }else{
        return true;
    }
}

}
