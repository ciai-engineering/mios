#pragma once

#include <memory>
#include <optional>
#include <string>
#include <mutex>

#include "mios/controller_pipeline/mios_algorithm_executor.hpp"
#include "mios/interface/interface.hpp"
//#include "mios/interface/ros_node.hpp"
#include "mios/learning_module/learning_module.hpp"
#include "mios/memory/memory.hpp"
#include "mios/portal/portal.hpp"
#include "mios/skill/skill_engine.hpp"
#include "mios/skill/skill_library.hpp"
#include "mios/task/task_engine.hpp"
#include "mios/telemetry/telemetry_udp.hpp"

#include "mios/data_structures/actuator.hpp"
#include "mios/data_structures/percept.hpp"
#include "mios/utils/context.hpp"



namespace mios {

class Skill;
class RobotBackend;

class Core{
public:
#ifdef MIOS_HAS_DIRECT_PANDA_BACKEND
    // The standalone direct build owns FCI through PandaBody. This overload
    // is not compiled into the ROS-owned Core, preventing two FCI owners.
    explicit Core(const MiosContext &context);
#endif
    // The caller supplies the only robot backend. In the ROS-only path this
    // backend communicates through ROS 2 and never owns an FCI connection.
    Core(const MiosContext &context, std::unique_ptr<RobotBackend> robot_backend);
    ~Core();

    bool initialize();
    void start();
    void terminate();

    ControlReturnType execute_skill();
    void post_execution();
    void terminate_control_cycle();

    bool refresh_percept(std::optional<Eigen::Matrix<double, 3, 3> > O_R_TF, bool wait=true);
    bool try_refresh_percept(std::optional<Eigen::Matrix<double, 3, 3> > O_R_TF, bool wait=true);

    // Gripper
    bool grasp_object(const std::string& name, double speed=1);
    bool release_object(std::optional<double> width, double speed=1);
    bool grasp(double width, double speed, double force, double epsilon_inner, double epsilon_outer, std::string object_name="NullObject");
    bool move_gripper(double width,double speed);
    bool is_grasping();
    bool home_gripper();
    bool set_grasped_object(const std::string& name);

    bool lock_body();
    bool unlock_body();
    bool shutdown_body();
    bool reboot_body();
    bool pack_body();

    bool start_desk_task(const std::string& task);
    bool stop_desk_task();

    bool recover_body();

public:
    Memory* get_memory();
    SkillEngine* get_skill_engine();
    Portal* get_portal();
    TaskEngine* get_task_engine();
    CommandInterface* get_command_interface();
    //RosNode* get_ros_node();
    LearningModule* get_learning_module();
    TelemetryUDP* get_telemetry();
    const Percept *get_percept() const;
    bool is_ready() const;
    bool is_busy();  
    MiosContext m_context;

private:
    control::ArmCommand control_base_cycle(const control::RobotState& robot_state,
                                           const control::RobotModel& robot_model,
                                           const control::GripperState& gripper_state,
                                           control::CommandMode command_mode);
    bool configure_control_executor(std::unique_ptr<ControllerPipeline> pipeline,
                                    control::CommandMode command_mode,
                                    bool add_cartesian_velocity_damping);

    void handle_gripper(Actuator* cmd);

private:
    Percept m_percept;

    Memory m_memory;
    SkillEngine m_skill_engine;
    std::unique_ptr<RobotBackend> m_robot_backend;
    Portal m_portal;
    SkillLibrary m_skill_library;
    TaskEngine m_task_engine;
    CommandInterface m_command_interface;
    //RosNode m_ros_node;
    LearningModule m_learning_module;
    TelemetryUDP m_telemetry;
    std::unique_ptr<MiosAlgorithmExecutor> m_control_executor;

private:
    bool m_is_ready;
    bool m_blend_skill;
    std::mutex m_mtx_is_busy;
    std::mutex m_mtx_FCI;

private:
    unsigned m_hand_grace_period;
};

}
