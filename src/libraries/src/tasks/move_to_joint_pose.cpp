#include "mios/tasks/move_to_joint_pose.hpp"
#include "mios/data_structures/parameters.hpp"
#include "mios/skills/move_to_pose_joint.hpp"
#include "mirmi_cpp_utils/json/json.hpp"

namespace mios {

MoveToJointPose::MoveToJointPose(Core* core):Task("MoveToJointPose",core){

}

void MoveToJointPose::initialize_context(){
    reserve_skill("move");
}

void MoveToJointPose::execute(){
    // MoveToJointPose produces a bounded joint-velocity profile.  In the ROS
    // runtime that profile is integrated by JointPositionControllerPipeline
    // and sent through the dedicated, rate-limited joint-position controller.
    // Do not select the legacy joint-velocity pipeline here: it targets a
    // separate controller that is intentionally not activated for automatic
    // job motion.
    overwrite_context("move", "control", "control_mode",
                      static_cast<int>(ControlMode::mJointPosition));
    overwrite_context("move","skill","speed",m_speed);
    overwrite_context("move","skill","acc",m_acc);
    overwrite_context("move","skill","q_g",mirmi_utils::from_eigen<double,7,1>(m_q_g));
    write_skill_object("move","goal_pose",m_pose.value_or("NullObject"));
    execute_skill<MoveToPoseJoint,SkillParametersMoveToPoseJoint>("move");
}

bool MoveToJointPose::read_parameters(const nlohmann::json &params){
    m_pose = mirmi_utils::from_json<std::string>(params,"pose");
//    mirmi_utils::read_json_param(params,"pose",m_p);
    if(!mirmi_utils::read_json_param<double,7,1>(params,"q_g",m_q_g)){
        m_q_g.setZero();
    }
    if(m_q_g.isZero() && !m_pose.has_value()){
        spdlog::error("MoveToJointPose task requires either a location that exists in memory or an explicit joint pose.");
        return false;
    }
    if(!mirmi_utils::read_json_param(params,"speed",m_speed)){
        spdlog::error("Could not read parameter: speed");
        return false;
    }
    if(!mirmi_utils::read_json_param(params,"acc",m_acc)){
        spdlog::error("Could not read parameter: acc");
        return false;
    }
    return true;
}

void MoveToJointPose::get_default_context(nlohmann::json &context){
    context["parameters"] = nlohmann::json();
    context["parameters"]["pose"]=nlohmann::json();
    context["parameters"]["q_g"]=nlohmann::json();
    context["parameters"]["speed"]=0.5;
    context["parameters"]["acc"]=0.7;

    context["skills"]=nlohmann::json();
    context["skills"]["move"]=nlohmann::json();
    context["skills"]["move"]["control"] = {
        {"control_mode", static_cast<int>(ControlMode::mJointPosition)}};
    context["skills"]["move"]["type"]="MoveToPoseJoint";
}

}
