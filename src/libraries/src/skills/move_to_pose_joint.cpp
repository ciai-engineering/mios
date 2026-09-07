#include "mios/skills/move_to_pose_joint.hpp"
#include <spdlog/spdlog.h>
#include "mios/strategies/move_to_joint_pose.hpp"

namespace mios {

bool SkillParametersMoveToPoseJoint::from_json(const nlohmann::json &p){
    if(!mirmi_utils::read_json_param(p,"t_settle",t_settle)){
        t_settle=0;
    }
    if(!mirmi_utils::read_json_param(p,"speed",speed)){
        spdlog::error("Parameter speed could not be loaded but is mandatory.");
        return false;
    }
    if(!mirmi_utils::read_json_param(p,"acc",acc)){
        spdlog::error("Parameter acc could not be loaded but is mandatory.");
        return false;
    }
    if(!mirmi_utils::read_json_param<double,7,1>(p,"q_g_offset",q_g_offset)){
        q_g_offset.setZero();
    }
    bool object_set=false;
    if(!p["objects"].is_null()){
        if(p["objects"].find("goal_pose")!=p["objects"].end()){
            object_set=true;
        }
    }

    if(!mirmi_utils::read_json_param<double,7,1>(p,"q_g",q_g) && !object_set){
        spdlog::error("Parameter q_g could not be loaded but is mandatory.");
        return false;
    }
    return true;
}

std::map<std::string, std::set<std::string> > SkillParametersMoveToPoseJoint::get_parameter_list(){
    return {{"t_settle",{}},{"speed",{}},{"acc",{}},{"q_g_offset",{}},{"q_g",{}}};
}

MoveToPoseJoint::MoveToPoseJoint(const std::string &id, Memory *memory, Portal* portal):Skill("MoveToPoseJoint",{"goal_pose"},id,memory,portal,{ControlMode::mJointTorque,ControlMode::mJointVelocity,ControlMode::mJointPosition}),
m_finished(false){
}

std::shared_ptr<ManipulationPrimitive> MoveToPoseJoint::get_initial_mp(const Percept &p_0){
    std::shared_ptr<SkillParametersMoveToPoseJoint> skill_params = get_parameters<SkillParametersMoveToPoseJoint>();
    std::shared_ptr<ManipulationPrimitive> mp = create_mp("move",p_0);
    mp->create_strategy<MoveToJointPoseStrategy>("s_0",1);
    std::shared_ptr<MoveToJointPoseStrategy> move_s0 = mp->get_strategy<MoveToJointPoseStrategy>("s_0");
    Eigen::Matrix<double,7,1> q_g;
    if(this->get_object("goal_pose")->name=="NoneObject" || this->get_object("goal_pose")->name=="NullObject"){
        q_g=skill_params->q_g;
    }else{
        q_g=get_object("goal_pose")->q;
    }
    q_g+=skill_params->q_g_offset;
    move_s0->set_goal(q_g,skill_params->speed,skill_params->acc);
    return mp;
}

bool MoveToPoseJoint::check_local_suc_conditions([[maybe_unused]] const Percept &p){
    if(get_active_mp()->get_strategy<MoveToJointPoseStrategy>("s_0")->finished()){
        if(!m_finished){
            m_finished=true;
            m_t_finished=std::chrono::high_resolution_clock::now();
        }
        return true;
    }else{
        return false;
    }
}

bool MoveToPoseJoint::check_local_ex_conditions(const Percept &p){
    if(std::chrono::duration_cast<std::chrono::seconds>(p.time-m_t_finished).count()>=get_parameters<SkillParametersMoveToPoseJoint>()->t_settle){
        return true;
    }else{
        return false;
    }
}

}
