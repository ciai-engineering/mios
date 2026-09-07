#include "mios/skills/move_to_pose_cart.hpp"
#include "mios/strategies/move_to_pose.hpp"

namespace mios {

bool SkillParametersMoveToPoseCart::from_json(const nlohmann::json &p){
    if(!mirmi_utils::read_json_param(p,"t_settle",t_settle)){
        t_settle=0;
    }
    if(!mirmi_utils::read_json_param<double,2,1>(p,"speed",speed)){
        spdlog::error("Parameter speed could not be loaded but is mandatory.");
        return false;
    }
    if(!mirmi_utils::read_json_param<double,2,1>(p,"acc",acc)){
        spdlog::error("Parameter acc could not be loaded but is mandatory.");
        return false;
    }
    if(!mirmi_utils::read_json_param<double,4,4>(p,"T_T_EE_g_offset",T_T_EE_g_offset)){
        T_T_EE_g_offset.setIdentity();
    }
    bool object_set=false;
    if(!p["objects"].is_null()){
        if(p["objects"].find("goal_pose")!=p["objects"].end()){
            object_set=true;
        }
    }

    if(!mirmi_utils::read_json_param<double,4,4>(p,"T_T_EE_g",T_T_EE_g) && !object_set){
        spdlog::error("Parameter T_T_EE_g could not be loaded but is mandatory.");
        return false;
    }
    return true;
}

std::map<std::string, std::set<std::string> > SkillParametersMoveToPoseCart::get_parameter_list(){
    return {{"t_settle",{}},{"speed",{}},{"acc",{}},{"T_T_EE_g_offset",{}},{"T_T_EE_g",{}}};
}

MoveToPoseCart::MoveToPoseCart(const std::string &id, Memory *memory, Portal *portal):Skill("MoveToPoseCart",{"goal_pose"},id,memory,portal,{ControlMode::mCartTorque,ControlMode::mCartVelocity}),
m_finished(false){
}

std::shared_ptr<ManipulationPrimitive> MoveToPoseCart::get_initial_mp(const Percept &p_0){
    std::shared_ptr<SkillParametersMoveToPoseCart> skill_params = get_parameters<SkillParametersMoveToPoseCart>();
    std::shared_ptr<ManipulationPrimitive> mp = create_mp("move",p_0);
    mp->create_strategy<MoveToPoseStrategy>("s_0",1);
    Eigen::Matrix<double,4,4> T_g;
    if(this->get_object("goal_pose")->name=="NoneObject" || this->get_object("goal_pose")->name=="NullObject"){
        T_g=skill_params->T_T_EE_g;
    }else{
        T_g=get_object("goal_pose")->O_T_OB;
    }
    T_g.block<3,1>(0,3)+=skill_params->T_T_EE_g_offset.block<3,1>(0,3);
    T_g.block<3,3>(0,0)=skill_params->T_T_EE_g_offset.block<3,3>(0,0)*T_g.block<3,3>(0,0);
    Eigen::Matrix<double,2,1> speed;
    Eigen::Matrix<double,2,1> acc;
    speed<<skill_params->speed(0),skill_params->speed(1);
    acc<<skill_params->acc(0),skill_params->acc(1);
    mp->get_strategy<MoveToPoseStrategy>("s_0")->set_goal(T_g,speed,acc);
    Eigen::Matrix<double,2,1> scale;
    scale<<1,1;
    mp->get_strategy<MoveToPoseStrategy>("s_0")->set_scale(scale);
    return mp;
}

bool MoveToPoseCart::check_local_suc_conditions([[maybe_unused]] const Percept &p){
    if(get_active_mp()->get_strategy<MoveToPoseStrategy>("s_0")->finished()){
        if(!m_finished){
            m_finished=true;
            m_t_finished=std::chrono::high_resolution_clock::now();
        }
        return true;
    }else{
        return false;
    }
}

bool MoveToPoseCart::check_local_ex_conditions(const Percept &p){
    if(std::chrono::duration_cast<std::chrono::milliseconds>(p.time-m_t_finished).count()>=get_parameters<SkillParametersMoveToPoseCart>()->t_settle*1000){
        return true;
    }else{
        return false;
    }
}

}
