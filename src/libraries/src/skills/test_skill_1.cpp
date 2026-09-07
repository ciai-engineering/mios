#include "mios/skills/test_skill_1.hpp"
#include "mios/strategies/null_strategy.hpp"
#include <spdlog/spdlog.h>

namespace mios{

bool SkillParametersTestSkill1::from_json(const nlohmann::json& parameters){
    if(!mirmi_utils::read_json_param(parameters,"run_time",run_time)){
        spdlog::debug("Could not load parameter: run_time [double]");
        run_time=0;
    }
    if(!mirmi_utils::read_json_param(parameters,"success",success)){
        spdlog::debug("Could not load parameter: success [bool]");
        success=false;
    }
    if(!mirmi_utils::read_json_param(parameters,"t_exception",t_exception)){
        spdlog::debug("Could not load parameter: t_exception [double]");
        t_exception=0;
    }
    if(!mirmi_utils::read_json_param(parameters,"exception",exception)){
        spdlog::debug("Could not load parameter: exception [string]");
        exception="none";
    }
    if(!mirmi_utils::read_json_param(parameters,"cost_suc",cost_suc)){
        spdlog::debug("Could not load parameter: cost_suc [double]");
        cost_suc=0;
    }
    if(!mirmi_utils::read_json_param(parameters,"cost_err",cost_err)){
        spdlog::debug("Could not load parameter: cost_err [double]");
        cost_err=0;
    }
    if(!mirmi_utils::read_json_param<int>(parameters,"mp_sequence",mp_sequence)){
        spdlog::debug("Could not load parameter: mp_sequence [vector<int>]");
        mp_sequence.resize(0);
    }
    spdlog::debug("###############################   Skill parameters   ######################################");
    spdlog::debug("run_time: "+std::to_string(run_time));
    spdlog::debug("success: "+std::to_string(success));
    spdlog::debug("t_exception: "+std::to_string(t_exception));
    spdlog::debug("exception: "+exception);
    spdlog::debug("###########################################################################################");
    return true;
}

std::map<std::string,std::set<std::string> > SkillParametersTestSkill1::get_parameter_list(){
    return {{"run_time",{}},{"success",{}},{"t_exception",{}},{"exception",{}},{"cost_err",{}},{"cost_suc",{}},{"mp_sequence",{}}};
}

TestSkill1::TestSkill1(const std::string &name, Memory *memory,Portal* portal):Skill("TestSkill1",{"object"},name,memory,portal,{ControlMode::mCartTorque}),m_result_code(-1),m_sequence_index(0),
    m_mp_graph({"mp_0","mp_1","mp_2","mp_3","mp_4","mp_5","mp_6"}),m_result_graph({}){}

std::shared_ptr<ManipulationPrimitive> TestSkill1::get_initial_mp(const Percept &p_0){
    std::shared_ptr<ManipulationPrimitive> mp = create_mp(m_mp_graph[m_sequence_index],p_0);
    mp->create_strategy<NullStrategy>("s_0",1);
    return mp;
}

std::optional<std::shared_ptr<ManipulationPrimitive> > TestSkill1::graph_transition(const Percept& p){
    std::shared_ptr<SkillParametersTestSkill1> params = get_parameters<SkillParametersTestSkill1>();
    std::string mp_name = get_active_mp()->get_name();
    if(m_sequence_index>params->mp_sequence.size() || params->mp_sequence.size()==0){
        return {};
    }
    if(mp_name != m_mp_graph[params->mp_sequence[m_sequence_index]]){
        m_result_graph.emplace_back(m_mp_graph[params->mp_sequence[m_sequence_index]]);
        std::shared_ptr<ManipulationPrimitive> mp = create_mp(m_mp_graph[params->mp_sequence[m_sequence_index++]],p);
        mp->create_strategy<NullStrategy>("s_0",1);
        return mp;
    }else{
        return {};
    }
}

bool TestSkill1::check_local_suc_conditions(const Percept& p){
    std::shared_ptr<SkillParametersTestSkill1> c = get_parameters<SkillParametersTestSkill1>();
    double t_run=std::chrono::duration_cast<std::chrono::seconds>(p.time-m_memory->get_live_context()->t_skill).count();
    if(c->success && c->mp_sequence.size()>0){
        if(m_sequence_index==c->mp_sequence.size()){
            return true;
        }else{
            return false;
        }
    }
    if(c->success && t_run>c->run_time && c->exception=="none"){
        spdlog::debug("Local success condition triggered at "+std::to_string(t_run));
        return true;
    }else{
        return false;
    }
}

bool TestSkill1::check_local_err_conditions(const Percept &p){
    std::shared_ptr<SkillParametersTestSkill1> c = get_parameters<SkillParametersTestSkill1>();
    double t_run=std::chrono::duration_cast<std::chrono::seconds>(p.time-m_memory->get_live_context()->t_skill).count();
    if(!c->success && t_run>c->run_time){
        spdlog::debug("Local error condition triggered at "+std::to_string(t_run));
        return true;
    }else{
        return false;
    }
}

Eigen::Matrix<double,3,3> TestSkill1::get_O_R_T_0(const Percept &p) const{
    spdlog::debug("get_O_R_T_0");
    Eigen::Matrix<double,3,3> R = get_object_grasp_pose_O("object").block<3,3>(0,0);
    return R;
}

void TestSkill1::auxiliaries(const Percept &p){
    std::shared_ptr<SkillParametersTestSkill1> c = get_parameters<SkillParametersTestSkill1>();
    double t_run=std::chrono::duration_cast<std::chrono::seconds>(p.time-m_memory->get_live_context()->t_skill).count();
    if(t_run>c->t_exception){
        if(c->exception=="control"){
            spdlog::trace("TestSkill1::auxiliaries::control");
            m_result_code=1;
            throw SkillException();
        }
        if(c->exception=="invalid"){
            spdlog::trace("TestSkill1::auxiliaries::invalid");
            m_result_code=2;
            throw SkillException();
        }
        if(c->exception=="network"){
            spdlog::trace("TestSkill1::auxiliaries::network");
            m_result_code=3;
            throw SkillException();
        }
        if(c->exception=="realtime"){
            spdlog::trace("TestSkill1::auxiliaries::realtime");
            m_result_code=4;
            throw SkillException();
        }
        if(c->exception=="skill"){
            spdlog::trace("TestSkill1::auxiliaries::skill");
            m_result_code=5;
            spdlog::error("This is a skill exception that has been thrown for test purposes");
            throw SkillException();
        }
    }
    m_result_code=-1;
}

void TestSkill1::write_custom_results(nlohmann::json& custom_results){
    spdlog::debug("Evaluate");
    std::shared_ptr<SkillParametersTestSkill1> c = get_parameters<SkillParametersTestSkill1>();
    custom_results["exception"]=c->exception;
    custom_results["run_time"]=c->run_time;
    custom_results["success"]=c->success;
    custom_results["t_exception"]=c->t_exception;
    custom_results["result_code"]=m_result_code;
    custom_results["result_graph"]=m_result_graph;
    if(m_memory->get_live_parameter("test_parameter_1").has_value()){
        custom_results["test_parameter_1"]=m_memory->get_live_parameter("test_parameter_1").value();
    }
    if(m_memory->get_live_parameter("test_parameter_2").has_value()){
        custom_results["test_parameter_2"]=m_memory->get_live_parameter("test_parameter_2").value();
    }
    if(m_memory->get_live_parameter("test_parameter_3").has_value()){
        custom_results["test_parameter_3"]=m_memory->get_live_parameter("test_parameter_3").value();
    }
}

void TestSkill1::parallels(){
    double cnt=0;
    mirmi_utils::read_json_param(get_custom_results(),"parallels_cnt",cnt);
    get_custom_results()["parallels_cnt"]=cnt+1;
}

}
