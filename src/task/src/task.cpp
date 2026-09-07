#include "mios/task/task.hpp"

#include "mios/core/core.hpp"
#include "mios/memory/memory.hpp"
#include "mios/tasks/null_task.hpp"

#include "spdlog/spdlog.h"
#include "mirmi_cpp_utils/system/system.hpp"

#include <sstream>

namespace mios {

Task::Task(const std::string& id, Core* core):m_core(core),m_memory(core->get_memory()),m_portal(core->get_portal()),m_skill_engine(core->get_skill_engine()),m_flag_stop(false),m_flag_recover(false),m_flag_in_recovery(false),m_id(id),
    m_uuid(mirmi_utils::generate_uuid()),m_active_subtask(nullptr){
    spdlog::trace("Task::Task()");
}

Task::~Task(){
    spdlog::trace("Task::~Task()");
    spdlog::debug("Task ("+m_uuid+"): destructor");
    notify_observers();
}

void Task::recover_task(){
    spdlog::warn("Recovery routine of task "+m_id+" is empty.");
}

void Task::stop_task(bool raise_exception,bool recover, bool empty_queue){
    if(m_flag_stop){
        return;
    }
    if(m_flag_in_recovery && !raise_exception){
        spdlog::warn("Can not invoke nominal stop while in recovery mode.");
        return;
    }
    if(raise_exception){
        spdlog::warn("Exception has been raised. I will not attempt to recover task " + m_id + ".");
    }
    m_result.exception=raise_exception;
    m_result.external_stop=true;
    m_result.empty_queue=empty_queue;
    m_flag_recover=recover && !raise_exception;
    if(m_active_subtask!=nullptr){
        m_active_subtask->stop_task(raise_exception,recover,false);
    }
    m_skill_engine->stop_skill();
    m_flag_stop=true;
    m_mtx_execution.lock();
    m_mtx_execution.unlock();
}

void Task::execute_desk_timeline(const std::string &id){
    if(m_flag_stop){
        return;
    }
    //    m_core->start_desk_task(id);
}

void Task::write_result(){
    m_result.success=true;
    for(const auto& r : m_result.skill_results){
        if(!r.second.success){
            m_result.success=false;
        }
    }
    write_custom_results(m_result.custom_results);
    if(!m_core->refresh_percept({})){
        spdlog::warn("Could not refresh perception, final checks may be invalid.");
    }
    const Percept* p_final = m_core->get_percept();
    if(p_final->robot_mode==control::RobotMode::kUserStopped){
        write_error("UserStopped");
    }
}

void Task::write_custom_results(nlohmann::json &custom_results){
    custom_results=nlohmann::json();
}

void Task::write_error(const std::string &error){
    m_result.errors.push_back(error);
}

bool Task::load_context(const nlohmann::json &user_context){
    try{
        spdlog::info("Loading context for task " + m_id + "...");
//        if(!m_memory->load_default_task_context(m_id,m_context)){
//            spdlog::error("Could not load a valid task context for "+m_id+".");
//            return false;
//        }
        m_context.clear();
        get_default_context(m_context);

        // merge default task parameters with user task parameters
        if(m_context.find("parameters")!=m_context.end() && user_context.find("parameters")!=user_context.end()){
            for(const auto& el : user_context["parameters"].items()){
                if((m_context["parameters"].find(el.key())==m_context["parameters"].end())){
                    spdlog::error("Task parameter "+el.key()+" given by user does not exist in default task context.");
                    return false;
                }
                mirmi_utils::overwrite_valid_json(user_context["parameters"][el.key()],m_context["parameters"][el.key()]);
            }
        }

        if(!read_parameters(m_context["parameters"])){
            spdlog::error("Could not read task parameters.");
            return false;
        }

        if(user_context.find("subtasks")!=user_context.end()){
            m_context["subtasks"]=user_context["subtasks"];
        }
        initialize_context();

        nlohmann::json default_parameters;
        if(!m_memory->load_default_parameters(default_parameters)){
            return false;
        }
        if(m_context.find("skills")!=m_context.end()){
            for(const auto& s : m_reserved_skills){
                if(m_context["skills"].find(s)==m_context["skills"].end()){
                    spdlog::error("Reserved skill " + s + " is not contained in default task context.");
                    return false;
                }
            }
            for(auto& s : m_context["skills"].items()){
                // CHECK; skill has been reserved
                if(m_reserved_skills.find(s.key())==m_reserved_skills.end()){
                    spdlog::error("Skill with name " + s.key() + " has not been reserved.");
                }
                std::string id_skill=s.key();
                // CHECK: skill has type
                if(s.value().find("type")==s.value().end()){
                    spdlog::error("Skill " + id_skill + " in task " + m_id + " has no type.");
                    return false;
                }
                std::string skill_type=s.value()["type"];
                // CHECK: skill has valid context
                default_parameters["skill"]=nlohmann::json();
                if(!m_memory->load_default_skill_context(skill_type,default_parameters["skill"])){
                    spdlog::error("Could not load a valid skill context for "+id_skill+".");
                    return false;
                }
                nlohmann::json tmp_params;
                for(auto& cat : default_parameters.items()){
                    if(s.value().find(cat.key())!=s.value().end()){
                        tmp_params = s.value()[cat.key()];
                    }
                    s.value()[cat.key()]=cat.value();
                    for(auto& p : tmp_params.items()){
                        s.value()[cat.key()][p.key()]=p.value();
                    }
                }
            }
        }else{
            if(m_reserved_skills.size()>0){
                spdlog::error("At least one skill has been reserved but the default task context does not contain any.");
                return false;
            }
        }

        if(user_context.find("skills")!=user_context.end()){
            for(auto& s : user_context["skills"].items()){
                if(m_reserved_skills.find(s.key())==m_reserved_skills.end()){
                    spdlog::error("Task " + m_id + " did not reserve a skill with name " + s.key() + " as defined in the user context.");
                    return false;
                }
                std::string id_skill=s.key();
                for(auto& cat : user_context["skills"][id_skill].items()){
                    for(auto& p: user_context["skills"][id_skill][cat.key()].items()){
                        if(p.value().is_object()){
                            for(auto& p_sub: user_context["skills"][id_skill][cat.key()][p.key()].items()){
                                m_context["skills"][id_skill][cat.key()][p.key()][p_sub.key()]=p_sub.value();
                            }
                        }else{
                            m_context["skills"][id_skill][cat.key()][p.key()]=p.value();
                        }
                    }
                }

            }
        }

        spdlog::info("Checking task context for consistency...");
        if(!check_context(m_context,user_context)){
            spdlog::error("Detected errors in task context, aborting execution of task "+ m_id +" with uuid " + m_uuid);
            return false;
        }
    }catch(const nlohmann::detail::type_error& e){
        spdlog::debug(e.what());
        spdlog::critical("An exception occured in Task::load_context");
        return false;
    }
    spdlog::info("Task context has been loaded");
    return true;
}

const nlohmann::json& Task::get_context() const{
    return m_context;
}

void Task::overwrite_context(const std::string &skill_name, const std::string &parameter_type, const std::string &parameter,const nlohmann::json& value){
    try {
        m_context["skills"][skill_name][parameter_type][parameter]=value;
    }catch(const nlohmann::detail::type_error& e){
        spdlog::debug(e.what());
        spdlog::error("Error when attempting to overwrite task context. Make sure the skill and the parameter exist in the default context.");
        throw TaskException();
    }
}

void Task::write_skill_object(const std::string skill, const std::string groundable, const std::string object){
    try{
        m_context["skills"][skill]["skill"]["objects"][groundable]=object;
    }catch(const nlohmann::detail::type_error& e){
        spdlog::debug(e.what());
        spdlog::error("Error when attempting to overwrite skill objects. Make sure the skill and the parameter exist in the default context.");
        throw TaskException();
    }
}

void Task::overwrite_context(const std::string& subtask_name, const std::string &skill_name, const std::string &parameter_type, const std::string &parameter,const nlohmann::json& value){
    try {
        m_context["subtasks"][subtask_name]["skills"][skill_name][parameter_type][parameter]=value;
    }catch(const nlohmann::detail::type_error& e){
        spdlog::debug(e.what());
        spdlog::error("Error when attempting to overwrite task context. Make sure the skill and the parameter exist in the default context.");
        throw TaskException();
    }
}

bool Task::grasp_object(const std::string &name, double speed){
    if(!m_flag_stop){
        return m_core->grasp_object(name,speed);
    }else{
        return true;
    }
}

bool Task::release_object(double speed){
    if(!m_flag_stop){
        return m_core->release_object(speed);
    }else{
        return true;
    }
}

bool Task::move_gripper(double width, double speed){
    if(!m_flag_stop){
        return m_core->move_gripper(width,speed);
    }else{
        return true;
    }
}

bool Task::get_percept(Percept &percept, std::optional<Eigen::Matrix<double, 3, 3> > O_R_T){
    if(!m_core->refresh_percept(O_R_T)){
        return false;
    }else{
        percept=*m_core->get_percept();
        return true;
    }
}

void Task::set_state(const std::string& state){
    if(!m_flag_stop){
        m_state=state;
    }
}

std::string Task::get_state() const{
    return m_state;
}

bool Task::reserve_skill(const std::string &name){
    if(m_reserved_skills.find(name)==m_reserved_skills.end()){
        m_reserved_skills.insert(name);
        return true;
    }else{
        spdlog::error("A skill with name " + name + " has already been reserved");
        return false;
    }
}

bool Task::insert_skill(const std::string &name, const std::string &type){
    if(m_context.find("skills")==m_context.end()){
        m_context["skills"]=nlohmann::json();
    }
    if(m_context["skills"].find(name)==m_context["skills"].end()){
        m_context["skills"][name]=nlohmann::json();
        m_context["skills"][name]["type"]=type;
        return true;
    }else{
        spdlog::error("A skill with name " + name + " already exists, cannot insert.");
        return false;
    }
}

bool Task::reserve_subtask(const std::string &name){
    if(m_reserved_subtasks.find(name)==m_reserved_subtasks.end()){
        m_reserved_subtasks.insert(name);
        m_subtask_results.insert(std::make_pair(name,TaskResult()));
        return true;
    }else{
        spdlog::error("A subtask with name " + name + " has already been reserved");
        return false;
    }
}

void Task::execute_skill_queue(){
    ControlReturnType result=m_skill_engine->execute_skill_queue();

    for(std::pair<std::string,SkillResult> r : m_skill_engine->get_results()){
        m_result.skill_results[r.first] = r.second;
    }
    m_skill_engine->clear_results();

    if(result.exception){
//        m_result.skill_results[name].success=false;
        spdlog::error("An exception occurred when executing skill queue.");
        throw TaskException();
    }
}

void Task::execute_subtask(const std::string& task_id,const std::string task_name){
    std::scoped_lock<std::mutex> lock(m_mtx_execution);
    if(m_reserved_subtasks.find(task_name)==m_reserved_subtasks.end()){
        stop_task(true);
        spdlog::error("Subtask with name "+task_name+" is not contained in task "+ m_id +". Stopping task.");
        throw TaskException();
    }
    if(m_flag_stop){
        return;
    }
    spdlog::info("Executing subtask "+task_name+"...");
    m_active_subtask = m_memory->load_subtask(task_id,m_context["subtasks"][task_name],m_core);
    if(m_active_subtask->get_id()=="NullTask"){
        spdlog::error("Error when loading subtask with name " + task_name);
        throw TaskException();
    }
    spdlog::info("Executing subtask "+task_name+"...");
    m_active_subtask->execute();
    spdlog::info("Subtask "+task_name+" has terminated.");
    m_active_subtask->write_result();
    if(m_active_subtask->do_recovery()){
        m_active_subtask->start_recovery();
        spdlog::info("Subtask "+task_name+" is attempting recovery.");
        m_active_subtask->recover_task();
    }
    if(m_context.find("subtasks")==m_context.end()){
        m_context["subtasks"]=nlohmann::json();
    }
    m_context["subtasks"][task_name]=m_active_subtask->get_context();
    TaskResult r = m_active_subtask->get_result();
    assert(m_subtask_results.find(task_name)!=m_subtask_results.end());
    m_subtask_results[task_name]=m_active_subtask->get_result();
    spdlog::info("End of lifecycle of subtask "+task_name+".");
    m_active_subtask.reset();
}

bool Task::read_parameters(const nlohmann::json &params){
    spdlog::error("This task has not overwritten the read_parameters method, yet the task context contains user-defined parameters. Undefined behavior is possible, aborting...");
    return false;
}

void Task::start_recovery(){
    m_flag_in_recovery=true;
    m_flag_stop=false;
}

void Task::complete_recovery(){
    m_flag_in_recovery=false;
}

bool Task::do_recovery() const{
    return m_flag_recover;
}

TaskResult Task::get_result() const{
    return m_result;
}

TaskResult Task::get_subtask_result(const std::string &subtask_name) const{
    if(m_subtask_results.find(subtask_name)==m_subtask_results.end()){
        spdlog::error("Cannot return result for non-existing subtask with name " + subtask_name + ".");
        throw TaskException();
    }
    return m_subtask_results.at(subtask_name);
}

void Task::sleep_1ms() const{
    while(!m_flag_stop){
        if(m_core->m_context.shutdown_signal){
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

bool Task::is_grasping() const{
    return m_core->is_grasping();
}

std::string Task::get_id() const{
    return m_id;
}

bool Task::get_stop_flag() const{
    return m_flag_stop;
}

bool Task::get_recovery_flag() const{
    return m_flag_in_recovery;
}

std::string Task::get_uuid() const{
    return m_uuid;
}

bool Task::check_context(const nlohmann::json &default_context, const nlohmann::json &user_context) const{

    std::unordered_set<std::string> top_level={"name","parameters","skills","_id","subtasks"};
    std::unordered_set<std::string> skill_level={"skill","control","limits","safety","frames","user","type"};  //,"system"

    try{
        for(const auto& el : default_context.items()){
            if(top_level.find(el.key())==top_level.end()){
                spdlog::error("Syntax error in default task context. Symbol with value "+el.key()+" is not valid on top level.");
                return false;
            }
        }
        for(const auto& el : user_context.items()){
            if(top_level.find(el.key())==top_level.end()){
                spdlog::error("Syntax error in user task context. Symbol with value "+el.key()+" is not valid on top level.");
                return false;
            }
        }
        if(user_context.find("parameters")!=user_context.end()){
            if(default_context.find("parameters")==default_context.end()){
                spdlog::error("Task " + m_id + " has no parameters but some where given by user input.");
                return false;
            }else{
                for(const auto& el : user_context["parameters"].items()){
                    if(default_context["parameters"].find(el.key())==default_context["parameters"].end()){
                        spdlog::error("Task " + m_id + " does not have a parameter " + el.key() + " as provided by user input.");
                        return false;
                    }
                }
            }
        }
        if(default_context.find("skills")!=default_context.end()){
            for(const auto& el_skill : default_context["skills"].items()){
                std::string skill=el_skill.key();
                if(default_context["skills"][skill].find("type")==default_context["skills"][skill].end()){
                    spdlog::error("Syntax error in task context for task "+m_id+". Skill " + skill + " is missing a type definition.");
                    return false;
                }
                std::string skill_type=default_context["skills"][skill]["type"];
                nlohmann::json default_skill_context;
                if(!m_memory->load_default_skill_context(skill_type,default_skill_context)){
                    spdlog::error("Could not load a valid skill context for "+skill+" of type " + skill_type +".");
                    return false;
                }
                if(user_context.find("skills")!=user_context.end()){
                    if(user_context["skills"].find(skill)!=user_context["skills"].end()){
                        for(const auto& el_cat : user_context["skills"][skill].items()){
                            if(skill_level.find(el_cat.key())==skill_level.end()){
                                spdlog::error("Syntax error in user input for task "+m_id+". Symbol with value "+el_cat.key()+" is not valid on skill level for skill " +skill+" of type "+skill_type +".");
                                return false;
                            }
                        }
                        if(user_context["skills"][skill].find("skill")!=user_context["skills"][skill].end()){
                            for(const auto& el_p : user_context["skills"][skill]["skill"].items()){
                                if(default_skill_context.find(el_p.key())==default_skill_context.end()){
                                    spdlog::error("Syntax error in user task context for task "+m_id+". Symbol with value "+el_p.key()+" is not valid in skill context for skill "+skill+" of type "+skill_type+".");
                                    return false;
                                }
                            }
                        }
                    }
                    for(const auto& el_skill : user_context["skills"].items()){
                        std::string skill=el_skill.key();
                        if(default_context["skills"].find(skill)==default_context["skills"].end()){
                            spdlog::error("Syntax error in user input for task "+m_id+". Skill " + skill + " does not exist in default task context.");
                            return false;
                        }
                    }
                }
                for(const auto& el_cat : default_context["skills"][skill].items()){
                    if(skill_level.find(el_cat.key())==skill_level.end()){
                        spdlog::error("Syntax error in task context for task "+m_id+". Symbol with value "+el_cat.key()+" is not valid on skill level for skill " +skill+" of type "+skill_type +".");
                        return false;
                    }
                }
                if(default_context["skills"][skill].find("skill")==default_context["skills"][skill].end()){
                    for(const auto& el_p : default_context["skills"][skill]["skill"].items()){
                        if(default_skill_context.find(el_p.key())==default_skill_context.end()){
                            spdlog::error("Syntax error in task context for task "+m_id+". Symbol with value "+el_p.key()+" is not valid in skill context for skill "+skill+" of type "+skill_type+".");
                            return false;
                        }
                    }
                }
            }
        }
    }catch(const nlohmann::detail::type_error& e){
        spdlog::debug(e.what());
        return false;
    }
    return true;
}

void Task::notify_observers(){
    for(auto& o : m_observers){
        o->finish();
    }
}

void Task::subscribe(std::shared_ptr<TaskObserver> observer){
    m_observers.insert(observer);
}

}
