#include "mios/memory/lt_memory.hpp"
#include "mios/task/task.hpp"
#include "mios/task/taskfactory.hpp"
#include "mios/skill/skill_library.hpp"
#include "spdlog/spdlog.h"
#include "mirmi_cpp_utils/json/json.hpp"


namespace mios {

LTMemory::LTMemory(const MiosContext &context):
    m_mongodb_client(context.config.database_name, context.config.database_port,
                     !context.config.defer_database_connection),
    m_context(context){
        spdlog::trace("LTMemory::LTMemory");

}

bool LTMemory::is_ok() const{
    spdlog::trace("LTMemory::is_ok");
    if(!m_mongodb_client.health_check()){
        spdlog::error("Database health check failed.");
        return false;
    }
    return true;
}

void LTMemory::link_to_st_memory(STMemory *st_memory){
    spdlog::trace("LTMemory::link_to_st_memory");
    m_st_memory=st_memory;
}

void LTMemory::link_to_skill_library(SkillLibrary *skill_library){
    spdlog::trace("LTMemory::link_to_skill_library");
    m_skill_library=skill_library;
}

bool LTMemory::initialize(){
    spdlog::trace("LTMemory::initialize");
    if(!m_mongodb_client.connect()){
        spdlog::error("Could not connect to the database.");
        return false;
    }
    if(!make_database_consistent()){
        return false;
    }

    // nlohmann::json system_parameters;
    // m_mongodb_client.read_document("system","parameters",system_parameters);
    // switch(m_context.config.robot_configuration){
    // case 0:
    //     system_parameters["has_robot"]=true;
    //     system_parameters["gripper"]="Default";
    //     m_mongodb_client.write_document("system","parameters",system_parameters,true);
    //     break;
    // case 1:
    //     system_parameters["has_robot"]=true;
    //     system_parameters["gripper"]="None";
    //     m_mongodb_client.write_document("system","parameters",system_parameters,true);
    //     break;
    // case 2:
    //     system_parameters["has_robot"]=true;
    //     system_parameters["gripper"]="Softhand2";
    //     m_mongodb_client.write_document("system","parameters",system_parameters,true);
    //     break;
    // case 3:
    //     system_parameters["has_robot"]=false;
    //     system_parameters["gripper"]="None";
    //     m_mongodb_client.write_document("system","parameters",system_parameters,true);
    //     break;
    // default:
    //     spdlog::error("Robot configuration " + std::to_string(m_context.config.robot_configuration) + " does not exist.");
    //     return false;
    // }

    return true;
}

bool LTMemory::make_database_consistent(){
    spdlog::trace("LTMemory::make_database_consistent");
    nlohmann::json default_values;
    // default_values=SystemParameters().to_json();
    // default_values["name"]="system";
    // if(!m_mongodb_client.make_document_consistent("system","parameters",default_values)){
    //     return false;
    // }
    default_values=SafetyParameters().to_json();
    default_values["name"]="safety";
    if(!m_mongodb_client.make_document_consistent("safety","parameters",default_values)){
        return false;
    }
    default_values=ControlParameters().to_json();
    default_values["name"]="control";
    if(!m_mongodb_client.make_document_consistent("control","parameters",default_values)){
        return false;
    }
    default_values=LimitParameters().to_json();
    default_values["name"]="limits";
    if(!m_mongodb_client.make_document_consistent("limits","parameters",default_values)){
        return false;
    }
    default_values=FramesParameters().to_json();
    default_values["name"]="frames";
    if(!m_mongodb_client.make_document_consistent("frames","parameters",default_values)){
        return false;
    }
    default_values=UserParameters().to_json();
    default_values["name"]="user";
    if(!m_mongodb_client.make_document_consistent("user","parameters",default_values)){
        return false;
    }
    if(!make_ros2_no_motion_self_test_consistent()){
        return false;
    }
//    if(!make_default_tasks_consistent()){
    //        return false;
    //    }
    if(!make_default_environment_consistent()){
        return false;
    }
    if(!m_mongodb_client.health_check()){
        spdlog::error("Could not check database health.");
        return false;
    }
    return true;
}

bool LTMemory::make_ros2_no_motion_self_test_consistent(){
    spdlog::trace("LTMemory::make_ros2_no_motion_self_test_consistent");
    nlohmann::json default_values;
    default_values["name"]="Ros2NoMotionSelfTest";
    default_values["description"]=
        "ROS 2 migration scheduler self-test. It never commands the arm, gripper, or robot parameters.";
    default_values["parameters"]=nlohmann::json::object();
    default_values["skills"]=nlohmann::json::object();
    return m_mongodb_client.make_document_consistent(
        "Ros2NoMotionSelfTest", "tasks", default_values);
}

bool LTMemory::make_default_tasks_consistent(){
    spdlog::trace("LTMemory::make_default_tasks_consistent");
    nlohmann::json default_values;
    default_values["name"]="TestTask1";
    default_values["skills"]={
    {"t1_s1",{
    {"type","TestSkill1"},
    {"skill",{
    {"objects",{{"object","TestObject1"}}}
}
},
    {"control",{{"control_mode",0}}}
},
},
    {"t1_s2",{
    {"type","TestSkill1"},
    {"skill",{
    {"objects",{{"object","TestObject1"}}}
}
},
    {"control",{{"control_mode",0}}}
},
}
};
    default_values["parameters"]={
    {"a",{0,0,0}},
    {"b",false},
    {"success",false},
    {"exception","none"},
    {"skill_test",0},
    {"queue_number",0},
    {"mp_sequence",{}}
};
    if(!m_mongodb_client.make_document_consistent("TestTask1","tasks",default_values)){
        return false;
    }

    default_values.clear();
    default_values["name"]="TestTask2";
    default_values["skills"]={
    {"t2_s1",{
    {"type","TestSkill1"},
    {"skill",{
    {"objects",{{"object","TestObject1"}}}
}
},
    {"control",{{"control_mode",0}}}
},
},
    {"t2_s2",{
    {"type","TestSkill1"},
    {"skill",{
    {"objects",{{"object","TestObject1"}}}
}
},
    {"control",{{"control_mode",0}}}
},
}
};
    default_values["parameters"]={
    {"d",{0,0}},
    {"e",0},
    {"f",false},
    {"stop_level",0},
    {"success",false}
};
    if(!m_mongodb_client.make_document_consistent("TestTask2","tasks",default_values)){
        return false;
    }

    default_values.clear();
    default_values["name"]="TestTask3";
    default_values["skills"]={
    {"t3_s1",{
    {"type","TestSkill1"},
    {"skill",{
    {"objects",{{"object","TestObject1"}}}
}
},
    {"control",{{"control_mode",0}}}
},
},
    {"t3_s2",{
    {"type","TestSkill1"},
    {"skill",{
    {"objects",{{"object","TestObject1"}}}
}
},
    {"control",{{"control_mode",0}}}
},
},
    {"t3_s3",{
    {"type","TestSkill1"},
    {"skill",{
    {"objects",{{"object","TestObject1"}}}
}
},
    {"control",{{"control_mode",0}}}
},
}
};
    default_values["parameters"]={
    {"g",{0,0,0,0}},
    {"h",false},
    {"i",0},
    {"j",""},
    {"stop_level",0},
    {"success",false}
};
    if(!m_mongodb_client.make_document_consistent("TestTask3","tasks",default_values)){
        return false;
    }


    default_values.clear();
    default_values["name"]="IdleTask";
    default_values["skills"]={
    {"sleep",{
    {"type","GenericWiggleMotion"},
},
},
    {"hold",{
    {"type","HoldPose"},
},},
    {"move",{
    {"type","MoveToPoseJoint"},
},}
};
    default_values["parameters"]={
    {"idle_mode","none"},
};
    if(!m_mongodb_client.make_document_consistent("IdleTask","tasks",default_values)){
        return false;
    }

    return true;
}

bool LTMemory::make_default_environment_consistent(){
    spdlog::trace("LTMemory::make_default_environment_consistent");
    nlohmann::json default_values;
    Object o = Object("TestObject1");
    o.grasp_force=1;
    if(!m_mongodb_client.make_document_consistent("TestObject1","environment",o.to_json())){
        return false;
    }
    return true;
}

bool LTMemory::get_task_data(const std::string uuid, TaskData &data) const{
    spdlog::trace("LTMemory::get_task_data");
    if(m_task_data.find(uuid)==m_task_data.end()){
        return false;
    }else{
        data=m_task_data.at(uuid);
        return true;
    }
}

std::shared_ptr<Task> LTMemory::load_task(const std::string& task_id, const nlohmann::json& user_context,Core* core){
    spdlog::trace("LTMemory::load_task");
    std::shared_ptr<Task> task = TaskFactory::create_task(TaskFactory::get_task_name(task_id),core);
    //    if(task->get_context().find("parameters")!=task->get_context().end()){
    //        if(!task->read_parameters(task->get_context()["parameters"])){
    //            spdlog::error("Could not read parameters for task " + task->get_id());
    //            return TaskFactory::create_task(TaskName::TaskNameNullTask,core);
    //        }
    //    }
    if(!task->load_context(user_context)){
        spdlog::error("Could not load context for task " + task->get_id());
        return TaskFactory::create_task(TaskName::TaskNameNullTask,core);
    }
    return task;
}

std::shared_ptr<Task> LTMemory::load_subtask(const std::string& task_id, const nlohmann::json& user_context,Core* core){
    spdlog::trace("LTMemory::load_subtask");
    std::shared_ptr<Task> task = TaskFactory::create_task(TaskFactory::get_task_name(task_id),core);
    if(!task->load_context(user_context)){
        spdlog::error("Could not load context for subtask " + task->get_id());
        return TaskFactory::create_task(TaskName::TaskNameNullTask,core);
    }
    if(task->get_context().find("parameters")!=task->get_context().end()){
        if(!task->read_parameters(task->get_context()["parameters"])){
            spdlog::error("Could not read parameters for subtask " + task->get_id());
            return TaskFactory::create_task(TaskName::TaskNameNullTask,core);
        }
    }
    m_st_memory->put_subtask(task_id,task->get_context());
    return task;
}

bool LTMemory::load_default_parameters(nlohmann::json &parameters){
    spdlog::trace("LTMemory::load_default_parameters");
    if(!m_mongodb_client.read_document("control","parameters",parameters["control"])){
        return false;
    }
    if(!m_mongodb_client.read_document("frames","parameters",parameters["frames"])){
        return false;
    }
    if(!m_mongodb_client.read_document("safety","parameters",parameters["safety"])){
        return false;
    }
    // if(!m_mongodb_client.read_document("system","parameters",parameters["system"])){
    //     return false;
    // }
    if(!m_mongodb_client.read_document("user","parameters",parameters["user"])){
        return false;
    }
    if(!m_mongodb_client.read_document("limits","parameters",parameters["limits"])){
        return false;
    }
    return true;
}

bool LTMemory::load_default_task_context(const std::string task_id,nlohmann::json& task_context){
    spdlog::trace("LTMemory::load_default_task_context");
    return m_mongodb_client.read_document(task_id,"tasks",task_context);
}

bool LTMemory::load_default_skill_context(const std::string skill_type,nlohmann::json& skill_context){
    spdlog::trace("LTMemory::load_default_skill_context");
    std::map<std::string, std::set<std::string> > skill_parameters;
    if(m_skill_library->get_skill_parameters()->find(skill_type)==m_skill_library->get_skill_parameters()->end()){
        spdlog::error("Could not find skill type " + skill_type + " in library.");
        skill_context=nlohmann::json();
        return false;
    }else{
        skill_parameters=m_skill_library->get_skill_parameters()->at(skill_type)->get_parameter_list();
    }
    for(const auto& param : skill_parameters){
        skill_context[param.first]=nlohmann::json();
        if(param.second.size()>0){
            for(const auto& sub_param: param.second){
                skill_context[param.first][sub_param]=nlohmann::json();
            }
        }
    }
    nlohmann::json parameters = SkillParameters::get_default_values();
    for(const auto& param : parameters.items()){
        skill_context[param.key()]=param.value();
    }
    return true;
}

void LTMemory::store_task_data(const std::string &uuid, const std::string &task_id, const nlohmann::json &context, const TaskResult &result){
    spdlog::trace("LTMemory::store_task_data");
    m_task_data.emplace(std::make_pair(uuid,TaskData(task_id,context,result)));
}

bool LTMemory::load_environment(std::unordered_map<std::string, Object> &environment){
    spdlog::trace("LTMemory::load_environment");
    std::set<nlohmann::json> docs;
    environment.emplace(std::make_pair("NullObject",Object("NullObject")));
    environment.emplace(std::make_pair("NoneObject",Object("NoneObject")));
    if(!m_mongodb_client.read_documents("environment",docs)){
        return false;
    }
    for(const auto& d: docs){
        environment.emplace(std::make_pair(d["name"],Object::from_json(d)));
    }
    return true;
}

bool LTMemory::upload_environment_element(const Object& element){
    spdlog::trace("LTMemory::upload_environment_element");
    return m_mongodb_client.write_document(element.name,"environment",element.to_json(),true);
}
bool LTMemory::upload_log_element(const nlohmann::json& log_entry){
    spdlog::trace("LTMemory::upload_log_element");
    return m_mongodb_client.write_large_document("datalogs",log_entry);
}

bool LTMemory::update_database(){
    spdlog::trace("LTMemory::update_database");
    // if(!m_mongodb_client.write_document("system","parameters",m_st_memory->read_parameters()->system.to_json(),true)){
    //     return false;
    // }
    //    if(!m_mongodb_client.write_document("user","parameters",m_st_memory->read_parameters()->user.to_json(),true)){
    //        return false;
    //    }
    //    if(!m_mongodb_client.write_document("frames","parameters",m_st_memory->read_parameters()->frames.to_json(),true)){
    //        return false;
    //    }
    //    if(!m_mongodb_client.write_document("control","parameters",m_st_memory->read_parameters()->control.to_json(),true)){
    //        return false;
    //    }
    //    if(!m_mongodb_client.write_document("safety","parameters",m_st_memory->read_parameters()->safety.to_json(),true)){
    //        return false;
    //    }
    for(const auto& env : *m_st_memory->get_environment()){
        spdlog::debug("Updating object: " + env.first);
        if(!m_mongodb_client.write_document(env.first,"environment",env.second.to_json(),true)){
            return false;
        }
    }
    return true;
}

}
