#include "mios/interface/interface.hpp"

#include "mios/core/core.hpp"
#include "mios/memory/memory.hpp"
#include "mios/portal/portal.hpp"
#include "mios/task/task_engine.hpp"

#include "spdlog/spdlog.h"


namespace mios {

using mirmi_utils::ArgPair;

CommandInterface::CommandInterface(Core *core, TaskEngine *task_engine,Portal* portal,Memory* memory):m_core(core),m_task_engine(task_engine),m_portal(portal),m_memory(memory){
    spdlog::trace("ComanndInterface::CommandInterface()");
    bind_methods();
}

void CommandInterface::bind_methods(){
    spdlog::trace("ComanndInterface::bind_methods()");
    m_portal->bind_method_to_all("start_task",std::bind(&CommandInterface::start_task,this,std::placeholders::_1),{ArgPair("task",{}),ArgPair("parameters",{}),ArgPair("queue",false)});
    m_portal->bind_method_to_all("stop_task",std::bind(&CommandInterface::stop_task,this,std::placeholders::_1),
    {ArgPair("raise_exception",false),ArgPair("recover",false),ArgPair("empty_queue",false)});
    m_portal->bind_method_to_all("remove_task",std::bind(&CommandInterface::remove_task,this,std::placeholders::_1),{ArgPair("task_uuid",{})});
    m_portal->bind_method_to_all("wait_for_task",std::bind(&CommandInterface::wait_for_task,this,std::placeholders::_1),{ArgPair("task_uuid",{})});
    m_portal->bind_method_to_all("is_busy",std::bind(&CommandInterface::is_busy,this,std::placeholders::_1),{});

    m_portal->bind_method_to_all("set_grasped_object",std::bind(&CommandInterface::set_grasped_object,this,std::placeholders::_1),{ArgPair("object",{})});
    m_portal->bind_method_to_all("grasp_object",std::bind(&CommandInterface::grasp_object,this,std::placeholders::_1),{ArgPair("object",{}),ArgPair("speed",1)});
    m_portal->bind_method_to_all("grasp",std::bind(&CommandInterface::grasp,this,std::placeholders::_1),{ArgPair("width",{}),ArgPair("speed",{}),ArgPair("force",{}),ArgPair("epsilon_inner",0.001),ArgPair("epsilon_outer",0.001)});
    m_portal->bind_method_to_all("release_object",std::bind(&CommandInterface::release_object,this,std::placeholders::_1),{ArgPair("speed",1),ArgPair("width",{-1})});
    m_portal->bind_method_to_all("move_gripper",std::bind(&CommandInterface::move_gripper,this,std::placeholders::_1),{ArgPair("width",{}),ArgPair("speed",{})});
    m_portal->bind_method_to_all("home_gripper",std::bind(&CommandInterface::home_gripper,this,std::placeholders::_1),{});

    m_portal->bind_method_to_all("teach_object",std::bind(&CommandInterface::teach_object,this,std::placeholders::_1),{ArgPair("object",{}),ArgPair("teach_width",false)});
    m_portal->bind_method_to_all("set_partial_object_data",std::bind(&CommandInterface::set_partial_object_data,this,std::placeholders::_1),{ArgPair("object",{}),ArgPair("data",{})});
    m_portal->bind_method_to_all("set_object",std::bind(&CommandInterface::set_object,this,std::placeholders::_1),{ArgPair("object",{}),ArgPair("O_T_OB",nlohmann::json()),
                                                                                                                   ArgPair("OB_T_TCP",nlohmann::json()),ArgPair("OB_T_gp",nlohmann::json()),
                                                                                                                   ArgPair("geometry",nlohmann::json()),ArgPair("grasp_force",nlohmann::json()),
                                                                                                                   ArgPair("grasp_width",nlohmann::json()),ArgPair("mass",nlohmann::json()),
                                                                                                                   ArgPair("q",nlohmann::json()),ArgPair("OB_I",nlohmann::json())});
    //    m_portal->bind_method_to_all("apply_reference_frame",std::bind(&CommandInterface::apply_reference_frame,this,std::placeholders::_1),{ArgPair("frame",{})});
    m_portal->bind_method_to_all("download_task_context",std::bind(&CommandInterface::download_task_context,this,std::placeholders::_1),{ArgPair("task",{})});
    m_portal->bind_method_to_all("download_skill_context",std::bind(&CommandInterface::download_skill_context,this,std::placeholders::_1),{ArgPair("skill",{})});
    m_portal->bind_method_to_all("download_object_context",std::bind(&CommandInterface::download_object_context,this,std::placeholders::_1),{ArgPair("object",{})});

    m_portal->bind_method_to_all("get_state",std::bind(&CommandInterface::get_state,this,std::placeholders::_1),{});
    m_portal->bind_method_to_all("get_model",std::bind(&CommandInterface::get_model,this,std::placeholders::_1),{});
    m_portal->bind_method_to_all("reload_database",std::bind(&CommandInterface::reload_database,this,std::placeholders::_1),{});
    m_portal->bind_method_to_all("subscribe_telemetry",std::bind(&CommandInterface::subscribe_telemetry,this,std::placeholders::_1),{});
    m_portal->bind_method_to_all("unsubscribe_telemetry",std::bind(&CommandInterface::unsubscribe_telemetry,this,std::placeholders::_1),{});

    m_portal->bind_method_to_all("unlock_brakes",std::bind(&CommandInterface::unlock_brakes,this,std::placeholders::_1),{});
    m_portal->bind_method_to_all("lock_brakes",std::bind(&CommandInterface::lock_brakes,this,std::placeholders::_1),{});
    m_portal->bind_method_to_all("shutdown",std::bind(&CommandInterface::shutdown,this,std::placeholders::_1),{});
    m_portal->bind_method_to_all("reboot",std::bind(&CommandInterface::reboot,this,std::placeholders::_1),{});
    m_portal->bind_method_to_all("pack_pose",std::bind(&CommandInterface::pack_pose,this,std::placeholders::_1),{});
    m_portal->bind_method_to_all("start_desk_task",std::bind(&CommandInterface::start_desk_task,this,std::placeholders::_1),{ArgPair("task",{})});
    m_portal->bind_method_to_all("stop_desk_task",std::bind(&CommandInterface::stop_desk_task,this,std::placeholders::_1),{});

    m_portal->bind_method_to_all("terminate",std::bind(&CommandInterface::terminate,this,std::placeholders::_1),{});

    m_portal->bind_method_to_all("post_event",std::bind(&CommandInterface::post_event,this,std::placeholders::_1),{ArgPair("name",{}),ArgPair("content",nlohmann::json())});
    m_portal->bind_method_to_all("get_event",std::bind(&CommandInterface::get_event,this,std::placeholders::_1),{ArgPair("name",{})});

    m_portal->bind_method_to_all("set_live_parameter",std::bind(&CommandInterface::set_live_parameter,this,std::placeholders::_1),{ArgPair("key",{}),ArgPair("value",{})});

    m_portal->bind_method_to_all("learn_task",std::bind(&CommandInterface::learn_task,this,std::placeholders::_1),{ArgPair("problem_definition",{}),
                                 ArgPair("service_configuration",nlohmann::json()),ArgPair("agents",{})});
    m_portal->bind_method_to_all("stop_learning",std::bind(&CommandInterface::stop_learning,this,std::placeholders::_1),{ArgPair("uuid",{})});

}

nlohmann::json CommandInterface::terminate([[maybe_unused]] const nlohmann::json &request){
    spdlog::trace("CommandInterface:terminate()");
    m_task_engine->stop();
    return nlohmann::json();
}

nlohmann::json CommandInterface::start_task(const nlohmann::json &request){
    spdlog::trace("CommandInterface:start_task()");
    nlohmann::json response;
    bool result;
    std::string task_uuid;
    std::string error_message;
    std::tie(result,task_uuid,error_message)=m_task_engine->start_task(request["task"],request["parameters"],request["queue"]);
    response["task_uuid"]=task_uuid;
    response["error"]=error_message;
    response["result"]=result;
    return response;
}

nlohmann::json CommandInterface::stop_task(const nlohmann::json &request){
    spdlog::trace("CommandInterface:stop_task()");
    nlohmann::json response;
    bool result;
    std::string error_message;
    std::tie(result,error_message)=m_task_engine->stop_task(request["raise_exception"],request["recover"],request["empty_queue"]);
    response["error"]=error_message;
    response["result"]=result;
    return response;
}

nlohmann::json CommandInterface::remove_task(const nlohmann::json &request){
    spdlog::trace("CommandInterface:remove_task()");
    nlohmann::json response;
    bool result;
    std::string error_message;
    std::string task_uuid;
    request["task_uuid"].get_to(task_uuid);

    spdlog::info("Removing task with uuid " + task_uuid + ".");
    std::tie(result,error_message)=m_task_engine->remove_task(task_uuid);
    response["result"]=result;
    response["error"]=error_message;
    return response;
}

nlohmann::json CommandInterface::wait_for_task(const nlohmann::json &request){
    spdlog::trace("CommandInterface:wait_for_task()");
    nlohmann::json response;
    std::string task_uuid;
    bool result;
    std::string error_message;
    TaskResult task_result;
    request["task_uuid"].get_to(task_uuid);
    std::tie(result,task_result,error_message)=m_task_engine->wait_for_task(task_uuid);
    nlohmann::json task_result_response;
    if(result){
        task_result_response["success"]=task_result.success;
        task_result_response["external_stop"]=task_result.external_stop;
        task_result_response["exception"]=task_result.exception;
        task_result_response["results"]=task_result.custom_results;
        task_result_response["error"]=task_result.errors;
        nlohmann::json skill_results;
        for(const auto& r : task_result.skill_results){
            nlohmann::json skill_result;
            skill_result["cost"] = r.second.cost.to_json();
            skill_result["heuristic"] = r.second.heuristic;
            skill_results[r.first]=skill_result;
        }
        task_result_response["skill_results"]=skill_results;
    }else{
        task_result_response["success"]=false;
        task_result_response["external_stop"]=false;
        task_result_response["exception"]=true;
        task_result_response["results"]=nlohmann::json();
        task_result_response["error"]={};
        task_result_response["skill_results"]=nlohmann::json();
    }
    response["result"]=result;
    response["error"]=error_message;
    response["task_result"]=task_result_response;
    return response;
}

nlohmann::json CommandInterface::is_busy([[maybe_unused]] const nlohmann::json &request){
    spdlog::trace("CommandInterface:is_busy()");
    nlohmann::json response;
    response["result"]=true;
    response["busy"]=m_task_engine->is_busy();
    return response;
}

nlohmann::json CommandInterface::set_grasped_object(const nlohmann::json &request){
    spdlog::trace("CommandInterface:set_grasped_object()");
    nlohmann::json response;
    response["result"]=false;
    response["error"]="";
    if(!m_core->set_grasped_object(request["object"])){
        response["error"]="Could not set grasped object.";
    }else{
        response["result"]=true;
    }
    return response;
}

nlohmann::json CommandInterface::grasp_object(const nlohmann::json &request){
    spdlog::trace("CommandInterface:grasp_object()");
    nlohmann::json response;
    response["result"]=false;
    response["error"]="";
    if(!m_core->grasp_object(request["object"],request["speed"])){
        response["error"]="Grasping has failed.";
    }else{
        response["result"]=true;
    }
    return response;
}

nlohmann::json CommandInterface::grasp(const nlohmann::json &request){
    spdlog::trace("CommandInterface:grasp()");
    nlohmann::json response;
    response["result"]=false;
    response["error"]="";
    if(!m_core->grasp(request["width"],request["speed"],request["force"],request["epsilon_inner"],request["epsilon_outer"])){
        response["error"]="Could not move gripper.";
    }else{
        response["result"]=true;
    }
    return response;
}

nlohmann::json CommandInterface::release_object(const nlohmann::json &request){
    spdlog::trace("CommandInterface:release_object()");
    spdlog::info("Releasing object");
    nlohmann::json response;
    std::optional<double> width;
    if(request["width"]==-1){
        width={};
    }else{
        double width_tmp;
        request["width"].get_to(width_tmp);
        width=width_tmp;
    }
    if(!m_core->release_object(width,request["speed"])){
        response["result"]=false;
        response["error"]="Releasing has failed.";
    }else{
        response["result"]=true;
    }
    return response;
}

nlohmann::json CommandInterface::move_gripper(const nlohmann::json &request){
    spdlog::trace("CommandInterface:move_gripper()");
    nlohmann::json response;
    response["result"]=false;
    response["error"]="";
    if(!m_core->move_gripper(request["width"],request["speed"])){
        response["error"]="Could not move gripper.";
    }else{
        response["result"]=true;
    }
    return response;
}

nlohmann::json CommandInterface::home_gripper([[maybe_unused]] const nlohmann::json &request){
    spdlog::trace("CommandInterface:home_gripper()");
    nlohmann::json response;
    response["result"]=false;
    response["error"]="";
    if(!m_core->home_gripper()){
        response["error"]="Homing has failed.";
    }else{
        response["result"]=true;
    }
    return response;
}

nlohmann::json CommandInterface::teach_object(const nlohmann::json &request){
    spdlog::trace("CommandInterface:teach_object()");
    nlohmann::json response;
    std::string object_name;
    bool teach_width=false;
    double teach_force=100;
    request["object"].get_to(object_name);
    if(request.contains("width")){
        request["width"].get_to(teach_width);
    }
    if(request.contains("force")){
        request["force"].get_to(teach_force);
    }
    bool result=true;
    std::string error_message="";
    if(!m_core->refresh_percept({})){
        error_message="Could not teach the object because no current percept is available.";
        result=false;
    }
    if(!m_memory->update_object(object_name,teach_width,teach_force,*m_core->get_percept())){
        error_message="Could not teach object because memory returned an error.";
        result=false;
    }
    response["result"]=result;
    response["error"]=error_message;
    return response;
}

nlohmann::json CommandInterface::set_partial_object_data(const nlohmann::json &request){
    spdlog::trace("CommandInterface:set_partial_object_data()");
    nlohmann::json response;
    std::string object_name;
    request["object"].get_to(object_name);
    bool result=true;
    std::string error_message="";
    if(!m_core->refresh_percept({})){
        error_message="Could not teach the object because no current percept is available.";
        result=false;
    }
    if(!m_memory->update_partial_object(object_name,request["data"])){
        error_message="Could not update object because memory returned an error.";
        result=false;
    }
    response["result"]=result;
    response["error"]=error_message;
    return response;
}

nlohmann::json CommandInterface::set_object(const nlohmann::json &request){
    spdlog::trace("CommandInterface:set_object()");
    nlohmann::json response;
    std::string name;
    request["object"].get_to(name);

    nlohmann::json description={
        {"O_T_OB",request["O_T_OB"]},
        {"OB_T_gp",request["OB_T_gp"]},
        {"OB_T_TCP",request["OB_T_TCP"]},
        {"OB_I",request["OB_I"]},
        {"q",request["q"]},
        {"grasp_width",request["grasp_width"]},
        {"grasp_force",request["grasp_force"]},
        {"mass",request["mass"]},
        {"geometry",request["geometry"]}
    };
    bool result = m_memory->update_object(name,description);
    response["error"]="";
    if(!result){
        response["error"]="Could not update object with name " + name;
    }
    response["result"]=result;

    return response;
}

//nlohmann::json CommandInterface::apply_reference_frame(const nlohmann::json &request){
//    nlohmann::json response;
//    if(!this->_core->get_kb()->apply_reference_frame(request["frame"])){
//        response["result"]=false;
//        response["error"]="Reference frame could not be applied.";
//    }else{
//        response["result"]=true;
//    }
//    return response;
//}

nlohmann::json CommandInterface::download_task_context(const nlohmann::json &request){
    spdlog::trace("CommandInterface:download_task_context()");
    nlohmann::json response, context;
    std::string task_id;
    request["task"].get_to(task_id);
    if(m_core->get_memory()->load_default_task_context(task_id,context)){
        response["context"]=context;
        response["result"]=true;
    }else{
        response["result"]=false;
        response["error"]="Could not download task with name "+task_id+".";
    }
    spdlog::debug("CommandInterface: download_task_context_end");
    return response;
}

nlohmann::json CommandInterface::download_skill_context(const nlohmann::json &request){
    spdlog::trace("CommandInterface:download_skill_context()");
    nlohmann::json response, context;
    std::string skill_id;
    request["skill"].get_to(skill_id);
    if(m_core->get_memory()->load_default_skill_context(skill_id,context)){
        response["context"]=context;
        response["result"]=true;
    }else{
        response["result"]=false;
        response["error"]="Could not download skill with name "+skill_id+".";
    }
    spdlog::debug("CommandInterface: download_skill_context");
    return response;
}

nlohmann::json CommandInterface::download_object_context(const nlohmann::json &request){
    spdlog::trace("CommandInterface:download_object_context()");
    nlohmann::json response, context;
    std::string object;
    request["object"].get_to(object);
    const Object* o = m_core->get_memory()->get_object(object);
    if(o->name!="NullObject"){
        response["context"]=o->to_json();
        response["result"]=true;
    }else{
        response["result"]=false;
        response["error"]="Could not download object with name "+object+".";
    }
    spdlog::debug("CommandInterface: download_object_context_end");
    return response;
}

nlohmann::json CommandInterface::get_state([[maybe_unused]] const nlohmann::json &request){
    spdlog::trace("CommandInterface:get_state()");
    nlohmann::json response;
    bool result=true;
    std::string error_message="";
    if(!m_core->refresh_percept({})){
        error_message="No current state available, could not refresh perception.";
        result=false;
    }
    const Percept* p = m_core->get_percept();
    mirmi_utils::write_json_array<double,7,1>(response["gravity"],p->internal_model.G);
    mirmi_utils::write_json_array<double,7,1>(response["q"],p->proprioception.q);
    mirmi_utils::write_json_array<double,4,4>(response["O_T_EE"],p->proprioception.O_T_EE);
    response["grasped_object"]=m_memory->get_live_context()->grasped_object->name;
    if(p->robot_mode==control::RobotMode::kIdle){
        response["status"]="Idle";
    }
    if(p->robot_mode==control::RobotMode::kReflex){
        response["status"]="Reflex";
    }
    if(p->robot_mode==control::RobotMode::kUserStopped){
        response["status"]="UserStopped";
    }
    response["result"]=result;
    response["error_message"]=error_message;
    response["current_task"] = m_core->get_task_engine()->get_active_task_id();
    response["gripper_width"] = p->proprioception.finger_width;

    return response;
}

nlohmann::json CommandInterface::get_model([[maybe_unused]] const nlohmann::json &request){
    spdlog::trace("CommandInterface:get_model()");
    nlohmann::json response;
    bool result=true;
    std::string error_message="";
    if(!m_core->refresh_percept({})){
        error_message="No current state available, could not refresh perception.";
        result=false;
    }
    const Percept* p = m_core->get_percept();
    mirmi_utils::write_json_array<double,7,7>(response["M"],p->internal_model.M);
    mirmi_utils::write_json_array<double,7,1>(response["C"],p->internal_model.C);
    mirmi_utils::write_json_array<double,7,1>(response["G"],p->internal_model.G);
    mirmi_utils::write_json_array<double,6,7>(response["B_J_O"],p->internal_model.B_J_O);
    mirmi_utils::write_json_array<double,6,7>(response["B_J_EE"],p->internal_model.B_J_EE);
    response["result"]=result;
    response["error_message"]=error_message;

    return response;
}

nlohmann::json CommandInterface::reload_database([[maybe_unused]] const nlohmann::json &request){
    spdlog::trace("CommandInterface:reload_database()");
    nlohmann::json response;
    response["result"]=true;
    if(!m_memory->set_default_parameters()){
        response["result"]=false;
        response["error_message"]="Could not load from database.";
    }
    return response;
}

nlohmann::json CommandInterface::subscribe_telemetry(const nlohmann::json &request){
    spdlog::trace("CommandInterface:subscribe_telemetry()");
    nlohmann::json response;
    response["result"] = true; 
    if(request.find("ip") == request.end()){
        response["result"] = false;
        response["error"] = "CommandInterface.subscribe_telemetry: No ip in request "+request.dump();
        return response;
    }
    if(request.find("port") == request.end()){
        response["result"] = false;
        response["error"] = "CommandInterface.subscribe_telemetry: No port in request "+request.dump();
        return response;
    }
    if(request.find("subscribe") == request.end()){
        response["result"] = false;
        response["error"] = "CommandInterface.subscribe_telemetry: No subscribe in request "+request.dump();
        return response;
    }
    bool sendWithTerminatingNullCharacter = false;
    if (request.find("withTerminatingNullCharacter") != request.end()) {
        sendWithTerminatingNullCharacter = request["withTerminatingNullCharacter"];
    }

    if(!m_core->get_telemetry()->add_subscriber(request["ip"], request["port"], request["subscribe"],
                                                sendWithTerminatingNullCharacter)){
        response["result"] = false;
        response["error"] = "Could not add subscriber "+request.dump();
    };
    return response;
}
nlohmann::json CommandInterface::unsubscribe_telemetry(const nlohmann::json &request){
    spdlog::trace("CommandInterface:unsubscribe_telemetry()");
    nlohmann::json response;
    response["result"] = m_core->get_telemetry()->remove_subscriber(request["ip"]);
    return response;
}
//nlohmann::json CommandInterface::subscribe_to_event_stream(const nlohmann::json &request){
//    nlohmann::json response;
//    EventSubscriber subscriber;
//    request["address"].get_to(subscriber.address);
//    request["port"].get_to(subscriber.port);
//    request["endpoint"].get_to(subscriber.endpoint);
//    request["method_name"].get_to(subscriber.method_name);

//    response["subscriber_uuid"] = EventPublisher::subscribe(subscriber);
//    return response;
//}

//nlohmann::json CommandInterface::unsubscribe_from_event_stream(const nlohmann::json &request){
//    nlohmann::json response;
//    std::string subscriber_uuid;
//    request["subscriber_uuid"].get_to(subscriber_uuid);
//    EventPublisher::unsubscribe(subscriber_uuid);
//    return response;
//}

nlohmann::json CommandInterface::start_desk_task(const nlohmann::json &request){
    spdlog::trace("CommandInterface:start_desk_task()");
    nlohmann::json response;
    std::string task;
    request["task"].get_to(task);
    bool result=m_core->start_desk_task(task);
    response["result"]=result;
    return response;
}

nlohmann::json CommandInterface::stop_desk_task([[maybe_unused]] const nlohmann::json &request){
    spdlog::trace("CommandInterface:stop_desk_task()");
    nlohmann::json response;
    bool result=m_core->stop_desk_task();
    response["result"]=result;
    return response;
}

nlohmann::json CommandInterface::unlock_brakes([[maybe_unused]] const nlohmann::json &request){
    spdlog::trace("CommandInterface:unlock_brakes()");
    nlohmann::json response;
    bool result=m_core->unlock_body();
    response["result"]=result;
    return response;
}

nlohmann::json CommandInterface::lock_brakes([[maybe_unused]] const nlohmann::json &request){
    spdlog::trace("CommandInterface:lock_brakes()");
    nlohmann::json response;
    bool result=m_core->lock_body();
    response["result"]=result;
    return response;
}

nlohmann::json CommandInterface::shutdown([[maybe_unused]] const nlohmann::json &request){
    spdlog::trace("CommandInterface:shutdown()");
    nlohmann::json response;
    m_core->terminate();
    bool result=m_core->shutdown_body();
    response["result"]=result;
    return response;
}

nlohmann::json CommandInterface::reboot([[maybe_unused]] const nlohmann::json &request){
    spdlog::info("CommandInterface:reboot()");
    nlohmann::json response;
    bool result=m_core->reboot_body();
    if(result){
        spdlog::info("Rebooting robot arm... Shutdown MIOS. You can restart MIOS as soon as the robot is booted.");
        result = this->terminate(request);
    }
    response["result"]=result;
    return response;
}

nlohmann::json CommandInterface::pack_pose([[maybe_unused]] const nlohmann::json &request){
    spdlog::trace("CommandInterface:pack_pose()");
    nlohmann::json response;
    response["error"]="";
    response["result"]=true;
    if(m_core->is_grasping()){
        response["error"]="The robot might be holding an object. Moving to pack pose is not safe.";
        response["result"]=false;
    }else{
        response["result"]=m_core->unlock_body();
    }
    return response;
}

nlohmann::json CommandInterface::set_live_parameter(const nlohmann::json &request){
    spdlog::trace("CommandInterface:set_live_parameter()");
    nlohmann::json response;
    m_memory->set_live_parameter(request["key"],request["value"]);
    response["error"]="";
    response["result"]=true;
    return response;
}

nlohmann::json CommandInterface::post_event(const nlohmann::json &request){
    spdlog::trace("CommandInterface:post_event()");
    nlohmann::json response;
    std::string name;
    request["name"].get_to(name);
    m_memory->post_event(name,request["content"]);
    response["result"]=true;
    return response;
}

nlohmann::json CommandInterface::get_event(const nlohmann::json &request){
    spdlog::trace("CommandInterface:get_event()");
    nlohmann::json response;
    std::string name;
    request["name"].get_to(name);

    const Event* event=m_memory->get_event(name);
    if(event==nullptr){
        response["content"]="INVALID_EVENT";
    }else{
        response["content"]=event->get_content();
    }
    return response;
}

nlohmann::json CommandInterface::learn_task(const nlohmann::json &request){
    spdlog::trace("CommandInterface:learn_task()");
    nlohmann::json response;
    bool result=true;

    // Problem definition checks
    if(request["problem_definition"].find("domain")==request["problem_definition"].end()){
        response["error"]="Problem definition is missing a domain.";
        result=false;
    }else if(request["problem_definition"]["domain"].find("limits")==request["problem_definition"]["domain"].end()){
        response["error"]="Domain is missing limits.";
        result=false;
    }else if(request["problem_definition"]["domain"].find("context_mapping")==request["problem_definition"]["domain"].end()){
        response["error"]="Domain is missing context mapping.";
        result=false;
    }

    if(request["service_configuration"].find("service_name")==request["service_configuration"].end()){
        response["error"]="Configuration is missing a service name.";
        result=false;
    }

    if(result){
        spdlog::debug("CommandInterface::learn_task.to_learning_module");
        response["problem_uuid"] = m_core->get_learning_module()->learn_task(request["problem_definition"],request["service_configuration"],request["agents"]);
    }else{
        response["uuid"]="INVALID";
    }
    response["result"]=result;
    return response;
}

nlohmann::json CommandInterface::stop_learning([[maybe_unused]] const nlohmann::json &request){
    spdlog::trace("CommandInterface::stop_learning()");
    return nlohmann::json();
}

}
