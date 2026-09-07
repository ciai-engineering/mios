#include "mios/task/task_engine.hpp"

#include "mios/task/taskfactory.hpp"
#include "mios/core/core.hpp"
#include "mios/memory/memory.hpp"

#include "spdlog/spdlog.h"

#include <sstream>

namespace mios {



TaskEngine::TaskEngine(Core *core):m_keep_running(true),m_core(core),m_memory(core->get_memory()),m_task_life_cycle(TaskLifeCycle::Switch),m_active_task(TaskFactory::create_task(TaskName::TaskNameIdleTask,core)){
    spdlog::trace("TaskEngine::TaskEngine");
}

void TaskEngine::reset(){

}

void TaskEngine::stop(){
    m_keep_running=false;
    m_active_task->stop_task(false,false,true);
}

std::string TaskEngine::get_active_task_id() const{
    return m_active_task->get_id();
}

void TaskEngine::life_cycle(){
    bool user_stop=false;
    bool invalid_mode=false;
    bool guiding=false;
    bool reflex=false;
    bool recovery=false;
    while(m_keep_running){
        if(m_core->m_context.shutdown_signal){
            m_core->terminate();
        }
        if(m_task_life_cycle==TaskLifeCycle::PreChecks){
            if(!m_core->is_ready()){
                spdlog::warn("Core is not ready, I will attempt to reinitialize...");
                if(!m_core->initialize()){
                    spdlog::error("Core initialization failed. I will re-attempt in 1 second.");
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    continue;
                }
            }
            control::RobotMode mode;
            if(m_core->refresh_percept({})){
                mode=m_core->get_percept()->robot_mode;
            }else{
                mode=control::RobotMode::kOther;
            }
            // A ROS controller changes Franka to MOVE as soon as it claims a
            // command interface, even when its fail-safe command is zero. The
            // legacy FCI backend never had that state before a task started,
            // so retain its IDLE-only behavior unless the ROS-only runtime
            // explicitly commissions this ownership hand-off.
            if(mode==control::RobotMode::kMove &&
               !m_core->m_context.config.allow_controller_owned_move_mode){
                spdlog::critical("Robot is moving but it is not expected that it moves in the current state.");
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
            // Handle reflex
            if(reflex && mode!=control::RobotMode::kReflex){
                reflex=false;
            }
            if(!reflex && mode==control::RobotMode::kReflex){
                spdlog::warn("Robot has executed a reflex, attempting to recover...");
                if(!m_core->recover_body()){
                    spdlog::error("Automatic recovery failed, please toggle the user stop...");
                    m_mtx_task_queue.lock();
                    m_task_queue.clear();
                    m_mtx_task_queue.unlock();
                }
                reflex=true;
                continue;
            }
            if(reflex && mode==control::RobotMode::kReflex){
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            // Handle other mode
            if(invalid_mode && mode!=control::RobotMode::kOther){
                invalid_mode=false;
            }
            if(!invalid_mode && mode==control::RobotMode::kOther){
                spdlog::trace("TaskEngine::invalid_mode_recovery");
                if(!m_core->recover_body()){
                    spdlog::error("Robot is in invalid mode. Check if the brakes are locked or please toggle the user stop...");
                    m_mtx_task_queue.lock();
                    m_task_queue.clear();
                    m_mtx_task_queue.unlock();
                }
                invalid_mode=true;
                continue;
            }
            if(invalid_mode && mode==control::RobotMode::kOther){
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            // Handle automatic error recovery
            if(recovery && mode!=control::RobotMode::kAutomaticErrorRecovery){
                recovery=false;
            }
            if(!recovery && mode==control::RobotMode::kAutomaticErrorRecovery){
                spdlog::trace("TaskEngine::life_cycle.recovery_mode");
                m_mtx_task_queue.lock();
                m_task_queue.clear();
                m_mtx_task_queue.unlock();
                recovery=true;
                continue;
            }
            if(recovery && mode==control::RobotMode::kAutomaticErrorRecovery){
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            // Handle guiding mode
            if(guiding && mode!=control::RobotMode::kGuiding){
                guiding=false;
            }
            if(!guiding && mode==control::RobotMode::kGuiding){
                spdlog::warn("Robot is in guiding mode, waiting for stop...");
                m_mtx_task_queue.lock();
                m_task_queue.clear();
                m_mtx_task_queue.unlock();
                guiding=true;
                continue;
            }
            if(guiding && mode==control::RobotMode::kGuiding){
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            // Handle user stop
            if(user_stop && mode!=control::RobotMode::kUserStopped){
                user_stop=false;
            }
            if(!user_stop && mode==control::RobotMode::kUserStopped){
                spdlog::warn("User stop has been pressed, waiting for release...");
                m_mtx_task_queue.lock();
                m_task_queue.clear();
                m_mtx_task_queue.unlock();
                user_stop=true;
                continue;
            }
            if(user_stop && mode==control::RobotMode::kUserStopped){
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            m_mtx_task_queue.lock();
            if(m_task_queue.empty()){
                m_task_life_cycle=TaskLifeCycle::Switch;
            }else{
                m_task_life_cycle=TaskLifeCycle::Startup;
            }
            m_mtx_task_queue.unlock();
        }
        bool exceptional_event=false;
        if(m_task_life_cycle==TaskLifeCycle::Startup){
            spdlog::debug("TaskLifeCycle: startup, task_uuid: "+m_active_task->get_uuid());
            spdlog::info("Loading task " + m_active_task->get_id() + " with uuid " + m_active_task->get_uuid());
            m_memory->get_live_context()->t_task=std::chrono::high_resolution_clock::now();
            m_task_life_cycle=TaskLifeCycle::Execution;
        }
        if(m_task_life_cycle==TaskLifeCycle::Execution){
            spdlog::debug("TaskLifeCycle: execution, task_uuid: "+m_active_task->get_uuid());
            try{
                spdlog::info("Executing task " + m_active_task->get_id() + " with uuid " + m_active_task->get_uuid());
                m_active_task->execute();
            }catch(const TaskException& e){
                spdlog::error("A task exception occured.");
                exceptional_event=true;
            }catch(const std::exception& e){
                spdlog::critical("An unexpected exception occured: " + std::string(e.what()));
                exceptional_event=true;
            }
            if(exceptional_event){
                m_task_life_cycle=TaskLifeCycle::Termination;
            }else{
                m_task_life_cycle=TaskLifeCycle::Recovery;
            }
        }
        if(m_task_life_cycle==TaskLifeCycle::Recovery){
            spdlog::debug("TaskLifeCycle: recovery, task_uuid: "+m_active_task->get_uuid());
            try{
                if(m_active_task->do_recovery()){
                    spdlog::info("Recovering task " + m_active_task->get_id() + " with uuid " + m_active_task->get_uuid());
                    m_active_task->start_recovery();
                    m_active_task->recover_task();
                    m_active_task->complete_recovery();
                }
            }catch(const TaskException& e){
                spdlog::error("A task exception occured.");
                exceptional_event=true;
            }catch(const std::exception& e){
                spdlog::critical("An unexpected exception occured: " + std::string(e.what()));
                exceptional_event=true;
            }
            m_task_life_cycle=TaskLifeCycle::Termination;
        }
        if(m_task_life_cycle==TaskLifeCycle::Termination){
            spdlog::debug("TaskLifeCycle: termination, task_uuid: "+m_active_task->get_uuid());
            m_active_task->write_result();
            m_memory->store_task_data(m_active_task->get_uuid(),m_active_task->get_id(),m_active_task->get_context(),m_active_task->get_result());
            m_memory->clear_reserved_skills();
            m_memory->clear_skill_parameters();
            m_task_life_cycle=TaskLifeCycle::Switch;
        }
        if(m_task_life_cycle==TaskLifeCycle::Switch){
            std::scoped_lock<std::mutex> queue_lock(m_mtx_task_queue);
            spdlog::debug("TaskLifeCycle: switch, task_uuid: "+m_active_task->get_uuid());
            if(m_active_task->get_result().exception || exceptional_event || m_active_task->get_result().empty_queue){
                spdlog::trace("TaskEngine::life_cycle.empty_queue");
                m_task_queue.clear();
            }
            if(!m_task_queue.empty()){
                spdlog::trace("TaskEngine::life_cycle.get_first_element");
                m_task_queue.pop_front();
            }
            if(m_task_queue.empty()){
                spdlog::trace("TaskEngine::life_cycle.add_idle_task");
                m_task_queue.emplace_back(std::make_tuple("IdleTask",m_memory->load_task("IdleTask",nlohmann::json(),m_core),nlohmann::json()));
            }
            m_active_task = std::get<1>(m_task_queue.front());
            m_task_life_cycle = TaskLifeCycle::PreChecks;
            spdlog::debug("TaskLifeCycle: switched to task "+m_active_task->get_id() + " with uuid " + m_active_task->get_uuid());
        }
    }
}

std::tuple<bool,std::string,std::string> TaskEngine::start_task(const std::string &task_id, const nlohmann::json &parameters, bool queue_task){
    std::scoped_lock<std::mutex> queue_lock(m_mtx_task_queue);
    std::string err="";
    std::string task_uuid="INVALID";
    spdlog::debug("TaskEngine::start_task");
    bool result=false;

    if(task_id=="IdleTask"){
        err="Cannot explicitly start idle task.";
        task_uuid="INVALID";
        result=false;
        return std::make_tuple(result,task_uuid,err);
    }

    if(!queue_task){
        spdlog::debug("TaskEngine::start_task.task_name: " + m_active_task->get_id());
        spdlog::debug("TaskEngine::start_task.queue_size: " + std::to_string(m_task_queue.size()));
        if(m_active_task->get_id()=="IdleTask" && m_task_queue.size()==1){
            queue_task=true;
        }else{
            err="Cannot start new task while another task (" + m_active_task->get_id() + ") is still active. In order to queue a new task set the argument queue to true.";
            task_uuid="INVALID";
            result=false;
        }
    }
    if(queue_task){
        TaskName task_name = TaskFactory::get_task_name(task_id);
        if(task_name==TaskName::TaskNameNullTask){
            err="No task with name " + task_id + " exists.";
            task_uuid="INVALID";
            result=false;
        }else{
            spdlog::debug("TaskEngine::load_task");
            std::shared_ptr<Task> new_task=m_memory->load_task(task_id,parameters,m_core);
            if(new_task->get_id()!="NullTask"){
                spdlog::info("Queuing task with uuid " + new_task->get_uuid());
                m_task_queue.emplace_back(std::tuple<std::string,std::shared_ptr<Task>,nlohmann::json>(new_task->get_uuid(),new_task,parameters));
                result=true;
                task_uuid=new_task->get_uuid();
                if(m_active_task->get_id()=="IdleTask"){
                    spdlog::debug("Stopping IdleTask with uuid " +m_active_task->get_uuid());
                    m_active_task->stop_task(false,true,false);
                }
            }else{
                result=false;
                task_uuid="INVALID";
            }
        }
    }
    return std::make_tuple(result,task_uuid,err);
}

std::pair<bool,std::string> TaskEngine::stop_task(bool raise_exception, bool recover,bool empty_queue){
    if(m_active_task->get_id()=="IdleTask"){
        return std::make_pair(true,"");
    }
    spdlog::info("Stopping active task.");
    m_active_task->stop_task(raise_exception,recover,empty_queue);
    return std::make_pair(true,"");
}

std::pair<bool,std::string> TaskEngine::remove_task(const std::string& uuid){
    std::scoped_lock<std::mutex> lock(m_mtx_task_queue);
    auto it=m_task_queue.begin();
    while(it!=m_task_queue.end()){
        if(std::get<1>(*it)->get_uuid()==uuid && m_task_queue.begin()==it){
            return std::make_pair(false,"Cannot remove active task_from queue");
        }else if(std::get<1>(*it)->get_uuid()==uuid){
            m_task_queue.erase(it++);
            return std::make_pair(true,"Removed task with uuid "+uuid+" from queue.");
        }else{
            ++it;
        }
    }
    return std::make_pair(false,"No task with uuid "+uuid+" in queue.");
}

bool TaskEngine::subscribe(const std::string& task_uuid, std::shared_ptr<TaskObserver> observer){
    std::scoped_lock<std::mutex> lock(m_mtx_task_queue);
    bool found_uuid=false;
    for(const auto& t : m_task_queue){
        if(std::get<1>(t)->get_uuid()==task_uuid){
            std::get<1>(t)->subscribe(observer);
            found_uuid=true;
        }
    }
    if(!found_uuid){
        return false;
    }else{
        spdlog::info("Subscribing to task with uuid "+task_uuid+".");
        return true;
    }
}

std::tuple<bool,TaskResult,std::string> TaskEngine::wait_for_task(const std::string &task_uuid){
    std::shared_ptr<TaskObserver> observer = std::make_shared<TaskObserver>();
    TaskData task_data;
    std::string err="";
    bool result=false;
    if(this->subscribe(task_uuid,observer)){
        observer->wait_for_finish();
    }/*else{
        spdlog::error("Could not subscribe to task with uuid " + task_uuid);
        err = "Could not subscribe to task with uuid " + task_uuid;
        result = false;
        return std::make_tuple(result,task_data.result,err);
    }*/
    if(m_memory->get_task_data(task_uuid,task_data)){
        spdlog::info("Loaded task result for task with uuid "+task_uuid+" from memory.");
        result=true;
    }else{
        err="I have no memory of a task with uuid " + task_uuid;
        result=false;
    }
    return std::make_tuple(result,task_data.result,err);
}

bool TaskEngine::is_busy() const{
    if((m_task_life_cycle==TaskLifeCycle::Startup || m_task_life_cycle==TaskLifeCycle::Execution) && m_active_task->get_id()=="IdleTask"){
        return false;
    }else{
        return true;
    }
}

const std::list<std::tuple<std::string, std::shared_ptr<Task>, nlohmann::json> > *TaskEngine::get_task_queue(){
    std::scoped_lock<std::mutex> lock(m_mtx_task_queue);
    return &m_task_queue;
}

}
