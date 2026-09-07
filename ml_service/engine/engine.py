import time
import datetime
import logging
import os
from threading import Thread
from threading import Lock
from queue import Queue
from queue import Empty
from copy import deepcopy
import uuid
import numpy as np
from mongodb_client.mongodb_client import MongoDBClient
from problem_definition.problem_definition import ProblemDefinition
#from collective_manager.video_recorder import FFMpegWebcamRecorder
from engine.task_result import TaskResult
from utils.exception import *
from utils.ws_client import *
from utils.helper_functions import *
import redis
import json


logger = logging.getLogger("ml_service")


class Trial:
    def __init__(self, task_context: dict, reset_instructions: list, rescue_instructions:list, theta: dict, log: bool = True, external = False):
        self.task_context = task_context
        self.reset_instructions = reset_instructions
        self.rescue_instructions = rescue_instructions
        self.theta = theta
        self.task_result = TaskResult()

        self.t_0 = None
        self.t_1 = None
        self.t_delta = None

        self.task_uuid = "INVALID"
        self.trial_uuid = "INVALID"
        self.agent = "INVALID"

        self.trial_number = 0
        self.log = log

        self.external = external

    def to_dict(self):
        for key in self.theta.keys():
            self.theta[key] = float(self.theta[key])
        trial_dict = {
            "task_context": self.task_context,
            "reset_instructions": self.reset_instructions,
            "theta": self.theta,
            "task_result": self.task_result.to_dict(),
            "t_0": self.t_0,
            "t_1": self.t_1,
            "t_delta": self.t_delta,
            "task_uuid": self.task_uuid,
            "trial_uuid": self.trial_uuid,
            "agent": self.agent,
            "trial_number": self.trial_number,
            "log": self.log,
            "external": self.external
        }
        return trial_dict

    def is_valid(self):
        if "name" not in self.task_context:
            logger.error("Task context has no name.")
            return False
        return True


class Engine:
    def __init__(self, agents: set = None, mios_port=12000, mongo_port=27017):
        logger.debug("Engine.__init__(" + str(agents) + ") with mios-port:" + str(mios_port))
        if agents is None:
            agents = set()
        self.mios_port = mios_port
        self.mongo_port = mongo_port
        self.agents = agents
        self.free_agents = agents
        self.queued_trials = Queue()
        self.completed_trials = dict()
        # Redis is only an optional notification sink.  Keep it disabled by
        # default for a standalone robot deployment; enable it explicitly for
        # a collective-learning deployment with MIOS_ENABLE_REDIS=true.
        self.redisClient = None
        if os.getenv("MIOS_ENABLE_REDIS", "false").strip().lower() in {"1", "true", "yes", "on"}:
            try:
                self.redisClient = redis.Redis(
                        host="redis-master.global",
                        port=6379,
                        db=0,
                        decode_responses=True,
                        password="QqJ3JDqNjN",
                        socket_connect_timeout=2,
                        socket_timeout=2,
                     )
                self.redisClient.ping()
            except redis.RedisError as e:
                logger.error("Redis connection failed: " + str(e))
                self.redisClient = None
        self.database_client = MongoDBClient(port=self.mongo_port)
        self.log_client = MongoDBClient(
            os.getenv("MIOS_ML_LOG_MONGO_HOST", "localhost"),
            port=self.mongo_port,
        )
        self.database_results_collection = None
        self.database_results_id = None

        self.problem_definition = None
        self.meta_data = dict()

        self.keep_running = False
        self.pause_execution = False
        self.max_trial_repeats = 3

        self.cnt_trial = 0
        self.cnt_pushed = 0
        self.cnt_completed = 0
        self.cnt_optimal = 0
        self.stop_condition = None

        self.x = np.empty((0, 0))
        self.y = np.empty((0,))

        self.lock_data = Lock()

        self.exploration_mode = False
        #self.camera_path =  os.getenv("cameraPath")
        #if self.camera_path is None:
        #    self.camera_path = "/dev/video4"
        #self.video_recorder = FFMpegWebcamRecorder(self.camera_path)
        self.skill_count=0

    def _get_log_timestamp(self) -> str:
        return datetime.datetime.now(datetime.timezone.utc).isoformat()
    

    def _get_trial_job_name(self, trial: Trial) -> str:
        job_name = trial.task_context["name"]
        for skill_name in trial.task_context.get("parameters", {}).get("skill_names", []):
            skill = trial.task_context.get("skills", {}).get(skill_name, {})
            objects = skill.get("skill", {}).get("objects", {})
            if "Insertable" in objects:
                return objects["Insertable"]
        return job_name

    def initialize(self, problem_definition: ProblemDefinition, exploration_mode: bool = False):
        self.x = np.empty((0, len(problem_definition.domain.limits)))
        self.exploration_mode = exploration_mode
        return self.initialize_results(problem_definition)
 
    def register_stop_condition(self, stop_condition):  # unused?
        self.stop_condition = stop_condition

    def add_agent(self, agent: str):
        logger.debug("Engine.add_agent(" + str(agent) + ")")
        self.agents.add(agent)

    def remove_agent(self, agent: str):
        logger.debug("Engine.remove_agent(" + str(agent) + ")")
        if agent in self.agents:
            self.agents.remove(agent)

    def stop(self):
        logger.info("Engine.stop() - received stop signal.")
        self.keep_running = False
    
    def pause(self):
        self.pause_execution = True

    def resume(self):
        self.pause_execution = False

    def push_trial(self, trial: Trial) -> str:
        #logger.debug("Engine.push_trial()")
        if trial.is_valid() is False:
            return "INVALID"
        trial.trial_uuid = str(uuid.uuid4())
        self.cnt_pushed += 1
        self.queued_trials.put(deepcopy(trial))
        return trial.trial_uuid

    def wait_for_trial(self, trial_uuid: str, max_wait_time: float) -> Trial:
        #logger.debug("Engine.wait_for_trial(" + trial_uuid + ", " + str(max_wait_time) + ")")
        t_0 = time.time()
        while trial_uuid not in self.completed_trials:
            # logger.debug("Engine::wait_for_trial.loop")
            if time.time() - t_0 > max_wait_time:
                logger.error("Wait time for trial has been exceeded.")
                return Trial(dict(), [],[], dict(), False)
            if self.keep_running is False:
                logger.error("Service has been stopped.")
                return Trial(dict(), [],[], dict(), False)
            # time.sleep(0.1)

        self.cnt_completed += 1

        return self.completed_trials[trial_uuid]

    def initialize_results(self, problem_definition: ProblemDefinition):
        self.problem_definition = problem_definition
        self.meta_data = problem_definition.to_dict()
        self.meta_data["t_0"] = time.time()
        now = datetime.datetime.now()
        now.strftime("%Y-%m-%d_%H:%M:%S")
        self.meta_data["date"] = now.strftime("%Y-%m-%d_%H:%M:%S")
        self.database_results_collection = self.database_client.client.ml_results[problem_definition.skill_class]
        self.database_results_id = self.database_results_collection.insert_one(
            {"meta": self.meta_data}).inserted_id
        return self.database_results_id

    def is_learned(self) -> bool:
        if self.exploration_mode is True:
            #logger.debug("Engine.is_learned() -> exploration_mode=True")
            return False
        else:
            #logger.debug("Engine.is_learned()? "+str(self.cnt_optimal > self.problem_definition.cost_function.finish_thr))
            return self.cnt_optimal > self.problem_definition.cost_function.finish_thr  # finsih threshold states how often a optimal_thr was undercut to finish

    def main_loop(self):
        logger.debug("Engine.main_loop()")
        self.cnt_trial = 1
        self.keep_running = True
        worker_threads = dict()
        for a in self.agents:
            worker_threads[a] = None

        logger.info("Setting up experiment.")
        for a in self.free_agents:
            worker_threads[a] = Thread(target=self.setup_experiment, args=(a,))
            worker_threads[a].start()

        for a in self.free_agents:
            worker_threads[a].join()

        logger.info("Setup procedure done.")

        while self.keep_running is True:
            try:
                #logger.debug("Engine::main_loop.get_trial")
                trial = self.queued_trials.get(False)
                # logger.debug("Engine::main_loop.new_trial: " + trial.trial_uuid)
            except Empty:
                # time.sleep(0.1)
                continue
            # logger.debug("Engine.main_loop.while1: For trial_uuid: " + trial.trial_uuid)
            thread_started = False
            while self.keep_running is True and thread_started is False:
                # logger.debug("Engine::main_loop.while2")
                if self.is_learned() is True:
                    logger.debug("Engine::main_loop.is_learned")
                    self.keep_running = False
                    continue
                for a in self.agents.copy():
                    if a not in self.free_agents:
                        logger.debug("Agent " + a + " not in self.free_agents")
                        # time.sleep(1)
                        continue
                    if worker_threads[a] is not None and worker_threads[a].is_alive() is True:
                        # logger.debug("Thread of agent " + a + " is alive")
                        time.sleep(0.1)
                        continue

                    # logger.debug("Engine.main_loop().is_busy(" + a + ")")
                    response = call_method(a, self.mios_port, "is_busy")
                    if response is None:
                        logger.debug("is_busy on agent " + a + ": response is None")
                        # time.sleep(1)
                        continue
                    if response["result"]["busy"] is True:
                        logger.debug("is_busy on agent " + a + ": is busy")
                        # time.sleep(1)
                        continue

                    self.free_agents.remove(a)
                    trial.agent = a
                    worker_threads[a] = Thread(target=self._worker_loop, args=(a, trial,))
                    worker_threads[a].start()
                    thread_started = True
                    break

            # time.sleep(0.1)

        logger.debug("Engine::main_loop.after_loop")
        self.write_final_results()
        for a in self.agents:
            if worker_threads[a] is not None:
                worker_threads[a].join(5)
        logger.debug("Engine::main_loop.last_line")

    def _worker_loop(self, agent: str, trial: Trial):
        logger.debug("Engine._worker_loop(" + agent + ", " + trial.trial_uuid + ")")
        self._run_trial(agent, trial)
        self.free_agents.add(agent)
        logger.debug("Free agent " + agent)
        #self.video_recorder.stop_stream()

    def _run_trial(self, agent: str, trial: Trial):
        if trial.is_valid() is False:
            raise ProblemDefinitionError
        trial.trial_number = self.cnt_trial
        self.cnt_trial += 1
        trial.t_0 = time.time()
        start_timestamp = self._get_log_timestamp()
        end_timestamp = start_timestamp
        job_name = self._get_trial_job_name(trial)
        doc = {
            "jobName" : job_name,
            "timestamp": start_timestamp,
            "step": "running/" + str(trial.trial_number),
            "message": "start trial"
        }
        self.log_client.write("collective_learning_system", "jobs_log", doc)
        # start video recording
        folder = str(self.problem_definition.tags[0]) + "/" +str(self.problem_definition.tags[1]) + "/"+str(self.problem_definition.skill_instance)
        filename = "n_"+str(trial.trial_number)
        # if not self.video_recorder.start_stream(
        #     output_folder="videos/"+folder,
        #     base_filename=filename,
        #     compressed=False,rotate=True,
        #     framerate="30",
        #     resolution="1920x1080",
        #     pixel_format="yuyv422"):
        #     logger.error("!!!!Cannot start video recording!!!!")
        #     pass

        for i in range(self.problem_definition.n_variations):
            #print("Running variation " + str(i))
            self.problem_definition.apply_object_modifiers(trial.task_context)
            result, variation_result = self._execute_task(agent, trial)
            if self.keep_running is True:
                if result is False:
                    logger.warning("Could not execute task for agent " + agent + ". Trial will be re-inserted into queue.")
                    self.queued_trials.put(trial)
                    self._reset_task(agent, trial)
                    return
            else:
                self._reset_task(agent, trial)
                break

            theta = np.zeros((1, (len(self.problem_definition.domain.limits))))
            for j in range(len(self.problem_definition.domain.limits)):
                theta[0][j] = trial.theta[self.problem_definition.domain.vector_mapping[j]]

            if i == 0:
                trial.task_result = variation_result
            else:
                trial.task_result.add_variation(variation_result.q_metric)

            trial.t_1 = time.time()
            end_timestamp = self._get_log_timestamp()
            trial.t_delta = trial.t_1 - trial.t_0
            #print(trial.trial_number)

            self.lock_data.acquire()
            if trial.task_result.q_metric.final_cost < 1:
                self.x = np.append(self.x, theta, axis=0)
                self.y = np.append(self.y, trial.task_result.q_metric.final_cost)
            self.lock_data.release()
            self._reset_task(agent, trial)
            
            if self.problem_definition.variate_only_success is True and trial.task_result.q_metric.success is False:
                #logger.debug("ENGINE: do not variate")
                break
            logger.debug("ENGINE:"+ str(i+1)+ ". variation done. Do "+str(self.problem_definition.n_variations)+" in total. ")

        #self.video_recorder.stop_stream()
        #logger.debug("Cost: " + str(trial.task_result.q_metric.final_cost))
        #logger.debug("FINISHED trial " + str(self.cnt_trial) + " with uuid " + trial.trial_uuid)
        if trial.task_result.q_metric.optimal is True:
            logger.debug("Engine::_worker_loop.is_optimal")
            self.cnt_optimal += 1

        trial.task_result.q_metric.heuristic = trial.task_result.q_metric.heuristic * (1 - trial.task_result.q_metric.success_rate)
        if trial.log is True:
            self.write_task_result(trial)
        #logger.debug("Engine::_worker_loop.trial_done")
        self.queued_trials.task_done()
        cost = str(variation_result.q_metric.final_cost) if variation_result is not None else "None"
        logger.debug(f"\n#################################\n ENGINE: success {str(trial.task_result.q_metric.success)} on trial {str(trial.trial_number)}\n"
            f"ENGINE: Cost: {cost} \n#################################\n")
        self.completed_trials[trial.trial_uuid] = deepcopy(trial)
        
        doc = {
            "jobName" : job_name,
            "timestamp": end_timestamp,
            "step": "running/" + str(trial.trial_number),
            "message": "finish trial with result: " + str(trial.task_result.q_metric.success)
        }
        self.log_client.write("collective_learning_system", "jobs_log", doc)
        doc = {
            "jobName" : job_name,
            "trial_no": trial.trial_number,
            "result": trial.task_result.q_metric.success,
            "start_timestamp": start_timestamp,
            "end_timestamp": end_timestamp,
            "theta": trial.theta if trial.task_result.q_metric.success is True else ""
        }
        self.log_client.write("collective_learning_system", "jobs_results", doc)

        if self.redisClient is not None:
            try: 
                self.redisClient.lpush("ml_result", json.dumps({"arm_label": trial.agent, "trial_count": trial.trial_number, "is_succeed": trial.task_result.q_metric.success}))
            except redis.RedisError as e:
                logger.error("Redis push data failed: " + str(e)) 

    def _execute_task(self, agent: str, trial: Trial) -> (bool, TaskResult):
        logger.debug("Engine._execute_task(" + agent + ") with trial " + trial.trial_uuid)
        # logger.debug("Engine::_execute_task.task_context: " + str(trial.task_context))
        cnt_repeat = -1
        variation_result = None
        while cnt_repeat < self.max_trial_repeats and self.keep_running is True:
            logger.debug("Engine::_execute_task.loop")
            cnt_repeat += 1
            for skill_name in trial.task_context["skills"].keys():

                trial.task_context["skills"][skill_name]["skill"] = udpate_dict(trial.task_context["skills"][skill_name]["skill"], self.problem_definition.add_skill_info)  # ["log_name"] = is log_data, log_name meta etc realy in trial.task_context??
                if "log_name" in self.problem_definition.add_skill_info:
                    trial.task_context["skills"][skill_name]["skill"]["log_name"]+= "n"+str(trial.trial_number)+"/learning"
                trial.task_context["skills"][skill_name]["skill"]["meta"] = {
                        "description":"Execution of a trial as part of the learning process.",
                    }
            #print(str(trial.task_context))
            result, task_uuid = self._start_task(agent, trial.task_context)
            if result is False:
                logger.error("Result was False after start_task")
                return False, None
            result, variation_result = self._wait_for_task(agent, task_uuid)
            if result is False:
                logger.error("Result was False after wait_for_task")
                return False, None
            if "TaskError" in trial.task_result.errors:
                logger.error("Received an task error, service will terminate.")
                self.stop()
                return False, None
            if "RealTimeError" in trial.task_result.errors:
                logger.warning("Received a realtime error, trial will be repeated.")
                time.sleep(1)
                return False, None
            if "UserStopped" in trial.task_result.errors:
                logger.warning("Received a user stop error, trial will be repeated.")
                time.sleep(1)
                return False, None
            variation_result.q_metric = self.problem_definition.calculate_cost(variation_result)
            #print(variation_result.q_metric.final_cost)
            break
        #logger.debug("Engine::_execute_task.end")
        return cnt_repeat < self.max_trial_repeats and self.keep_running is True, variation_result

    def _reset_task(self, agent: str, trial: Trial):
        logger.debug("Engine::_reset_task()")
        for i in trial.reset_instructions:
            #logger.debug("Engine::_reset_task.instructions: " + str(i["parameters"]))
            instruction_done = False
            while instruction_done is False:
                #logger.debug("Engine::_reset_task.loop")
                # append meta information to skill context
                for skill_name in i["parameters"]["skills"].keys():
                    i["parameters"]["skills"][skill_name]["skill"] = udpate_dict(i["parameters"]["skills"][skill_name]["skill"], self.problem_definition.add_skill_info)
                    if "log_name" in self.problem_definition.add_skill_info:
                        i["parameters"]["skills"][skill_name]["skill"]["log_name"] += "n"+str(trial.trial_number)+"/reset_trial"
                        i["parameters"]["skills"][skill_name]["skill"]["meta"] = {
                            "description":"Resetting the trial to initial state"
                        }
                if "preconditions" in i:
                    status = call_method(agent,self.mios_port,"get_state")
                    for precondition in i["preconditions"]:
                        if status is not None:  
                            if status["result"][precondition] != i["preconditions"][precondition]:
                                logger.debug("precondition for reset not fullfilled. Skipping reset...")
                                instruction_done = True
                                continue
                        else:
                            time.sleep(1)
                            continue
                if i["method"] == "start_task":
                    result, task_uuid = self._start_task(agent, i["parameters"])
                    if result is False:
                        logger.debug("Reset task could not be started.")
                        logger.debug(result)
                        time.sleep(1)
                        continue

                    result, task_result = self._wait_for_task(agent, task_uuid)
                    if result is False or task_result.q_metric.success is False:
                        logger.debug("Could not wait for reset_task. do rescue...")
                        logger.debug(result)
                        time.sleep(1)
                        self._rescue_task(agent, trial)
                        continue
                else:
                    response = call_method(agent, self.mios_port, i["method"], i["parameters"])
                    if response is None:
                        logger.debug(response)
                        time.sleep(1)
                        continue

                instruction_done = True
        #logger.debug("Engine::_reset_task.end")

    def _rescue_task(self,agent:str, trial:Trial):
        logger.debug("Engine::_rescue_task() - try this move once")
        for i in trial.rescue_instructions:
            for skill_name in i["parameters"]["skills"].keys():
                i["parameters"]["skills"][skill_name]["skill"] = udpate_dict(i["parameters"]["skills"][skill_name]["skill"], self.problem_definition.add_skill_info)
                if "log_name" in self.problem_definition.add_skill_info:
                    i["parameters"]["skills"][skill_name]["skill"]["log_name"] += "n"+str(trial.trial_number)+"/reset_trial"
                    i["parameters"]["skills"][skill_name]["skill"]["meta"] = {
                        "description":"Resetting the trial to initial state didn\'t work. Try to move away from stuck position"
                    }
            if "preconditions" in i:
                status = call_method(agent,self.mios_port,"get_state")
                for precondition in i["preconditions"]:
                    if status is not None:  
                        if status["result"][precondition] != i["preconditions"][precondition]:
                            logger.debug("precondition for reset not fullfilled. Skipping reset...")
                            instruction_done = True
                            continue
                    else:
                        time.sleep(1)
                        continue
            if i["method"] == "start_task":
                result, task_uuid = self._start_task(agent, i["parameters"])
                if result is False:
                    logger.debug("Rescue task could not be started.")
                    logger.debug(result)
                    time.sleep(1)
                    continue

                result, task_result = self._wait_for_task(agent, task_uuid)
                if result is False or task_result.q_metric.success is False:
                    logger.debug("Could not wait for rescue_task.")
                    logger.debug(result)
                    time.sleep(1)
                    continue
            else:
                response = call_method(agent, self.mios_port, i["method"], i["parameters"])
                if response is None:
                    logger.debug(response)
                    time.sleep(1)
                    continue

    def _start_task(self, agent: str, task_context: dict) -> (bool, str):
        task_uuid = "INVALID"
        task_name = task_context["name"]
        for skill_name in task_context["skills"].keys():
                if "log_name" in task_context["skills"][skill_name]["skill"]:
                    task_context["skills"][skill_name]["skill"]["log_name"] += "_"+skill_name+"-"+str(self.skill_count)
                    task_context["skills"][skill_name]["skill"]["meta"]["context"] = copy.deepcopy(task_context["skills"][skill_name])
                    task_context["skills"][skill_name]["skill"]["meta"]["time"] = time.time()
                    task_context["skills"][skill_name]["skill"]["meta"]["tags"] = self.problem_definition.tags
        while(self.pause_execution and self.keep_running):
            time.sleep(1)
        logger.info("_start_task::Executing task " + str(task_name) + " on agent " + str(agent) + ".")
        # logger.debug("Task context: " + str(task_context))
        response = start_task(agent, task_name, task_context, True, port=self.mios_port)
        if response is None:
            logger.warning("Agent " + agent + " is not responding.")
            time.sleep(1)
            return False, task_uuid

        if "result" not in response or "result" not in response["result"]:
            logger.warning("I received no proper response from agent " + agent + ".")
            logger.debug("Response was: " + str(response))
            time.sleep(1)
            return False, task_uuid

        if response["result"]["result"] is False:
            logger.warning("The task " + task_name + " could not be started on agent " + agent + ".")
            logger.warning("Received message: " + response["result"]["error"])
            time.sleep(1)
            return False, task_uuid

        if "task_uuid" not in response["result"] or response["result"]["task_uuid"] == "INVALID":
            logger.error("Response from agent " + agent + " did not contain a valid task uuid.")
            time.sleep(1)
            return False, task_uuid

        task_uuid = response["result"]["task_uuid"]
        self.skill_count+=1
        return True, task_uuid

    def _wait_for_task(self, agent: str, task_uuid: str) -> (bool, TaskResult):
        task_result = TaskResult()
        response = wait_for_task(agent, task_uuid, port=self.mios_port)
        # logger.debug("Engine._wait_for_task.response: " + str(response))
        if response is None:
            logger.warning("Agent " + agent + " is not responding.")
            time.sleep(1)
            return False, task_result

        if "result" not in response or "result" not in response["result"] or "task_result" not in response["result"]:
            logger.warning("I received no proper response from agent " + agent + ".")
            logger.debug("Response was: " + str(response))
            time.sleep(1)
            return False, task_result

        if response["result"]["result"] is False:
            logger.warning("The task " + task_uuid + " was not properly executed on " + agent + ".")
            logger.warning("Received message: " + response["result"]["error"])
            time.sleep(1)
            return False, task_result

        if task_result.calculate(response["result"]["task_result"]) is False:
            time.sleep(1)
            return False, task_result

        #logger.debug("Engine::_wait_for_task.end")
        return True, task_result

    def setup_experiment(self, agent):
        logger.debug("Engine::_reset_task()")
        for i in self.problem_definition.setup_instructions:
            logger.debug("Engine::setup_experiment.instructions: " + str(i["parameters"]))
            for skill_name in i["parameters"]["skills"]:
                i["parameters"]["skills"][skill_name]["skill"] = udpate_dict(i["parameters"]["skills"][skill_name]["skill"], self.problem_definition.add_skill_info)
                if "log_name" in self.problem_definition.add_skill_info:
                    i["parameters"]["skills"][skill_name]["skill"]["log_name"] += "setup_experiment"
                i["parameters"]["skills"][skill_name]["skill"]["meta"] = {
                        "description": "Setting up the experiment to initial state."
                    }
            instruction_done = False
            while instruction_done is False:
                logger.debug("Engine::_reset_task.loop")
                if i["method"] == "start_task":
                    result, task_uuid = self._start_task(agent, i["parameters"])
                    if result is False:
                        logger.debug("Setup experiment could not be started.")
                        logger.debug(result)
                        time.sleep(1)
                        continue

                    result, task_result = self._wait_for_task(agent, task_uuid)
                    if result is False or task_result.q_metric.success is False:
                        logger.debug("Could not wait for setup_experiment")
                        logger.debug(result)
                        time.sleep(1)
                        continue
                else:
                    response = call_method(agent, self.mios_port, i["method"], i["parameters"])
                    if response is None:
                        logger.debug(response)
                        time.sleep(1)
                        continue

                instruction_done = True

        #logger.debug("Engine::setup_experiment.end")

    def write_final_results(self):
        data = {
            "time": time.time() - self.meta_data["t_0"],
            "n_trials": self.cnt_trial - 1
        }
        self.database_results_collection.update_one({"_id": self.database_results_id},
                                                    {"$set": {"final_results": data}}, upsert=False)

    def write_task_result(self, trial: Trial):
        data = {
            "theta": trial.theta,
            "q_metric": trial.task_result.q_metric.to_dict(),
            "t_0": trial.t_0,
            "t_1": trial.t_1,
            "t_delta": trial.t_delta,
            "agent": trial.agent,
            "external": trial.external
        }
        #logger.debug("Engine::write_task_result.data: " + str(data))
        self.database_results_collection.update_one({'_id': self.database_results_id},
                                                    {'$set': {'n' + str(trial.trial_number): data}}, upsert=False)
