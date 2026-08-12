from desk.mongodb_client import MongoDBClient
import os
from utils.ws_client import *
import json
import socket

import time

class Task:
    def __init__(self, robot, port=12000):
        self.skill_names = []
        self.skill_types = []
        self.skill_context = dict()
        self.context = {
            "parameters": {
                "skill_names": [],
                "skill_types": [],
                "as_queue": False
            },
            "skills": self.skill_context
        }

        self.robot = robot
        self.port = port
        self.task_uuid = "INVALID"
        self.t_0 = 0

    def add_skill(self, name, skill_class, context):
        self.skill_names.append(name)
        self.skill_types.append(skill_class)
        self.skill_context[name] = context

        self.context["parameters"]["skill_names"] = self.skill_names
        self.context["parameters"]["skill_types"] = self.skill_types
        self.context["skills"] = self.skill_context

    def start(self, queue: bool = False):
        self.t_0 = time.time()
        self.context["parameters"]["as_queue"] = queue
        response = start_task(self.robot, "GenericTask", parameters=self.context, port=self.port)
        self.task_uuid = response["result"]["task_uuid"]

    def wait(self):
        result = wait_for_task(self.robot, self.task_uuid, port=self.port)
        #print("Task execution took " + str(time.time() - self.t_0) + " s.")
        return result

    def stop(self):
        result = stop_task(self.robot, port=self.port)
        #print("Task execution took " + str(time.time() - self.t_0) + " s.")
        return result

def get_ip(hostname: str):
    print("hostname: ",hostname)
    return socket.gethostbyname(hostname)

def populate_database(host:str, db:str, ip:str, user_name="franka", user_pw="frankaRSI"):
    '''
    host: mios IP
    db: mios Database (typically miosL)
    ip: IP of Robot ControlBox connected to the mios PC
    user_name: DESK username
    user_pw: DESK user password
    '''
    try:
        client = MongoDBClient(host)
        new_params = {"desk_name":user_name, "desk_pwd":user_pw,"robot_ip":ip, "spoc_token":"","spoc_in_control":False}
        client.update(db,"parameters",{"name":"system"}, new_params)
        print("updated ", host,": ",db)
    except:
            print(host, " not updated")

def teach_position(robot, position_name, teach_gripper_width=False):
    # Teaches the pose in Cartesian and joint space for the specified object. If the object does not existin a
    # new object is created. The object can also be a reference frame for other objects.
    # To teach panda have to be in guiding mode (white light at panda arm)
    return call_method(robot, 12000, "teach_object", {"object": position_name, "teach_width": teach_gripper_width})

def grasp(robot):
    # grasp sth smaller than 10cm (epsilon_outer=0.1)
    return call_method(robot, 12000, "grasp",
                       {"width": 0.0, "speed": 1, "force": 200, "epsilon_inner": 1, "epsilon_outer": 0.1})

def open_gripper(robot):
    # opens the gripper completely
    return call_method(robot, 12000, "release_object", {"speed": 1})

def move_gripper(robot,gripper_width):
    # open the gripper with gripper_width in [m] for e.g. 0.06 = 6cm
    return call_method(robot, 12000, "move_gripper", {"width": gripper_width, "speed": 0.15})

def set_grasped_object(robot, object_name):
    # set the grasped object so the robot know that it grabs something
    return call_method(robot, 12000, "set_grasped_object", {"object": object_name})


def move_to_contact(robot, location, port = 12000, wait=True):
    context = {
                "skill": {
                    "speed": 0.5,
                    "objects": {
                        "goal_pose": location
                    }
                },
                "control": {
                    "control_mode": 2
                },
                "user":{
                    "F_ext_contact": [10,5]
                }

            }
    t = Task(robot, port=port)
    t.add_skill("contact", "MoveToContact", context)
    t.start()
    if wait:
        return t.wait()

def move(robot:str, location:str, offset = [0,0,0], port=12000, wait = True,f_ext = [10,5], add_nullspace=False,
         p_g=[]):
    '''
    robot: ip of mios instance
    location: position name that was teached
    '''
    context = {
        "skill": {
            "p0":{
                "dX_d": [0.3, 0.8],
                "ddX_d": [0.5, 1],
                "K_x": [2000, 2000, 2000, 250, 250, 250],
                "T_T_EE_g_offset": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, offset[0], offset[1], offset[2], 1],
                "T_T_EE_g":p_g

            },
            "time_max":10,
            "objects": {
                    "GoalPose": location
                }
        },
        "control": {
            "control_mode": 0
        },
        "user":{
            "F_ext_max": f_ext,
            #"env_X": [0.002, 0.002, 0.002, 0.0175, 0.0175, 0.0175]  #[0.001, 0.001, 0.001, 0.001, 0.001, 0.001]
        }
    }
    if p_g:
        context["skill"]["objects"] = {}
    if add_nullspace:
        context["control"]["nullspace"] = {
                                                    "K_theta": [20, 20, 15, 10, 7, 5, 2],
                                                    "xi_theta": [0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7],
                                                    "active": True
                                                    }
    t = Task(robot, port=port)
    t.add_skill("move", "TaxMove", context)
    t.start()
    if wait:
        return t.wait()

    #print("Result: " + str(result))

def init_position(robot):
    import math
    # move robot to start position
    M_PI_2 = math.pi / 2
    M_PI_4 = math.pi / 4
    initial_joint_pose = [0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4]
    return start_task(robot, "MoveToJointPose", parameters={"parameters": {"q_g": initial_joint_pose, "pose":"NoneObject"}})


def move_joint(robot, location, port=12000, offset=[0,0,0,0,0,0,0], wait=True, speed = [], q_g=[]):
    '''
    robot: ip of mios instance
    location: position name that was teached
    '''
    path_to_default_context = os.getcwd() + "/taxonomy/default_contexts/"
    f = open(path_to_default_context + "move_joint.json")
    move_context = json.load(f)
    if not q_g:
        move_context["skill"]["objects"]["goal_pose"] = location
        move_context["skill"]["q_g_offset"] = offset
    else:
        move_context["skill"]["objects"]["goal_pose"] = "NoneObject"
        move_context["skill"]["q_g"] = q_g
    move_context["skill"]["time_max"] = 10
    move_context["user"]["env_X"] = [0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001]
    move_context["user"]["F_ext_max"] = [15,15]
    if speed:
        move_context["skill"]["speed"] = speed[0]
        move_context["skill"]["acc"] = speed[1]
    print(move_context)
    t0 = Task(robot, port=port)
    t0.add_skill("move", "MoveToPoseJoint", move_context)
    t0.start()
    if wait:
        return t0.wait()

def hold_pose(robot, duration, port, control="joint"):
    hold_context = {
        "skill": {
            "t_max": duration,
        },
        "control": {
            "control_mode": 1,
            "joint_imp":{
                "K_theta":[10000,10000,10000,10000,10000,10000,10000]
            }
            
        },
        #"user": {"F_ext_max": [100, 50]}
    }
    if control == "cart":
        hold_context["control"] = { "control_mode": 0,
                                    "cart_imp": {
                                        "K_x": [3000, 3000, 3000, 200, 200, 200]
                                        }
                                    }
    t = Task(robot, port)
    t.add_skill("hold","HoldPose",hold_context)
    t.start(queue=False)


def extract(robot, extractable, extractTo, container, port=12000):
    path_to_default_context = os.getcwd() + "/taxonomy/default_contexts/"
    f = open(path_to_default_context + "extraction.json")
    move_context = json.load(f)
    move_context["skill"]["objects"]["Container"] = container
    move_context["skill"]["objects"]["ExtractTo"] = extractTo
    move_context["skill"]["objects"]["Extractable"] = extractable
    move_context["skill"]["time_max"] = 10
    #move_context["user"]["env_X"] = [0, 0, 1, 1, 1, 1]
    t = Task(robot, port)
    t.add_skill("extraction","TaxExtraction",move_context)
    t.start(queue=False)
    return t.wait()

def insert(robot, insertable, approach, container, deltaX =[0,0,0,0,0,0], port=12000):
    path_to_default_context = os.getcwd() + "/taxonomy/default_contexts/"
    f = open(path_to_default_context + "insertion.json")
    move_context = json.load(f)
    move_context["skill"]["objects"]["Container"] = container
    move_context["skill"]["objects"]["Approach"] = approach
    move_context["skill"]["objects"]["Insertable"] = insertable
    move_context["skill"]["time_max"] = 7
    move_context["skill"]["p2"]["f_push"][2] = 25
    move_context["skill"]["p0"]["DeltaX"] = deltaX
    #move_context["user"]["env_X"] = [0, 0, 1, 1, 1, 1]
    t = Task(robot, port)
    t.add_skill("insertion","TaxInsertion",move_context)
    t.start(queue=False)
    return t.wait()

def insert2(robot, insertable, approach, container, deltaX =[0,0,0,0,0,0], port=12000):
    path_to_default_context = os.getcwd() + "/taxonomy/default_contexts/"
    f = open(path_to_default_context + "insertion2.json")
    move_context = json.load(f)
    move_context["skill"]["objects"]["Container"] = container
    move_context["skill"]["objects"]["Approach"] = approach
    move_context["skill"]["objects"]["Insertable"] = insertable
    move_context["skill"]["time_max"] = 6.5
    move_context["skill"]["p2"]["search_c"] = [0,0,20,0,0,0]
    move_context["skill"]["p2"]["search_a"] = [5,5,0,0,0,0]
    move_context["skill"]["p2"]["search_f"] = [0.75,1,0,0,0,0]
    move_context["skill"]["p2"]["delta_a"] = [.0,.0,0,0,0,0.1]
    move_context["skill"]["p2"]["delta_f"] = [0.75,0,0,0,0,0.5]
    move_context["skill"]["p2"]["t_d"] = 4
    move_context["skill"]["p2"]["K_X"] = [2000, 2000, 1000, 200, 200, 200],
    move_context["skill"]["p0"]["DeltaX"] = deltaX
    t = Task(robot, port)
    t.add_skill("insertion","Insertion2",move_context)
    t.start(queue=False)
    return t.wait()

def press_button(robot,tippable, approach):
    path_to_default_context = os.getcwd() + "/taxonomy/default_contexts/"
    f = open(path_to_default_context + "press_button.json")
    move_context = json.load(f)
    move_context["skill"]["objects"]["Button"] = tippable
    move_context["skill"]["objects"]["Approach"] = approach
    move_context["skill"]["condition_level_success"] = "Model"
    move_context["skill"]["condition_level_error"] = "Model"
    t = Task(robot)
    t.add_skill("press_button","TaxPressButton",move_context)
    t.start(queue=False)
    return t.wait()  


def handguiding(robot):
    context = {
        "skill": {
            "record_trajectory": False,
            #"recording_length": 1,
            #"recording_name": None,
            
        },
        "control": {
            "control_mode": 0
        }
    }
    t = Task(robot)
    t.add_skill("record_trajectory", "HandGuiding", context)
    t.start()
    input("stop now?")
    result = t.stop()
    print("Result: " + str(result))

def update_object(robot, name, content={}):
    obj = call_method(robot,12000,"download_object_context",{"object":name})
    obj = obj["result"]["context"]
    for key, o in content.items():
        if key in obj:
            obj[key] = content[key]
    obj["object"] = obj["name"]
    call_method(robot,12000,"set_object",obj)


   
def teach_insertion(robot:str, object_name:str):
    insertable = object_name
    
    print("\nteaching ",insertable, "for ", robot,"\n")

    handguiding(robot, "Insert the object into the robot\'s fingers. [Press any key to continue]")
    call_method(robot, 12000, "grasp", {"width":0,"speed":1,"force":100})
    call_method(robot, 12000, "teach_object", {"object": insertable, "width":True})
    current_finger_width = call_method(robot,12000,"get_state")["result"]["gripper_width"]
    call_method(robot,12000,"move_gripper",{"speed":1,"force":100,"width":current_finger_width+0.005})
    time.sleep(0.2)
    print(call_method(robot, 12000, "grasp_object", {"object": insertable}))
    handguiding(robot, "Teach approach pose slightly above the object\'s container. [Press any key to continue]")
    call_method(robot, 12000, "teach_object", {"object": insertable+"_container_approach"})
    handguiding(robot, "Teach container pose with the object fully inserted into the container. [Press any key to continue]")
    call_method(robot, 12000, "teach_object", {"object": insertable+"_container"})
    # print(call_method(robot, 12000, "grasp_object", {"object": insertable}))
    handguiding(robot, "Extract robot and object again. [Press any key to continue]")

def handguiding(robot:str, message:str="Press any key to stop"):
    context = {
        "skill": {
            "record_trajectory": False,
            #"recording_length": 1,
            #"recording_name": None,
            
        },
        "control": {
            "control_mode": 0
        }
    }
    t = Task(robot)
    t.add_skill("record_trajectory", "HandGuiding", context)
    t.start()
    input(message)
    result = t.stop()
    print("Result: " + str(result))


if __name__ == "__main__":
    robot = "miosr.nuc3"
    teach_insertion(robot, "samuelnew")