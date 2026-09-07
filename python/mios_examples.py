import argparse
import math
import os
import re
import shutil
import subprocess
import sys
from utils.ws_client import *
import json
import socket

import time


MAX_GRIPPER_WIDTH = 0.08
DEFAULT_GRIPPER_SPEED = 0.005
DEFAULT_GRASP_FORCE = 10.0
DEFAULT_GRASP_TOLERANCE = 0.002
GRIPPER_PORTAL_TIMEOUT_SECONDS = 30
DEFAULT_EFFORT_CONTROLLER = "mios_effort_controller"
DEFAULT_EFFORT_HOLD_CONTROLLER = "mios_effort_hold_controller"


class Ros2EffortControllerSession:
    """Own the commissioned MIOS torque-controller lifecycle for a teaching run.

    A Portal ``HandGuiding`` task is only compliant when the ROS 2 effort
    controller owns the Franka effort interfaces. The learning-service image
    intentionally has no ROS 2 tooling, so this session is available only
    when the teaching script runs inside the ROS motion container. It never
    publishes a low-level arm command: it only manages the pre-configured
    controller around the original MIOS task workflow.
    """

    def __init__(self, effort_controller=DEFAULT_EFFORT_CONTROLLER,
                 controller_manager="/controller_manager", timeout=15.0,
                 command_runner=subprocess.run):
        self.effort_controller = effort_controller
        self.controller_manager = controller_manager
        self.timeout = timeout
        self._command_runner = command_runner
        self._activated_by_session = False

    @staticmethod
    def _ros2_command_environment():
        return Ros2HandGuidingHandoff._ros2_command_environment()

    def _run(self, *arguments):
        command = ["ros2", "control", *arguments, "-c", self.controller_manager]
        completed = self._command_runner(
            command, capture_output=True, text=True, timeout=self.timeout, check=False,
            env=self._ros2_command_environment(),
        )
        output = (completed.stdout or "") + (completed.stderr or "")
        if completed.returncode != 0:
            raise RuntimeError(
                f"ROS 2 controller command failed ({' '.join(command)}): {output.strip()}"
            )
        return output

    def _states(self):
        states = {}
        for line in self._run("list_controllers").splitlines():
            fields = line.split()
            if len(fields) >= 3 and not fields[0].startswith("["):
                states[fields[0]] = fields[-1]
        return states

    def _parameter_bool(self, parameter):
        command = ["ros2", "param", "get", f"/{self.effort_controller}", parameter]
        completed = self._command_runner(
            command, capture_output=True, text=True, timeout=self.timeout, check=False,
            env=self._ros2_command_environment(),
        )
        output = (completed.stdout or "") + (completed.stderr or "")
        if completed.returncode != 0:
            raise RuntimeError(
                f"Cannot read /{self.effort_controller}/{parameter}: {output.strip()}"
            )
        value = output.strip().lower()
        if value.endswith("true"):
            return True
        if value.endswith("false"):
            return False
        raise RuntimeError(
            f"/{self.effort_controller}/{parameter} is not a Boolean: {output.strip()}"
        )

    def preflight(self):
        """Check controller gates without claiming FCI torque control."""
        ros_environment = self._ros2_command_environment()
        if shutil.which("ros2", path=ros_environment.get("PATH")) is None:
            raise RuntimeError(
                "Code-guided teaching must run in the ROS motion container, where the "
                "controller manager is available. Do not run --ros-handguiding from "
                "mios-ml-service."
            )
        state = self._states().get(self.effort_controller)
        if state is None:
            self._run("load_controller", self.effort_controller)
            state = self._states().get(self.effort_controller)
        # controller_manager loads a controller in the unconfigured state.
        # Configuration claims no command interface, so it is safe to perform
        # here; activation remains immediately before the Portal
        # HandGuiding task and still requires every commissioning gate below.
        if state == "unconfigured":
            self._run("set_controller_state", self.effort_controller, "inactive")
            state = self._states().get(self.effort_controller)
        if state != "inactive":
            raise RuntimeError(
                f"Code-guided teaching requires {self.effort_controller}=inactive before "
                f"the run, found {state or 'missing'}. Refusing to take over an active controller."
            )
        for parameter in (
            "allow_effort_activation",
            "allow_zero_effort_activation",
        ):
            if not self._parameter_bool(parameter):
                raise RuntimeError(
                    f"Code-guided teaching is locked: /{self.effort_controller}/"
                    f"{parameter} is false."
                )

    def ensure_active(self):
        """Activate the torque controller once, immediately before HandGuiding."""
        state = self._states().get(self.effort_controller)
        if state == "active":
            if not self._activated_by_session:
                raise RuntimeError(
                    f"{self.effort_controller} became active outside this teaching session. "
                    "Refusing to continue."
                )
            return
        if state != "inactive":
            raise RuntimeError(
                f"Cannot activate {self.effort_controller}: controller state is "
                f"{state or 'missing'}."
            )
        self._run("set_controller_state", self.effort_controller, "active")
        if self._states().get(self.effort_controller) != "active":
            raise RuntimeError(
                f"{self.effort_controller} did not become active for HandGuiding."
            )
        self._activated_by_session = True

    def close(self):
        """Return FCI to its non-torque state if this session activated it."""
        if not self._activated_by_session:
            return
        state = self._states().get(self.effort_controller)
        if state != "active":
            raise RuntimeError(
                f"Cannot release {self.effort_controller}: expected active, found "
                f"{state or 'missing'}."
            )
        self._run("set_controller_state", self.effort_controller, "inactive")
        if self._states().get(self.effort_controller) != "inactive":
            raise RuntimeError(
                f"{self.effort_controller} did not become inactive during cleanup."
            )
        self._activated_by_session = False


class Ros2HandGuidingHandoff:
    """Switch only between pre-configured torque controllers around HandGuiding.

    The class intentionally never loads a controller or changes a controller
    gate/limit.  Those decisions remain an explicit commissioning step outside
    the teaching script.  The hold controller is a separate instance of the
    bounded MIOS effort controller: it captures the released measured pose at
    activation and continues on the same Franka effort command interfaces.
    This avoids an unsafe torque-to-native-position mode jump after compliant
    HandGuiding.
    """

    def __init__(self, effort_controller=DEFAULT_EFFORT_CONTROLLER,
                 hold_controller=DEFAULT_EFFORT_HOLD_CONTROLLER,
                 controller_manager="/controller_manager", timeout=15.0,
                 settle_seconds=3.0,
                 command_runner=subprocess.run):
        self.effort_controller = effort_controller
        self.hold_controller = hold_controller
        self.controller_manager = controller_manager
        self.timeout = timeout
        if not math.isfinite(settle_seconds) or settle_seconds < 0.0:
            raise ValueError("settle_seconds must be finite and non-negative.")
        self.settle_seconds = settle_seconds
        self._command_runner = command_runner

    @staticmethod
    def _ros2_command_environment():
        """Keep the Portal-client import path out of child ROS 2 CLI calls.

        The teaching script is normally started with
        ``PYTHONPATH=/opt/mios/python`` so it can import ``utils.ws_client``.
        That replaces the Python paths exported by ROS setup scripts, and the
        Jazzy ``ros2`` console script can then no longer discover its
        ``ros2cli`` metadata.  The import has already happened in this parent
        process, so reconstruct the child CLI's ROS Python paths from the
        sourced AMENT prefixes instead.
        """
        environment = os.environ.copy()
        # ``docker exec`` starts a fresh process and therefore does not inherit
        # the setup files sourced by the container entrypoint.  Reconstruct
        # that environment here so the documented direct invocation of this
        # script works as well as an interactive shell inside the container.
        if shutil.which("ros2", path=environment.get("PATH")) is None:
            setup = subprocess.run(
                [
                    "bash", "-lc",
                    "source /opt/ros/jazzy/setup.bash && "
                    "source /ws/install/setup.bash && env -0",
                ],
                capture_output=True,
                check=False,
            )
            if setup.returncode == 0:
                for entry in setup.stdout.split(b"\0"):
                    if not entry or b"=" not in entry:
                        continue
                    key, value = entry.split(b"=", 1)
                    environment[key.decode()] = value.decode()
        python_version = f"python{sys.version_info.major}.{sys.version_info.minor}"
        ros_python_paths = []
        for prefix in environment.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
            candidate = os.path.join(prefix, "lib", python_version, "site-packages")
            if prefix and os.path.isdir(candidate):
                ros_python_paths.append(candidate)

        portal_python_path = os.path.realpath("/opt/mios/python")
        for path in environment.get("PYTHONPATH", "").split(os.pathsep):
            if path and os.path.realpath(path) != portal_python_path:
                ros_python_paths.append(path)

        if ros_python_paths:
            environment["PYTHONPATH"] = os.pathsep.join(dict.fromkeys(ros_python_paths))
        else:
            environment.pop("PYTHONPATH", None)
        return environment

    def _run(self, *arguments):
        command = ["ros2", "control", *arguments, "-c", self.controller_manager]
        completed = self._command_runner(
            command, capture_output=True, text=True, timeout=self.timeout, check=False,
            env=self._ros2_command_environment(),
        )
        output = (completed.stdout or "") + (completed.stderr or "")
        if completed.returncode != 0:
            raise RuntimeError(
                f"ROS 2 controller command failed ({' '.join(command)}): {output.strip()}"
            )
        return output

    def _parameter_bool(self, node, parameter):
        command = ["ros2", "param", "get", node, parameter]
        completed = self._command_runner(
            command, capture_output=True, text=True, timeout=self.timeout, check=False,
            env=self._ros2_command_environment(),
        )
        output = (completed.stdout or "") + (completed.stderr or "")
        if completed.returncode != 0:
            raise RuntimeError(
                f"Cannot read {node}/{parameter}: {output.strip()}"
            )
        value = output.strip().lower()
        if value.endswith("true"):
            return True
        if value.endswith("false"):
            return False
        raise RuntimeError(f"{node}/{parameter} is not a Boolean: {output.strip()}")

    def _parameter_double(self, node, parameter):
        command = ["ros2", "param", "get", node, parameter]
        completed = self._command_runner(
            command, capture_output=True, text=True, timeout=self.timeout, check=False,
            env=self._ros2_command_environment(),
        )
        output = (completed.stdout or "") + (completed.stderr or "")
        if completed.returncode != 0:
            raise RuntimeError(
                f"Cannot read {node}/{parameter}: {output.strip()}"
            )
        match = re.search(r"(?:Double|Integer) value is:\s*([^\s]+)", output)
        if match is None:
            raise RuntimeError(f"{node}/{parameter} is not numeric: {output.strip()}")
        try:
            value = float(match.group(1))
        except ValueError as error:
            raise RuntimeError(
                f"{node}/{parameter} is not numeric: {output.strip()}"
            ) from error
        if not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(f"{node}/{parameter} must be a positive finite value.")
        return value

    def _measured_joint_velocities(self):
        command = [
            "ros2", "topic", "echo", "--once", "--field",
            "measured_joint_state.velocity",
            "/franka_robot_state_broadcaster/robot_state",
        ]
        completed = self._command_runner(
            command, capture_output=True, text=True, timeout=self.timeout, check=False,
            env=self._ros2_command_environment(),
        )
        output = (completed.stdout or "") + (completed.stderr or "")
        if completed.returncode != 0:
            raise RuntimeError(
                f"Cannot read measured joint velocities: {output.strip()}"
            )
        match = re.search(r"\[([^\]]*)\]", output)
        if match is None:
            raise RuntimeError(f"Cannot parse measured joint velocities: {output.strip()}")
        try:
            velocities = [float(value.strip()) for value in match.group(1).split(",")]
        except ValueError as error:
            raise RuntimeError(
                f"Cannot parse measured joint velocities: {output.strip()}"
            ) from error
        if len(velocities) != 7 or not all(math.isfinite(value) for value in velocities):
            raise RuntimeError(f"Invalid measured joint velocities: {output.strip()}")
        return velocities

    def _require_stationary_effort_activation(self):
        """Fail before a strict switch would unconfigure a moving controller."""
        threshold = self._parameter_double(
            f"/{self.effort_controller}", "activation_velocity_threshold"
        )
        velocities = self._measured_joint_velocities()
        moving_joints = [
            f"{index + 1} ({velocity:.5f} rad/s)"
            for index, velocity in enumerate(velocities)
            if abs(velocity) > threshold
        ]
        if moving_joints:
            raise RuntimeError(
                "Cannot leave bounded effort hold: robot is not stationary for "
                f"{self.effort_controller} activation (threshold {threshold:.5f} rad/s; "
                f"joints {', '.join(moving_joints)}). Hold remains active."
            )

    def _states(self):
        states = {}
        for line in self._run("list_controllers").splitlines():
            fields = line.split()
            if len(fields) >= 3 and not fields[0].startswith("["):
                states[fields[0]] = fields[-1]
        return states

    def _require_states(self, expected_effort, expected_hold, operation):
        states = self._states()
        actual_effort = states.get(self.effort_controller, "missing")
        actual_hold = states.get(self.hold_controller, "missing")
        if actual_effort != expected_effort or actual_hold != expected_hold:
            raise RuntimeError(
                f"Cannot {operation}: expected {self.effort_controller}={expected_effort} and "
                f"{self.hold_controller}={expected_hold}, but found "
                f"{actual_effort} and {actual_hold}."
            )

    def _strict_switch(self, deactivate, activate):
        """Make one controller-manager switch transaction, not two transitions."""
        # ros2controlcli in Jazzy does not give --switch-timeout an argparse
        # numeric type.  Passing a value makes the CLI forward a string to
        # rclpy.duration.Duration(), where it is multiplied as text and the
        # switch fails before it reaches controller_manager.  Omit the option
        # to use its valid built-in 5-second service timeout; _run still has
        # this class's longer subprocess watchdog.
        self._run(
            "switch_controllers",
            "--deactivate", deactivate,
            "--activate", activate,
            "--strict",
        )

    def preflight(self):
        """Validate the manually commissioned controller pair without switching it."""
        states = self._states()
        controller_pair = (
            states.get(self.effort_controller, "missing"),
            states.get(self.hold_controller, "missing"),
        )
        if controller_pair not in {("active", "inactive"), ("inactive", "active")}:
            raise RuntimeError(
                "HandGuiding hold handoff requires exactly one prepared controller active: "
                f"{self.effort_controller}={controller_pair[0]}, "
                f"{self.hold_controller}={controller_pair[1]}."
            )
        # Returning from the bounded hold reactivates the ordinary
        # HandGuiding controller in its exact-zero external-torque baseline.
        # Check both of its lifecycle gates before a teaching sequence starts,
        # rather than discovering a relock after a pose has been taught.
        if not self._parameter_bool(f"/{self.effort_controller}",
                                    "allow_effort_activation"):
            raise RuntimeError(
                "HandGuiding effort mode is locked: allow_effort_activation is false."
            )
        if not self._parameter_bool(f"/{self.effort_controller}",
                                    "allow_zero_effort_activation"):
            raise RuntimeError(
                "HandGuiding effort mode is locked: "
                "allow_zero_effort_activation is false."
            )
        if not self._parameter_bool(f"/{self.hold_controller}",
                                    "allow_effort_activation"):
            raise RuntimeError("Effort hold is locked: allow_effort_activation is false.")
        if not self._parameter_bool(f"/{self.hold_controller}",
                                    "position_hold_enabled"):
            raise RuntimeError("Effort hold is locked: position_hold_enabled is false.")
        for parameter in (
            "allow_external_effort_commands",
            "allow_runtime_effort_commands",
            "allow_runtime_actuator_commands",
            "allow_runtime_cartesian_actuator_commands",
            "allow_runtime_cartesian_force_commands",
            "allow_runtime_nullspace_commands",
        ):
            if self._parameter_bool(f"/{self.hold_controller}", parameter):
                raise RuntimeError(
                    f"Effort-hold runtime command gate must stay disabled: {parameter} is true."
                )

    def enter_effort_mode(self):
        """Leave bounded effort hold before starting a new HandGuiding task."""
        states = self._states()
        effort_state = states.get(self.effort_controller)
        hold_state = states.get(self.hold_controller)
        if effort_state == "active" and hold_state == "inactive":
            return
        if effort_state != "inactive" or hold_state != "active":
            self._require_states("inactive", "active", "switch to HandGuiding effort mode")
        if self.settle_seconds:
            print("Waiting briefly for bounded effort hold to settle before HandGuiding.")
            time.sleep(self.settle_seconds)
        self._require_stationary_effort_activation()
        self._strict_switch(self.hold_controller, self.effort_controller)
        self._require_states("active", "inactive", "verify HandGuiding effort mode")

    def enter_effort_hold(self):
        """Capture a bounded torque hold while HandGuiding still owns MOVE."""
        self._require_states("active", "inactive", "switch to bounded effort hold")
        # The operator has released the arm, but the HandGuiding task is kept
        # alive for this short bounded pause.  Native Franka command-mode
        # switches are not involved here: both controllers claim the native
        # effort interfaces. The hold controller nevertheless requires a
        # fresh, stationary robot state before it captures the pose.
        if self.settle_seconds:
            print("Waiting briefly for the robot to settle before bounded effort hold.")
            time.sleep(self.settle_seconds)
        self._strict_switch(self.effort_controller, self.hold_controller)
        self._require_states("inactive", "active", "verify bounded effort hold")

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
        # Most examples use only the Portal client.  Keep MongoDB optional so
        # teaching and dry-run entry points work without pymongo installed.
        from desk.mongodb_client import MongoDBClient
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


   
def _portal_result_or_raise(response, operation: str):
    result = response.get("result", {}) if isinstance(response, dict) else {}
    if not result.get("result", False):
        error = result.get("error", "unknown Portal error")
        raise RuntimeError(f"{operation} failed: {error}")
    return response


def _bounded_gripper_width(width: float) -> float:
    if not math.isfinite(width):
        raise ValueError("Gripper width must be finite.")
    return min(MAX_GRIPPER_WIDTH, max(0.0, width))


def _grasp_with_contact_confirmation(robot: str, width: float, speed: float,
                                     force: float, tolerance: float,
                                     accept_contact_width: bool):
    """Grasp at the taught width, optionally confirming a non-zero contact width.

    Franka's grasp action reports failure when an object is securely contacted
    outside the requested epsilon window.  For teaching an object whose stored
    width may be stale, the opt-in fallback reads the measured opening after
    that completed close and reissues a verification grasp at that opening.
    It never accepts a nearly fully closed gripper as a contact grasp.
    """
    request = {
        "width": width,
        "speed": speed,
        "force": force,
        "epsilon_inner": tolerance,
        "epsilon_outer": tolerance,
    }
    response = call_method(robot, 12000, "grasp", request,
                           timeout=GRIPPER_PORTAL_TIMEOUT_SECONDS)
    result = response.get("result", {}) if isinstance(response, dict) else {}
    if result.get("result", False):
        return response
    if not accept_contact_width:
        return _portal_result_or_raise(response, "Grasp")

    state_response = call_method(robot, 12000, "get_state", {},
                                 timeout=GRIPPER_PORTAL_TIMEOUT_SECONDS)
    state = state_response.get("result", {}) if isinstance(state_response, dict) else {}
    contact_width = state.get("gripper_width")
    minimum_contact_width = 0.001
    if (not isinstance(contact_width, (int, float)) or
            not math.isfinite(contact_width) or
            not minimum_contact_width < contact_width <= MAX_GRIPPER_WIDTH):
        return _portal_result_or_raise(response, "Grasp")

    print(
        f"Gripper stopped at {contact_width:.5f} m outside the taught width "
        f"{width:.5f} m; verifying this non-zero contact width."
    )
    verification_tolerance = max(tolerance, 0.0005)
    return _portal_result_or_raise(
        call_method(robot, 12000, "grasp", {
            "width": contact_width,
            "speed": speed,
            "force": force,
            "epsilon_inner": verification_tolerance,
            "epsilon_outer": verification_tolerance,
        }, timeout=GRIPPER_PORTAL_TIMEOUT_SECONDS),
        "Contact-width grasp verification",
    )


def _detect_and_verify_grasp_width(robot: str, speed: float, force: float,
                                   tolerance: float):
    """Close until contact, then verify the measured non-zero opening.

    A zero-width grasp deliberately cannot succeed when an object is present:
    the resulting action failure is expected.  The gripper encoder reports
    the contact opening, which is then used for a second, ordinary grasp.  A
    nearly closed gripper is rejected rather than being mistaken for an object.
    """
    detection_response = call_method(robot, 12000, "grasp", {
        "width": 0.0,
        "speed": speed,
        "force": force,
        "epsilon_inner": tolerance,
        "epsilon_outer": tolerance,
    }, timeout=GRIPPER_PORTAL_TIMEOUT_SECONDS)

    state_response = call_method(robot, 12000, "get_state", {},
                                 timeout=GRIPPER_PORTAL_TIMEOUT_SECONDS)
    state = state_response.get("result", {}) if isinstance(state_response, dict) else {}
    contact_width = state.get("gripper_width")
    minimum_contact_width = 0.001
    if (not isinstance(contact_width, (int, float)) or
            not math.isfinite(contact_width) or
            not minimum_contact_width < contact_width <= MAX_GRIPPER_WIDTH):
        result = detection_response.get("result", {}) if isinstance(detection_response, dict) else {}
        error = result.get("error", "unknown Portal error")
        raise RuntimeError(
            "Could not detect an object between the gripper fingers "
            f"(measured opening: {contact_width!r}; grasp result: {error})."
        )

    print(f"Detected gripper contact width: {contact_width:.5f} m; verifying grasp.")
    verification_tolerance = max(tolerance, 0.0005)
    return _portal_result_or_raise(
        call_method(robot, 12000, "grasp", {
            "width": contact_width,
            "speed": speed,
            "force": force,
            "epsilon_inner": verification_tolerance,
            "epsilon_outer": verification_tolerance,
        }, timeout=GRIPPER_PORTAL_TIMEOUT_SECONDS),
        "Detected-width grasp verification",
    )


def teach_insertion(robot: str, object_name: str,
                    grasp_speed: float = DEFAULT_GRIPPER_SPEED,
                    grasp_force: float = DEFAULT_GRASP_FORCE,
                    grasp_width: float | None = None,
                    grasp_tolerance: float = DEFAULT_GRASP_TOLERANCE,
                    pregrasp_open_width: float = MAX_GRIPPER_WIDTH,
                    handguiding_handoff: Ros2HandGuidingHandoff | None = None,
                    effort_session: Ros2EffortControllerSession | None = None,
                    accept_contact_width: bool = False,
                    desk_guiding: bool = False,
                    detect_grasp_width: bool = False):
    insertable = object_name

    if not 0.0 < grasp_speed <= 0.15:
        raise ValueError("grasp_speed must be in (0.0, 0.15] m/s.")
    if not 0.0 <= grasp_force <= 70.0:
        raise ValueError("grasp_force must be in [0.0, 70.0] N.")
    if grasp_width is None and not detect_grasp_width:
        raise ValueError(
            "A measured --grasp-width or --detect-grasp-width is required for execution."
        )
    if grasp_width is not None and not 0.0 <= grasp_width <= MAX_GRIPPER_WIDTH:
        raise ValueError("grasp_width must be in [0.0, 0.08] m.")
    if not 0.0 <= grasp_tolerance <= 0.01:
        raise ValueError("grasp_tolerance must be in [0.0, 0.01] m.")
    if not 0.0 <= pregrasp_open_width <= MAX_GRIPPER_WIDTH:
        raise ValueError("pregrasp_open_width must be in [0.0, 0.08] m.")

    print("\nteaching ",insertable, "for ", robot,"\n")

    _portal_result_or_raise(
        call_method(robot, 12000, "move_gripper", {
            "width": pregrasp_open_width,
            "speed": grasp_speed,
        }, timeout=GRIPPER_PORTAL_TIMEOUT_SECONDS),
        "Open gripper for object placement",
    )
    handguiding(
        robot,
        "Insert the object into the open robot fingers. [Press any key to continue]",
        handguiding_handoff,
        effort_session,
        desk_guiding,
    )
    if detect_grasp_width:
        print("Closing the gripper until object contact is detected...", flush=True)
        _detect_and_verify_grasp_width(robot, grasp_speed, grasp_force, grasp_tolerance)
    else:
        print("Closing the gripper at the supplied grasp width...", flush=True)
        _grasp_with_contact_confirmation(
            robot, grasp_width, grasp_speed, grasp_force, grasp_tolerance,
            accept_contact_width,
        )
    # The deployed Portal handler reads the legacy ``width`` spelling.
    print("Saving the grasp pose...", flush=True)
    _portal_result_or_raise(
        call_method(robot, 12000, "teach_object", {"object": insertable, "width": True}),
        "Teach grasp pose",
    )
    # Keep the object clamped while teaching the remaining poses.  In this ROS
    # setup robot parameter application is intentionally disabled;
    # ``grasp_object`` would perform a second physical grasp and then fail when
    # it tried to apply the object's load/TCP parameters.
    print("Grasp pose taught; keeping the object clamped for approach and container teaching.")
    handguiding(robot, "Teach approach pose slightly above the object\'s container. [Press any key to continue]",
                handguiding_handoff, effort_session, desk_guiding)
    _portal_result_or_raise(
        call_method(robot, 12000, "teach_object", {"object": insertable+"_container_approach"}),
        "Teach container approach",
    )
    handguiding(robot, "Teach container pose with the object fully inserted into the container. [Press any key to continue]",
                handguiding_handoff, effort_session, desk_guiding)
    _portal_result_or_raise(
        call_method(robot, 12000, "teach_object", {"object": insertable+"_container"}),
        "Teach container pose",
    )
    handguiding(robot, "Extract robot and object again. [Press any key to continue]",
                handguiding_handoff, effort_session, desk_guiding)

def handguiding(robot: str, message: str = "Press any key to stop",
                handguiding_handoff: Ros2HandGuidingHandoff | None = None,
                effort_session: Ros2EffortControllerSession | None = None,
                desk_guiding: bool = False):
    if desk_guiding:
        if handguiding_handoff is not None or effort_session is not None:
            raise ValueError("Desk-guiding cannot use a ROS effort-controller handoff.")
        print("Desk-guiding: use the FR3 Pilot-Grip in Desk Programming mode, then confirm here.")
        try:
            input(message)
        except (KeyboardInterrupt, EOFError) as error:
            raise RuntimeError("Desk-guided teaching was interrupted before pose capture.") from error
        return None

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
    if effort_session is not None:
        effort_session.ensure_active()
    if handguiding_handoff is not None:
        handguiding_handoff.enter_effort_mode()
    t = Task(robot)
    t.add_skill("record_trajectory", "HandGuiding", context)
    t.start()
    try:
        input(message)
    except (KeyboardInterrupt, EOFError):
        print("\nHandGuiding input interrupted; stopping task.")
    finally:
        handoff_error = None
        if handguiding_handoff is not None:
            # Establish the bounded effort hold before stopping the Portal
            # task. The switch stays on the native effort interfaces and
            # must occur while HandGuiding still owns MOVE; t.stop() may
            # immediately return the robot to IDLE. If the guarded hold
            # transition is rejected, always stop the task before surfacing
            # that error to the caller.
            try:
                handguiding_handoff.enter_effort_hold()
            except Exception as error:
                handoff_error = error
        result = t.stop()
        print("Result: " + str(result))
        _portal_result_or_raise(result, "Stop HandGuiding")
        if handoff_error is not None:
            raise handoff_error
    return result


def main():
    parser = argparse.ArgumentParser(description="Teach an insertion object through MIOS Portal.")
    parser.add_argument("--robot", default="127.0.0.1",
                        help="MIOS Portal host (default: %(default)s)")
    parser.add_argument("--object", required=True, dest="object_name",
                        help="Object name to create or update in MIOS memory")
    parser.add_argument("--grasp-speed", type=float, default=DEFAULT_GRIPPER_SPEED,
                        help="Gripper speed in m/s (default: %(default)s)")
    parser.add_argument("--grasp-force", type=float, default=DEFAULT_GRASP_FORCE,
                        help="Grasp force in N (default: %(default)s)")
    parser.add_argument("--grasp-width", type=float,
                        help="Measured object width between the fingers in m")
    parser.add_argument("--detect-grasp-width", action="store_true",
                        help=("Close until object contact, measure the opening, then verify the grasp. "
                              "Rejects an empty or nearly closed gripper."))
    parser.add_argument("--grasp-tolerance", type=float, default=DEFAULT_GRASP_TOLERANCE,
                        help="Allowed grasp-width error in m (default: %(default)s)")
    parser.add_argument("--pregrasp-open-width", type=float, default=MAX_GRIPPER_WIDTH,
                        help="Opening before object placement in m (default: %(default)s)")
    parser.add_argument("--accept-contact-width", action="store_true",
                        help=("If grasping stops at a non-zero width outside the taught "
                              "width tolerance, verify the measured contact width instead"))
    parser.add_argument("--hold-between-handguiding", action="store_true",
                        help=("Switch to the prepared mios_effort_hold_controller after each "
                              "HandGuiding stop, then switch back before the next one"))
    parser.add_argument("--ros-handguiding", action="store_true",
                        help=("Run code-guided HandGuiding through the ROS effort controller. "
                              "This must be run in the ROS motion container, not mios-ml-service."))
    parser.add_argument("--direct-handguiding", action="store_true",
                        help=("Run code-guided HandGuiding through the direct-libfranka MIOS Core. "
                              "Use only when mios-direct is the sole FCI owner; run this Portal "
                              "client from mios-ml-service."))
    parser.add_argument("--desk-guiding", action="store_true",
                        help=("Use FR3 Pilot-Grip guidance in Desk Programming mode; do not "
                              "start or stop a MIOS HandGuiding task between taught poses"))
    parser.add_argument("--controller-manager", default="/controller_manager",
                        help="Controller-manager node used for opt-in hold handoffs (default: %(default)s)")
    parser.add_argument("--execute", action="store_true",
                        help="Run the physical HandGuiding/gripper/database workflow")
    args = parser.parse_args()

    if not args.execute:
        print("Dry run only: no task, gripper, Portal, or database operation will be started.")
        print(f"Would teach '{args.object_name}' through Portal host {args.robot}.")
        print("Re-run with --execute only after the HandGuiding workspace and gripper path are clear.")
        return

    if args.grasp_width is None and not args.detect_grasp_width:
        parser.error("--execute requires --grasp-width or --detect-grasp-width")
    if args.grasp_width is not None and args.detect_grasp_width:
        parser.error("--grasp-width and --detect-grasp-width cannot be combined")
    if args.desk_guiding and (args.hold_between_handguiding or args.ros_handguiding
                              or args.direct_handguiding):
        parser.error("--desk-guiding cannot be combined with code-guided HandGuiding")
    if args.hold_between_handguiding and args.ros_handguiding:
        parser.error("--hold-between-handguiding cannot be combined with --ros-handguiding")
    if args.direct_handguiding and (args.hold_between_handguiding or args.ros_handguiding):
        parser.error("--direct-handguiding cannot be combined with a ROS controller hand-guiding mode")
    if not args.desk_guiding and not (args.hold_between_handguiding or args.ros_handguiding
                                      or args.direct_handguiding):
        parser.error(
            "Code-guided teaching requires --ros-handguiding or --direct-handguiding. It prevents "
            "starting a HandGuiding task without an explicit torque-control owner."
        )

    handguiding_handoff = None
    effort_session = None
    if args.hold_between_handguiding:
        handguiding_handoff = Ros2HandGuidingHandoff(
            controller_manager=args.controller_manager,
        )
        handguiding_handoff.preflight()
    elif args.ros_handguiding:
        effort_session = Ros2EffortControllerSession(
            controller_manager=args.controller_manager,
        )
        effort_session.preflight()

    try:
        teach_insertion(args.robot, args.object_name, args.grasp_speed,
                        args.grasp_force, args.grasp_width,
                        args.grasp_tolerance,
                        args.pregrasp_open_width,
                        handguiding_handoff,
                        effort_session,
                        args.accept_contact_width,
                        args.desk_guiding,
                        args.detect_grasp_width)
    except Exception:
        if effort_session is not None:
            try:
                effort_session.close()
            except Exception as cleanup_error:
                print(f"Failed to release the ROS effort controller: {cleanup_error}", file=sys.stderr)
        raise
    else:
        if effort_session is not None:
            effort_session.close()


if __name__ == "__main__":
    main()
