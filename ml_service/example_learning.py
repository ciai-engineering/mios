import math
import os
import re
import shlex
import subprocess
import time

from problem_definition.problem_definition import ProblemDefinition
from services.base_service import ServiceConfiguration
from services.knowledge import Knowledge
from utils.experiment_wizard import start_experiment
from definitions.templates import InsertionFactory
from definitions.cost_functions import TimeMetric
from definitions.service_configs import SVMLearner, CMAESLearner
from utils.ws_client import call_method

from xmlrpc.client import ServerProxy


DEFAULT_EFFORT_CONTROLLER = "mios_effort_controller"
DEFAULT_EFFORT_HOLD_CONTROLLER = "mios_effort_hold_controller"
DEFAULT_JOINT_POSITION_CONTROLLER = "mios_joint_position_controller"
# The ROS-only runtime compose file assigns this stable name to the container
# that owns controller_manager and the sole FCI connection.  The learning
# service uses it only for controller lifecycle checks; all MIOS tasks still
# enter through the ROS-owned Core Portal on the configured robot host.
DEFAULT_ROS2_CONTAINER = "mios-ros2-control"
# A physical commissioning run must prove one candidate and its complete
# controller handoff before permitting a learning batch.  Increase only after
# a supervised single-trial run has finished and returned to native hold.
SUPERVISED_TRIAL_COUNT = 1
SUPERVISED_SETUP_JOINT_SPEED = 0.05
SUPERVISED_SETUP_JOINT_ACCELERATION = 0.1
# The insertion context has a hard angular-twist guard of 1.0 rad/s.  The
# generic p0 approach used 1.0 rad/s with 4.0 rad/s² acceleration, which
# reached 1.006 rad/s in the first physical baseline run.  Use a substantially
# lower profile while preserving that guard as the independent backstop.
SUPERVISED_INSERTION_APPROACH_DX = (0.025, 0.20)
SUPERVISED_INSERTION_APPROACH_DDX = (0.10, 0.40)

# Keep the supervised smoke-test candidate close to the taught insertion
# context.  These are commissioning bounds, deliberately much narrower than
# the general learning domain.  The first trial is seeded at the exact
# midpoint below; the reduced p1 speed applies while it seeks first contact.
SUPERVISED_INSERTION_LIMITS = {
    "p0_offset_x": (-0.001, 0.001),
    "p0_offset_y": (-0.001, 0.001),
    "p0_offset_phi": (-2.0, 2.0),
    "p0_offset_chi": (-2.0, 2.0),
    "p1_dx_d": (0.025, 0.030),
    "p1_dphi_d": (0.25, 0.50),
    "p1_ddx_d": (0.25, 0.50),
    "p1_ddphi_d": (0.50, 1.00),
    "p1_K_x": (800.0, 1200.0),
    "p1_K_phi": (80.0, 120.0),
    "p2_dx_d": (0.05, 0.10),
    "p2_dphi_d": (0.25, 0.50),
    "p2_ddx_d": (0.25, 0.50),
    "p2_ddphi_d": (0.50, 1.00),
    "p2_wiggle_a_x": (0.0, 3.0),
    "p2_wiggle_a_y": (0.0, 3.0),
    "p2_wiggle_a_phi": (0.0, 0.5),
    "p2_wiggle_a_chi": (0.0, 0.5),
    "p2_wiggle_f_x": (0.5, 1.5),
    "p2_wiggle_f_y": (0.5, 1.5),
    "p2_wiggle_f_phi": (0.0, 0.5),
    "p2_wiggle_f_chi": (0.0, 0.5),
    "p2_K_x": (800.0, 1200.0),
    "p2_K_y": (800.0, 1200.0),
    "p2_K_z": (800.0, 1200.0),
    "p2_K_phi": (80.0, 120.0),
    "p2_K_chi": (80.0, 120.0),
    "p2_K_psi": (80.0, 120.0),
    "p2_f_push_x": (-1.0, 1.0),
    "p2_f_push_y": (-1.0, 1.0),
    "p2_f_push_z": (5.0, 8.0),
}


class Ros2CoreTaskHandoff:
    """Enter bounded MIOS effort control and release it to FCI IDLE afterward.

    MIOS Core's torque pipeline publishes ``MiosEffortCommand`` messages to
    ``mios_effort_controller``. This helper deliberately does not configure
    controllers or change runtime-command gates: enabling that raw-effort
    path remains an explicit supervised commissioning decision. It switches
    only from a prepared ``MiosJointPositionController`` into the bounded
    effort controller.  At the end of a task it releases effort control to
    IDLE rather than attempting a non-real-time position-mode handoff.  The
    latter can leave FCI in MOVE while it waits for ``q``/``q_d`` alignment,
    which is neither necessary nor safe for a completed physical trial.
    """

    def __init__(self, effort_controller=DEFAULT_EFFORT_CONTROLLER,
                 hold_controller=DEFAULT_JOINT_POSITION_CONTROLLER,
                 controller_manager="/controller_manager", timeout=15.0,
                 settle_seconds=3.0, position_handoff_timeout=45.0,
                 ros2_container=None,
                 command_runner=subprocess.run):
        if not math.isfinite(settle_seconds) or settle_seconds < 0.0:
            raise ValueError("settle_seconds must be finite and non-negative.")
        if (not math.isfinite(position_handoff_timeout) or
                position_handoff_timeout <= 0.0):
            raise ValueError("position_handoff_timeout must be positive and finite.")
        self.effort_controller = effort_controller
        self.hold_controller = hold_controller
        self.controller_manager = controller_manager
        self.timeout = timeout
        self.settle_seconds = settle_seconds
        self.position_handoff_timeout = position_handoff_timeout
        self.ros2_container = (
            os.getenv("MIOS_ROS2_CONTAINER", DEFAULT_ROS2_CONTAINER)
            if ros2_container is None else ros2_container
        )
        self._command_runner = command_runner

    def _ros2_command(self, *arguments):
        command = ["ros2", *arguments]
        if not self.ros2_container:
            return command
        shell_command = " && ".join((
            "source /opt/ros/jazzy/setup.bash",
            "source /ws/install/setup.bash",
            "export ROS_DOMAIN_ID=0",
            "unset ROS_LOCALHOST_ONLY",
            shlex.join(command),
        ))
        return ["docker", "exec", self.ros2_container, "bash", "-lc", shell_command]

    def _run(self, *arguments):
        command = self._ros2_command(*arguments)
        completed = self._command_runner(
            command, capture_output=True, text=True, timeout=self.timeout, check=False,
        )
        output = (completed.stdout or "") + (completed.stderr or "")
        if completed.returncode != 0:
            raise RuntimeError(
                f"ROS 2 command failed ({shlex.join(command)}): {output.strip()}"
            )
        return output

    def _parameter_bool(self, node, parameter):
        output = self._run("param", "get", node, parameter).strip().lower()
        if output.endswith("true"):
            return True
        if output.endswith("false"):
            return False
        raise RuntimeError(f"{node}/{parameter} is not a Boolean: {output}")

    def _states(self):
        states = {}
        for line in self._run("control", "list_controllers", "-c", self.controller_manager).splitlines():
            fields = line.split()
            if len(fields) >= 3 and not fields[0].startswith("["):
                states[fields[0]] = fields[-1]
        return states

    def _robot_state_array(self, field):
        output = self._run(
            "topic", "echo", "--once", "--field", field,
            "/franka_robot_state_broadcaster/robot_state",
        )
        match = re.search(r"\[([^\]]*)\]", output)
        if match is None:
            raise RuntimeError(f"Cannot parse robot-state field {field}: {output.strip()}")
        values = [
            float(value) for value in re.findall(
                r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?", match.group(1)
            )
        ]
        if len(values) != 7 or not all(math.isfinite(value) for value in values):
            raise RuntimeError(f"Invalid robot-state field {field}: {output.strip()}")
        return values

    def _stationary(self):
        return self._robot_state_array("measured_joint_state.velocity")

    def _positive_parameter(self, node, parameter):
        output = self._run("param", "get", node, parameter)
        match = re.search(r"(?:Double|Integer) value is:\s*([^\s]+)", output)
        if match is None:
            raise RuntimeError(f"Cannot parse {node}/{parameter}: {output.strip()}")
        value = float(match.group(1))
        if not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(f"{node}/{parameter} must be positive and finite.")
        return value

    def _require_stationary(self):
        threshold = self._positive_parameter(
            f"/{self.effort_controller}", "activation_velocity_threshold"
        )
        moving = [
            f"joint {index + 1} ({velocity:.5f} rad/s)"
            for index, velocity in enumerate(self._stationary())
            if abs(velocity) > threshold
        ]
        if moving:
            raise RuntimeError(
                "Cannot switch MIOS task control while the robot is moving: " + ", ".join(moving)
            )

    def _position_handoff_ready(self):
        """Return whether native position activation can preserve Franka's q_d.

        ``MiosJointPositionController`` repeats these checks against its live
        loaned interfaces on activation. This non-real-time preflight avoids
        asking controller manager to switch modes while the ROS state already
        proves that the handoff would be discontinuous.
        """
        node = f"/{self.hold_controller}"
        velocity_threshold = self._positive_parameter(node, "activation_velocity_threshold")
        position_tolerance = self._positive_parameter(node, "activation_position_tolerance")
        acceleration_threshold = self._positive_parameter(
            node, "activation_acceleration_threshold"
        )
        measured_position = self._robot_state_array("measured_joint_state.position")
        desired_position = self._robot_state_array("desired_joint_state.position")
        measured_velocity = self._robot_state_array("measured_joint_state.velocity")
        desired_velocity = self._robot_state_array("desired_joint_state.velocity")
        desired_acceleration = self._robot_state_array("ddq_d")
        max_position_error = max(
            abs(measured - desired)
            for measured, desired in zip(measured_position, desired_position)
        )
        max_measured_velocity = max(abs(value) for value in measured_velocity)
        max_desired_velocity = max(abs(value) for value in desired_velocity)
        max_desired_acceleration = max(abs(value) for value in desired_acceleration)
        return (
            max_position_error <= position_tolerance and
            max_measured_velocity <= velocity_threshold and
            max_desired_velocity <= velocity_threshold and
            max_desired_acceleration <= acceleration_threshold,
            {
                "max_abs_q_minus_qd": max_position_error,
                "max_abs_measured_velocity": max_measured_velocity,
                "max_abs_desired_velocity": max_desired_velocity,
                "max_abs_desired_acceleration": max_desired_acceleration,
                "position_tolerance": position_tolerance,
                "velocity_threshold": velocity_threshold,
                "acceleration_threshold": acceleration_threshold,
            },
        )

    def _wait_for_position_handoff(self):
        deadline = time.monotonic() + self.position_handoff_timeout
        last_state = None
        while time.monotonic() < deadline:
            ready, state = self._position_handoff_ready()
            if ready:
                return
            last_state = state
            time.sleep(0.1)
        raise RuntimeError(
            "Cannot enter native post-task position hold before the Franka "
            f"reference is settled: {last_state}. The effort controller remains active."
        )

    def _strict_switch(self, deactivate, activate):
        # Do not pass --switch-timeout: Jazzy's CLI forwards it as text and
        # fails before the controller-manager service call.
        self._run(
            "control", "switch_controllers",
            "--deactivate", deactivate,
            "--activate", activate,
            "--strict",
            "-c", self.controller_manager,
        )

    def preflight(self):
        states = self._states()
        pair = (states.get(self.effort_controller, "missing"),
                states.get(self.hold_controller, "missing"))
        if pair not in {("inactive", "active"), ("active", "inactive")}:
            raise RuntimeError(
                "Core-task handoff requires exactly one prepared controller active: "
                f"{self.effort_controller}={pair[0]}, {self.hold_controller}={pair[1]}."
            )
        for parameter in (
            "allow_effort_activation",
            "allow_zero_effort_activation",
            "allow_runtime_effort_commands",
            "allow_runtime_actuator_commands",
            "allow_runtime_cartesian_actuator_commands",
        ):
            if not self._parameter_bool(f"/{self.effort_controller}", parameter):
                raise RuntimeError(
                    f"MIOS Core task mode is locked: {self.effort_controller}/"
                    f"{parameter} is false. Commission it explicitly before a physical task."
                )
        if not self._parameter_bool(f"/{self.hold_controller}", "allow_position_activation"):
            raise RuntimeError("Native post-task position hold is locked: allow_position_activation is false.")
        if self._parameter_bool(f"/{self.hold_controller}",
                                "allow_runtime_joint_position_commands"):
            raise RuntimeError(
                "Native post-task position hold must not accept runtime joint-position commands."
            )

    def enter_core_task_mode(self):
        states = self._states()
        if (states.get(self.effort_controller), states.get(self.hold_controller)) == ("active", "inactive"):
            return
        if (states.get(self.effort_controller), states.get(self.hold_controller)) != ("inactive", "active"):
            raise RuntimeError("Cannot enter Core task mode: controller states changed after preflight.")
        if self.settle_seconds:
            print("Waiting briefly for native position hold to settle before Core task mode.")
            time.sleep(self.settle_seconds)
        self._require_stationary()
        self._strict_switch(self.hold_controller, self.effort_controller)
        if (self._states().get(self.effort_controller), self._states().get(self.hold_controller)) != ("active", "inactive"):
            raise RuntimeError("Core task controller switch did not reach the expected active/inactive state.")

    def enter_post_task_hold(self):
        """Release MIOS effort control immediately after a completed task.

        The task engine's synchronous ``wait=True`` result means Core has
        already completed its success/failure recovery.  Do not keep the FCI
        callback alive while polling robot-state topics for a position hold;
        switch the command controller inactive and let the hardware return to
        IDLE.  A later task will explicitly prepare position hold again.
        """
        states = self._states()
        if (states.get(self.effort_controller), states.get(self.hold_controller)) == ("inactive", "active"):
            return
        if states.get(self.effort_controller) != "active":
            raise RuntimeError("Cannot enter post-task hold: controller states changed during the task.")
        try:
            self._run(
                "control", "set_controller_state", self.effort_controller, "inactive",
                "-c", self.controller_manager,
            )
        except Exception as release_error:
            raise RuntimeError(
                "Could not release the MIOS effort controller after the task. Stop the "
                "robot and inspect controller_manager immediately."
            ) from release_error
        if self._states().get(self.effort_controller) != "inactive":
            raise RuntimeError(
                "MIOS effort controller is still active after task cleanup. Stop the robot "
                "and inspect controller_manager immediately."
            )


class DirectCoreTaskHandoff:
    """Validate the direct-Core task route without starting any ROS controller.

    The direct-libfranka MIOS Core owns the complete FCI callback for a task,
    so there is no controller-manager transition before or after learning.
    The deployment fence is operational: no franka_bringup/ROS motion
    container may run while this handoff is selected.
    """

    def __init__(self, robot: str):
        self.robot = robot

    def preflight(self):
        response = call_method(self.robot, 12000, "get_state", {})
        result = response.get("result", {}) if isinstance(response, dict) else {}
        error = result.get("error", "") if isinstance(result, dict) else "invalid response"
        if error:
            raise RuntimeError(f"Direct MIOS Core is not ready: {error}")
        if not isinstance(result, dict):
            raise RuntimeError("Direct MIOS Core returned an invalid state response.")

    def enter_core_task_mode(self):
        # Core starts the bounded direct-libfranka callback when the task is
        # dispatched.  A non-real-time controller handoff here would create a
        # competing FCI owner, so this is intentionally a no-op.
        return

    def enter_post_task_hold(self):
        # Direct Core ends its callback and returns FCI to IDLE after the task.
        # Do not start a ROS position controller as a replacement owner.
        return


def constrain_supervised_insertion_domain(problem_definition):
    """Apply bounded, non-degenerate limits for the physical smoke test."""
    domain = problem_definition.domain
    for parameter, bounds in SUPERVISED_INSERTION_LIMITS.items():
        if parameter not in domain.limits:
            raise RuntimeError(f"Missing insertion-domain parameter: {parameter}")
        lower, upper = bounds
        original_lower, original_upper = domain.limits[parameter]
        if not (original_lower <= lower < upper <= original_upper):
            raise RuntimeError(
                f"Unsafe supervised bounds for {parameter}: {bounds} outside "
                f"the declared domain {domain.limits[parameter]}."
            )
        domain.limits[parameter] = bounds
        domain.x_0[parameter] = (lower + upper) / 2.0


def supervised_nominal_knowledge(problem_definition):
    """Seed the one supervised candidate at the taught insertion baseline.

    A blank :class:`Knowledge` object makes ``SVMLearner`` choose a random
    Latin-hypercube sample.  That is appropriate for later exploration, but
    not for the first physical commissioning run: it can add an orientation
    offset before the taught geometry itself has been verified.  Passing the
    domain's ordered nominal parameters makes the single first sample exactly
    the taught baseline.  The service's confidence value is immaterial with
    a batch width of one, but zero documents that no perturbation is intended.
    """
    domain = problem_definition.domain
    parameters = {
        parameter: domain.x_0[parameter]
        for parameter in domain.vector_mapping
    }
    if set(parameters) != set(domain.limits):
        raise RuntimeError("Nominal supervised parameters do not match the insertion domain.")
    return Knowledge(mode=None, parameters=parameters, confidence=0.0).to_dict()


def configure_supervised_insertion_dynamics(problem_definition):
    """Make the non-learned p0 approach compatible with its hard twist guard.

    ``TaxInsertion`` first moves to the taught approach pose using p0 before
    the learned contact motion (p1) begins.  The generic p0 angular profile
    was equal to the context's 1.0 rad/s safety limit, leaving no allowance
    for a measured-tracking overshoot.  This commissioning profile lowers
    both p0 speed and acceleration; it intentionally does *not* raise the
    configured safety limit.
    """
    try:
        insertion = problem_definition.default_context["skills"]["insertion"]
        p0 = insertion["skill"]["p0"]
        twist_limits = insertion["limits"]["cartesian_space"]["dX_max"]
    except (KeyError, TypeError) as error:
        raise RuntimeError(
            "Supervised insertion context is missing p0 dynamics or twist limits."
        ) from error

    if (not isinstance(twist_limits, list) or len(twist_limits) != 2 or
            twist_limits[0] <= SUPERVISED_INSERTION_APPROACH_DX[0] or
            twist_limits[1] <= SUPERVISED_INSERTION_APPROACH_DX[1]):
        raise RuntimeError(
            "Supervised p0 profile must remain strictly below the insertion twist limits."
        )
    p0["dX_d"] = list(SUPERVISED_INSERTION_APPROACH_DX)
    p0["ddX_d"] = list(SUPERVISED_INSERTION_APPROACH_DDX)


def configure_supervised_joint_moves(problem_definition):
    """Route learned-task joint moves through the commissioned torque path.

    ``MoveToPoseJoint`` defaults to MIOS control mode 3, which emits a joint
    velocity command.  The commissioned effort controller deliberately does
    not accept that command mode; it accepts the joint-torque pipeline's
    bounded effort output instead.  Apply the same conservative setting to
    setup, reset, rescue, and termination instructions so a trial cannot
    later revert to the unsupported velocity transport.
    """
    instruction_groups = (
        problem_definition.setup_instructions,
        problem_definition.reset_instructions,
        problem_definition.rescue_instructions,
        problem_definition.termination_instructions,
    )
    for instructions in instruction_groups:
        for instruction in instructions:
            context = instruction.get("parameters", {})
            parameters = context.get("parameters", {})
            names = parameters.get("skill_names", [])
            types = parameters.get("skill_types", [])
            skills = context.get("skills", {})
            for name, skill_type in zip(names, types):
                if skill_type != "MoveToPoseJoint":
                    continue
                move = skills.get(name)
                if not isinstance(move, dict):
                    raise RuntimeError(
                        f"Missing MoveToPoseJoint context for supervised skill {name!r}."
                    )
                move.setdefault("control", {})["control_mode"] = 1  # mJointTorque
                move.setdefault("skill", {})["speed"] = SUPERVISED_SETUP_JOINT_SPEED
                move["skill"]["acc"] = SUPERVISED_SETUP_JOINT_ACCELERATION



def learn_task(robot:str, problem_definition: ProblemDefinition, service_config: ServiceConfiguration, tags: list,
               n_iterations: int = 10, keep_record: bool = False, knowledge = None, wait: bool = False, service_port:int = 8000):
    start_experiment(robot, [robot], problem_definition, service_config, n_iterations, knowledge=knowledge, tags=tags,
                     keep_record=keep_record, wait=wait,service_port=service_port)


def set_active_grasped_object(robot: str, object_name: str):
    """Align Core's task context with an object that is already clamped."""
    response = call_method(robot, 12000, "set_grasped_object", {"object": object_name})
    result = response.get("result", {}) if isinstance(response, dict) else {}
    if result.get("result") is True:
        return

    # The deployed Core reports a failed duplicate request after it has
    # already set the requested object.  Verify the resulting state rather
    # than rejecting an idempotent retry.
    state = call_method(robot, 12000, "get_state", {})
    active_object = (
        state.get("result", {}).get("grasped_object")
        if isinstance(state, dict)
        else None
    )
    if active_object != object_name:
        raise RuntimeError(
            f"Could not set active grasped object to {object_name!r}: "
            f"{result.get('error', response)}"
        )
    
    
def example_learning(robot: str = "127.0.0.1", handoff=None, direct_fci=False):
    # tasks = {hosts: insertables}
    tasks = {robot: "samuelnew"}
    if handoff is None:
        direct_fci = direct_fci or os.getenv("MIOS_CONTROL_BACKEND", "").lower() == "direct"
        handoff = DirectCoreTaskHandoff(robot) if direct_fci else Ros2CoreTaskHandoff()
    for host, insertable in tasks.items():
        container = insertable + "_container"
        approach = container + "_approach"
        
        # configuring the learning problem (problem definition):
        # for every skill there is a definition class (eg InsertionFactory) that creates the problem_definition
        # input: list of agents (usually one robot -> IP of mios)
        #        cost function: see from definitions.cost_functions, eg.: TimeMetric (skill_class, max_time, heuristic=np.exp(var)")
        #        objects: for insertion: insertable, pose when insertable is inserted, pose when insertable is above the container
        pd = InsertionFactory([host], TimeMetric("insertion", {"time": 15}),
                            {"Insertable": insertable, "Container": container,
                            "Approach": approach}).get_problem_definition(insertable)
        constrain_supervised_insertion_domain(pd)
        configure_supervised_insertion_dynamics(pd)
        configure_supervised_joint_moves(pd)
        pd.variate_only_success = True  # repeat trial only when successful
        # A supervised commissioning run must dispatch exactly one physical
        # insertion attempt.  ``n_variations`` repeats a candidate even when
        # the learner itself is limited to one trial, so it must remain one
        # here as well.
        pd.n_variations = 1
        pd.host = host  # host (only for ducumentation)
        pd.optimum_thr = 0.3  # trial is taged as optimal when cost is under this threshold 
        pd.cost_function.finish_thr = 2  # learning is finished when this threshold is reached with optimal trials; if exploration mode of the learning service is True, learning will still contiue

        # Leaning Service configuration:
        # For Example: SVMLearner (https://proceedings.mlr.press/v155/voigt21a/voigt21a.pdf)
        # inputs: max trials
        #         batch size
        #         number of immigrants (old way of sharing knowledge; not used right now, keep it to 0)
        #         exploration mode: whether the learner should contiue optimizing after finding a solution
        #         batch synchronization: only used in a multi robot setup when all robots should start a batch at the same time; not need -> keep it to False
        #         request probability: new way of sharing knowledge - defines the probability for the ml_service to request knowledge from other agents instead of creating a new trial itself.
        #                              0.4 is a good probability in multi robot systems
        #         request_probability_decrease: whether the request probability should be automaticcly adapt to success rate (True) or be keept steady (False)
        # Run a small supervised learning sequence one physical candidate at
        # a time. The batch width remains one, so the service cannot launch a
        # group of arm motions concurrently.
        sc = SVMLearner(SUPERVISED_TRIAL_COUNT, 1, 0, True, False, -1, True).get_configuration()
        
        # Knowledge source definition:
        # all information regarding where to find knowledge and kind of knowledge should be used
        # mode = mode  # either None, "specific", "local", "global"     (if "None", but parameters is not empty, the parameters will be used as centroid)
        # type = type  # also possible: "predicted" (use prediction), "all" (gives list of knowledges, no predicted ones),
        # scope = scope  # scope (tags of results to make this knowledge)
        # kb_location = kb_location  # location of the knowledge base
        # kb_db = kb_db  # needed if type is specific
        # kb_task_type = kb_task_type  # needed if type is specific
        # parameters = parameters #dict() with unnormalised Theta
        # confidence = confidence
        # uuid = uuid   # single uuid or list of uuids
        # prediction = prediction  # bool, wether this knowledge was predicted or not
        # prediction_error = prediction_error
        # identity = identity  # task identity
        # skill_class = skill_class  # eg. "insertion"
        # skill_instance = skill_instance  #  skill_instance from problem_definition
        # source = source  # uuid(s) of the source ml_results
        # expected_cost = expected_cost
        # time = time  # time when knowledge point was created (time.time())
        # datetime = datetime  # time.ctime()
        # tags = tags  #actual tags of the knowledge itself
        # equal_start = equal_start  # if True the svm.py will use the same first batch (from equal_tags) every time
        # equal_tags = equal_tags
        # cost_function = cost_function
        # identification_name = identification_name  # identification string, because uuid is random
        # time_range = time_range  # time window from which knowledge can be collected to create new knowledge points
        # similarity = similarity  # list of objects with request probabilities
        # The first physical trial validates the pose that was just taught.
        # Seed SVM at the nominal domain point rather than drawing a random
        # orientation/contact-approach perturbation.
        knowledge = supervised_nominal_knowledge(pd)

        # this is a list of tags to find the entries on the mongoDB; 
        # the experiment wizard will append also some information here
        tags = ["example_learning", "tutorial"]  
        
        # helper function (experiment wizard):
        # mios IP
        # problem definition
        # service configuration
        # tags
        # knowledge source dict
        # number of iterations: how often should this experiment be repeated
        # service port: 8000
        # whether the function should return immediately or wait until learning is finished 
        handoff.preflight()
        handoff.enter_core_task_mode()
        try:
            set_active_grasped_object(host, insertable)
            learn_task(host, pd, sc, tags, knowledge=knowledge, n_iterations=1,
                       service_port=8000, wait=True)
        finally:
            handoff.enter_post_task_hold()
        
def stop_services(robots:list = ["localhost"]):
    for r in robots:
        s = ServerProxy("http://" + r + ":8000", allow_none=True)
        try:
            s.stop_service()
        except Exception as e:
            print("Error with robot ",r)
            print(e)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Run the supervised MIOS insertion-learning example.")
    parser.add_argument("--robot", default=os.getenv("MIOS_ROBOT", "127.0.0.1"),
                        help="MIOS Portal host (default: %(default)s)")
    parser.add_argument("--direct-fci", action="store_true",
                        help=("Use the direct-libfranka MIOS Core as the sole FCI owner. "
                              "Do not run a ROS motion container at the same time."))
    args = parser.parse_args()
    example_learning(args.robot, direct_fci=args.direct_fci)
