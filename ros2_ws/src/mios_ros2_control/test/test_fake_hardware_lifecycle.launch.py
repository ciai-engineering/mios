"""Controller-manager lifecycle test using only ros2_control GenericSystem.

The test deliberately keeps both production hardware gates false. Loading and
configuring prove the plugin can be used by a controller manager with seven
real command/state interface names; the rejected activations prove the image
keeps both the generic effort lock and the exact-zero-effort lock in place.
"""

from pathlib import Path
import threading
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ConfigureController
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import LoadController
from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import UnloadController
from franka_msgs.msg import FrankaRobotState
import launch
from launch_ros.actions import Node as LaunchNode
import launch_testing
import pytest
import rclpy
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


CONTROLLER_MANAGER = "/controller_manager"
CONTROLLER_NAME = "mios_effort_controller"
HOLD_CONTROLLER_NAME = "mios_effort_hold_controller"
MODEL_BROADCASTER_NAME = "mios_robot_model_broadcaster"
SERVICE_TIMEOUT_SECONDS = 20.0


@pytest.mark.launch_test
def generate_test_description():
    package_share = Path(get_package_share_directory("mios_ros2_control"))
    robot_description = (package_share / "test" / "mios_mock_hardware.urdf").read_text()
    controller_parameters = package_share / "config" / "mios_controllers.yaml"

    # Jazzy controller_manager receives its robot description from this
    # transient-local topic rather than reading the deprecated direct
    # parameter. No real robot or Franka hardware node is launched.
    robot_state_publisher = LaunchNode(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )
    controller_manager = LaunchNode(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[str(controller_parameters)],
    )
    return (
        launch.LaunchDescription(
            [robot_state_publisher, controller_manager, launch_testing.actions.ReadyToTest()]
        ),
        {"controller_manager": controller_manager},
    )


class TestMiosFakeHardwareLifecycle(unittest.TestCase):
    """Exercise load, configure, failed safe activation, and post-state."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("mios_fake_hardware_lifecycle_test")
        cls.robot_state_publisher = cls.node.create_publisher(
            FrankaRobotState,
            "/franka_robot_state_broadcaster/robot_state",
            # Match the controller's best-effort state subscription. A
            # reliable publisher can be compatible, but matching avoids
            # DDS-dependent discovery behaviour in this offline test.
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        cls.stationary_idle_message = FrankaRobotState()
        cls.stationary_idle_message.robot_mode = FrankaRobotState.ROBOT_MODE_IDLE
        cls.stationary_idle_message.measured_joint_state.position = [0.0] * 7
        cls.stationary_idle_message.measured_joint_state.velocity = [0.0] * 7
        cls.stationary_idle_message.measured_joint_state.effort = [0.0] * 7

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_publisher(cls.robot_state_publisher)
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call(self, service_type, service_name, request):
        client = self.node.create_client(service_type, service_name)
        self.assertTrue(
            client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_SECONDS),
            f"Service did not become available: {service_name}",
        )
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=SERVICE_TIMEOUT_SECONDS
        )
        self.assertTrue(future.done(), f"Timed out calling {service_name}")
        self.assertIsNotNone(future.result(), f"No response from {service_name}")
        return future.result()

    def _controllers(self):
        return self._call(
            ListControllers,
            f"{CONTROLLER_MANAGER}/list_controllers",
            ListControllers.Request(),
        ).controller

    def _start_stationary_idle_state_stream(self):
        """Publish while controller_manager waits for the mode switch.

        The real state broadcaster runs independently at 1 kHz. In this test
        the service client can block for the controller-manager switch timeout,
        so a one-shot state emitted before the request is not sufficient for
        the production 100 ms freshness requirement.
        """
        stop = threading.Event()

        def publish() -> None:
            while not stop.is_set():
                self.robot_state_publisher.publish(self.stationary_idle_message)
                stop.wait(0.01)

        thread = threading.Thread(target=publish, daemon=True)
        thread.start()
        return stop, thread

    @staticmethod
    def _stop_stationary_idle_state_stream(stop, thread):
        stop.set()
        thread.join(timeout=1.0)

    def _wait_for_robot_state_subscription(self):
        deadline = time.monotonic() + SERVICE_TIMEOUT_SECONDS
        while (
            self.robot_state_publisher.get_subscription_count() == 0
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self.node, timeout_sec=0.01)
            time.sleep(0.01)
        self.assertGreater(
            self.robot_state_publisher.get_subscription_count(),
            0,
            "MIOS controller did not create its robot-state subscription",
        )

    def test_plugin_configures_and_commissioning_lock_rejects_activation(self):
        # Construct the read-only model broadcaster before touching any effort
        # controller. This catches a missing pluginlib export without requiring
        # real Franka semantic state interfaces in GenericSystem.
        model_load_result = self._call(
            LoadController,
            f"{CONTROLLER_MANAGER}/load_controller",
            LoadController.Request(name=MODEL_BROADCASTER_NAME),
        )
        self.assertTrue(model_load_result.ok, "MIOS model broadcaster plugin did not load")
        model_unload_result = self._call(
            UnloadController,
            f"{CONTROLLER_MANAGER}/unload_controller",
            UnloadController.Request(name=MODEL_BROADCASTER_NAME),
        )
        self.assertTrue(model_unload_result.ok, "MIOS model broadcaster plugin did not unload")

        load_result = self._call(
            LoadController,
            f"{CONTROLLER_MANAGER}/load_controller",
            LoadController.Request(name=CONTROLLER_NAME),
        )
        self.assertTrue(load_result.ok, "MIOS controller plugin did not load")
        # GenericSystem has no Franka state box, including for this first
        # rejected activation. Select the test-only topic source before the
        # controller declares its required state interfaces.
        set_parameter_result = self._call(
            SetParameters,
            f"/{CONTROLLER_NAME}/set_parameters",
            SetParameters.Request(
                parameters=[
                    Parameter(
                        "robot_state_source", Parameter.Type.STRING, "topic"
                    ).to_parameter_msg(),
                    Parameter(
                        "allow_runtime_actuator_commands", Parameter.Type.BOOL, False
                    ).to_parameter_msg(),
                    Parameter(
                        "allow_runtime_cartesian_actuator_commands", Parameter.Type.BOOL, False
                    ).to_parameter_msg(),
                ]
            ),
        )
        self.assertTrue(set_parameter_result.results[0].successful)
        configure_result = self._call(
            ConfigureController,
            f"{CONTROLLER_MANAGER}/configure_controller",
            ConfigureController.Request(name=CONTROLLER_NAME),
        )
        self.assertTrue(configure_result.ok, "MIOS controller did not configure")

        controllers = {controller.name: controller for controller in self._controllers()}
        self.assertIn(CONTROLLER_NAME, controllers)
        self.assertEqual(controllers[CONTROLLER_NAME].state, "inactive")
        self.assertEqual(
            controllers[CONTROLLER_NAME].required_command_interfaces,
            [f"fr3_joint{index}/effort" for index in range(1, 8)]
        )

        activation_result = self._call(
            SwitchController,
            f"{CONTROLLER_MANAGER}/switch_controller",
            SwitchController.Request(
                activate_controllers=[CONTROLLER_NAME],
                strictness=SwitchController.Request.STRICT,
            ),
        )
        self.assertFalse(
            activation_result.ok,
            "The production commissioning lock unexpectedly allowed fake-hardware activation",
        )

        # In Jazzy a controller whose activation callback returns ERROR is
        # returned to ``unconfigured`` while command interfaces are released.
        # This is the required fail-safe result of the commissioning lock.
        controllers = {controller.name: controller for controller in self._controllers()}
        self.assertEqual(controllers[CONTROLLER_NAME].state, "unconfigured")

        # Reload after the failed transition. This mirrors a production
        # operator's cleanup path and gives the new mock controller a fresh
        # subscription endpoint before its successful lifecycle test.
        unload_result = self._call(
            UnloadController,
            f"{CONTROLLER_MANAGER}/unload_controller",
            UnloadController.Request(name=CONTROLLER_NAME),
        )
        self.assertTrue(unload_result.ok, "MIOS controller did not unload after safe rejection")
        load_result = self._call(
            LoadController,
            f"{CONTROLLER_MANAGER}/load_controller",
            LoadController.Request(name=CONTROLLER_NAME),
        )
        self.assertTrue(load_result.ok, "MIOS controller plugin did not reload")

        # These overrides are local to the mock controller node and exist only
        # to verify the succeeding lifecycle path. GenericSystem cannot export
        # Franka's in-process RobotState box, so the test explicitly uses the
        # retained ROS-topic source; production uses the hardware source.
        set_parameter_result = self._call(
            SetParameters,
            f"/{CONTROLLER_NAME}/set_parameters",
            SetParameters.Request(
                parameters=[
                    Parameter(
                        "robot_state_source", Parameter.Type.STRING, "topic"
                    ).to_parameter_msg(),
                    Parameter(
                        "allow_effort_activation", Parameter.Type.BOOL, True
                    ).to_parameter_msg(),
                    Parameter(
                        "allow_zero_effort_activation", Parameter.Type.BOOL, True
                    ).to_parameter_msg(),
                    Parameter(
                        "allow_runtime_actuator_commands", Parameter.Type.BOOL, False
                    ).to_parameter_msg(),
                    Parameter(
                        "allow_runtime_cartesian_actuator_commands", Parameter.Type.BOOL, False
                    ).to_parameter_msg(),
                ]
            ),
        )
        self.assertTrue(set_parameter_result.results[0].successful)

        configure_result = self._call(
            ConfigureController,
            f"{CONTROLLER_MANAGER}/configure_controller",
            ConfigureController.Request(name=CONTROLLER_NAME),
        )
        self.assertTrue(configure_result.ok, "MIOS controller did not configure after reload")
        self._wait_for_robot_state_subscription()
        stream_stop, stream_thread = self._start_stationary_idle_state_stream()
        try:
            activation_result = self._call(
                SwitchController,
                f"{CONTROLLER_MANAGER}/switch_controller",
                SwitchController.Request(
                    activate_controllers=[CONTROLLER_NAME],
                    strictness=SwitchController.Request.STRICT,
                ),
            )
        finally:
            self._stop_stationary_idle_state_stream(stream_stop, stream_thread)
        self.assertTrue(activation_result.ok, "Mock-safe zero-effort activation failed")
        controllers = {controller.name: controller for controller in self._controllers()}
        self.assertEqual(controllers[CONTROLLER_NAME].state, "active")
        self.assertEqual(
            controllers[CONTROLLER_NAME].claimed_interfaces,
            [f"fr3_joint{index}/effort" for index in range(1, 8)]
        )

        # The post-HandGuiding hold uses a second, separately locked instance
        # of the same effort controller. Switching between these two
        # controllers retains the effort command interface rather than
        # starting a native joint-position motion generator at an arbitrary
        # compliant pose.
        load_result = self._call(
            LoadController,
            f"{CONTROLLER_MANAGER}/load_controller",
            LoadController.Request(name=HOLD_CONTROLLER_NAME),
        )
        self.assertTrue(load_result.ok, "MIOS effort-hold controller did not load")
        set_parameter_result = self._call(
            SetParameters,
            f"/{HOLD_CONTROLLER_NAME}/set_parameters",
            SetParameters.Request(
                parameters=[
                    Parameter(
                        "robot_state_source", Parameter.Type.STRING, "topic"
                    ).to_parameter_msg(),
                    Parameter(
                        "allow_runtime_actuator_commands", Parameter.Type.BOOL, False
                    ).to_parameter_msg(),
                    Parameter(
                        "allow_runtime_cartesian_actuator_commands", Parameter.Type.BOOL, False
                    ).to_parameter_msg(),
                ]
            ),
        )
        self.assertTrue(set_parameter_result.results[0].successful)
        configure_result = self._call(
            ConfigureController,
            f"{CONTROLLER_MANAGER}/configure_controller",
            ConfigureController.Request(name=HOLD_CONTROLLER_NAME),
        )
        self.assertTrue(configure_result.ok, "MIOS effort-hold controller did not configure")
        set_parameter_result = self._call(
            SetParameters,
            f"/{HOLD_CONTROLLER_NAME}/set_parameters",
            SetParameters.Request(
                parameters=[
                    Parameter(
                        "allow_effort_activation", Parameter.Type.BOOL, True
                    ).to_parameter_msg(),
                ]
            ),
        )
        self.assertTrue(set_parameter_result.results[0].successful)

        self._wait_for_robot_state_subscription()
        stream_stop, stream_thread = self._start_stationary_idle_state_stream()
        try:
            activation_result = self._call(
                SwitchController,
                f"{CONTROLLER_MANAGER}/switch_controller",
                SwitchController.Request(
                    deactivate_controllers=[CONTROLLER_NAME],
                    activate_controllers=[HOLD_CONTROLLER_NAME],
                    strictness=SwitchController.Request.STRICT,
                ),
            )
        finally:
            self._stop_stationary_idle_state_stream(stream_stop, stream_thread)
        self.assertTrue(activation_result.ok, "Mock effort-to-effort hold switch failed")
        controllers = {controller.name: controller for controller in self._controllers()}
        self.assertEqual(controllers[CONTROLLER_NAME].state, "inactive")
        self.assertEqual(controllers[HOLD_CONTROLLER_NAME].state, "active")
        self.assertEqual(
            controllers[HOLD_CONTROLLER_NAME].claimed_interfaces,
            [f"fr3_joint{index}/effort" for index in range(1, 8)]
        )

        self._wait_for_robot_state_subscription()
        stream_stop, stream_thread = self._start_stationary_idle_state_stream()
        try:
            activation_result = self._call(
                SwitchController,
                f"{CONTROLLER_MANAGER}/switch_controller",
                SwitchController.Request(
                    deactivate_controllers=[HOLD_CONTROLLER_NAME],
                    activate_controllers=[CONTROLLER_NAME],
                    strictness=SwitchController.Request.STRICT,
                ),
            )
        finally:
            self._stop_stationary_idle_state_stream(stream_stop, stream_thread)
        self.assertTrue(activation_result.ok, "Mock effort-hold-to-effort switch failed")
        controllers = {controller.name: controller for controller in self._controllers()}
        self.assertEqual(controllers[CONTROLLER_NAME].state, "active")
        self.assertEqual(controllers[HOLD_CONTROLLER_NAME].state, "inactive")

        deactivation_result = self._call(
            SwitchController,
            f"{CONTROLLER_MANAGER}/switch_controller",
            SwitchController.Request(
                deactivate_controllers=[CONTROLLER_NAME],
                strictness=SwitchController.Request.STRICT,
            ),
        )
        self.assertTrue(deactivation_result.ok, "Mock-safe effort deactivation failed")
        controllers = {controller.name: controller for controller in self._controllers()}
        self.assertEqual(controllers[CONTROLLER_NAME].state, "inactive")

        # Relocking an already configured inactive controller must take effect
        # before the next lifecycle transition. This catches stale cached
        # zero-effort-gate values without involving hardware.
        set_parameter_result = self._call(
            SetParameters,
            f"/{CONTROLLER_NAME}/set_parameters",
            SetParameters.Request(
                parameters=[
                    Parameter(
                        "allow_zero_effort_activation", Parameter.Type.BOOL, False
                    ).to_parameter_msg()
                ]
            ),
        )
        self.assertTrue(set_parameter_result.results[0].successful)
        self._wait_for_robot_state_subscription()
        stream_stop, stream_thread = self._start_stationary_idle_state_stream()
        try:
            activation_result = self._call(
                SwitchController,
                f"{CONTROLLER_MANAGER}/switch_controller",
                SwitchController.Request(
                    activate_controllers=[CONTROLLER_NAME],
                    strictness=SwitchController.Request.STRICT,
                ),
            )
        finally:
            self._stop_stationary_idle_state_stream(stream_stop, stream_thread)
        self.assertFalse(
            activation_result.ok,
            "Runtime zero-effort relock unexpectedly allowed activation",
        )
        controllers = {controller.name: controller for controller in self._controllers()}
        self.assertEqual(controllers[CONTROLLER_NAME].state, "unconfigured")

    def test_joint_velocity_plugin_configures_and_its_lock_rejects_activation(self):
        """Load the legacy MoveToJointPose ROS boundary without hardware."""
        controller_name = "mios_joint_velocity_controller"
        load_result = self._call(
            LoadController,
            f"{CONTROLLER_MANAGER}/load_controller",
            LoadController.Request(name=controller_name),
        )
        self.assertTrue(load_result.ok, "MIOS joint-velocity plugin did not load")
        configure_result = self._call(
            ConfigureController,
            f"{CONTROLLER_MANAGER}/configure_controller",
            ConfigureController.Request(name=controller_name),
        )
        self.assertTrue(configure_result.ok, "MIOS joint-velocity controller did not configure")
        controllers = {controller.name: controller for controller in self._controllers()}
        self.assertEqual(controllers[controller_name].state, "inactive")
        self.assertEqual(
            controllers[controller_name].required_command_interfaces,
            [f"fr3_joint{index}/velocity" for index in range(1, 8)],
        )

        activation_result = self._call(
            SwitchController,
            f"{CONTROLLER_MANAGER}/switch_controller",
            SwitchController.Request(
                activate_controllers=[controller_name],
                strictness=SwitchController.Request.STRICT,
            ),
        )
        self.assertFalse(
            activation_result.ok,
            "The shipped joint-velocity commissioning lock unexpectedly activated",
        )
        controllers = {controller.name: controller for controller in self._controllers()}
        self.assertEqual(controllers[controller_name].state, "unconfigured")
