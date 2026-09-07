"""Offline controller-cleanup tests for the ROS supervised-learning route."""

import subprocess

from example_learning import Ros2CoreTaskHandoff


def test_post_task_cleanup_releases_effort_without_position_handoff():
    commands = []
    controller_states = [
        "mios_effort_controller MiosEffortController active\n"
        "mios_joint_position_controller MiosJointPositionController inactive\n",
        "mios_effort_controller MiosEffortController inactive\n"
        "mios_joint_position_controller MiosJointPositionController inactive\n",
    ]

    def runner(command, **_kwargs):
        commands.append(command)
        joined = " ".join(command)
        if "list_controllers" in joined:
            return subprocess.CompletedProcess(command, 0, controller_states.pop(0), "")
        if "set_controller_state" in joined:
            assert "mios_effort_controller inactive" in joined
            return subprocess.CompletedProcess(command, 0, "", "")
        raise AssertionError(f"Unexpected ROS command: {joined}")

    handoff = Ros2CoreTaskHandoff(
        ros2_container=None, command_runner=runner, settle_seconds=0.0,
    )
    handoff.enter_post_task_hold()

    assert any("set_controller_state mios_effort_controller inactive" in " ".join(command)
               for command in commands)
    assert not any("switch_controllers" in " ".join(command) for command in commands)
