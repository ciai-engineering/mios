#!/usr/bin/env bash
set -euo pipefail

# This process owns no command interface.  It makes Franka's read-only model
# data available to the ROS-owned MIOS Core before that Core opens its Portal.
# Keep controller activation separate from every arm-command controller.
controller_manager="${MIOS_CONTROLLER_MANAGER:-/controller_manager}"
controller="${MIOS_MODEL_BROADCASTER:-mios_robot_model_broadcaster}"
deadline=$((SECONDS + ${MIOS_CONTROLLER_WAIT_SECONDS:-60}))

while ! ros2 control list_controllers -c "${controller_manager}" >/dev/null 2>&1; do
  if (( SECONDS >= deadline )); then
    echo "Timed out waiting for ${controller_manager}." >&2
    exit 1
  fi
  sleep 1
done

state="$(ros2 control list_controllers -c "${controller_manager}" \
  | awk -v name="${controller}" '$1 == name { print $NF }')"
if [[ -z "${state}" ]]; then
  ros2 control load_controller "${controller}" -c "${controller_manager}"
  state="$(ros2 control list_controllers -c "${controller_manager}" \
    | awk -v name="${controller}" '$1 == name { print $NF }')"
fi

if [[ "${state}" == "unconfigured" ]]; then
  ros2 control set_controller_state "${controller}" inactive -c "${controller_manager}"
  state="inactive"
fi

if [[ "${state}" == "inactive" ]]; then
  ros2 control set_controller_state "${controller}" active -c "${controller_manager}"
  state="active"
fi

if [[ "${state}" != "active" ]]; then
  echo "${controller} is ${state:-missing}, expected active." >&2
  exit 1
fi

echo "${controller} is active; ROS model data is available to the MIOS Core."
