#pragma once

#include "mios/control/control_types.hpp"
#include "mios_ros2_runtime/robot_snapshot.hpp"

namespace mios_ros2_runtime {

// Convert the validated state snapshot received from ROS to the original
// MIOS-neutral contract. This conversion deliberately has no ROS dependency,
// so a future Core backend can be tested without a controller or FCI.
mios::control::RobotState to_mios_robot_state(const RobotSnapshot& snapshot);

}  // namespace mios_ros2_runtime
