#pragma once

#include <array>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <Eigen/Dense>

#include "mios/control/control_types.hpp"
#include "mios/utils/types.hpp"

namespace mios {

// Core owns behaviour, not robot transport. Implementations may use ROS 2 or
// a legacy adapter, but only the backend can touch its transport details.
class RobotBackend {
 public:
  using ControlCallback = std::function<control::ArmCommand(
      const control::RobotState&, const control::RobotModel&, const control::GripperState&,
      double period_seconds)>;
  using RobotParameterProvider =
      std::function<std::optional<control::RobotParameters>()>;

  virtual ~RobotBackend() = default;

  virtual bool initialize() = 0;
  virtual bool connect_to_robot(const std::optional<std::string>& ip) = 0;
  virtual bool connect_to_gripper(const std::optional<std::string>& ip) = 0;
  virtual void disconnect_from_robot() = 0;
  virtual void disconnect_from_gripper() = 0;
  virtual ControlReturnType recover() = 0;
  virtual bool pre_run_checks() const = 0;

  // Transport-neutral control contract. Implementations own all middleware
  // details; Core returns only a MIOS ArmCommand.
  virtual ControlReturnType control(control::CommandMode mode, ControlCallback callback) = 0;

  // Core provides the current parameter snapshot after its Memory is
  // constructed. Legacy adapters may ignore it if they already own Memory.
  virtual void set_robot_parameter_provider(RobotParameterProvider /*provider*/) {}
  virtual bool set_robot_parameters() = 0;
  // Core uses this transport-neutral snapshot outside the real-time callback.
  virtual bool get_robot_snapshot(control::RobotState& robot_state,
                                  control::RobotModel& robot_model,
                                  control::GripperState& gripper_state) const = 0;

  virtual bool grasp(double width, double speed, double force, double epsilon_inner,
                     double epsilon_outer) const = 0;
  virtual bool move_to_finger_position(double width, double speed) const = 0;
  virtual bool home_gripper() const = 0;
  virtual bool unlock_brakes() = 0;
  virtual bool lock_brakes() = 0;
  virtual bool shutdown_robot() = 0;
  virtual bool reboot_robot() = 0;
};

}  // namespace mios
