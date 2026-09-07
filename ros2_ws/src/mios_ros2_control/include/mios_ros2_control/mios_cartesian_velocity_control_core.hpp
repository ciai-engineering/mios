#pragma once

#include <array>
#include <cstdint>

namespace mios_ros2_control {

struct MiosCartesianVelocityRequest {
  std::array<double, 6> velocity{};
  std::int64_t received_nanoseconds{0};
  bool user_stopped{false};
};

// Command conditioning for the legacy CartesianVelocityControllerPipeline.
// Components use Franka's [vx, vy, vz, wx, wy, wz] interface order.
class MiosCartesianVelocityControlCore {
 public:
  using CartesianVelocity = std::array<double, 6>;
  using JointVelocity = std::array<double, 7>;

  void configure(const CartesianVelocity& velocity_limits,
                 const CartesianVelocity& acceleration_limits,
                 std::int64_t command_timeout_nanoseconds);
  void reset();

  [[nodiscard]] CartesianVelocity step(const JointVelocity& measured_joint_velocity,
                                       bool user_stopped,
                                       const MiosCartesianVelocityRequest* request,
                                       std::int64_t now_nanoseconds,
                                       double period_seconds);

 private:
  CartesianVelocity velocity_limits_{};
  CartesianVelocity acceleration_limits_{};
  CartesianVelocity applied_velocity_{};
  std::int64_t command_timeout_nanoseconds_{0};
};

}  // namespace mios_ros2_control
