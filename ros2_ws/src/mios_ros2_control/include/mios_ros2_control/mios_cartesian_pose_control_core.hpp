#pragma once

#include <array>
#include <cstdint>

namespace mios_ros2_control {

struct MiosCartesianPoseRequest {
  std::array<double, 16> base_to_end_effector{};
  std::int64_t received_nanoseconds{0};
  bool user_stopped{false};
};

// Deterministic Cartesian-pose conditioning for MIOS. Poses use the Franka
// and libfranka column-major 4x4 base-to-end-effector representation.
class MiosCartesianPoseControlCore {
 public:
  using Pose = std::array<double, 16>;
  using Translation = std::array<double, 3>;

  void configure(const Translation& lower_workspace_limits,
                 const Translation& upper_workspace_limits,
                 double linear_velocity_limit, double angular_velocity_limit,
                 std::int64_t command_timeout_nanoseconds);
  void reset();
  void capture_hold_reference(const Pose& pose);

  // Invalid state, stop input, or stale input returns the current live pose in
  // the same update cycle. A default/zero pose is never a safe fallback.
  [[nodiscard]] Pose step(const Pose& measured_pose, bool user_stopped,
                          const MiosCartesianPoseRequest* request,
                          std::int64_t now_nanoseconds, double period_seconds);

  [[nodiscard]] static bool is_valid_pose(const Pose& pose);

 private:
  Translation lower_workspace_limits_{};
  Translation upper_workspace_limits_{};
  double linear_velocity_limit_{0.0};
  double angular_velocity_limit_{0.0};
  Pose applied_pose_{};
  bool hold_reference_valid_{false};
  std::int64_t command_timeout_nanoseconds_{0};
};

}  // namespace mios_ros2_control
