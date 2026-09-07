// Copyright (c) 2026 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>

/**
 * @file mobile_model.h
 * Contains the franka::MobileModel type for mobile robot kinematics.
 */

namespace franka {

/**
 * Enumerates the frames of a mobile robot's drive modules.
 */
enum class MobileFrame : std::uint8_t {
  kFrontDriveModule,  ///< Front swerve drive module frame
  kRearDriveModule    ///< Rear swerve drive module frame
};

/// Number of swerve drive modules.
inline constexpr size_t kNumDriveModules = 2;

/// Number of joints per drive module (steering + drive).
inline constexpr size_t kJointsPerDriveModule = 2;

/**
 * Joint positions for all controllable subsystems of a mobile robot.
 *
 * All arrays are default-initialized to zero (neutral configuration).
 */
struct MobileJointPositions {
  /// Drive module joint positions: {front_steering, front_drive, rear_steering, rear_drive}.
  std::array<double, kNumDriveModules * kJointsPerDriveModule> drive_modules{};
};

/**
 * Calculates poses for mobile robot frames using Pinocchio.
 *
 * This class provides forward kinematics for mobile robots. It is constructed from a mobile robot
 * URDF and will reject arm robot URDFs.
 *
 * @note The pose() method is RT-safe (zero heap allocation). Passive joints (casters, rocker arm)
 * are held at their neutral configuration internally.
 */
class MobileModel final {
 public:
  /// Number of swerve drive modules.
  static constexpr size_t kNumModules = kNumDriveModules;

  /// Number of joints per drive module (steering + drive).
  static constexpr size_t kJointsPerModule = kJointsPerDriveModule;

  /**
   * Constructs a MobileModel from a URDF string.
   *
   * @param[in] urdf_model The URDF model string describing a mobile robot.
   *
   * @throw ModelException if the URDF describes an arm robot rather than a mobile robot.
   */
  explicit MobileModel(const std::string& urdf_model);

  /**
   * Move-constructs a new MobileModel instance.
   *
   * @param[in] other Other MobileModel instance.
   */
  MobileModel(MobileModel&& other) noexcept;

  /**
   * Move-assigns this MobileModel from another MobileModel instance.
   *
   * @param[in] other Other MobileModel instance.
   *
   * @return MobileModel instance.
   */
  MobileModel& operator=(MobileModel&& other) noexcept;

  /**
   * Destructor.
   */
  ~MobileModel() noexcept;

  /// @cond DO_NOT_DOCUMENT
  MobileModel(const MobileModel&) = delete;
  MobileModel& operator=(const MobileModel&) = delete;
  /// @endcond

  /**
   * Gets the 4x4 pose matrix for the given mobile frame relative to the robot's base frame
   * (URDF root link, body-fixed).
   *
   * The pose is represented as a 4x4 matrix in column-major format.
   * This method is RT-safe: it performs zero heap allocation.
   *
   * @param[in] frame The desired drive module frame (kFrontDriveModule or kRearDriveModule).
   * @param[in] joint_positions Joint positions for all controllable subsystems. Angles are in
   * radians; the conversion to the internal Pinocchio representation is handled automatically.
   *
   * @return Vectorized 4x4 pose matrix, column-major.
   */
  std::array<double, 16> pose(MobileFrame frame, const MobileJointPositions& joint_positions) const;

 private:
  class Data;
  std::unique_ptr<Data> data_;
};

}  // namespace franka
