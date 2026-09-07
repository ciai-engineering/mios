#pragma once

#include <algorithm>
#include <array>

#include "mios/control/control_types.hpp"

namespace mios::control {
namespace detail {

template <std::size_t Size, typename EigenMatrix>
std::array<double, Size> parameter_matrix_to_array(const EigenMatrix& matrix) {
  std::array<double, Size> values{};
  std::copy_n(matrix.data(), Size, values.begin());
  return values;
}

}  // namespace detail

// Maps the exact values consumed by legacy PandaBody::set_robot_parameters().
// This template deliberately has no Memory, ROS, or libfranka dependency and
// works with mios::Parameters when Core supplies its immutable configuration.
template <typename MiosParameters>
RobotParameters make_robot_parameters(const MiosParameters& parameters) {
  RobotParameters result;
  result.load_mass = parameters.user.load_m;
  result.load_center_of_mass =
      detail::parameter_matrix_to_array<3>(parameters.user.load_com);
  result.load_inertia = detail::parameter_matrix_to_array<9>(parameters.user.load_I);
  result.tcp_frame = detail::parameter_matrix_to_array<16>(parameters.frames.EE_T_TCP);
  result.stiffness_frame = detail::parameter_matrix_to_array<16>(parameters.frames.EE_T_K);
  result.joint_stiffness =
      detail::parameter_matrix_to_array<kJointCount>(parameters.control.joint_imp.K_theta);
  result.cartesian_stiffness =
      detail::parameter_matrix_to_array<6>(parameters.control.cart_imp.K_x);
  result.lower_torque_thresholds =
      detail::parameter_matrix_to_array<kJointCount>(parameters.user.tau_ext_contact);
  result.upper_torque_thresholds =
      detail::parameter_matrix_to_array<kJointCount>(parameters.user.tau_ext_max);

  const auto force_contact = detail::parameter_matrix_to_array<2>(parameters.user.F_ext_contact);
  const auto force_max = detail::parameter_matrix_to_array<2>(parameters.user.F_ext_max);
  result.lower_force_thresholds = {force_contact[0], force_contact[0], force_contact[0],
                                   force_contact[1], force_contact[1], force_contact[1]};
  result.upper_force_thresholds = {force_max[0], force_max[0], force_max[0],
                                   force_max[1], force_max[1], force_max[1]};
  return result;
}

}  // namespace mios::control
