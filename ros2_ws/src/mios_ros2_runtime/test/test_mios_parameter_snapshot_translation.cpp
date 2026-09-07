#include <cassert>

#include <Eigen/Dense>

#include "mios/control/robot_parameter_translation.hpp"
#include "mios_ros2_runtime/mios_parameter_snapshot_translation.hpp"

namespace {

struct FakeMiosParameters {
  struct {
    double load_m{0.0};
    Eigen::Matrix<double, 3, 1> load_com;
    Eigen::Matrix<double, 3, 3> load_I;
    Eigen::Matrix<double, 2, 1> F_ext_contact;
    Eigen::Matrix<double, 7, 1> tau_ext_contact;
    Eigen::Matrix<double, 2, 1> F_ext_max;
    Eigen::Matrix<double, 7, 1> tau_ext_max;
  } user;
  struct {
    Eigen::Matrix<double, 4, 4> EE_T_TCP;
    Eigen::Matrix<double, 4, 4> EE_T_K;
  } frames;
  struct {
    struct {
      Eigen::Matrix<double, 7, 1> K_theta;
    } joint_imp;
    struct {
      Eigen::Matrix<double, 6, 1> K_x;
    } cart_imp;
  } control;
};

FakeMiosParameters valid_parameters() {
  FakeMiosParameters parameters;
  parameters.user.load_m = 1.25;
  parameters.user.load_com << 0.1, 0.2, 0.3;
  parameters.user.load_I << 1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0;
  parameters.user.F_ext_contact << 4.0, 2.0;
  parameters.user.F_ext_max << 40.0, 20.0;
  parameters.user.tau_ext_contact << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0;
  parameters.user.tau_ext_max << 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0;
  parameters.frames.EE_T_TCP.setIdentity();
  parameters.frames.EE_T_TCP(0, 3) = 0.4;
  parameters.frames.EE_T_K.setIdentity();
  parameters.frames.EE_T_K(1, 3) = 0.5;
  parameters.control.joint_imp.K_theta << 100.0, 200.0, 300.0, 400.0, 500.0, 600.0,
      700.0;
  parameters.control.cart_imp.K_x << 10.0, 20.0, 30.0, 40.0, 50.0, 60.0;
  return parameters;
}

}  // namespace

int main() {
  const auto parameters = valid_parameters();
  const auto core_parameters = mios::control::make_robot_parameters(parameters);
  const auto snapshot = mios_ros2_runtime::make_robot_parameter_snapshot(core_parameters);
  assert(snapshot);
  assert(snapshot->load_mass == 1.25);
  assert((snapshot->load_center_of_mass == std::array<double, 3>{0.1, 0.2, 0.3}));
  assert((snapshot->load_inertia == std::array<double, 9>{1.0, 0.0, 0.0, 0.0, 2.0, 0.0,
                                                          0.0, 0.0, 3.0}));
  assert(snapshot->tcp_frame[12] == 0.4);
  assert(snapshot->stiffness_frame[13] == 0.5);
  assert((snapshot->joint_stiffness == std::array<double, 7>{100.0, 200.0, 300.0, 400.0,
                                                              500.0, 600.0, 700.0}));
  assert((snapshot->cartesian_stiffness ==
          std::array<double, 6>{10.0, 20.0, 30.0, 40.0, 50.0, 60.0}));
  assert((snapshot->lower_torque_thresholds ==
          std::array<double, 7>{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0}));
  assert((snapshot->upper_torque_thresholds ==
          std::array<double, 7>{11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0}));
  assert((snapshot->lower_force_thresholds == std::array<double, 6>{4.0, 4.0, 4.0, 2.0, 2.0,
                                                                     2.0}));
  assert((snapshot->upper_force_thresholds == std::array<double, 6>{40.0, 40.0, 40.0, 20.0,
                                                                     20.0, 20.0}));

  auto invalid = valid_parameters();
  invalid.frames.EE_T_TCP(3, 3) = 0.0;
  assert(!mios_ros2_runtime::make_robot_parameter_snapshot(
      mios::control::make_robot_parameters(invalid)));
  return 0;
}
