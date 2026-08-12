#pragma once

#include <memory>
#include <string>
#include <tuple>

#include <Eigen/Geometry>
#include <controller_interface/controller_interface.hpp>
#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>
#include <franka_semantic_components/franka_cartesian_velocity_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>

namespace mios_franka_ros2 {

struct PoseTarget {
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  bool valid{false};
};

struct VelocityTarget {
  Eigen::Vector3d linear{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular{Eigen::Vector3d::Zero()};
  double received_at{0.0};
  bool valid{false};
};

class CartesianPoseController : public controller_interface::ControllerInterface {
 public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;

 private:
  std::string arm_prefix_;
  std::string base_frame_;
  double max_linear_velocity_{0.1};
  double max_angular_velocity_{0.5};
  Eigen::Quaterniond commanded_orientation_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d commanded_position_{Eigen::Vector3d::Zero()};
  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> interface_;
  realtime_tools::RealtimeBuffer<PoseTarget> target_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};

class CartesianVelocityController : public controller_interface::ControllerInterface {
 public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;

 private:
  std::string arm_prefix_;
  std::string base_frame_;
  double command_timeout_{0.1};
  double max_linear_velocity_{0.1};
  double max_angular_velocity_{0.5};
  std::unique_ptr<franka_semantic_components::FrankaCartesianVelocityInterface> interface_;
  realtime_tools::RealtimeBuffer<VelocityTarget> target_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_;
};

}  // namespace mios_franka_ros2
