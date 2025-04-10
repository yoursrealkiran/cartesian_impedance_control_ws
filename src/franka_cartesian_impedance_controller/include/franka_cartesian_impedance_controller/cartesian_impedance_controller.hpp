#pragma once

#include "controller_interface/controller_interface.hpp"
#include "franka_semantic_components/franka_cartesian_pose_interface.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <rclcpp/subscription.hpp>

namespace franka_cartesian_impedance_controller {

class CartesianImpedanceController : public controller_interface::ControllerInterface {
public:
  CartesianImpedanceController();

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;

  Eigen::Vector3d stiffness_;
  Eigen::Vector3d damping_;

  Eigen::Vector3d current_position_;
  Eigen::Quaterniond current_orientation_;
  Eigen::Vector3d previous_position_;
  Eigen::Vector3d velocity_;
  Eigen::Vector3d pose_error_;
  Eigen::Vector3d velocity_error_;
  Eigen::Vector3d desired_position_;
  Eigen::Quaterniond desired_orientation_;

  bool initialization_flag_ = true;

  // Pose subscriber
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscriber_;
  geometry_msgs::msg::Pose desired_pose_msg_;
  std::mutex pose_mutex_;
};

}  // namespace franka_cartesian_impedance_controller
