#include "franka_cartesian_impedance_controller/cartesian_impedance_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <rclcpp/logging.hpp>

namespace franka_cartesian_impedance_controller {

CartesianImpedanceController::CartesianImpedanceController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn CartesianImpedanceController::on_init() {
  RCLCPP_INFO(rclcpp::get_logger("CartesianImpedanceController"), "on_init called");
  franka_cartesian_pose_ = std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(false);
  stiffness_ << 24.0, 24.0, 24.0;
  damping_ << 2.0, 2.0, 2.0;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_configure(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(rclcpp::get_logger("CartesianImpedanceController"), "on_configure called");

  pose_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Pose>(
      "desired_cartesian_pose", 10,
      [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        desired_pose_msg_ = *msg;
      });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_activate(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(rclcpp::get_logger("CartesianImpedanceController"), "on_activate called");
  initialization_flag_ = true;
  franka_cartesian_pose_->assign_loaned_command_interfaces(command_interfaces_);
  franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_deactivate(const rclcpp_lifecycle::State&) {
  franka_cartesian_pose_->release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration CartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_command_interface_names();
  return config;
}

controller_interface::InterfaceConfiguration CartesianImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_state_interface_names();
  return config;
}

controller_interface::return_type CartesianImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {

  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000, "update running...");
  const double dt = period.seconds();

  std::tie(current_orientation_, current_position_) =
      franka_cartesian_pose_->getCurrentOrientationAndTranslation();

  if (initialization_flag_) {
    previous_position_ = current_position_;
    initialization_flag_ = false;
    return controller_interface::return_type::OK;
  }

  velocity_ = (current_position_ - previous_position_) / dt;
  previous_position_ = current_position_;

  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    desired_position_ << desired_pose_msg_.position.x,
                         desired_pose_msg_.position.y,
                         desired_pose_msg_.position.z;

    desired_orientation_.coeffs() << desired_pose_msg_.orientation.x,
                                     desired_pose_msg_.orientation.y,
                                     desired_pose_msg_.orientation.z,
                                     desired_pose_msg_.orientation.w;
  }

  pose_error_ = desired_position_ - current_position_;
  velocity_error_ = -velocity_;

  Eigen::Vector3d impedance_delta =
      stiffness_.cwiseProduct(pose_error_) + damping_.cwiseProduct(velocity_error_);

  Eigen::Vector3d commanded_position = current_position_ + impedance_delta * dt;

  if (franka_cartesian_pose_->setCommand(desired_orientation_, commanded_position)) {
    return controller_interface::return_type::OK;
  } else {
    RCLCPP_FATAL(get_node()->get_logger(), "Set command failed.");
    return controller_interface::return_type::ERROR;
  }
}

}  // namespace franka_cartesian_impedance_controller

PLUGINLIB_EXPORT_CLASS(franka_cartesian_impedance_controller::CartesianImpedanceController,
                       controller_interface::ControllerInterface)
