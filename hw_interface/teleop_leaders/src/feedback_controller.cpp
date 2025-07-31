// Copyright 2021 ros2_control development team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "feedback_controller/feedback_controller.hpp"
#include <chrono>
#include <string>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "controller_interface/helpers.hpp"

namespace feedback_controller
{
FeedbackController::FeedbackController()
: controller_interface::ControllerInterface(),
  dither_switch_(false)
{
}

controller_interface::InterfaceConfiguration
FeedbackController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : joint_names_) {
    for (const auto & interface_type : command_interface_types_) {
      config.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return config;
}

controller_interface::InterfaceConfiguration
FeedbackController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : joint_names_) {
    for (const auto & interface_type : state_interface_types_) {
      config.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return config;
}

controller_interface::return_type FeedbackController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto assign_point_from_interface =
    [&](std::vector<double> & trajectory_point_interface, const auto & joint_interface) {
      for (size_t index = 0; index < dof_; ++index) {
        trajectory_point_interface[index] = joint_interface[index].get().get_value();
      }
    };

  assign_point_from_interface(joint_positions_, joint_state_interface_[0]);
  assign_point_from_interface(joint_velocities_, joint_state_interface_[1]);

  // Create KDL objects for computation
  KDL::TreeIdSolver_RNE idsolver(tree_, KDL::Vector(0, 0, -9.81));
  KDL::JntArray q(joint_names_.size());
  KDL::JntArray q_dot(joint_names_.size());
  KDL::JntArray torques(joint_names_.size());

  // Populate joint positions and velocities from state interfaces
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    q(i) = joint_positions_[i];
    q_dot(i) = joint_velocities_[i];
  }


  std::string torques_str = "===================\n";
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    if (i >= joint_names_.size()) {
      continue;
    }
    std::string joint_name = joint_names_[i];
    double diff = joint_init_pose_[i] - q(i);
    torques(i) = diff;

    torques(i) = std::max(-1.0, torques(i));
    torques(i) = std::min(torques(i), 1.0);
    torques(i) = torques(i) * Kp_[i];

    torques_str += "" + joint_name + ": base: " + std::to_string(joint_init_pose_[i]) +
      ", joint: " +  std::to_string(q(i)) + ", diff: " + std::to_string(q(i) - joint_init_pose_[i]) +
      ", torque: " + std::to_string(torques(i)) + "\n";

    joint_command_interface_[0][i].get().set_value(torques(i));
  }

  // Print the torques
  // RCLCPP_INFO(get_node()->get_logger(), "%s", torques_str.c_str());

  dither_switch_ = !dither_switch_;  // Flip the dither switch

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn FeedbackController::on_init()
{
  try {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  // set_base_pose_service_ = get_node()->create_service<teleop_msgs::srv::SetBasePose>(
  //   "set_base_pos",
  //   std::bind(&FeedbackController::set_base_pose_callback, this, std::placeholders::_1, std::placeholders::_2));

  set_base_pose_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    "set_base_pos", 10,
    std::bind(&FeedbackController::set_base_pose_callback, this, std::placeholders::_1));

  fprintf(stderr, "Successfully got Base");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FeedbackController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  if (!param_listener_) {
    RCLCPP_ERROR(logger, "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  // get degrees of freedom
  dof_ = params_.joints.size();
  joint_positions_.resize(dof_);
  joint_velocities_.resize(dof_);
  joint_init_pose_.resize(dof_);
  Kp_.resize(dof_);

  if (params_.joints.empty()) {
    // TODO(destogl): is this correct? Can we really move-on if no joint names are not provided?
    RCLCPP_WARN(logger, "'joints' parameter is empty.");
  }

  joint_names_ = params_.joints;

  // Read init_pose from params_, it is dictionary of joint names to init_pose value
  std::string init_pose_str = "";
  for (size_t index = 0; index < joint_names_.size(); ++index) {
    std::string joint_name = joint_names_[index];
    joint_init_pose_[index] = params_.init_pose[index];
    Kp_[index] = params_.Kp[index];
    init_pose_str += joint_name + ": " + std::to_string(joint_init_pose_[index]) + ", ";
  }
  RCLCPP_INFO(logger, "Joint init pose: %s", init_pose_str.c_str());

  n_joints_ = joint_names_.size();

  command_joint_names_ = params_.command_joints;

  if (command_joint_names_.empty()) {
    command_joint_names_ = params_.joints;
    RCLCPP_INFO(
      logger, "No specific joint names are used for command interfaces. Using 'joints' parameter.");
  }

  joint_command_interface_.resize(command_interface_types_.size());
  joint_state_interface_.resize(state_interface_types_.size());

  const std::string & urdf = params_.robot_description;
  if (!urdf.empty()) {
    if (!kdl_parser::treeFromString(urdf, tree_)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse robot description!");
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_node()->get_logger(), "[KDL] number of joints: %d", tree_.getNrOfJoints());

    for (const auto & segment : tree_.getSegments()) {
      RCLCPP_INFO(get_node()->get_logger(), "[KDL] segment name: %s", segment.first.c_str());
    }

    RCLCPP_INFO(get_node()->get_logger(), "Successfully parsed the robot description.");

    q_ddot_.resize(joint_names_.size());
  } else {
    // empty URDF is used for some tests
    RCLCPP_DEBUG(get_node()->get_logger(), "No URDF file given");
  }

  RCLCPP_INFO(get_node()->get_logger(), "FeedbackController configured successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FeedbackController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();
  // order all joints in the storage
  for (const auto & interface : params_.command_interfaces) {
    auto it =
      std::find(command_interface_types_.begin(), command_interface_types_.end(), interface);
    auto index = static_cast<size_t>(std::distance(command_interface_types_.begin(), it));
    if (!controller_interface::get_ordered_interfaces(
        command_interfaces_, command_joint_names_, interface, joint_command_interface_[index]))
    {
      RCLCPP_ERROR(
        logger, "Expected %zu '%s' command interfaces, got %zu.", dof_, interface.c_str(),
        joint_command_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }
  for (const auto & interface : params_.state_interfaces) {
    auto it =
      std::find(state_interface_types_.begin(), state_interface_types_.end(), interface);
    auto index = static_cast<size_t>(std::distance(state_interface_types_.begin(), it));
    if (!controller_interface::get_ordered_interfaces(
        state_interfaces_, params_.joints, interface, joint_state_interface_[index]))
    {
      RCLCPP_ERROR(
        logger, "Expected %zu '%s' state interfaces, got %zu.", dof_, interface.c_str(),
        joint_state_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(get_node()->get_logger(), "FeedbackController activated successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FeedbackController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < n_joints_; ++i) {
    for (size_t j = 0; j < command_interface_types_.size(); ++j) {
      command_interfaces_[i * command_interface_types_.size() + j].set_value(0.0);
    }
  }
  RCLCPP_INFO(get_node()->get_logger(), "FeedbackController deactivated successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FeedbackController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset flags and parameters
  dither_switch_ = false;

  // Clear KDL tree and joint name map
  tree_ = KDL::Tree();

  // Clear vectors
  f_ext_.clear();

  RCLCPP_INFO(get_node()->get_logger(), "FeedbackController cleaned up successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FeedbackController::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(get_node()->get_logger(), "Error occurred in FeedbackController.");
  return CallbackReturn::ERROR;
}

controller_interface::CallbackReturn FeedbackController::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Shutting down FeedbackController.");
  return CallbackReturn::SUCCESS;
}

std::string FeedbackController::formatVector(const std::vector<double> & vec)
{
  std::ostringstream oss;
  for (size_t i = 0; i < vec.size(); ++i) {
    oss << vec[i];
    if (i != vec.size() - 1) {
      oss << ", ";
    }
  }
  return oss.str();
}

// void FeedbackController::set_base_pose_callback(
//   const std::shared_ptr<teleop_msgs::srv::SetBasePose::Request> request,
//   std::shared_ptr<teleop_msgs::srv::SetBasePose::Response> response)
// {

//   RCLCPP_INFO(
//     get_node()->get_logger(), "Got base pose");
//   for (size_t i = 0; i < joint_names_.size(); ++i) {
//     auto it = std::find(request->joint_state.name.begin(), request->joint_state.name.end(), joint_names_[i]);
//     if (it != request->joint_state.name.end()) {
//       size_t index = static_cast<size_t>(std::distance(request->joint_state.name.begin(), it));
//       joint_init_pose_[i] = static_cast<double>(request->joint_state.position.at(index));
//       Kp_[i] = static_cast<double>(request->joint_state.velocity.at(index));
//     } else {
//       RCLCPP_ERROR(get_node()->get_logger(), "Joint name %s not found in feedback message.", joint_names_[i].c_str());
//     }
//   }
//   response->success = true;
// }
void FeedbackController::set_base_pose_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{

  RCLCPP_INFO(
    get_node()->get_logger(), "Got base pose");
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
    if (it != msg->name.end()) {
      size_t index = static_cast<size_t>(std::distance(msg->name.begin(), it));
      joint_init_pose_[i] = static_cast<double>(msg->position.at(index));
      Kp_[i] = static_cast<double>(msg->velocity.at(index));
    } 
  }

}


}  // namespace feedback_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  feedback_controller::FeedbackController,
  controller_interface::ControllerInterface)
