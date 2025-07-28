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
#include <Eigen/QR> 

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

  KDL::JntArray q(joint_names_.size());
  KDL::JntArray q_dot(joint_names_.size());
  KDL::JntArray torques(joint_names_.size());
  KDL::JntArray null_space_torques(joint_names_.size());


  // Populate joint positions and velocities from state interfaces
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    q(i) = joint_positions_[i];
    q_dot(i) = joint_velocities_[i];
  }

  // Calculate null space torques
  for (size_t limb_id = 0; limb_id < limb_names_.size(); ++limb_id) {
    if (limb_id < 2){
      continue;
    }
    unsigned int num_joints = limb_chains_[limb_id].getNrOfJoints();
    KDL::JntArray limb_q(num_joints);
    for (size_t j = 0; j < num_joints; ++j) {
      limb_q(j) = joint_positions_[limb_joint_indices_[limb_id][j]];
    }
    RCLCPP_INFO(
      get_node()->get_logger(), "Limb %s joint positions: %s",
      limb_names_[limb_id].c_str(), formatVector(std::vector<double>(limb_q.data.data(), limb_q.data.data() + limb_q.data.rows())).c_str());
    KDL::Jacobian jacobian(num_joints);
    KDL::ChainJntToJacSolver jac_solver(limb_chains_[limb_id]);
    int ret = jac_solver.JntToJac(limb_q, jacobian);
    // if (ret < 0) {
    //   RCLCPP_ERROR(
    //     get_node()->get_logger(), "Failed to compute Jacobian for limb %s", limb_names_[limb_id].c_str());

    // } else {
    //   RCLCPP_INFO(
    //     get_node()->get_logger(), "Jacobian for limb %s: %s",
    //     limb_names_[limb_id].c_str(), formatVector(std::vector<double>(jacobian.data.data(), jacobian.data.data() + jacobian.data.size())).c_str());
    // }

    Eigen::MatrixXd J = jacobian.data;
    Eigen::MatrixXd J_dagger = J.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(num_joints, num_joints);
    Eigen::MatrixXd null_space_projector = I - J_dagger * J;
    Eigen::VectorXd null_space_joint_target = Eigen::VectorXd::Zero(num_joints);
    for (size_t i = 0; i < num_joints; ++i) {
      null_space_joint_target(i) = null_joint_positions_[limb_joint_indices_[limb_id][i]];
    }
    Eigen::VectorXd q_error = limb_q.data - null_space_joint_target.head(num_joints);
    Eigen::VectorXd tau_n = null_space_projector * (-q_error);
    for (size_t i = 0; i < num_joints; ++i) {
      double kp_value = null_joint_kp_[limb_joint_indices_[limb_id][i]];
      if (kp_value > 0.0){
        null_space_torques(limb_joint_indices_[limb_id][i]) += tau_n(i) * kp_value;
        RCLCPP_INFO(
          get_node()->get_logger(), "Null space torque for joint %s: %f",
          joint_names_[limb_joint_indices_[limb_id][i]].c_str(), tau_n(i) * kp_value
        );
        RCLCPP_INFO(
          get_node()->get_logger(), "Null space  KP %s: %f",
          joint_names_[limb_joint_indices_[limb_id][i]].c_str(), kp_value
        );
      } else {
        null_space_torques(limb_joint_indices_[limb_id][i]) = 0.0;
      }
    }
  }

  // Base Pose Feedback
  std::string torques_str = "===================\n";
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    if (i >= joint_names_.size()) {
      continue;
    }
    std::string joint_name = joint_names_[i];
    double diff = base_joint_positions_[i] - q(i);
    torques(i) = diff;

    torques(i) = std::max(-1.0, torques(i));
    torques(i) = std::min(torques(i), 1.0);
    torques(i) = torques(i) * base_joint_kp_[i];

    torques(i) += null_space_torques(i);  // Add null space torques to the base torques

    torques_str += "" + joint_name + ": base: " + std::to_string(base_joint_positions_[i]) +
      ", joint: " +  std::to_string(q(i)) + ", diff: " + std::to_string(q(i) - base_joint_positions_[i]) +
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
  base_joint_positions_.resize(dof_);
  base_joint_kp_.resize(dof_);
  null_joint_positions_.resize(dof_);
  null_joint_kp_.resize(dof_);

  if (params_.joints.empty()) {
    // TODO(destogl): is this correct? Can we really move-on if no joint names are not provided?
    RCLCPP_WARN(logger, "'joints' parameter is empty.");
  }

  joint_names_ = params_.joints;

  // Read init_pose from params_, it is dictionary of joint names to init_pose value
  std::string init_pose_str = "";
  std::string null_pose_str = "";
  for (size_t index = 0; index < joint_names_.size(); ++index) {
    std::string joint_name = joint_names_[index];
    base_joint_positions_[index] = params_.init_pose[index];
    base_joint_kp_[index] = params_.BaseKp[index];
    init_pose_str += joint_name + ": " + std::to_string(base_joint_positions_[index]) + ", ";
    null_joint_positions_[index] = params_.nullspace_pose[index];
    null_joint_kp_[index] = params_.NullKp[index];
    null_pose_str += joint_name + ": " + std::to_string(null_joint_positions_[index]) + ", ";
  }
  RCLCPP_INFO(logger, "Joint init pose: %s", init_pose_str.c_str());
  RCLCPP_INFO(logger, "Joint null space pose: %s", null_pose_str.c_str());

  n_joints_ = joint_names_.size();

  command_joint_names_ = params_.command_joints;

  if (command_joint_names_.empty()) {
    command_joint_names_ = params_.joints;
    RCLCPP_INFO(
      logger, "No specific joint names are used for command interfaces. Using 'joints' parameter.");
  }

  joint_command_interface_.resize(command_interface_types_.size());
  joint_state_interface_.resize(state_interface_types_.size());

  // Initalize limb names, base links, and end effector links
  limb_names_ = params_.limb_names;
  base_links_ = params_.base_links;
  end_effector_links_ = params_.end_effector_links;

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

    
    for (size_t i = 0; i < limb_names_.size(); ++i) {
      KDL::Chain chain;
      if (!tree_.getChain(base_links_[i], end_effector_links_[i], chain)) {
        RCLCPP_ERROR(
          get_node()->get_logger(), "Failed to get chain from %s to %s",  base_links_[i].c_str(), end_effector_links_[i].c_str());
        return CallbackReturn::ERROR;
      }
      limb_chains_.push_back(chain);

      RCLCPP_INFO(
        get_node()->get_logger(), "Successfully got chain from %s to %s with %zu joints",
        base_links_[i].c_str(), end_effector_links_[i].c_str(), chain.getNrOfJoints());

      std::vector<int> joint_indices;
      for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
          const KDL::Segment& segment = chain.getSegment(i);
          const KDL::Joint& joint = segment.getJoint();
        
          // Only list non-fixed joints
          if (joint.getType() != KDL::Joint::None) {
              std::string joint_name = joint.getName();

              auto it = std::find(joint_names_.begin(), joint_names_.end(), joint_name);
              if (it != joint_names_.end()) {
                size_t index = static_cast<size_t>(std::distance(joint_names_.begin(), it));
                joint_indices.push_back(static_cast<int>(index));
                RCLCPP_INFO(
                  get_node()->get_logger(), "Joint %s found at index %zu", joint_name.c_str(), index);
              } else {
                RCLCPP_WARN(
                  get_node()->get_logger(), "Joint %s not found in joint_names_ vector",
                  joint_name.c_str());
              }
          
          }
      }
      limb_joint_indices_.push_back(joint_indices);
      RCLCPP_INFO(
        get_node()->get_logger(), "Limb %s has %zu joints", limb_names_[i].c_str(),
        joint_indices.size());  
    }

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
//       base_joint_positions_[i] = static_cast<double>(request->joint_state.position.at(index));
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
      base_joint_positions_[i] = static_cast<double>(msg->position.at(index));
      base_joint_kp_[i] = static_cast<double>(msg->velocity.at(index));
    } 
  }

}


}  // namespace feedback_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  feedback_controller::FeedbackController,
  controller_interface::ControllerInterface)
