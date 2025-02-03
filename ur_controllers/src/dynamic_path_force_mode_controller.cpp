// Copyright 2023, FZI Forschungszentrum Informatik, Created on behalf of Universal Robots A/S
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//----------------------------------------------------------------------
/*!\file dynamic_path_force_mode_controller.cpp
 * \brief Implementation of dynamic force mode across a defined robot trajectory
 *
 * \author  George kouretas gkouretas@scu.edu
 * \date    2023-06-29
 */
//----------------------------------------------------------------------

#include <limits>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/logging.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <ur_controllers/dynamic_path_force_mode_controller.hpp>

static double duration_to_double(const builtin_interfaces::msg::Duration& duration)
{
  return duration.sec + (duration.nanosec / 1000000000.0);
}

namespace ur_controllers
{
controller_interface::CallbackReturn DynamicPathForceModeController::on_init()
{
  try {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<dynamic_path_force_mode_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::InterfaceConfiguration DynamicPathForceModeController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const std::string tf_prefix = params_.tf_prefix;
  RCLCPP_DEBUG(get_node()->get_logger(), "Configure UR dynamic_force_mode controller with tf_prefix: %s",
               tf_prefix.c_str());

  // Get all the command interfaces needed for force mode from the hardware interface
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/task_frame_x");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/task_frame_y");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/task_frame_z");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/task_frame_rx");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/task_frame_ry");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/task_frame_rz");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/selection_vector_x");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/selection_vector_y");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/selection_vector_z");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/selection_vector_rx");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/selection_vector_ry");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/selection_vector_rz");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/wrench_x");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/wrench_y");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/wrench_z");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/wrench_rx");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/wrench_ry");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/wrench_rz");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/type");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/limits_x");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/limits_y");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/limits_z");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/limits_rx");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/limits_ry");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/limits_rz");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/force_mode_async_success");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/disable_cmd");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/damping");
  config.names.emplace_back(tf_prefix + "dynamic_force_mode/gain_scaling");

  return config;
}

controller_interface::InterfaceConfiguration
ur_controllers::DynamicPathForceModeController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const std::string tf_prefix = params_.tf_prefix;
  // Get the state interface indicating whether the hardware interface has been initialized
  config.names.emplace_back(tf_prefix + "system_interface/initialized");

  return config;
}

controller_interface::CallbackReturn
ur_controllers::DynamicPathForceModeController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  const auto logger = get_node()->get_logger();

  if (!param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during configuration");
    return controller_interface::CallbackReturn::ERROR;
  }

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // Create the service server that will be used to start force mode
  try {
    dynamic_force_mode_action_server_ = rclcpp_action::create_server<DynamicForceModeAction>(
        get_node(), std::string(get_node()->get_name()) + "/dynamic_force_mode_path",
        std::bind(&DynamicPathForceModeController::goal_received_callback, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&DynamicPathForceModeController::goal_cancelled_callback, this, std::placeholders::_1),
        std::bind(&DynamicPathForceModeController::goal_accepted_callback, this, std::placeholders::_1));
    // disable_force_mode_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
    //     "~/stop_force_mode",
    //     std::bind(&DynamicPathForceModeController::disableForceMode, this, std::placeholders::_1,
    //     std::placeholders::_2));
  } catch (...) {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ur_controllers::DynamicPathForceModeController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  change_requested_ = false;
  force_mode_active_ = false;
  async_state_ = std::numeric_limits<double>::quiet_NaN();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ur_controllers::DynamicPathForceModeController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Stop force mode if this controller is deactivated.
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_DISABLE_CMD].set_value(1.0);
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ur_controllers::DynamicPathForceModeController::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
  set_force_mode_srv_.reset();
  disable_force_mode_srv_.reset();
  return CallbackReturn::SUCCESS;
}

void DynamicPathForceModeController::initialize_force_mode()
{
  const auto force_mode_parameters = force_mode_params_buffer_.readFromRT();
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_TASK_FRAME_X].set_value(
      force_mode_parameters->initial_task_frame[0]);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_TASK_FRAME_Y].set_value(
      force_mode_parameters->initial_task_frame[1]);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_TASK_FRAME_Z].set_value(
      force_mode_parameters->initial_task_frame[2]);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_TASK_FRAME_RX].set_value(
      force_mode_parameters->initial_task_frame[3]);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_TASK_FRAME_RY].set_value(
      force_mode_parameters->initial_task_frame[4]);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_TASK_FRAME_RZ].set_value(
      force_mode_parameters->initial_task_frame[5]);

  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_SELECTION_VECTOR_X].set_value(
      force_mode_parameters->initial_selection_vec[0]);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_SELECTION_VECTOR_Y].set_value(
      force_mode_parameters->initial_selection_vec[1]);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_SELECTION_VECTOR_Z].set_value(
      force_mode_parameters->initial_selection_vec[2]);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_SELECTION_VECTOR_RX].set_value(
      force_mode_parameters->initial_selection_vec[3]);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_SELECTION_VECTOR_RY].set_value(
      force_mode_parameters->initial_selection_vec[4]);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_SELECTION_VECTOR_RZ].set_value(
      force_mode_parameters->initial_selection_vec[5]);

  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_WRENCH_X].set_value(
      force_mode_parameters->initial_wrench.force.x);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_WRENCH_Y].set_value(
      force_mode_parameters->initial_wrench.force.y);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_WRENCH_Z].set_value(
      force_mode_parameters->initial_wrench.force.z);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_WRENCH_RX].set_value(
      force_mode_parameters->initial_wrench.torque.x);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_WRENCH_RY].set_value(
      force_mode_parameters->initial_wrench.torque.y);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_WRENCH_RZ].set_value(
      force_mode_parameters->initial_wrench.torque.z);

  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_LIMITS_X].set_value(force_mode_parameters->limits[0]);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_LIMITS_Y].set_value(force_mode_parameters->limits[1]);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_LIMITS_Z].set_value(force_mode_parameters->limits[2]);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_LIMITS_RX].set_value(force_mode_parameters->limits[3]);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_LIMITS_RY].set_value(force_mode_parameters->limits[4]);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_LIMITS_RZ].set_value(force_mode_parameters->limits[5]);

  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_TYPE].set_value(force_mode_parameters->type);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_DAMPING].set_value(force_mode_parameters->damping_factor);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_GAIN_SCALING].set_value(
      force_mode_parameters->gain_scaling);

  // Signal that we are waiting for confirmation that force mode is activated
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_ASYNC_SUCCESS].set_value(ASYNC_WAITING);
}

void DynamicPathForceModeController::update_trajectory_points()
{
  // TODO(george): fill this out
  return;
}

controller_interface::return_type
ur_controllers::DynamicPathForceModeController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  async_state_ = command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_ASYNC_SUCCESS].get_value();

  // Publish state of dynamic_force_mode?
  if (change_requested_) {
    if (force_mode_active_) {
      initialize_force_mode();
      async_state_ = ASYNC_WAITING;
    } else {
      command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_DISABLE_CMD].set_value(1.0);
      command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_ASYNC_SUCCESS].set_value(ASYNC_WAITING);
      async_state_ = ASYNC_WAITING;
    }
    change_requested_ = false;
  } else if (force_mode_active_) {
    // Update dynamic force mode parameters
    auto logger = this->get_node()->get_logger();

    // Update speed scaling factor
    if (scaling_state_interface_.has_value()) {
      scaling_factor_ = scaling_state_interface_->get().get_value();
    }

    update_trajectory_points();
  }

  return controller_interface::return_type::OK;
}

bool DynamicPathForceModeController::setForceMode(const DynamicForceModeParameters& req)
{
  // // Reject if controller is not active
  // if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
  //   RCLCPP_ERROR(get_node()->get_logger(), "Can't accept new requests. Controller is not running.");
  //   resp->success = false;
  //   return false;
  // }

  // DynamicForceModeParameters force_mode_parameters;

  // // transform task frame into base
  // const std::string tf_prefix = params_.tf_prefix;
  // if (std::abs(req->task_frame.pose.orientation.x) < 1e-6 && std::abs(req->task_frame.pose.orientation.y) < 1e-6 &&
  //     std::abs(req->task_frame.pose.orientation.z) < 1e-6 && std::abs(req->task_frame.pose.orientation.w) < 1e-6) {
  //   RCLCPP_ERROR(get_node()->get_logger(), "Received task frame with all-zeros quaternion. It should have at least
  //   one "
  //                                          "non-zero entry.");
  //   resp->success = false;
  //   return false;
  // }

  // try {
  //   auto task_frame_transformed = tf_buffer_->transform(req->task_frame, tf_prefix + "base");

  //   force_mode_parameters.initial_task_frame[0] = task_frame_transformed.pose.position.x;
  //   force_mode_parameters.initial_task_frame[1] = task_frame_transformed.pose.position.y;
  //   force_mode_parameters.initial_task_frame[2] = task_frame_transformed.pose.position.z;

  //   tf2::Quaternion quat_tf;
  //   tf2::convert(task_frame_transformed.pose.orientation, quat_tf);
  //   tf2::Matrix3x3 rot_mat(quat_tf);
  //   rot_mat.getRPY(force_mode_parameters.initial_task_frame[3], force_mode_parameters.initial_task_frame[4],
  //                  force_mode_parameters.initial_task_frame[5]);
  // } catch (const tf2::TransformException& ex) {
  //   RCLCPP_ERROR(get_node()->get_logger(), "Could not transform %s to robot base: %s",
  //                req->task_frame.header.frame_id.c_str(), ex.what());
  //   resp->success = false;
  //   return false;
  // }

  // // The selection vector dictates which axes the robot should be compliant along and around
  // force_mode_parameters.initial_selection_vec[0] = req->selection_vector_x;
  // force_mode_parameters.initial_selection_vec[1] = req->selection_vector_y;
  // force_mode_parameters.initial_selection_vec[2] = req->selection_vector_z;
  // force_mode_parameters.initial_selection_vec[3] = req->selection_vector_rx;
  // force_mode_parameters.initial_selection_vec[4] = req->selection_vector_ry;
  // force_mode_parameters.initial_selection_vec[5] = req->selection_vector_rz;

  // // The wrench parameters dictate the amount of force/torque the robot will apply to its environment. The robot will
  // // move along/around compliant axes to match the specified force/torque. Has no effect for non-compliant axes.
  // force_mode_parameters.initial_wrench = req->wrench;

  // /* The limits specifies the maximum allowed speed along/around compliant axes. For non-compliant axes this value is
  //  * the maximum allowed deviation between actual tcp position and the one that has been programmed. */
  // force_mode_parameters.limits[0] = req->selection_vector_x ? req->speed_limits.linear.x : req->deviation_limits[0];
  // force_mode_parameters.limits[1] = req->selection_vector_y ? req->speed_limits.linear.y : req->deviation_limits[1];
  // force_mode_parameters.limits[2] = req->selection_vector_z ? req->speed_limits.linear.z : req->deviation_limits[2];
  // force_mode_parameters.limits[3] = req->selection_vector_rx ? req->speed_limits.angular.x :
  // req->deviation_limits[3]; force_mode_parameters.limits[4] = req->selection_vector_ry ? req->speed_limits.angular.y
  // : req->deviation_limits[4]; force_mode_parameters.limits[5] = req->selection_vector_rz ?
  // req->speed_limits.angular.z : req->deviation_limits[5];

  // if (req->type < 1 || req->type > 3) {
  //   RCLCPP_ERROR(get_node()->get_logger(), "The force mode type has to be 1, 2, or 3. Received %u", req->type);
  //   resp->success = false;
  //   return false;
  // }

  // /* The type decides how the robot interprets the force frame (the one defined in task_frame). See ur_script manual
  //  * for explanation, under dynamic_force_mode. */
  // force_mode_parameters.type = static_cast<double>(req->type);

  // /* The damping factor decides how fast the robot decelarates if no force is present. 0 means no deceleration, 1
  //  * means quick deceleration*/
  // if (req->damping_factor < 0.0 || req->damping_factor > 1.0) {
  //   RCLCPP_ERROR(get_node()->get_logger(), "The damping factor has to be between 0 and 1. Received %f",
  //                req->damping_factor);
  //   resp->success = false;
  //   return false;
  // }
  // force_mode_parameters.damping_factor = req->damping_factor;

  // /*The gain scaling factor scales the force mode gain. A value larger than 1 may make force mode unstable. */
  // if (req->gain_scaling < 0.0 || req->gain_scaling > 2.0) {
  //   RCLCPP_ERROR(get_node()->get_logger(), "The gain scaling has to be between 0 and 2. Received %f",
  //                req->gain_scaling);
  //   resp->success = false;
  //   return false;
  // }
  // if (req->gain_scaling > 1.0) {
  //   RCLCPP_WARN(get_node()->get_logger(),
  //               "A gain_scaling >1.0 can make force mode unstable, e.g. in case of collisions or pushing against "
  //               "hard surfaces. Received %f",
  //               req->gain_scaling);
  // }
  // force_mode_parameters.gain_scaling = req->gain_scaling;

  // force_mode_params_buffer_.writeFromNonRT(force_mode_parameters);
  // force_mode_active_ = true;
  // change_requested_ = true;

  // RCLCPP_DEBUG(get_node()->get_logger(), "Waiting for dynamic force mode to be set.");
  // const auto maximum_retries = params_.check_io_successful_retries;
  // int retries = 0;
  // while (async_state_ == ASYNC_WAITING || change_requested_) {
  //   std::this_thread::sleep_for(std::chrono::milliseconds(10));
  //   retries++;

  //   if (retries > maximum_retries) {
  //     resp->success = false;
  //   }
  // }

  // resp->success = async_state_ == 1.0;

  // if (resp->success) {
  //   RCLCPP_INFO(get_node()->get_logger(), "Dynamic force mode has been set successfully.");
  // } else {
  //   RCLCPP_ERROR(get_node()->get_logger(), "Could not set the dynamic force mode.");
  //   return false;
  // }

  return true;
}

bool DynamicPathForceModeController::disableForceMode()
{
  // force_mode_active_ = false;
  // change_requested_ = true;
  // RCLCPP_DEBUG(get_node()->get_logger(), "Waiting for dynamic force mode to be disabled.");
  // while (async_state_ == ASYNC_WAITING || change_requested_) {
  //   // Asynchronous wait until the hardware interface has set the force mode
  //   std::this_thread::sleep_for(std::chrono::milliseconds(10));
  // }
  // resp->success = async_state_ == 1.0;
  // if (resp->success) {
  //   RCLCPP_INFO(get_node()->get_logger(), "Dynamic force mode has been disabled successfully.");
  // } else {
  //   RCLCPP_ERROR(get_node()->get_logger(), "Could not disable dynamic force mode.");
  //   return false;
  // }
  return true;
}

/// @section Trajectory-related functions

rclcpp_action::GoalResponse DynamicPathForceModeController::goal_received_callback(
    const rclcpp_action::GoalUUID& /*uuid*/, std::shared_ptr<const DynamicForceModeAction::Goal> goal)
{
  RCLCPP_INFO(get_node()->get_logger(), "Received new trajectory.");
  // Precondition: Running controller
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    RCLCPP_ERROR(get_node()->get_logger(), "Can't accept new trajectories. Controller is not running.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (force_mode_active_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Can't accept new trajectory. A trajectory is already executing.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DynamicPathForceModeController::goal_cancelled_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<DynamicForceModeAction>> goal_handle)
{
  // Check that cancel request refers to currently active goal (if any)
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal && active_goal->gh_ == goal_handle) {
    RCLCPP_INFO(get_node()->get_logger(), "Cancelling active trajectory requested.");

    // Mark the current goal as canceled
    auto result = std::make_shared<DynamicForceModeAction::Result>();
    active_goal->setCanceled(result);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
    force_mode_active_ = false;
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

// Action goal was accepted, initialise values for a new trajectory.
void DynamicPathForceModeController::goal_accepted_callback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<DynamicForceModeAction>> goal_handle)
{
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "Accepted new trajectory with "
                                                   << goal_handle->get_goal()->force_mode_path.poses.size()
                                                   << " points.");

  // TODO(george): accepted callback contents
  RealtimeGoalHandlePtr rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);
  force_mode_active_ = true;
  return;
}

void DynamicPathForceModeController::end_goal()
{
  force_mode_active_ = false;
  transfer_command_interface_->get().set_value(TRANSFER_STATE_IDLE);
}

}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::DynamicPathForceModeController, controller_interface::ControllerInterface)
