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

static double time_to_double(const builtin_interfaces::msg::Time& time)
{
  return time.sec + (time.nanosec / 1000000000.0);
}

namespace ur_controllers
{
controller_interface::CallbackReturn DynamicPathForceModeController::on_init()
{
  try {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<dynamic_path_force_mode_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
    current_index_ = 0;
    initial_time_ = 0.0;
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
    RCLCPP_INFO(get_node()->get_logger(), (std::string(get_node()->get_name()) + "/dynamic_force_mode_path").c_str());
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

double DynamicPathForceModeController::time_from_start(const builtin_interfaces::msg::Time& time) const
{
  return time_to_double(time) - initial_time_;
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

void DynamicPathForceModeController::update_pose_actual_desired(std::shared_ptr<RealtimeGoalHandle> active_goal)
{
  auto goal = active_goal->gh_->get_goal();
  auto _tf = tf_buffer_->transform(goal->task_frame, params_.tf_prefix + "base");

  pose_actual_.setOrigin(tf2::Vector3(_tf.pose.position.x, _tf.pose.position.y, _tf.pose.position.z));
  pose_actual_.setRotation(
      tf2::Quaternion(_tf.pose.orientation.x, _tf.pose.orientation.y, _tf.pose.orientation.z, _tf.pose.orientation.w));

  if (!find_pose_desired()) {
    // Set the desired pose to the final pose if we are unable to interpolate
    _tf = active_path_.poses.back();
    pose_desired_.setOrigin(tf2::Vector3(_tf.pose.position.x, _tf.pose.position.y, _tf.pose.position.z));
    pose_desired_.setRotation(tf2::Quaternion(_tf.pose.orientation.x, _tf.pose.orientation.y, _tf.pose.orientation.z,
                                              _tf.pose.orientation.w));
  }
}

bool DynamicPathForceModeController::find_pose_desired()
{
    double _t0, _t1;
    size_t _index = current_index_;
    const double _current_duration = duration_to_double(active_path_elapsed_time_);

    while (_index < active_path_.poses.size() - 1) {
        _t0 = time_from_start(active_path_.poses[_index].header.stamp);
        _t1 = time_from_start(active_path_.poses[_index + 1].header.stamp);
        if (_current_duration >= _t0 && _current_duration <= _t1) {
            pose_desired_ = interpolate_poses(active_path_.poses[_index].pose, active_path_.poses[_index + 1].pose,
                                                std::clamp(((_current_duration - _t0) / (_t1 - _t0)), 0.0, 1.0));
            return true;
        }
        _index++;
    }

    return false;
}

tf2::Transform DynamicPathForceModeController::interpolate_poses(geometry_msgs::msg::Pose& t1,
                                                                 geometry_msgs::msg::Pose& t2, double factor)
{
  tf2::Transform tf_out, tf2_t1, tf2_t2;

  tf2_t1.setOrigin(tf2::Vector3(t1.position.x, t1.position.y, t1.position.z));
  tf2_t1.setRotation(tf2::Quaternion(t1.orientation.x, t1.orientation.y, t1.orientation.z, t1.orientation.w));

  tf2_t2.setOrigin(tf2::Vector3(t2.position.x, t2.position.y, t2.position.z));
  tf2_t2.setRotation(tf2::Quaternion(t2.orientation.x, t2.orientation.y, t2.orientation.z, t2.orientation.w));

  auto dt = ((1.0 - factor) * tf2_t1.getOrigin()) + (factor * tf2_t2.getOrigin());
  auto dq = tf2_t1.getRotation().slerp(tf2_t2.getRotation(), factor);

  tf_out.setOrigin(dt);
  tf_out.setRotation(dq);

  return tf_out;
}

void DynamicPathForceModeController::update_trajectory_points(std::shared_ptr<RealtimeGoalHandle> active_goal)
{
  const auto current_transfer_state = transfer_command_interface_->get().get_value();

  if (current_transfer_state != TRANSFER_STATE_IDLE) {
    // Check if the trajectory has been aborted from the hardware interface. E.g. the robot was stopped on the teach
    // pendant.
    if (abort_command_interface_->get().get_value() == 1.0 && current_index_ > 0) {
      RCLCPP_INFO(get_node()->get_logger(), "Trajectory aborted by hardware, aborting action.");
      std::shared_ptr<DynamicForceModeAction::Result> result = std::make_shared<DynamicForceModeAction::Result>();
      active_goal->setAborted(result);
      end_goal();
      return;
    }
  }

  // Update speed scaling factor
  // TODO(george): use the speed scaling factor?
  if (scaling_state_interface_.has_value()) {
    scaling_factor_ = scaling_state_interface_->get().get_value();
  }

  active_path_ = active_goal->gh_->get_goal()->force_mode_path;
  if (current_index_ == 0 && current_transfer_state == TRANSFER_STATE_IDLE) {
    active_path_elapsed_time_ = rclcpp::Duration(0, 0);
    max_path_trajectory_time_ = rclcpp::Duration::from_seconds(time_from_start(active_path_.poses.back().header.stamp));
    transfer_command_interface_->get().set_value(TRANSFER_WAITING_FOR_POINT);
  }

  if (current_transfer_state == TRANSFER_WAITING_FOR_POINT) {
    if (current_index_ < active_path_.poses.size()) {
      time_from_start_command_interface_->get().set_value(
          time_from_start(active_path_.poses[current_index_].header.stamp));

      compute_task_frame(active_path_.poses[current_index_].pose);

      // TODO(george): this should get pre-computed
      if (current_index_ == active_path_.poses.size()-1)
      {
        compute_compliance_vector(active_path_.poses[current_index_-1].pose,
                                  active_path_.poses[current_index_].pose);
      }
      else
      {
        compute_compliance_vector(active_path_.poses[current_index_].pose,
                                  active_path_.poses[current_index_+1].pose);
      }

      //update_pose_actual_desired(active_goal);
      current_index_++;
      transfer_command_interface_->get().set_value(TRANSFER_STATE_IN_MOTION);
    } else if (current_index_ == active_path_.poses.size()) {
        transfer_command_interface_->get().set_value(TRANSFER_STATE_DONE);
    } else {
        RCLCPP_ERROR(get_node()->get_logger(), "Hardware waiting for trajectory point while none is present!");
    }
  }

  if (current_transfer_state == TRANSFER_STATE_IN_MOTION)
  {
    if (current_index_ < active_path_.poses.size()) {
      // Get current pose
      auto task_frame_transformed = tf_buffer_->transform(active_goal->gh_->get_goal()->task_frame, params_.tf_prefix + "base");
      auto target_frame = active_path_.poses[current_index_];

      tf2::Transform t_diff = compute_relative_transform(task_frame_transformed.pose, target_frame.pose);
      if (check_pose_tolerance(t_diff, active_goal->gh_->get_goal()->waypoint_tolerances))
      {
        // Reached target within threshold
        RCLCPP_INFO(get_node()->get_logger(), "Reached waypoint index %lu", current_index_.load());
        transfer_command_interface_->get().set_value(TRANSFER_WAITING_FOR_POINT);
      }
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "No pose to query");
    }
  }
}

bool DynamicPathForceModeController::check_pose_tolerance(tf2::Transform &tf, std::array<float, 6> tolerances)
{
  auto t = tf.getOrigin();
  return std::fabs(t.getX()) < tolerances[0] &&
         std::fabs(t.getY()) < tolerances[1] &&
         std::fabs(t.getZ()) < tolerances[2];
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
  }

  const auto active_goal = *rt_active_goal_.readFromRT();
  if (active_goal && force_mode_active_) {
    // Update dynamic force mode parameters
    auto logger = this->get_node()->get_logger();

    update_trajectory_points(active_goal);
  }

  return controller_interface::return_type::OK;
}

tf2::Transform DynamicPathForceModeController::compute_relative_transform(geometry_msgs::msg::Pose& t1,
                                                                          geometry_msgs::msg::Pose& t2)
{
  tf2::Transform tf2_t1, tf2_t2;

  tf2_t1.setOrigin(tf2::Vector3(t1.position.x, t1.position.y, t1.position.z));
  tf2_t1.setRotation(tf2::Quaternion(t1.orientation.x, t1.orientation.y, t1.orientation.z, t1.orientation.w));

  tf2_t2.setOrigin(tf2::Vector3(t2.position.x, t2.position.y, t2.position.z));
  tf2_t2.setRotation(tf2::Quaternion(t2.orientation.x, t2.orientation.y, t2.orientation.z, t2.orientation.w));

  return compute_relative_transform(tf2_t1, tf2_t2);
}

tf2::Transform DynamicPathForceModeController::compute_relative_transform(tf2::Transform& t1,
                                                                           tf2::Transform& t2)
{
  return t1.inverse() * t2;
}

void DynamicPathForceModeController::compute_task_frame(geometry_msgs::msg::Pose& pose)
{
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_TASK_FRAME_X].set_value(pose.position.x);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_TASK_FRAME_Y].set_value(pose.position.x);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_TASK_FRAME_Z].set_value(pose.position.x);

  tf2::Quaternion quat_tf;
  tf2::convert(pose.orientation, quat_tf);
  std::array<double, 3> rpy;
  tf2::Matrix3x3(quat_tf).getRPY(rpy[0], rpy[1], rpy[2]);

  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_TASK_FRAME_RX].set_value(rpy[0]);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_TASK_FRAME_RY].set_value(rpy[1]);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_TASK_FRAME_RZ].set_value(rpy[2]);
}

void DynamicPathForceModeController::compute_compliance_vector(geometry_msgs::msg::Pose& t1,
                                                               geometry_msgs::msg::Pose& t2)
{
  static constexpr double POSITION_THRESH = 1e-3;
  // TEMP(george): let's always keep the orientation active and just see what happens...
  static constexpr double ORIENTATION_THRESH = 0; 
  tf2::Transform tf_out = compute_relative_transform(t1, t2);

  // Get the roll, pitch, and yaw 
  std::array<double, 3> rpy;
  tf2::Matrix3x3(tf_out.getRotation()).getRPY(rpy[0], rpy[1], rpy[2]);

  RCLCPP_INFO(get_node()->get_logger(), "Relative transformation: (%3.3f, %3.3f, %3.3f)m, (%3.3f, %3.3f, %3.3f)rad", 
              tf_out.getOrigin().getX(), tf_out.getOrigin().getY(), tf_out.getOrigin().getZ(), rpy[0], rpy[1], rpy[2]);

  // Set the compliance vector to be active if the position/orientation are greater than
  // the configured threshold
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_SELECTION_VECTOR_X].set_value(
      (std::fabs(tf_out.getOrigin().getX()) > POSITION_THRESH) ? 1.0 : 0.0);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_SELECTION_VECTOR_Y].set_value(
      (std::fabs(tf_out.getOrigin().getY()) > POSITION_THRESH) ? 1.0 : 0.0);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_SELECTION_VECTOR_Z].set_value(
      (std::fabs(tf_out.getOrigin().getZ()) > POSITION_THRESH) ? 1.0 : 0.0);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_SELECTION_VECTOR_RX].set_value(
      (std::fabs(rpy[0]) > ORIENTATION_THRESH) ? 1.0 : 0.0);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_SELECTION_VECTOR_RY].set_value(
      (std::fabs(rpy[1]) > ORIENTATION_THRESH) ? 1.0 : 0.0);
  command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_SELECTION_VECTOR_RZ].set_value(
      (std::fabs(rpy[2]) > ORIENTATION_THRESH) ? 1.0 : 0.0);
}

bool DynamicPathForceModeController::waitForAsyncCommand(std::function<double(void)> get_value)
{
  const auto maximum_retries = dynamic_force_mode_params_.check_io_successful_retries;
  int retries = 0;
  while (get_value() == ASYNC_WAITING) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    retries++;

    if (retries > maximum_retries)
      return false;
  }
  return true;
}

bool DynamicPathForceModeController::setForceMode(const ForceModeRequest* req)
{
  // Reject if controller is not active
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    RCLCPP_ERROR(get_node()->get_logger(), "Can't accept new requests. Controller is not running.");
    return false;
  }

  DynamicForceModeParameters force_mode_parameters;

  // transform task frame into base
  const std::string tf_prefix = params_.tf_prefix;
  if (std::abs(req->task_frame.pose.orientation.x) < 1e-6 && std::abs(req->task_frame.pose.orientation.y) < 1e-6 &&
      std::abs(req->task_frame.pose.orientation.z) < 1e-6 && std::abs(req->task_frame.pose.orientation.w) < 1e-6) {
    RCLCPP_ERROR(get_node()->get_logger(), "Received task frame with all-zeros quaternion. It should have at least one "
                                           "non-zero entry.");
    return false;
  }

  try {
    auto task_frame_transformed = tf_buffer_->transform(req->task_frame, tf_prefix + "base");

    force_mode_parameters.initial_task_frame[0] = task_frame_transformed.pose.position.x;
    force_mode_parameters.initial_task_frame[1] = task_frame_transformed.pose.position.y;
    force_mode_parameters.initial_task_frame[2] = task_frame_transformed.pose.position.z;

    tf2::Quaternion quat_tf;
    tf2::convert(task_frame_transformed.pose.orientation, quat_tf);
    tf2::Matrix3x3 rot_mat(quat_tf);
    rot_mat.getRPY(force_mode_parameters.initial_task_frame[3], force_mode_parameters.initial_task_frame[4],
                   force_mode_parameters.initial_task_frame[5]);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not transform %s to robot base: %s",
                 req->task_frame.header.frame_id.c_str(), ex.what());
    return false;
  }

  // The selection vector dictates which axes the robot should be compliant along and around
  force_mode_parameters.initial_selection_vec[0] = req->selection_vector_x;
  force_mode_parameters.initial_selection_vec[1] = req->selection_vector_y;
  force_mode_parameters.initial_selection_vec[2] = req->selection_vector_z;
  force_mode_parameters.initial_selection_vec[3] = req->selection_vector_rx;
  force_mode_parameters.initial_selection_vec[4] = req->selection_vector_ry;
  force_mode_parameters.initial_selection_vec[5] = req->selection_vector_rz;

  // The wrench parameters dictate the amount of force/torque the robot will apply to its environment. The robot will
  // move along/around compliant axes to match the specified force/torque. Has no effect for non-compliant axes.
  force_mode_parameters.initial_wrench = req->wrench;

  /* The limits specifies the maximum allowed speed along/around compliant axes. For non-compliant axes this value is
   * the maximum allowed deviation between actual tcp position and the one that has been programmed. */
  force_mode_parameters.limits[0] = req->selection_vector_x ? req->speed_limits.linear.x : req->deviation_limits[0];
  force_mode_parameters.limits[1] = req->selection_vector_y ? req->speed_limits.linear.y : req->deviation_limits[1];
  force_mode_parameters.limits[2] = req->selection_vector_z ? req->speed_limits.linear.z : req->deviation_limits[2];
  force_mode_parameters.limits[3] = req->selection_vector_rx ? req->speed_limits.angular.x : req->deviation_limits[3];
  force_mode_parameters.limits[4] = req->selection_vector_ry ? req->speed_limits.angular.y : req->deviation_limits[4];
  force_mode_parameters.limits[5] = req->selection_vector_rz ? req->speed_limits.angular.z : req->deviation_limits[5];

  if (req->type < 1 || req->type > 3) {
    RCLCPP_ERROR(get_node()->get_logger(), "The force mode type has to be 1, 2, or 3. Received %u", req->type);
    return false;
  }

  /* The type decides how the robot interprets the force frame (the one defined in task_frame). See ur_script manual
   * for explanation, under dynamic_force_mode. */
  force_mode_parameters.type = static_cast<double>(req->type);

  /* The damping factor decides how fast the robot decelarates if no force is present. 0 means no deceleration, 1
   * means quick deceleration*/
  if (req->damping_factor < 0.0 || req->damping_factor > 1.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "The damping factor has to be between 0 and 1. Received %f",
                 req->damping_factor);
    return false;
  }
  force_mode_parameters.damping_factor = req->damping_factor;

  /*The gain scaling factor scales the force mode gain. A value larger than 1 may make force mode unstable. */
  if (req->gain_scaling < 0.0 || req->gain_scaling > 2.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "The gain scaling has to be between 0 and 2. Received %f",
                 req->gain_scaling);
    return false;
  }
  if (req->gain_scaling > 1.0) {
    RCLCPP_WARN(get_node()->get_logger(),
                "A gain_scaling >1.0 can make force mode unstable, e.g. in case of collisions or pushing against "
                "hard surfaces. Received %f",
                req->gain_scaling);
  }
  force_mode_parameters.gain_scaling = req->gain_scaling;

  force_mode_params_buffer_.writeFromNonRT(force_mode_parameters);

  return true;
}

bool DynamicPathForceModeController::disableForceMode()
{
  force_mode_active_ = false;
  change_requested_ = true;

  if (!waitForAsyncCommand(
          [&]() { return command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_ASYNC_SUCCESS].get_value(); })) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not verify that dynamic force mode was set. (This might happen when "
                                          "using the "
                                          "mocked interface)");
  }

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

  // If we have not rejected the goal, attempt to set force mode
  ForceModeRequest req;
  req.task_frame = goal->task_frame;
  // selection vector defaults to null, so no additional assignment
  req.type = goal->type;
  req.speed_limits = goal->speed_limits;
  req.deviation_limits = goal->deviation_limits;
  req.damping_factor = goal->damping_factor;
  req.gain_scaling = goal->gain_scaling;

  if (!setForceMode(&req)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Unable to set force mode.");
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

    disableForceMode();
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

  current_index_ = 0;
  initial_time_ = time_to_double(goal_handle->get_goal()->force_mode_path.poses[0].header.stamp);

  // Activate force mode and make change request
  force_mode_active_ = true;
  change_requested_ = true;

  // Wait for change to occur
  if (!waitForAsyncCommand(
          [&]() { return command_interfaces_[CommandInterfaces::DYNAMIC_FORCE_MODE_ASYNC_SUCCESS].get_value(); })) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not verify that dynamic force mode was set. (This might happen when "
                                          "using the "
                                          "mocked interface)");
  }

  rt_active_goal_.writeFromNonRT(rt_goal);

  return;
}

void DynamicPathForceModeController::end_goal()
{
  disableForceMode();
  transfer_command_interface_->get().set_value(TRANSFER_STATE_IDLE);
}

}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::DynamicPathForceModeController, controller_interface::ControllerInterface)
