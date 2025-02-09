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
/*!\file
 *
 * \author  George Kouretas gkouretas@scu.edu
 * \date    2025-02-01
 */
//----------------------------------------------------------------------

#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_server_goal_handle.h>

#include <array>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/clock.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ur_msgs/srv/set_force_mode.hpp>
#include <ur_msgs/action/dynamic_force_mode_path.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include "dynamic_path_force_mode_controller_parameters.hpp"

namespace ur_controllers
{
const double TRANSFER_STATE_IDLE = 0.0;
const double TRANSFER_WAITING_FOR_POINT = 1.0;
const double TRANSFER_STATE_IN_MOTION = 2.0;
const double TRANSFER_STATE_DONE = 3.0;

using namespace std::chrono_literals;  // NOLINT

enum CommandInterfaces
{
  DYNAMIC_FORCE_MODE_TASK_FRAME_X = 0u,
  DYNAMIC_FORCE_MODE_TASK_FRAME_Y = 1,
  DYNAMIC_FORCE_MODE_TASK_FRAME_Z = 2,
  DYNAMIC_FORCE_MODE_TASK_FRAME_RX = 3,
  DYNAMIC_FORCE_MODE_TASK_FRAME_RY = 4,
  DYNAMIC_FORCE_MODE_TASK_FRAME_RZ = 5,
  DYNAMIC_FORCE_MODE_SELECTION_VECTOR_X = 6,
  DYNAMIC_FORCE_MODE_SELECTION_VECTOR_Y = 7,
  DYNAMIC_FORCE_MODE_SELECTION_VECTOR_Z = 8,
  DYNAMIC_FORCE_MODE_SELECTION_VECTOR_RX = 9,
  DYNAMIC_FORCE_MODE_SELECTION_VECTOR_RY = 10,
  DYNAMIC_FORCE_MODE_SELECTION_VECTOR_RZ = 11,
  DYNAMIC_FORCE_MODE_WRENCH_X = 12,
  DYNAMIC_FORCE_MODE_WRENCH_Y = 13,
  DYNAMIC_FORCE_MODE_WRENCH_Z = 14,
  DYNAMIC_FORCE_MODE_WRENCH_RX = 15,
  DYNAMIC_FORCE_MODE_WRENCH_RY = 16,
  DYNAMIC_FORCE_MODE_WRENCH_RZ = 17,
  DYNAMIC_FORCE_MODE_TYPE = 18,
  DYNAMIC_FORCE_MODE_LIMITS_X = 19,
  DYNAMIC_FORCE_MODE_LIMITS_Y = 20,
  DYNAMIC_FORCE_MODE_LIMITS_Z = 21,
  DYNAMIC_FORCE_MODE_LIMITS_RX = 22,
  DYNAMIC_FORCE_MODE_LIMITS_RY = 23,
  DYNAMIC_FORCE_MODE_LIMITS_RZ = 24,
  DYNAMIC_FORCE_MODE_ASYNC_SUCCESS = 25,
  DYNAMIC_FORCE_MODE_DISABLE_CMD = 26,
  DYNAMIC_FORCE_MODE_DAMPING = 27,
  DYNAMIC_FORCE_MODE_GAIN_SCALING = 28,
};
enum StateInterfaces
{
  INITIALIZED_FLAG = 0u,
};

struct DynamicForceModeParameters
{
  std::array<double, 6> initial_task_frame;
  std::array<double, 6> initial_selection_vec;
  std::array<double, 6> dynamic_task_frame;
  std::array<double, 6> dynamic_selection_vec;
  std::array<double, 6> limits;
  geometry_msgs::msg::Wrench initial_wrench;
  geometry_msgs::msg::Wrench dynamic_wrench;
  double type;
  double damping_factor;
  double gain_scaling;
};

// HACK(george): should use a msg type for this...
struct ForceModeRequest
{
  geometry_msgs::msg::PoseStamped task_frame;
  bool selection_vector_x = false;
  bool selection_vector_y = false;
  bool selection_vector_z = false;
  bool selection_vector_rx = false;
  bool selection_vector_ry = false;
  bool selection_vector_rz = false;
  uint8_t type = 2;
  geometry_msgs::msg::Wrench wrench;
  geometry_msgs::msg::Twist speed_limits;
  std::array<float, 6> deviation_limits = { 0.01, 0.01, 0.01, 0.01, 0.01, 0.01 };
  float damping_factor = 0.025;
  float gain_scaling = 0.5;
};

// TODO(george): inherit from `force_mode_controller.cpp`???
// for now, just making independent...
class DynamicPathForceModeController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_init() override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

private:
  double time_from_start(const builtin_interfaces::msg::Time& time) const;
  bool setForceMode(const ForceModeRequest* req);
  bool disableForceMode(void);
  rclcpp::Service<ur_msgs::srv::SetForceMode>::SharedPtr set_force_mode_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_force_mode_srv_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<dynamic_path_force_mode_controller::ParamListener> param_listener_;
  dynamic_path_force_mode_controller::Params params_;

  realtime_tools::RealtimeBuffer<DynamicForceModeParameters> force_mode_params_buffer_;
  std::atomic<bool> force_mode_active_;
  std::atomic<bool> change_requested_;
  std::atomic<double> async_state_;

  tf2::Transform pose_desired_;
  tf2::Transform pose_actual_;
  geometry_msgs::msg::Twist pose_error_;

  static constexpr double ASYNC_WAITING = 2.0;
  /**
   * @brief wait until a command interface isn't in state ASYNC_WAITING anymore or until the parameter maximum_retries
   * have been reached
   */
  bool waitForAsyncCommand(std::function<double(void)> get_value);

  /* Trajectory stuff */
  using DynamicForceModeAction = ur_msgs::action::DynamicForceModePath;
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<DynamicForceModeAction>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;
  using RealtimeGoalHandleBuffer = realtime_tools::RealtimeBuffer<RealtimeGoalHandlePtr>;

  RealtimeGoalHandleBuffer rt_active_goal_;         ///< Currently active action goal, if any.
  rclcpp::TimerBase::SharedPtr goal_handle_timer_;  ///< Timer to frequently check on the running goal
  rclcpp::Duration action_monitor_period_ = rclcpp::Duration(50ms);

  void initialize_force_mode();
  void update_trajectory_points(std::shared_ptr<RealtimeGoalHandle> active_goal);
  void update_pose_actual_desired(std::shared_ptr<RealtimeGoalHandle> active_goal);
  bool find_pose_desired(void);
  tf2::Transform interpolate_poses(geometry_msgs::msg::Pose& t1, geometry_msgs::msg::Pose& t2, double factor);
  bool check_pose_tolerance(tf2::Transform &tf, std::array<float, 6> tolerances);
  void compute_compliance_vector(geometry_msgs::msg::Pose& t1, geometry_msgs::msg::Pose& t2);
  tf2::Transform compute_relative_transform(geometry_msgs::msg::Pose& t1, geometry_msgs::msg::Pose& t2);
  tf2::Transform compute_relative_transform(tf2::Transform& t1, tf2::Transform& t2);

  void end_goal();
  std::shared_ptr<dynamic_path_force_mode_controller::ParamListener> dynamic_force_mode_listener_;
  dynamic_path_force_mode_controller::Params dynamic_force_mode_params_;

  rclcpp_action::Server<DynamicForceModeAction>::SharedPtr dynamic_force_mode_action_server_;

  rclcpp_action::GoalResponse goal_received_callback(const rclcpp_action::GoalUUID& uuid,
                                                     std::shared_ptr<const DynamicForceModeAction::Goal> goal);

  rclcpp_action::CancelResponse
  goal_cancelled_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<DynamicForceModeAction>> goal_handle);

  void goal_accepted_callback(std::shared_ptr<rclcpp_action::ServerGoalHandle<DynamicForceModeAction>> goal_handle);

  realtime_tools::RealtimeBuffer<std::vector<std::string>> joint_names_;
  std::vector<std::string> state_interface_types_;

  nav_msgs::msg::Path active_path_;
  std::atomic<size_t> current_index_;
  std::atomic<double> initial_time_;
  std::atomic<bool> path_active_;
  rclcpp::Duration active_path_elapsed_time_ = rclcpp::Duration::from_nanoseconds(0);
  rclcpp::Duration max_path_trajectory_time_ = rclcpp::Duration::from_nanoseconds(0);
  double scaling_factor_;
  static constexpr double NO_VAL = std::numeric_limits<double>::quiet_NaN();

  std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> scaling_state_interface_;
  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> abort_command_interface_;
  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> transfer_command_interface_;
  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> time_from_start_command_interface_;

  rclcpp::Clock::SharedPtr clock_;
};
}  // namespace ur_controllers
