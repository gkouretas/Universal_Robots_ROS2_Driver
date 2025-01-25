// Copyright 2024, FZI Forschungszentrum Informatik, Created on behalf of Universal Robots A/S
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
 * \author  Vincenzo Di Pentima dipentima@fzi.de
 * \date    2024-09-26
 */
//----------------------------------------------------------------------
#ifndef UR_CONTROLLERS__FREEDRIVE_MODE_CONTROLLER_HPP_
#define UR_CONTROLLERS__FREEDRIVE_MODE_CONTROLLER_HPP_

#pragma once

#include <realtime_tools/realtime_buffer.h>

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <limits>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include "std_msgs/msg/bool.hpp"

#include <ur_msgs/srv/set_freedrive_params.hpp>

#include "freedrive_mode_controller_parameters.hpp"

namespace ur_controllers
{
enum CommandInterfaces
{
  FREEDRIVE_MODE_ASYNC_SUCCESS = 0u,
  FREEDRIVE_MODE_ENABLE = 1,
  FREEDRIVE_MODE_ABORT = 2,
  FREEDRIVE_MODE_PARAMS_VECTOR_X = 3,
  FREEDRIVE_MODE_PARAMS_VECTOR_Y = 4,
  FREEDRIVE_MODE_PARAMS_VECTOR_Z = 5,
  FREEDRIVE_MODE_PARAMS_VECTOR_RX = 6,
  FREEDRIVE_MODE_PARAMS_VECTOR_RY = 7,
  FREEDRIVE_MODE_PARAMS_VECTOR_RZ = 8,
  FREEDRIVE_MODE_PARAMS_FEATURE_POSE_VECTOR_X = 9,
  FREEDRIVE_MODE_PARAMS_FEATURE_POSE_VECTOR_Y = 10,
  FREEDRIVE_MODE_PARAMS_FEATURE_POSE_VECTOR_Z = 11,
  FREEDRIVE_MODE_PARAMS_FEATURE_POSE_VECTOR_RX = 12,
  FREEDRIVE_MODE_PARAMS_FEATURE_POSE_VECTOR_RY = 13,
  FREEDRIVE_MODE_PARAMS_FEATURE_POSE_VECTOR_RZ = 14,
  FREEDRIVE_MODE_PARAMS_FEATURE_CONSTANT_BASE = 15,
  FREEDRIVE_MODE_PARAMS_FEATURE_CONSTANT_TOOL = 16
};

using namespace std::chrono_literals;  // NOLINT

class FreedriveModeController : public controller_interface::ControllerInterface
{
public:
  struct FreedriveModeParamaters
  {
    enum class FreedriveModeConstants
    {
      TOOL,
      BASE
    };

    std::optional<std::array<bool, 6>> free_axes_;
    std::optional<FreedriveModeConstants> feature_constant_;
    std::optional<std::array<double, 6>> feature_vector_;
  };

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // Change the input for the update function
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_init() override;

private:
  // Command interfaces: optional is used only to avoid adding reference initialization
  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> async_success_command_interface_;
  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> enable_command_interface_;
  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> abort_command_interface_;

  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> enable_freedrive_mode_sub_;

  rclcpp::Service<ur_msgs::srv::SetFreedriveParams>::SharedPtr set_freedrive_params_srv_;

  realtime_tools::RealtimeBuffer<FreedriveModeParamaters> freedrive_params_buffer_;

  rclcpp::TimerBase::SharedPtr freedrive_sub_timer_;  ///< Timer to check for timeout on input
  mutable std::chrono::seconds timeout_interval_;
  void freedrive_cmd_callback(const std_msgs::msg::Bool::SharedPtr msg);

  bool set_freedrive_params(const ur_msgs::srv::SetFreedriveParams::Request::SharedPtr req,
                            ur_msgs::srv::SetFreedriveParams::Response::SharedPtr resp);

  std::shared_ptr<freedrive_mode_controller::ParamListener> freedrive_param_listener_;
  freedrive_mode_controller::Params freedrive_params_;

  std::atomic<bool> freedrive_active_;
  std::atomic<bool> change_requested_;
  std::atomic<double> async_state_;
  std::atomic<double> first_log_;
  std::atomic<double> timer_started_;

  void start_timer();
  void timeout_callback();

  std::thread logging_thread_;
  std::atomic<bool> logging_thread_running_;
  std::atomic<bool> logging_requested_;
  std::condition_variable logging_condition_;
  std::mutex log_mutex_;
  void log_task();
  void start_logging_thread();
  void stop_logging_thread();

  static constexpr double ASYNC_WAITING = 2.0;
  static constexpr double NO_VAL = std::numeric_limits<double>::quiet_NaN();
  /**
   * @brief wait until a command interface isn't in state ASYNC_WAITING anymore or until the parameter maximum_retries
   * have been reached
   */
  bool waitForAsyncCommand(std::function<double(void)> get_value);
};
}  // namespace ur_controllers
#endif  // UR_CONTROLLERS__PASSTHROUGH_TRAJECTORY_CONTROLLER_HPP_
