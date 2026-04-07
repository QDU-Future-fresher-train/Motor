#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: IMU interface module
constructor_args:
  - motor_: '@&motor'
  - speed_pid_param:
      k: 1.0
      p: 1.0
      i: 0.0
      d: 0.0
      i_limit: 0.1
      out_limit: 0.1
      cycle: true
  - position_pid_param:
      k: 1.0
      p: 1.0
      i: 0.0
      d: 0.0
      i_limit: 1.0
      out_limit: 1.0
      cycle: true
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "pid.hpp"
#include "thread.hpp"


class Action : public LibXR::Application {
 public:
  Action(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
         RMMotor* motor_, LibXR::PID<float>::Param speed_pid_param,
         LibXR::PID<float>::Param position_pid_param)
      : motor(motor_),
        pid_speed(speed_pid_param),
        pid_position(position_pid_param) {
    thread_.Create(this, ThreadFunc, "ActionThread", 2048,
                   LibXR::Thread::Priority::MEDIUM);
  }
  static void ThreadFunc(Action* action) {
    action->motor->Update();
    action->feedback = action->motor->GetFeedback();
    float start_pos = action->feedback.position;
    float target_position = start_pos + 0.7854f;
    while (true) {
      action->motor->Update();
      action->feedback = action->motor->GetFeedback();
      float pos_out = action->pid_position.Calculate(
          target_position, action->feedback.position, 0.002f);
      float tor_out =
          action->pid_speed.Calculate(pos_out, action->feedback.omega, 0.002f);
      Motor::MotorCmd cmd;
      cmd.mode = Motor::ControlMode::MODE_TORQUE;
      cmd.torque = tor_out;
      action->motor->Control(cmd);
      LibXR::Thread::Sleep(2);

    }
  }

  void OnMonitor() override {}

 private:
  LibXR::Thread thread_;
  RMMotor* motor;
  Motor::Feedback feedback;
  LibXR::PID<float> pid_speed;
  LibXR::PID<float> pid_position;
};
