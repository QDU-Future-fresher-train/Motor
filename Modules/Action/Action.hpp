#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: IMU interface module
constructor_args: []
template_args: []
required_hardware:
  - imu
  - scl
  - sda
depends: []
=== END MANIFEST === */
// clang-format on

#include "app_framework.hpp"
#include "pid.h"
#include "RMMotor.hpp"
#define task_stack_depth 1024*1024*50
#define TARGET_ANGLE M_PI_4;

LibXR::Thread thread_;
RMMotor* motor;
pid_struct_t pid;
const MotorCmd cmd;

static void ThreadFunc(RMMotor* motor) {
  pid_init(&pid, 30.0f, 0.0f, 0.0f, 300.0f, 300.0f);

  while (true) {
    feedback_ = motor->GetFeedback();
    float pid_out = pid_calc(&pid, TARGET_ANGLE, feedback_.abs_angle);
    cmd.mode = ControlMode::MODE_CURRENT;
    motor->Control(cmd);
    LibXR::Thread::Sleep(3); 
  }

}

class Action : public LibXR::Application {
public:
  Action(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app) {
    // Hardware initialization example:
    // auto dev = hw.template Find<LibXR::GPIO>("led");
    thread_.Create(this, ThreadFunc, "ActionThread", task_stack_depth,
               LibXR::Thread::Priority::MEDIUM);
  }
  
  void OnMonitor() override {}
  
private:
};
