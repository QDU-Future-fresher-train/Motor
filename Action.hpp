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

#include "MOTOR.hpp"
#include "app_framework.hpp"
#include "app_main.h"
#include "can.hpp"
#include "cdc_uart.hpp"
#include "flash_map.hpp"
#include "libxr.hpp"
#include "libxr_def.hpp"
#include "main.h"
#include "pid.hpp"
#include "stm32_adc.hpp"
#include "stm32_can.hpp"
#include "stm32_canfd.hpp"
#include "stm32_dac.hpp"
#include "stm32_flash.hpp"
#include "stm32_gpio.hpp"
#include "stm32_i2c.hpp"
#include "stm32_power.hpp"
#include "stm32_pwm.hpp"
#include "stm32_spi.hpp"
#include "stm32_timebase.hpp"
#include "stm32_uart.hpp"
#include "stm32_usb_dev.hpp"
#include "stm32_watchdog.hpp"
#include "thread.hpp"


class Action : public LibXR::Application {
 public:
  Action(
      LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
      const LibXR::PID<float>::Param &params, motor *motor)
      : pid(params), motor_app(motor) {
    // Hardware initialization example:
    // auto dev = hw.template Find<LibXR::GPIO>("led");

    thread.Create(this, &Action::threadfunc, "action_thread", 2048,
                  LibXR::Thread::Priority::HIGH);

  }

  void OnMonitor() override {}

  void control_loop(int target_value) { this->target_value = target_value; }

 private:
  LibXR::MillisecondTimestamp last_time_;
  LibXR::MicrosecondTimestamp last_update_time_;
  float dt_ = 0.0f;

  LibXR::PID<float> pid;
  LibXR::CAN *can;
  LibXR::Thread thread;
  float target_value;
  motor *motor_app;

  struct Feedback {
    float abs_angle;
    float omega;
    float torque;
    float temp;
    uint8_t state;
  };
  Feedback feedback_;

  static void threadfunc(Action *action) {
    float target = 0.0f;
    action->last_update_time_ = LibXR::Timebase::GetMicroseconds();
    action->last_time_ = LibXR::Timebase::GetMilliseconds();
    LibXR::Thread::Sleep(10);
    action->Update();

    target=action->feedback_.abs_angle +
                         45.0f * 3.1415926f / 180.0f;
    while (true) {
      action->Update();
      action->control(target);
      LibXR::Thread::Sleep(10);
    }
  }

  void Update() {
    motor_app->Update();
    auto now = LibXR::Timebase::GetMicroseconds();
    dt_ = (now - last_update_time_).ToSecondf();
    last_update_time_ = now;
    feedback_.abs_angle = motor_app->feedback_.abs_angle;
    feedback_.omega = motor_app->feedback_.omega;
    feedback_.torque = motor_app->feedback_.torque;
    feedback_.temp = motor_app->feedback_.temp;
    feedback_.state = motor_app->feedback_.state;
  }

  void control(float out) {
    float target = pid.Calculate(out, feedback_.abs_angle, dt_);
    motor_app->control(target);
  }
};
