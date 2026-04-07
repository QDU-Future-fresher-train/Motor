
#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: GM6020 Motor Action Loop
constructor_args: []
template_args: []
required_hardware:
  - can
depends: []
=== END MANIFEST === */
// clang-format on

// 依赖的头文件
#include <sys/syslimits.h>
#include <cmath>
#include "app_framework.hpp"
#include "motor.hpp"
#include"pid.hpp"
#include "thread.hpp"

    // GM6020 电机动作循环类
    float target;  // 目标位置（度）
    class action : public LibXR::Application {
 public:
  /**
   * 构造函数
   * @param hw 硬件容器引用
   * @param app 应用管理器引用
   * @param motor_1 要控制的电机实例指针
   */
  action(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
         motor* motor_1,LibXR::PID<float>::Param speed_pid_param, LibXR::PID<float>::Param position_pid_param)
      : motor_2_(motor_1), speed_pid_(speed_pid_param), position_pid_(position_pid_param){  // 保存电机指针和PID参数
    UNUSED(hw);
    UNUSED(app);
      LibXR::Thread::Sleep(100);  // 等待系统稳定

    motor_2_->Update();

    target = 45.0f +motor_2_->current_pos_;


    thread_.Create(this, ThreadFunc, "actionThread", 2048,
                   LibXR::Thread::Priority::MEDIUM);



  }

  /**
   * 监控接口（当前为空实现）
   */
  void OnMonitor() override {}

  // 成员变量


 private:
  static constexpr uint32_t TASK_STACK_DEPTH = 2048;  // 任务栈深度定义
  motor* motor_2_;                                   // 电机控制对象指针

  /**
   * 线程执行函数
   * @param self 指向当前action实例的指针
   */
  static void ThreadFunc(action* self) {
    constexpr float kControlPeriod = 0.01f;     // 10ms
    constexpr float kPositionTolerance = 5.0f;  // 0.5度容差
    while (true) {

      // 更新电机控制循环
      self->motor_2_->Update();
      self->feedback_= self->motor_2_->GetFeedback();  // 获取电机反馈数据


      float error = target-self->motor_2_->current_pos_;  // 计算位置误差

      float voltage = self->position_pid_.Calculate(
          target*M_PI/180.0f , (self->motor_2_->current_pos_)*M_PI/180.0f,
          kControlPeriod);  // 计算控制电压

      voltage = std::clamp(voltage, -3.0f, 3.0f);  // 限制控制电压在 -24V ~ +24V 范围内

      self->motor_2_->VoltageControl(voltage);  // 输出控制电压

      if(std::abs(error) <= kPositionTolerance) {  // 如果位置误差在容差范围内
        LibXR::Thread::Sleep(1000);
        target += 45.0f;  // 更新目标位置，执行下一个阶跃
      }
      // 1ms控制周期
      LibXR::Thread::Sleep(10);
    }
  }
    LibXR::Thread thread_;  // 控制线程
  LibXR::PID<float> speed_pid_;     // 速度PID参数
  LibXR::PID<float> position_pid_;  // 位置PID参数
  motor::Feedback feedback_;  // 电机反馈数据

};
