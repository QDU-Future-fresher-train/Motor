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
#include <cstdint>
#include <cmath>
#define SIMULATION_MODE   // 开启模拟模式（无电机时用）
#include "app_framework.hpp"
#include "can.hpp"
#define GM6020_FB_ID_BASE (0x205)
#define GM6020_FB_ID_EXTAND (0x209)
#define GM6020_CTRL_ID_BASE (0x1fe)
#define GM6020_CTRL_ID_EXTAND (0x2fe)
#define GM6020_MAX_ABS_CUR (3)
#define MOTOR_ENC_RES (8192)  /* 电机编码器分辨率 */
#define MOTOR_CUR_RES (16384) /* 电机转矩电流分辨率 */

class motor : public LibXR::Application {

public:
enum class ControlMode : uint8_t {
    MODE_NONE    = 0,
    MODE_CURRENT,     // 电流控制模式
    MODE_TORQUE,      // 转矩控制模式
    MODE_VELOCITY,    // 速度控制模式（可后续扩展）
    MODE_POSITION     // 位置控制模式（可后续扩展）
};

struct MotorCmd {
    ControlMode mode = ControlMode::MODE_NONE;   // 控制模式
    float velocity = 0.0f;      // 在电流模式下 = 电流值（A）
    float torque   = 0.0f;      // 在转矩模式下 = 扭矩值（N·m）
    float reduction_ratio = 1.0f;  // 减速比（可选）
};
    struct Param {
    bool reverse;//电机反转
    uint16_t feedback_id;
    const char* can_bus_name;
    struct PID {
        float kp = 10.0f;     // 比例（建议 8~12）
        float ki = 0.08f;     // 积分（建议 0.05~0.15）
        float kd = 0.8f;      // 微分（建议 0.5~1.5）
        float integral = 0.0f;
        float last_error = 0.0f;

        float Compute(float error, float dt) {
            integral += error * dt;
            float deriv = (error - last_error) / dt;
            last_error = error;
            return kp * error + ki * integral + kd * deriv;
        }
  };
  motor(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,const Param& param)
  : param_(param),
        can_(hw.template FindOrExit<LibXR::CAN>({param_.can_bus_name}))
         {
        UNUSED(app);
        reverse_flag_ = param_.reverse ? -1.0f : 1.0f;
        if (param_.feedback_id >= 0x205 && param_.feedback_id <= 0x208) {
          config_param_.id_control = GM6020_CTRL_ID_BASE;
          config_param_.id_feedback = param_.feedback_id;
        } else if (param_.feedback_id >= 0x209 && param_.feedback_id <= 0x20B) {
          config_param_.id_control = GM6020_CTRL_ID_EXTAND;
          config_param_.id_feedback = param_.feedback_id;
        } else {
          config_param_.id_control = 0;
          config_param_.id_feedback = 0;
        }
    // Hardware initialization example:
    // auto dev = hw.template Find<LibXR::GPIO>("led");
  auto rx_callback = LibXR::CAN::Callback::Create(
    [](bool in_isr, motor* self, const LibXR::CAN::ClassicPack& pack) {
      RxCallback(in_isr, self, pack);
    },
    this);
  can_->Register(rx_callback,
          LibXR::CAN::Type::STANDARD,
          LibXR::CAN::FilterMode::ID_RANGE,
          config_param_.id_feedback,
          config_param_.id_feedback);  // 精确匹配单个 ID
          float current_deg = GetFeedback().position * 180.0f / M_PI;
        SetTargetPosition(current_deg + 45.0f);   // 一上电就执行 45° 阶跃
}
  float target_pos_  = 0.0f;   // 目标角度（度）
  float current_pos_ = 0.0f;   // 当前角度（度）← Ozone 重点抓这个
  void SetTargetPosition(float deg) 
  {target_pos_ = deg;}
  void Enable() override { return; }
  void Disable() override { CurrentControl(0.0f); }
  void Relax() override { this->CurrentControl(0.0f); }

   ErrorCode Update() override {
    LibXR::CAN::ClassicPack pack;
    while (recv_queue_.Pop(pack) == ErrorCode::OK) {
      this->Decode(pack);
    }
    float error = target_pos_ - current_pos_;
    float pid_out = pid_.Compute(error, 0.001f);          // dt = 1ms
    pid_out = std::clamp(pid_out, -GM6020_MAX_CUR, GM6020_MAX_CUR);
    CurrentControl(pid_out);   // 输出电流指令
    return ErrorCode::OK;
  }

  const Feedback& GetFeedback() override { return feedback_; }
  void Control(const MotorCmd& cmd) override {
        if (cmd.mode == ControlMode::MODE_CURRENT) {
            CurrentControl(cmd.velocity);
        } else if (cmd.mode == ControlMode::MODE_TORQUE) {
            CurrentControl(cmd.torque);
        }
    }
  void ClearError() override { }
  void SaveZeroPoint() override { }
  void OnMonitor() override {}

private:
Param param_;
    struct {
        uint32_t id_feedback;
        uint32_t id_control;
    } config_param_;

    float reverse_flag_ = 1.0f;
    Motor::Feedback feedback_;
    PID pid_;

    LibXR::CAN* can_;
    LibXR::LockFreeQueue<LibXR::CAN::ClassicPack> recv_queue_{1};
    static void RxCallback(bool in_isr, motor* self,
                           const LibXR::CAN::ClassicPack& pack) {
        UNUSED(in_isr);
        while (self->recv_queue_.Push(pack) != ErrorCode::OK) {
            self->recv_queue_.Pop();  // 队列满时丢弃最早的数据
        }
    }

    void Decode(const LibXR::CAN::ClassicPack& pack) {
        uint16_t raw_angle   = (pack.data[0] << 8) | pack.data[1];
        int16_t  raw_speed   = (pack.data[2] << 8) | pack.data[3];
        int16_t  raw_current = (pack.data[4] << 8) | pack.data[5];
        uint8_t  raw_temp    = pack.data[6];
        float pos_rad = static_cast<float>(raw_angle) / MOTOR_ENC_RES * static_cast<float>(M_2PI);
       if (param_.reverse) pos_rad = -pos_rad;

        feedback_.position = pos_rad;
        current_pos_ = pos_rad * 180.0f / M_PI;   // 转为度，供 Ozone 画图

        feedback_.velocity = static_cast<float>(raw_speed) * (param_.reverse ? -1.0f : 1.0f);
        feedback_.torque   = static_cast<float>(raw_current) * 0.741f * GM6020_MAX_CUR / MOTOR_CUR_RES;
        feedback_.temp     = static_cast<float>(raw_temp);
        feedback_.state    = 1;
//#ifdef SIMULATION_MODE   // ← 加上这个宏开关
  //      static float sim_pos = 0.0f;
    //    sim_pos += (target_pos_ - sim_pos) * 0.15f;   // 模拟电机响应（可调）
      //  current_pos_ = sim_pos;
        //feedback_.position = sim_pos * M_PI / 180.0f;
    #endif
    }
    void CurrentControl(float current_A) {
        if (feedback_.temp > 75.0f) current_A = 0.0f;

        int16_t cmd = static_cast<int16_t>(current_A / GM6020_MAX_CUR * MOTOR_CUR_RES * reverse_flag_);

        LibXR::CAN::ClassicPack tx{};
        tx.id   = config_param_.id_control;
        tx.type = LibXR::CAN::Type::STANDARD;
        tx.dlc  = 8;
        tx.data[0] = (cmd >> 8) & 0xFF;
        tx.data[1] = cmd & 0xFF;

        can_->AddMessage(tx);
    }
  }
};
