#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: IMU interface module
constructor_args:
  - reverse: false
  - feedback_id: 0x209
  - can_bus_name: "can1"
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
#include <sys/types.h>
#define SIMULATION_MODE   // 开启模拟模式（无电机时用）
#include "app_framework.hpp"
#include "can.hpp"
#include "pid.hpp"

struct Feedback {
    float position = 0.0f;   // 位置 (rad)
    float velocity = 0.0f;   // 速度
    float torque = 0.0f;     // 扭矩
    float temp = 0.0f;       // 温度
    uint8_t state = 0;       // 状态
};

#define GM6020_FB_ID_BASE (0x205)
#define GM6020_FB_ID_EXTAND (0x209)
#define GM6020_CTRL_ID_BASE (0x1ff)
#define GM6020_CTRL_ID_EXTAND (0x2ff)
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
      //LibXR::PID<float> pid_;  // ← 新增 PID 参数结构体
  };
  motor(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
        const Param& param)
      : param_(param),
        can_(hw.template FindOrExit<LibXR::CAN>({param_.can_bus_name})),
        pid_(LibXR::PID<float>::Param{
            // ← 必须这样写
            .k = 1.0f,
            .p = 1.0f,         // 比例
            .i = 0.0f,         // 积分
            .d = 0.0f,          // 微分
            .i_limit = 8.0f,    // 积分限幅
            .out_limit = 3.0f,  // 输出电流限幅 ±3A
            .cycle = true       // 开启周期角度处理（位置环强烈推荐）
        }) {
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
    [](bool in_isr, motor* self, const LibXR::CAN::ClassicPack& pack)  {
      RxCallback(in_isr, self, pack);
    },
    this);
  can_->Register(rx_callback,
          LibXR::CAN::Type::STANDARD,
          LibXR::CAN::FilterMode::ID_RANGE,
          config_param_.id_feedback,
          config_param_.id_feedback);  // 精确匹配单个 ID
  }

void Initialize() {
    float current_deg = GetFeedback().position ;
    SetTargetPosition(current_deg + 45.0f);   // 一上电就执行 45° 阶跃
}

float target_pos_  = 0.0f;   // 目标角度（度）
float current_pos_ = 0.0f;   // 当前角度（度）← Ozone 重点抓这个

void SetTargetPosition(float deg) {
  target_pos_ = deg;
}
int test = 0;
void Enable() { return; }
  void Disable() { VoltageControl(0.0f); }
  void Relax(){ this->VoltageControl(0.0f); }

   ErrorCode Update(){
    LibXR::CAN::ClassicPack pack;
    while (recv_queue_.Pop(pack) == ErrorCode::OK) {
      this->Decode(pack);
    }
    float pid_out = pid_.Calculate(target_pos_, current_pos_, 0.001f);
    //pid_out = std::clamp(pid_out, -static_cast<float>(GM6020_MAX_ABS_CUR), static_cast<float>(GM6020_MAX_ABS_CUR));
    VoltageControl(pid_out*8000.0f);  // 输出电流指令
    test++;
    return ErrorCode::OK;
  }

  const Feedback& GetFeedback() { return feedback_; }
  void Control(const MotorCmd& cmd)  {
        if (cmd.mode == ControlMode::MODE_CURRENT) {
            VoltageControl(cmd.velocity);
        } else if (cmd.mode == ControlMode::MODE_TORQUE) {
            VoltageControl(cmd.torque);
        }
    }
  void ClearError()  { }
  void SaveZeroPoint()  { }
  void OnMonitor()  {}

private:
 //LibXR::Thread motor_thread_;

 Param param_;
 struct {
   uint32_t id_feedback;
   uint32_t id_control;
    } config_param_;

    float reverse_flag_ = 1.0f;
    Feedback feedback_;


    LibXR::CAN* can_;
    LibXR::PID<float> pid_;  // ← 新增 PID 参数结构体
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
        current_pos_ = pos_rad ;   // 转为度，供 Ozone 画图

        feedback_.velocity = static_cast<float>(raw_speed) * (param_.reverse ? -1.0f : 1.0f);
        feedback_.torque   = static_cast<float>(raw_current) * 0.741f * GM6020_MAX_ABS_CUR / MOTOR_CUR_RES;
        feedback_.temp     = static_cast<float>(raw_temp);
        feedback_.state    = 1;


    }
    int TEST_VAR = 0;
    void VoltageControl(float voltage_cmd) {  // voltage_cmd 范围建议 -20000 ~ 20000
      if (feedback_.temp > 75.0f) voltage_cmd = 0.0f;

      // 限幅（电压控制的合理范围）
      voltage_cmd = std::clamp(voltage_cmd, -20000.0f, 20000.0f);

      int16_t cmd = static_cast<int16_t>(voltage_cmd * reverse_flag_);

      LibXR::CAN::ClassicPack tx{};
      tx.id = config_param_.id_control;  // 0x2FF
      tx.type = LibXR::CAN::Type::STANDARD;
      tx.dlc = 8;

      tx.data[2] = (cmd >> 8) & 0xFF;
      tx.data[3] = cmd & 0xFF;
      TEST_VAR ++;
      can_->AddMessage(tx);
    }
};
