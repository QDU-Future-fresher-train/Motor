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
#include "app_framework.hpp"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include "Motor.hpp"
#include "app_framework.hpp"
#include "can.hpp"
#include "cycle_value.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "mutex.hpp"
#include "ramfs.hpp"
#include "thread.hpp"
#include "timebase.hpp"

#define GM6020_CON_V_id1 0x1FF
#define GM6020_CON_V_id2 0x2FF
#define GM6020__CON_A_id1 0x1FE
#define GM6020__CON_A_id2 0x2FE
#define GM6020_FB_id1 0x204
#define GM6020_CON_A_MAX 16384
#define GM6020_CON_V_MAX 25000
class motor : public LibXR::Application {
 public:
  enum CONTROL_Type {
    CONTROL_V = 0,
    CONTROL_A = 1,
  };


  struct ConfigParam {
    uint16_t id_control;
    uint16_t id_feedback;
    uint32_t max_control_value;
    uint8_t can_id;
    uint8_t id;
  };

  motor(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
        const ConfigParam& param, LibXR::CAN* can)
      : can(can), param(param) {
    // Hardware initialization example:
    // auto dev = hw.template Find<LibXR::GPIO>("led");
    /*if (param.control_type == CONTROL_Type::CONTROL_V) {
      config_param.max_control_value = GM6020_CON_V_MAX;
      if (param.id >= 1 && param.id <= 4) {
        config_param.id_control = GM6020_CON_V_id1;
        config_param.id_feedback = GM6020_FB_id1 + param.id;
        config_param.can_id = 0;

      } else if (param.id <= 7 && param.id > 4) {
        config_param.id_control = GM6020_CON_V_id2;
        config_param.id_feedback = GM6020_FB_id1 + param.id;
        config_param.can_id = 1;

      } else {
        return;
      }
    } else if (param.control_type == CONTROL_Type::CONTROL_A) {
      config_param.max_control_value = GM6020_CON_A_MAX;
      if (param.id >= 1 && param.id <= 4) {
        config_param.id_control = GM6020__CON_A_id1;
        config_param.id_feedback = GM6020_FB_id1 + param.id;
        config_param.can_id = 0;

      } else if (param.id <= 7 && param.id > 4) {
        config_param.id_control = GM6020__CON_A_id2;
        config_param.id_feedback = GM6020_FB_id1 + param.id;
        config_param.can_id = 1;

      } else {
        return;
      }  // 根据传入的电机参数，定义电机的控制、反馈ID
    }
*/
    auto rx_callback = LibXR::CAN::Callback::Create(
        [](bool in_isr, motor* self, const LibXR::CAN::ClassicPack& pack) {
          RxCallback(in_isr, self, pack);
        },
        this);

    can->Register(rx_callback, LibXR::CAN::Type::STANDARD,
                  LibXR::CAN::FilterMode::ID_RANGE, 0x20A,
                  0x20A);
    this->param = param;
    if(param.id>4){
      this->param.id-=5;
    }
  }

  void OnMonitor() override {}

 private:
  static inline uint8_t motor_tx_buff_[2][4][8]{};
  static inline uint8_t motor_tx_pending_mask_[2][4]{};
  static inline uint8_t motor_group_mask_[2][4]{};

  LibXR::CAN* can;
  LibXR::LockFreeQueue<LibXR::CAN::ClassicPack> recv_queue_{1};
  int text=0;
  ConfigParam param;

  static void RxCallback(bool in_isr, motor* self,
                         const LibXR::CAN::ClassicPack& pack) {
    (in_isr);
    while (self->recv_queue_.Push(pack) != ErrorCode::OK) {
      self->recv_queue_.Pop();
    }
  }
  void SendData(const LibXR::CAN::ClassicPack& tx_pack) {
    can->AddMessage(tx_pack);
  }

  void PackAndSend(int ctrl_cmd) {
    LibXR::CAN::ClassicPack tx_pack{};
    motor_tx_buff_[0][1][2] =
        static_cast<uint8_t>((ctrl_cmd >> 8) & 0xFF);
    motor_tx_buff_[0][1][3] =
        static_cast<uint8_t>(ctrl_cmd & 0xFF);
    tx_pack.id = 0x2FF;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    tx_pack.dlc = 8;
    LibXR::Memory::FastCopy(tx_pack.data,
                            motor_tx_buff_[0][1],
                            sizeof(tx_pack.data));
    motor_tx_pending_mask_[0][param.id] = 0U;
    SendData(tx_pack);
  }

 public:
  struct Feedback {
    float abs_angle;
    float omega;
    float torque;
    float temp;
    uint8_t state;
  };
  Feedback feedback_;

  void Decode(LibXR::CAN::ClassicPack& pack) {
    uint16_t raw_angle =
        static_cast<uint16_t>((pack.data[0] << 8) | pack.data[1]);
    int16_t raw_velocity =
        static_cast<int16_t>((pack.data[2] << 8) | pack.data[3]);
    int16_t raw_current =
        static_cast<int16_t>((pack.data[4] << 8) | pack.data[5]);
    uint8_t raw_temp = pack.data[6];
    feedback_.abs_angle = static_cast<float>(raw_angle)/8191.0f*2.0f*3.1415926f;

    feedback_.omega = feedback_.omega * (static_cast<float>(M_2PI) / 60.0f);

    feedback_.torque = 0;

    feedback_.temp = static_cast<float>(raw_temp);

    feedback_.state = 1;
  }

 public:
  void Update() {
    LibXR::CAN::ClassicPack pack;
    while (recv_queue_.Pop(pack) == ErrorCode::OK) {
      Decode(pack);
    }
  }
  void control(float out) {
    int16_t ctrl_cmd = static_cast<int16_t>(out*20000.0f);
    ctrl_cmd = std::clamp(ctrl_cmd, static_cast<int16_t>(-25000),
                         static_cast<int16_t>(25000));
    PackAndSend(ctrl_cmd);
  }
};
