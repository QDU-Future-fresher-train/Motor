/* === MODULE MANIFEST V2 ===
module_description: GM6020 motor driver
constructor_args:
  - param:
      name: feedback_id
      type: uint32_t
      description: CAN feedback ID of the motor
template_args: []
required_hardware:
  - can
depends: []
=== END MANIFEST === */

#pragma once

#include "can.hpp"
#include "PID.hpp"

class Motor {
public:
    Motor(LibXR::CAN* can, uint32_t feedback_id);
    void SetTargetPosition(float position_rad);
    float GetCurrentPosition() const;
    void Update();   // 由 TIM3 中断调用

private:
  static void CANRxCallback(bool in_isr, Motor* self, const LibXR::CAN::ClassicPack& pack);
    void ControlTask();

    LibXR::CAN* can_;
    uint32_t fb_id_;
    float target_position_;
    float total_angle_, last_angle_;
    bool first_run_;
    LibXR::PID<> speed_pid_, position_pid_;

    struct {
        uint16_t angle_raw;
        int16_t speed_rpm;
        int16_t current_raw;
        uint8_t temperature;
        float angle_rad;
        float speed_rad_per_s;
    } fb_;
};