#define _USE_MATH_DEFINES
#include <cmath>
#include "Motor.hpp"

Motor::Motor(LibXR::CAN* can, uint32_t feedback_id)
    : can_(can), fb_id_(feedback_id),
      target_position_(0.0f), total_angle_(0.0f), last_angle_(0.0f), first_run_(true),
      speed_pid_(LibXR::PID<>::Param{1.0, 0.5, 0.1, 0.0, 1000.0, 30000.0, false}),
      position_pid_(LibXR::PID<>::Param{1.0, 2.0, 0.0, 0.0, 1000.0, 500.0, true})
{
    can_->Register(LibXR::CAN::Callback::Create(CANRxCallback, this),
                   LibXR::CAN::Type::STANDARD,
                   LibXR::CAN::FilterMode::ID_RANGE,
                   fb_id_, fb_id_);
}


void Motor::CANRxCallback(bool in_isr, Motor* self, const LibXR::CAN::ClassicPack& pack) {
    self->fb_.angle_raw   = (pack.data[0] << 8) | pack.data[1];
    self->fb_.speed_rpm   = (pack.data[2] << 8) | pack.data[3];
    self->fb_.current_raw = (pack.data[4] << 8) | pack.data[5];
    self->fb_.temperature = pack.data[6];

    self->fb_.angle_rad = self->fb_.angle_raw * (2 * M_PI / 8192.0f);
    self->fb_.speed_rad_per_s = self->fb_.speed_rpm * (2 * M_PI / 60.0f);
}

void Motor::ControlTask() {
    float current_angle = fb_.angle_rad;
    if (first_run_) {
        last_angle_ = current_angle;
        total_angle_ = current_angle;
        first_run_ = false;
    } else {
        float delta = current_angle - last_angle_;
        if (delta > M_PI) delta -= 2 * M_PI;
        if (delta < -M_PI) delta += 2 * M_PI;
        total_angle_ += delta;
        last_angle_ = current_angle;
    }

    static uint32_t tick = 0;
    float target_speed = 0.0f;
    tick++;
    if (tick >= 10) {
        tick = 0;
        // 位置环：目标位置 vs 当前位置
        target_speed = position_pid_.Calculate(target_position_, total_angle_, 0.01f);
    }

    // 速度环：目标转速 vs 当前转速
    float current_cmd = speed_pid_.Calculate(target_speed, fb_.speed_rpm, 0.001f);
    if (current_cmd > 30000.0f) current_cmd = 30000.0f;
    if (current_cmd < -30000.0f) current_cmd = -30000.0f;

    LibXR::CAN::ClassicPack pack;
    pack.id = 0x1FF;
    pack.type = LibXR::CAN::Type::STANDARD;
    pack.data[0] = (static_cast<int16_t>(current_cmd) >> 8) & 0xFF;
    pack.data[1] = static_cast<int16_t>(current_cmd) & 0xFF;
    can_->AddMessage(pack);
}

void Motor::SetTargetPosition(float position_rad) {
    target_position_ = position_rad;
}

float Motor::GetCurrentPosition() const {
    return total_angle_;
}
void Motor::Update() {
    ControlTask();
}