#define _USE_MATH_DEFINES
#include <cmath>
#include "PositionAction.hpp"
#include "main.h"   // 包含 HAL_GetTick

PositionAction::PositionAction(Motor& motor) : motor_(motor), step_(0), last_time_(0) {}

void PositionAction::Run() {
    uint32_t now = HAL_GetTick();
    if (now - last_time_ > 5000) {
        last_time_ = now;
        step_ = (step_ + 1) % 3;
        float targets[3] = {0.0f, 5 * M_PI, 0.0f};  // 0 -> 5圈 -> 0
        motor_.SetTargetPosition(targets[step_]);
    }
}