/* === MODULE MANIFEST V2 ===
module_description: Position control action for motor
constructor_args:
  - param:
      name: motor_id
      type: Motor*
      description: Pointer to Motor module instance
template_args: []
required_hardware: []
depends:
  - Motor
=== END MANIFEST === */

#pragma once

#include "Motor.hpp"

class PositionAction {
public:
    PositionAction(Motor& motor);
    void Run();

private:
    Motor& motor_;
    int step_;
    uint32_t last_time_;
};