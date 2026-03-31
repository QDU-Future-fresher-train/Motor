#ifndef GM6020_DRIVER_H
#define GM6020_DRIVER_H

#include "motor_common.h"
#include <stdint.h>

#define GM6020_CAN_ID 0x200

void GM6020_PID(Motor_t *m);
void GM6020_Send(Motor_t *m);

#endif // GM6020_DRIVER_H
