#include "gm6020_driver.h"
#include "libxr.h"
#include "libxr_can.h"
#include "motor_common.h"

void GM6020_PID(Motor_t *m) {
    int32_t error = m->target_pos - m->current_pos;
    m->output_current = (int16_t)(error * 10);
    if (m->output_current > 20000) m->output_current = 20000;
    if (m->output_current < -20000) m->output_current = -20000;
}

void GM6020_Send(Motor_t *m) {
    uint8_t data[8] = {0};
    data[0] = (m->output_current >> 8) & 0xFF;
    data[1] = m->output_current & 0xFF;
    XR_CAN_Send(GM6020_CAN_ID, data, 8);
}
