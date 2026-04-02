#include "motor.h"

extern CAN_HandleTypeDef hcan1;

void motor_send_current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
    uint8_t data[8];

    data[0] = iq1 >> 8;
    data[1] = iq1;
    data[2] = iq2 >> 8;
    data[3] = iq2;
    data[4] = iq3 >> 8;
    data[5] = iq3;
    data[6] = iq4 >> 8;
    data[7] = iq4;

    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = 0x200;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    uint32_t mailbox;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &mailbox);
}

void motor_init(Motor_t *m)
{
    m->kp = 5.0f;
    m->ki = 0.01f;
    m->integral = 0;
    m->total_angle = 0;
    m->angle = 0;
    m->last_angle = 0;
    m->speed = 0;
    m->current = 0; 
}

/* GM6020反馈解析 */
void motor_update(Motor_t *m, uint8_t data[8])
{
    m->last_angle = m->angle;

    m->angle = (int16_t)(data[0] << 8 | data[1]);
    m->speed = (int16_t)(data[2] << 8 | data[3]);
    m->current = (int16_t)(data[4] << 8 | data[5]);

    int16_t delta = m->angle - m->last_angle;

    if (delta > 4096) delta -= 8192;
    if (delta < -4096) delta += 8192;

    m->total_angle += delta;
}

/* 位置环 PID */
int16_t motor_pid(Motor_t *m)
{
    float err = m->target - m->total_angle;

    m->integral += err;
    if (m->integral > 10000) m->integral = 10000;
    if (m->integral < -10000) m->integral = -10000;

    float out = m->kp * err + m->ki * m->integral;

    if (out > 16000) out = 16000;
    if (out < -16000) out = -16000;

    return (int16_t)out;
}

