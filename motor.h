#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

typedef struct
{
    int16_t angle;
    int16_t last_angle;
    int32_t total_angle;

    int16_t speed;
    int16_t current;

    float target;

    float kp;
    float ki;
    float integral;

} Motor_t;

/* 初始化 */
void motor_init(Motor_t *m);

/* 更新反馈（CAN） */
void motor_update(Motor_t *m, uint8_t data[8]);

/* PID */
int16_t motor_pid(Motor_t *m);

/* 发送电流 */
void motor_send_current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

#endif