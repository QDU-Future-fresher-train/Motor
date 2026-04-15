#ifndef DEFINE_H
#define DEFINE_H

#include <assert.h>
#include "libxr.hpp"

#define USART1_PORT  1       // 串口1端口，随便定义的
#define CAN1_BAUDRATE 500000 // CAN1波特率500kbps
#define GM6020_ID 0x201      // GM6020电机ID，从数据手册找的
#define PID_KP 2.0           // PID比例系数，随便设的
#define PID_KI 0.1           // PID积分系数
#define PID_KD 0.05          // PID微分系数

typedef enum
{
    ERROR_NONE = 0,    // 无错误
    ERROR_PARAM = 1,   // 参数错误
    ERROR_USART = 2,   // 串口错误
    ERROR_CAN = 3,     // CAN错误
    ERROR_MOTOR = 4    // 电机错误
} ErrorCode;

//断言
void Motor_Init_Check(uint16_t motor_id)
{
    assert(motor_id != 0); // 如果motor_id是0，程序报错，提示这里有问题
}


#endif