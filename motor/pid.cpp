#include "define.hpp"

//串口通信
ErrorCode USART1_Init(void)
{
    xr_usart_config_t usart_cfg;
    usart_cfg.port = USART1_PORT;
    usart_cfg.baudrate = 115200;
    usart_cfg.data_bits = 8;
    usart_cfg.stop_bits = 1;
    usart_cfg.parity = 0;

    if(xr_usart_init(&usart_cfg) != ERROR_NONE)
    {
        return ERROR_USART;
    }
    return ERROR_NONE;
}

void USART1_Send(uint8_t* data, uint16_t len)
{
    if(USART_SendData(data, len) == ERROR_NONE){

    }
    else{

    }
}

//CAN通信
ErrorCode CAN1_Init(void)
{
    xr_can_config_t can_cfg;
    can_cfg.port = 1;
    can_cfg.baudrate = CAN1_BAUDRATE;
    can_cfg.mode = 0;

    if(xr_can_init(&can_cfg) != ERROR_NONE)
    {
        return ERROR_CAN;
    }
    return ERROR_NONE;
}

ErrorCode CAN1_Send_Motor(uint16_t motor_id, int16_t current)
{
    xr_can_msg_t can_msg;
    can_msg.id = motor_id;
    can_msg.len = 8;
    can_msg.data[0] = current & 0xFF;
    can_msg.data[1] = (current >> 8) & 0xFF;
    can_msg.data[2] = 0;
    can_msg.data[3] = 0;
    can_msg.data[4] = 0;
    can_msg.data[5] = 0;
    can_msg.data[6] = 0;
    can_msg.data[7] = 0;

    return xr_can_send(&can_msg);
}

//PID
typedef struct
{
    float kp;       // 比例系数
    float ki;       // 积分系数
    float kd;       // 微分系数
    float target;   // 目标位置
    float actual;   // 实际位置
    float error;    // 误差
    float error_last;// 上一次误差
    float integral; // 积分值
    float output;   // PID输出
} PID_HandleTypeDef;

void PID_Init(PID_HandleTypeDef* pid, float kp, float ki, float kd)
{
    assert(pid != NULL);
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->target = 0;
    pid->actual = 0;
    pid->error = 0;
    pid->error_last = 0;
    pid->integral = 0;
    pid->output = 0;
}

float PID_Calc(PID_HandleTypeDef* pid, float actual)
{
    assert(pid != NULL);
    pid->actual = actual;
    pid->error = pid->target - pid->actual;

    pid->integral += pid->error;

    pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * (pid->error - pid->error_last);

    pid->error_last = pid->error;

    if(pid->output > 2048) pid->output = 2048;
    if(pid->output < -2048) pid->output = -2048;

    return pid->output;
}

ErrorCode USART_Module_Init(void)
{
    return USART1_Init();
}

void USART_Module_Send(uint8_t* data, uint16_t len)
{
    USART1_Send(data, len);
}

ErrorCode CAN_Module_Init(void)
{
    return CAN1_Init();
}

ErrorCode CAN_Module_Send_Motor(uint16_t motor_id, int16_t current)
{
    return CAN1_Send_Motor(motor_id, current);
}