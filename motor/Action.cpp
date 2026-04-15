#include "define.hpp"
#include "pid.cpp"

//通用电机模块
typedef enum
{
    MOTOR_GM6020,
    MOTOR_OTHER,
} Motor_TypeDef;

typedef struct
{
    Motor_TypeDef type;       // 电机类型
    uint16_t id;              // 电机ID
    PID_HandleTypeDef pid;    // PID结构体（位置环）
    int16_t current;          // 电机电流指令
    float target_pos;         // 电机目标位置
    float actual_pos;         // 电机实际位置
} Motor_HandleTypeDef;

ErrorCode Motor_Init(Motor_HandleTypeDef* motor, Motor_TypeDef type, uint16_t id)
{
    assert(motor != NULL);
    motor->type = type;
    motor->id = id;
    motor->target_pos = 0;
    motor->actual_pos = 0;
    motor->current = 0;

    PID_Init(&motor->pid, PID_KP, PID_KI, PID_KD);

    if(type == MOTOR_GM6020)
    {
        return CAN1_Init();
    }
    else if(type == MOTOR_OTHER)
    {
        return USART1_Init();
    }
    else
    {
        return ERROR_PARAM;
    }
}

void Motor_SetPos(Motor_HandleTypeDef* motor, float target_pos)
{
    assert(motor != NULL);
    motor->target_pos = target_pos;
    motor->pid.target = target_pos;

    motor->actual_pos += 0.1;
    motor->current = (int16_t)PID_Calc(&motor->pid, motor->actual_pos);

    if(motor->type == MOTOR_GM6020)
    {
        CAN1_Send_Motor(motor->id, motor->current);
    }
    else if(motor->type == MOTOR_OTHER)
    {
        uint8_t data[2] = {motor->current & 0xFF, (motor->current >> 8) & 0xFF};
        USART1_Send(data, 2);
    }
}


Motor_HandleTypeDef gm6020_motor;

ErrorCode Action_Init(void)
{
    return Motor_Init(&gm6020_motor, MOTOR_GM6020, GM6020_ID);
}

void Action_Motor_Run(float target_pos)
{
    Motor_SetPos(&gm6020_motor, target_pos);
}


int main(void)
{
    xr_init();

    if(Action_Init() != ERROR_NONE)
    {
        while(1);
    }

    while(1)
    {
        Action_Motor_Run(100.0f);
    }
}