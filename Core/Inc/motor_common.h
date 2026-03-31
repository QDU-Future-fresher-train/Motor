#ifndef MOTOR_COMMON_H
#define MOTOR_COMMON_H
#include <stdint.h>
typedef enum {
    MOTOR_GM6020,
} MotorType;
typedef struct {
    MotorType type;    
    int32_t target_pos;   
    int32_t current_pos;  
    int16_t output_current; 
} Motor_t;
void Motor_Init(Motor_t *m, MotorType type);
void Motor_Set_Target(Motor_t *m, int32_t pos);
void Motor_Update(Motor_t *m);
int32_t Read_Encoder_Value(Motor_t *m);

#endif