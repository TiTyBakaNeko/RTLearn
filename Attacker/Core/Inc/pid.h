#ifndef PID_H
#define PID_H

#include "main.h"

// PID控制结构体
typedef struct
{
    float K_P;            // 比例系数
    float K_I;            // 积分系数
    float K_D;            // 微分系数
    float K_F;						// 前馈系数
	
    float Out_Max;       // PID输出最大值
    float I_Out_Max;      // 积分限幅
    
		float I_Variable_Speed_A;
		float I_Variable_Speed_B;
		float I_Separate_Threshold;
    
		float Target;				 // 目标转速
		float Now;					 // 实际转速
    float Error;         // 当前误差
    float Last_Error;    // 上一次误差
    float Integral_Error;// 积分误差
    float Output;        // PID输出
} PID_HandleTypeDef;

extern PID_HandleTypeDef pid_chassis_motor[4];
extern PID_HandleTypeDef pid_shovel_motor;

void PID_Init(PID_HandleTypeDef *pid, float kp, float ki, float kd, float kf);
float PID_Calculate(PID_HandleTypeDef *pid, float target, float now,float dt);
void PID_SetIntegralLimit(PID_HandleTypeDef *pid, float limit);
void PID_SetOutputLimit(PID_HandleTypeDef *pid, float limit);
void PID_SetVariableSpeedParam(PID_HandleTypeDef *pid, float a, float b);
void PID_SetIntegralSeparateThreshold(PID_HandleTypeDef *pid, float threshold);
void PID_Reset(PID_HandleTypeDef *pid);

#endif
