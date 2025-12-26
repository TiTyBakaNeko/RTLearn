#include "pid.h"
#include <math.h>

PID_HandleTypeDef pid_chassis_motor[4];
PID_HandleTypeDef pid_shovel_motor;
 
void PID_Init(PID_HandleTypeDef *pid, float kp, float ki, float kd, float kf) 
{

    pid->K_P = kp;
    pid->K_I = ki;
    pid->K_D = kd;
    pid->K_F = kf;
    
    pid->I_Out_Max = 0.0f;
    pid->Out_Max = 0.0f;
    
    pid->I_Variable_Speed_A = 0.0f;
    pid->I_Variable_Speed_B = 0.0f;
    pid->I_Separate_Threshold = 0.0f;
    
    pid->Target = 0.0f;
    pid->Now = 0.0f;
    pid->Error = 0.0f;
    pid->Last_Error = 0.0f;
    pid->Integral_Error = 0.0f;
    pid->Output = 0.0f;
}
 
// 设置积分限幅
void PID_SetIntegralLimit(PID_HandleTypeDef *pid, float limit) 
{
    pid->I_Out_Max = fabsf(limit);
}
 
// 设置输出限幅
void PID_SetOutputLimit(PID_HandleTypeDef *pid, float limit) 
{
    pid->Out_Max = fabsf(limit);
}
 
// 设置变速积分参数
void PID_SetVariableSpeedParam(PID_HandleTypeDef *pid, float a, float b) 
{
    pid->I_Variable_Speed_A = fabsf(a);
    pid->I_Variable_Speed_B = fabsf(b);
}
 
// 设置积分分离阈值
void PID_SetIntegralSeparateThreshold(PID_HandleTypeDef *pid, float threshold) 
{
    pid->I_Separate_Threshold = fabsf(threshold);
}
 
// PID计算函数
float PID_Calculate(PID_HandleTypeDef *pid, float target, float now,float dt) 
{
    // 更新目标值和当前值
    pid->Target = target;
    pid->Now = now;
    
    // 计算当前误差
    pid->Error = target - now;
    
    // 计算积分项
    float integral = 0.0f;
    
    // 积分分离
    if (pid->I_Separate_Threshold > 0.0f && fabsf(pid->Error) > pid->I_Separate_Threshold) 
        integral = 0.0f; // 误差超过阈值时不进行积分
    else 
		{
        // 变速积分
        float error_abs = fabsf(pid->Error);
        float alpha = 1.0f;
        
        if (pid->I_Variable_Speed_B > 0.0f && error_abs > pid->I_Variable_Speed_A) 
				{
            if (error_abs > (pid->I_Variable_Speed_A + pid->I_Variable_Speed_B)) 
                alpha = 0.0f; // 误差过大时取消积分
             else 
                alpha = 1.0f - (error_abs - pid->I_Variable_Speed_A) / pid->I_Variable_Speed_B;
        }
        
        // 计算积分项
        pid->Integral_Error += pid->Error ;
        
        // 积分限幅
        if (pid->I_Out_Max > 0.0f) 
				{
            if (pid->Integral_Error > pid->I_Out_Max) 
						{
                pid->Integral_Error = pid->I_Out_Max;
            } else if (pid->Integral_Error < -pid->I_Out_Max) 
						{
                pid->Integral_Error = -pid->I_Out_Max;
            }
        }
        
        integral = pid->K_I * pid->Integral_Error * alpha;
    }
    
    // 计算微分项 (使用当前误差和上一次误差的差值)
    float derivative = pid->K_D * (pid->Error - pid->Last_Error) ;
    
    // 计算比例项
    float proportional = pid->K_P * pid->Error;
    
    // 计算前馈项
    float feedforward = pid->K_F * target;
    
    // 计算总输出
    pid->Output = proportional + integral*dt + derivative/dt + feedforward;
    
    // 输出限幅
    if (pid->Out_Max > 0.0f) 
		{
        if (pid->Output > pid->Out_Max) 
				{
            pid->Output = pid->Out_Max;
        } else if (pid->Output < -pid->Out_Max) 
				{
            pid->Output = -pid->Out_Max;
        }
		}
    
    // 保存当前误差用于下一次计算
    pid->Last_Error = pid->Error;
    
    return pid->Output;
}
 
// 重置PID控制器
void PID_Reset(PID_HandleTypeDef *pid) 
{
    pid->Error = 0.0f;
    pid->Last_Error = 0.0f;
    pid->Integral_Error = 0.0f;
    pid->Output = 0.0f;
}
