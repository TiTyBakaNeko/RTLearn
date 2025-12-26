#ifndef CAN3508_H
#define CAN3508_H

#define CHASSIS_CAN hcan1
#define SHOVEL_CAN hcan2

#include "main.h"
#include "bsp_can.h"
#include "pid.h"

//Left:M1,M4
//Right:M2,M3
//Shovel:M5

//CAN ID
typedef enum
{
    CAN3508_ALL_ID = 0x200,
    CAN3508_M1_ID = 0x201,
    CAN3508_M2_ID = 0x202,
    CAN3508_M3_ID = 0x203,
    CAN3508_M4_ID = 0x204,
		CAN3508_SHOVEL_ID = 0x1ff,
		CAN3508_M5_ID = 0x205,

} CANID;

//3508 Motor data
typedef struct
{
    uint16_t ECD;
    int16_t Speed;
    int16_t Current;
    uint8_t Temperate;
    int16_t LastECD;
} MotorData;

extern PID_HandleTypeDef pid_chassis_motor[4];
extern PID_HandleTypeDef pid_shovel_motor;
extern MotorData Motor_Data[5];
extern void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
extern void CAN3508_Chassis_TransmitCurrent(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN3508_Shovel_TransmitCurrent(int16_t motor);

#endif
