#ifndef CAN3508_H
#define CAN3508_H

#include "main.h"
#include "bsp_can.h"

//CAN ID
typedef enum
{
    CAN3508_ALL_ID = 0x200,
    CAN3508_M1_ID = 0x201,
    CAN3508_M2_ID = 0x202,
    CAN3508_M3_ID = 0x203,
    CAN3508_M4_ID = 0x204,

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

extern void CAN3508_ResetID(void);
extern void CAN3508_TransmitCurrent(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

#endif
