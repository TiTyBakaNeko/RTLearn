#include "can3508.h"

extern CAN_HandleTypeDef hcan1;

#define MotorData_Get(ptr, data)                                    \
    {                                                                   \
        (ptr)->LastECD = (ptr)->ECD;                                   \
        (ptr)->ECD = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->Speed = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->Current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->Temperate = (data)[6];                                   \
    }

//电机数据 0:3508M1 1:3508M2 2:3508M3 3:3508M4
MotorData Motor_Data[4];
static CAN_TxHeaderTypeDef  MotorTransmit;
static uint8_t              MotorTransmitData[8];

//中断回调,接收电机数据
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        case CAN3508_M1_ID:
        case CAN3508_M2_ID:
        case CAN3508_M3_ID:
        case CAN3508_M4_ID:
        {
            static uint8_t Motor_Index = 0;
            //Get motor ID
            Motor_Index = rx_header.StdId - CAN3508_M1_ID;
						if (Motor_Index < 4)
								MotorData_Get(&Motor_Data[Motor_Index], rx_data);
            break;
        }
        default:
        {
            break;
        }
    }
}

//发送ID为0x700的CAN包,使3508电机快速设置ID
void CAN3508_ResetID(void)
{
    uint32_t send_mail_box;
    MotorTransmit.StdId = 0x700;
    MotorTransmit.IDE = CAN_ID_STD;
    MotorTransmit.RTR = CAN_RTR_DATA;
    MotorTransmit.DLC = 0x08;
    MotorTransmitData[0] = 0;
    MotorTransmitData[1] = 0;
    MotorTransmitData[2] = 0;
    MotorTransmitData[3] = 0;
    MotorTransmitData[4] = 0;
    MotorTransmitData[5] = 0;
    MotorTransmitData[6] = 0;
    MotorTransmitData[7] = 0;
    HAL_CAN_AddTxMessage(&hcan1, &MotorTransmit, MotorTransmitData, &send_mail_box);
}


//发送电机控制电流(0x201,0x202,0x203,0x204)3508电机控制电流, 范围 [-16384,16384]
void CAN3508_TransmitCurrent(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    MotorTransmit.StdId = CAN3508_ALL_ID;
    MotorTransmit.IDE = CAN_ID_STD;
    MotorTransmit.RTR = CAN_RTR_DATA;
    MotorTransmit.DLC = 0x08;
    MotorTransmitData[0] = motor1 >> 8;
    MotorTransmitData[1] = motor1;
    MotorTransmitData[2] = motor2 >> 8;
    MotorTransmitData[3] = motor2;
    MotorTransmitData[4] = motor3 >> 8;
    MotorTransmitData[5] = motor3;
    MotorTransmitData[6] = motor4 >> 8;
    MotorTransmitData[7] = motor4;
    HAL_CAN_AddTxMessage(&hcan1, &MotorTransmit, MotorTransmitData, &send_mail_box);
}
