#include "bsp_sbus.h"
#include "string.h"
#include "usart.h"
#include "dma.h"
#include "pid.h"
#include "can3508.h"
#include <math.h>

#define SBUS_RECV_MAX    25
#define SBUS_START       0x0F
#define SBUS_END         0x00

#define SHOVEL_MAX_ANGLE 90
#define K_STRAIGHT_SPEED 12.0f
#define K_TURN_SPEED 3.5f
#define K_SHOVEL_SPEED 5.0f

float straightspeed = 0 , turnspeed = 0;
float shovelspeed = 0;

//接收数据相关参数
uint8_t sbus_start = 0;
uint8_t sbus_buf_index = 0;
uint8_t sbus_new_cmd = 0;

//数据缓存
uint8_t inBuffer[SBUS_RECV_MAX] = {0};
uint8_t sbus_data[SBUS_RECV_MAX] = {0};
int16_t g_sbus_channels[18] = {0};

void SBUS_Init(void)
{
    // 清空缓冲区和全局变量
    memset(inBuffer, 0, sizeof(inBuffer));
    memset(sbus_data, 0, sizeof(sbus_data));
    memset(g_sbus_channels, 0, sizeof(g_sbus_channels));
    
    // 启动USART3 DMA接收
    HAL_UART_Receive_DMA(&huart3, inBuffer, SBUS_RECV_MAX);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//    if (huart->Instance == USART3)
//    {
//        // 校验帧头和帧尾，确保数据有效
//        if (sbus_data[0] == SBUS_START && sbus_data[SBUS_RECV_MAX-1] == SBUS_END)
//        {
//						memcpy(sbus_data, inBuffer, SBUS_RECV_MAX);
//            sbus_new_cmd = 1;
//        }
//				else
//						sbus_new_cmd = 0;
					HAL_UART_Receive_DMA(&huart3, sbus_data, SBUS_RECV_MAX);
//    }
}

//解析SBUS的数据，转化成通道数值

//void SBUS_Parse_Data(void)
//{
//    g_sbus_channels[0]  = ((sbus_data[0] | sbus_data[1] << 8) & 0x07FF);
//    g_sbus_channels[1]  = ((sbus_data[1] >> 3 | sbus_data[2] << 5) & 0x07FF);
//    g_sbus_channels[2]  = ((sbus_data[2] >> 6 | sbus_data[3] << 2 | sbus_data[5] << 10) & 0x07FF);
//    g_sbus_channels[3]  = ((sbus_data[4] >> 1 | sbus_data[5] << 7) & 0x07FF);
//    g_sbus_channels[4]  = ((sbus_data[5] >> 4 | sbus_data[6] << 4) & 0x07FF);
//    g_sbus_channels[5]  = ((sbus_data[6] >> 7 | sbus_data[7] << 1 | sbus_data[9] << 9) & 0x07FF);
//    g_sbus_channels[6]  = ((sbus_data[8] >> 2 | sbus_data[9] << 6) & 0x07FF);
//    g_sbus_channels[7]  = ((sbus_data[9] >> 5 | sbus_data[10] << 3) & 0x07FF);
//    g_sbus_channels[8]  = ((sbus_data[11] | sbus_data[12] << 8) & 0x07FF);
//    g_sbus_channels[9]  = ((sbus_data[12] >> 3 | sbus_data[13] << 5) & 0x07FF);
//    #ifdef ALL_CHANNELS
//    g_sbus_channels[10] = ((sbus_data[13] >> 6 | sbus_data[14] << 2 | sbus_data[16] << 10) & 0x07FF);
//    g_sbus_channels[11] = ((sbus_data[15] >> 1 | sbus_data[16] << 7) & 0x07FF);
//    g_sbus_channels[12] = ((sbus_data[16] >> 4 | sbus_data[17] << 4) & 0x07FF);
//    g_sbus_channels[13] = ((sbus_data[17] >> 7 | sbus_data[18] << 1 | sbus_data[20] << 9) & 0x07FF);
//    g_sbus_channels[14] = ((sbus_data[19] >> 2 | sbus_data[20] << 6) & 0x07FF);
//    g_sbus_channels[15] = ((sbus_data[20] >> 5 | sbus_data[21] << 3) & 0x07FF);
//    #endif
//}

void SBUS_Parse_Data(void)
{
    g_sbus_channels[0]  = ((sbus_data[1] | sbus_data[2] << 8) & 0x07FF);
    g_sbus_channels[1]  = ((sbus_data[2] >> 3 | sbus_data[3] << 5) & 0x07FF);
    g_sbus_channels[2]  = ((sbus_data[3] >> 6 | sbus_data[4] << 2 | sbus_data[5] << 10) & 0x07FF);
    g_sbus_channels[3]  = ((sbus_data[5] >> 1 | sbus_data[6] << 7) & 0x07FF);
    g_sbus_channels[4]  = ((sbus_data[6] >> 4 | sbus_data[7] << 4) & 0x07FF);
    g_sbus_channels[5]  = ((sbus_data[7] >> 7 | sbus_data[8] << 1 | sbus_data[9] << 9) & 0x07FF);
    g_sbus_channels[6]  = ((sbus_data[9] >> 2 | sbus_data[10] << 6) & 0x07FF);
    g_sbus_channels[7]  = ((sbus_data[10] >> 5 | sbus_data[11] << 3) & 0x07FF);
    g_sbus_channels[8]  = ((sbus_data[12] | sbus_data[13] << 8) & 0x07FF);
    g_sbus_channels[9]  = ((sbus_data[13] >> 3 | sbus_data[14] << 5) & 0x07FF);
    #ifdef ALL_CHANNELS
    g_sbus_channels[10] = ((sbus_data[14] >> 6 | sbus_data[15] << 2 | sbus_data[16] << 10) & 0x07FF);
    g_sbus_channels[11] = ((sbus_data[16] >> 1 | sbus_data[17] << 7) & 0x07FF);
    g_sbus_channels[12] = ((sbus_data[17] >> 4 | sbus_data[18] << 4) & 0x07FF);
    g_sbus_channels[13] = ((sbus_data[18] >> 7 | sbus_data[19] << 1 | sbus_data[20] << 9) & 0x07FF);
    g_sbus_channels[14] = ((sbus_data[20] >> 2 | sbus_data[21] << 6) & 0x07FF);
    g_sbus_channels[15] = ((sbus_data[21] >> 5 | sbus_data[22] << 3) & 0x07FF);
    #endif
}

//SBUS接收处理数据
void SBUS_Handle(void)
{
    SBUS_Parse_Data();
		memcpy(sbus_data, inBuffer, SBUS_RECV_MAX);
		//底盘运动,右摇杆控制,横向CH0（右加左减）纵向CH1（前加后减）
		//Left:前M3,后M4
		//Right:前M1,后M2
	
		straightspeed = 0 ;
		turnspeed = 0;
		if(g_sbus_channels[1] < 300 )
				g_sbus_channels[1] = 1024;
		if(g_sbus_channels[0] < 300 )
				g_sbus_channels[0] = 1024;
		if(g_sbus_channels[1] < 1024+75 && g_sbus_channels[1] > 1024-75 )
				g_sbus_channels[1] = 1024;
		if(g_sbus_channels[0] < 1024+75 && g_sbus_channels[0] > 1024-75 )
				g_sbus_channels[0] = 1024;
		straightspeed = (g_sbus_channels[1]-1024) * K_STRAIGHT_SPEED;
		turnspeed = (g_sbus_channels[0]-1024) * K_TURN_SPEED;
		pid_chassis_motor[2].Target = straightspeed + turnspeed;
		pid_chassis_motor[3].Target = straightspeed + turnspeed;
		pid_chassis_motor[0].Target = - straightspeed + turnspeed;
		pid_chassis_motor[1].Target = - straightspeed + turnspeed;
		
		//铲子运动,左摇杆控制，横向CH2（右加左减）M5
		//角度在135°- 微小值 以上时只允许负速度, 在0°+ 微小值以下时只允许正速度 微小值=100
		shovelspeed = 0;
		if(g_sbus_channels[2] <300)
				g_sbus_channels[2] = 1024;
		shovelspeed = (g_sbus_channels[2]-1024) * K_SHOVEL_SPEED;
//		if(Motor_Data[4].ECD > (int)(8192 * (float)(SHOVEL_MAX_ANGLE/360.0f) - 100) && Motor_Data[4].ECD < (int)(8192 * (float)(180.0f/360.0f) - 100))
//				pid_shovel_motor.Target = fminf(shovelspeed,0.0f);
//		else if(Motor_Data[4].ECD < (100) || Motor_Data[4].ECD > (int)(8192 * (float)(180.0f/360.0f) - 100))
//				pid_shovel_motor.Target = fmaxf(shovelspeed,0.0f);
//		else
				pid_shovel_motor.Target = shovelspeed;
}

