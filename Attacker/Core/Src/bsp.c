#include "bsp.h"

// The peripheral device is initialized  外设设备初始化
void Bsp_Init(void)
{
	USART_Init();
}


// main.c中循环调用此函数，避免多次修改main.c文件。
// This function is called in a loop in main.c to avoid multiple modifications to the main.c file
void Bsp_Loop(void)
{
	SBUS_Handle();
	HAL_Delay(10);
}
