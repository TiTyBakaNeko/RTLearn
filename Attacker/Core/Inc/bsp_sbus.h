#ifndef BSP_SBUS_H_
#define BSP_SBUS_H_

#include "stdint.h"
#include "usart.h"

#define SBUS_RECV_MAX    25

void SBUS_Init(void);
void SBUS_Receive(uint8_t data);
void SBUS_Handle(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

extern uint8_t sbus_data[SBUS_RECV_MAX];
extern int16_t g_sbus_channels[18];

#endif /* BSP_SBUS_H_ */
