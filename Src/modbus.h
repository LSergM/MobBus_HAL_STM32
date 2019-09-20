/*
 * modbus.h
 *
 *  Created on: 5 May 2019
 *      Author: serg
 */

#ifndef SRC_MODBUS_H_
#define SRC_MODBUS_H_

#include "stm32f4xx_hal.h"

#define MODBUS_RX_BUFF_SIZE	100
#define MODBUS_TX_BUFF_SIZE	20
#define MODBUS_TIMEOUT_115200	1	//msec

#define NUMBER_OF_RW_REGS	10
#define NUMBER_OF_R_REGS	10

//#define RS485

#ifdef RS485
	#define DE_ON	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3)
	#define DE_OFF	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3)
	#define RE_ON	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3)
	#define RE_OFF	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3)
#endif

void vModbusProtocol(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
int32_t i32ModbusSetUart(UART_HandleTypeDef * phuart);
int32_t i32SetModbusAdress(uint8_t u8Adr);

#endif /* SRC_MODBUS_H_ */
