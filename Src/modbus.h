/*
 * modbus.h
 *
 *  Created on: 5 May 2019
 *      Author: serg
 */

#ifndef SRC_MODBUS_H_
#define SRC_MODBUS_H_

#include "stm32f4xx_hal.h"

#define MODBUS_RX_BUFF_SIZE	20
#define MODBUS_TIMEOUT_115200	1	//msec

void vModbusProtocol(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* SRC_MODBUS_H_ */
