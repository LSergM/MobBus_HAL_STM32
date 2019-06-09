/*
 * modbus.c
 *
 *  Created on: 5 May 2019
 *      Author: serg
 */
#include "modbus.h"

static uint8_t aRxBuf[MODBUS_RX_BUFF_SIZE];
static uint32_t u32ResetTimeout = 0;
static uint32_t u32CntRxData = 0;

static uint32_t u32ModbusGetTimeout(void);
static void vModbus_ParseNewFrame(void);

void vModbusProtocol(void)
{
  if(u32ModbusGetTimeout() > MODBUS_TIMEOUT_115200) {
	  vModbus_ParseNewFrame();
  }
}

static uint32_t u32ModbusGetTimeout(void)
{
static uint32_t u32Timeout = 0;
static uint32_t u32LastTickValue = 0;

	if(u32ResetTimeout) {
		u32Timeout = 0;
		u32ResetTimeout = 0;
		u32LastTickValue = HAL_GetTick();
	}
	else if (u32CntRxData) {
		if(HAL_GetTick() - u32LastTickValue){
			u32Timeout++;
		}
	}
	else {
		u32Timeout = 0;
	}

	return(u32Timeout);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	u32ResetTimeout = 1;
	u32CntRxData++;
}

static void vModbus_ParseNewFrame(void) {
	u32CntRxData = 0;
}
