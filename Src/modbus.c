/*
 * modbus.c
 *
 *  Created on: 5 May 2019
 *      Author: serg
 */
#include "modbus.h"
#include "stm32f4xx_hal.h"

static uint8_t au8RxBuf[MODBUS_RX_BUFF_SIZE];
static uint8_t u8Buf;
static uint32_t u32ResetTimeout = 0;
static uint32_t u32CntRxData = 0;
static UART_HandleTypeDef * s_pHuart;

static uint32_t u32ModbusGetTimeout(void);
static void vModbus_ParseNewFrame(void);

int32_t i32ModbusSetUart(UART_HandleTypeDef * phuart)
{
	HAL_SetTickFreq(HAL_TICK_FREQ_1KHZ);
	s_pHuart = phuart;
	HAL_UART_Receive_IT(s_pHuart, &u8Buf, 1);

	return(0);
}

void vModbusProtocol(void)
{
  if (u32CntRxData)
  {
	  if(u32ModbusGetTimeout() > MODBUS_TIMEOUT_115200)
	  {
		  vModbus_ParseNewFrame();
	  }
  }
}

static uint32_t u32ModbusGetTimeout(void)
{
	uint32_t u32Timeout = 0;
	static uint32_t u32LastTickValue = 0;

	if(u32ResetTimeout)
	{
		u32Timeout = 0;
		u32ResetTimeout = 0;
		u32LastTickValue = HAL_GetTick();
	}
	else
	{
		u32Timeout = HAL_GetTick() - u32LastTickValue;
	}

	return(u32Timeout);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (u32CntRxData < MODBUS_RX_BUFF_SIZE)
	{
		au8RxBuf[u32CntRxData] = u8Buf;
	}
	HAL_UART_Receive_IT(s_pHuart, &u8Buf, 1);
	u32ResetTimeout = 1;
	u32CntRxData++;
}

static void vModbus_ParseNewFrame(void)
{
	u32CntRxData = 0;
}
