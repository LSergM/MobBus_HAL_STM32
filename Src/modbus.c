/*
 * modbus.c
 *
 *  Created on: 5 May 2019
 *      Author: serg
 */
#include "modbus.h"
#include "stm32f4xx_hal.h"
#include "crc16.h"

#define FRAME_SIZE_MIN		7
#define MODBUS_FUNCTION		1

static uint8_t au8RxBuf[MODBUS_RX_BUFF_SIZE];
static uint8_t au8TxBuf[MODBUS_TX_BUFF_SIZE];
static uint8_t u8Buf;
static uint8_t u8DeviceAdress = 0;
static uint32_t u32ResetTimeout = 0;
static uint32_t u32CntRxData = 0;
static uint32_t u32CntTxData = 0;
static UART_HandleTypeDef * s_pHuart;

static uint32_t u32ModbusGetTimeout(void);
static void vModbus_ParseNewFrame(void);
static uint8_t u8ModbusReadHoldingRegisters(void);
static uint8_t u8ModbusReadInputRegisters(void);
static uint8_t u8ModbusWriteMultipleHoldindRegisters(void);

static uint16_t au16ModbusRwReg[NUMBER_OF_RW_REGS];
static uint16_t au16ModbusRReg[NUMBER_OF_R_REGS];

int32_t i32SetModbusAdress(uint8_t u8Adr)
{
	u8DeviceAdress = u8Adr;
	return (0);
}

int32_t i32ModbusSetUart(UART_HandleTypeDef * phuart)
{
	HAL_SetTickFreq(HAL_TICK_FREQ_1KHZ);
	s_pHuart = phuart;
#ifdef RS485
	DE_OFF;
	RE_ON;
#endif
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
		  u32CntRxData = 0;
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

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
#ifdef RS485
	DE_OFF;
	RE_ON;
#endif
}

static void vModbus_ParseNewFrame(void)
{
	uint16_t u16FrameCRC;
	uint8_t u8Error = 0;

	if (u32CntRxData < FRAME_SIZE_MIN)
	{
		return;
	}
	if (au8RxBuf[0] != u8DeviceAdress)
	{
		return;
	}
	u16FrameCRC = (*(uint16_t *)&au8RxBuf[u32CntRxData - 2]);
	if (u16FrameCRC != u16GetCRC16((uint8_t *)&au8RxBuf, u32CntRxData - 2))
	{
		return;
	}

	switch (au8RxBuf[MODBUS_FUNCTION])
    {
	case 3:
		u8Error = u8ModbusReadHoldingRegisters();
		break;
	case 4:
		u8Error = u8ModbusReadInputRegisters();
		break;
	case 16:
		u8Error = u8ModbusWriteMultipleHoldindRegisters();
		break;
	default:
		u8Error = 1;
		break;
    }

	if (u8Error)
	{
		au8TxBuf[0] = au8RxBuf[0];
		au8TxBuf[1] = au8RxBuf[1] | 0x80;
		au8TxBuf[2] = u8Error;
		u32CntTxData = 3;
	}
	*(uint16_t *)&au8TxBuf[u32CntTxData] = u16GetCRC16((uint8_t *)au8TxBuf, u32CntTxData);
	u32CntTxData += 2;
#ifdef RS485
	RE_OFF;
	DE_ON;
#endif
	HAL_UART_Transmit_IT(s_pHuart, (uint8_t *)&au8TxBuf, u32CntTxData);
}

static uint8_t u8ModbusReadHoldingRegisters(void)
{
	uint16_t u16Regs = 0;
	uint16_t u16NumWords = 0;
	uint16_t i = 0;
	uint8_t u8Error = 0;

	u16Regs = ((au8RxBuf[2] << 8) | au8RxBuf[3]);
	u16NumWords = ((au8RxBuf[4] << 8) | au8RxBuf[5]);

	if (u16Regs >= NUMBER_OF_RW_REGS)
    {
    	u8Error = 2;
    	return (u8Error);
    }
	if((u16NumWords+u16Regs > NUMBER_OF_RW_REGS) ||  !u16NumWords
          ||  ((u16NumWords * 2) > MODBUS_TX_BUFF_SIZE))
	{
		  u8Error=3;
		  return (u8Error);
	}
	for(i=0; i < u16NumWords; i++)
	{
	  au8TxBuf[i*2 + 3] = au16ModbusRwReg[u16Regs + i] >> 8;
	  au8TxBuf[i*2 + 4] = au16ModbusRwReg[u16Regs + i];
	}

	au8TxBuf[0] = au8RxBuf[0];
	au8TxBuf[1] = au8RxBuf[1]; //func
	au8TxBuf[2] = u16NumWords << 1; //
	u32CntTxData = (u16NumWords << 1) + 3;

	return (u8Error);
}

static uint8_t u8ModbusReadInputRegisters(void)
{
	uint16_t u16Regs = 0;
	uint16_t u16NumWords = 0;
	uint16_t i = 0;
	uint8_t u8Error = 0;

	u16Regs = ((au8RxBuf[2] << 8) | au8RxBuf[3]);
	u16NumWords = ((au8RxBuf[4] << 8) | au8RxBuf[5]);

	if (u16Regs >= NUMBER_OF_R_REGS)
    {
    	u8Error = 2;
    	return (u8Error);
    }
	if((u16NumWords+u16Regs > NUMBER_OF_R_REGS) ||  !u16NumWords
          ||  ((u16NumWords * 2) > MODBUS_TX_BUFF_SIZE))
	{
		  u8Error=3;
		  return (u8Error);
	}
	for(i=0; i < u16NumWords; i++)
	{
	  au8TxBuf[i*2 + 3] = au16ModbusRReg[u16Regs + i] >> 8;
	  au8TxBuf[i*2 + 4] = au16ModbusRReg[u16Regs + i];
	}

	au8TxBuf[0] = au8RxBuf[0];
	au8TxBuf[1] = au8RxBuf[1]; //func
	au8TxBuf[2] = u16NumWords << 1; //
	u32CntTxData = (u16NumWords << 1) + 3;

	return (u8Error);
}

static uint8_t u8ModbusWriteMultipleHoldindRegisters(void)
{
	uint16_t u16Regs = 0;
	uint16_t u16NumWords = 0;
	uint16_t i = 0;
	uint8_t u8Error = 0;
	uint16_t u16ai = 0;
	uint16_t u16aa = 0;

	u16Regs = ((au8RxBuf[2] << 8) | au8RxBuf[3]);
	u16NumWords = ((au8RxBuf[4] << 8) | au8RxBuf[5]);

	if (u16Regs >= NUMBER_OF_RW_REGS)
    {
    	u8Error = 2;
    	return (u8Error);
    }
	if((u16NumWords+u16Regs > NUMBER_OF_RW_REGS) ||  !u16NumWords
          ||  ((u16NumWords * 2) > MODBUS_TX_BUFF_SIZE))
	{
		  u8Error=3;
		  return (u8Error);
	}
	for(i=0; i < u16NumWords; i++)
	{
	  u16ai = i*2 + 7;
	  u16aa = u16Regs + i;
	  au16ModbusRwReg[u16aa] = (au8RxBuf[u16ai] << 8) | au8RxBuf[u16ai+1];
	}

	u32CntTxData = 6;
	for(i=0; i < u32CntTxData; i++)
	{
		au8TxBuf[i] = au8RxBuf[i];
	}
	return (u8Error);
}
