/**
  ******************************************************************************
  * @file    uowi.h
  * @author  Khusainov Timur
  * @version 0.0.0.0
  * @date    31.10.2012
  * @brief   
  ******************************************************************************
  * @attention
  * <h2><center>&copy; COPYRIGHT timyik@gmail.com </center></h2>
  ******************************************************************************
  */

#ifndef UOWI_H
#define UOWI_H

typedef enum
{
	UOWI_STATE_NOINIT = 0,
	UOWI_STATE_DETECT_WAIT,
	UOWI_STATE_DETECT_COMPLETE,
	UOWI_STATE_DETECT_ERROR,
	UOWI_STATE_SEND_START,
	UOWI_STATE_SEND_WAIT,
	UOWI_STATE_SEND_COMPLETE,
	UOWI_STATE_SEND_ERROR,
	UOWI_STATE_RECV_WAIT,
	UOWI_STATE_RECV_COMPLETE,
	UOWI_STATE_STATUS_BUSY,
	UOWI_STATE_STATUS_EMPTY,
	UOWI_STATE_STATUS_ERROR
} tUOWI_State;

extern volatile tUOWI_State UOWI_State;

enum
{
	UOWI_CMD_DETECT       = (0xf0),
	UOWI_CMD_WRITE_BIT0   = (0x00),
	UOWI_CMD_WRITE_BIT1   = (0xff),
	UOWI_CMD_READ_BIT     = (0xff),
	UOWI_CMD_READ_STATE   = (0xff),
	UOWI_RET_DETECT_ERROR = (0xf0),
	UOWI_RET_READ_BIT1    = (0xff),
	UOWI_RET_STATE_EMPTY  = (0xff)
};

enum
{
	UOWI_TX_DATA_MAX_LENGTH = (12),
	UOWI_RX_DATA_MAX_LENGTH = (16)
};

void UOWI_Init();

void UOWI_StartDetect();
void UOWI_StartSend();
void UOWI_SendData(const uint8_t *pdata, uint16_t txlen, uint16_t rxlen);
void UOWI_RecvData(uint8_t *pdata, uint16_t rxlen);
tUOWI_State UOWI_GetState();
uint8_t UOWI_IsDetect();
uint8_t UOWI_IsSend();
uint8_t UOWI_IsRecv();
uint8_t UOWI_IsBusy();

#endif // UOWI_H
