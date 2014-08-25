/**
  ******************************************************************************
  * @file    rs485.h
  * @author  Khusainov Timur
  * @version 0.0.0.1
  * @date    18.01.2012
  * @brief   
  ******************************************************************************
  * @attention
  * <h2><center>&copy; COPYRIGHT timyik@gmail.com </center></h2>
  ******************************************************************************
  */

#ifndef RS485_H
#define RS485_H

#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------
typedef enum
{
	RS485_SPEED_BAUDRATE_1200 = 0,
	RS485_SPEED_BAUDRATE_4800,
	RS485_SPEED_BAUDRATE_9600,
	RS485_SPEED_BAUDRATE_19200,
	RS485_SPEED_BAUDRATE_38400,
	RS485_SPEED_BAUDRATE_57600,
	RS485_SPEED_BAUDRATE_115200,
	RS485_SPEED_BAUDRATE_230400,
	RS485_SPEED_BAUDRATE_COUNT,
} tRS485_Speed;

typedef enum
{
	RS485_PARITY_NONE = 0,
	RS485_PARITY_EVEN,
	RS485_PARITY_ODD,
	RS485_PARITY_COUNT
} tRS485_Parity;

typedef enum 
{
	RS485_STOPBIT_1 = 0,
	RS485_STOPBIT_2,
	RS485_STOPBIT_COUNT
} tRS485_Stopbit;
//------------------------------------------------------------------------------
typedef enum 
{ 
  RS485_ACTION_DATA_TX = 0,
  RS485_ACTION_DATA_RX,
  RS485_ACTION_ERROR_TX,
  RS485_ACTION_ERROR_RX,
} tRS485_Action;

typedef void (*tRS485_OnAction)(tRS485_Action action, const uint8_t *data, size_t length);
//------------------------------------------------------------------------------
void BSP_InitRS485(tRS485_Speed speed, tRS485_Parity parity, tRS485_Stopbit stopbits, tRS485_OnAction pOnAction);
void BSP_SetPriority485(tBSP_Priority priority);

void BSP_EnableRS485();
void BSP_DisableRS485();

void RS485_TransmitData(const uint8_t * data, size_t length);
void RS485_ReceiveData(uint8_t * const data, size_t length);
void RS485_Process();
//------------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif

#endif // RS485_H
