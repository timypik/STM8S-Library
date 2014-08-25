/**
  ******************************************************************************
  * @file    ucon.h
  * @author  Khusainov Timur
  * @version 0.0.0.0
  * @date    22.08.2012
  * @brief   Uart (UART1) connection part
  ******************************************************************************
  * @attention
  * <h2><center>&copy; COPYRIGHT timyik@gmail.com </center></h2>
  ******************************************************************************
  */

#ifndef UCON_H
#define UCON_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum 
{ 
  UCON_ACTION_DATA_TX = 0,
  UCON_ACTION_DATA_RX,
  UCON_ACTION_ERROR_TX,
  UCON_ACTION_ERROR_RX,
} tUCON_Action;

typedef void (*tUCON_OnAction)(tUCON_Action action, const uint8_t *data, size_t length);

void BSP_InitUCON(tUCON_OnAction pOnAction);
void BSP_SetPriorityUCON(tBSP_Priority priority);

void BSP_EnableUCON();
void BSP_DisableUCON();

void UCON_TransmitData(const uint8_t *data, size_t length);
void UCON_ReceiveData(uint8_t *data, size_t length);
void UCON_Process();

#ifdef __cplusplus
}
#endif

#endif // UADC_H
