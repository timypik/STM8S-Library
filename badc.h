/**
  ******************************************************************************
  * @file    adc.c
  * @author  Khusainov Timur
  * @version 0.0.0.1
  * @date    7.04.2014
  * @brief   Board Analog-to-Digital converter support
  ******************************************************************************
  * @attention
  * <h2><center>&copy; COPYRIGHT timyik@gmail.com </center></h2>
  ******************************************************************************
  */

#ifndef BADC_H
#define BADC_H
//------------------------------------------------------------------------------
typedef enum 
{ 
  BADC_ACTION_MEASURE,
  BADC_ACTION_ERROR,
} tBADC_Action;

typedef void (*tBADC_OnAction)(tBADC_Action action, uint16_t data);
//------------------------------------------------------------------------------
void BADC_DeInit();
void BADC_Init(tBADC_OnAction pOnAction);
uint16_t BADC_GetMeasureImmediately(size_t channel);
void BADC_GetMeasureLazily(size_t channel);
void BADC_Process();
//------------------------------------------------------------------------------
#endif // BADC_H