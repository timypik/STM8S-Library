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

#include <stdint.h>
#include <stddef.h>

#include <stm8s_adc2.h>

#include "board.h"
#include "irq/interrupt.h"
#include "sys_timer.h"

#include "badc.h"

//------------------------------------------------------------------------------
// Technical characteristics of 10-bit ADC from STM8S208
//------------------------------------------------------------------------------
// fADC = 1-4 MHz (3.0 to 5.5V)
// fADC = 1-6 MHz (4.5 to 5.5V)
//------------------------------------------------------------------------------
// tS = 0.75us (fADC = 4MHz)
// tS = 0.50us (fADC = 6MHz)
//------------------------------------------------------------------------------
// tSTAB = 7us
//------------------------------------------------------------------------------
// tCONV = 3.50us (fADC = 4MHz)
// tCONV = 2.33us (fADC = 6MHz)
// tCONV = 14 of 1/fADC
//------------------------------------------------------------------------------
// tFIRST = tSTAB + tCONV
//------------------------------------------------------------------------------
#define _ADC_Enable()  do { ADC2->CR1 |=  ADC2_CR1_ADON; } while(0)
#define _ADC_Disable() do { ADC2->CR1 &=~ ADC2_CR1_ADON; } while(0)
//------------------------------------------------------------------------------
#define _ADC_EnableIRQ()  do { ADC2->CSR |=  ADC2_CSR_EOCIE; } while(0)
#define _ADC_DisableIRQ() do { ADC2->CSR &=~ ADC2_CSR_EOCIE; } while(0)
//------------------------------------------------------------------------------
#define _ADC_SetRightAligement() do { ADC2->CR2 |=  ADC2_CR2_ALIGN; } while(0)
#define _ADC_SetLeftAligement()  do { ADC2->CR2 &=~ ADC2_CR2_ALIGN; } while(0)

#define _ADC_DATA_ALIGNMENT_RIGHT
//#define _ADC_DATA_ALIGNMENT_LEFT

#if defined (_ADC_DATA_ALIGNMENT_RIGHT)
# define _ADC_SetAligement() _ADC_SetRightAligement()
#elif defined(_ADC_DATA_ALIGNMENT_LEFT)
# define _ADC_SetAligement() _ADC_SetLeftAligement()
#else 
# error "[badc.c] ADC2 select Aligement!"
#endif
//------------------------------------------------------------------------------
#define _ADC_SetContinuousMode() do { ADC2->CR1 |=  ADC2_CR1_CONT; } while(0)
#define _ADC_SetSingleMode()     do { ADC2->CR1 &=~ ADC2_CR1_CONT; } while(0)
//------------------------------------------------------------------------------
#define _ADC_SelectChannel(_chnl) do { ADC2->CSR &=~ ADC2_CSR_CH; ADC2->CSR |= ((_chnl) & ADC2_CSR_CH); } while(0)
#define _ADC_SetPrescaler(_presc) do { ADC2->CR1 &=~ ADC2_CR1_SPSEL; ADC2->CR1 |= ((_presc) & ADC2_CR1_SPSEL); } while(0)

#define _ADC_CurrentChannel() (ADC2->CSR & ADC2_CSR_CH)
//------------------------------------------------------------------------------
#define _ADC_DisableSTRG(_chnl) do { \
                                  if (_chnl < 8) ADC2->TDRL |= (1 << _chnl); \
                                  else ADC2->TDRH |= (1 << (_chnl - 8)); \
                                } while(0)

#define _ADC_EnableSTRG(_chnl) do { \
                                  if (_chnl < 8) ADC2->TDRL &= ~(1 << _chnl); \
                                  else ADC2->TDRH &= ~(1 << (_chnl - 8)); \
                                } while(0)
//------------------------------------------------------------------------------
#define _ADC_StartMeasure() do { ADC2->CR1 |=  ADC2_CR1_ADON; } while(0)
#define _ADC_StopMeasure()  do { ADC2->CR1 &=~ ADC2_CR1_ADON; } while(0)
//------------------------------------------------------------------------------
#define _ADC_CheckMeasureEvent() (ADC2->CSR & ADC2_CSR_EOC)
#define _ADC_ClearMeasureEvent() do { ADC2->CSR &=~ ADC2_CSR_EOC; } while(0)
//------------------------------------------------------------------------------
#define _ADC_AREF (5.f)
#define _ADC_ARES (10)
#define _ADC_ALSB (_ADC_AREF/(1<<_ADC_ARES))

#define _ADC_CodeToVoltage(_code) (_ADC_ALSB*(float)(_code))
#define _ADC_VoltageToCode(_volt) ((uint16_t)(_volt/_ADC_ALSB + 0.5))
//------------------------------------------------------------------------------
static tBSP_SysTimer BADC_TimeoutTimer = {0, 0};
#define BADC_TIMEOUT_WAIT_MEASURE_MS (1)

#define _IST_StartMeasure() do { BSP_SysTimerNew(BADC_TimeoutTimer, BADC_TIMEOUT_WAIT_MEASURE_MS); } while(0)
#define _IST_ResetMeasure() do { BSP_SysTimerRe(BADC_TimeoutTimer); } while(0)
#define _IST_CheckEvent() (BSP_SysTimerCheck(BADC_TimeoutTimer))
//------------------------------------------------------------------------------
volatile static enum
{
	BADC_STATE_NOINIT = 0,
	BADC_STATE_IDLE,
	BADC_STATE_WAIT,
	BADC_STATE_COMPLETE,
	BADC_STATE_ERROR,
} BADC_State = BADC_STATE_NOINIT;

static tBADC_OnAction BADC_OnAction = NULL;
static uint16_t BADC_Measure;
//------------------------------------------------------------------------------
static inline uint16_t _ADC_ReadMeasure()
{
	uint16_t m_result;

	#if defined (_ADC_DATA_ALIGNMENT_RIGHT)
	{
		uint8_t _drl = (ADC2->DRL & 0xff); /* read LSB first */
		uint8_t _drh = (ADC2->DRH & 0x03); /* then read MSB  */

		m_result = ((_drh << 8)|(_drl));
	}
	#elif defined(_ADC_DATA_ALIGNMENT_LEFT)
	{
		uint8_t _drh = (ADC2->DRH & 0xff); /* read MSB first */
		uint8_t _drl = (ADC2->DRL & 0x03); /* then read LSB  */

		m_result = ((_drh << 2)|(_drl));
	}
	#else
	{
		m_result = 0;	
	}
	#endif

	return m_result;
}
//------------------------------------------------------------------------------
void BADC_IRQHandlerADC2()
{
	BADC_Measure = _ADC_ReadMeasure();
	_ADC_DisableSTRG(_ADC_CurrentChannel());

	BADC_State = BADC_STATE_COMPLETE;
	_ADC_ClearMeasureEvent(); /* reset end of conversion flag */
}
//------------------------------------------------------------------------------
void BADC_DeInit()
{
	ADC2->CSR  = ADC2_CSR_RESET_VALUE;
	ADC2->CR1  = ADC2_CR1_RESET_VALUE;
	ADC2->CR2  = ADC2_CR2_RESET_VALUE;
	ADC2->TDRH = ADC2_TDRH_RESET_VALUE;
	ADC2->TDRL = ADC2_TDRL_RESET_VALUE;
}

void BADC_Init(tBADC_OnAction pOnAction)
{
	_ADC_Disable(); /* disable ADC peripheral */

	_ADC_SetAligement();  /* set data alignment type */
	_ADC_SetSingleMode(); /* set the conversion mode */
	_ADC_SetPrescaler(ADC2_PRESSEL_FCPU_D18); /* Prescaler selection fADC = fcpu/18 */

	_ADC_Enable(); /* enable ADC peripheral */

	BADC_OnAction = pOnAction;
	BADC_State = BADC_STATE_IDLE;
}

uint16_t BADC_GetMeasureImmediately(size_t channel)
{
	_ADC_DisableSTRG(channel); /* disable Schmitt Trigger */
	_ADC_SelectChannel(channel); /* select target channel */

	_ADC_ClearMeasureEvent(); /* reset end of conversion flag */
	_ADC_StartMeasure(); /* start measure */

	uint16_t m_timeout = 500; /* need for down counter */
	uint8_t m_operation_status; /* result of current opertion */

	do /* wait measure end */
	{
		m_operation_status = _ADC_CheckMeasureEvent();
	}
	while ((m_operation_status == 0) && (--m_timeout != 0));

	if ((m_timeout == 0) && (m_operation_status == 0))
		BADC_Measure = 0;
	else
		BADC_Measure = _ADC_ReadMeasure();

	_ADC_EnableSTRG(channel); /* enable Schmitt Trigger */

	return BADC_Measure;
}

void BADC_GetMeasureLazily(size_t channel)
{
	_ADC_DisableSTRG(channel); /* disable Schmitt Trigger */
	_ADC_SelectChannel(channel); /* select target channel */

	_ADC_ClearMeasureEvent(); /* reset end of conversion flag */
	_IST_StartMeasure(); /* start timeout timer */

	_ADC_EnableIRQ(); /* enable end of conversion interrupt */
	_ADC_StartMeasure(); /* start measure */
}
//------------------------------------------------------------------------------
void BADC_Process()
{
	switch(BADC_State)
	{
		case BADC_STATE_WAIT:
		{
			if (_ADC_CheckMeasureEvent())
			{
				BADC_State = BADC_STATE_IDLE;
				BADC_OnAction(BADC_ACTION_MEASURE, BADC_Measure);
			}
			else if (_IST_CheckEvent())
			{
				BADC_State = BADC_STATE_IDLE;
				BADC_OnAction(BADC_ACTION_ERROR, BADC_Measure);
			}
		}
		break;

		case BADC_STATE_COMPLETE:
		{
			BADC_State = BADC_STATE_IDLE;
			BADC_OnAction(BADC_ACTION_MEASURE, BADC_Measure);
		}
		break;

		case BADC_STATE_ERROR:
		{
			BADC_State = BADC_STATE_IDLE;
			BADC_OnAction(BADC_ACTION_ERROR, BADC_Measure);
		}
		break;
	}
}
//------------------------------------------------------------------------------