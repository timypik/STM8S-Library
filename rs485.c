/**
  ******************************************************************************
  * @file    rs485.c
  * @author  Khusainov Timur
  * @version 0.0.0.1
  * @date    18.01.2012
  * @brief   
  ******************************************************************************
  * @attention
  * <h2><center>&copy; COPYRIGHT timyik@gmail.com </center></h2>
  ******************************************************************************
  */

#include <stdint.h>
#include <stddef.h>

#include "board.h"
#include "irq/interrupt.h"
#include "rs485.h"

//------------------------------------------------------------------------------
#define _UART_EnableTx()  do { UART3->SR = 0; UART3->CR2 |= UART3_CR2_TEN; UART3->CR2 |= UART3_CR2_TCIEN; } while(0)
#define _UART_EnableRx()  do { UART3->SR = 0; UART3->CR2 |= UART3_CR2_REN; UART3->CR2 |= UART3_CR2_RIEN;  } while(0)
#define _UART_DisableTx() do { UART3->CR2 &= ~UART3_CR2_TCIEN; UART3->CR2 &= ~UART3_CR2_TEN; } while(0)
#define _UART_DisableRx() do { UART3->CR2 &= ~UART3_CR2_RIEN;  UART3->CR2 &= ~UART3_CR2_REN; } while(0)
//------------------------------------------------------------------------------
#define _UART_GetData()  (UART3->DR)
#define _UART_GetError() (UART3->SR & (UART3_SR_PE|UART3_SR_FE|UART3_SR_NF|UART3_SR_OR))
#define _UART_SetData(_data) do { UART3->DR = (uint8_t)(_data); } while(0)
//------------------------------------------------------------------------------
#define _UART_CheckEventRx() (UART3->SR & (UART3_SR_RXNE|UART3_SR_PE|UART3_SR_FE|UART3_SR_NF|UART3_SR_OR))
#define _UART_CheckEventTx() (UART3->SR & UART3_SR_TC)
#define _UART_ResetEventRx() do { UART3->SR &= ~(UART3_SR_RXNE|UART3_SR_PE|UART3_SR_FE|UART3_SR_NF|UART3_SR_OR); } while (0)
#define _UART_ResetEventTx() do { UART3->SR &= ~(UART3_SR_TC|UART3_SR_TXE); } while (0)
//------------------------------------------------------------------------------
#define _IST_Enable()       do { TIM3->CR1 |= TIM3_CR1_CEN; } while (0)
#define _IST_Disable()      do { TIM3->CR1 &=~TIM3_CR1_CEN; } while (0)
#define _IST_ResetCounter() do { TIM3->CNTRH = 0; TIM3->CNTRL = 0; } while (0)
#define _IST_ResetStatus()  do { TIM3->SR1 = 0; TIM3->SR2 = 0; } while (0)
#define _IST_CheckEvent()   (TIM3->SR1 & TIM3_SR1_UIF)
//------------------------------------------------------------------------------
tRS485_OnAction RS485_OnAction = NULL;
//------------------------------------------------------------------------------
static size_t RS485_CountTx = 0;
static size_t RS485_CountRx = 0;

static uint8_t * RS485_DataTx = NULL;
static uint8_t * RS485_DataRx = NULL;
//------------------------------------------------------------------------------
volatile static enum
{
	RS485_STATE_NOINIT = 0,
	RS485_STATE_IDLE,
	RS485_STATE_WAIT_TX,
	RS485_STATE_WAIT_RX,
	RS485_STATE_COMPLITE_TX,
	RS485_STATE_COMPLITE_RX,
	RS485_STATE_ERROR_TX,
	RS485_STATE_ERROR_RX,
} RS485_State = RS485_STATE_NOINIT;
//------------------------------------------------------------------------------
typedef union
{
	uint16_t Result;
	struct
	{
		uint16_t BRR2_03 :4;
		uint16_t BRR1    :8;
		uint16_t BRR2_47 :4;
	};
} tRS485_BaudCalc;

static const struct
{
	uint32_t Speed;
	tRS485_BaudCalc Result;
} RS485_BAUDRATE[RS485_SPEED_BAUDRATE_COUNT] = 
{
	/* recommended calculation at Reference Manual for STM8S */
	{   1200, (uint16_t)(((float)F_CPU/1200.0)   + 0.5) },
	{   4800, (uint16_t)(((float)F_CPU/4800.0)   + 0.5) },
	{   9600, (uint16_t)(((float)F_CPU/9600.0)   + 0.5) },
	{  19200, (uint16_t)(((float)F_CPU/19200.0)  + 0.5) },
	{  38400, (uint16_t)(((float)F_CPU/38400.0)  + 0.5) },
	{  57600, (uint16_t)(((float)F_CPU/57600.0)  + 0.5) },
	{ 115200, (uint16_t)(((float)F_CPU/115200.0) + 0.5) },
	{ 230400, (uint16_t)(((float)F_CPU/230400.0) + 0.5) },
};
//------------------------------------------------------------------------------
void _InitIntervalSymbolTimer(tRS485_Speed speed, tRS485_Parity parity, tRS485_Stopbit stopbits)
{
	// Calc prescaler and autoreloader value for timer
	
	/* 3,5 * ((Data(8) + Start-Stop(2) + Parity(1) + StopBit(1))/Badrate) */
	float m_time_3_5bit = 3.5 * ((8 + 2 + (parity?1:0) + (stopbits?1:0))/((float)RS485_BAUDRATE[speed].Speed));
	
	/* CPU clocks per 3,5 symbol */
	uint32_t m_clk_3_5bit = (uint32_t)(F_CPU * m_time_3_5bit);
	
	/* Timer value of prescaler: is power of 2 */
	uint16_t m_pre = 0;
	
	/* if num of clock > timer max value => up the prescaler */
	while (m_clk_3_5bit/65536)
	{
		m_clk_3_5bit /= 2;
		m_pre++;
	}
	
	/* Timer value of autoreloader */
	uint16_t m_arl = (uint16_t)(m_clk_3_5bit);
	
	// Configure timer
	
	/* Disable all and enable one pulse mode */
	TIM3->CR1 = TIM3_CR1_OPM;
	
	/* Set prescaler */
	TIM3->PSCR = m_pre;
	
	/* Set autoreloader */
	TIM3->ARRH = (uint8_t)(m_arl >> 8);
	TIM3->ARRL = (uint8_t)(m_arl & 0xFF);
	
	/* Generate an update event to reload the Prescaler value immediatly */
	TIM3->EGR |= TIM3_EGR_UG;
	
	/* Clear all flags */
	_IST_ResetCounter();
	_IST_ResetStatus();
	
	/* Enable update interrupt */
	TIM3->IER |= TIM3_IER_UIE;

	/* Clear all flags */
	_IST_ResetCounter();
	_IST_ResetStatus();
}
//------------------------------------------------------------------------------
void RS485_IRQHandlerUART3_Rx()
{
	static size_t m_data_count = 0;
	
	if (_IST_CheckEvent())
	{
		_IST_ResetStatus();

		RS485_CountRx = m_data_count;
		m_data_count  = 0;

		_UART_DisableRx();
		RS485_State  = RS485_STATE_COMPLITE_RX;
	}
	else
	{
		_IST_Disable();

		const uint8_t m_error = _UART_GetError();
		const uint8_t m_data  = _UART_GetData();

		if (!m_error)
		{
			RS485_DataRx[m_data_count] = m_data;
			m_data_count++;

			if (m_data_count != RS485_CountRx)
			{
				_IST_ResetCounter();
				_IST_ResetStatus();
				_IST_Enable();

				RS485_State = RS485_STATE_WAIT_RX;
			}
			else
			{
				RS485_CountRx = m_data_count;
				m_data_count  = 0;

				_UART_DisableRx();
				RS485_State = RS485_STATE_COMPLITE_RX;
			}
		}
		else
		{
			RS485_CountRx = m_data_count;
			m_data_count  = 0;

			_UART_DisableRx();
			RS485_State = RS485_STATE_ERROR_RX;
		}

		_UART_ResetEventRx();
	}
}
//------------------------------------------------------------------------------
void RS485_IRQHandlerUART3_Tx()
{
	static size_t m_data_count = 0;
	m_data_count++;

	if (m_data_count != RS485_CountTx)
	{
		_UART_SetData(RS485_DataTx[m_data_count]);

		RS485_State = RS485_STATE_WAIT_TX;
	}
	else
	{
		m_data_count = 0;

		BSP_DIR485_Rx();
		_UART_DisableTx();

		RS485_State = RS485_STATE_COMPLITE_TX;
	}

	_UART_ResetEventTx();
}
//------------------------------------------------------------------------------
void BSP_InitRS485(tRS485_Speed speed, tRS485_Parity parity, tRS485_Stopbit stopbits, tRS485_OnAction pOnAction)
{
	/* init direction control system */
	BSP_InitD485();
	BSP_DIR485_Rx();
	//----------------------------------------------------------------------------
	BSP_PIN_INPP(GPIOD, (1 << 5)); /* TX pin as input and pull-up */
	BSP_PIN_INPP(GPIOD, (1 << 6)); /* RX pin as input and pull-up */
	//----------------------------------------------------------------------------
	UART3->CR1 = UART3_CR1_UARTD; /* disable UART: 8-bit, no parity */
	UART3->CR2  = 0; /* reset register: disable transceiver and interrupts */
	UART3->CR3  = 0; /* reset register: 1-stopbit */
	UART3->CR4  = 0; /* reset register */
	UART3->CR6  = 0; /* reset register */

	tRS485_BaudCalc m_result = RS485_BAUDRATE[speed].Result;

	/* set baudrate */
	UART3->BRR2 = (uint8_t)(m_result.BRR2_47 << 4)|(m_result.BRR2_03 & 0x0f);
	UART3->BRR1 = (uint8_t) m_result.BRR1;

	switch (parity)
	{
		case RS485_PARITY_ODD:
			UART3->CR1 |= UART3_CR1_PS;
		case RS485_PARITY_EVEN:
			UART3->CR1 |= (UART3_CR1_M|UART3_CR1_PCEN|UART3_CR1_PIEN);
			break;
	}

	if (stopbits == RS485_STOPBIT_2)
		UART3->CR3 |= (1 << 5);

	/* set interrupt vector handlers */
	BSP_InterruptSet(RS485_IRQHandlerUART3_Tx, BSP_IRQ_VECTOR_UART3_TX, bspitStd);
	BSP_InterruptSet(RS485_IRQHandlerUART3_Rx, BSP_IRQ_VECTOR_UART3_RX, bspitStd);
	//----------------------------------------------------------------------------
	/* setup 3,5 interval timer */
	_InitIntervalSymbolTimer(speed, parity, stopbits);

	/* set 3,5 interval timer vector handler */
	BSP_InterruptSet(RS485_IRQHandlerUART3_Rx, BSP_IRQ_VECTOR_TIM3_OVR_UIF, bspitStd);
	//----------------------------------------------------------------------------
	/* set action handlers */
	RS485_OnAction = pOnAction;
	
	RS485_State = RS485_STATE_IDLE;
}
//------------------------------------------------------------------------------
void BSP_SetPriority485(tBSP_Priority priority)
{
	BSP_SetPriority(BSP_IRQ_VECTOR_UART3_TX,     priority);
	BSP_SetPriority(BSP_IRQ_VECTOR_UART3_RX,     priority);
	BSP_SetPriority(BSP_IRQ_VECTOR_TIM3_OVR_UIF, priority);
}
//------------------------------------------------------------------------------
void BSP_EnableRS485()
{
	UART3->CR1 &=~ UART3_CR1_UARTD; /* disable UART */
	RS485_State = RS485_STATE_IDLE;
}
//------------------------------------------------------------------------------
void BSP_DisableRS485()
{
	UART3->CR1 |=  UART3_CR1_UARTD; /* enable UART */
	RS485_State = RS485_STATE_IDLE;
}
//------------------------------------------------------------------------------
void RS485_TransmitData(const uint8_t * data, size_t length)
{
	RS485_DataTx  = (uint8_t *)data;
	RS485_CountTx = length;

	RS485_State = RS485_STATE_WAIT_TX;

	_UART_EnableTx();
	BSP_DIR485_Tx();

	_UART_SetData(data[0]);
}
//------------------------------------------------------------------------------
void RS485_ReceiveData(uint8_t * const data, size_t length)
{
	RS485_DataRx  = (uint8_t *)data;
	RS485_CountRx = length;

	RS485_State = RS485_STATE_WAIT_RX;

	_UART_EnableRx();
	BSP_DIR485_Rx();
}
//------------------------------------------------------------------------------
void RS485_Process()
{
	switch (RS485_State)
	{
		case RS485_STATE_NOINIT:
		case RS485_STATE_IDLE:
		case RS485_STATE_WAIT_RX:
		case RS485_STATE_WAIT_TX:
			break;
		
		case RS485_STATE_COMPLITE_RX:
		{
			RS485_State = RS485_STATE_IDLE;

			if (RS485_OnAction)
				RS485_OnAction(RS485_ACTION_DATA_RX, RS485_DataRx, RS485_CountRx);
		}
		break;

		case RS485_STATE_COMPLITE_TX:
		{
			RS485_State = RS485_STATE_IDLE;

			if (RS485_OnAction)
				RS485_OnAction(RS485_ACTION_DATA_TX, RS485_DataTx, RS485_CountTx);
		}
		break;

		case RS485_STATE_ERROR_RX:
		{
			RS485_State = RS485_STATE_IDLE;

			if (RS485_OnAction)
				RS485_OnAction(RS485_ACTION_ERROR_RX, NULL, RS485_CountRx);
		}
		break;

		case RS485_STATE_ERROR_TX:
		{
			RS485_State = RS485_STATE_IDLE;

			if (RS485_OnAction)
				RS485_OnAction(RS485_ACTION_ERROR_TX, NULL, RS485_CountTx);
		}
		break;
	}
}
//------------------------------------------------------------------------------