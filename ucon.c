/**
  ******************************************************************************
  * @file    ucon.c
  * @author  Khusainov Timur
  * @version 0.0.0.0
  * @date    22.08.2012
  * @brief   Uart (UART1) connection part
  ******************************************************************************
  * @attention
  * <h2><center>&copy; COPYRIGHT timyik@gmail.com </center></h2>
  ******************************************************************************
  */

#include <stdint.h>
#include <stddef.h>

#include "board.h"
#include "irq/interrupt.h"
#include "ucon.h"
#include "sys_timer.h"

//------------------------------------------------------------------------------
#define BSP_UCON_UART_BAUDRATE (19200)
//------------------------------------------------------------------------------
#define _UART_Enable() do { UART1->SR = 0; UART1->CR1 &=~ UART1_CR1_UARTD; } while(0)
#define _UART_Disable() do { UART1->CR1 |= UART1_CR1_UARTD; UART1->SR = 0; } while(0)
//------------------------------------------------------------------------------
#define _UART_EnableTx()  do { UART1->SR = 0; UART1->CR2 |= UART1_CR2_TEN; } while(0)
#define _UART_EnableRx()  do { UART1->SR = 0; UART1->CR2 |= UART1_CR2_REN; } while(0)
#define _UART_DisableTx() do { UART1->CR2 &= ~UART1_CR2_TEN; } while(0)
#define _UART_DisableRx() do { UART1->CR2 &= ~UART1_CR2_REN; } while(0)
#define _UART_IsEnableTx() (UART1->CR1 & USART_CR1_TE)
#define _UART_IsEnableRx() (UART1->CR1 & USART_CR1_RE)
//------------------------------------------------------------------------------
#define _UART_EnableTxIRQ()  do { UART1->SR = 0; UART1->CR2 |= UART1_CR2_TCIEN; } while(0)
#define _UART_EnableRxIRQ()  do { UART1->SR = 0; UART1->CR2 |= UART1_CR2_RIEN;  } while(0)
#define _UART_DisableTxIRQ() do { UART1->CR2 &= ~(UART1_CR2_TCIEN); } while(0)
#define _UART_DisableRxIRQ() do { UART1->CR2 &= ~(UART1_CR2_RIEN);  } while(0)
//------------------------------------------------------------------------------
#define _UART_GetData() ((uint8_t)UART1->DR)
#define _UART_SetData(_data) do { UART1->DR = (uint8_t)_data; } while(0)
#define _UART_GetError() ((uint8_t)(UART1->SR & (UART1_SR_PE|UART1_SR_FE|UART1_SR_NF|UART1_SR_OR)))
#define _UART_GetDummy() do { uint8_t _dummy = UART1->SR; _dummy = UART1->DR; } while(0)
//------------------------------------------------------------------------------
#define _UART_CheckEventRx() (UART1->SR & (UART1_SR_RXNE|UART1_SR_PE|UART1_SR_FE|UART1_SR_NF|UART1_SR_OR))
#define _UART_CheckEventTx() (UART1->SR & UART1_SR_TC)
#define _UART_ResetEventRx() do { UART1->SR &=~ (UART1_SR_RXNE|UART1_SR_PE|UART1_SR_FE|UART1_SR_NF|UART1_SR_OR); } while(0)
#define _UART_ResetEventTx() do { UART1->SR &=~ (UART1_SR_TC|UART1_SR_TXE); } while(0)
//------------------------------------------------------------------------------
volatile tUCON_OnAction UCON_OnAction = NULL;
//------------------------------------------------------------------------------
#define UCON_BUFFER_SIZE (64)

volatile static uint8_t UCON_DataBuffer[UCON_BUFFER_SIZE] = {0};

volatile static size_t  UCON_CountTx = 0;
volatile static size_t  UCON_CountRx = 0;
//------------------------------------------------------------------------------
volatile static tBSP_SysTimer UCON_TimeoutTimer = {0, 0};

#define UCON_TIMEOUT_WAIT_MESSAGE_MS (1000)
#define UCON_TIMEOUT_WAIT_RECEIVE_MS (2)

#define _IST_StartMessage() do { BSP_SysTimerNew(UCON_TimeoutTimer, UCON_TIMEOUT_WAIT_MESSAGE_MS); } while(0)
#define _IST_StartReceive() do { BSP_SysTimerNew(UCON_TimeoutTimer, UCON_TIMEOUT_WAIT_RECEIVE_MS); } while(0)

#define _IST_CheckEvent() (BSP_SysTimerCheck(UCON_TimeoutTimer))
#define _IST_ResetCounter() do { BSP_SysTimerRe(UCON_TimeoutTimer); } while(0)
//------------------------------------------------------------------------------
volatile static enum
{
	UCON_STATE_NOINIT = 0,
	UCON_STATE_STOP,
	UCON_STATE_EMPTY,
	UCON_STATE_DATA_WAIT,
	UCON_STATE_DATA_COMPL,
	UCON_STATE_ERROR
} UCON_TxState = UCON_STATE_NOINIT, UCON_RxState = UCON_STATE_NOINIT;
//------------------------------------------------------------------------------

/**
 * @brief  Call when byte received from UART
 * @param  None
 * @return None
 */
void UCON_IRQHandlerUART1_Rx()
{
	const uint8_t m_error = _UART_GetError();
	const uint8_t m_data  = _UART_GetData();

	if (!m_error)
	{
		UCON_DataBuffer[UCON_CountRx] = m_data;
		UCON_CountRx++;

		_IST_StartReceive();

		if (UCON_CountRx == UCON_BUFFER_SIZE)
		{
			_UART_DisableRx();
			UCON_RxState = UCON_STATE_DATA_COMPL;
		}
	}
	else
	{
		_UART_DisableRx();
		UCON_RxState = UCON_STATE_ERROR;
	}
}

/**
 * @brief  Call when byte transmit from UART
 * @param  None
 * @return None
 */
void UCON_IRQHandlerUART1_Tx()
{
	static size_t m_data_count = 0;
	m_data_count++;

	if (m_data_count != UCON_CountTx)
	{
		_UART_SetData(UCON_DataBuffer[m_data_count]);
		UCON_TxState = UCON_STATE_DATA_WAIT;
	}
	else
	{
		m_data_count = 0;
		_UART_DisableTx();

		UCON_TxState = UCON_STATE_DATA_COMPL;
	}

	_UART_ResetEventTx();
}

/**
 * @brief  Init slave uart
 * @param  pOnAction - callback
 * @return None
 */
void BSP_InitUCON(tUCON_OnAction pOnAction)
{
	//----------------------------------------------------------------------------
	// Init default pin state
	//----------------------------------------------------------------------------
	BSP_PIN_1(GPIOA, (1 << 4));
	BSP_PIN_INPZ(GPIOA, (1 << 4)); /* RX pin as input and Z-state */

	BSP_PIN_1(GPIOA, (1 << 5));
	BSP_PIN_OUTP(GPIOA, (1 << 5)); /* TX pin as output and push-pull */
	//----------------------------------------------------------------------------
	// Init UART1 peripheral
	//----------------------------------------------------------------------------
	UART1->CR1 = UART1_CR1_UARTD;  /* disable UART */
	
	static const union
	{
		uint16_t Result;
		struct
		{
			uint16_t BRR2_03 :4;
			uint16_t BRR1    :8;
			uint16_t BRR2_47 :4;
		};
	}_CALC_BAUDRATE = 
	{
		 /* recommended calculation at Reference Manual for STM8S */
		(uint16_t)(((float)F_CPU/(float)BSP_UCON_UART_BAUDRATE) + 0.5)
	};

	/* set baudrate */
	UART1->BRR2 = (uint8_t)(_CALC_BAUDRATE.BRR2_47 << 4)|(_CALC_BAUDRATE.BRR2_03 & 0x0f);
	UART1->BRR1 = (uint8_t) _CALC_BAUDRATE.BRR1;
	
	/* set the Clock Polarity, lock Phase, last Bit Clock pulse, stop bit 1 */
	UART1->CR3 = (UART1_CR3_CPOL|UART1_CR3_CPHA|UART1_CR3_LBCL);

	/* enable Parity Control: Even */
	UART1->CR1 |= UART1_CR1_PCEN|UART1_CR1_M|UART1_CR1_PIEN;

	UART1->CR4 = 0; /* reset register */
	UART1->CR5 = 0; /* reset register */

	_UART_EnableRxIRQ(); /* enable Receiver interrupt event */
	_UART_EnableTxIRQ(); /* enable Transmitter interrupt event */

	/* set interrupt vector handlers */
	BSP_InterruptSet(UCON_IRQHandlerUART1_Rx, BSP_IRQ_VECTOR_UART1_RX, bspitStd);
	BSP_InterruptSet(UCON_IRQHandlerUART1_Tx, BSP_IRQ_VECTOR_UART1_TX, bspitFast);
	//----------------------------------------------------------------------------
	// Init UCON start state
	//----------------------------------------------------------------------------
	UCON_OnAction = pOnAction; /* set action handlers */

	UCON_RxState = UCON_STATE_NOINIT;
	UCON_TxState = UCON_STATE_NOINIT;
	//----------------------------------------------------------------------------
}

/**
 * @brief  Set HW process priority
 * @param  priority - need priority
 * @return None
 */
void BSP_SetPriorityUCON(tBSP_Priority priority)
{
	BSP_SetPriority(BSP_IRQ_VECTOR_UART1_RX, priority);
	BSP_SetPriority(BSP_IRQ_VECTOR_UART1_TX, priority);
}

/**
 * @brief  Enable UCON receiver
 * @param  None
 * @return None
 */
void BSP_EnableUCON()
{
	UCON_RxState = UCON_STATE_STOP;
	UCON_TxState = UCON_STATE_STOP;

	_UART_GetDummy();
	_UART_Enable();
}

/**
 * @brief  Disable UCON receiver
 * @param  None
 * @return None
 */
void BSP_DisableUCON()
{
	UCON_RxState = UCON_STATE_NOINIT;
	UCON_TxState = UCON_STATE_NOINIT;

	_UART_GetDummy();
	_UART_Disable();
}

/**
 * @brief  Transmit data through UART
 * @param  data - ptr to data need transmit
 * @param  length - length of data need transmit
 * @return None
 */
void UCON_TransmitData(const uint8_t *data, size_t length)
{
	if (length <= UCON_BUFFER_SIZE)
	{
		for (UCON_CountTx = 0; UCON_CountTx != length; ++UCON_CountTx)
			UCON_DataBuffer[UCON_CountTx] = data[UCON_CountTx];

		UCON_TxState = UCON_STATE_DATA_WAIT;

		_UART_GetDummy();
		_UART_EnableTx();

		_UART_SetData(data[0]);
	}
	else
	{
		UCON_CountTx = 0;
		UCON_TxState = UCON_STATE_ERROR;
	}
}

/**
 * @brief  Receive data through UART
 * @param  data - dummy
 * @param  length - dummy
 * @return None
 */
void UCON_ReceiveData(uint8_t *data, size_t length)
{
	UCON_CountRx = 0;
	UCON_RxState = UCON_STATE_DATA_WAIT;

	_UART_GetDummy();
	_UART_EnableRx();

	_IST_StartMessage();
}

/**
 * @brief  Main UCON process
 * @param  None
 * @return None
 */
void UCON_Process()
{
	switch (UCON_RxState)
	{
		case UCON_STATE_NOINIT:
		case UCON_STATE_STOP:
		case UCON_STATE_EMPTY:
			break;

		case UCON_STATE_DATA_WAIT:
		{
			disableInterrupts();
			size_t _is_event = _IST_CheckEvent();
			enableInterrupts();

			if (_is_event)
			{
				UCON_RxState = UCON_STATE_EMPTY;
				_UART_DisableRx();

				if (UCON_OnAction)
				{
					tUCON_Action _action = UCON_CountRx ? UCON_ACTION_DATA_RX : UCON_ACTION_ERROR_RX;
					UCON_OnAction(_action, (void  *)UCON_DataBuffer, UCON_CountRx);
				}
			}
		}
		break;

		case UCON_STATE_DATA_COMPL:
		{
			UCON_RxState = UCON_STATE_EMPTY;

			if (UCON_OnAction)
				UCON_OnAction(UCON_ACTION_DATA_RX, (void  *)UCON_DataBuffer, UCON_CountRx);
		}
		break;

		case UCON_STATE_ERROR:
		{
			UCON_RxState = UCON_STATE_STOP;

			if (UCON_OnAction)
				UCON_OnAction(UCON_ACTION_ERROR_RX, NULL, 0);
		}
		break;
	}

	switch (UCON_TxState)
	{
		case UCON_STATE_NOINIT:
		case UCON_STATE_STOP:
		case UCON_STATE_EMPTY:
		case UCON_STATE_DATA_WAIT:
			break;

		case UCON_STATE_DATA_COMPL:
		{
			UCON_TxState = UCON_STATE_EMPTY;

			if (UCON_OnAction)
				UCON_OnAction(UCON_ACTION_DATA_TX, (void  *)UCON_DataBuffer, UCON_CountTx);
		}
		break;

		case UCON_STATE_ERROR:
		{
			UCON_TxState = UCON_STATE_STOP;

			if (UCON_OnAction)
				UCON_OnAction(UCON_ACTION_ERROR_TX, NULL, 0);
		}
		break;
	}
}