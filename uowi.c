/**
  ******************************************************************************
  * @file    uowi.c
  * @author  Khusainov Timur
  * @version 0.0.0.0
  * @date    31.10.2012
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
#include "uowi.h"

volatile tUOWI_State UOWI_State = UOWI_STATE_NOINIT;
volatile uint8_t UOWI_TxData[UOWI_TX_DATA_MAX_LENGTH];
volatile uint8_t UOWI_RxData[UOWI_RX_DATA_MAX_LENGTH];
volatile uint16_t UOWI_TxLen = 0;
volatile uint16_t UOWI_RxLen = 0;

typedef enum
{
	UOWI_MODE_DETECT,
	UOWI_MODE_RX_TX
} tUOWI_Mode;

void UOWI_IRQHandlerUART1_Rx(void);

void UOWI_SetMode(tUOWI_Mode mode)
{
	UART1->CR1 |= UART1_CR1_UARTD; /* disable UART peripheral */
	
	static const union
	{
		uint16_t Result;
		struct
		{
			uint16_t BRR2_03 :4;
			uint16_t BRR1    :8;
			uint16_t BRR2_47 :4;
		};
	} CalcBaud[2] = 
	{
		{(uint16_t)(((float)F_CPU/(float)9600)   + 0.5)}, /* generate detect presence */
		{(uint16_t)(((float)F_CPU/(float)115200) + 0.5)}, /* write/read single bit */
	};
	
	/* get magic value */
	const uint8_t m_brr2 = (uint8_t)(CalcBaud[mode].BRR2_47 << 4)|(CalcBaud[mode].BRR2_03 & 0x0f);
	const uint8_t m_brr1 = (uint8_t) CalcBaud[mode].BRR1;
	
	UART1->BRR2 = m_brr2; /* set new baudrate */
	UART1->BRR1 = m_brr1; /* set new baudrate */
	
	/* clear status and data register */
	uint8_t _dummy = UART1->SR;
	_dummy = UART1->DR;
	
	UART1->CR1 &=~ UART1_CR1_UARTD; /* enable UART peripheral */
}

void UOWI_Init()
{
	//----------------------------------------------------------------------------
	// Init port pinouts
	//----------------------------------------------------------------------------
	BSP_OWI_PORT->ODR |= BSP_OWI_P_TX;  // set 1
	BSP_OWI_PORT->DDR |= BSP_OWI_P_TX;  // as output
	BSP_OWI_PORT->CR1 |= BSP_OWI_P_TX;  // push-pull
	BSP_OWI_PORT->CR2 |= BSP_OWI_P_TX;  // fast mode

	BSP_OWI_PORT->ODR &= ~BSP_OWI_P_RX; // set 0
	BSP_OWI_PORT->DDR &= ~BSP_OWI_P_RX; // as input
	BSP_OWI_PORT->CR1 &= ~BSP_OWI_P_RX; // floating mode, Z-state
	BSP_OWI_PORT->CR2 &= ~BSP_OWI_P_TX; // no interrupt
	//----------------------------------------------------------------------------
	BSP_InterruptSet(UOWI_IRQHandlerUART1_Rx, BSP_IRQ_VECTOR_UART1_RX, bspitStd);
	//----------------------------------------------------------------------------
	// Init UART peripheral
	//----------------------------------------------------------------------------
	UART1->CR1 = UART1_CR1_UARTD; /* disable UART peripheral */

	/* enable receiver, transmitter and Rx interrupt */
	UART1->CR2 = UART1_CR2_REN|UART1_CR2_TEN|UART1_CR2_RIEN;
	
	/* Set the Clock Polarity, lock Phase, last Bit Clock pulse, stop bit 1 */
	UART1->CR3 = UART1_CR3_CPOL|UART1_CR3_CPHA|UART1_CR3_LBCL;
	
	UART1->CR4 = 0; /* clear register */
	UART1->CR5 = 0; /* clear register */

	/* enable half-duplex (single wire) mode */
	//UART1->CR5 = UART1_CR5_HDSEL;
	
	/* clear status and data register */
	uint8_t _dummy = UART1->SR;
	_dummy = UART1->DR;
	//----------------------------------------------------------------------------
}

tUOWI_State UOWI_GetState()
{
	return UOWI_State;
}

uint8_t UOWI_IsDetect()
{
	return (UOWI_State == UOWI_STATE_DETECT_COMPLETE);
}

uint8_t UOWI_IsSend()
{
	return (UOWI_State == UOWI_STATE_SEND_COMPLETE);
}

uint8_t UOWI_IsRecv()
{
	return (UOWI_State == UOWI_STATE_RECV_COMPLETE);
}

uint8_t UOWI_IsBusy()
{
	if (UOWI_State != UOWI_STATE_STATUS_EMPTY)
	{
		UART1->DR  = UOWI_CMD_READ_STATE;
		UOWI_State = UOWI_STATE_STATUS_BUSY;
	}
	
	return (UOWI_State == UOWI_STATE_STATUS_EMPTY);
}

void UOWI_StartDetect()
{
	UOWI_SetMode(UOWI_MODE_DETECT);
	UART1->DR  = UOWI_CMD_DETECT;
	UOWI_State = UOWI_STATE_DETECT_WAIT;
}

void UOWI_StartSend()
{
	UOWI_SetMode(UOWI_MODE_DETECT);
	UART1->DR  = UOWI_CMD_DETECT;
	UOWI_State = UOWI_STATE_SEND_START;
}

void UOWI_SendData(const uint8_t *pdata, uint16_t txlen, uint16_t rxlen)
{
	if (pdata && txlen < UOWI_TX_DATA_MAX_LENGTH && rxlen < UOWI_RX_DATA_MAX_LENGTH)
	{
		UOWI_TxLen = txlen;
		UOWI_RxLen = rxlen;
		
		uint8_t *_out_buf = (uint8_t *)(UOWI_TxData);
		
		while (txlen--)
			*(_out_buf++) = *(pdata++);
		
		UOWI_StartSend();
	}
}

void UOWI_RecvData(uint8_t * pdata, uint16_t rxlen)
{
	if (rxlen <= UOWI_RxLen)
	{
		uint8_t *_in_buf = (uint8_t *)(UOWI_RxData);
		
		while (rxlen--)
			*(pdata++) = *(_in_buf++);
	}
}

void UOWI_IRQHandlerUART1_Rx(void)
{
	uint8_t m_err  = UART1->SR & (UART1_SR_NF|UART1_SR_PE|UART1_SR_FE|UART1_SR_OR);
	uint8_t m_data = UART1->DR;
	
	static uint16_t _bit_counter;
	static uint8_t _current_byte;
	
	switch (UOWI_State)
	{
		case UOWI_STATE_DETECT_WAIT:
		case UOWI_STATE_DETECT_ERROR:
		{
			if (m_data == UOWI_RET_DETECT_ERROR)
			{
				UOWI_State = UOWI_STATE_DETECT_ERROR;
				UART1->DR  = UOWI_CMD_DETECT;
			}
			else
				UOWI_State = UOWI_STATE_DETECT_COMPLETE;
		}
		break;
		
		case UOWI_STATE_SEND_START:
		case UOWI_STATE_SEND_ERROR:
		{
			if (m_data == UOWI_RET_DETECT_ERROR)
			{
				UOWI_State = UOWI_STATE_SEND_ERROR;
				UART1->DR  = UOWI_CMD_DETECT;
				
				break;
			}
			else
			{
				UOWI_State = UOWI_STATE_SEND_WAIT;
				UOWI_SetMode(UOWI_MODE_RX_TX);
				
				_bit_counter = 0;
			}
		}
		case UOWI_STATE_SEND_WAIT:
		{
			uint16_t _tx_byte_num = (_bit_counter >> 3);
			
			if (_tx_byte_num != UOWI_TxLen)
			{
				if ((_bit_counter & 0x7) == 0)
					_current_byte = UOWI_TxData[_tx_byte_num];
				
				UART1->DR = (_current_byte & 1) ? UOWI_CMD_WRITE_BIT1 : UOWI_CMD_WRITE_BIT0;
				
				_current_byte >>= 1;
				_bit_counter++;
			}
			else
			{
				if (UOWI_RxLen)
				{
					UART1->DR  = UOWI_CMD_READ_BIT;
					UOWI_State = UOWI_STATE_RECV_WAIT;
					
					_bit_counter  = 0;
					_current_byte = 0;
				}
				else
					UOWI_State = UOWI_STATE_SEND_COMPLETE;
			}
		}
		break;
			
		case UOWI_STATE_RECV_WAIT:
		{
			_bit_counter++;
			_current_byte >>= 1;
			
			if (m_data == UOWI_RET_READ_BIT1)
				_current_byte |= 0x80;
			
			if ((_bit_counter & 0x7) == 0)
			{
				uint16_t _rx_byte_num = (_bit_counter >> 3);
				
				UOWI_RxData[_rx_byte_num - 1] = _current_byte;
				
				if (_rx_byte_num == UOWI_RxLen)
				{
					UOWI_State = UOWI_STATE_RECV_COMPLETE;
				}
				else
				{
					UART1->DR = UOWI_CMD_READ_BIT;
					_current_byte = 0;
				}
			}
			else
				UART1->DR = UOWI_CMD_READ_BIT;
		}
		break;
		
		case UOWI_STATE_STATUS_BUSY:
		{
			if (m_data == UOWI_RET_STATE_EMPTY)
				UOWI_State = UOWI_STATE_STATUS_EMPTY;
		}
		break;
	}
	
	UART1->SR &=~ UART1_SR_RXNE;
}
