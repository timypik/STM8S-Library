/**
  ******************************************************************************
  * @file    board.c
  * @author  Khusainov Timur
  * @version v0.0.0.2
  * @date    18.01.2012
  * @brief   
  ******************************************************************************
  * @attention
  * <h2><center>&copy; COPYRIGHT timyik@gmail.com </center></h2>
  ******************************************************************************
  */

#include <stdint.h>
#include <stddef.h>

#include <stm8s_clk.h>
#include <stm8s_iwdg.h>

#include "board.h"
#include "irq/interrupt.h"

/**
 * @brief  Init MCU clock system
 * @param  None
 * @return None
 */
void BSP_InitCLK()
{
	//----------------------------------------------------------------------------
	uint16_t m_timeout; /* need for down counter */
	size_t m_operation_status; /* result of current opertion */
	//----------------------------------------------------------------------------
	// Start HSI (internal RC) and LSI (internal RC) oscillators
	//----------------------------------------------------------------------------
	/* MVR regulator ON in Active-halt mode */
	/* Low speed internal RC oscillator enable */
	/* Fast wakeup from Halt/Active-halt modes disabled */
	/* High speed internal RC oscillator enable */
	CLK->ICKR = CLK_ICKR_HSIEN|CLK_ICKR_LSIEN;

	m_timeout = 2048; /* wait for end opertion: delay of 2048 osc cycles */

	do /* wait until operation completion: LSI and HSI clock ready */
	{
		m_operation_status = ((CLK->ICKR & CLK_ICKR_LSIRDY) && (CLK->ICKR & CLK_ICKR_HSIRDY));
	}
	while ((m_operation_status == 0) && (--m_timeout != 0));
	//----------------------------------------------------------------------------
	// Start HSE (external crystal) oscillator
	//----------------------------------------------------------------------------
	CLK->ECKR = CLK_ECKR_HSEEN; /* High speed external crystal oscillator enable */

	m_timeout = 2048; /* wait for end opertion: delay of 2048 osc cycles */

	do /* wait until operation completion: HSE clock is stabilized and available */
	{
		m_operation_status = (CLK->ECKR & CLK_ECKR_HSERDY);
	}
	while ((m_operation_status == 0) && (--m_timeout != 0));
	//----------------------------------------------------------------------------
	// Select HSE as master clock
	//----------------------------------------------------------------------------
	CLK->SWCR = CLK_SWCR_SWEN; /* Enable clock switch execution */
	CLK->SWR = CLK_SOURCE_HSE; /* Select external clock source as master clock */

	m_timeout = 2048; /* wait for end opertion: delay of 2048 osc cycles */

	do /* wait until operation completion: HSE selected as master clock source */
	{
		m_operation_status = ((CLK->SWCR & CLK_SWCR_SWBSY) && (CLK->CMSR != CLK_SOURCE_HSE));
	}
	while ((m_operation_status != 0) && (--m_timeout != 0));
	//----------------------------------------------------------------------------
	// Configure CLK system: divider, CCO and etc
	//----------------------------------------------------------------------------
	CLK->CKDIVR = 0; /* No clock devider: fHSI = fHSI RC output, fCPU = fMASTER */
 	CLK->CCOR = CLK_CCOR_RESET_VALUE; /* CCO clock output disabled */
	CLK->SWIMCCR = CLK_SWIMCCR_RESET_VALUE; /* SWIM clock divided by 2 */
	//----------------------------------------------------------------------------
	// Enables the Clock Security System (CSS)
	//----------------------------------------------------------------------------
	CLK->CSSR = CLK_CSSR_CSSDIE; /* Enable CSS detection interrupt */
	CLK->CSSR |= CLK_CSSR_CSSEN; /* Enable CSS once, cannot be disable */

	/* Set interrupt handler */
	// BSP_InterruptSet(NULL, BSP_IRQ_VECTOR_CLK, bspitFast);
	//----------------------------------------------------------------------------
}

/**
 * @brief  Init WatchDog timer system
 * @param  None
 * @return None
 */
void BSP_InitWDT()
{
	//----------------------------------------------------------------------------
	// Init Independend wathcdog (IWDG)
	//----------------------------------------------------------------------------
	IWDG->KR = IWDG_KEY_ENABLE; /* Enable IWDG (the LSI will be enabled by hardware) */
	IWDG->KR = IWDG_WriteAccess_Enable; /* Enable write access to IWDG_PR and IWDG_RLR */
	//----------------------------------------------------------------------------
	// Config IWDG to 1000ms timeout period
	//----------------------------------------------------------------------------
	// По формуле:
	//     T = 2 * 1/fLSI * 2^(PR+2) * (RLR+1)
	//     RLR = (fLSI * T)/(2 * 2^(PR+2)) - 1
	// 
	// Известно, что:
	//     fLSI = 128kHz
	//     PR = 6 -> предделитель 256
	//     T  = 1 -> 1000мс
	// 
	// Подставляем значения:
	//     RLR = (128000 * 1)/(2 * 256) - 1
	//     RLR = 249
	// 
	// Время между последним обновлением IWDG и сбросом:
	//     D = T + (6 * 1/fLSI)
	//     D = 1 + 46,875 мкс
	//----------------------------------------------------------------------------
	IWDG->PR = IWDG_Prescaler_256; // (128kHz /2)/256 = 250 Hz
	IWDG->RLR = 249;               // (1/250)*249(F9) + 1 = 1000 ms
	//----------------------------------------------------------------------------
}

/**
 * @brief  Reset WatchDog timer system
 * @param  None
 * @return None
 */
void BSP_ResetWDT()
{
	IWDG->KR = IWDG_KEY_REFRESH; /* Reload IWDG counter */
}

/**
 * @brief  Set IRQ vector priority
 * @param  vector and need priority
 * @return None
 */
void BSP_SetPriority(uint8_t vector, tBSP_Priority priority)
{
	tBSP_InterruptPriority m_priority;

	switch(priority)
	{
		case BSP_PRIORITY_DEFAULT: m_priority = BSP_IP_DISABLE; break;
		case BSP_PRIORITY_LEVEL_0: m_priority = BSP_IP_LOW2;    break;
		case BSP_PRIORITY_LEVEL_1: m_priority = BSP_IP_LOWER1;  break;
		case BSP_PRIORITY_LEVEL_2: m_priority = BSP_IP_LOWEST0; break;
	}

	BSP_SetInterruptPriority(vector, m_priority);
}