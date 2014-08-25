/**
  ******************************************************************************
  * @file    sys_timer.c
  * @author  Khusainov Timur
  * @version 0.0.0.4
  * @date    18.01.2012
  * @brief   System interval timer (ms)
  ******************************************************************************
  * @attention
  * <h2><center>&copy; COPYRIGHT timyik@gmail.com </center></h2>
  ******************************************************************************
  */

#include <stdint.h>
#include <stddef.h>

#include "board.h"
#include "irq/interrupt.h"
#include "sys_timer.h"

volatile tBSP_SysTimerType  BSP_SysTimer = 0;
volatile tBSP_SysTimerState BSP_SysTimerState = BSP_SYS_TIMER_STATE_NOINIT;

void SysTimer_IRQHandlerTIM1()
{
	++BSP_SysTimer;
	TIM4->SR1 &=~ TIM4_SR1_UIF;
}

void BSP_InitSysTimer()
{
	uint32_t m_clk_timeout = (uint32_t)((F_CPU * (1e-3)) * BSP_SYS_TIMER_DISCRETE_MS);
	
	/* timer value of prescaler */
	uint16_t m_pre = 0;
	
	/* if num of clock > timer max value => up the prescaler */
	while (m_clk_timeout/256)
	{
		m_clk_timeout >>= 1; // div by 2
		m_pre++;             // pow of 2
	}
	
	/* set interrupt handler for SysTimer discrete */
	BSP_InterruptSet(SysTimer_IRQHandlerTIM1, BSP_IRQ_VECTOR_TIM4_OVR_UIF, bspitFast);
	
	TIM4->CR1 &= ~(TIM4_CR1_ARPE|TIM4_CR1_OPM);
	
	/* set prescaler and timer counter */
	TIM4->PSCR = (uint8_t)(m_pre);
	TIM4->ARR  = (uint8_t)(m_clk_timeout);
	
	/* enable SysTimer discrete interrupt and clear status */
	TIM4->IER |= TIM4_IER_UIE;
	TIM4->SR1 = 0;
}

void BSP_StartSysTimer(void)
{
	TIM4->CR1 |= TIM4_CR1_CEN;
}

void BSP_StopSysTimer(void)
{
	TIM4->CR1 &=~ TIM4_CR1_CEN;
}