/**
  ******************************************************************************
  * @file    interrupt.c
  * @author  Khusainov Timur
  * @version 0.0.0.2
  * @date    27.06.2012
  * @brief   Board interrupt sub-system
  ******************************************************************************
  * @attention
  * <h2><center>&copy; COPYRIGHT timyik@gmail.com </center></h2>
  ******************************************************************************
  */

#include <stdint.h>
#include <stddef.h>

#include <stm8s.h>
#include "interrupt.h"

volatile tBSP_Interrupt BSP_InterruptsMap[] =
{
	/* RESET */
	/* TRAP */
	{bspitFast, BSP_InterruptException}, /* IRQ_0  - TLI         */
	{bspitFast, BSP_InterruptException}, /* IRQ_1  - AWU         */
	{bspitFast, BSP_InterruptException}, /* IRQ_2  - CLK         */
	{bspitFast, BSP_InterruptException}, /* IRQ_3  - EXTI0       */
	{bspitFast, BSP_InterruptException}, /* IRQ_4  - EXTI1       */
	{bspitFast, BSP_InterruptException}, /* IRQ_5  - EXTI2       */
	{bspitFast, BSP_InterruptException}, /* IRQ_6  - EXTI3       */
	{bspitFast, BSP_InterruptException}, /* IRQ_7  - EXTI4       */
	{bspitFast, BSP_InterruptException}, /* IRQ_8  - beCAN RX    */
	{bspitFast, BSP_InterruptException}, /* IRQ_9  - beCAN TX    */
	{bspitFast, BSP_InterruptException}, /* IRQ_10 - SPI         */
	{bspitFast, BSP_InterruptException}, /* IRQ_11 - TIM1 U/O    */
	{bspitFast, BSP_InterruptException}, /* IRQ_12 - TIM1 CAPCAM */
	{bspitFast, BSP_InterruptException}, /* IRQ_13 - TIM2 U/O    */
	{bspitFast, BSP_InterruptException}, /* IRQ_14 - TIM2 CAPCAM */
	{bspitFast, BSP_InterruptException}, /* IRQ_15 - TIM3 U/O    */
	{bspitFast, BSP_InterruptException}, /* IRQ_16 - TIM3 CAPCAM */
	{bspitFast, BSP_InterruptException}, /* IRQ_17 - UART1 TX    */
	{bspitFast, BSP_InterruptException}, /* IRQ_18 - UART1 RX    */
	{bspitFast, BSP_InterruptException}, /* IRQ_19 - I2C         */
	{bspitFast, BSP_InterruptException}, /* IRQ_20 - UART3 TX    */
	{bspitFast, BSP_InterruptException}, /* IRQ_21 - UART3 RX    */
	{bspitFast, BSP_InterruptException}, /* IRQ_22 - ADC2        */
	{bspitFast, BSP_InterruptException}, /* IRQ_23 - TIM4        */
	{bspitFast, BSP_InterruptException}, /* IRQ_24 - Flash       */
	{bspitFast, BSP_InterruptException}, /* IRQ_25 - RESERVED    */
	{bspitFast, BSP_InterruptException}, /* IRQ_26 - RESERVED    */
	{bspitFast, BSP_InterruptException}, /* IRQ_27 - RESERVED    */
	{bspitFast, BSP_InterruptException}, /* IRQ_28 - RESERVED    */
	{bspitFast, BSP_InterruptException}, /* IRQ_29 - RESERVED    */
};

void BSP_InterruptException(void)
{	
	while(1);
}

void BSP_InterruptPriority(uint8_t vector, uint8_t priority)
{
	uint8_t m_reg  = ((vector & 3) << 1);
	uint8_t m_mask = ~(BSP_IP_DISABLE << m_reg);
	uint8_t m_prio = (priority << m_reg);
	
	switch (vector >> 2)
	{
		case 0: ITC->ISPR1 &= m_mask; ITC->ISPR1 |= m_prio; break;
		case 1: ITC->ISPR2 &= m_mask; ITC->ISPR2 |= m_prio; break;
		case 2: ITC->ISPR3 &= m_mask; ITC->ISPR3 |= m_prio; break;
		case 3: ITC->ISPR4 &= m_mask; ITC->ISPR4 |= m_prio; break;
		case 4: ITC->ISPR5 &= m_mask; ITC->ISPR5 |= m_prio; break;
		case 5: ITC->ISPR6 &= m_mask; ITC->ISPR6 |= m_prio; break;
		case 6: ITC->ISPR7 &= m_mask; ITC->ISPR7 |= m_prio; break;
	}
}