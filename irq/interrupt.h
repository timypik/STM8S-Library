/**
  ******************************************************************************
  * @file    interrupt.h
  * @author  Khusainov Timur
  * @version 0.0.0.1
  * @date    27.06.2012
  * @brief   Board interrupt sub-system
  ******************************************************************************
  * @attention
  * <h2><center>&copy; COPYRIGHT timyik@gmail.com </center></h2>
  ******************************************************************************
  */

#ifndef BSP_INTERRUPTS_H
#define BSP_INTERRUPTS_H
//------------------------------------------------------------------------------
#if defined(__IAR_SYSTEMS_ICC__) || defined(__ICCSTM8__) || defined (__IAR_SYSTEMS_ASM__)
#define BSP_IRQ_VECTOR_BASE (2)
#else
#define BSP_IRQ_VECTOR_BASE (0)
#endif
//------------------------------------------------------------------------------
#ifdef BSP_IRQ_VECTOR_FULL
#define BSP_IRQ_VECTOR_TLI               (BSP_IRQ_VECTOR_BASE + 0)
#define BSP_IRQ_VECTOR_AWU               (BSP_IRQ_VECTOR_BASE + 1)
#define BSP_IRQ_VECTOR_CLK_CSS           (BSP_IRQ_VECTOR_BASE + 2)
#define BSP_IRQ_VECTOR_CLK_SWITCH        (BSP_IRQ_VECTOR_BASE + 2)
#define BSP_IRQ_VECTOR_EXTI0             (BSP_IRQ_VECTOR_BASE + 3)
#define BSP_IRQ_VECTOR_EXTI1             (BSP_IRQ_VECTOR_BASE + 4)
#define BSP_IRQ_VECTOR_EXTI2             (BSP_IRQ_VECTOR_BASE + 5)
#define BSP_IRQ_VECTOR_EXTI3             (BSP_IRQ_VECTOR_BASE + 6)
#define BSP_IRQ_VECTOR_EXTI4             (BSP_IRQ_VECTOR_BASE + 7)
#define BSP_IRQ_VECTOR_beCAN_FMP         (BSP_IRQ_VECTOR_BASE + 8)
#define BSP_IRQ_VECTOR_beCAN_FULL        (BSP_IRQ_VECTOR_BASE + 8)
#define BSP_IRQ_VECTOR_beCAN_FOVR        (BSP_IRQ_VECTOR_BASE + 8)
#define BSP_IRQ_VECTOR_beCAN_EWGF        (BSP_IRQ_VECTOR_BASE + 9)
#define BSP_IRQ_VECTOR_beCAN_EPVF        (BSP_IRQ_VECTOR_BASE + 9)
#define BSP_IRQ_VECTOR_beCAN_BOFF        (BSP_IRQ_VECTOR_BASE + 9)
#define BSP_IRQ_VECTOR_beCAN_LEC0        (BSP_IRQ_VECTOR_BASE + 9)
#define BSP_IRQ_VECTOR_beCAN_LEC1        (BSP_IRQ_VECTOR_BASE + 9)
#define BSP_IRQ_VECTOR_beCAN_LEC2        (BSP_IRQ_VECTOR_BASE + 9)
#define BSP_IRQ_VECTOR_beCAN_RQCP0       (BSP_IRQ_VECTOR_BASE + 9)
#define BSP_IRQ_VECTOR_beCAN_RQCP1       (BSP_IRQ_VECTOR_BASE + 9)
#define BSP_IRQ_VECTOR_beCAN_RQCP2       (BSP_IRQ_VECTOR_BASE + 9)
#define BSP_IRQ_VECTOR_beCAN_WKUI        (BSP_IRQ_VECTOR_BASE + 9)
#define BSP_IRQ_VECTOR_SPI_TXE           (BSP_IRQ_VECTOR_BASE + 10)
#define BSP_IRQ_VECTOR_SPI_RXNE          (BSP_IRQ_VECTOR_BASE + 10)
#define BSP_IRQ_VECTOR_SPI_WKUP          (BSP_IRQ_VECTOR_BASE + 10)
#define BSP_IRQ_VECTOR_SPI_MODF          (BSP_IRQ_VECTOR_BASE + 10)
#define BSP_IRQ_VECTOR_SPI_OVR           (BSP_IRQ_VECTOR_BASE + 10)
#define BSP_IRQ_VECTOR_SPI_CRCERR        (BSP_IRQ_VECTOR_BASE + 10)
#define BSP_IRQ_VECTOR_TIM1_OVR_UIF      (BSP_IRQ_VECTOR_BASE + 11)
#define BSP_IRQ_VECTOR_TIM1_CAPCOM_BIF   (BSP_IRQ_VECTOR_BASE + 12)
#define BSP_IRQ_VECTOR_TIM1_CAPCOM_TIF   (BSP_IRQ_VECTOR_BASE + 12)
#define BSP_IRQ_VECTOR_TIM1_CAPCOM_CC1IF (BSP_IRQ_VECTOR_BASE + 12)
#define BSP_IRQ_VECTOR_TIM1_CAPCOM_CC2IF (BSP_IRQ_VECTOR_BASE + 12)
#define BSP_IRQ_VECTOR_TIM1_CAPCOM_CC3IF (BSP_IRQ_VECTOR_BASE + 12)
#define BSP_IRQ_VECTOR_TIM1_CAPCOM_CC4IF (BSP_IRQ_VECTOR_BASE + 12)
#define BSP_IRQ_VECTOR_TIM1_CAPCOM_COMIF (BSP_IRQ_VECTOR_BASE + 12)
#define BSP_IRQ_VECTOR_TIM2_OVR_UIF      (BSP_IRQ_VECTOR_BASE + 13)
#define BSP_IRQ_VECTOR_TIM2_CAPCOM_TIF   (BSP_IRQ_VECTOR_BASE + 14)
#define BSP_IRQ_VECTOR_TIM2_CAPCOM_CC1IF (BSP_IRQ_VECTOR_BASE + 14)
#define BSP_IRQ_VECTOR_TIM2_CAPCOM_CC2IF (BSP_IRQ_VECTOR_BASE + 14)
#define BSP_IRQ_VECTOR_TIM2_CAPCOM_CC3IF (BSP_IRQ_VECTOR_BASE + 14)
#define BSP_IRQ_VECTOR_TIM3_OVR_UIF      (BSP_IRQ_VECTOR_BASE + 15)
#define BSP_IRQ_VECTOR_TIM3_CAPCOM_TIF   (BSP_IRQ_VECTOR_BASE + 16)
#define BSP_IRQ_VECTOR_TIM3_CAPCOM_CC1IF (BSP_IRQ_VECTOR_BASE + 16)
#define BSP_IRQ_VECTOR_TIM3_CAPCOM_CC2IF (BSP_IRQ_VECTOR_BASE + 16)
#define BSP_IRQ_VECTOR_TIM3_CAPCOM_CC3IF (BSP_IRQ_VECTOR_BASE + 16)
#define BSP_IRQ_VECTOR_UART1_T_TXE       (BSP_IRQ_VECTOR_BASE + 17)
#define BSP_IRQ_VECTOR_UART1_T_TC        (BSP_IRQ_VECTOR_BASE + 17)
#define BSP_IRQ_VECTOR_UART1_R_RXNE      (BSP_IRQ_VECTOR_BASE + 18)
#define BSP_IRQ_VECTOR_UART1_R_OR        (BSP_IRQ_VECTOR_BASE + 18)
#define BSP_IRQ_VECTOR_UART1_R_IDLE      (BSP_IRQ_VECTOR_BASE + 18)
#define BSP_IRQ_VECTOR_UART1_R_PE        (BSP_IRQ_VECTOR_BASE + 18)
#define BSP_IRQ_VECTOR_UART1_R_LBDF      (BSP_IRQ_VECTOR_BASE + 18)
#define BSP_IRQ_VECTOR_I2C_SB            (BSP_IRQ_VECTOR_BASE + 19)
#define BSP_IRQ_VECTOR_I2C_ADDR          (BSP_IRQ_VECTOR_BASE + 19)
#define BSP_IRQ_VECTOR_I2C_ADD10         (BSP_IRQ_VECTOR_BASE + 19)
#define BSP_IRQ_VECTOR_I2C_STOPF         (BSP_IRQ_VECTOR_BASE + 19)
#define BSP_IRQ_VECTOR_I2C_BTF           (BSP_IRQ_VECTOR_BASE + 19)
#define BSP_IRQ_VECTOR_I2C_WUFH          (BSP_IRQ_VECTOR_BASE + 19)
#define BSP_IRQ_VECTOR_I2C_RXNE          (BSP_IRQ_VECTOR_BASE + 19)
#define BSP_IRQ_VECTOR_I2C_TXE           (BSP_IRQ_VECTOR_BASE + 19)
#define BSP_IRQ_VECTOR_I2C_BERR          (BSP_IRQ_VECTOR_BASE + 19)
#define BSP_IRQ_VECTOR_I2C_ARLO          (BSP_IRQ_VECTOR_BASE + 19)
#define BSP_IRQ_VECTOR_I2C_AF            (BSP_IRQ_VECTOR_BASE + 19)
#define BSP_IRQ_VECTOR_I2C_OVR           (BSP_IRQ_VECTOR_BASE + 19)
#define BSP_IRQ_VECTOR_UART3_T_TXE       (BSP_IRQ_VECTOR_BASE + 20)
#define BSP_IRQ_VECTOR_UART3_T_TC        (BSP_IRQ_VECTOR_BASE + 20)
#define BSP_IRQ_VECTOR_UART3_R_RXNE      (BSP_IRQ_VECTOR_BASE + 21)
#define BSP_IRQ_VECTOR_UART3_R_OR        (BSP_IRQ_VECTOR_BASE + 21)
#define BSP_IRQ_VECTOR_UART3_R_IDLE      (BSP_IRQ_VECTOR_BASE + 21)
#define BSP_IRQ_VECTOR_UART3_R_PE        (BSP_IRQ_VECTOR_BASE + 21)
#define BSP_IRQ_VECTOR_UART3_R_LBDF      (BSP_IRQ_VECTOR_BASE + 21)
#define BSP_IRQ_VECTOR_UART3_R_LHDF      (BSP_IRQ_VECTOR_BASE + 21)
#define BSP_IRQ_VECTOR_ADC2_AWDG         (BSP_IRQ_VECTOR_BASE + 22)
#define BSP_IRQ_VECTOR_ADC2_EOC          (BSP_IRQ_VECTOR_BASE + 22)
#define BSP_IRQ_VECTOR_TIM4_OVR_UIF      (BSP_IRQ_VECTOR_BASE + 23)
#define BSP_IRQ_VECTOR_FLASH_EOP         (BSP_IRQ_VECTOR_BASE + 24)
#define BSP_IRQ_VECTOR_FLASH_WR_PG_DIS   (BSP_IRQ_VECTOR_BASE + 24)
#else
#define BSP_IRQ_VECTOR_TLI          (BSP_IRQ_VECTOR_BASE + 0)
#define BSP_IRQ_VECTOR_AWU          (BSP_IRQ_VECTOR_BASE + 1)
#define BSP_IRQ_VECTOR_CLK          (BSP_IRQ_VECTOR_BASE + 2)
#define BSP_IRQ_VECTOR_EXTI0        (BSP_IRQ_VECTOR_BASE + 3)
#define BSP_IRQ_VECTOR_EXTI1        (BSP_IRQ_VECTOR_BASE + 4)
#define BSP_IRQ_VECTOR_EXTI2        (BSP_IRQ_VECTOR_BASE + 5)
#define BSP_IRQ_VECTOR_EXTI3        (BSP_IRQ_VECTOR_BASE + 6)
#define BSP_IRQ_VECTOR_EXTI4        (BSP_IRQ_VECTOR_BASE + 7)
#define BSP_IRQ_VECTOR_beCAN_RX     (BSP_IRQ_VECTOR_BASE + 8)
#define BSP_IRQ_VECTOR_beCAN_TX     (BSP_IRQ_VECTOR_BASE + 9)
#define BSP_IRQ_VECTOR_SPI_EOT      (BSP_IRQ_VECTOR_BASE + 10)
#define BSP_IRQ_VECTOR_TIM1_OVR_UIF (BSP_IRQ_VECTOR_BASE + 11)
#define BSP_IRQ_VECTOR_TIM1_CAPCOM  (BSP_IRQ_VECTOR_BASE + 12)
#define BSP_IRQ_VECTOR_TIM2_OVR_UIF (BSP_IRQ_VECTOR_BASE + 13)
#define BSP_IRQ_VECTOR_TIM2_CAPCOM  (BSP_IRQ_VECTOR_BASE + 14)
#define BSP_IRQ_VECTOR_TIM3_OVR_UIF (BSP_IRQ_VECTOR_BASE + 15)
#define BSP_IRQ_VECTOR_TIM3_CAPCOM  (BSP_IRQ_VECTOR_BASE + 16)
#define BSP_IRQ_VECTOR_UART1_TX     (BSP_IRQ_VECTOR_BASE + 17)
#define BSP_IRQ_VECTOR_UART1_RX     (BSP_IRQ_VECTOR_BASE + 18)
#define BSP_IRQ_VECTOR_I2C          (BSP_IRQ_VECTOR_BASE + 19)
#define BSP_IRQ_VECTOR_UART3_TX     (BSP_IRQ_VECTOR_BASE + 20)
#define BSP_IRQ_VECTOR_UART3_RX     (BSP_IRQ_VECTOR_BASE + 21)
#define BSP_IRQ_VECTOR_ADC2_EOC     (BSP_IRQ_VECTOR_BASE + 22)
#define BSP_IRQ_VECTOR_TIM4_OVR_UIF (BSP_IRQ_VECTOR_BASE + 23)
#define BSP_IRQ_VECTOR_FLASH        (BSP_IRQ_VECTOR_BASE + 24)
#endif
//------------------------------------------------------------------------------
#ifndef _BSP_ASSEMBLER
//------------------------------------------------------------------------------
#define BSP_IRQ_VECTOR_MAX          (BSP_IRQ_VECTOR_BASE + 31)
//------------------------------------------------------------------------------
typedef enum
{
	bspitFast, /*!< no store virtual register */
	bspitStd
} tBSP_InterruptType;

typedef enum
{
	BSP_IP_DISABLE = 3, /*!< as main */
	BSP_IP_LOW2    = 0, /*!< low     */
	BSP_IP_LOWER1  = 1, /*!< lower   */
	BSP_IP_LOWEST0 = 2  /*!< lowest  */
} tBSP_InterruptPriority;

typedef void (* tBSP_InterruptHandler)(void);
//------------------------------------------------------------------------------
typedef struct
{
	tBSP_InterruptType Type;
	tBSP_InterruptHandler Handler;
} tBSP_Interrupt;

extern volatile tBSP_Interrupt BSP_InterruptsMap[];
//------------------------------------------------------------------------------
void BSP_InterruptException(void);
void BSP_InterruptPriority(uint8_t vector, uint8_t priority);
//------------------------------------------------------------------------------
#define BSP_InterruptSet(pHandler, irq_vector, irq_type)\
    do{\
      BSP_InterruptsMap[irq_vector - BSP_IRQ_VECTOR_BASE].Type    = irq_type;\
      BSP_InterruptsMap[irq_vector - BSP_IRQ_VECTOR_BASE].Handler = pHandler;\
    }while(0)
//------------------------------------------------------------------------------
#define BSP_InterruptReset(irq_vector)\
    do{\
      BSP_InterruptsMap[irq_vector - BSP_IRQ_VECTOR_BASE].Type    = bspitFast;\
      BSP_InterruptsMap[irq_vector - BSP_IRQ_VECTOR_BASE].Handler = BSP_InterruptException;\
    }while(0)
//------------------------------------------------------------------------------
#define BSP_SetInterruptPriority(irq_vector, irq_prio) \
        BSP_InterruptPriority((irq_vector - BSP_IRQ_VECTOR_BASE), irq_prio)
//------------------------------------------------------------------------------
#endif // _BSP_ASSEMBLER
//------------------------------------------------------------------------------
#endif // BSP_INTERRUPTS_H