/**
  ******************************************************************************
  * @file    board.h
  * @author  Khusainov Timur
  * @version v0.0.0.3
  * @date    7.02.2012
  * @brief   Board Support Package
  ******************************************************************************
  * @attention
  * <h2><center>&copy; COPYRIGHT timyik@gmail.com </center></h2>
  ******************************************************************************
  */

#ifndef BOARD_H
#define BOARD_H

#include <stm8s.h>

#if defined(__CSMC__)
# define BSP_CRITICAL_STORE      uint8_t ccr
# define BSP_CRITICAL_START()    _asm ("push CC\npop a\nld (X),A\nsim", &ccr)
# define BSP_CRITICAL_END()      _asm ("ld A,(X)\npush A\npop CC", &ccr)

# define BSP_IRQ_HANDLER(vec)    @far @interrupt void handler_##vec(void)
#elif defined(__IAR_SYSTEMS_ICC__) || defined(__ICCSTM8__)
# define BSP_CRITICAL_STORE      __istate_t _istate
# define BSP_CRITICAL_START()    _istate = __get_interrupt_state(); __disable_interrupt()
# define BSP_CRITICAL_END()      __set_interrupt_state(_istate)

# define _TOSTR(x) #x
# define _TOVEC(vec) _TOSTR(vector = (vec))
# define BSP_IRQ_HANDLER(vec)    _Pragma(_TOVEC((vec))) __interrupt void handler_##vec(void)
#elif defined(__RCSTM8__) || defined (__RCST7__)
# define BSP_CRITICAL_STORE      unsigned char ccr
# define BSP_CRITICAL_START()    ccr = _getCC_(); _sim_()
# define BSP_CRITICAL_END()      _setCC_(ccr)

# define BSP_IRQ_HANDLER(vec) void handler_##vec(void) interrupt vec
#endif

/**
 *  CPU pin control
 */
#define BSP_PIN_INPZ(_port, _pin) do { _port->CR1 &=~(_pin); _port->DDR &=~(_pin); _port->CR2 &=~(_pin); } while (0)
#define BSP_PIN_INPP(_port, _pin) do { _port->CR1 |= (_pin); _port->DDR &=~(_pin); _port->CR2 &=~(_pin); } while (0)
#define BSP_PIN_OUTP(_port, _pin) do { _port->CR1 |= (_pin); _port->DDR |= (_pin); _port->CR2 |= (_pin); } while (0)
#define BSP_PIN_OUTD(_port, _pin) do { _port->CR1 |= (_pin); _port->DDR |= (_pin); _port->CR2 &=~(_pin); } while (0)

#define BSP_PIN_STATE(_port, _pin) (_port->IDR & _pin)

#define BSP_PIN_0(_port, _pin) do { _port->ODR &=~(_pin); } while (0)
#define BSP_PIN_1(_port, _pin) do { _port->ODR |= (_pin); } while (0)
#define BSP_PIN_T(_port, _pin) do { _port->ODR ^= (_pin); } while (0)

#ifdef DEBUG
# warning "Don't use DEBUG firmware!"
# define BSP_DBG_PORT   GPIOE
# define BSP_DBG_PIN    (1<<3)
# define BSP_DBG_PINIT  BSP_PIN_OUTP(BSP_DBG_PORT, BSP_DBG_PIN)
# define BSP_DBG_PDINIT BSP_PIN_INPZ(BSP_DBG_PORT, BSP_DBG_PIN)
# define BSP_DBG_P0     BSP_PIN_0(BSP_DBG_PORT, BSP_DBG_PIN)
# define BSP_DBG_P1     BSP_PIN_1(BSP_DBG_PORT, BSP_DBG_PIN)
# define BSP_DBG_PT     BSP_PIN_T(BSP_DBG_PORT, BSP_DBG_PIN)
#else
# define BSP_DBG_PINIT
# define BSP_DBG_P0
# define BSP_DBG_P1
# define BSP_DBG_PT
#endif

#ifndef F_CPU
  static uint32_t F_CPU = 0;
# define BSP_DETECTED_SPEED
# warning "Don't define F_CPU, use BSP_DETECTED_SPEED!"
#endif

/**
 *  RS-485 direction control
 */
#define BSP_DIR485_PORT    (GPIOD)
#define BSP_DIR485_PIN     (1<<7)
#define BSP_DIR485_INIT()  BSP_PIN_OUTP(BSP_DIR485_PORT, BSP_DIR485_PIN)
#define BSP_DIR485_STATE() BSP_PIN_STATE(BSP_DIR485_PORT, BSP_DIR485_PIN)

#define BSP_DIR485_Tx()    BSP_PIN_1(BSP_DIR485_PORT, BSP_DIR485_PIN)
#define BSP_DIR485_Rx()    BSP_PIN_0(BSP_DIR485_PORT, BSP_DIR485_PIN)

/**
 *  RS-485 terminator control
 */
#define BSP_TRIM485_PORT (GPIOC)
#define BSP_TRIM485_P_A  (1<<6)
#define BSP_TRIM485_P_B  (1<<7)

#define BSP_TRIM485_INIT()  BSP_PIN_OUTP(BSP_TRIM485_PORT, BSP_TRIM485_P_A|BSP_TRIM485_P_B)
#define BSP_TRIM485_DINIT() BSP_PIN_INPZ(BSP_TRIM485_PORT, BSP_TRIM485_P_A|BSP_TRIM485_P_B)

#define BSP_TRIM485_PA_EN() BSP_PIN_1(BSP_TRIM485_PORT, BSP_TRIM485_P_A)
#define BSP_TRIM485_PB_EN() BSP_PIN_0(BSP_TRIM485_PORT, BSP_TRIM485_P_B)

#define BSP_TRIM485_En() do { BSP_TRIM485_PA_EN(); BSP_TRIM485_PB_EN(); BSP_TRIM485_INIT(); } while(0)
#define BSP_TRIM485_Ds() do { BSP_TRIM485_DINIT(); } while(0)

/**
 *  Onboard 1-wire(OWI) implementation
 */
#define BSP_OWI_PORT (GPIOA)
#define BSP_OWI_P_RX (1<<4)
#define BSP_OWI_P_TX (1<<5)

void BSP_InitCLK();
void BSP_InitWDT();
void BSP_ResetWDT();

typedef enum
{
	BSP_PRIORITY_DEFAULT = (0),
	BSP_PRIORITY_LEVEL_0,
	BSP_PRIORITY_LEVEL_1,
	BSP_PRIORITY_LEVEL_2,
} tBSP_Priority;

void BSP_SetPriority(uint8_t vector, tBSP_Priority priority);

#endif // BOARD_H
