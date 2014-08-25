;------------------------------------------------------------------------
	name interrupt_iar
;------------------------------------------------------------------------
	#define _BSP_ASSEMBLER
	#include "vregs.inc"
	#include "interrupt.h"
	#undef _BSP_ASSEMBLER
;------------------------------------------------------------------------
	extern BSP_InterruptsMap
;------------------------------------------------------------------------
	#if __CODE_MODEL__ == __MEDIUM_CODE_MODEL__
	section `.near.bss`:DATA
_handler:
	DS8  3
	#endif
;------------------------------------------------------------------------
DECLARE_ISR macro vector_num
  pubweak _interrupt_<vector_num>
_interrupt_<vector_num>:
	#if __CODE_MODEL__ == __SMALL_CODE_MODEL__
	; NOTE:
	;      pointer size - 2
	;      struct size  - 3
	; (<vector_num> - 2) - IAR calculate irq vector num
	; (iar_irq_vector * 3) - calc position at BSP_Interrupts.Type
	; ((iar_irq_vector * 3) + 1) - calc position at BSP_Interrupts.Handler
	ld  a, l:(BSP_InterruptsMap + ((<vector_num> - 2)*3))       ;[1] get BSP_Interrupts[<vector_num>].Type
	ldw x, l:(BSP_InterruptsMap + (((<vector_num> - 2)*3) + 1)) ;[2] get BSP_Interrupts[<vector_num>].Handler
	ldw y, #(<vector_num> - 2) ;[2] store vector number (need for debug exception)
	jp  _interrupt_handler     ;[2] jump to global handler
	#elif __CODE_MODEL__ == __MEDIUM_CODE_MODEL__
	; NOTE:
	;      pointer size - 3
	;      struct size  - 4
	ld  a, l:(BSP_InterruptsMap + ((<vector_num> - 2)*4)) ;[1] get BSP_Interrupts[<vector_num>].Type
	;[?] get BSP_Interrupts[<vector_num>].Handler and store to <_handler>
	mov  l:(_handler + 0), l:(BSP_InterruptsMap + (((<vector_num> - 2)*4) + 1))
	mov  l:(_handler + 1), l:(BSP_InterruptsMap + (((<vector_num> - 2)*4) + 2))
	mov  l:(_handler + 2), l:(BSP_InterruptsMap + (((<vector_num> - 2)*4) + 3))
	ldw  y, #(<vector_num> - 2) ;[2] store vector number (need for debug exception)
	jpf  _interrupt_handler     ;[2] jump to global handler
	#elif __CODE_MODEL__ == __LARGE_CODE_MODEL__
	error "Add support this CODE model!"
	#endif
  endm
;------------------------------------------------------------------------
_STORE_VREGS: macro
	push s:?b3  ;[1] stotre virtual register
	push s:?b2  ; --------------------------
	push s:?b1  ; --------------------------
	push s:?b0  ; --------------------------
	push s:?b7  ; --------------------------
	push s:?b6  ; --------------------------
	push s:?b5  ; --------------------------
	push s:?b4  ; --------------------------
	push s:?b11 ; --------------------------
	push s:?b10 ; --------------------------
	push s:?b9  ; --------------------------
	push s:?b8  ;[12] <- total mcu clk
	endm
;------------------------------------------------------------------------
_RESTORE_VREGS: macro
	pop s:?b8  ;[1] restotre virtual register
	pop s:?b9  ; ----------------------------
	pop s:?b10 ; ----------------------------
	pop s:?b11 ; ----------------------------
	pop s:?b4  ; ----------------------------
	pop s:?b5  ; ----------------------------
	pop s:?b6  ; ----------------------------
	pop s:?b7  ; ----------------------------
	pop s:?b0  ; ----------------------------
	pop s:?b1  ; ----------------------------
	pop s:?b2  ; ----------------------------
	pop s:?b3  ;[12] <- total mcu clk
	endm
;------------------------------------------------------------------------
	section __DEFAULT_CODE_SECTION__:CODE
	code
;------------------------------------------------------------------------
BSP_INTERRUPT_HANDLER:
	;----------------------------------------------------------------------
	;DECLARE_ISR 0 ; RESET
	;DECLARE_ISR 1 ; TRAP
	DECLARE_ISR 2  ; IRQ_0  - TLI
	DECLARE_ISR 3  ; IRQ_1  - AWU
	DECLARE_ISR 4  ; IRQ_2  - CLK
	DECLARE_ISR 5  ; IRQ_3  - EXTI0
	DECLARE_ISR 6  ; IRQ_4  - EXTI1
	DECLARE_ISR 7  ; IRQ_5  - EXTI2
	DECLARE_ISR 8  ; IRQ_6  - EXTI3
	DECLARE_ISR 9  ; IRQ_7  - EXTI4
	DECLARE_ISR 10 ; IRQ_8  - beCAN RX
	DECLARE_ISR 11 ; IRQ_9  - beCAN TX
	DECLARE_ISR 12 ; IRQ_10 - SPI
	DECLARE_ISR 13 ; IRQ_11 - TIM1 U/O
	DECLARE_ISR 14 ; IRQ_12 - TIM1 CAPCAM
	DECLARE_ISR 15 ; IRQ_13 - TIM2 U/O
	DECLARE_ISR 16 ; IRQ_14 - TIM2 CAPCAM
	DECLARE_ISR 17 ; IRQ_15 - TIM3 U/O
	DECLARE_ISR 18 ; IRQ_16 - TIM3 CAPCAM
	DECLARE_ISR 19 ; IRQ_17 - UART1 TX
	DECLARE_ISR 20 ; IRQ_18 - UART1 RX
	DECLARE_ISR 21 ; IRQ_19 - I2C
	DECLARE_ISR 22 ; IRQ_20 - UART3 TX
	DECLARE_ISR 23 ; IRQ_21 - UART3 RX
	DECLARE_ISR 24 ; IRQ_22 - ADC2
	DECLARE_ISR 25 ; IRQ_23 - TIM4
	DECLARE_ISR 26 ; IRQ_24 - Flash
	DECLARE_ISR 27 ; IRQ_25 - RESERVED
	DECLARE_ISR 28 ; IRQ_26 - RESERVED
	DECLARE_ISR 29 ; IRQ_27 - RESERVED
	DECLARE_ISR 30 ; IRQ_28 - RESERVED
	DECLARE_ISR 31 ; IRQ_29 - RESERVED
	;----------------------------------------------------------------------
_interrupt_handler:
	cp   a, #0x0                 ;[1] if (Type == bspFast)
	jreq _interrupt_handler_call ;[2] true -> call Handler
	_STORE_VREGS                 ;[12] false -> store virtual register
_interrupt_handler_call:
	push a                       ;[1] store interrupt type
	#if   __CODE_MODEL__ == __SMALL_CODE_MODEL__
	call (x)                     ;[4+4] call Handler
	#elif __CODE_MODEL__ == __MEDIUM_CODE_MODEL__
	callf [_handler.e]           ;[?+?] call Handler
	#elif __CODE_MODEL__ == __LARGE_CODE_MODEL__
	error "Add support this CODE model!"
	#endif
	pop  a                       ;[1] restore interrupt type
	cp   a, #0x0                 ;[1] if (Type == bspFast)
	jreq _interrupt_handler_out  ;[2] true -> out from handler
	_RESTORE_VREGS               ;[12] false -> restore virtual register
	;----------------------------------------------------------------------
_interrupt_handler_out:
	iret
;------------------------------------------------------------------------
	end
;------------------------------------------------------------------------