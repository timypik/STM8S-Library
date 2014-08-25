/**
  ******************************************************************************
  * @file    sys_timer.h
  * @author  Khusainov Timur
  * @version 0.0.0.4
  * @date    18.01.2012
  * @brief   System interval timer (ms)
  ******************************************************************************
  * @attention
  * <h2><center>&copy; COPYRIGHT timyik@gmail.com </center></h2>
  ******************************************************************************
  */

#ifndef BSP_SYS_TIMER_H
#define BSP_SYS_TIMER_H

//------------------------------------------------------------------------------
typedef uint16_t tBSP_SysTimerType;
extern  volatile tBSP_SysTimerType BSP_SysTimer;

#define BSP_SysTimerGet()  (BSP_SysTimer)
#define BSP_SysTimerSet(t) (BSP_SysTimer = t)
//------------------------------------------------------------------------------

enum {BSP_SYS_TIMER_DISCRETE_MS = (1)};

#define BSP_SysTimerCalcMs(value)  (value/BSP_SYS_TIMER_DISCRETE_MS)
#define BSP_SysTimerCalcSec(value) ((value * 1000)/BSP_SYS_TIMER_DISCRETE_MS)

//------------------------------------------------------------------------------
typedef struct
{
	tBSP_SysTimerType TimeStart;
	tBSP_SysTimerType TimeNeed;
} tBSP_SysTimer;

#define BSP_SysTimerNew(m_st, m_time_need)\
				do\
				{\
					m_st.TimeStart = BSP_SysTimerGet();\
					m_st.TimeNeed  = m_time_need;\
				} while(0)

#define BSP_SysTimerRe(m_st)\
				do\
				{\
					m_st.TimeStart = BSP_SysTimerGet();\
				} while(0)

#define BSP_SysTimerCheck(m_st) ((BSP_SysTimer - m_st.TimeStart) >= m_st.TimeNeed)
#define BSP_SysTimerOver(m_st) ((tBSP_SysTimerType)(BSP_SysTimer - m_st.TimeStart))
//------------------------------------------------------------------------------
typedef enum
{
	BSP_SYS_TIMER_STATE_NOINIT = 0,
	BSP_SYS_TIMER_STATE_STOP,
	BSP_SYS_TIMER_STATE_RUN
} tBSP_SysTimerState;

extern volatile tBSP_SysTimerState BSP_SysTimerState;

#define BSP_SysTimerGetState() (BSP_SysTimerState)
//------------------------------------------------------------------------------
void BSP_InitSysTimer(void);
void BSP_StartSysTimer(void);
void BSP_StopSysTimer(void);
#define BSP_RestartSysTimer() do{BSP_SysTimerSet(0);}while(0)
//------------------------------------------------------------------------------
#endif // BSP_SYS_TIMER_H
