/**
  ******************************************************************************
  * @file    can.c
  * @author  Khusainov Timur
  * @version 0.0.0.1
  * @date    10.02.2012
  * @brief   
  ******************************************************************************
  * @attention
  * <h2><center>&copy; COPYRIGHT timyik@gmail.com </center></h2>
  ******************************************************************************
  */

#include <stdint.h>
#include <stddef.h>

#include <stm8s_can.h>

#include "board.h"
#include "irq/interrupt.h"
#include "can.h"

//------------------------------------------------------------------------------
CAN_Page_TypeDef _CurrentPage;

#define _GetCurrentPage()     ((CAN_Page_TypeDef)(CAN->PSR))
#define _SetCurrentPage(_p)   do {CAN->PSR = (uint8_t)(_p);        } while(0)
#define _StoreCurrentPage()   do {_CurrentPage = _GetCurrentPage();} while(0)
#define _RestoreCurrentPage() do {_SetCurrentPage(_CurrentPage);   } while(0)
//------------------------------------------------------------------------------
#define _IRQEnableWKU()  do {CAN->IER |= CAN_IER_WKUIE;} while(0) /*!< Wakeup Interrupt Enable */
#define _IRQEnableTME()  do {CAN->IER |= CAN_IER_TMEIE;} while(0) /*!< Transmit Mailbox Empty Interrupt Enable */
#define _IRQEnableFOV()  do {CAN->IER |= CAN_IER_FOVIE;} while(0) /*!< FIFO Overrun Interrupt Enable */
#define _IRQEnableFMP()  do {CAN->IER |= CAN_IER_FMPIE;} while(0) /*!< FIFO Message Pending Interrupt Enable */
#define _IRQEnableFF()   do {CAN->IER |= CAN_IER_FFIE; } while(0) /*!< FIFO Full Interrupt Enable */
#define _IRQEnableFI()   do {CAN->IER |= (CAN_IER_FFIE|CAN_IER_FMPIE|CAN_IER_FMPIE);} while(0)
//------------------------------------------------------------------------------
#define _IRQDislbleWKU() do {CAN->IER &=~CAN_IER_WKUIE;} while(0) /*!< Disable -/- */
#define _IRQDislbleTME() do {CAN->IER &=~CAN_IER_TMEIE;} while(0)
#define _IRQDislbleFOV() do {CAN->IER &=~CAN_IER_FOVIE;} while(0)
#define _IRQDislbleFMP() do {CAN->IER &=~CAN_IER_FMPIE;} while(0)
#define _IRQDislbleFF()  do {CAN->IER &=~CAN_IER_FFIE; } while(0)
#define _IRQDislbleFI()  do {CAN->IER &=~(CAN_IER_FFIE|CAN_IER_FMPIE|CAN_IER_FMPIE);} while(0)
//------------------------------------------------------------------------------
#define _EnableABOM()    do {CAN->MCR |= CAN_MCR_ABOM; } while(0) /*!< Enable Automatic Bus-Off Management */
#define _DisableABOM()   do {CAN->MCR &=~CAN_MCR_ABOM; } while(0) /*!< Disable -/- */
//------------------------------------------------------------------------------
#define _EnableTXM2()    do {CAN->DGR |= CAN_DGR_TXM2E;} while(0) /*!< Enables the third TX Mailbox (Mailbox 2) */ 
#define _DisableTXM2()   do {CAN->DGR &=~CAN_DGR_TXM2E;} while(0) /*!< Disable -/- */
//------------------------------------------------------------------------------
#define _SetLoopBackMode() do {CAN->DGR |= CAN_DGR_LBKM;} while(0)
#define _SetSilentMode()   do {CAN->DGR |= CAN_DGR_SLIM;} while(0)
#define _SetCombSLBMode()  do {CAN->DGR |=  (CAN_DGR_SLIM|CAN_DGR_LBKM);} while(0)
#define _SetNormalMode()   do {CAN->DGR &= ~(CAN_DGR_SLIM|CAN_DGR_LBKM);} while(0)
//------------------------------------------------------------------------------
#define _GetFifoMsgPending()  ((uint16_t)(CAN->RFR & CAN_RFR_FMP01))
#define _IsFifoFull()         (CAN->RFR & CAN_RFR_FULL)
#define _IsFifoOverrun()      (CAN->RFR & CAN_RFR_FOVR)
#define _ReleaseFifoMailbox() do {CAN->RFR |= CAN_RFR_RFOM;} while(0)
//------------------------------------------------------------------------------
#define _EXID_MASK ((uint8_t)(1<<6))
#define _RTR_MASK  ((uint8_t)(1<<5))
#define _DLC_MASK  ((uint8_t)(0x0F))
//------------------------------------------------------------------------------
#define _IsExtID() ((uint8_t) CAN->Page.RxFIFO.MIDR1 & _EXID_MASK)
#define _IsRTR()   ((uint8_t) CAN->Page.RxFIFO.MIDR1 & _RTR_MASK)
#define _GetDLC()  ((uint8_t) CAN->Page.RxFIFO.MDLCR & _DLC_MASK)
#define _GetFMI()  ((uint8_t) CAN->Page.RxFIFO.MFMI)
//------------------------------------------------------------------------------
#define _RequestTx() do {CAN->Page.TxMailbox.MCSR |= CAN_MCSR_TXRQ;} while(0)
#define _IsTxOK0()   (CAN->TSR & CAN_TSR_TXOK0)
#define _IsTxOK1()   (CAN->TSR & CAN_TSR_TXOK1)
#define _IsTxOK2()   (CAN->TSR & CAN_TSR_TXOK2)
#define _RQ0Conf()   do {CAN->TSR |= CAN_TSR_RQCP0;} while(0)
#define _RQ1Conf()   do {CAN->TSR |= CAN_TSR_RQCP1;} while(0)
#define _RQ2Conf()   do {CAN->TSR |= CAN_TSR_RQCP2;} while(0)
//------------------------------------------------------------------------------
typedef enum
{
	rbsEmpty = 0,
	rbsPop,
	rbsPush,
	rbsFull
} _tRB_State;

typedef struct
{
	size_t Head;  /*!< */
	size_t Tail;  /*!< */
	_tRB_State State;
}_tRingBuffer;

volatile static _tRingBuffer CAN_RxQueue = {0, 0, rbsEmpty};
volatile static _tRingBuffer CAN_TxQueue = {0, 0, rbsEmpty};

enum
{
	CAN_RX_BUFFER_MASK = (CAN_RX_BUFFER_SIZE - 1),
	CAN_TX_BUFFER_MASK = (CAN_TX_BUFFER_SIZE - 1)
};

static tCAN_RxMsg CAN_RxBuffer[CAN_RX_BUFFER_SIZE] = {0};
static tCAN_TxMsg CAN_TxBuffer[CAN_TX_BUFFER_SIZE] = {0};
//------------------------------------------------------------------------------
enum
{
	CAN_FILTER_SCALE = (CAN_FilterScale_16Bit),
	CAN_FILTER_MODE  = (CAN_FilterMode_IdMask),
	CAN_FILTER_BANK  = (6),
	CAN_FILTER_MAX   = (CAN_FILTER_BANK * 4),
	CAN_FILTER_RESET = (0xFFE7)
};

volatile static size_t CAN_FilterCount = 0;
volatile static struct
{
	uint16_t     Config;
	tCAN_OnMsgRx Event;
} CAN_FilterMap[CAN_FILTER_MAX] = 
{
	{CAN_FILTER_RESET, NULL}, // Filter 0
	{CAN_FILTER_RESET, NULL}, // Filter 1
	{CAN_FILTER_RESET, NULL}, // Filter 2
	{CAN_FILTER_RESET, NULL}, // Filter 3
	{CAN_FILTER_RESET, NULL}, // Filter 4
	{CAN_FILTER_RESET, NULL}, // Filter 5
	{CAN_FILTER_RESET, NULL}, // Filter 6
	{CAN_FILTER_RESET, NULL}, // Filter 7
	{CAN_FILTER_RESET, NULL}, // Filter 8
	{CAN_FILTER_RESET, NULL}, // Filter 9
	{CAN_FILTER_RESET, NULL}, // Filter 10
	{CAN_FILTER_RESET, NULL}, // Filter 11
	{CAN_FILTER_RESET, NULL}, // Filter 12
	{CAN_FILTER_RESET, NULL}, // Filter 13
	{CAN_FILTER_RESET, NULL}, // Filter 14
	{CAN_FILTER_RESET, NULL}, // Filter 15
	{CAN_FILTER_RESET, NULL}, // Filter 16
	{CAN_FILTER_RESET, NULL}, // Filter 17
	{CAN_FILTER_RESET, NULL}, // Filter 18
	{CAN_FILTER_RESET, NULL}, // Filter 19
	{CAN_FILTER_RESET, NULL}, // Filter 20
	{CAN_FILTER_RESET, NULL}, // Filter 21
	{CAN_FILTER_RESET, NULL}, // Filter 22
	{CAN_FILTER_RESET, NULL}, // Filter 23
};
//------------------------------------------------------------------------------
volatile static enum
{
	CAN_RX_STATE_NOINIT = 0,
	CAN_RX_STATE_STOP,
	CAN_RX_STATE_MSG_WAIT,
	CAN_RX_STATE_MSG_RECV,
	CAN_RX_STATE_ERROR
} CAN_RxState;

volatile static enum
{
	CAN_TX_STATE_NOINIT = 0,
	CAN_TX_STATE_STOP,
	CAN_TX_STATE_EMPTY,
	CAN_TX_STATE_MSG_WAIT,
	CAN_TX_STATE_MSG_END,
	CAN_TX_STATE_ERROR
} CAN_TxState;
//------------------------------------------------------------------------------
const static struct
{
	uint8_t BRP;
	uint8_t BS1;
	uint8_t BS2;
	uint8_t SJW;
} CANSpeedSetting[CAN_SPEED_COUNT] = 
{
	#define _CSS(_brp, _bs1, _bs2, _sjw) {(_brp-1),(_bs1-1),(_bs2-1),(_sjw)}
    // BRP BS1 BS2 SJW
//_CSS(30, 16,  3,  2), // 20
	_CSS(30,  6,  1,  0), // 50
	_CSS(12,  6,  1,  0), // 152
	_CSS( 6,  6,  1,  0), // 250
	_CSS( 3,  6,  1,  0), // 500
	_CSS( 1, 12,  2,  1), // 800
	_CSS( 1,  9,  2,  1), // 1000
	#undef _CSS
};

void _SetSpeed(tCAN_Speed speed)
{
	_StoreCurrentPage();
	_SetCurrentPage(CAN_Page_Config);
	
	CAN->Page.Config.BTR1  = CANSpeedSetting[speed].BRP;
	CAN->Page.Config.BTR1 |= CANSpeedSetting[speed].SJW<<6;
	
	CAN->Page.Config.BTR2  = CANSpeedSetting[speed].BS1;
	CAN->Page.Config.BTR2 |= CANSpeedSetting[speed].BS2<<4;
	
	_RestoreCurrentPage();
}
//------------------------------------------------------------------------------
uint8_t _SetOpMode(CAN_OperatingMode_TypeDef op_mode)
{
	uint16_t m_timeout = 0xFFFF;
	uint8_t  m_checker;
	
	switch (op_mode)
	{
		case CAN_OperatingMode_Initialization:
			/* Request initialisation */
			CAN->MCR  = ((CAN->MCR & (~CAN_MCR_SLEEP)) | CAN_MCR_INRQ);
			m_checker = CAN_MSR_INAK;
			break;
		
		case CAN_OperatingMode_Sleep:
			/* Request Sleep mode */
			CAN->MCR  = ((CAN->MCR & (~CAN_MCR_INRQ)) | CAN_MCR_SLEEP);
			m_checker = CAN_MSR_SLAK;
			break;
		
		case CAN_OperatingMode_Normal:
			/* Request leave initialisation and sleep mode and enter Normal mode */
			CAN->MCR &= ~(CAN_MCR_SLEEP|CAN_MCR_INRQ);
			m_checker = 0;
			break;
	}
	
	/* wath while CPU complete operation */
	while (((CAN->MSR & 0x3) != m_checker) && m_timeout)
			m_timeout--;
	
	return ((CAN->MSR & 0x3) == m_checker);
}
//------------------------------------------------------------------------------
void _SetFilter16Idlist(uint8_t Bank, uint8_t Num, uint16_t Config)
{
	//----------------------------------------------------------------------------
	_StoreCurrentPage();
	
	_SetOpMode(CAN_OperatingMode_Initialization);
	_SetCurrentPage(CAN_Page_Config);
	//----------------------------------------------------------------------------
	// Deactivation filter 
	//----------------------------------------------------------------------------
	uint8_t m_fcr_deact = 7;
	
	if (Bank & 1)
		m_fcr_deact <<= 4;
	
	m_fcr_deact ^= 0xFF;
	
	switch (Bank >> 1)
	{
		case 0: CAN->Page.Config.FCR1 &= m_fcr_deact; break;
		case 1: CAN->Page.Config.FCR2 &= m_fcr_deact; break; // TODO check it, last: CAN->Page.Config.FCR1
		case 2: CAN->Page.Config.FCR3 &= m_fcr_deact; break;
	}
	//----------------------------------------------------------------------------
	// Enable Id/List mode
	//----------------------------------------------------------------------------
	uint8_t m_fmr  = 3 << (Bank & 3); 
	
	if (Bank >> 2)
		CAN->Page.Config.FMR2 |= m_fmr;
	else
		CAN->Page.Config.FMR1 |= m_fmr;
	//----------------------------------------------------------------------------
	// Set filter Id value
	//----------------------------------------------------------------------------
	_SetCurrentPage(CAN_Page_Filter01 + (Bank >> 1));
	
	if (Bank & 1)
	{
		switch (Num)
		{
			case 0:
				CAN->Page.Filter.FR09 = (uint8_t)(Config >> 8);
				CAN->Page.Filter.FR10 = (uint8_t)(Config);
				break;
			
			case 1:
				CAN->Page.Filter.FR11 = (uint8_t)(Config >> 8);
				CAN->Page.Filter.FR12 = (uint8_t)(Config);
				break;
			
			case 2:
				CAN->Page.Filter.FR13 = (uint8_t)(Config >> 8);
				CAN->Page.Filter.FR14 = (uint8_t)(Config);
				break;
				
			case 3:
				CAN->Page.Filter.FR15 = (uint8_t)(Config >> 8);
				CAN->Page.Filter.FR16 = (uint8_t)(Config);
				break;
		}
	}
	else
	{
		switch (Num)
		{
			case 0:
				CAN->Page.Filter.FR01 = (uint8_t)(Config >> 8);
				CAN->Page.Filter.FR02 = (uint8_t)(Config);
				break;
			
			case 1:
				CAN->Page.Filter.FR03 = (uint8_t)(Config >> 8);
				CAN->Page.Filter.FR04 = (uint8_t)(Config);
				break;
			
			case 2:
				CAN->Page.Filter.FR05 = (uint8_t)(Config >> 8);
				CAN->Page.Filter.FR06 = (uint8_t)(Config);
				break;
				
			case 3:
				CAN->Page.Filter.FR07 = (uint8_t)(Config >> 8);
				CAN->Page.Filter.FR08 = (uint8_t)(Config);
				break;
		}
	}
	//----------------------------------------------------------------------------
	// Filter enable & 16-bit mode 
	//----------------------------------------------------------------------------
	_SetCurrentPage(CAN_Page_Config);
	
	uint8_t m_fcr = (2 << 1)|(1 << 0);
	
	if (Bank & 1)
		m_fcr <<= 4;
	
	switch (Bank >> 1)
	{
		case 0: CAN->Page.Config.FCR1 |= m_fcr; break;
		case 1: CAN->Page.Config.FCR2 |= m_fcr; break; // TODO check it, last: CAN->Page.Config.FCR1
		case 2: CAN->Page.Config.FCR3 |= m_fcr; break;
	}
	//----------------------------------------------------------------------------
	// Set normal operation mode and resctore current page
	//----------------------------------------------------------------------------
	_SetOpMode(CAN_OperatingMode_Normal);
	_RestoreCurrentPage();
	//----------------------------------------------------------------------------
}
//------------------------------------------------------------------------------
void CAN_FilterAdd(uint32_t Id, uint8_t IDE, uint8_t RTR, tCAN_OnMsgRx pEvent)
{
	uint8_t m_bank = (CAN_FilterCount >> 2);
	uint8_t m_num  = (CAN_FilterCount & 3);
	
	union
	{
		struct
		{
			uint8_t EXID28_21;
			uint8_t EXID17_15 :3;
			uint8_t IDE       :1;
			uint8_t RTR       :1;
			uint8_t EXID20_18 :3;
		}Ex;
		
		struct
		{
			uint8_t STID10_3;
			uint8_t RESERVED  :3;
			uint8_t IDE       :1;
			uint8_t RTR       :1;
			uint8_t STID2_0   :3;
		}Std;
		
		uint16_t config;
	}m_filter;
	
	if (IDE)
	{
		m_filter.config  = ((uint16_t)(Id >> 13)) & 0xFFE0; /*!< Set EXID[28:21] and EXID[20:18] */ 
		m_filter.config |= ((uint16_t)(Id >> 15)) & 0x0007; /*!< Set EXID[17:15] */
		
		m_filter.config |= (1 << 3);
	}
	else
		m_filter.config = ((uint16_t)(Id << 5));
	
	if (RTR)
		m_filter.config |= (1 << 4);
	
	_SetFilter16Idlist(m_bank, m_num, m_filter.config);
	
	CAN_FilterMap[CAN_FilterCount].Config = m_filter.config;
	CAN_FilterMap[CAN_FilterCount].Event  = pEvent;
	
	CAN_FilterCount++;
}
//------------------------------------------------------------------------------
void _CAN_ReceiveMsg(tCAN_RxMsg *pMsg)
{
	_StoreCurrentPage();
	_SetCurrentPage(CAN_Page_RxFifo);
	
	pMsg->IDE = _IsExtID();
	
	if (_IsExtID())
	{
		uint8_t *_id = ((uint8_t *)&pMsg->ID);
		
		_id[0] = CAN->Page.RxFIFO.MIDR1; /*!< EXID[28:24] */ /* index invert because big endian */
		_id[1] = CAN->Page.RxFIFO.MIDR2; /*!< EXID[23:16] */
		_id[2] = CAN->Page.RxFIFO.MIDR3; /*!< EXID[15:8]  */
		_id[3] = CAN->Page.RxFIFO.MIDR4; /*!< EXID[7:0]   */
		
		pMsg->ID &= CAN_EXTID_SIZE;
	}
	else
	{
		uint16_t *_id = ((uint16_t *)&pMsg->ID) + 1; /* +1 because big endian */
		
		*_id  = (CAN->Page.RxFIFO.MIDR2 >> 2); /*!< STID[5:0]  */
		*_id |= ((uint16_t)(CAN->Page.RxFIFO.MIDR1 & 0x1F) << 6); /*!< STID[10:6]  */
		
		*_id &= CAN_STDID_SIZE;
	}
	
	pMsg->RTR = _IsRTR();
	
	if (!_IsRTR())
	{
		pMsg->Data[0] = CAN->Page.RxFIFO.MDAR1;
		pMsg->Data[1] = CAN->Page.RxFIFO.MDAR2;
		pMsg->Data[2] = CAN->Page.RxFIFO.MDAR3;
		pMsg->Data[3] = CAN->Page.RxFIFO.MDAR4;
		pMsg->Data[4] = CAN->Page.RxFIFO.MDAR5;
		pMsg->Data[5] = CAN->Page.RxFIFO.MDAR6;
		pMsg->Data[6] = CAN->Page.RxFIFO.MDAR7;
		pMsg->Data[7] = CAN->Page.RxFIFO.MDAR8;
	}
	
	pMsg->DLC = _GetDLC();
	pMsg->FMI = _GetFMI();
	
	_ReleaseFifoMailbox();
	_RestoreCurrentPage();
}

void _CAN_TransmittMsg(const tCAN_TxMsg *pMsg)
{
	_StoreCurrentPage();
	
	CAN_Page_TypeDef m_page;
	
	/* Get empty MailBox */
	if (CAN->TPR & CAN_TPR_TME0)
	{ m_page = CAN_Page_TxMailBox0;
	}
	else if (CAN->TPR & CAN_TPR_TME1)
	{ m_page = CAN_Page_TxMailBox1;
	}
	else if (CAN->TPR & CAN_TPR_TME2)
	{ 
		m_page = CAN_Page_TxMailBox2;
	}
	
	//TODO: if no empty page?
	
	_SetCurrentPage(m_page);
	
	/* Set up the Id */
	if (pMsg->IDE)
	{
		const uint8_t *_id = ((uint8_t *)&pMsg->ID);
		
		CAN->Page.TxMailbox.MIDR4 = _id[3]; /*!< EXID[7:0]   */ /* index invert because big endian */
		CAN->Page.TxMailbox.MIDR3 = _id[2]; /*!< EXID[15:8]  */
		CAN->Page.TxMailbox.MIDR2 = _id[1]; /*!< EXID[23:16] */
		CAN->Page.TxMailbox.MIDR1 = _id[0]; /*!< EXID[28:24] */
		CAN->Page.TxMailbox.MIDR1 |= _EXID_MASK;
	}
	else
	{
		const uint16_t *_id = ((uint16_t *)&pMsg->ID) + 1;  /* +1 because big endian */
		
		CAN->Page.TxMailbox.MIDR1 = (uint8_t)(*_id >> 6); /*!< STID[10:6] */
		CAN->Page.TxMailbox.MIDR2 = (uint8_t)(*_id << 2); /*!< STID[5:0]  */
	}
	
	if (pMsg->RTR)
	{
		/* Set up the RTR flag */
		CAN->Page.TxMailbox.MIDR1 |= _RTR_MASK;
	}
	else
	{
		/* Set up the data field */
		CAN->Page.TxMailbox.MDAR1 = pMsg->Data[0];
		CAN->Page.TxMailbox.MDAR2 = pMsg->Data[1];
		CAN->Page.TxMailbox.MDAR3 = pMsg->Data[2];
		CAN->Page.TxMailbox.MDAR4 = pMsg->Data[3];
		CAN->Page.TxMailbox.MDAR5 = pMsg->Data[4];
		CAN->Page.TxMailbox.MDAR6 = pMsg->Data[5];
		CAN->Page.TxMailbox.MDAR7 = pMsg->Data[6];
		CAN->Page.TxMailbox.MDAR8 = pMsg->Data[7];
	}

	/* Set up the DLC  */
	CAN->Page.TxMailbox.MDLCR = (pMsg->DLC & _DLC_MASK);
	
	_RequestTx();
	_RestoreCurrentPage();
}
//------------------------------------------------------------------------------
void _CAN_InterruptRx()
{
	while (_GetFifoMsgPending())
	{
		size_t m_head = (CAN_RxQueue.Head + 1) & CAN_RX_BUFFER_MASK;
		
		if (m_head != CAN_RxQueue.Tail)
		{
			_CAN_ReceiveMsg(&CAN_RxBuffer[m_head]);
			
			CAN_RxQueue.Head  = m_head;
			CAN_RxQueue.State = rbsPush;
		}
		else
		{
			CAN_RxQueue.State = rbsFull;
			break;
		}
	}
	
	CAN_RxState = CAN_RX_STATE_MSG_RECV;
}

void _CAN_InterruptTx()
{
	uint8_t m_state;
	
	if (CAN->TPR & CAN_TPR_TME0)
	{
		m_state = (CAN->TSR & (CAN_TSR_TXOK0|CAN_TSR_RQCP0));
		
		_RQ0Conf();
	}
	else if (CAN->TPR & CAN_TPR_TME1)
	{
		m_state = (CAN->TSR & (CAN_TSR_TXOK1|CAN_TSR_RQCP1)) >> 1;
		
		_RQ1Conf();
	}
	else if (CAN->TPR & CAN_TPR_TME2)
	{
		m_state = (CAN->TSR & (CAN_TSR_TXOK2|CAN_TSR_RQCP2)) >> 2;
		
		_RQ2Conf();
	}
	else
		m_state = 0;
	
	switch (m_state)
	{
		case 0x11:
			// TODO: good msg
			break;
		
		case 0x01:
			// TODO: check this state & make action
			break;
		
		case 0x0:
			// TODO: check this state & make action
			break;

		default:
			// TODO: wtf ?
			break;
	}

	const size_t _msg_index = (CAN_TxQueue.Tail + 1) & CAN_TX_BUFFER_MASK;
	const tCAN_TxMsg *_msg = &CAN_TxBuffer[_msg_index];

	if (_msg->Event)
		_msg->Event();

	if (_msg_index != CAN_TxQueue.Head)
	{
		_msg = &CAN_TxBuffer[(_msg_index + 1) & CAN_TX_BUFFER_MASK];
		
		_CAN_TransmittMsg(_msg);

		CAN_TxQueue.State = rbsPop;
	}
	else
	{
		CAN_TxQueue.State = rbsEmpty;
		CAN_TxState = CAN_TX_STATE_MSG_END;
	}
		
	CAN_TxQueue.Tail = _msg_index;
}
//------------------------------------------------------------------------------
void CAN_TransmitMsg(const tCAN_TxMsg* pMsg)
{
	size_t m_head = (CAN_TxQueue.Head + 1) & CAN_TX_BUFFER_MASK;
	
	if (m_head != CAN_TxQueue.Tail)
	{
		CAN_TxBuffer[m_head] = *pMsg;
		
		CAN_TxQueue.Head  = m_head;
		CAN_TxQueue.State = rbsPush;
	}
	else
		CAN_TxQueue.State = rbsFull;
}
//------------------------------------------------------------------------------
void BSP_InitCAN(tCAN_Speed Speed)
{
	//----------------------------------------------------------------------------
	BSP_PIN_INPP(GPIOG, (1 << 0)); /* TX pin as ipnut and pull-up */
	BSP_PIN_INPP(GPIOG, (1 << 1)); /* RX pin as ipnut and pull-up */
	//----------------------------------------------------------------------------

	_StoreCurrentPage();
	
	/* Request initialisation beCAN */
	_SetOpMode(CAN_OperatingMode_Initialization);
	
	/* Enable Automatic bus-off management */
	_EnableABOM();
	
	/* Enable Silent Mode at beCAN */
	_SetSilentMode();
	
	/* Enable TxMailBox2 */
	_EnableTXM2();
	
	_SetSpeed(Speed);
	
	/* Set Rx/TX interrupt handlers */
	BSP_InterruptSet(&_CAN_InterruptRx, BSP_IRQ_VECTOR_beCAN_RX, bspitStd);
	BSP_InterruptSet(&_CAN_InterruptTx, BSP_IRQ_VECTOR_beCAN_TX, bspitStd);
	
	/* Enable operating mode Normal (Rx/Tx work) */
	_SetOpMode(CAN_OperatingMode_Normal);
	
	/* Fill filter list default value */
	for (size_t i = 0; i < CAN_FILTER_MAX; ++i)
		_SetFilter16Idlist((i >> 2), (i & 3), CAN_FILTER_RESET);

	/* Reset filter counter */
	CAN_FilterCount = 0;
	
	CAN_RxState = CAN_RX_STATE_STOP;
	CAN_TxState = CAN_TX_STATE_STOP;
}

void BSP_SetPriorityCAN(tBSP_Priority priority)
{
	BSP_SetPriority(BSP_IRQ_VECTOR_beCAN_RX, priority);
	BSP_SetPriority(BSP_IRQ_VECTOR_beCAN_TX, priority);
}

void BSP_StartCAN()
{
	/* Enable normal mode, and no loopback */
	_SetNormalMode();
	
	CAN_RxState = CAN_RX_STATE_MSG_WAIT;
	CAN_TxState = CAN_TX_STATE_EMPTY;
	
	/* Enable FIFO Msg panding and TX Msg Empty interrupts */
	_IRQEnableFMP();
	_IRQEnableTME();
}

void BSP_StopCAN()
{
	/* Disable FIFO Msg panding and TX Msg Empty interrupts */
	_IRQEnableFMP();
	_IRQEnableTME();
	
	CAN_RxState = CAN_RX_STATE_STOP;
	CAN_TxState = CAN_TX_STATE_STOP;
	
	/* Enable silent mode */
	_SetSilentMode();
}
//------------------------------------------------------------------------------
void CAN_Process()
{
	//----------------------------------------------------------------------------
	switch (CAN_RxState)
	{
		case CAN_RX_STATE_NOINIT:
		case CAN_RX_STATE_STOP:
		case CAN_RX_STATE_MSG_WAIT:
			break;
		
		case CAN_RX_STATE_MSG_RECV:
		{
			do
			{
				const size_t _index    = (CAN_RxQueue.Tail + 1) & CAN_RX_BUFFER_MASK;
				const tCAN_RxMsg *_msg = &CAN_RxBuffer[_index];
				const tCAN_OnMsgRx _event = CAN_FilterMap[_msg->FMI].Event;
				
				if (_event)
					_event(_msg);
				
				if (_index != CAN_RxQueue.Head)
					CAN_RxQueue.State = rbsPop;
				else
					CAN_RxQueue.State = rbsEmpty;
					
				CAN_RxQueue.Tail = _index;
				
			} while (CAN_RxQueue.State != rbsEmpty);
			
			CAN_RxState = CAN_RX_STATE_MSG_WAIT;
		}
		break;
		
		case CAN_RX_STATE_ERROR:
			// TODO: error event and handler
			break;
	}
	//----------------------------------------------------------------------------
	switch (CAN_TxState)
	{
		case CAN_TX_STATE_NOINIT:
		case CAN_TX_STATE_STOP:
		case CAN_TX_STATE_MSG_WAIT:
			// TODO: check tx err counter
			break;
		
		case CAN_TX_STATE_MSG_END:
		case CAN_TX_STATE_EMPTY:
		{
			if (CAN_TxQueue.State != rbsEmpty)
			{
				const size_t _index = (CAN_TxQueue.Tail + 1) & CAN_TX_BUFFER_MASK;

				_CAN_TransmittMsg(&CAN_TxBuffer[_index]);

				CAN_TxQueue.State = rbsPop;
				CAN_TxState = CAN_TX_STATE_MSG_WAIT;
			}
		}
		break;
		
		case CAN_TX_STATE_ERROR:
			// TODO: error event and handler
			break;
	}
	//----------------------------------------------------------------------------
}
//------------------------------------------------------------------------------
