/**
  ******************************************************************************
  * @file    can.h
  * @author  Khusainov Timur
  * @version 0.0.0.1
  * @date    10.02.2012
  * @brief   
  ******************************************************************************
  * @attention
  * <h2><center>&copy; COPYRIGHT timyik@gmail.com </center></h2>
  ******************************************************************************
  */

#ifndef CAN_H
#define CAN_H

#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------
typedef enum
{
	CAN_SPEED_50 = 0,
	CAN_SPEED_125,
	CAN_SPEED_250,
	CAN_SPEED_500,
	CAN_SPEED_800,
	CAN_SPEED_1000,
	CAN_SPEED_COUNT
} tCAN_Speed;
//------------------------------------------------------------------------------
typedef struct
{
	uint32_t ID;
	uint8_t IDE;
	uint8_t RTR;
	uint8_t DLC;
	uint8_t FMI;
	uint8_t Data[8];
} tCAN_RxMsg;
//------------------------------------------------------------------------------
typedef void (* tCAN_OnMsgRx)(const tCAN_RxMsg *pMsg);
typedef void (* tCAN_OnMsgTx)(void);
//------------------------------------------------------------------------------
typedef struct
{
	uint32_t ID;
	uint8_t IDE;
	uint8_t RTR;
	uint8_t DLC;
	tCAN_OnMsgTx Event;
	uint8_t Data[8];
} tCAN_TxMsg;
//------------------------------------------------------------------------------
enum /*!< Should be power of 2 */
{
	CAN_TX_BUFFER_SIZE = (8),
	CAN_RX_BUFFER_SIZE = (8)
};
//------------------------------------------------------------------------------
void BSP_InitCAN(tCAN_Speed Speed);
void BSP_SetPriorityCAN(tBSP_Priority priority);

void CAN_FilterAdd(uint32_t Id, uint8_t IDE, uint8_t RTR, tCAN_OnMsgRx pEvent);
void CAN_TransmitMsg(const tCAN_TxMsg* pMsg);
void BSP_StartCAN();
void BSP_StopCAN();
void CAN_Process();
//------------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif

#endif // CAN_H
