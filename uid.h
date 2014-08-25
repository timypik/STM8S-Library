/**
  ******************************************************************************
  * @file    uid.h
  * @author  Khusainov Timur
  * @version 0.0.0.0
  * @date    31.10.2012
  * @brief   
  ******************************************************************************
  * @attention
  * <h2><center>&copy; COPYRIGHT timyik@gmail.com </center></h2>
  ******************************************************************************
  */

#ifndef UID_H
#define UID_H

#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------
#define BSP_UID_LENGTH_BIT  (96)
#define BSP_UID_LENGTH_BYTE (BSP_UID_LENGTH_BIT/8)
//------------------------------------------------------------------------------
enum
{
	BSP_UID_START_ADDRESS     = (0x48CD),
	BSP_UID_WAFER_X_ADDRESS   = (BSP_UID_START_ADDRESS),
	BSP_UID_WAFER_X_LENGTH    = (sizeof(uint16_t)),
	BSP_UID_WAFER_Y_ADDRESS   = (BSP_UID_WAFER_X_ADDRESS + BSP_UID_WAFER_X_LENGTH),
	BSP_UID_WAFER_Y_LENGTH    = (sizeof(uint16_t)),
	BSP_UID_WAFER_NUM_ADDRESS = (BSP_UID_WAFER_Y_ADDRESS + BSP_UID_WAFER_Y_LENGTH),
	BSP_UID_WAFER_NUM_LENGTH  = (sizeof(uint16_t)),
	BSP_UID_LOT_NUM_ADDRESS   = (BSP_UID_WAFER_NUM_ADDRESS + BSP_UID_WAFER_NUM_LENGTH),
	BSP_UID_LOT_NUM_LENGTH    = (sizeof(uint8_t) * 7),
	BSP_UID_LAST_ADDRESS      = (BSP_UID_LOT_NUM_ADDRESS + BSP_UID_LOT_NUM_LENGTH),
	BSP_UID_TOTAL_LENGTH      = ((BSP_UID_LAST_ADDRESS - BSP_UID_START_ADDRESS) - 1)
};
//------------------------------------------------------------------------------
typedef struct
{
	uint16_t WAFER_X;
	uint16_t WAFER_Y;
	uint8_t WAFER_NUM;
	uint8_t LOT_NUM[7];
} tBSP_UID;
//------------------------------------------------------------------------------
#define BSP_GetWaferX()   (*(uint16_t *)BSP_UID_WAFER_X_ADDRESS)
#define BSP_GetWaferY()   (*(uint16_t *)BSP_UID_WAFER_Y_ADDRESS)
#define BSP_GetWaferNum() (*(uint8_t *)BSP_UID_WAFER_NUM_ADDRESS)
#define BSP_GetLotNum()   ((uint8_t *)BSP_UID_LOT_NUM_ADDRESS)
//------------------------------------------------------------------------------
inline static tBSP_UID BSP_GetUID() { return (*(tBSP_UID *)(BSP_UID_START_ADDRESS)); }
//------------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif

#endif // UID_H