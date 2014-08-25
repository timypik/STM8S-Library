/**
  ******************************************************************************
  * @file    eeprom.h
  * @author  Khusainov Timur
  * @version 0.0.0.0
  * @date    24.10.2012
  * @brief   
  ******************************************************************************
  * @attention
  * <h2><center>&copy; COPYRIGHT timyik@gmail.com </center></h2>
  ******************************************************************************
  */

#ifndef EEPROM_H
#define EEPROM_H

#ifdef __cplusplus
extern "C"{
#endif
//------------------------------------------------------------------------------
enum
{
	BSP_EEPROM_BASE_ADDRESS = (0x4000),
	BSP_EEPROM_LAST_ADDRESS = (0x4800),
	BSP_EEPROM_TOTAL_SIZE   = (BSP_EEPROM_LAST_ADDRESS - BSP_EEPROM_BASE_ADDRESS),
	BSP_EEPROM_BLOCK_SIZE   = (128),
	BSP_EEPROM_PAGE_SIZE    = (512),
	BSP_EEPROM_HASH_SIZE    = (2),
	BSP_EEPROM_TOTAL_BLOCK  = (BSP_EEPROM_TOTAL_SIZE/BSP_EEPROM_BLOCK_SIZE),
	BSP_EEPROM_TOTAL_PAGE   = (BSP_EEPROM_TOTAL_SIZE/BSP_EEPROM_PAGE_SIZE),
};	
//------------------------------------------------------------------------------
typedef enum
{
	EERPOM_ERROR_NONE = 0,
	EERPOM_ERROR_ADDRESS,
	EERPOM_ERROR_WRITE,
	EERPOM_ERROR_TIMEOUT,
	EERPOM_ERROR_SIZE,
	EEPROM_ERROR_HASH
} tEERPOM_Error;

typedef struct
{
	size_t Address;
	size_t Length;
} tEEPROM_File;
//------------------------------------------------------------------------------
#define EEPROM_FILE_SIZE_FROM_SIZE(_size) ((_size) + BSP_EEPROM_HASH_SIZE)
#define EEPROM_FILE_SIZE_FORM_TYPE(_type) EEPROM_FILE_SIZE_FROM_SIZE(sizeof(_type))
//------------------------------------------------------------------------------
tEERPOM_Error EEPROM_CheckFile(tEEPROM_File *file);
tEERPOM_Error EEPROM_UpdateFile(tEEPROM_File *file);
tEERPOM_Error EEPROM_WriteFile(tEEPROM_File *file, void *buffer, size_t offset, size_t length);
tEERPOM_Error EEPROM_ReadFile(tEEPROM_File *file, void *buffer, size_t offset, size_t length);
//------------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif

#endif // EEPROM_H