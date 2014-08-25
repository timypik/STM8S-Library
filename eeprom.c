/**
  ******************************************************************************
  * @file    eeprom.c
  * @author  Khusainov Timur
  * @version 0.0.0.0
  * @date    24.10.2012
  * @brief   
  ******************************************************************************
  * @attention
  * <h2><center>&copy; COPYRIGHT timyik@gmail.com </center></h2>
  ******************************************************************************
  */

#include <stdint.h>
#include <stddef.h>

#include "board.h"
#include "eeprom.h"

#include <crc16.h>

//------------------------------------------------------------------------------
#define _EEPROM_UNLOCK() do { FLASH->DUKR = 0xAE; FLASH->DUKR = 0x56; } while(0)
#define _EEPROM_LOCK()   do { FLASH->IAPSR &= ~FLASH_IAPSR_DUL; } while(0)
//------------------------------------------------------------------------------
#define _EEPROM_TIMEOUT_MS (10)
//------------------------------------------------------------------------------
tEERPOM_Error _EEPROM_WAIT_FOR_END()
{
	uint8_t  m_operation_status;
	uint32_t m_timeout = (uint32_t)(F_CPU * (1e-3) * _EEPROM_TIMEOUT_MS);
	
	do
	{
		m_operation_status = (FLASH->IAPSR & (FLASH_IAPSR_WR_PG_DIS|FLASH_IAPSR_EOP));
	}
	while ((m_operation_status == 0) && (m_timeout-- != 0));

	if (m_timeout == 0)
	{
		return EERPOM_ERROR_TIMEOUT;
	}
	else if (m_operation_status & FLASH_IAPSR_WR_PG_DIS)
	{
		return EERPOM_ERROR_WRITE;
	}
	else if (m_operation_status & FLASH_IAPSR_EOP)
	{
		return EERPOM_ERROR_NONE;
	}
}

#define _EEPROM_WRITE_U8(_addr, _u8) \
	do \
	{ \
		(*((PointerAttr uint8_t *)((uint16_t)(_addr)))) = (uint8_t)_u8; \
	} while(0)

static inline void _EEPROM_WRITE_U32(const void * address, const uint32_t *data)
{
	/* Enable Word Write Once */
	FLASH->CR2  |= FLASH_CR2_WPRG;
	FLASH->NCR2 &= ~FLASH_NCR2_NWPRG;
	
	(*((PointerAttr uint8_t *)((uint16_t)(address))))     = (*((uint8_t*)(data)));
	(*((PointerAttr uint8_t *)((uint16_t)(address)) + 1)) = (*((uint8_t*)(data) + 1));
	(*((PointerAttr uint8_t *)((uint16_t)(address)) + 2)) = (*((uint8_t*)(data) + 2));
	(*((PointerAttr uint8_t *)((uint16_t)(address)) + 3)) = (*((uint8_t*)(data) + 3));
}
//------------------------------------------------------------------------------
tEERPOM_Error _EEPROM_WRITE(void * address, const void * data, const size_t length)
{
	size_t m_index  = 0;
	uint8_t m_pre   = ((uint16_t)address) & 0x0003;
	uint8_t *m_addr = (uint8_t *)address;
	
	if (m_pre == 3)
	{
		_EEPROM_WRITE_U8(address, ((uint8_t *)data)[0]);
		
		if (_EEPROM_WAIT_FOR_END() != EERPOM_ERROR_NONE)
			return EERPOM_ERROR_WRITE;
		
		++m_index;
		++m_addr;
		
		m_pre = 0;
	}
	
	m_addr -= m_pre;
	
	while (m_index < length)
	{
		//--------------------------------------------------------------------------
		if ((length - m_index) == 1)
		{
			_EEPROM_WRITE_U8((m_addr + m_pre), ((uint8_t *)data)[m_index]);
			
			return _EEPROM_WAIT_FOR_END();
		}
		//--------------------------------------------------------------------------
		uint8_t _buffer[4];
		
		for (size_t j = 0; j < 4; ++j)
		{
			if (m_pre != 0)
			{
				_buffer[j] = m_addr[j];
				--m_pre;
			}
			else if (m_index < length)
			{
				_buffer[j] = ((uint8_t *)data)[m_index];
				++m_index;
			}
			else
			{
				_buffer[j] = m_addr[j];
			}
		}
		//--------------------------------------------------------------------------
		//_EEPROM_WRITE_U32(m_addr, (uint32_t *)_buffer);
		//--------------------------------------------------------------------------
		FLASH->CR2  |= FLASH_CR2_WPRG;
		FLASH->NCR2 &= ~FLASH_NCR2_NWPRG;
		
		m_addr[0] = _buffer[0];
		m_addr[1] = _buffer[1];
		m_addr[2] = _buffer[2];
		m_addr[3] = _buffer[3];
		
		if (_EEPROM_WAIT_FOR_END() != EERPOM_ERROR_NONE)
			return EERPOM_ERROR_WRITE;
		//--------------------------------------------------------------------------
		m_addr += 4;
		//--------------------------------------------------------------------------
	}
	
	return EERPOM_ERROR_NONE;
}
//------------------------------------------------------------------------------
tEERPOM_Error EEPROM_CheckFile(tEEPROM_File *file)
{
	uint16_t m_length = (file->Length - BSP_EEPROM_HASH_SIZE);
	uint16_t m_crc = CRC16((const void *)(file->Address), m_length);

	m_crc ^= (*(uint16_t *)(file->Address + m_length));

	return (m_crc)?(EEPROM_ERROR_HASH):(EERPOM_ERROR_NONE);
}
//------------------------------------------------------------------------------
tEERPOM_Error EEPROM_UpdateFile(tEEPROM_File *file)
{
	uint16_t m_length = (file->Length - BSP_EEPROM_HASH_SIZE);
	uint16_t m_crc = CRC16((const void *)(file->Address), m_length);

	return _EEPROM_WRITE((void *)((uint16_t)file->Address + m_length), &m_crc, BSP_EEPROM_HASH_SIZE);
}
//------------------------------------------------------------------------------
tEERPOM_Error EEPROM_WriteFile(tEEPROM_File *file, void *buffer, size_t offset, size_t length)
{
	tEERPOM_Error m_error = EERPOM_ERROR_NONE;

	_EEPROM_UNLOCK();

	if ((offset + length) <= (file->Length - BSP_EEPROM_HASH_SIZE))
	{
		void *m_addr = (void *)(file->Address + offset);

		m_error = _EEPROM_WRITE(m_addr, buffer, length);

		if (m_error == EERPOM_ERROR_NONE)
		{
			m_error = EEPROM_UpdateFile(file);
			
			if (m_error == EERPOM_ERROR_NONE)
				m_error = EEPROM_CheckFile(file);
		}
	}
	else
		m_error = EERPOM_ERROR_SIZE;

	_EEPROM_LOCK();
	
	return m_error;
}
//------------------------------------------------------------------------------
tEERPOM_Error EEPROM_ReadFile(tEEPROM_File *file, void *buffer, size_t offset, size_t length)
{
	if ((offset + length) <= (file->Length - BSP_EEPROM_HASH_SIZE))
	{
		const uint8_t *m_data = (const uint8_t*)(file->Address + offset);

		for (size_t i = 0; i < length; ++i)
			((uint8_t *)buffer)[i] = m_data[i];

		return EERPOM_ERROR_NONE;
	}
	else
		return EERPOM_ERROR_SIZE;
}
//------------------------------------------------------------------------------