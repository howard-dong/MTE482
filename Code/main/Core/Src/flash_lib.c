/*
 * flash_lib.c
 *
 *  Created on: Feb 10, 2024
 *      Author: ajitr
 */


/* TODO:
 * change flash lib write to just take in a struct/amount of data?
 * change read?
 * i want to wrap the existing read and write functions
 * can probably delete read/write_float
 * figure out error code scheme (what do if read/write fail? how do we want to bubble up errors?)
 * * could just propagate the HAL_FLASH_error
 *
 */

/*
 * INCLUDES
 */

#include "flash_lib.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include "stdio.h"

struct record_manager_S record_manager = {0U};

/*
 * DEFINES
 */

/* Flash sectors defined in ref manual:
 *  Sector 0 = 16KB (stores FW, avoid using if possible)
 *  Sector 1 = 16KB
 *  Sector 2 = 16KB
 *  Sector 3 = 16KB
 *  Sector 4 = 64KB
 *  Sector 5 = 128KB
 *  Sector 6 = 128KB
 *  Sector 7 = 128KB
 */

#define START_SECTOR_0 0x08000000U
#define START_SECTOR_1 0x08004000U
#define START_SECTOR_2 0x08008000U
#define START_SECTOR_3 0x0800C000U
#define START_SECTOR_4 0x08010000U
#define START_SECTOR_5 0x08020000U
#define START_SECTOR_6 0x08040000U
#define START_SECTOR_7 0x08060000U
#define END_OF_MEMORY  0x0807FFFFU

#define START_OF_MEMORY START_SECTOR_3


/*
 * PRIVATE/HELPER FUNCTIONS
 */


/*
 * TODO: Function descriptions
 */
// TODO: error codes for return
static uint32_t _flash_lib_get_sector(uint32_t addr, uint8_t* sector)
{
	uint32_t ret = 0U;
	if ((START_SECTOR_0 <= addr) && (addr < START_SECTOR_1))
	{
		*sector = FLASH_SECTOR_0;
	}
	else if ((START_SECTOR_1 <= addr) && (addr < START_SECTOR_2))
	{
		*sector = FLASH_SECTOR_1;
	}
	else if ((START_SECTOR_2 <= addr) && (addr < START_SECTOR_3))
	{
		*sector = FLASH_SECTOR_2;
	}
	else if ((START_SECTOR_3 <= addr) && (addr < START_SECTOR_4))
	{
		*sector = FLASH_SECTOR_3;
	}
	else if ((START_SECTOR_4 <= addr) && (addr < START_SECTOR_5))
	{
		*sector = FLASH_SECTOR_4;
	}
	else if ((START_SECTOR_5 <= addr) && (addr < START_SECTOR_6))
	{
		*sector = FLASH_SECTOR_5;
	}
	else if ((START_SECTOR_6 <= addr) && (addr < START_SECTOR_7))
	{
		*sector = FLASH_SECTOR_6;
	}
	else if ((START_SECTOR_7 <= addr) && (addr < END_OF_MEMORY))
	{
		*sector = FLASH_SECTOR_7;
	}

	else
	{
		// TODO: set up proper error codes and handling here
		ret = 1U;
	}

	return ret;
}



// TODO: Do we need to handle doubles?
void _float_to_bytes(uint8_t* byte_buf, float var)
{
	union converter_u{
		float var_to_convert;
		uint8_t converted_bytes[4];
	} converter_u;

	converter_u.var_to_convert = var;

	memcpy(byte_buf, converter_u.converted_bytes, 4);

//	for (uint8_t i = 0; i < 4; i++) {
//		byte_buf[i] = converter_u.buf[i];
//	}
}

// TODO: Do we need to handle doubles
void _bytes_to_float(uint8_t* bytes_to_convert, float* var)
{
    union converter_u{
      float converted_var;
      uint8_t byte_buf[4];
    } converter_u;

    memcpy(converter_u.byte_buf, bytes_to_convert, 4);

//    for (uint8_t i = 0; i < 4; i++) {
//    	thing.bytes[i] = ftoa_bytes_temp[i];
//    }

    *var = converter_u.converted_var;
}


void _flash_lib_read_header(uint32_t memory_address, struct record_header_S* rx_header)
{
	flash_lib_read_from_addr(memory_address, (struct record_S*)rx_header, sizeof(struct record_header_S));
}


/*
 * PUBLIC FUNCTIONS
 */

uint32_t flash_lib_write_to_addr(uint32_t memory_address, struct record_S* data_to_write, uint16_t num_words)
{

	static FLASH_EraseInitTypeDef erase_init_S;
	uint32_t sector_error;
	uint8_t StartSector;
	uint8_t EndSector;
	uint32_t error = 0U;
	int words_completed=0;


	/* Unlock the Flash to enable the flash control register access */
	HAL_FLASH_Unlock();

	/* Erase the user Flash area */
	/* Get the number of sector to erase from 1st sector */

	error = _flash_lib_get_sector(memory_address, &StartSector);
//	uint32_t EndSectorAddress = memory_address + num_words*4; // TODO: Delete?
	error = _flash_lib_get_sector(memory_address, &EndSector);

	/* Fill EraseInit structure */
	erase_init_S.TypeErase     = FLASH_TYPEERASE_SECTORS;
	erase_init_S.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	erase_init_S.Sector        = StartSector;
	erase_init_S.NbSectors     = (EndSector - StartSector) + 1;

	/*
	* Note: If an erase operation in Flash memory also concerns data_to_write in the data_to_write or instruction cache,
	* you have to make sure that these data_to_write are rewritten before they are accessed during code
	* execution. If this cannot be done safely, it is recommended to flush the caches by setting the
	* DCRST and ICRST bits in the FLASH_CR register.
	*/
	if (HAL_OK == HAL_FLASHEx_Erase(&erase_init_S, &sector_error))
	{
		/* Program the user Flash area word by word
		(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) */
		while (words_completed<num_words)
		{
			if (HAL_OK == HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, memory_address, ((uint64_t*)(data_to_write))[words_completed]))
			{
				/* use memory_address += 2 for half word and 8 for double word */
				memory_address += 4;
				words_completed++;
			}
			else
			{
				/* Error occurred while writing data_to_write in Flash memory */
				error = HAL_FLASH_GetError();
			}
		}
	}
	else
	{
		/* Error occurred during erase */
		error = HAL_FLASH_GetError();
	}

	/*
	 * Lock the Flash to disable the flash control register access (recommended
	 * to protect the FLASH memory against possible unwanted operation)
	 */
	HAL_FLASH_Lock();

	return error;
}


void flash_lib_read_from_addr(uint32_t memory_address, struct record_S* rx_buf, uint16_t num_words)
{
	uint32_t addr = memory_address;
	// TODO: Should this be num_words+1?
	for (uint8_t i = 0U; i < num_words; i++)
	{
		addr += (4*i);
		((uint32_t*)(rx_buf))[i] = *(__IO uint32_t *)(addr);
	}
}


//// TODO: We can probably delete this func tbh, mainly here for testing
//void flash_lib_convert_to_str(uint32_t *data_to_write, char *buf)
//{
//	int numberofbytes = ((strlen((char *)data_to_write)/4) + ((strlen((char *)data_to_write) % 4) != 0)) *4;
//
//	for (int i=0; i<numberofbytes; i++)
//	{
//		buf[i] = data_to_write[i/4]>>(8*(i%4));
//	}
//}


//void flash_lib_write_float(uint32_t memory_address, float value)
//{
//	struct record_S temp_record = {0};
//	_float_to_bytes(bytes_temp, value);
//
//	flash_lib_write_to_addr(memory_address, &temp_record, 1);
//}


//float flash_lib_read_float(uint32_t memory_address)
//{
//	uint8_t buffer[4];
//	float value;
//
//	flash_lib_read_from_addr(memory_address, (uint32_t*)buffer, 1);
//	_bytes_to_float(buffer, &value);
//	return value;
//}


uint32_t flash_lib_write_current_record(uint32_t* data_to_write)
{
	uint32_t error = 0U;

	// FIX: sizeof should be of whatever the size of the data will be; make a const?
	memcpy(&record_manager.current_record.data, data_to_write, sizeof(uint32_t));
	record_manager.current_record.header.record_length = sizeof(uint32_t) + sizeof(struct record_header_S);

	flash_lib_write_to_addr(record_manager.current_addr, &record_manager.current_record, record_manager.current_record.header.record_length);

	record_manager.current_addr += record_manager.current_record.header.record_length;
	record_manager.current_record.header.record_count++;

	return error;
}

uint32_t flash_lib_read_current_record(uint32_t* rx_buf)
{
	uint32_t error = 0U;

	flash_lib_read_from_addr(record_manager.current_addr, &record_manager.current_record, record_manager.current_record.header.record_length);

	memcpy(rx_buf, &record_manager.current_record, record_manager.current_record.header.record_length);

	return error;
}


// TODO: Implement error handling
uint32_t flash_lib_init(void)
{
	uint32_t cur_addr = START_OF_MEMORY;
	uint32_t error = 0U;
	struct record_header_S prev_header = {0};
	struct record_header_S header = {0};
//	error = _flash_lib_read_header(START_OF_MEMORY, &header);
	_flash_lib_read_header(cur_addr, &header);

	while(header.record_count < 0xFFU)
	{
		cur_addr += header.record_length;
		memcpy(&prev_header, &header, sizeof(struct record_header_S));
//		error = _flash_lib_read_header(START_OF_MEMORY + header.record_length, &header);
		_flash_lib_read_header(cur_addr, &header);
	}

	memcpy(&record_manager.current_record.header, &prev_header, sizeof(struct record_header_S));
	record_manager.current_addr = cur_addr - prev_header.record_length;

	return error;
}



void test_write(void)
{
	record_manager.current_addr = START_OF_MEMORY;

	uint32_t data = 0xDDU;
	flash_lib_write_current_record(&data);
}

void test_read(void)
{
	uint32_t data = 0U;
	flash_lib_read_current_record(&data);

}


