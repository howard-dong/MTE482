/*
 * soft_pwm_lib.h
 *
 *  Created on: Mar 12, 2024
 *      Author: ajitr
 */

#ifndef INC_FLASH_LIB_H_
#define INC_FLASH_LIB_H_

#include "stdint.h"

struct record_header_S {
	uint16_t record_count; // record num
	uint16_t record_length; // num of bytes in record including the size of the header
};

struct record_S {
	struct record_header_S header;
	uint32_t data; // FIX: CHANGE THIS SO THAT IT ACTUALLY MATCHES THE DATA WE'RE SAVING
};


struct record_manager_S {
	uint32_t current_addr;
	struct record_S current_record;
};

uint32_t flash_lib_write_to_addr(uint32_t memory_address, struct record_S* data_to_write, uint16_t num_words);

void flash_lib_read_from_addr(uint32_t memory_address, struct record_S* rx_buf, uint16_t num_words);

//void flash_lib_convert_to_str(uint32_t* data_to_write, char* buf);
//
//void flash_lib_write_float(uint32_t memory_address, float value);
//
//float flash_lib_read_float(uint32_t memory_address);

uint32_t flash_lib_write_current_record(uint32_t* data_to_write);

uint32_t flash_lib_read_current_record(uint32_t* rx_buf);

uint32_t flash_lib_init(void);


// TODO: Delete
void test_write(void);
void test_read(void);

#endif /* INC_FLASH_LIB_H_ */
