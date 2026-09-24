#ifndef BALANCE_HOST_STUBS_H_
#define BALANCE_HOST_STUBS_H_

#include <stdint.h>

/* Replace only the target-specific interrupt and Flash headers for the host test. */
#define _zf_common_interrupt_h_
#define _zf_driver_flash_h_

typedef uint8_t uint8;
typedef uint32_t uint32;
typedef union { uint32 uint32_type; } flash_data_union;

#define EEPROM_PAGE_LENGTH 256
extern flash_data_union flash_union_buffer[EEPROM_PAGE_LENGTH];
uint32 interrupt_global_disable(void);
void interrupt_global_enable(uint32 state);
void flash_read_page_to_buffer(uint32 sector, uint32 page);
uint8 flash_write_page_from_buffer(uint32 sector, uint32 page);

#endif
