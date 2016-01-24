#ifndef CRC16_H_INCLUDED
#define CRC16_H_INCLUDED

#include "types.h"
#include "stdint.h"
uint16_t crc_init();
uint16_t crc_add( uint16_t crc, uint8_t data );

#endif
