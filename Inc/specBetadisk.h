#ifndef SPEC_BETADISK_H_INCLUDED
#define SPEC_BETADISK_H_INCLUDED

#include "types.h"
#include "system.h"

#if ADVANCED_BETADISK

    #ifdef __cplusplus
    extern "C"
    {
    #endif

    #include "fdc.h"
    #include "floppy.h"
    #include "wd1793.h"

    #ifdef __cplusplus
    }
    #endif

#else

    #define fdc_init            beta_init
    #define fdc_open_image      !open_dsk_image
    #define fdc_write           beta_write_port
    #define fdc_read            beta_read_port

    #define fdc_reset()
    #define fdc_dispatch()

    #define floppy_leds         beta_leds

#endif

#define BETADSK_NUM_DRIVES 4

enum
{
    BETA_IDLE = 0,
    BETA_READ,
    BETA_READ_TRK,
    BETA_READ_ADR,
    BETA_WRITE,
    BETA_WRITE_TRK,
    BETA_SEEK,
};

void beta_init();

int open_dsk_image( uint8_t drv_id, const char *filename );
void close_dsk_image( uint8_t drv_id );

void beta_write_port( uint8_t port, uint8_t data );
uint8_t beta_read_port( uint8_t port );

uint8_t beta_get_state();
uint8_t beta_cur_drv();
int beta_leds();

uint8_t beta_is_disk_wp( uint8_t drv );
void beta_set_disk_wp( uint8_t drv, uint8_t wp );

uint8_t beta_is_disk_loaded( uint8_t drv );

#endif

