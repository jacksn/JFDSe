#ifndef SPEC_BETADISK_H_INCLUDED
#define SPEC_BETADISK_H_INCLUDED

#include "types.h"
#define ADVANCED_BETADISK 1

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

#define BETADSK_NUM_DRIVES 2

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

int open_dsk_image( byte drv_id, const char *filename );
void close_dsk_image( byte drv_id );

void beta_write_port( byte port, byte data );
byte beta_read_port( byte port );

byte beta_get_state();
byte beta_cur_drv();
int beta_leds();

byte beta_is_disk_wp( byte drv );
void beta_set_disk_wp( byte drv, byte wp );

byte beta_is_disk_loaded( byte drv );

#endif

