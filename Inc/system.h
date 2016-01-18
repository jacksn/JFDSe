//  patches to toolchain
// uncomment mthumb-interwork in gcc/config/t-arm-elf
// 	--disable-hosted-libstdcxx
// 	empty crt0.s in newlib/libgloss

#ifndef _SYSTEM_
#define _SYSTEM_

#include "types.h"

#ifdef __cplusplus
extern "C"
{
#endif


    #include "ff.h"
    #include "diskio.h"

    #include "crc16.h"

    #include <stdio.h>
    #include <string.h>
    #include <time.h>

    //-------------------------------------------------------------------------

    void UART0_WriteText( const char *str );
    void FPGA_Config();

    void Keyboard_Reset();
    void Mouse_Reset();

    void Spectrum_UpdateConfig();
    void Spectrum_UpdateDisks();

    void BDI_Routine();
    void BDI_ResetWrite();
    void BDI_Write( byte data );
    void BDI_ResetRead( word counter );
    bool BDI_Read( byte *data );
    void BDI_StopTimer();
    void BDI_StartTimer();


    #define CNTR_INTERVAL 1

    dword get_ticks();

    void __TRACE( const char *str, ... );

#ifdef __cplusplus
}
#endif

#define PATH_SIZE 0x80

#ifdef __cplusplus
    #include <string.h>

    bool SystemBus_TestConfiguration();
    void SystemBus_SetAddress( dword address );
    word SystemBus_Read();
    word SystemBus_Read( dword address );
    void SystemBus_Write( word data );
    void SystemBus_Write( dword address, word data );

    bool FileExists( const char *str );


    void Timer_Routine();
    dword Timer_GetTickCounter();

#endif

#endif
