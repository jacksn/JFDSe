#ifndef TYPES_H_INCLUDED
#define TYPES_H_INCLUDED

#include "system.h"

typedef unsigned int dword;
//typedef unsigned short word;
//typedef unsigned char byte;
#define word uint16_t
#define byte uint8_t

#ifndef __cplusplus
typedef enum { false = 0, true  = !false } bool;
#endif

#define BOOL bool
#define TRUE true
#define FALSE false

#ifndef min
	#define min( x, y ) ( x > y ? y : x )
#endif

#ifndef max
	#define max( x, y ) ( x > y ? x : y )
#endif

#define countof(a)   (sizeof(a) / sizeof(*(a)))

#ifdef __GNUC__
#define PACKED __attribute__((packed))
#define WEAK __attribute__((weak))
#else
#define PACKED
#define WEAK
#endif

#if defined(PIC_CTRL)
#define PROGMEM __attribute__((space(auto_psv)))
#define PROGMEM_PTR
#else
#define PROGMEM
#define PROGMEM_PTR
#endif

typedef struct
{
	char name[ PATH_SIZE ];
	int readOnly;
} CDiskImage;

#endif
