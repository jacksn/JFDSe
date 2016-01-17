//#include "stm32l1xx_rcc.h"
#include <stdint.h>

uint_fast8_t i2cBuffer[32];
uint_fast8_t i2cCount;

void initI2C (void);
void i2cSendBytes(uint_fast8_t i2cAddress);
void i2cReadBytes(uint_fast8_t i2cAddress);
void i2cReadBytes8bitAddress(uint_fast8_t i2cAddress, uint_fast8_t i2cTargetAddress);
