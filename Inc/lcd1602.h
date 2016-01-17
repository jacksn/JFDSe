#include "binary.h"
#include <stdint.h>

#define lcdAddress				0x4E
// commands
#define LCD_CLEARDISPLAY 		0x01
#define LCD_RETURNHOME 			0x02
#define LCD_ENTRYMODESET 		0x04
#define LCD_DISPLAYCONTROL 		0x08
#define LCD_CURSORSHIFT 		0x10
#define LCD_FUNCTIONSET 		0x20
#define LCD_SETCGRAMADDR 		0x40
#define LCD_SETDDRAMADDR 		0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 			0x00
#define LCD_ENTRYLEFT 			0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 			0x04
#define LCD_DISPLAYOFF 			0x00
#define LCD_CURSORON 			0x02
#define LCD_CURSOROFF 			0x00
#define LCD_BLINKON 			0x01
#define LCD_BLINKOFF 			0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 		0x08
#define LCD_CURSORMOVE 			0x00
#define LCD_MOVERIGHT 			0x04
#define LCD_MOVELEFT 			0x00

// flags for function set
#define LCD_8BITMODE 			0x10
#define LCD_4BITMODE 			0x00
#define LCD_2LINE 				0x08
#define LCD_1LINE 				0x00
#define LCD_5x10DOTS 			0x04
#define LCD_5x8DOTS 			0x00

// flags for backlight control
#define LCD_BACKLIGHT 			0x08
#define LCD_NOBACKLIGHT 		0x00

#define EN B00000100  // Enable bit
#define RW B00000010  // Read/Write bit
#define RS B00000001  // Register select bit

uint8_t i2cBuffer[32];
uint16_t i2cCount;
extern I2C_HandleTypeDef hi2c2;

uint8_t lcdDisplayFunction;
uint8_t lcdDisplayControl;
uint8_t lcdDisplayMode;
uint8_t lcdNumLines;
uint8_t lcdCols;
uint8_t lcdRows;
uint8_t lcdBacklightVal;

void lcdInit();
void lcdClear();
void lcdHome();
void lcdDisplayOff();
void lcdDisplayOn();
void lcdBlinkOff();
void lcdBlinkOn();
void lcdCursorOff();
void lcdCursorOn();
void lcdScrollDisplayLeft();
void lcdScrollDisplayRight();
void lcdPrintLeft();
void lcdPrintRight();
void lcdLeftToRight();
void lcdRightToLeft();
void lcdShiftIncrement();
void lcdShiftDecrement();
void lcdBacklightOff();
void lcdBacklightOn();
void lcdAutoscrollOn();
void lcdAutoscrollOff();
void lcdCreateChar(uint8_t, uint8_t[]);
void lcdSetCursor(uint8_t, uint8_t);
void lcdWrite(uint8_t);
void lcdSendCommand(uint8_t);
void lcdSendData(uint8_t);

void lcdSend(uint8_t, uint8_t);
void lcdWrite4bits(uint8_t);
void lcdExpanderWrite(uint8_t);
void lcdPrintStr(char *Text);
void lcdPrintNum(int value);

