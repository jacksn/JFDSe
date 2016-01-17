//YWROBOT
//last updated on 21/12/2011
//Tim Starling Fix the reset bug (Thanks Tim)
//wiki doc http://www.dfrobot.com/wiki/index.php?title=I2C/TWI_LCD1602_Module_(SKU:_DFR0063)
//Support Forum: http://www.dfrobot.com/forum/
//Compatible with the Arduino IDE 1.0
//Library version:1.1

#include "i2c.h"
#include "binary.h"
#include "lcd1602.h"
#include "stdio.h"

// When the display powers up, it is configured as follows:
//
// 1. Display clear
// 2. Function set: 
//    DL = 1; 8-bit interface data 
//    N = 0; 1-line display 
//    F = 0; 5x8 dot character font 
// 3. Display on/off control: 
//    D = 0; Display off 
//    C = 0; Cursor off 
//    B = 0; Blinking off 
// 4. Entry mode set: 
//    I/D = 1; Increment by 1
//    S = 0; No shift 
//
// Note, however, that resetting the Arduino doesn't reset the LCD, so we
// can't assume that its in that state when a sketch starts (and the
// LiquidCrystal constructor is called).

void lcdInit()
{
	lcdCols = 16;
	lcdRows = 2;


	lcdDisplayFunction = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;
	lcdNumLines = lcdRows;

	// SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
	// according to datasheet, we need at least 40ms after power rises above 2.7V
	// before sending commands. Arduino can turn on way befer 4.5V so we'll wait 50
  
	// Now we pull both RS and R/W low to begin commands
	lcdExpanderWrite(lcdBacklightVal);	// reset expander and turn backlight off (Bit 8 =1)
	//delay_ms(200);

  	//put the LCD into 4 bit mode
	// this is according to the hitachi HD44780 datasheet
	// figure 24, pg 46
	
	// we start in 8bit mode, try to set 4 bit mode
	lcdWrite4bits(0x03 << 4);
	HAL_Delay(5); // wait min 4.1ms
   
	// second try
	lcdWrite4bits(0x03 << 4);
	HAL_Delay(5); // wait min 4.1ms
   
	// third go!
	lcdWrite4bits(0x03 << 4);
	HAL_Delay(2);
   
	// finally, set to 4-bit interface
	lcdWrite4bits(0x02 << 4);
	
	// set # lines, font size, etc.
	lcdSendCommand(LCD_FUNCTIONSET | lcdDisplayFunction);
	
	// turn the display on with no cursor or blinking default
	lcdDisplayControl = LCD_DISPLAYOFF | LCD_CURSOROFF | LCD_BLINKOFF;
	lcdSendCommand(LCD_DISPLAYCONTROL | lcdDisplayControl);
	
	// clear it off
	lcdClear();
	
	// Initialize to default text direction (for roman languages)
	// set the entry mode
	lcdDisplayMode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
	lcdSendCommand(LCD_ENTRYMODESET | lcdDisplayMode);
	
	//home();
	lcdBacklightOn();
	lcdDisplayOn();
	
}

/********** high level commands, for the user! */
void lcdClear(){
	lcdSendCommand(LCD_CLEARDISPLAY);// clear display, set cursor position to zero
	HAL_Delay(2);  // this command takes a long time!
}

void lcdHome(){
	lcdSendCommand(LCD_RETURNHOME);  // set cursor position to zero
	HAL_Delay(2);  // this command takes a long time!
}

void lcdSetCursor(uint8_t col, uint8_t row){
	int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	if ( row > lcdNumLines ) {
		row = lcdNumLines-1;    // we count rows starting w/0
	}
	lcdSendCommand(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

// Turn the display on/off (quickly)
void lcdDisplayOff() {
	lcdDisplayControl &= ~LCD_DISPLAYON;
	lcdSendCommand(LCD_DISPLAYCONTROL | lcdDisplayControl);
}
void lcdDisplayOn() {
	lcdDisplayControl |= LCD_DISPLAYON;
	lcdSendCommand(LCD_DISPLAYCONTROL | lcdDisplayControl);
}

// Turns the underline cursor on/off
void lcdCursorOff() {
	lcdDisplayControl &= ~LCD_CURSORON;
	lcdSendCommand(LCD_DISPLAYCONTROL | lcdDisplayControl);
}
void lcdCursorOn() {
	lcdDisplayControl |= LCD_CURSORON;
	lcdSendCommand(LCD_DISPLAYCONTROL | lcdDisplayControl);
}

// Turn on and off the blinking cursor
void lcdBlinkOff() {
	lcdDisplayControl &= ~LCD_BLINKON;
	lcdSendCommand(LCD_DISPLAYCONTROL | lcdDisplayControl);
}
void lcdBlinkOn() {
	lcdDisplayControl |= LCD_BLINKON;
	lcdSendCommand(LCD_DISPLAYCONTROL | lcdDisplayControl);
}

// These commands scroll the display without changing the RAM
void lcdScrollDisplayLeft(void) {
	lcdSendCommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void lcdScrollDisplayRight(void) {
	lcdSendCommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void lcdLeftToRight(void) {
	lcdDisplayMode |= LCD_ENTRYLEFT;
	lcdSendCommand(LCD_ENTRYMODESET | lcdDisplayMode);
}

// This is for text that flows Right to Left
void lcdRightToLeft(void) {
	lcdDisplayMode &= ~LCD_ENTRYLEFT;
	lcdSendCommand(LCD_ENTRYMODESET | lcdDisplayMode);
}

// This will 'right justify' text from the cursor
void lcdAutoscrollOn(void) {
	lcdDisplayMode |= LCD_ENTRYSHIFTINCREMENT;
	lcdSendCommand(LCD_ENTRYMODESET | lcdDisplayMode);
}

// This will 'left justify' text from the cursor
void lcdAutoscrollOff(void) {
	lcdDisplayMode &= ~LCD_ENTRYSHIFTINCREMENT;
	lcdSendCommand(LCD_ENTRYMODESET | lcdDisplayMode);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void lcdCreateChar(uint8_t location, uint8_t charmap[]) {
	location &= 0x7; // we only have 8 locations 0-7
	lcdSendCommand(LCD_SETCGRAMADDR | (location << 3));
	uint8_t i = 0;
	for (i = 0; i<8; i++) {
		lcdWrite(charmap[i]);
	}
}

// Turn the (optional) backlight off/on
void lcdBacklightOff() {
	lcdBacklightVal = LCD_NOBACKLIGHT;
	lcdExpanderWrite(0);
}

void lcdBacklightOn() {
	lcdBacklightVal = LCD_BACKLIGHT;
	lcdExpanderWrite(0);
}

/*********** mid level commands, for sending data/cmds */

void lcdSendCommand(uint8_t value)
{
	lcdSend(value, 0);
}

void lcdSendData(uint8_t value)
{
	lcdSend(value, RS);
}

/************ low level data pushing commands **********/

// write either command or data
void lcdSend(uint8_t value, uint8_t mode) {
	uint8_t highnib = value & 0xf0;
	uint8_t lownib = (value<<4) & 0xf0;
    lcdWrite4bits((highnib) | mode);
	lcdWrite4bits((lownib) | mode);
}

void lcdWrite4bits(uint8_t value) {
	i2cCount = 3;

	i2cBuffer[0] = value | lcdBacklightVal;
	i2cBuffer[1] = value | lcdBacklightVal | EN;
	i2cBuffer[2] = value | lcdBacklightVal;

	HAL_I2C_Master_Transmit_IT(&hi2c1, lcdAddress, i2cBuffer, i2cCount);

	HAL_Delay(2);
}

void lcdExpanderWrite(uint8_t value){

	i2cBuffer[0] = value;

	i2cCount = 1;

	//i2cSendBytes(lcdAddress);
	HAL_I2C_Master_Transmit_IT(&hi2c1, lcdAddress, i2cBuffer, i2cCount);

	HAL_Delay(2);
}

void lcdPrintStr(char *Text)
{
    char *c;
    c = Text;
    while ((c != 0) && (*c != 0))
    {
        lcdSendData(*c);
        c++;
    }
}


void lcdPrintNum(int value)
{
	char buf[16];				// буфер
	sprintf(buf, "%d", value);	// число превращаем в строку
	lcdPrintStr(buf);
}
