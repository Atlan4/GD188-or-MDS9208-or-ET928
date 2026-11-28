/*
 * lcd.h
 *
 *  Created on: Nov 4, 2026
 *      Author: 42190
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "stm32f4xx.h"
#include "rotary_switch.h"

// ==============================================================================
// HARDWARE PIN DEFINITIONS
// ==============================================================================

// 1. Control lines for commands and data latching (Port B)
#define LCD_WR_PIN      14   // Write control pin
#define LCD_RS_PIN      15   // Register Select pin (Command/Data switch)

// 2. Static lines configured during hardware setup
#define LCD_RD_PORT     GPIOB
#define LCD_RD_PIN      13   // Statically pulled high (Logical 1)

#define LCD_RST_PORT    GPIOC
#define LCD_RST_PIN     8    // Hardware Reset pin

// 3. Power management and backlight
#define LCD_PWR_PORT    GPIOA
#define LCD_PWR_PIN     2

#define LCD_BL_PORT     GPIOB
#define LCD_BL_PIN      11

// Display resolution constraints in landscape orientation
#define LCD_WIDTH       320
#define LCD_HEIGHT      240

// Predefined color palettes using RGB565 format
#define LCD_BLACK   0x0000
#define LCD_WHITE   0xFFFF
#define LCD_RED     0xF800
#define LCD_GREEN   0x07E0
#define LCD_BLUE    0x001F
#define LCD_YELLOW  0xFFE0
#define LCD_ORANGE  0xFD20 // Vibrant orange matching the multimeter highlights
#define LCD_CYAN    0x07FF  // Jasná tyrkysová farba pre RE
#define LCD_GREY    0x52AA // Dark grey optimized for oscilloscope grid lines
#define LCD_LIGHTGREY 0xC618 // Light grey for secondary indicators
#define LCD_DARKBLUE  0x010A // Deep background blue if needed
#define LCD_SEL_BG    LCD_GREEN  // Pozadie aktívnej voľby (Hz / DUTY, OFF / ON v menu)
#define LCD_SEL_FG    LCD_BLACK  // Text aktívnej voľby

// Definícia rozmerov fontu pre jednoduchú zmenu v budúcnosti
#define FONT_7SEG_WIDTH         45    // Skutočná šírka cifier v pixeloch
#define FONT_7SEG_HEIGHT        72    // Výška fontu v pixeloch (riadky)
// Tento vzorec zaokrúhli bajty vždy správne nahor (pre 31px to bude presne 4 bajty)
//#define FONT_7SEG_BYTES_PER_ROW ((FONT_7SEG_WIDTH + 7) / 8)
#define FONT_7SEG_BYTES_PER_ROW 6


// Core display engine functions

void LCD_Power_Accessories(void);
void LCD_Power_Off(void);
void LCD_Init_Hardware(void);
void LCD_Init(void);
void LCD_BL_on(void);
void LCD_BL_SetBrightness(uint8_t duty);
uint8_t LCD_BL_GetBrightness(void);       // jas v % (25 / 50 / 75)
void LCD_WriteCommand(uint8_t cmd);
void LCD_WriteData(uint8_t data);
void LCD_Clear(uint16_t color);

// Graphics, text rendering, and coordinate tracking functions
void LCD_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void LCD_DrawFastHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color);
void LCD_DrawFastVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color);
void LCD_DrawGrid(uint16_t step, uint16_t color);
void LCD_DrawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t size);
uint8_t LCD_FontColumn(char c, uint8_t col);   // v0.70: stĺpec znaku fontu 5x7 (bit 0 = hore)
void LCD_DrawString(uint16_t x, uint16_t y, const char* str, uint16_t color, uint16_t bg, uint8_t size);

#define ICON_NONE   0
#define ICON_DIODE  1
#define ICON_BEEP   2
#define ICON_RESISTOR 3
void LCD_DrawIcon(uint16_t x, uint16_t y, uint8_t icon, uint16_t color, uint16_t bg, uint8_t size);
void LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void LCD_DrawBattery(uint16_t x, uint16_t y, uint8_t percent);
void LCD_DrawBottomMenu(SwitchPosition_t pos, SubMode_t sub, bool rel, int8_t pressed);
void LCD_DrawMenuFrame(void);
void LCD_DrawMenuCell(uint8_t idx, const char *text, uint16_t fg, uint16_t bg);


// Geometrické vykresľovanie číslic z nového 48x60 (360 bajtov) fontu
void LCD_Draw7SegDigit(uint16_t x, uint16_t y, char num, uint16_t color, uint16_t bg);
void LCD_Draw7SegDot(uint16_t x, uint16_t y, uint16_t color, uint16_t bg);


#endif // INC_LCD_H_
