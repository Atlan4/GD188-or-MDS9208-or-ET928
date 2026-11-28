#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/*
 * lcd.c
 *
 *  Created on: Nov 4, 2026
 *      Author: 42190
 */


#include "lcd.h"
#include "font5x7.h"
#include "font7seg.h"
//#include "rotary_switch.h"
#include "button.h"
#include "settings.h"
//#include "meas_chip.h"

// Zviditeľnenie štruktúry časovača TIM5, ktorá je definovaná v main.c
extern TIM_HandleTypeDef htim5;



// Safe and verified write to PB0-PB7 via BSRR register (Will not affect PB11, PB13, PB14, PB15)
__STATIC_INLINE void LCD_WriteBus(uint8_t byte) {
    GPIOB->BSRR = 0x00FF0000 | (uint32_t)byte;
}

// Hardware power initialization
void LCD_Power_Accessories(void) {
    // Turn on power to electronics (PA2 = 1)
    LCD_PWR_PORT->BSRR = (1 << LCD_PWR_PIN);
}

// Vypnutie napájania LCD a meracieho čipu (PA2 = 0).
// Najprv všetky signály k displeju do 0 – inak by cez ochranné diódy vypnutého LCD
// tiekol prúd z pinov MCU a vypnutá časť by sa čiastočne napájala.
void LCD_Power_Off(void) {
    GPIO_InitTypeDef gi = {0};

    // Podsvietenie PB11: z PWM (TIM5) na obyčajný výstup v 0
    gi.Pin   = GPIO_PIN_11;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LCD_BL_PORT, &gi);
    LCD_BL_PORT->BSRR = (1u << (LCD_BL_PIN + 16));

    // Dátová zbernica D0..D7 a riadiace RD, WR, RS (všetko GPIOB) do 0
    GPIOB->BSRR = (uint32_t)(lcd_D0_Pin | lcd_D1_Pin | lcd_D2_Pin | lcd_D3_Pin |
                             lcd_D4_Pin | lcd_D5_Pin | lcd_D6_Pin | lcd_D7_Pin |
                             lcd_RD_Pin | lcd_WR_Pin | lcd_RS_Pin) << 16;
    // Reset displeja (PC8) do 0
    LCD_RST_PORT->BSRR = (1u << (LCD_RST_PIN + 16));

    // Vypnúť napájanie elektroniky
    LCD_PWR_PORT->BSRR = (1u << (LCD_PWR_PIN + 16));
}


// Hardware pins and signal lines initialization
void LCD_Init_Hardware(void) {
    // Statically unlock RD pin (PB13 = 1)
    LCD_RD_PORT->BSRR = (1 << LCD_RD_PIN);

    // Hardware reset sequence via unlocked PC8 pin
    LCD_RST_PORT->BSRR = (1 << (LCD_RST_PIN + 16)); // RST = 0 (0V - reset active)
    HAL_Delay(20);
    LCD_RST_PORT->BSRR = (1 << LCD_RST_PIN);        // RST = 1 (3.3V - reset released)
    HAL_Delay(10);

    // Set default idle state for WR and RS lines (Logical 1)
    GPIOB->BSRR = (1 << LCD_WR_PIN);
    GPIOB->BSRR = (1 << LCD_RS_PIN);
}

void LCD_Init(void) {
// Initialize hardware display engine config registers
  LCD_WriteCommand(0x01); // Software Reset
  HAL_Delay(150);//20
  LCD_WriteCommand(0x11); // Sleep Out
  HAL_Delay(10);

  // Memory Data Access Control (Display orientation configuration)
  LCD_WriteCommand(0x36);
  LCD_WriteData(0x60);    // Landscape orientation orientation layout
  LCD_WriteCommand(0x3A); // Pixel Format
  LCD_WriteData(0x55);    // RGB565
  LCD_WriteCommand(0x29); // Display ON

  // Clean the screen canvas
  LCD_Clear(LCD_BLACK);
  HAL_Delay(80);

  // Turn on backlight LEDs
  //LCD_BL_on();
  LCD_BL_SetBrightness(25); // Maximálny výkon podsvietenia
  LCD_BL_on();

  }

// Turn on display backlight
void LCD_BL_on(void) {
    // Turn on backlight pin (PB11 = 1)
    //LCD_BL_PORT->BSRR = (1 << LCD_BL_PIN);
    /* USER CODE BEGIN 2 */
// Spustí hardvérové generovanie PWM na kanáli 4 pre TIM5
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
/* USER CODE END 2 */

}



// 2. Nová funkcia na zápis jasu 0-100% pre TIM5 cez HAL makro
static uint8_t bl_duty = 50;          // posledný nastavený jas (odhad času chodu batérie)

uint8_t LCD_BL_GetBrightness(void) {
    return bl_duty;
}

void LCD_BL_SetBrightness(uint8_t duty) {
    if (duty > 100) duty = 100;
    bl_duty = duty;

    // Zápis striedy priamo do registra CCR4 pre časovač TIM5
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, duty);
}


// Write configuration command (RS = 0)
void LCD_WriteCommand(uint8_t cmd) {
    GPIOB->BSRR = (1 << (LCD_RS_PIN + 16));  // RS = 0 (Command mode)

    LCD_WriteBus(cmd);

    GPIOB->BSRR = (1 << (LCD_WR_PIN + 16));  // WR = 0 (Clock pulse low)
    __NOP();                                 // Stabilization delay
    GPIOB->BSRR = (1 << LCD_WR_PIN);         // WR = 1 (Latch command byte)
    __NOP();
}

// Write data byte or pixel color data (RS = 1)
void LCD_WriteData(uint8_t data) {
    GPIOB->BSRR = (1 << LCD_RS_PIN);         // RS = 1 (Data mode)

    LCD_WriteBus(data);

    GPIOB->BSRR = (1 << (LCD_WR_PIN + 16));  // WR = 0 (Clock pulse low)
    __NOP();
    GPIOB->BSRR = (1 << LCD_WR_PIN);         // WR = 1 (Latch data byte)
    __NOP();
}

// Clear entire screen to a single color
void LCD_Clear(uint16_t color) {
    LCD_SetAddressWindow(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
    uint8_t hi = color >> 8, lo = color & 0xFF;
    for (uint32_t i = 0; i < (LCD_WIDTH * LCD_HEIGHT); i++) {
        LCD_WriteData(hi);
        LCD_WriteData(lo);
    }
}

//**************************************************************************

// Set drawing window coordinates on the display
void LCD_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    LCD_WriteCommand(0x2A); // Column Address Set
    LCD_WriteData(x0 >> 8); LCD_WriteData(x0 & 0xFF);
    LCD_WriteData(x1 >> 8); LCD_WriteData(x1 & 0xFF);

    LCD_WriteCommand(0x2B); // Row Address Set
    LCD_WriteData(y0 >> 8); LCD_WriteData(y0 & 0xFF);
    LCD_WriteData(y1 >> 8); LCD_WriteData(y1 & 0xFF);

    LCD_WriteCommand(0x2C); // Memory Write
}

// v0.70: stĺpec znaku fontu 5x7 (bit 0 = horný riadok), napr. pre skladanie textu v paneli osciloskopu
uint8_t LCD_FontColumn(char c, uint8_t col) {
    uint8_t uc = (uint8_t)c;
    if (uc < 32 || uc > 130) uc = ' ';
    if (col >= 5u) return 0;
    return font5x7[(uint16_t)((uc - 32u) * 5u + col)];
}

// Draw a single character using font bitmap
void LCD_DrawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t size) {
    uint8_t uc = (uint8_t)c;                 // char môže byť so znamienkom (0x80, 0x81)
    if (uc < 32 || uc > 130) uc = ' ';
    uint16_t font_index = (uint16_t)((uc - 32u) * 5u);

    for (uint8_t i = 0; i < 5; i++) {
        uint8_t line = font5x7[font_index + i];
        for (uint8_t j = 0; j < 8; j++) {
            if (line & 0x01) {
                if (size == 1) LCD_DrawPixel(x + i, y + j, color);
                else {
                    for (uint8_t sx = 0; sx < size; sx++) {
                        for (uint8_t sy = 0; sy < size; sy++) {
                            LCD_DrawPixel(x + (i * size) + sx, y + (j * size) + sy, color);
                        }
                    }
                }
            } else if (bg != color) {
                if (size == 1) LCD_DrawPixel(x + i, y + j, bg);
                else {
                    for (uint8_t sx = 0; sx < size; sx++) {
                        for (uint8_t sy = 0; sy < size; sy++) {
                            LCD_DrawPixel(x + (i * size) + sx, y + (j * size) + sy, bg);
                        }
                    }
                }
            }
            line >>= 1;
        }
    }
}

// Draw a single pixel within display boundary limits
void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT) return;
    LCD_SetAddressWindow(x, y, x, y);
    LCD_WriteData(color >> 8);
    LCD_WriteData(color & 0xFF);
}

// Fast horizontal line drawing for landscape mode
void LCD_DrawFastHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color) {
    if (y >= LCD_HEIGHT || x >= LCD_WIDTH) return;
    if ((x + w) > LCD_WIDTH) w = LCD_WIDTH - x;
    LCD_SetAddressWindow(x, y, x + w - 1, y);
    uint8_t hi = color >> 8, lo = color & 0xFF;
    for (uint16_t i = 0; i < w; i++) {
        LCD_WriteData(hi);
        LCD_WriteData(lo);
    }
}

// Fast vertical line drawing for landscape mode
void LCD_DrawFastVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color) {
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT) return;
    if ((y + h) > LCD_HEIGHT) h = LCD_HEIGHT - y;
    LCD_SetAddressWindow(x, y, x, y + h - 1);
    uint8_t hi = color >> 8, lo = color & 0xFF;
    for (uint16_t i = 0; i < h; i++) {
        LCD_WriteData(hi);
        LCD_WriteData(lo);
    }
}

// Draw oscilloscope background grid (Scales up to 320x240 screen area)
void LCD_DrawGrid(uint16_t step, uint16_t color) {
    for (uint16_t y = step; y < LCD_HEIGHT; y += step) {
        LCD_DrawFastHLine(0, y, LCD_WIDTH, color);
    }
    for (uint16_t x = step; x < LCD_WIDTH; x += step) {
        LCD_DrawFastVLine(x, 0, LCD_HEIGHT, color);
    }
}

// Draw text string with automatic 320px border word-wrap tracking
void LCD_DrawString(uint16_t x, uint16_t y, const char* str, uint16_t color, uint16_t bg, uint8_t size) {
    while (*str) {
        LCD_DrawChar(x, y, *str++, color, bg, size);
        x += 6 * size;
        if (x + 6 * size >= LCD_WIDTH) {
            x = 0;
            y += 8 * size;
        }
    }
}

//*************************************************************************************

// --- Ikony 12x8 px (bit 11 = ľavý stĺpec), kreslia sa zväčšené (size 2 = 24x16 = 2 znaky textu) ---
static const uint16_t icon_bitmaps[][8] = {
    { 0 },
    // ICON_DIODE: --|>|-- (začína v stĺpci 0, rovnako ako písmeno V pod ňou)
    {
      0x220,   // ..#...#.....
      0x320,   // ..##..#.....
      0x3A0,   // ..###.#.....
      0xFF0,   // ########....
      0x3A0,   // ..###.#.....
      0x320,   // ..##..#.....
      0x220,   // ..#...#.....
      0x000 }, // ............
    // ICON_BEEP: obrys reproduktora + dve zvukové vlny
    {
      0x088,   // ....#...#...
      0x1A4,   // ...##.#..#..
      0xE94,   // ###.#..#.#..
      0xA94,   // #.#.#..#.#..
      0xE94,   // ###.#..#.#..
      0x1A4,   // ...##.#..#..
      0x088,   // ....#...#...
      0x000 }, // ............
    // ICON_RESISTOR: odpor podľa IEC – obdĺžnik s vývodmi
    {
      0x000,   // ............
      0x3FC,   // ..########..
      0x204,   // ..#......#..
      0xE07,   // ###......###
      0x204,   // ..#......#..
      0x3FC,   // ..########..
      0x000,   // ............
      0x000 }, // ............
};

void LCD_DrawIcon(uint16_t x, uint16_t y, uint8_t icon, uint16_t color, uint16_t bg, uint8_t size) {
    if (icon >= sizeof(icon_bitmaps) / sizeof(icon_bitmaps[0])) icon = 0;
    for (uint8_t row = 0; row < 8; row++) {
        uint16_t bits = icon_bitmaps[icon][row];
        for (uint8_t col = 0; col < 12; col++) {
            uint16_t c = (bits & (0x800 >> col)) ? color : bg;
            for (uint8_t sy = 0; sy < size; sy++) {
                LCD_DrawFastHLine(x + col * size, y + row * size + sy, size, c);
            }
        }
    }
}

// Vyplnený obdĺžnik
void LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    for (uint16_t r = 0; r < h; r++) {
        LCD_DrawFastHLine(x, y + r, w, color);
    }
}

// Batéria 34x16 px (telo 31 px + kontakt 3 px), vnútri percentá malým písmom (zmestí sa "100%").
// Farba: zelená > 50 %, žltá > 20 %, inak červená.
void LCD_DrawBattery(uint16_t x, uint16_t y, uint8_t percent) {
    const uint16_t w = 31, h = 16;
    uint16_t color = (percent > 50) ? LCD_GREEN : (percent > 20) ? LCD_YELLOW : LCD_RED;
    if (percent > 100) percent = 100;

    // Obrys tela
    LCD_DrawFastHLine(x, y, w, color);
    LCD_DrawFastHLine(x, y + h - 1, w, color);
    LCD_DrawFastVLine(x, y, h, color);
    LCD_DrawFastVLine(x + w - 1, y, h, color);
    // Kontakt (+) vpravo
    LCD_FillRect(x + w, y + 5, 3, 6, color);

    // Vnútro + text v strede (font 5x7, krok 6 px)
    LCD_FillRect(x + 1, y + 1, w - 2, h - 2, LCD_BLACK);
    char txt[6];
    snprintf(txt, sizeof(txt), "%u%%", percent);
    uint16_t tw = (uint16_t)strlen(txt) * 6 - 1;
    LCD_DrawString(x + (w - tw) / 2, y + 4, txt, color, LCD_BLACK, 1);
}

// Rozloženie spodného menu (šírka 320 px):
//   okraj 1 px | políčko 78 | čiara 2 | políčko 78 | čiara 2 | políčko 78 | čiara 2 | políčko 78 | okraj 1
//   x:  0       1..78        79-80     81..158      159-160   161..238     239-240   241..318     319
#define MENU_Y       212                    // v0.62: 28 px, text 6 px nad aj pod
#define MENU_H       28
#define MENU_CELL_W  78
#define MENU_PITCH   80

// Jedno políčko spodného menu: vyplní pozadie a vycentruje text (font 2x = 12 px na znak)
void LCD_DrawMenuCell(uint8_t idx, const char *text, uint16_t fg, uint16_t bg) {
    uint16_t cell_x = 1 + idx * MENU_PITCH;

    for (int r = 1; r < (MENU_H - 1); r++) {
        LCD_DrawFastHLine(cell_x, MENU_Y + r, MENU_CELL_W, bg);
    }
    uint16_t len = (uint16_t)strlen(text);
    uint16_t x = cell_x + (MENU_CELL_W - len * 12) / 2;
    LCD_DrawString(x, MENU_Y + 7, text, fg, bg, 2);
}

// Rámik spodného menu: 1 px okraj a 2 px deliace čiary medzi 4 políčkami
void LCD_DrawMenuFrame(void) {
    const uint16_t menu_y = MENU_Y, menu_h = MENU_H;
    LCD_DrawFastHLine(0, menu_y, 320, LCD_WHITE);
    LCD_DrawFastHLine(0, menu_y + menu_h - 1, 320, LCD_WHITE);
    LCD_DrawFastVLine(0,   menu_y, menu_h, LCD_WHITE);
    for (uint16_t i = 1; i <= 3; i++) {
        LCD_DrawFastVLine(i * MENU_PITCH - 1, menu_y, menu_h, LCD_WHITE);
        LCD_DrawFastVLine(i * MENU_PITCH,     menu_y, menu_h, LCD_WHITE);
    }
    LCD_DrawFastVLine(319, menu_y, menu_h, LCD_WHITE);
}

// Vykreslí na spodku obrazovky 4 políčka s popiskami funkcií tlačidiel F1..F4.
//   pos    – poloha otočného prepínača
//   sub    – aktívny sub-mód (v polohe Hz/% sa zvýrazní aktívne Hz alebo %)
//   rel     – relatívne meranie aktívne (REL má trvalo tyrkysové pozadie)
//   pressed – políčko práve držaného tlačidla (0..3), -1 = žiadne; SELECT a RANGE
//             majú zmenené pozadie len kým je tlačidlo stlačené
// Pri "Quick R/D/C keys" = ON v polohe Ω / dióda / pípak:
//   odpor:  F2 ikona diódy,  F3 ikona reproduktora, F4 RANGE
//   dióda:  F2 HOLD,         F3 ikona reproduktora, F4 ikona odporu
//   pípak:  F2 HOLD,         F3 ikona diódy,        F4 ikona odporu

// Políčko s ikonou (skratka na režim): žltá ikona, pri držaní tlačidla čierna na žltom
static void DrawIconCell(uint8_t idx, uint8_t icon, bool pressed) {
    uint16_t fg = pressed ? LCD_BLACK  : LCD_YELLOW;
    uint16_t bg = pressed ? LCD_YELLOW : LCD_BLACK;
    // viditeľná šírka ikony pri size 2: dióda 16 px, reproduktor 22 px, odpor 24 px
    uint16_t w  = (icon == ICON_DIODE) ? 16 : (icon == ICON_BEEP) ? 22 : 24;
    LCD_DrawMenuCell(idx, "", fg, bg);
    LCD_DrawIcon(1 + idx * MENU_PITCH + (MENU_CELL_W - w) / 2, MENU_Y + 7, icon, fg, bg, 2);
}
void LCD_DrawBottomMenu(SwitchPosition_t pos, SubMode_t sub, bool rel, int8_t pressed) {
    extern volatile bool is_hold_active;   // stav HOLD z main.c

    LCD_DrawMenuFrame();

    bool range_pos = (pos == SWITCH_POS_V  || pos == SWITCH_POS_mV ||
                      pos == SWITCH_POS_uA || pos == SWITCH_POS_mA || pos == SWITCH_POS_10A);

    // F1: REL – vo všetkých polohách (tyrkysové pozadie, keď je aktívne)
    if (rel) LCD_DrawMenuCell(0, "REL", LCD_BLACK, LCD_CYAN);
    else     LCD_DrawMenuCell(0, "REL", LCD_CYAN,  LCD_BLACK);

    bool quick = g_settings.quick_rdc && pos == SWITCH_POS_OHM_DIODE_CONT;

    // F2: HOLD (oranžové pozadie, keď je aktívny); pri rýchlych tlačidlách v odpore ikona diódy
    if (quick && sub == SUB_MODE_RESISTANCE) DrawIconCell(1, ICON_DIODE, pressed == 1);
    else if (is_hold_active) LCD_DrawMenuCell(1, "HOLD", LCD_BLACK,  LCD_ORANGE);
    else                     LCD_DrawMenuCell(1, "HOLD", LCD_ORANGE, LCD_BLACK);

    // F3 / F4
    if (quick) {
        DrawIconCell(2, (sub == SUB_MODE_CONTINUITY) ? ICON_DIODE : ICON_BEEP, pressed == 2);
        if (sub == SUB_MODE_RESISTANCE) {
            if (pressed == 3) LCD_DrawMenuCell(3, "RANGE", LCD_BLACK,  LCD_YELLOW);
            else              LCD_DrawMenuCell(3, "RANGE", LCD_YELLOW, LCD_BLACK);
        } else {
            DrawIconCell(3, ICON_RESISTOR, pressed == 3);
        }
    }
    else if (pos == SWITCH_POS_Hz_Pct) {
        // Priama voľba: F3 = Hz, F4 = DUTY. Aktívna voľba: čierny text na zelenom pozadí.
        bool duty = (sub == SUB_MODE_DUTY);
        if (!duty) LCD_DrawMenuCell(2, "Hz",   LCD_SEL_FG, LCD_SEL_BG);
        else       LCD_DrawMenuCell(2, "Hz",   LCD_WHITE,  LCD_BLACK);
        if (duty)  LCD_DrawMenuCell(3, "DUTY", LCD_SEL_FG, LCD_SEL_BG);
        else       LCD_DrawMenuCell(3, "DUTY", LCD_WHITE,  LCD_BLACK);
    }
    else if (range_pos) {
        // F3: "AC/DC" – aktívna časť farebne (DC zelené, AC oranžové ako pri číslach), zvyšok sivý
        bool ac = (sub == SUB_MODE_AC);
        uint16_t bg  = (pressed == 2) ? LCD_GREY : LCD_BLACK;
        uint16_t dim = (pressed == 2) ? LCD_LIGHTGREY : LCD_GREY;
        uint16_t x = 1 + 2 * MENU_PITCH + (MENU_CELL_W - (5 * 12 - 2)) / 2;
        LCD_DrawMenuCell(2, "", dim, bg);                       // iba pozadie
        LCD_DrawString(x,      MENU_Y + 7, "AC", ac ? LCD_ORANGE : dim, bg, 2);
        LCD_DrawString(x + 24, MENU_Y + 7, "/",  dim, bg, 2);
        LCD_DrawString(x + 36, MENU_Y + 7, "DC", ac ? dim : LCD_GREEN, bg, 2);
        if (pressed == 3) LCD_DrawMenuCell(3, "RANGE", LCD_BLACK,  LCD_YELLOW);
        else              LCD_DrawMenuCell(3, "RANGE", LCD_YELLOW, LCD_BLACK);
    }
    else if (pos == SWITCH_POS_OHM_DIODE_CONT) {
        // F3: Ω, dióda, reproduktor vedľa seba – aktuálny režim tyrkysový, ostatné sivé
        // (viditeľné šírky 10 + 16 + 20 px, medzery 8 px -> 62 px, v strede 78 px políčka)
        uint16_t bg  = (pressed == 2) ? LCD_GREY : LCD_BLACK;
        uint16_t dim = (pressed == 2) ? LCD_LIGHTGREY : LCD_GREY;
        uint16_t cx  = 1 + 2 * MENU_PITCH;
        uint16_t c_r = (sub == SUB_MODE_RESISTANCE) ? LCD_CYAN : dim;
        uint16_t c_d = (sub == SUB_MODE_DIODE)      ? LCD_CYAN : dim;
        uint16_t c_c = (sub == SUB_MODE_CONTINUITY) ? LCD_CYAN : dim;
        LCD_DrawMenuCell(2, "", dim, bg);
        LCD_DrawString(cx + 8,  MENU_Y + 7, "\x7f", c_r, bg, 2);          // Ω
        LCD_DrawIcon  (cx + 26, MENU_Y + 7, ICON_DIODE, c_d, bg, 2);
        LCD_DrawIcon  (cx + 50, MENU_Y + 7, ICON_BEEP,  c_c, bg, 2);

        // F4: odpor -> RANGE, dióda -> ikona odporu (skratka na odpor), pípak -> ikona diódy
        uint16_t fg4 = (pressed == 3) ? LCD_BLACK  : LCD_YELLOW;
        uint16_t bg4 = (pressed == 3) ? LCD_YELLOW : LCD_BLACK;
        uint16_t x4  = 1 + 3 * MENU_PITCH;
        if (sub == SUB_MODE_DIODE) {
            LCD_DrawMenuCell(3, "", fg4, bg4);
            LCD_DrawIcon(x4 + (MENU_CELL_W - 24) / 2, MENU_Y + 7, ICON_RESISTOR, fg4, bg4, 2);
        } else if (sub == SUB_MODE_CONTINUITY) {
            LCD_DrawMenuCell(3, "", fg4, bg4);
            LCD_DrawIcon(x4 + (MENU_CELL_W - 16) / 2, MENU_Y + 7, ICON_DIODE, fg4, bg4, 2);
        } else {
            LCD_DrawMenuCell(3, "RANGE", fg4, bg4);
        }
    }
    else {
        LCD_DrawMenuCell(2, "---", LCD_GREY, LCD_BLACK);
        LCD_DrawMenuCell(3, "---", LCD_GREY, LCD_BLACK);
    }
}



//*************************************************************************************

// Vykreslenie znaku 45x72 v pomere 1:1
void LCD_Draw7SegDigit(uint16_t x, uint16_t y, char num, uint16_t color, uint16_t bg) {
    uint8_t font_idx = 11;
    if (num >= '0' && num <= '9') font_idx = num - '0';
    else if (num == '-') font_idx = 10;
    else if (num == ' ') font_idx = 11;
    else if (num == 'L') font_idx = 12;
    else if (num == 'O') font_idx = 13;

    for (uint8_t j = 0; j < FONT_7SEG_HEIGHT; j++) {
        for (uint8_t byte_idx = 0; byte_idx < FONT_7SEG_BYTES_PER_ROW; byte_idx++) {
            uint8_t byte_val = font_7seg[font_idx][j * FONT_7SEG_BYTES_PER_ROW + byte_idx];

            for (uint8_t bit = 0; bit < 8; bit++) {
                uint8_t x_pixel = byte_idx * 8 + bit;
                if (byte_val & (0x80 >> bit)) {
                    LCD_DrawPixel(x + x_pixel, y + j, color);
                } else if (bg != color) {
                    LCD_DrawPixel(x + x_pixel, y + j, bg);
                }
            }
        }
    }
}

// Desatinná bodka prispôsobená pre 72px výšku (rozmer 7x7 pixelov)
void LCD_Draw7SegDot(uint16_t x, uint16_t y, uint16_t color, uint16_t bg) {
    uint16_t dot_w = 7;
    uint16_t dot_h = 7;
    uint16_t dot_y = y + (FONT_7SEG_HEIGHT - dot_h); // Automaticky lícuje so spodkom 72px

    LCD_SetAddressWindow(x, dot_y, x + dot_w - 1, dot_y + dot_h - 1);
    uint8_t hi = color >> 8, lo = color & 0xFF;
    for (uint16_t i = 0; i < (dot_w * dot_h); i++) {
        LCD_WriteData(hi);
        LCD_WriteData(lo);
    }
}

