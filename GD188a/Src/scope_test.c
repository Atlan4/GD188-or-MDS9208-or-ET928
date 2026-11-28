/*
 * scope_test.c
 *
 *  Testovací režim osciloskopu – viď scope_test.h. Spúšťa sa v polohe V alebo mV.
 *
 *  Obrazovka (rámik ako generátor, horný riadok kreslí main.c):
 *    y  36–163  priebeh 300 vzoriek (kód ADC 0–255 -> 128 px, 2 kódy na pixel)
 *    y 168      stav výstupov
 *    y 180      P-P a priemer (veľké písmo)
 *    y 199      MIN / MAX a nápoveda
 *    y 210      spodné menu F1–F4 (zelené = výstup v 1)
 *  Po krátkom AUTO sa namiesto priebehu ukáže tabuľka 16 kombinácií; ľubovoľné tlačidlo
 *  (F1–F4, SAVE) vykoná svoju funkciu a vráti zobrazenie priebehu.
 */

#include <stdio.h>
#include <string.h>

#include "scope_test.h"
#include "main.h"
#include "lcd.h"
#include "settings.h"

// --- Piny ---
#define PIN_REL_10   8u      // PA8  relé 1/10
#define PIN_REL_ACDC 9u      // PA9  relé AC/DC
#define PIN_STBY     10u     // PA10 MS9280 STBY (1 = úsporný režim)
#define PIN_O1O2     15u     // PA15 optočleny O1 + O2
#define PIN_U6       10u     // PC10
#define PIN_U7       11u     // PC11
#define PIN_ADCCLK   12u     // PC12 hodiny ADC

// --- Zachytávanie ---
#define N_SAMPLES     300u
#define SAMPLE_CYC    1000u          // 100 MHz / 1000 = 100 ksps
#define WARMUP        16u            // hodiny navyše na začiatku (pipeline MS9280 = 3 takty)
#define REFRESH_MS    100u
#define SETTLE_MS     200u           // relé, optočleny a U6/U7 pri automatickom prechode (60 ms nestačilo)

// --- Rozloženie ---
#define PX0          10
#define PY0          36
#define PH           128
#define PY_BOT       (PY0 + PH - 1)  // y kódu 0
#define STAT_Y       168
#define BIG_Y        180
#define INFO_Y       199

typedef struct { uint8_t min, max; uint16_t avg10; } Stat_t;   // avg10 = priemer x 10

static uint8_t  buf[N_SAMPLES];
static uint8_t  prev_top[N_SAMPLES], prev_bot[N_SAMPLES];
static bool     have_prev;
static uint32_t last_capture;
static bool     table_view;
static bool     auto_pending;
static uint32_t auto_tick;

// Stav výstupov (bity): 0 = O1O2, 1 = U6, 2 = U7, 3 = 1/10, 4 = AC/DC
#define B_O1O2   0x01u
#define B_U6     0x02u
#define B_U7     0x04u
#define B_10     0x08u
#define B_ACDC   0x10u
static uint8_t  outs;

static char     last_cells[4][8];
static char     last_stat[64], last_big[64], last_info[64];

// ---------------------------------------------------------------------------
// GPIO
// ---------------------------------------------------------------------------
static void PinOut(GPIO_TypeDef *p, uint32_t pin) {
    p->MODER   = (p->MODER   & ~(3u << (pin * 2))) | (1u << (pin * 2));   // výstup
    p->OTYPER &= ~(1u << pin);                                            // push-pull
    p->PUPDR  &= ~(3u << (pin * 2));                                      // bez pull-up (PA15 ho má po resete)
}

static void PinSet(GPIO_TypeDef *p, uint32_t pin, bool on) {
    p->BSRR = on ? (1u << pin) : (1u << (pin + 16));
}

static void ApplyOuts(void) {
    PinSet(GPIOA, PIN_O1O2,     outs & B_O1O2);
    PinSet(GPIOC, PIN_U6,       outs & B_U6);
    PinSet(GPIOC, PIN_U7,       outs & B_U7);
    PinSet(GPIOA, PIN_REL_10,   outs & B_10);
    PinSet(GPIOA, PIN_REL_ACDC, outs & B_ACDC);
}

void ScopeTest_InitPins(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
    outs = 0;
    PinSet(GPIOA, PIN_STBY, true);                   // ADC zatiaľ v úspornom režime
    PinSet(GPIOC, PIN_ADCCLK, false);
    ApplyOuts();
    PinOut(GPIOA, PIN_REL_10);
    PinOut(GPIOA, PIN_REL_ACDC);
    PinOut(GPIOA, PIN_STBY);
    PinOut(GPIOA, PIN_O1O2);
    PinOut(GPIOC, PIN_U6);
    PinOut(GPIOC, PIN_U7);
    PinOut(GPIOC, PIN_ADCCLK);
    GPIOC->OSPEEDR |= (3u << (PIN_ADCCLK * 2));      // hodiny ADC: najvyššia rýchlosť
    // dáta ADC PC0–PC7: vstupy bez pull-up / pull-down
    GPIOC->MODER &= ~0xFFFFu;
    GPIOC->PUPDR &= ~0xFFFFu;
}

// ---------------------------------------------------------------------------
// Zachytenie: hodiny programovo, čítanie dát na konci nízkej fázy (dáta sú ustálené).
// Prerušenia sú vypnuté ~3 ms; stratené prerušenia SysTick sa doplnia.
// ---------------------------------------------------------------------------
static void Capture(uint8_t *dst) {
    const uint32_t clk_set = 1u << PIN_ADCCLK, clk_clr = 1u << (PIN_ADCCLK + 16);
    __disable_irq();
    uint32_t t0 = DWT->CYCCNT;
    for (uint32_t i = 0; i < N_SAMPLES + WARMUP; i++) {
        uint32_t ts = t0 + i * SAMPLE_CYC;
        while ((int32_t)(DWT->CYCCNT - ts) < 0) { }
        GPIOC->BSRR = clk_set;                                   // nábežná hrana = vzorkovanie
        while ((int32_t)(DWT->CYCCNT - (ts + SAMPLE_CYC / 2)) < 0) { }
        GPIOC->BSRR = clk_clr;
        while ((int32_t)(DWT->CYCCNT - (ts + SAMPLE_CYC - SAMPLE_CYC / 8)) < 0) { }
        uint8_t v = (uint8_t)(GPIOC->IDR & 0xFFu);
        if (i >= WARMUP) dst[i - WARMUP] = v;
    }
    uint32_t ms = (DWT->CYCCNT - t0) / (SystemCoreClock / 1000u);
    if (ms > 1u) uwTick += ms - 1u;                              // jeden tik sa spracuje po povolení
    __enable_irq();
}

static Stat_t Stats(const uint8_t *b) {
    Stat_t s = { 255, 0, 0 };
    uint32_t sum = 0;
    for (uint32_t i = 0; i < N_SAMPLES; i++) {
        if (b[i] < s.min) s.min = b[i];
        if (b[i] > s.max) s.max = b[i];
        sum += b[i];
    }
    s.avg10 = (uint16_t)((sum * 10u + N_SAMPLES / 2u) / N_SAMPLES);
    return s;
}

// ---------------------------------------------------------------------------
// Kreslenie
// ---------------------------------------------------------------------------
static void DrawFrame(void) {
    LCD_DrawFastHLine(0, 0, 320, LCD_WHITE);
    LCD_DrawFastHLine(0, 239, 320, LCD_WHITE);
    LCD_DrawFastVLine(0, 0, 240, LCD_WHITE);
    LCD_DrawFastVLine(319, 0, 240, LCD_WHITE);
    LCD_DrawFastHLine(0, 27, 320, LCD_WHITE);
}

// Mriežka v jednom stĺpci medzi y0 a y1 (po zmazaní starej stopy)
static void GridColumn(uint16_t x, uint16_t y0, uint16_t y1) {
    bool vcol = ((x - PX0) % 25u) == 0u;
    for (uint16_t y = y0; y <= y1; y++) {
        bool hrow = (y == PY0 + PH / 4) || (y == PY0 + PH / 2) || (y == PY0 + 3 * PH / 4);
        if ((vcol && (y % 4u) == 0u) || (hrow && (x % 4u) == 0u))
            LCD_DrawPixel(x, y, LCD_GREY);
    }
}

static void DrawPlotArea(void) {
    LCD_FillRect(1, 32, 318, 178, LCD_BLACK);
    LCD_DrawFastHLine(PX0 - 1, PY0 - 1, N_SAMPLES + 2, LCD_LIGHTGREY);
    LCD_DrawFastHLine(PX0 - 1, PY0 + PH, N_SAMPLES + 2, LCD_LIGHTGREY);
    LCD_DrawFastVLine(PX0 - 1, PY0 - 1, PH + 2, LCD_LIGHTGREY);
    LCD_DrawFastVLine(PX0 + N_SAMPLES, PY0 - 1, PH + 2, LCD_LIGHTGREY);
    for (uint16_t x = PX0; x < PX0 + N_SAMPLES; x++) GridColumn(x, PY0, PY_BOT);
    have_prev = false;
    last_stat[0] = last_big[0] = last_info[0] = '\0';
}

static uint16_t CodeY(uint8_t c) { return (uint16_t)(PY_BOT - (c >> 1)); }

// Prekreslenie priebehu stĺpec po stĺpci (zmaže iba starú stopu – bez blikania)
static void DrawTrace(void) {
    for (uint32_t i = 0; i < N_SAMPLES; i++) {
        uint16_t x  = (uint16_t)(PX0 + i);
        uint16_t y  = CodeY(buf[i]);
        uint16_t yp = (i > 0) ? CodeY(buf[i - 1]) : y;          // spojnica s predchádzajúcim bodom
        uint16_t top = (y < yp) ? y : yp, bot = (y < yp) ? yp : y;
        if (have_prev) {
            if (prev_top[i] == top && prev_bot[i] == bot) continue;
            LCD_DrawFastVLine(x, prev_top[i], (uint16_t)(prev_bot[i] - prev_top[i] + 1u), LCD_BLACK);
            GridColumn(x, prev_top[i], prev_bot[i]);
        }
        LCD_DrawFastVLine(x, top, (uint16_t)(bot - top + 1u), LCD_YELLOW);
        prev_top[i] = (uint8_t)top;
        prev_bot[i] = (uint8_t)bot;
    }
    have_prev = true;
}

static void TextIfChanged(char *cache, size_t n, uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                          const char *txt, uint16_t color, uint8_t size) {
    if (strcmp(cache, txt) == 0) return;
    snprintf(cache, n, "%s", txt);
    LCD_FillRect(x, y, w, h, LCD_BLACK);
    LCD_DrawString(x, y, txt, color, LCD_BLACK, size);
}

static void DrawStats(const Stat_t *s) {
    char b[64];
    snprintf(b, sizeof(b), "AC/DC=%u  O1O2=%u  U6=%u  U7=%u  1/10=%u  STBY=0",
             (outs & B_ACDC) ? 1 : 0, (outs & B_O1O2) ? 1 : 0, (outs & B_U6) ? 1 : 0,
             (outs & B_U7) ? 1 : 0, (outs & B_10) ? 1 : 0);
    TextIfChanged(last_stat, sizeof(last_stat), 10, STAT_Y, 300, 8, b, LCD_LIGHTGREY, 1);

    snprintf(b, sizeof(b), "P-P %3u  AVG %3u.%u", (unsigned)(s->max - s->min), s->avg10 / 10u, s->avg10 % 10u);
    TextIfChanged(last_big, sizeof(last_big), 10, BIG_Y, 300, 16, b, LCD_WHITE, 2);

    snprintf(b, sizeof(b), "MIN %3u  MAX %3u   AUTO=sweep  hold AUTO=exit", s->min, s->max);
    TextIfChanged(last_info, sizeof(last_info), 10, INFO_Y, 300, 8, b, LCD_LIGHTGREY, 1);
}

static void DrawCell(uint8_t idx, const char *txt, bool on) {
    char key[8];
    snprintf(key, sizeof(key), "%.5s%c", txt, on ? '1' : '0');
    if (strcmp(key, last_cells[idx]) == 0) return;
    strcpy(last_cells[idx], key);
    if (on) LCD_DrawMenuCell(idx, txt, LCD_SEL_FG, LCD_SEL_BG);
    else    LCD_DrawMenuCell(idx, txt, LCD_WHITE,  LCD_BLACK);
}

static void DrawCells(void) {
    DrawCell(0, "AC/DC", outs & B_ACDC);
    DrawCell(1, "O1O2",  outs & B_O1O2);
    DrawCell(2, "U6",    outs & B_U6);
    DrawCell(3, "U7",    outs & B_U7);
}

// ---------------------------------------------------------------------------
// Automatický prechod 16 kombinácií (1/10, U7, U6, O1O2); AC/DC ostáva ako je nastavené
// ---------------------------------------------------------------------------
static void Sweep(void) {
    table_view = true;
    LCD_FillRect(1, 32, 318, 178, LCD_BLACK);
    LCD_DrawString(100, 110, "SWEEPING...", LCD_YELLOW, LCD_BLACK, 2);

    uint8_t saved = outs;
    Stat_t  st[16];
    for (uint8_t k = 0; k < 16; k++) {
        outs = (uint8_t)((saved & B_ACDC) | (k & 0x0Fu));     // bity k: 0 O1O2, 1 U6, 2 U7, 3 1/10
        ApplyOuts();
        HAL_Delay(SETTLE_MS);
        Capture(buf);
        st[k] = Stats(buf);
    }
    outs = saved;
    ApplyOuts();

    LCD_FillRect(1, 32, 318, 178, LCD_BLACK);
    char b[64];
    snprintf(b, sizeof(b), " #   1/10 U7 U6 O1O2   AVG    P-P  MIN  MAX AC/DC=%u",
             (saved & B_ACDC) ? 1 : 0);
    LCD_DrawString(6, 36, b, LCD_CYAN, LCD_BLACK, 1);
    for (uint8_t k = 0; k < 16; k++) {
        snprintf(b, sizeof(b), "%2u    %u   %u  %u   %u    %3u.%u   %3u  %3u  %3u", k,
                 (k >> 3) & 1u, (k >> 2) & 1u, (k >> 1) & 1u, k & 1u,
                 st[k].avg10 / 10u, st[k].avg10 % 10u, (unsigned)(st[k].max - st[k].min),
                 st[k].min, st[k].max);
        LCD_DrawString(6, (uint16_t)(47 + k * 10), b, (k & 1u) ? LCD_WHITE : LCD_LIGHTGREY, LCD_BLACK, 1);
    }
}

// ---------------------------------------------------------------------------
void ScopeTest_Open(void) {
    // Počítadlo taktov jadra na časovanie hodín ADC
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    PinSet(GPIOA, PIN_STBY, false);                  // ADC beží
    ApplyOuts();
    table_view = false;
    auto_pending = false;
    memset(last_cells, 0, sizeof(last_cells));

    LCD_Clear(LCD_BLACK);
    DrawFrame();
    LCD_DrawMenuFrame();
    DrawPlotArea();
    DrawCells();
    last_capture = HAL_GetTick() - REFRESH_MS;
}

void ScopeTest_Close(bool keep_state) {
    uint8_t keep = outs;
    outs = 0;                                        // relé a prepínače do 0 (bez prúdu)
    ApplyOuts();
    if (keep_state) outs = keep;                     // návrat z generátora: znova sa nastavia v Open
    PinSet(GPIOC, PIN_ADCCLK, false);
    PinSet(GPIOA, PIN_STBY, true);
    auto_pending = false;
}

bool ScopeTest_Task(ButtonType_t ev) {
    uint32_t now = HAL_GetTick();

    // AUTO: krátke = prechod kombinácií, dlhé = odchod
    if (ev == BTN_AUTO) {
        auto_pending = true;
        auto_tick = now;
        ev = BTN_NONE;
    }
    if (auto_pending) {
        if (BUTTON_GetHeld() == BTN_AUTO) {
            if (now - auto_tick >= Settings_LongPressMs()) {
                auto_pending = false;
                return false;
            }
        } else {
            auto_pending = false;
            Sweep();
            DrawCells();
            return true;
        }
    }

    uint8_t toggle = 0;
    switch (ev) {
        case BTN_F1:   toggle = B_ACDC; break;
        case BTN_F2:   toggle = B_O1O2; break;
        case BTN_F3:   toggle = B_U6;   break;
        case BTN_F4:   toggle = B_U7;   break;
        case BTN_SAVE: toggle = B_10;   break;
        default: break;
    }
    if (toggle) {
        outs ^= toggle;
        ApplyOuts();
        if (table_view) {                              // z tabuľky späť na priebeh
            table_view = false;
            DrawPlotArea();
        }
        last_stat[0] = '\0';
    }
    DrawCells();

    if (!table_view && now - last_capture >= REFRESH_MS) {
        last_capture = now;
        Capture(buf);
        Stat_t s = Stats(buf);
        DrawTrace();
        DrawStats(&s);
    }
    return true;
}
