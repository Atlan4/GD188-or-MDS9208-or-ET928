/*
 * scope.h
 *
 *  Osciloskop: zachytávanie cez časovače a DMA, mriežka, rozsahy V/div, časová základňa,
 *  spúšťanie (trigger) AUTO / NORM / SINGLE. Spúšťa sa dlhým AUTO v polohe V alebo mV (main.c).
 *
 *  Hardvér:
 *    MS9280: dáta PC0–PC7, hodiny PC12 = TIM11_CH1 (AF3), STBY PA10 (1 = úsporný režim)
 *    TIM1 (rovnaký takt 100 MHz) generuje DMA požiadavky: DMA2 Stream5 kanál 6 (TIM1_UP)
 *    číta GPIOC->IDR do zásobníka v nízkej fáze hodín ADC.
 *    Vstup: relé AC/DC PA9 (1 = AC), relé 1/10 PA8, O1O2 PA15 (1:10 iba s relé 1/10 -> 1:100),
 *    U6 PC10 (x4), U7 PC11 (x2). Poloha mV je 10x citlivejšia (prepínač vynechá delič).
 *
 *  Ovládanie:
 *    hlavné menu:  F1 V/DIV, F2 TIME, F3 TRIG, F4 RUN/STOP
 *                  F1 dlho = menu osciloskopu, F2 dlho = menu meraní, F3 dlho = všetky merania áno / nie
 *    V/DIV:        F1 EXIT, F2 SENS-, F3 SENS+, F4 AC/DC; F2 / F3 držané = posun stopy dole / hore
 *    TIME:         F1 EXIT, F2 TIME-, F3 TIME+ (držané = vodorovný posun), F4 ROLL
 *    TRIG:         F1 EXIT, F2 LEVEL-, F3 LEVEL+ (držané = opakovanie), F4 hrana; AUTO krátko = režim
 *    AUTO krátko = autoset (mimo TRIG), AUTO dlho = odchod
 *    SAVE krátko = pamäte priebehov s náhľadmi 2 x 5 (F2 UP, F3 DOWN, F4 OPEN, SAVE = uložiť;
 *                  otvorený priebeh: oranžový rámik, "WAVE n", RUN = späť na živý obraz), SAVE dlho = posledné nastavenie do EEPROM
 *    TIME F4 = ROLL (od 100 ms/div; nad 200 ms/div vždy); menu osciloskopu (2 stránky): Trig position, H-pos,
 *                  Trig delay, Meas panel, Interp, Peak det / View, Calibration, Test mode
 */
/*  v0.76: Interp LINE / SIN(X)/X / DOTS, Peak det OFF / ON, View WAVE / FFT (scope.c)
 */

#ifndef INC_SCOPE_H_
#define INC_SCOPE_H_

#include <stdint.h>
#include <stdbool.h>
#include "button.h"

typedef enum {
    SCOPE_STAY = 0,
    SCOPE_EXIT,          // dlhé AUTO -> odchod na voltmeter
    SCOPE_TEST           // menu osciloskopu -> testovací režim
} ScopeAction_t;

// Stav na pokračovanie po zobudení (main.c, Resume_t)
typedef struct {
    uint8_t vdiv_v;      // index rozsahu V/div v polohe V
    uint8_t vdiv_mv;     // index rozsahu V/div v polohe mV
    uint8_t tb;          // index časovej základne
    uint8_t ac;          // 1 = AC väzba
    int8_t  offset;      // zvislá poloha stopy v krokoch 1/5 dielika
    uint8_t slope;       // 0 = nábežná hrana, 1 = dobežná
    uint8_t mode;        // 0 = AUTO, 1 = NORM, 2 = SINGLE
    uint8_t tpos;        // bod spustenia: 0 = 1/4, 1 = stred, 2 = 3/4 šírky
    int32_t trig_mv;     // úroveň spúšťania v mV (na vstupe)
    uint16_t meas_mask;  // vybrané merania (bit = MEAS_*), najviac 4
    uint8_t meas_show;   // 1 = riadok s vybranými meraniami zapnutý
    uint8_t meas_all;    // 1 = panel so všetkými meraniami (dlhé F3)
    uint8_t roll;        // 1 = ROLL (časové základne od 100 ms/div; nad 200 ms/div vždy)
    int16_t hpos;        // vodorovný posun v px (+ = neskôr)
    uint8_t hkeep;       // 1 = pri zmene TIME zachovať čas posunu (KEEP), 0 = vynulovať
    uint8_t delay_idx;   // oneskorenie záznamu po spustení (0 = OFF)
    uint8_t meas_panel;  // panel všetkých meraní: 0 SOLID, 1 TRANSP (obrys), 2 TRANSP PLAIN
} ScopeState_t;

// v0.76: zobrazenie (SETUP stránka 1/2 a 2/2). Mimo ScopeState_t, aby sa nezmenil formát uložených priebehov.
enum { INTERP_LINE = 0, INTERP_SINC, INTERP_DOTS };
enum { VIEW_WAVE = 0, VIEW_FFT };
typedef struct {
    uint8_t interp;      // INTERP_*: LINE, SIN(X)/X (predvolené), DOTS
    uint8_t peak;        // 1 = Peak det (od 50 us/div všetky vzorky 1 MHz, pásmo min..max)
    uint8_t view;        // VIEW_*: WAVE / FFT
    uint8_t rsv;
} ScopeExt_t;

void          Scope_Open(bool mv_pos);              // nakreslí obrazovku, spustí ADC a zachytávanie
void          Scope_Close(void);                    // zastaví časovače a DMA, relé do 0, ADC do STBY
ScopeAction_t Scope_Task(ButtonType_t ev, bool mv_pos);
const char   *Scope_TopText(void);                  // horný riadok ako text (jednofarebne)
void          Scope_DrawTitle(uint16_t y, bool force); // horný riadok farebne (medzi časovačom a batériou)
const char   *Scope_WideMsg(uint16_t *col);          // v0.66: aktívne hlásenie dlhšie ako TOP_TXT_MAX, inak NULL

// v0.66: horný riadok – voľné miesto medzi časomierou (x 6..63, vždy 5 znakov) a batériou (od x 279
// pri rámiku). Font 5x7 x2: znak 10 px, krok 12 px. 16 znakov = 190 px -> asi 12 px medzera na oboch stranách.
// Dlhšie časované hlásenia sa zobrazia cez celý riadok (časomiera a batéria sa na ten čas skryjú).
#define TOP_FREE_X0   64
#define TOP_FREE_X1   279
#define TOP_TXT_MAX   16
#define TOP_WIDE_MAX  25
void          Scope_GetState(ScopeState_t *st);
void          Scope_SetState(const ScopeState_t *st);
void          Scope_LoadEeprom(bool setup);
void          Scope_GetExt(ScopeExt_t *x);          // v0.76: Interp / Peak det / View (zobudenie: Resume_t)
void          Scope_SetExt(const ScopeExt_t *x);
// Kalibrácia deliča 1:10 / 1:100: main.c prepne čip na DC V a posiela napätie (základ 0.1 mV)
bool          Scope_MeterWanted(void);
void          Scope_SetMeterReading(bool ok, int32_t v01mv);        // pri štarte: kalibrácia; setup = aj posledné nastavenie (nie po zobudení)

#endif /* INC_SCOPE_H_ */
