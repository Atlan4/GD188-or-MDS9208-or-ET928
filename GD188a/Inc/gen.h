/*
 * gen.h
 *
 *  Generátor signálu: DAC (PA5, DAC_OUT1) + DMA1 Stream1 kanál 7 (TIM6_UP) + TIM6.
 *  Výstup ide cez kondenzátor na OZ a svorku OUT gen (iba v polohe prepínača V).
 */

#ifndef INC_GEN_H_
#define INC_GEN_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Výstupné napätie (Vpp na svorke), ktoré zodpovedá plnému rozkmitu DAC 0..4095.
// Závisí od zosilnenia OZ – po odmeraní upraviť (návod v gen.c).
#define GEN_FULLSCALE_MVPP   3000u           // v0.77: VDDA = 3.0 V (odmerané 2.996 V), predtým 3300

typedef enum {
    GEN_WAVE_SINE = 0,
    GEN_WAVE_SQUARE,
    GEN_WAVE_RAMP,
    GEN_WAVE_COUNT
} GenWave_t;

#define GEN_NUM_FREQ   10      // 100 Hz .. 100 kHz (100 kHz iba sínus)
#define GEN_NUM_DUTY    9      // 10 .. 90 %
#define GEN_NUM_VOLT   11      // 0 .. 3.0 Vpp

typedef struct {
    uint8_t wave;              // GenWave_t
    uint8_t freq_idx;
    uint8_t duty_idx;
    uint8_t volt_idx;
    bool    on;                // výstup zapnutý
} GenParams_t;

extern GenParams_t g_gen;

// Zoznamy hodnôt
uint8_t     Gen_FreqCount(uint8_t wave);
uint32_t    Gen_FreqHz(uint8_t idx);
uint8_t     Gen_DutyPct(uint8_t idx);
uint16_t    Gen_VoltMv(uint8_t idx);
const char *Gen_WaveName(uint8_t wave);
bool        Gen_DutyUsed(uint8_t wave);          // pri sínuse sa strieda nepoužíva
void        Gen_FormatFreq(uint32_t hz, char *buf, size_t n);
void        Gen_FormatVolt(uint16_t mv, char *buf, size_t n);

// Tvar vlny: vzorka i z n vzoriek jednej periódy, výsledok -32767..+32767
int32_t     Gen_Shape(uint8_t wave, uint8_t duty_pct, uint32_t i, uint32_t n);

// Hardvér
void Gen_Update(void);                 // použije aktuálne g_gen (prepočíta tabuľku, reštart DMA)
void Gen_SetOutput(bool on);           // zapnúť / vypnúť výstup
bool Gen_IsOn(void);
void Gen_Stop(void);                   // úplné vypnutie DAC, DMA a TIM6 (pred spánkom)
void Gen_SetCal(int16_t permille);     // korekcia amplitúdy v 0.1 % (+20 = výstup o 2 % väčší)
int16_t Gen_GetCal(void);

#endif /* INC_GEN_H_ */
