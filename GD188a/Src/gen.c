/*
 * gen.c
 *
 *  Generátor signálu.
 *
 *  Hardvér (STM32F410):
 *    - DAC kanál 1 na PA5 (DAC_OUT1), výstupný buffer zapnutý.
 *    - DAC na F410 vie byť spúšťaný iba z TIM5 (ten riadi podsvietenie), preto sa
 *      DAC nespúšťa vlastným triggerom: TIM6 generuje pri každom pretečení DMA požiadavku
 *      (TIM6_UP = DMA1 Stream1, kanál 7) a DMA zapíše ďalšiu vzorku priamo do DAC_DHR12R1.
 *      Bez triggera DAC prevezme novú hodnotu jeden takt APB1 po zápise.
 *    - TIM6 beží zo 100 MHz (APB1 = 50 MHz, časovače x2).
 *
 *  Vzorkovanie: n = min(500, 1 MHz / f) vzoriek na periódu, krok TIM6 = 100 MHz / (f * n).
 *  Pre všetky ponúkané frekvencie vychádza presne (100 Hz .. 100 kHz), max. 1 Msps.
 *
 *  Výstup je AC väzbou (kondenzátor) -> stred signálu je v polovici DAC (2048).
 *  Pri vypnutom výstupe DAC drží stred, takže pri zapnutí nevznikne skok jednosmerného napätia.
 *
 *  Kalibrácia amplitúdy (GEN_FULLSCALE_MVPP v gen.h):
 *    nastaviť SINE, 1.00 kHz, 1.0 Vpp, výstup ON a odmerať iným multimetrom AC napätie
 *    na svorke OUT gen. Správne je 0.354 V (1.0 Vpp / 2.83).
 *    Nová hodnota = GEN_FULLSCALE_MVPP * nameraná / 0.354.
 *    v0.77: GEN_FULLSCALE_MVPP = 3000 (VDDA = 3.0 V, DAC 0..4095 = 0..3.0 V). Jemné doladenie
 *    robí CAL v nastaveniach generátora (SINE 100 Hz 2.0 Vpp, cieľ 0.707 V AC).
 */

#include <stdio.h>
#include "gen.h"
#include "stm32f4xx.h"

GenParams_t g_gen = {
    .wave     = GEN_WAVE_SINE,
    .freq_idx = 3,             // 1 kHz
    .duty_idx = 4,             // 50 %
    .volt_idx = 10,            // 3.0 Vpp
    .on       = false,
};

static const uint32_t FREQS_HZ[GEN_NUM_FREQ] = { 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000 };
static const uint16_t VOLTS_MV[GEN_NUM_VOLT] = { 0, 100, 200, 300, 400, 500, 1000, 1500, 2000, 2500, 3000 };
static const char *const WAVE_NAMES[GEN_WAVE_COUNT] = { "SINE", "SQUARE", "RAMP" };
static int16_t gen_cal = 0;            // korekcia amplitúdy v 0.1 % (nastavenia generátora, CAL)

#define GEN_MAX_SAMPLES   500u
#define GEN_MAX_RATE      1000000u      // max. 1 Msps (DAC)
#define GEN_TIM_CLK       100000000u    // TIM6: APB1 50 MHz x2
#define GEN_DAC_MID       2048

// DAC register je 32-bitový -> DMA prenáša slová
static uint32_t gen_table[GEN_MAX_SAMPLES];
static bool     hw_ready = false;

// Štvrtina sínusu, 128 krokov (+1), Q15
static const int16_t SINE_Q[129] = { 0, 402, 804, 1206, 1608, 2009, 2410, 2811, 3212, 3612, 4011, 4410, 4808, 5205, 5602, 5998, 6393, 6786, 7179, 7571, 7962, 8351, 8739, 9126, 9512, 9896, 10278, 10659, 11039, 11417, 11793, 12167, 12539, 12910, 13279, 13645, 14010, 14372, 14732, 15090, 15446, 15800, 16151, 16499, 16846, 17189, 17530, 17869, 18204, 18537, 18868, 19195, 19519, 19841, 20159, 20475, 20787, 21096, 21403, 21705, 22005, 22301, 22594, 22884, 23170, 23452, 23731, 24007, 24279, 24547, 24811, 25072, 25329, 25582, 25832, 26077, 26319, 26556, 26790, 27019, 27245, 27466, 27683, 27896, 28105, 28310, 28510, 28706, 28898, 29085, 29268, 29447, 29621, 29791, 29956, 30117, 30273, 30424, 30571, 30714, 30852, 30985, 31113, 31237, 31356, 31470, 31580, 31685, 31785, 31880, 31971, 32057, 32137, 32213, 32285, 32351, 32412, 32469, 32521, 32567, 32609, 32646, 32678, 32705, 32728, 32745, 32757, 32765, 32767 };

// ---------------------------------------------------------------------------
// Zoznamy hodnôt
// ---------------------------------------------------------------------------
uint8_t Gen_FreqCount(uint8_t wave) {
    return (wave == GEN_WAVE_SINE) ? GEN_NUM_FREQ : (GEN_NUM_FREQ - 1);   // 100 kHz iba sínus
}
uint32_t Gen_FreqHz(uint8_t idx)     { return FREQS_HZ[(idx < GEN_NUM_FREQ) ? idx : 0]; }
uint8_t  Gen_DutyPct(uint8_t idx)    { return (uint8_t)(10u * ((idx < GEN_NUM_DUTY) ? idx + 1u : 5u)); }
uint16_t Gen_VoltMv(uint8_t idx)     { return VOLTS_MV[(idx < GEN_NUM_VOLT) ? idx : 0]; }
const char *Gen_WaveName(uint8_t w)  { return WAVE_NAMES[(w < GEN_WAVE_COUNT) ? w : 0]; }
bool Gen_DutyUsed(uint8_t wave)      { return wave != GEN_WAVE_SINE; }

void Gen_FormatFreq(uint32_t hz, char *buf, size_t n) {
    if (hz < 1000u)        snprintf(buf, n, "%lu Hz", (unsigned long)hz);
    else if (hz < 10000u)  snprintf(buf, n, "%lu.%02lu kHz", (unsigned long)(hz / 1000u), (unsigned long)((hz % 1000u) / 10u));
    else if (hz < 100000u) snprintf(buf, n, "%lu.%lu kHz",   (unsigned long)(hz / 1000u), (unsigned long)((hz % 1000u) / 100u));
    else                   snprintf(buf, n, "%lu kHz", (unsigned long)(hz / 1000u));
}

void Gen_FormatVolt(uint16_t mv, char *buf, size_t n) {
    snprintf(buf, n, "%u.%u Vpp", mv / 1000u, (mv % 1000u) / 100u);
}

// ---------------------------------------------------------------------------
// Tvar vlny
// ---------------------------------------------------------------------------
static int32_t SineAt(uint32_t pos) {          // pos: 0..512*256 na periódu (8 bitov zlomok)
    uint32_t step = (pos >> 8) & 511u;
    uint32_t frac = pos & 255u;
    uint32_t q    = step >> 7;                 // štvrtina 0..3
    uint32_t k    = step & 127u;
    int32_t a, b;
    if (q & 1u) { a = SINE_Q[128 - k]; b = SINE_Q[127 - k]; }   // klesajúca štvrtina
    else        { a = SINE_Q[k];       b = SINE_Q[k + 1]; }
    int32_t v = a + (((b - a) * (int32_t)frac) >> 8);
    return (q >= 2u) ? -v : v;
}

int32_t Gen_Shape(uint8_t wave, uint8_t duty_pct, uint32_t i, uint32_t n) {
    if (n == 0) return 0;
    i %= n;
    switch (wave) {
        case GEN_WAVE_SQUARE: {
            uint32_t hi = (n * duty_pct + 50u) / 100u;
            return (i < hi) ? 32767 : -32767;
        }
        case GEN_WAVE_RAMP: {
            // strieda = podiel nábehu: 50 % = trojuholník, 10 / 90 % = pílovitý tvar
            uint32_t k = (n * duty_pct + 50u) / 100u;
            if (k < 1u) k = 1u;
            if (k >= n) k = n - 1u;
            if (i < k) return -32767 + (int32_t)((65534u * i) / k);
            return 32767 - (int32_t)((65534u * (i - k)) / (n - k));
        }
        default: {
            uint32_t pos = (uint32_t)(((uint64_t)i * (512u * 256u)) / n);
            return SineAt(pos);
        }
    }
}

// ---------------------------------------------------------------------------
// Hardvér
// ---------------------------------------------------------------------------
static void HwInit(void) {
    if (hw_ready) return;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_DACEN | RCC_APB1ENR_TIM6EN;
    (void)RCC->APB1ENR;

    // PA5 analógový režim, bez pull-up/down
    GPIOA->MODER |= (3u << (5u * 2u));
    GPIOA->PUPDR &= ~(3u << (5u * 2u));

    // DAC kanál 1: buffer zapnutý, bez triggera (zápis do DHR sa prejaví hneď)
    DAC1->CR = 0;
    DAC1->DHR12R1 = GEN_DAC_MID;
    DAC1->CR = DAC_CR_EN1;
    hw_ready = true;
}

static void StopStream(void) {
    TIM6->CR1  &= ~TIM_CR1_CEN;
    TIM6->DIER  = 0;
    DMA1_Stream1->CR &= ~DMA_SxCR_EN;
    for (uint32_t t = 0; (DMA1_Stream1->CR & DMA_SxCR_EN) && t < 100000u; t++) { }
    DMA1->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 |
                  DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;
}

void Gen_Update(void) {
    // Obdĺžnik a rampa iba do 50 kHz
    if (g_gen.freq_idx >= Gen_FreqCount(g_gen.wave)) g_gen.freq_idx = Gen_FreqCount(g_gen.wave) - 1u;

    if (!g_gen.on) {
        if (hw_ready) {
            StopStream();
            DAC1->DHR12R1 = GEN_DAC_MID;      // stred (po kondenzátore 0 V)
        }
        return;
    }

    HwInit();
    StopStream();

    uint32_t f = Gen_FreqHz(g_gen.freq_idx);
    uint32_t n = GEN_MAX_RATE / f;
    if (n > GEN_MAX_SAMPLES) n = GEN_MAX_SAMPLES;
    if (n < 2u) n = 2u;
    uint32_t div = GEN_TIM_CLK / (f * n);      // takty TIM6 na jednu vzorku
    if (div < 2u) div = 2u;
    if (div > 65536u) div = 65536u;

    // Amplitúda v digitoch DAC (špička-špička)
    uint32_t pp = ((uint32_t)Gen_VoltMv(g_gen.volt_idx) * 4095u + GEN_FULLSCALE_MVPP / 2u) / GEN_FULLSCALE_MVPP;
    pp = (pp * (uint32_t)(1000 + gen_cal) + 500u) / 1000u;    // kalibrácia amplitúdy
    if (pp > 4095u) pp = 4095u;
    uint8_t duty = Gen_DutyPct(g_gen.duty_idx);

    for (uint32_t i = 0; i < n; i++) {
        int32_t s = Gen_Shape(g_gen.wave, duty, i, n);
        int32_t v = GEN_DAC_MID + (int32_t)(((int64_t)s * (int32_t)pp) / 65534);
        if (v < 0) v = 0;
        if (v > 4095) v = 4095;
        gen_table[i] = (uint32_t)v;
    }

    // DMA1 Stream1, kanál 7 (TIM6_UP): pamäť -> DAC_DHR12R1, 32 bit, kruhový režim
    DMA_Stream_TypeDef *s = DMA1_Stream1;
    s->PAR  = (uint32_t)&DAC1->DHR12R1;
    s->M0AR = (uint32_t)gen_table;
    s->NDTR = n;
    s->FCR  = 0;                               // priamy režim
    s->CR   = (7u << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL_1 |
              DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 |
              DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_DIR_0;
    s->CR  |= DMA_SxCR_EN;

    // TIM6: perióda = jedna vzorka
    TIM6->CR1 = TIM_CR1_ARPE;
    TIM6->PSC = 0;
    TIM6->ARR = div - 1u;
    TIM6->CNT = 0;
    TIM6->EGR = TIM_EGR_UG;                    // načítať PSC/ARR (ešte bez DMA)
    TIM6->SR  = 0;
    TIM6->DIER = TIM_DIER_UDE;
    TIM6->CR1 |= TIM_CR1_CEN;
}

void Gen_SetOutput(bool on) {
    g_gen.on = on;
    Gen_Update();
}

void Gen_SetCal(int16_t permille) {
    gen_cal = permille;
    if (g_gen.on) Gen_Update();
}

int16_t Gen_GetCal(void) {
    return gen_cal;
}

bool Gen_IsOn(void) {
    return g_gen.on;
}

void Gen_Stop(void) {
    g_gen.on = false;
    if (!hw_ready) return;
    StopStream();
    DAC1->CR = 0;
    RCC->APB1ENR &= ~(RCC_APB1ENR_DACEN | RCC_APB1ENR_TIM6EN);
    hw_ready = false;
}
