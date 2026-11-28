/*
 * battery.c
 *
 *  ADC1 cez registre (HAL ADC v projekte nie je potrebný).
 *    PA7  = ADC1_IN7  – napätie batérie cez delič 10k/10k
 *    IN17 = VREFINT   – interná referencia ~1.21 V na výpočet skutočného VDDA
 */

#include <stdbool.h>

#include "battery.h"
#include "stm32f4xx_hal.h"

#define BAT_CHANNEL        7u
#define VREFINT_CHANNEL    17u
#define BAT_DIVIDER_NUM    2u          // (10k + 10k) / 10k
#define BAT_PERIOD_MS      2000u
#define BAT_SAMPLES        16u

// Kalibrácia VREFINT z výroby (meraná pri VDDA = 3.3 V, 30 °C), viď datasheet STM32F410
#define VREFINT_CAL_ADDR   ((const uint16_t *)0x1FFF7A2AUL)
#define VREFINT_CAL_VDDA   3300u
#define VREFINT_TYP_MV     1210u       // typická hodnota, ak kalibrácia nie je platná

static uint16_t bat_mv = 0;
static uint8_t  bat_pct = 100;
static uint32_t last_tick = 0;
static bool     first = true;
static uint32_t avg_mv16 = 0;      // dlhý priemer (~30 s) v 1/16 mV – odhad času chodu

// Li-ion: napätie -> percentá (vybíjacia krivka je nelineárna)
static const struct { uint16_t mv; uint8_t pct; } curve[] = {
    {4200, 100}, {4100, 90}, {4000, 80}, {3900, 65}, {3800, 50},
    {3700, 30},  {3600, 15}, {3500, 7},  {3400, 3},  {3300, 0},
};

static uint16_t ReadChannel(uint32_t ch) {
    ADC1->SQR3 = ch;                               // jediná konverzia, kanál ch
    uint32_t sum = 0;
    for (uint32_t i = 0; i < BAT_SAMPLES; i++) {
        ADC1->SR = 0;
        ADC1->CR2 |= ADC_CR2_SWSTART;
        uint32_t t0 = HAL_GetTick();
        while ((ADC1->SR & ADC_SR_EOC) == 0) {
            if (HAL_GetTick() - t0 > 5u) return 0;
        }
        sum += ADC1->DR & 0x0FFFu;
    }
    return (uint16_t)(sum / BAT_SAMPLES);
}

static uint8_t MvToPercent(uint16_t mv) {
    if (mv >= curve[0].mv) return 100;
    for (unsigned i = 1; i < sizeof(curve) / sizeof(curve[0]); i++) {
        if (mv >= curve[i].mv) {
            uint32_t span_mv  = curve[i - 1].mv - curve[i].mv;
            uint32_t span_pct = curve[i - 1].pct - curve[i].pct;
            return (uint8_t)(curve[i].pct + (uint32_t)(mv - curve[i].mv) * span_pct / span_mv);
        }
    }
    return 0;
}

void Battery_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();

    GPIO_InitTypeDef gi = {0};
    gi.Pin  = GPIO_PIN_7;
    gi.Mode = GPIO_MODE_ANALOG;
    gi.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gi);

    // ADC hodiny = PCLK2 / 4 = 25 MHz (max. 36 MHz), zapnúť VREFINT
    ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | (1u << ADC_CCR_ADCPRE_Pos) | ADC_CCR_TSVREFE;

    ADC1->CR1 = 0;                                 // 12 bit, bez skenovania
    ADC1->CR2 = 0;                                 // zarovnanie vpravo, jednorazová konverzia
    // Najdlhší čas vzorkovania (480 cyklov = 19 us): delič 5 kOhm aj VREFINT (min. 10 us)
    ADC1->SMPR2 = (ADC1->SMPR2 & ~ADC_SMPR2_SMP7)  | (7u << ADC_SMPR2_SMP7_Pos);
    ADC1->SMPR1 = (ADC1->SMPR1 & ~ADC_SMPR1_SMP17) | (7u << ADC_SMPR1_SMP17_Pos);
    ADC1->SQR1 = 0;                                // 1 konverzia v sekvencii
    ADC1->CR2 |= ADC_CR2_ADON;

    for (volatile uint32_t i = 0; i < 2000u; i++) { }   // stabilizácia ADC a VREFINT (~10 us)

    first = true;
    Battery_Task();
}

void Battery_Task(void) {
    uint32_t now = HAL_GetTick();
    if (!first && (now - last_tick) < BAT_PERIOD_MS) return;
    last_tick = now;

    uint16_t raw_ref = ReadChannel(VREFINT_CHANNEL);
    uint16_t raw_bat = ReadChannel(BAT_CHANNEL);
    if (raw_ref == 0) return;

    // VDDA z internej referencie; kalibráciu z výroby použiť len ak je rozumná
    uint32_t cal = *VREFINT_CAL_ADDR;
    uint32_t vdda_mv;
    if (cal > 1300u && cal < 1700u) {
        vdda_mv = VREFINT_CAL_VDDA * cal / raw_ref;
    } else {
        vdda_mv = VREFINT_TYP_MV * 4095u / raw_ref;
    }

    uint32_t mv = (uint32_t)raw_bat * vdda_mv / 4095u * BAT_DIVIDER_NUM;

    // Vyhladenie (priemer s predchádzajúcou hodnotou), prvé meranie priamo
    bat_mv = first ? (uint16_t)mv : (uint16_t)((bat_mv * 3u + mv) / 4u);
    // Dlhý priemer pre odhad času chodu: každé 2 s váha 1/15 -> časová konštanta ~30 s
    if (first) avg_mv16 = mv * 16u;
    else       avg_mv16 = (avg_mv16 * 14u + mv * 16u) / 15u;
    first  = false;
    bat_pct = MvToPercent(bat_mv);
}

uint16_t Battery_GetMillivolts(void) { return bat_mv; }
uint8_t  Battery_GetPercent(void)    { return bat_pct; }
uint8_t  Battery_GetAvgPercent(void) { return MvToPercent((uint16_t)(avg_mv16 / 16u)); }
uint16_t Battery_GetAvgMillivolts(void) { return (uint16_t)(avg_mv16 / 16u); }
