/*
 * beep.c
 *
 *  Pípanie na pozadí: Beep_Start() zapne bzučiak a zapamätá čas vypnutia, Beep_Task()
 *  v hlavnej slučke ho vypne. Prístroj sa počas pípania nezastaví (displej, tlačidlá bežia).
 */

#include "beep.h"
#include "main.h"
#include "settings.h"

#define BEEP_ERROR_MS    400u
#define BEEP_WARNING_MS  60u

static uint32_t beep_until = 0;
static uint8_t  beep_on = 0;

void Beep_Start(uint32_t ms) {
    uint32_t until = HAL_GetTick() + ms;
    if (!beep_on || (int32_t)(until - beep_until) > 0) beep_until = until;
    beep_on = 1;
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
}

void Beep_Task(void) {
    if (beep_on && (int32_t)(HAL_GetTick() - beep_until) >= 0) {
        beep_on = 0;
        HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
    }
}

void Beep_Error(void) {
    if (g_settings.beep) Beep_Start(BEEP_ERROR_MS);
}

void Beep_Warning(void) {
    if (g_settings.beep) Beep_Start(BEEP_WARNING_MS);
}
