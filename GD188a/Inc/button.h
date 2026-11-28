/*
 * button.h
 *
 *  Created on: Nov 10, 2026
 *      Author: 42190
 */

// button.h
// Matrix and standalone button handling using CubeMX User Labels

// button.h
#ifndef BUTTON_H_
#define BUTTON_H_

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "gpio.h"
#include "meas_chip.h" // Potrebné pre prístup k SwitchPosition_t a SubMode_t
#include "rotary_switch.h"

typedef enum {
    BTN_NONE = 0,
    BTN_F1,
    BTN_F2,
    BTN_F3,
    BTN_F4,
    BTN_AUTO,
    BTN_BL,
    BTN_SAVE
} ButtonType_t;

// Exportované funkcie
void BUTTON_Init(void);

// Vráti poslednú stlačenú klávesu (alebo BTN_NONE) a vynuluje ju. Volať z hlavnej slučky.
ButtonType_t BUTTON_GetEvent(void);

// Zistenie pustenia tlačidla – volať v každom prechode hlavnej slučky
void BUTTON_Poll(void);

// Práve držané tlačidlo (BTN_NONE = nič) – napr. pre zvýraznenie v menu
ButtonType_t BUTTON_GetHeld(void);

#endif // BUTTON_H_




