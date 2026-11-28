/*
 * rotary_switch.h
 *
 *  Created on: Nov 8, 2026
 *      Author: 42190
 */

// rotary_switch.h
// Logic for rotary range switch and SELECT button with integrated debounce

// rotary_switch.h
// Logic for rotary range switch and SELECT button with integrated debounce

#ifndef ROTARY_SWITCH_H_
#define ROTARY_SWITCH_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

// --- 1. CONFIGURATION OF SELECT BUTTON ---
//#define SELECT_BTN_PORT      GPIOA
//#define SELECT_BTN_PIN       GPIO_PIN_0

// --- 2. REAL HEX COMMANDS FOR YOUR MEASUREMENT CHIP ---
#define CMD_CHIP_6V_DC      0x05  // DC V
#define CMD_CHIP_6V_AC      0x06  // AC V
#define CMD_CHIP_600mV_DC   0x01  // DC mV
#define CMD_CHIP_600mV_AC   0x02  // AC mV
#define CMD_CHIP_HZ         0x12  // Hz (Frekvencia)
#define CMD_CHIP_PCT        0x12  // Duty % (Strieda - rovnaký základný rozsah čipu)
#define CMD_CHIP_uA_DC      0x0C  // DC uA
#define CMD_CHIP_uA_AC      0x0D  // AC uA
#define CMD_CHIP_mA_DC      0x0E  // DC mA
#define CMD_CHIP_mA_AC      0x0F  // AC mA
#define CMD_CHIP_10A_DC     0x10  // DC 10A
#define CMD_CHIP_10A_AC     0x11  // AC 10A
#define CMD_CHIP_CAP        0x0B  // Cx nF
#define CMD_CHIP_RES        0x07  // Resistance
#define CMD_CHIP_DIODE      0x0A  // Diode
#define CMD_CHIP_CONT       0x09  // Ohm continuity
#define CMD_CHIP_AUTO       0x5A  // Automatický rozsah
#define CMD_CHIP_MANUAL     0x5B  // Manuálny rozsah (každé 5B = ďalší rozsah)
#define CMD_CHIP_UNKNOWN    0x00  // Prechodový stav


// --- 3. ROTARY SWITCH POSITIONS (Exact Front Panel Order) ---
typedef enum {
    SWITCH_POS_V = 0,
    SWITCH_POS_mV,
    SWITCH_POS_OHM_DIODE_CONT, // Kombinovaná poloha
    SWITCH_POS_Cx,
    SWITCH_POS_Hz_Pct,
    SWITCH_POS_uA,
    SWITCH_POS_mA,
    SWITCH_POS_10A
} SwitchPosition_t;

// --- 4. SUB-MODES FOR SELECT BUTTON ---
typedef enum {
    SUB_MODE_DC = 0,
    SUB_MODE_AC,
    SUB_MODE_FREQ,
    SUB_MODE_DUTY,
    SUB_MODE_RESISTANCE,
    SUB_MODE_DIODE,
    SUB_MODE_CONTINUITY,
    SUB_MODE_CAPACITANCE
} SubMode_t;

// --- 5. EXPORTED FUNCTIONS ---
SwitchPosition_t ROTARY_GetSwitchPosition(void);
SubMode_t ROTARY_GetNextSubMode(SwitchPosition_t current_pos, SubMode_t current_sub);
uint8_t ROTARY_GetChipCommand(SwitchPosition_t pos, SubMode_t sub);
uint8_t ROTARY_GetQuery(SwitchPosition_t pos, SubMode_t sub);
SubMode_t ROTARY_GetDefaultSubMode(SwitchPosition_t pos);

// Odrušená poloha prepínača: vráti true, ak sa ustálila NOVÁ poloha (zapíše ju do *pos)
bool ROTARY_PollStable(SwitchPosition_t *pos);

#endif // ROTARY_SWITCH_H_
