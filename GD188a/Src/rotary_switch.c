/*
 * rotary_switch.c
 *
 *  Created on: Nov 8, 2026
 *      Author: 42190
 */

#include <string.h>
#include <stdio.h>

// rotary_switch.c
#include "rotary_switch.h"
#include "meas_chip.h"

// Prečíta piny PC13, PC14, PC15 a vráti reálnu logickú polohu prepínača
// presne podľa hardvérového zapojenia na plošnom spoji.
SwitchPosition_t ROTARY_GetSwitchPosition(void) {
    uint32_t port_c = GPIOC->IDR;

    // Maskovanie piny PC13, PC14, PC15 (0xE000) a posun o 13 bitov doprava
    uint8_t binary_combination = (uint8_t)((port_c & 0xE000) >> 13);

    // Presné priradenie kombinácií podľa tvojho merania na plošnom spoji
    switch (binary_combination) {
        case 0: return SWITCH_POS_10A;
        case 1: return SWITCH_POS_uA;
        case 2: return SWITCH_POS_Hz_Pct;
        case 3: return SWITCH_POS_mV;
        case 4: return SWITCH_POS_mA;
        case 5: return SWITCH_POS_OHM_DIODE_CONT;
        case 6: return SWITCH_POS_Cx;
        case 7: return SWITCH_POS_V;
        default: return SWITCH_POS_V; // Bezpečný default ak by niečo zlyhalo
    }
}

// Cyklovanie sub-režimov tlačidla SELECT na základe novej sady rozsahov
SubMode_t ROTARY_GetNextSubMode(SwitchPosition_t current_pos, SubMode_t current_sub) {
    switch (current_pos) {
        case SWITCH_POS_V:
        case SWITCH_POS_mV:
            // Pre hlavné napätie V a mV prepíname iba medzi DC a AC
            return (current_sub == SUB_MODE_DC) ? SUB_MODE_AC : SUB_MODE_DC;

        case SWITCH_POS_OHM_DIODE_CONT:
            // Kombinovaná poloha: Odpor -> Dióda -> Pípadlo -> späť na Odpor
            if (current_sub == SUB_MODE_RESISTANCE) return SUB_MODE_DIODE;
            else if (current_sub == SUB_MODE_DIODE) return SUB_MODE_CONTINUITY;
            else                                    return SUB_MODE_RESISTANCE;

        case SWITCH_POS_Hz_Pct:
            // Poloha Frekvencia a Strieda: Striedame medzi FREQ (Hz) a DUTY (%)
            return (current_sub == SUB_MODE_FREQ) ? SUB_MODE_DUTY : SUB_MODE_FREQ;

        case SWITCH_POS_uA:
        case SWITCH_POS_mA:
        case SWITCH_POS_10A:
            // Pre všetky prúdové rozsahy prepíname DC a AC režim
            return (current_sub == SUB_MODE_DC) ? SUB_MODE_AC : SUB_MODE_DC;

        case SWITCH_POS_Cx:
        default:
            return current_sub; // Pri kapacite SELECT nič nerobí, vráti nezmenený stav
    }
}

// Zostavenie príkazu pre merací čip na základe pozície a sub-režimu
uint8_t ROTARY_GetChipCommand(SwitchPosition_t pos, SubMode_t sub) {
    switch (pos) {
        case SWITCH_POS_V:
            return (sub == SUB_MODE_AC) ? CMD_CHIP_6V_AC : CMD_CHIP_6V_DC;

        case SWITCH_POS_mV:
            return (sub == SUB_MODE_AC) ? CMD_CHIP_600mV_AC : CMD_CHIP_600mV_DC;

        case SWITCH_POS_OHM_DIODE_CONT:
            if (sub == SUB_MODE_DIODE)      return CMD_CHIP_DIODE;
            if (sub == SUB_MODE_CONTINUITY) return CMD_CHIP_CONT;
            return CMD_CHIP_RES;

        case SWITCH_POS_Cx:
            return CMD_CHIP_CAP;

        case SWITCH_POS_Hz_Pct:
            return (sub == SUB_MODE_DUTY) ? CMD_CHIP_PCT : CMD_CHIP_HZ;

        case SWITCH_POS_uA:
            return (sub == SUB_MODE_AC) ? CMD_CHIP_uA_AC : CMD_CHIP_uA_DC;

        case SWITCH_POS_mA:
            return (sub == SUB_MODE_AC) ? CMD_CHIP_mA_AC : CMD_CHIP_mA_DC;

        case SWITCH_POS_10A:
            return (sub == SUB_MODE_AC) ? CMD_CHIP_10A_AC : CMD_CHIP_10A_DC;

        default:
            return CMD_CHIP_UNKNOWN;
    }
}

// Dotaz, ktorým sa čítajú dáta v danej polohe (podľa sigrok záznamov pôvodného FW):
//   Hz -> 61, Duty % -> 62, všetko ostatné (aj Cx) -> 60
uint8_t ROTARY_GetQuery(SwitchPosition_t pos, SubMode_t sub) {
    if (pos == SWITCH_POS_Hz_Pct) {
        return (sub == SUB_MODE_DUTY) ? MEAS_QUERY_DUTY : MEAS_QUERY_FREQ;
    }
    return MEAS_QUERY_MAIN;
}

// Východzí sub-mód po otočení prepínača
SubMode_t ROTARY_GetDefaultSubMode(SwitchPosition_t pos) {
    switch (pos) {
        case SWITCH_POS_OHM_DIODE_CONT: return SUB_MODE_RESISTANCE;
        case SWITCH_POS_Cx:             return SUB_MODE_CAPACITANCE;
        case SWITCH_POS_Hz_Pct:         return SUB_MODE_FREQ;
        default:                        return SUB_MODE_DC;
    }
}

// Odrušenie prepínača: pri otáčaní kontakty prechádzajú kombináciami iných polôh.
// Poloha sa prijme až keď je rovnaká aspoň ROTARY_STABLE_MS.
#define ROTARY_STABLE_MS  50

bool ROTARY_PollStable(SwitchPosition_t *pos) {
    static SwitchPosition_t candidate = (SwitchPosition_t)0xFF;
    static SwitchPosition_t accepted  = (SwitchPosition_t)0xFF;
    static uint32_t candidate_tick = 0;

    SwitchPosition_t raw = ROTARY_GetSwitchPosition();
    uint32_t now = HAL_GetTick();

    if (raw != candidate) {
        candidate = raw;
        candidate_tick = now;
        return false;
    }

    if (candidate != accepted && (now - candidate_tick) >= ROTARY_STABLE_MS) {
        accepted = candidate;
        *pos = accepted;
        return true;
    }
    return false;
}
