/*
 * meter_mem.h
 *
 *  v0.71: pamäť nameraných hodnôt multimetra (krátke SAVE na meracej obrazovke).
 *  v0.72: 24 / 48 / 72 / 96 pamätí (menu "Meter memory") v EEPROM (EE_METER_MEM_ADDR, 8 B na pamäť),
 *  zobrazené po stránkach 3 stĺpce x 8 riadkov.
 *  Hodnota sa ukladá tak, ako je na displeji (4 číslice, bodka, znamienko, OL), s jednotkou
 *  a režimom (V DC, mV AC, Ω, dióda ...). Popis ovládania v meter_mem.c.
 */

#ifndef METER_MEM_H_
#define METER_MEM_H_

#include <stdint.h>
#include <stdbool.h>
#include "button.h"

#define METER_MEM_SLOTS     96      // v0.72: najviac (miesto v EEPROM)
#define METER_MEM_SLOTS_MIN 24      // jedna stránka
#define METER_MEM_ROWS    8
#define METER_MEM_COLS    3

// Režim (veličina) uloženej hodnoty
typedef enum {
    MM_NONE = 0,
    MM_V_DC, MM_V_AC, MM_mV_DC, MM_mV_AC,
    MM_uA_DC, MM_uA_AC, MM_mA_DC, MM_mA_AC, MM_A_DC, MM_A_AC,
    MM_OHM, MM_DIODE, MM_CONT, MM_CAP, MM_FREQ, MM_DUTY,
    MM_COUNT
} MeterMemMode_t;

// Hodnota tak, ako je na displeji
typedef struct {
    bool     used;          // false = prázdna pamäť / zatiaľ žiadna hodnota
    uint8_t  mode;          // MeterMemMode_t
    bool     ol;            // mimo rozsah (OL)
    bool     neg;           // záporná hodnota
    bool     rel;           // hodnota s REL korekciou
    uint8_t  decimals;      // desatinné miesta (0..4)
    uint16_t mag;           // absolútna hodnota bez bodky 0..9999
    char     unit[4];       // jednotka, max. 3 znaky ("mV", "k\x7f", "kHz", "%")
    uint8_t  tag;           // v0.76: MM_TAG_* (hodnota MAX / MIN z MIN/MAX)
} MeterMemVal_t;
enum { MM_TAG_NONE = 0, MM_TAG_MAX, MM_TAG_MIN };

// Otvorenie obrazovky (načíta pamäte z EEPROM a nakreslí všetko okrem horného riadku)
void MeterMem_Open(void);

// Volať v každom prechode slučky. now = aktuálna hodnota z displeja, hold = HOLD je zapnutý.
// Vráti false = odchod (EXIT) – volajúci prekreslí meraciu obrazovku.
// v0.76: mmax / mmin = MIN/MAX z meracej obrazovky (NULL = MIN/MAX vypnuté). Pri zapnutom MIN/MAX
// uloží SAVE 3 pamäte za sebou: hodnota, MAX, MIN (MAX / MIN so značkou).
bool MeterMem_Task(ButtonType_t ev, const MeterMemVal_t *now, bool hold,
                   const MeterMemVal_t *mmax, const MeterMemVal_t *mmin);

// v0.72: nadpis horného riadku ("METER MEMORY" alebo "METER MEMORY 2/4")
const char *MeterMem_Title(void);

// v0.72: aktívne hlásenie (SAVED TO 5 ...) pre horný riadok cez celú šírku, inak NULL
const char *MeterMem_TopMsg(uint16_t *col);

#endif /* METER_MEM_H_ */
