/*
 * scope_test.h
 *
 *  Testovací režim osciloskopu (dlhé AUTO v polohe V alebo mV, odchod znova dlhým AUTO).
 *  Slúži na zistenie, ako prepínače vstupnej časti menia zosilnenie, aby sa dali priradiť rozsahy V/div.
 *
 *    F1    relé AC/DC         (PA9)
 *    F2    optočleny O1 a O2  (PA15)
 *    F3    prepínač U6        (PC10, TS5A3157)
 *    F4    prepínač U7        (PC11, TS5A3157)
 *    SAVE  relé 1/10          (PA8)
 *    AUTO krátko: automatický prechod 16 kombinácií (1/10, U7, U6, O1O2) -> tabuľka priemer / rozkmit
 *    AUTO dlho:   odchod
 *
 *  ADC MS9280: dáta PC0–PC7 (horných 8 bitov), hodiny PC12, STBY PA10 (0 = beží).
 *  V teste sa hodiny generujú programovo, 300 vzoriek pri 100 ksps (10 µs na vzorku).
 */

#ifndef INC_SCOPE_TEST_H_
#define INC_SCOPE_TEST_H_

#include <stdbool.h>
#include "button.h"

void ScopeTest_InitPins(void);           // pri štarte: všetky výstupy 0, ADC v STBY
void ScopeTest_Open(void);               // nakreslí obrazovku, spustí ADC
void ScopeTest_Close(bool keep_state);   // výstupy do 0, ADC do STBY; keep_state = zapamätať (návrat z generátora)
bool ScopeTest_Task(ButtonType_t ev);    // false = odchod (dlhé AUTO)

#endif /* INC_SCOPE_TEST_H_ */
