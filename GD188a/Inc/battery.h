/*
 * battery.h
 *
 *  Meranie napätia batérie: PA7 (U_bat) = ADC1_IN7, delič 10k / 10k -> Ubat / 2.
 *  Delič je napájaný cez tranzistor, ktorý zopne spolu s napájaním príslušenstva (PA2),
 *  preto sa meria až po LCD_Power_Accessories() a v spánku delič neodoberá prúd.
 *  Referencia ADC (VDDA) sa dopočíta z internej referencie VREFINT, takže výsledok
 *  nezávisí od presnosti 3 V stabilizátora.
 */

#ifndef BATTERY_H_
#define BATTERY_H_

#include <stdint.h>

// Hranice pre prepočet na percentá – Li-ion 1 článok (PREDPOKLAD, upraviť podľa batérie)
#define BAT_FULL_MV    4200u
#define BAT_EMPTY_MV   3300u

void     Battery_Init(void);
void     Battery_Task(void);        // volať v hlavnej slučke, meria každé 2 s
uint16_t Battery_GetMillivolts(void);
uint8_t  Battery_GetPercent(void);
uint8_t  Battery_GetAvgPercent(void);  // z priemeru ~30 s (odhad času chodu)
uint16_t Battery_GetAvgMillivolts(void); // priemer ~30 s (0 = ešte nemerané)

#endif /* BATTERY_H_ */
