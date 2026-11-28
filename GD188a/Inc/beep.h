/*
 * beep.h
 *
 *  Bzučiak PB10 (aktívny, log. 1 = pípa) – pípanie na pozadí, bez čakania.
 *    Beep_Error()   dlhé pípnutie (~400 ms) pri chybe (červená hláška)
 *    Beep_Warning() krátke pípnutie (~60 ms) pri upozornení (oranžová hláška)
 *  Oba sa riadia nastavením Beep (menu F1). Beep_Start() pípne vždy.
 */

#ifndef INC_BEEP_H_
#define INC_BEEP_H_

#include <stdint.h>

void Beep_Start(uint32_t ms);   // zapne bzučiak na ms (predĺži prebiehajúce pípnutie)
void Beep_Task(void);           // volať v hlavnej slučke – vypne bzučiak po uplynutí času
void Beep_Error(void);
void Beep_Warning(void);

#endif /* INC_BEEP_H_ */
