/*
 * menu.h
 *
 *  Menu nastavení (otvára sa dlhým stlačením F1).
 *  F1 = EXIT (uloží do flash), F2 = NEXT (ďalší riadok dokola), F3 = - / OFF, F4 = + / ON.
 */

#ifndef MENU_H_
#define MENU_H_

#include <stdbool.h>
#include "button.h"

void Menu_Open(void);

// Volať v každom prechode hlavnej slučky, kým je menu otvorené.
// ev = nová udalosť tlačidla (alebo BTN_NONE). Vráti false, keď sa menu zatvorilo.
bool Menu_Task(ButtonType_t ev);

#endif /* MENU_H_ */
