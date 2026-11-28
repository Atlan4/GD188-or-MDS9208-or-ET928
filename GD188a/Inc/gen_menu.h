/*
 * gen_menu.h
 *
 *  Obrazovka generátora (otvára sa dlhým BL v polohe V – z voltmetra alebo z osciloskopu).
 *  AUTO = výstup ON/OFF, SAVE krátko = menu MEMORY, SAVE dlho = uložiť posledné nastavenie,
 *  F1 dlho = nastavenia generátora (Auto output ON, ON at change, CAL).
 */

#ifndef INC_GEN_MENU_H_
#define INC_GEN_MENU_H_

#include <stdbool.h>
#include <stdint.h>
#include "button.h"

void GenMenu_Open(bool from_scope);   // vykreslí celú obrazovku (okrem horného riadku); from_scope = nápoveda "to scope"
void GenMenu_Task(ButtonType_t ev);   // tlačidlá F1–F4, AUTO, SAVE; prekreslí zmeny
void GenMenu_Close(void);             // zavrie zoznam / MEMORY / SETTINGS (nastavenia sa uložia)
const char *GenMenu_Title(void);      // nadpis horného riadku: GENERATOR / GEN MEMORY / GEN SETTINGS
bool GenMenu_IsMenuOpen(void);        // true = MEMORY alebo SETTINGS (odpočet vypnutia stojí)
uint32_t GenMenu_MemoryIdleLeftMs(void); // zostávajúci čas do zatvorenia MEMORY / SETTINGS (ms)

// Kalibrácia (SETTINGS, položka CAL): main.c prepne čip na AC V a posiela meranie
bool GenMenu_CalActive(void);
void GenMenu_SetCalReading(bool valid, int32_t v01mv);   // nameraná hodnota v 0.1 mV

#endif /* INC_GEN_MENU_H_ */
