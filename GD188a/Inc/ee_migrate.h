/*
 * ee_migrate.h
 *
 *  v0.72: jednorazový presun dát EEPROM z rozloženia v0.71 (popis v ee_migrate.c a eeprom_map.h).
 */

#ifndef EE_MIGRATE_H_
#define EE_MIGRATE_H_

#include <stdbool.h>

// true = migrácia ešte nie je dokončená (volať po EEPROM_Init)
bool EeMigrate_Needed(void);

// Dokončí migráciu (pokračuje od prerušeného kroku). true = hotovo, false = chyba EEPROM
// (pri ďalšom štarte sa pokračuje). Volať pred Settings_Load.
bool EeMigrate_Run(void);

#endif /* EE_MIGRATE_H_ */
