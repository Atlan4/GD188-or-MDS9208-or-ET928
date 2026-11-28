/*
 * eeprom_map.h
 *
 *  Rozdelenie EEPROM BL24C256A (32 KB, stránka 64 B, I2C1 na PB8 = SCL, PB9 = SDA).
 *
 *  Pravidlo: každá oblasť začína na začiatku stránky (násobok 64), aby zápis jednej
 *  oblasti nikdy nezasiahol do inej a aby jeden zápis stránky = jedna oblasť.
 *
 *  v0.72 – nové rozloženie (pri prvom štarte v0.72 sa dáta z v0.71 presunú, ee_migrate.c):
 *
 *   Adresa          Veľkosť   Obsah
 *   0x0000-0x007F   128 B     Nastavenia prístroja – kópia A / B (32 B záznam, settings.c)
 *                             v0.74: + bajt 13 = HV warning, v0.78: + bajt 14 = Diode beep (0xFF = predvolené)
 *   0x0080-0x00FF   128 B     Save range + uložené rozsahy + predvolené režimy (def_sub) – kópia A / B
 *                             v0.76: + bajt 18 = ~maska polôh s MIN/MAX zapnutým (dlhé SAVE), 0xFF = žiadna
 *   0x0100-0x03FF   768 B     Pamäť nameraných hodnôt multimetra, 96 x 8 B (meter_mem.c)
 *   0x0400-0x047F   128 B     Nastavenia generátora (posledné nastavenie, Auto ON, kalibrácia) – kópia A / B
 *   0x0480-0x04FF   128 B     Pamäte generátora 1..10 (8 B na pamäť, gen_preset.c)
 *   0x0500-0x05FF   256 B     Osciloskop: posledné nastavenie A/B (0x500/0x540), kalibrácia A/B (0x580/0x5C0)
 *                             v0.76: záznam nastavenia v2 "S2" = 32 B (+ Interp / Peak det / View), v1 "SS" sa číta
 *   0x0600-0x19FF  5 KB       Pamäte priebehov osciloskopu 10 x 512 B (bez kópií A/B, scope.c)
 *   0x1A00-0x7EFF  ~25 KB     Voľné
 *   0x7F00-0x7F7F   128 B     Migrácia v0.71 -> v0.72: záloha prevedených záznamov
 *   0x7F80-0x7FFF   128 B     Migrácia: stav (krok) – kópia A / B, po dokončení značka "hotovo"
 *
 *  Dve kópie záznamov sa zapisujú striedavo s počítadlom – ak sa zápis preruší
 *  (vypnutie napájania), zostane platná predchádzajúca kópia.
 *
 *  Rozloženie v0.71 (iba pre migráciu, EE_V071_*):
 *   0x0000/0x0040 nastavenia A/B (vrátane generátora a def_sub), 0x0080 pamäte generátora,
 *   0x0100-0x01FF osciloskop, 0x0200/0x0240 rozsahy A/B, 0x0280-0x033F pamäť hodnôt 24 x 8 B,
 *   0x0400-0x17FF pamäte priebehov 10 x 512 B.
 */

#ifndef EEPROM_MAP_H_
#define EEPROM_MAP_H_

#define EE_SIZE                 32768u
#define EE_PAGE_SIZE            64u

#define EE_SETTINGS_A_ADDR      0x0000u     // nastavenia prístroja
#define EE_SETTINGS_B_ADDR      0x0040u

#define EE_RANGES_A_ADDR        0x0080u     // Save range, rozsahy, def_sub
#define EE_RANGES_B_ADDR        0x00C0u

#define EE_METER_MEM_ADDR       0x0100u     // namerané hodnoty multimetra (96 x 8 B, 12 stránok)
#define EE_METER_MEM_SIZE       0x0300u

#define EE_GEN_A_ADDR           0x0400u     // nastavenia generátora
#define EE_GEN_B_ADDR           0x0440u

#define EE_PRESETS_ADDR         0x0480u     // pamäte generátora (2 stránky)
#define EE_PRESETS_SIZE         0x0080u

#define EE_SCOPE_AREA_ADDR      0x0500u     // osciloskop, 4 stránky
#define EE_SCOPE_AREA_SIZE      0x0100u
#define EE_SCOPE_SETUP_A        0x0500u
#define EE_SCOPE_SETUP_B        0x0540u
#define EE_SCOPE_CAL_A          0x0580u
#define EE_SCOPE_CAL_B          0x05C0u

#define EE_WAVE_ADDR            0x0600u     // pamäte priebehov: 10 x 512 B
#define EE_WAVE_SLOT_SIZE       512u
#define EE_WAVE_SLOTS           10u

#define EE_FREE_ADDR            0x1A00u     // voľné až po EE_MIGRATE_DATA_ADDR
#define EE_FREE_SIZE            (EE_MIGRATE_DATA_ADDR - EE_FREE_ADDR)

#define EE_MIGRATE_DATA_ADDR    0x7F00u     // záloha prevedených záznamov (2 stránky)
#define EE_MIGRATE_STATE_A      0x7F80u     // stav migrácie A / B
#define EE_MIGRATE_STATE_B      0x7FC0u

// --- Rozloženie v0.71 (zdroj migrácie) ---
#define EE_V071_SETTINGS_A      0x0000u
#define EE_V071_SETTINGS_B      0x0040u
#define EE_V071_PRESETS         0x0080u
#define EE_V071_SCOPE           0x0100u     // 256 B
#define EE_V071_RANGES_A        0x0200u
#define EE_V071_RANGES_B        0x0240u
#define EE_V071_METER_MEM       0x0280u     // 24 x 8 B
#define EE_V071_METER_MEM_SIZE  0x00C0u
#define EE_V071_WAVE            0x0400u     // 10 x 512 B

#endif /* EEPROM_MAP_H_ */
