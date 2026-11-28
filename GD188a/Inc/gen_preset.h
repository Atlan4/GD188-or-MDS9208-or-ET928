/*
 * gen_preset.h
 *
 *  Pamäte nastavení generátora (menu MEMORY, dlhé SAVE na obrazovke generátora).
 *
 *    položky 0..9  (na displeji 1..10) – užívateľské pamäte v EEPROM
 *    položka 10    (na displeji 11)    – pevné nastavenie SQUARE 1 kHz 50 % 3.0 Vpp, nedá sa prepísať
 *
 *  Uloženie v EEPROM: viď eeprom_map.h (EE_PRESETS_ADDR). Každá pamäť má vlastných 8 B
 *  so značkou a kontrolným súčtom, takže prerušený zápis pokazí nanajvýš jednu pamäť.
 */

#ifndef INC_GEN_PRESET_H_
#define INC_GEN_PRESET_H_

#include <stdint.h>
#include <stdbool.h>

#define GEN_PRESET_USER    10                       // počet užívateľských pamätí
#define GEN_PRESET_COUNT   (GEN_PRESET_USER + 1)    // + pevná položka
#define GEN_PRESET_FIXED   GEN_PRESET_USER          // index pevnej položky

typedef struct {
    bool    used;          // false = prázdna pamäť
    uint8_t wave;
    uint8_t freq_idx;
    uint8_t duty_idx;
    uint8_t volt_idx;
} GenPreset_t;

// Načíta všetky pamäte z EEPROM do RAM (volať pri otvorení menu MEMORY)
void GenPreset_ReadAll(void);

// Pamäť z RAM (po GenPreset_ReadAll). Pevná položka je vždy obsadená.
const GenPreset_t *GenPreset_Get(uint8_t idx);

// Uloží aktuálne nastavenie generátora (g_gen) do pamäte idx (0..9). true = OK (overené čítaním)
bool GenPreset_Store(uint8_t idx);

// Nastaví generátor (g_gen) podľa pamäte idx. false = prázdna pamäť. Výstup ON/OFF sa nemení.
bool GenPreset_Apply(uint8_t idx);

#endif /* INC_GEN_PRESET_H_ */
