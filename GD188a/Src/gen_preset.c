/*
 * gen_preset.c
 *
 *  Pamäte nastavení generátora v EEPROM.
 *
 *  Jedna pamäť = 8 B na adrese EE_PRESETS_ADDR + idx * 8 (8 pamätí na stránku 64 B,
 *  žiadna pamäť neprechádza cez hranicu stránky -> zápis jednej pamäte = jeden zápis stránky):
 *
 *    [0] značka 0x5A
 *    [1] wave   [2] freq_idx   [3] duty_idx   [4] volt_idx
 *    [5] [6] rezerva (0xFF)
 *    [7] kontrolný súčet = ~(súčet bajtov 0..6)
 *
 *  Prázdna (nikdy nezapísaná) EEPROM má 0xFF -> značka nesedí -> pamäť je prázdna.
 */

#include <string.h>

#include "gen_preset.h"
#include "gen.h"
#include "eeprom.h"

#define SLOT_SIZE    8u
#define SLOT_MARK    0x5Au

static GenPreset_t presets[GEN_PRESET_COUNT];

// Pevná položka 11: SQUARE, 1 kHz, 50 %, 3.0 Vpp (indexy do zoznamov v gen.c)
static const GenPreset_t FIXED = {
    .used     = true,
    .wave     = GEN_WAVE_SQUARE,
    .freq_idx = 3,             // 1 kHz
    .duty_idx = 4,             // 50 %
    .volt_idx = 10,            // 3.0 Vpp
};

static uint16_t SlotAddr(uint8_t idx) {
    return (uint16_t)(EE_PRESETS_ADDR + idx * SLOT_SIZE);
}

static uint8_t SlotSum(const uint8_t *b) {
    uint8_t s = 0;
    for (uint8_t i = 0; i < SLOT_SIZE - 1u; i++) s = (uint8_t)(s + b[i]);
    return (uint8_t)~s;
}

static void Decode(const uint8_t *b, GenPreset_t *p) {
    memset(p, 0, sizeof(*p));
    if (b[0] != SLOT_MARK || b[7] != SlotSum(b)) return;
    if (b[1] >= GEN_WAVE_COUNT || b[2] >= GEN_NUM_FREQ ||
        b[3] >= GEN_NUM_DUTY   || b[4] >= GEN_NUM_VOLT) return;
    p->used     = true;
    p->wave     = b[1];
    p->freq_idx = b[2];
    p->duty_idx = b[3];
    p->volt_idx = b[4];
}

void GenPreset_ReadAll(void) {
    uint8_t buf[GEN_PRESET_USER * SLOT_SIZE];
    bool ok = EEPROM_Read(EE_PRESETS_ADDR, buf, sizeof(buf));
    for (uint8_t i = 0; i < GEN_PRESET_USER; i++) {
        if (ok) Decode(&buf[i * SLOT_SIZE], &presets[i]);
        else    memset(&presets[i], 0, sizeof(presets[i]));     // EEPROM neodpovedá -> prázdne
    }
    presets[GEN_PRESET_FIXED] = FIXED;
}

const GenPreset_t *GenPreset_Get(uint8_t idx) {
    if (idx >= GEN_PRESET_COUNT) idx = GEN_PRESET_FIXED;
    return &presets[idx];
}

bool GenPreset_Store(uint8_t idx) {
    if (idx >= GEN_PRESET_USER) return false;          // pevná položka sa neprepisuje

    uint8_t b[SLOT_SIZE];
    b[0] = SLOT_MARK;
    b[1] = g_gen.wave;
    b[2] = g_gen.freq_idx;
    b[3] = g_gen.duty_idx;
    b[4] = g_gen.volt_idx;
    b[5] = 0xFF;
    b[6] = 0xFF;
    b[7] = SlotSum(b);

    if (!EEPROM_Write(SlotAddr(idx), b, SLOT_SIZE)) return false;

    // Kontrola spätným čítaním
    uint8_t chk[SLOT_SIZE];
    if (!EEPROM_Read(SlotAddr(idx), chk, SLOT_SIZE) || memcmp(chk, b, SLOT_SIZE) != 0) return false;

    Decode(b, &presets[idx]);
    return true;
}

bool GenPreset_Apply(uint8_t idx) {
    const GenPreset_t *p = GenPreset_Get(idx);
    if (!p->used) return false;
    g_gen.wave     = p->wave;
    g_gen.freq_idx = p->freq_idx;
    g_gen.duty_idx = p->duty_idx;
    g_gen.volt_idx = p->volt_idx;
    Gen_Update();              // obmedzenie frekvencie podľa tvaru; pri zapnutom výstupe hneď platí
    return true;
}
