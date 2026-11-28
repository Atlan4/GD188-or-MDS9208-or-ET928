/*
 * ee_migrate.c
 *
 *  v0.72: jednorazový presun dát EEPROM z rozloženia v0.71 na nové rozloženie (eeprom_map.h).
 *  Spúšťa sa pri každom štarte pred Settings_Load. Ak je v stave migrácie značka "hotovo",
 *  nerobí nič (iba prečíta 2 x 16 B).
 *
 *  Staré a nové oblasti sa prekrývajú, preto sa presúva v bezpečnom poradí po krokoch.
 *  Číslo nasledujúceho kroku sa po každom kroku zapíše do stavového záznamu (kópie A / B
 *  na 0x7F80 / 0x7FC0, striedavo s počítadlom). Pri výpadku napájania sa po ďalšom zapnutí
 *  pokračuje krokom, ktorý sa nedokončil – každý krok je opakovateľný, lebo jeho zdroj
 *  prepíše až niektorý neskorší krok.
 *
 *   Krok  0      staré nastavenia + rozsahy (v0.71) -> nové záznamy (prístroj, generátor,
 *                rozsahy s def_sub) do zálohy 0x7F00 (zo zálohy sa zapisujú v krokoch 13, 14, 17)
 *   Kroky 1..10  pamäte priebehov 10..1: 0x0400 + i*512 -> 0x0600 + i*512 (od konca, po stránkach)
 *   Krok 11      osciloskop (nastavenie + kalibrácia) 0x0100..0x01FF -> 0x0500..0x05FF
 *   Krok 12      pamäte generátora 0x0080..0x00FF -> 0x0480..0x04FF
 *   Krok 13      nastavenia generátora (záloha) -> 0x0400, kópia B 0x0440 sa vymaže
 *   Krok 14      Save range + rozsahy + def_sub (záloha) -> 0x0080, kópia B 0x00C0 sa vymaže
 *   Krok 15      pamäť hodnôt 24 x 8 B 0x0280..0x033F -> 0x0100..0x01BF
 *   Krok 16      vymazanie 0x01C0..0x03FF (pamäte 25..96 prázdne)
 *   Krok 17      nastavenia prístroja (záloha) -> 0x0000, kópia B 0x0040 sa vymaže
 *   Krok 18      značka "hotovo" (zapisuje sa ako posledná)
 *
 *  Prázdna EEPROM (nový prístroj) prejde rovnakými krokmi – presunú sa iba 0xFF a zapíšu sa
 *  predvolené nastavenia. Trvá to cca 1 s, iba raz.
 */

#include <string.h>

#include "ee_migrate.h"
#include "eeprom.h"
#include "eeprom_map.h"
#include "settings.h"

#define MIG_MAGIC     0x4D474447UL      // "GDGM"
#define MIG_STEP_DONE 18u

typedef struct {
    uint32_t magic;
    uint16_t seq;
    uint8_t  step;                       // nasledujúci krok, MIG_STEP_DONE = hotovo
    uint8_t  step_inv;                   // ~step (kontrola)
    uint32_t rsv;
    uint32_t crc;
} MigState_t;

_Static_assert(sizeof(MigState_t) == 16, "MigState_t = 16 B");

static const uint16_t STATE_ADDR[2] = { EE_MIGRATE_STATE_A, EE_MIGRATE_STATE_B };
static int8_t   st_slot = -1;
static uint16_t st_seq  = 0;

static uint32_t Crc32(const uint8_t *p, uint32_t n) {
    uint32_t crc = 0xFFFFFFFFUL;
    while (n--) {
        crc ^= *p++;
        for (int k = 0; k < 8; k++) crc = (crc & 1u) ? (crc >> 1) ^ 0xEDB88320UL : (crc >> 1);
    }
    return ~crc;
}

static bool StateValid(const MigState_t *s) {
    return s->magic == MIG_MAGIC && (uint8_t)~s->step == s->step_inv && s->step <= MIG_STEP_DONE &&
           s->crc == Crc32((const uint8_t *)s, sizeof(MigState_t) - 4u);
}

// Krok zo stavu; žiadny platný stav = 0 (migrácia ešte nezačala)
static uint8_t StateLoad(void) {
    MigState_t s[2];
    bool ok[2];
    for (int k = 0; k < 2; k++) ok[k] = EEPROM_Read(STATE_ADDR[k], (uint8_t *)&s[k], sizeof(s[k])) && StateValid(&s[k]);
    int best = -1;
    if (ok[0] && ok[1]) best = ((int16_t)(s[1].seq - s[0].seq) > 0) ? 1 : 0;
    else if (ok[0])     best = 0;
    else if (ok[1])     best = 1;
    if (best < 0) { st_slot = -1; st_seq = 0; return 0; }
    st_slot = (int8_t)best;
    st_seq  = s[best].seq;
    return s[best].step;
}

static bool StateSave(uint8_t step) {
    MigState_t s;
    memset(&s, 0xFF, sizeof(s));
    s.magic    = MIG_MAGIC;
    s.seq      = (uint16_t)(st_seq + 1u);
    s.step     = step;
    s.step_inv = (uint8_t)~step;
    s.crc      = Crc32((const uint8_t *)&s, sizeof(s) - 4u);
    int k = (st_slot == 0) ? 1 : 0;
    if (!EEPROM_Write(STATE_ADDR[k], (const uint8_t *)&s, sizeof(s))) return false;
    MigState_t chk;
    if (!EEPROM_Read(STATE_ADDR[k], (uint8_t *)&chk, sizeof(chk)) || memcmp(&chk, &s, sizeof(s)) != 0) return false;
    st_slot = (int8_t)k;
    st_seq  = s.seq;
    return true;
}

// Zápis s kontrolou spätným čítaním (po stránkach)
static bool WriteVerify(uint16_t addr, const uint8_t *b, uint16_t n) {
    uint8_t chk[EE_PAGE_SIZE];
    while (n) {
        uint16_t k = (uint16_t)(EE_PAGE_SIZE - (addr % EE_PAGE_SIZE));
        if (k > n) k = n;
        if (!EEPROM_Write(addr, b, k)) return false;
        if (!EEPROM_Read(addr, chk, k) || memcmp(chk, b, k) != 0) return false;
        addr = (uint16_t)(addr + k); b += k; n = (uint16_t)(n - k);
    }
    return true;
}

// Kopírovanie po stránkach od konca (src aj dst zarovnané na stránku; bezpečné aj pri posune dopredu)
static bool CopyBack(uint16_t src, uint16_t dst, uint16_t len) {
    uint8_t buf[EE_PAGE_SIZE];
    for (uint16_t off = len; off > 0; ) {
        uint16_t k = (off >= EE_PAGE_SIZE) ? EE_PAGE_SIZE : off;
        off = (uint16_t)(off - k);
        if (!EEPROM_Read((uint16_t)(src + off), buf, k)) return false;
        if (!WriteVerify((uint16_t)(dst + off), buf, k)) return false;
    }
    return true;
}

static bool Fill(uint16_t dst, uint16_t len) {
    uint8_t buf[EE_PAGE_SIZE];
    memset(buf, 0xFF, sizeof(buf));
    for (uint16_t off = 0; off < len; off = (uint16_t)(off + EE_PAGE_SIZE)) {
        uint16_t k = (uint16_t)((len - off >= EE_PAGE_SIZE) ? EE_PAGE_SIZE : (len - off));
        if (!WriteVerify((uint16_t)(dst + off), buf, k)) return false;
    }
    return true;
}

// Záznam zo zálohy (0 = prístroj, 1 = generátor, 2 = rozsahy) -> kópia A, kópia B sa vymaže
static bool FromBackup(uint8_t which, uint16_t addr_a, uint16_t addr_b) {
    uint8_t rec[SETTINGS_REC_SIZE];
    if (!EEPROM_Read((uint16_t)(EE_MIGRATE_DATA_ADDR + which * SETTINGS_REC_SIZE), rec, sizeof(rec))) return false;
    if (!WriteVerify(addr_a, rec, sizeof(rec))) return false;
    return Fill(addr_b, EE_PAGE_SIZE);
}

static bool DoStep(uint8_t step) {
    if (step == 0) {
        uint8_t b[3 * SETTINGS_REC_SIZE];
        Settings_LoadV071();
        Settings_EncodeNew(&b[0], &b[SETTINGS_REC_SIZE], &b[2 * SETTINGS_REC_SIZE]);
        return WriteVerify(EE_MIGRATE_DATA_ADDR, b, sizeof(b));
    }
    if (step <= EE_WAVE_SLOTS) {
        uint16_t i = (uint16_t)(EE_WAVE_SLOTS - step);          // 9, 8, ... 0
        return CopyBack((uint16_t)(EE_V071_WAVE + i * EE_WAVE_SLOT_SIZE),
                        (uint16_t)(EE_WAVE_ADDR + i * EE_WAVE_SLOT_SIZE), EE_WAVE_SLOT_SIZE);
    }
    switch (step) {
        case 11: return CopyBack(EE_V071_SCOPE, EE_SCOPE_AREA_ADDR, EE_SCOPE_AREA_SIZE);
        case 12: return CopyBack(EE_V071_PRESETS, EE_PRESETS_ADDR, EE_PRESETS_SIZE);
        case 13: return FromBackup(1, EE_GEN_A_ADDR, EE_GEN_B_ADDR);
        case 14: return FromBackup(2, EE_RANGES_A_ADDR, EE_RANGES_B_ADDR);
        case 15: return CopyBack(EE_V071_METER_MEM, EE_METER_MEM_ADDR, EE_V071_METER_MEM_SIZE);
        case 16: return Fill((uint16_t)(EE_METER_MEM_ADDR + EE_V071_METER_MEM_SIZE),
                             (uint16_t)(EE_METER_MEM_SIZE - EE_V071_METER_MEM_SIZE));
        case 17: return FromBackup(0, EE_SETTINGS_A_ADDR, EE_SETTINGS_B_ADDR);
        default: return true;
    }
}

bool EeMigrate_Needed(void) {
    if (!EEPROM_IsPresent()) return false;
    return StateLoad() < MIG_STEP_DONE;
}

bool EeMigrate_Run(void) {
    if (!EEPROM_IsPresent()) return false;
    uint8_t step = StateLoad();
    while (step < MIG_STEP_DONE) {
        if (!DoStep(step)) return false;                 // chyba: pri ďalšom štarte znova od tohto kroku
        step++;
        if (!StateSave(step)) return false;
    }
    return true;
}
