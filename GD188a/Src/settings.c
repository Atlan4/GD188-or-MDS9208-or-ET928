/*
 * settings.c
 *
 *  Užívateľské nastavenia v EEPROM (viď eeprom_map.h). v0.72: tri samostatné 32 B záznamy,
 *  každý v dvoch kópiách (A / B, každá vo vlastnej stránke):
 *    nastavenia prístroja        EE_SETTINGS_A/B  (0x0000 / 0x0040)
 *    Save range, rozsahy, def_sub EE_RANGES_A/B   (0x0080 / 0x00C0)
 *    nastavenia generátora       EE_GEN_A/B       (0x0400 / 0x0440)
 *
 *  Záznam: magic (4) | verzia (2) | počítadlo seq (2) | 20 B dát | CRC-32 (4).
 *  Zapisuje sa vždy do staršej kópie s vyšším počítadlom. Pri načítaní sa použije
 *  platná kópia s najvyšším seq. Prerušený zápis tak nikdy nezničí obe kópie.
 *  Ak EEPROM neodpovedá alebo obe kópie sú neplatné, použijú sa predvolené hodnoty.
 *  Každá položka sa kontroluje zvlášť (mimo rozsahu -> predvolená hodnota).
 *
 *  Starý formát v0.71 (jeden záznam s generátorom a def_sub, rozsahy na 0x0200) sa číta
 *  už iba pri migrácii (Settings_LoadV071, ee_migrate.c).
 */

#include <string.h>

#include "settings.h"
#include "eeprom.h"
#include "gen.h"
#include "eeprom_map.h"

// ---------------------------------------------------------------------------
// Všeobecný 32 B záznam s kópiami A / B
// ---------------------------------------------------------------------------
#define REC_DATA   20u

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t seq;              // počítadlo zápisov (pretečenie sa rieši porovnaním rozdielu)
    uint8_t  d[REC_DATA];
    uint32_t crc;              // CRC-32 cez všetky predchádzajúce bajty
} Rec_t;

_Static_assert(sizeof(Rec_t) == SETTINGS_REC_SIZE, "Rec_t musi mat 32 B");

typedef struct {
    uint16_t addr[2];          // kópia A, B
    uint32_t magic;
    uint16_t version;
    int8_t   slot;             // posledná platná kópia, -1 = žiadna
    uint16_t seq;
    uint8_t  last[REC_DATA];   // posledný uložený obsah (na zistenie zmien)
} Dual_t;

#define DEV_MAGIC     0x38314447UL      // "GD18" (rovnaká značka ako v0.71, nová verzia 2)
#define DEV_VERSION   2u
#define RNG_MAGIC     0x47524447UL      // "GDRG"
#define RNG_VERSION   2u                //  v2: + def_sub
#define GEN_MAGIC     0x4E474447UL      // "GDGN"
#define GEN_VERSION   1u

static Dual_t dev = { { EE_SETTINGS_A_ADDR, EE_SETTINGS_B_ADDR }, DEV_MAGIC, DEV_VERSION, -1, 0, {0} };
static Dual_t rng = { { EE_RANGES_A_ADDR,   EE_RANGES_B_ADDR   }, RNG_MAGIC, RNG_VERSION, -1, 0, {0} };
static Dual_t gen = { { EE_GEN_A_ADDR,      EE_GEN_B_ADDR      }, GEN_MAGIC, GEN_VERSION, -1, 0, {0} };

// Obsah záznamu prístroja (d[])
enum {
    D_AUTO_OFF = 0, D_AUTO_RESET, D_PROBE_RESET, D_GEN_OFF, D_PROBE_C, D_PROBE_VA, D_PROBE_VAC,
    D_QUICK_RDC, D_LONG_PRESS, D_BEEP, D_BAT_CAP, D_START_BL, D_METER_MEM,
    D_HV_WARN,               // v0.74: voľný bajt záznamu (v starších 0xFF -> predvolené ON+BEEP), verzia ostáva 2
    D_DIODE_BEEP             // v0.78: voľný bajt záznamu (v starších 0xFF -> predvolené ON), verzia ostáva 2
};
_Static_assert(D_DIODE_BEEP < REC_DATA, "D_DIODE_BEEP v zázname prístroja");
// Obsah záznamu generátora
enum { G_WAVE = 0, G_FREQ, G_DUTY, G_VOLT, G_AUTO_ON, G_ON_CHANGE, G_CAL };
// Obsah záznamu rozsahov: [0] Save range, [1..16] rozsahy, [17] def_sub (v1 z v0.70/71: 0xFF)
#define R_SAVE_RANGE  0u
#define R_RANGE       1u
#define R_DEF_SUB     (R_RANGE + SETTINGS_RANGE_SLOTS)
#define R_MINMAX      (R_DEF_SUB + 1u)  // v0.76: ~maska polôh s MIN/MAX (starší záznam 0xFF -> všetko OFF), verzia ostáva 2
_Static_assert(R_MINMAX < REC_DATA, "R_MINMAX v zázname rozsahov");

AppSettings_t g_settings;
uint8_t       g_saved_range[SETTINGS_RANGE_SLOTS];

static const AppSettings_t defaults = {
    .auto_off_min = 10,          // predvolený čas po nahratí firmvéru / prázdnej EEPROM
    .auto_reset   = true,
    .probe_reset  = true,
    .probe_c      = 1,           // 100 pF (ako pevný prah do v0.46)
    .probe_va     = 0,           // OFF
    .probe_vac    = 0,           // OFF
    .quick_rdc    = false,
    .def_sub      = 0,           // DC, odpor, Hz
    .gen_auto_on  = false,
    .gen_on_change = false,
    .gen_cal      = 0,
    .long_press   = 5,           // 500 ms
    .beep         = true,
    .battery_cap  = SETTINGS_BAT_CAP_DEF,   // 1500 mAh
    .start_bl     = 1,           // 50 %
    .hide_zeros   = true,        // v0.66: predvolene ON
    .save_range   = false,
    .meter_mem    = 0,           // v0.72: 24 pamätí
    .hv_warn      = HV_WARN_BEEP,  // v0.74: ON+BEEP
    .diode_beep   = true,        // v0.78: ON
    .gen_off_min  = 30,
    .gen_wave     = GEN_WAVE_SINE,
    .gen_freq     = 3,           // 1 kHz
    .gen_duty     = 4,           // 50 %
    .gen_volt     = 10,          // 3.0 Vpp
};

static uint32_t Crc32(const uint8_t *p, uint32_t n) {
    uint32_t crc = 0xFFFFFFFFUL;
    while (n--) {
        crc ^= *p++;
        for (int k = 0; k < 8; k++) {
            crc = (crc & 1u) ? (crc >> 1) ^ 0xEDB88320UL : (crc >> 1);
        }
    }
    return ~crc;
}

// Položka zo záznamu, alebo predvolená hodnota, ak je mimo rozsahu (napr. 0xFF)
static uint8_t Field(uint8_t v, uint8_t max, uint8_t def) {
    return (v <= max) ? v : def;
}

// Platná kópia s vyšším seq -> d[], true = nájdená
static bool DualLoad(Dual_t *s, uint8_t *d) {
    s->slot = -1;
    s->seq  = 0;
    Rec_t r[2];
    bool ok[2];
    for (int k = 0; k < 2; k++) {
        ok[k] = EEPROM_Read(s->addr[k], (uint8_t *)&r[k], sizeof(Rec_t)) &&
                r[k].magic == s->magic && r[k].version == s->version &&
                r[k].crc == Crc32((const uint8_t *)&r[k], sizeof(Rec_t) - 4u);
    }
    int best = -1;
    if (ok[0] && ok[1]) best = ((int16_t)(r[1].seq - r[0].seq) > 0) ? 1 : 0;   // aj po pretečení
    else if (ok[0])     best = 0;
    else if (ok[1])     best = 1;
    if (best < 0) return false;
    s->slot = (int8_t)best;
    s->seq  = r[best].seq;
    memcpy(s->last, r[best].d, REC_DATA);
    memcpy(d, r[best].d, REC_DATA);
    return true;
}

static void RecBuild(Rec_t *rec, const Dual_t *s, uint16_t seq, const uint8_t *d) {
    rec->magic   = s->magic;
    rec->version = s->version;
    rec->seq     = seq;
    memcpy(rec->d, d, REC_DATA);
    rec->crc     = Crc32((const uint8_t *)rec, sizeof(Rec_t) - 4u);
}

// Zapíše iba pri zmene, do staršej kópie, s kontrolou spätným čítaním. true = OK
static bool DualSave(Dual_t *s, const uint8_t *d) {
    if (s->slot >= 0 && memcmp(s->last, d, REC_DATA) == 0) return true;
    Rec_t rec;
    RecBuild(&rec, s, (uint16_t)(s->seq + 1u), d);
    int k = (s->slot == 0) ? 1 : 0;
    if (!EEPROM_Write(s->addr[k], (const uint8_t *)&rec, sizeof(rec))) return false;
    Rec_t chk;
    if (!EEPROM_Read(s->addr[k], (uint8_t *)&chk, sizeof(chk)) || memcmp(&chk, &rec, sizeof(rec)) != 0) {
        return false;
    }
    s->slot = (int8_t)k;
    s->seq  = rec.seq;
    memcpy(s->last, d, REC_DATA);
    return true;
}

// ---------------------------------------------------------------------------
// g_settings <-> dáta záznamov
// ---------------------------------------------------------------------------
#define START_BL_SHOW_ZEROS  0x04u

// Bajt start_bl: jas (bity 0–1) + bit 2 = nuly zobrazovať (Hide zeros OFF)
static uint8_t StartBlByte(void) {
    return (uint8_t)((g_settings.start_bl & 3u) | (g_settings.hide_zeros ? 0u : START_BL_SHOW_ZEROS));
}
static void StartBlDecode(uint8_t sb) {
    if (sb <= (START_BL_SHOW_ZEROS | 3u) && (sb & 3u) <= SETTINGS_START_BL_MAX) {
        g_settings.start_bl   = (uint8_t)(sb & 3u);
        g_settings.hide_zeros = (sb & START_BL_SHOW_ZEROS) == 0;
    } else {
        g_settings.start_bl   = defaults.start_bl;
        g_settings.hide_zeros = defaults.hide_zeros;
    }
}
static uint8_t LongPressDecode(uint8_t lp) {
    return (lp >= SETTINGS_LONG_PRESS_MIN && lp <= SETTINGS_LONG_PRESS_MAX) ? lp : defaults.long_press;
}
static uint8_t BatCapDecode(uint8_t bc) {
    return (bc >= SETTINGS_BAT_CAP_MIN && bc <= SETTINGS_BAT_CAP_MAX) ? bc : defaults.battery_cap;
}
static uint8_t GenOffDecode(uint8_t go) {
    return (go <= SETTINGS_GEN_OFF_MAX && go % SETTINGS_GEN_OFF_STEP == 0) ? go : defaults.gen_off_min;
}
static int8_t GenCalDecode(uint8_t gc) {
    return (gc >= 128u - SETTINGS_GEN_CAL_MAX && gc <= 128u + SETTINGS_GEN_CAL_MAX) ? (int8_t)((int)gc - 128) : 0;
}
static uint8_t DefSubDecode(uint8_t ds) {     // 0xFF = nezapísané; Ω 3 = neplatné -> odpor
    if (ds == 0xFF) ds = defaults.def_sub;
    if (((ds >> SETTINGS_DEF_OHM) & 3u) == 3u) ds &= (uint8_t)~(3u << SETTINGS_DEF_OHM);
    return ds;
}

static void DevEncode(uint8_t *d) {
    memset(d, 0xFF, REC_DATA);
    d[D_AUTO_OFF]    = g_settings.auto_off_min;
    d[D_AUTO_RESET]  = g_settings.auto_reset ? 1 : 0;
    d[D_PROBE_RESET] = g_settings.probe_reset ? 1 : 0;
    d[D_GEN_OFF]     = g_settings.gen_off_min;
    d[D_PROBE_C]     = g_settings.probe_c;
    d[D_PROBE_VA]    = g_settings.probe_va;
    d[D_PROBE_VAC]   = g_settings.probe_vac;
    d[D_QUICK_RDC]   = g_settings.quick_rdc ? 1 : 0;
    d[D_LONG_PRESS]  = g_settings.long_press;
    d[D_BEEP]        = g_settings.beep ? 1 : 0;
    d[D_BAT_CAP]     = g_settings.battery_cap;
    d[D_START_BL]    = StartBlByte();
    d[D_METER_MEM]   = g_settings.meter_mem;
    d[D_HV_WARN]     = g_settings.hv_warn;
    d[D_DIODE_BEEP]  = g_settings.diode_beep ? 1 : 0;
}
static void DevDecode(const uint8_t *d) {
    uint8_t ao = d[D_AUTO_OFF];
    g_settings.auto_off_min = (ao >= SETTINGS_AUTO_OFF_MIN_MIN && ao <= SETTINGS_AUTO_OFF_MIN_MAX) ? ao : defaults.auto_off_min;
    g_settings.auto_reset   = (d[D_AUTO_RESET]  <= 1) ? (d[D_AUTO_RESET]  == 1) : defaults.auto_reset;
    g_settings.probe_reset  = (d[D_PROBE_RESET] <= 1) ? (d[D_PROBE_RESET] == 1) : defaults.probe_reset;
    g_settings.gen_off_min  = GenOffDecode(d[D_GEN_OFF]);
    g_settings.probe_c      = Field(d[D_PROBE_C],   SETTINGS_PROBE_C_MAX,   defaults.probe_c);
    g_settings.probe_va     = Field(d[D_PROBE_VA],  SETTINGS_PROBE_VA_MAX,  defaults.probe_va);
    g_settings.probe_vac    = Field(d[D_PROBE_VAC], SETTINGS_PROBE_VAC_MAX, defaults.probe_vac);
    g_settings.quick_rdc    = (d[D_QUICK_RDC] == 1);
    g_settings.long_press   = LongPressDecode(d[D_LONG_PRESS]);
    g_settings.beep         = (d[D_BEEP] != 0);
    g_settings.battery_cap  = BatCapDecode(d[D_BAT_CAP]);
    StartBlDecode(d[D_START_BL]);
    g_settings.meter_mem    = Field(d[D_METER_MEM], SETTINGS_METER_MEM_MAX, defaults.meter_mem);
    g_settings.hv_warn      = Field(d[D_HV_WARN],   SETTINGS_HV_WARN_MAX,   defaults.hv_warn);
    g_settings.diode_beep   = (d[D_DIODE_BEEP] <= 1) ? (d[D_DIODE_BEEP] == 1) : defaults.diode_beep;
}

static void GenEncode(uint8_t *d) {
    memset(d, 0xFF, REC_DATA);
    d[G_WAVE]      = g_settings.gen_wave;
    d[G_FREQ]      = g_settings.gen_freq;
    d[G_DUTY]      = g_settings.gen_duty;
    d[G_VOLT]      = g_settings.gen_volt;
    d[G_AUTO_ON]   = g_settings.gen_auto_on ? 1 : 0;
    d[G_ON_CHANGE] = g_settings.gen_on_change ? 1 : 0;
    d[G_CAL]       = (uint8_t)(g_settings.gen_cal + 128);
}
static void GenDecode(const uint8_t *d) {
    g_settings.gen_wave      = Field(d[G_WAVE], GEN_WAVE_COUNT - 1, defaults.gen_wave);
    g_settings.gen_freq      = Field(d[G_FREQ], GEN_NUM_FREQ - 1,   defaults.gen_freq);
    g_settings.gen_duty      = Field(d[G_DUTY], GEN_NUM_DUTY - 1,   defaults.gen_duty);
    g_settings.gen_volt      = Field(d[G_VOLT], GEN_NUM_VOLT - 1,   defaults.gen_volt);
    g_settings.gen_auto_on   = (d[G_AUTO_ON] == 1);
    g_settings.gen_on_change = (d[G_ON_CHANGE] == 1);
    g_settings.gen_cal       = GenCalDecode(d[G_CAL]);
}

static void RngEncode(uint8_t *d) {
    memset(d, 0xFF, REC_DATA);
    d[R_SAVE_RANGE] = g_settings.save_range ? 1u : 0u;
    memcpy(&d[R_RANGE], g_saved_range, SETTINGS_RANGE_SLOTS);
    d[R_DEF_SUB]    = g_settings.def_sub;
    d[R_MINMAX]     = (uint8_t)~g_settings.minmax_pos;
}
static void RngDecode(const uint8_t *d) {
    g_settings.save_range = (d[R_SAVE_RANGE] == 1);
    memcpy(g_saved_range, &d[R_RANGE], SETTINGS_RANGE_SLOTS);
    g_settings.def_sub    = DefSubDecode(d[R_DEF_SUB]);
    g_settings.minmax_pos = (uint8_t)~d[R_MINMAX];
}

// ---------------------------------------------------------------------------
// Verejné funkcie
// ---------------------------------------------------------------------------
void Settings_Load(void) {
    uint8_t d[REC_DATA];
    g_settings = defaults;
    memset(g_saved_range, SETTINGS_RANGE_AUTO, sizeof(g_saved_range));   // prázdne = OFF + všetko AUTO
    if (DualLoad(&dev, d)) DevDecode(d);
    if (DualLoad(&gen, d)) GenDecode(d);
    if (DualLoad(&rng, d)) RngDecode(d);
}

bool Settings_Save(void) {
    uint8_t d[REC_DATA];
    bool ok = true;
    DevEncode(d); if (!DualSave(&dev, d)) ok = false;
    GenEncode(d); if (!DualSave(&gen, d)) ok = false;
    RngEncode(d); if (!DualSave(&rng, d)) ok = false;
    return ok;
}

// ---------------------------------------------------------------------------
// Posledné nastavenie generátora (stav výstupu ON/OFF sa neukladá)
// ---------------------------------------------------------------------------
void Settings_GenToRam(void) {
    g_gen.wave     = g_settings.gen_wave;
    g_gen.freq_idx = g_settings.gen_freq;
    g_gen.duty_idx = g_settings.gen_duty;
    g_gen.volt_idx = g_settings.gen_volt;
    if (g_gen.freq_idx >= Gen_FreqCount(g_gen.wave)) g_gen.freq_idx = Gen_FreqCount(g_gen.wave) - 1u;
}

bool Settings_GenSave(void) {
    g_settings.gen_wave = g_gen.wave;
    g_settings.gen_freq = g_gen.freq_idx;
    g_settings.gen_duty = g_gen.duty_idx;
    g_settings.gen_volt = g_gen.volt_idx;
    uint8_t d[REC_DATA];
    GenEncode(d);
    return DualSave(&gen, d);            // porovná s posledným záznamom, pri zhode nič nezapíše
}

// ---------------------------------------------------------------------------
// v0.72 migrácia: starý formát v0.71
// ---------------------------------------------------------------------------
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t seq;
    uint8_t  auto_off_min;
    uint8_t  auto_reset;
    uint8_t  probe_reset;
    uint8_t  gen_off_min;
    uint8_t  gen_wave;
    uint8_t  gen_freq;
    uint8_t  gen_duty;
    uint8_t  gen_volt;
    uint8_t  probe_c;
    uint8_t  probe_va;
    uint8_t  probe_vac;
    uint8_t  quick_rdc;
    uint8_t  def_sub;
    uint8_t  gen_auto_on;
    uint8_t  gen_on_change;
    uint8_t  gen_cal;
    uint8_t  long_press;
    uint8_t  beep;
    uint8_t  battery_cap;
    uint8_t  start_bl;
    uint32_t crc;
} RecV071_t;

_Static_assert(sizeof(RecV071_t) == 32, "RecV071_t musi mat 32 B");

// Platná kópia s vyšším seq zo starých adries, -1 = žiadna
static int V071Best(uint16_t a, uint16_t b, uint32_t magic, Rec_t *out) {
    Rec_t r[2];
    bool ok[2];
    uint16_t ad[2] = { a, b };
    for (int k = 0; k < 2; k++) {
        ok[k] = EEPROM_Read(ad[k], (uint8_t *)&r[k], sizeof(Rec_t)) && r[k].magic == magic &&
                r[k].version == 1u && r[k].crc == Crc32((const uint8_t *)&r[k], sizeof(Rec_t) - 4u);
    }
    int best = -1;
    if (ok[0] && ok[1]) best = ((int16_t)(r[1].seq - r[0].seq) > 0) ? 1 : 0;
    else if (ok[0])     best = 0;
    else if (ok[1])     best = 1;
    if (best >= 0) *out = r[best];
    return best;
}

void Settings_LoadV071(void) {
    g_settings = defaults;
    memset(g_saved_range, SETTINGS_RANGE_AUTO, sizeof(g_saved_range));

    Rec_t raw;
    if (V071Best(EE_V071_SETTINGS_A, EE_V071_SETTINGS_B, DEV_MAGIC, &raw) >= 0) {
        RecV071_t r;
        memcpy(&r, &raw, sizeof(r));
        if (r.auto_off_min >= SETTINGS_AUTO_OFF_MIN_MIN && r.auto_off_min <= SETTINGS_AUTO_OFF_MIN_MAX &&
            r.auto_reset <= 1 && r.probe_reset <= 1) {          // ako RecordValid v0.71
            g_settings.auto_off_min  = r.auto_off_min;
            g_settings.auto_reset    = r.auto_reset != 0;
            g_settings.probe_reset   = r.probe_reset != 0;
            g_settings.probe_c       = Field(r.probe_c,   SETTINGS_PROBE_C_MAX,   defaults.probe_c);
            g_settings.probe_va      = Field(r.probe_va,  SETTINGS_PROBE_VA_MAX,  defaults.probe_va);
            g_settings.probe_vac     = Field(r.probe_vac, SETTINGS_PROBE_VAC_MAX, defaults.probe_vac);
            g_settings.quick_rdc     = (r.quick_rdc == 1);
            g_settings.def_sub       = DefSubDecode(r.def_sub);
            g_settings.gen_auto_on   = (r.gen_auto_on == 1);
            g_settings.gen_on_change = (r.gen_on_change == 1);
            g_settings.long_press    = LongPressDecode(r.long_press);
            g_settings.beep          = (r.beep != 0);
            g_settings.battery_cap   = BatCapDecode(r.battery_cap);
            StartBlDecode(r.start_bl);
            g_settings.gen_cal       = GenCalDecode(r.gen_cal);
            g_settings.gen_off_min   = GenOffDecode(r.gen_off_min);
            g_settings.gen_wave      = Field(r.gen_wave, GEN_WAVE_COUNT - 1, defaults.gen_wave);
            g_settings.gen_freq      = Field(r.gen_freq, GEN_NUM_FREQ - 1,   defaults.gen_freq);
            g_settings.gen_duty      = Field(r.gen_duty, GEN_NUM_DUTY - 1,   defaults.gen_duty);
            g_settings.gen_volt      = Field(r.gen_volt, GEN_NUM_VOLT - 1,   defaults.gen_volt);
        }
    }
    // Rozsahy v0.70/71: [0] Save range, [1..16] rozsahy (rovnaké poradie ako v2, def_sub ostáva zo záznamu vyššie)
    if (V071Best(EE_V071_RANGES_A, EE_V071_RANGES_B, RNG_MAGIC, &raw) >= 0) {
        g_settings.save_range = (raw.d[R_SAVE_RANGE] == 1);
        memcpy(g_saved_range, &raw.d[R_RANGE], SETTINGS_RANGE_SLOTS);
    }
}

void Settings_EncodeNew(uint8_t dv[SETTINGS_REC_SIZE], uint8_t gn[SETTINGS_REC_SIZE], uint8_t rg[SETTINGS_REC_SIZE]) {
    uint8_t d[REC_DATA];
    Rec_t   rec;
    DevEncode(d); RecBuild(&rec, &dev, 1u, d); memcpy(dv, &rec, sizeof(rec));
    GenEncode(d); RecBuild(&rec, &gen, 1u, d); memcpy(gn, &rec, sizeof(rec));
    RngEncode(d); RecBuild(&rec, &rng, 1u, d); memcpy(rg, &rec, sizeof(rec));
}
