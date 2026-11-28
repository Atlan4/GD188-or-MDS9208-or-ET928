/*
 * settings.h
 *
 *  Užívateľské nastavenia uložené v I2C EEPROM (viď eeprom_map.h).
 *  Načítajú sa pri štarte (aj po zobudení zo spánku – to je reset).
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_

#include <stdint.h>
#include <stdbool.h>

#define SETTINGS_AUTO_OFF_MIN_MIN   1
#define SETTINGS_AUTO_OFF_MIN_MAX   99
#define SETTINGS_GEN_OFF_MAX        120     // generátor / osciloskop: 0 = OFF, krok 5 min
#define SETTINGS_GEN_OFF_STEP       5
#define SETTINGS_GEN_CAL_MAX        100     // +-10.0 % (krok 0.1 %)
#define SETTINGS_LONG_PRESS_MIN     4       // 400 ms
#define SETTINGS_LONG_PRESS_MAX     14      // 1400 ms
#define SETTINGS_BAT_CAP_MIN        15      // 1500 mAh (predvolené)
#define SETTINGS_BAT_CAP_DEF        15
#define SETTINGS_BAT_CAP_MAX        35      // 3500 mAh
#define SETTINGS_START_BL_MAX       2       // 0 = 25 %, 1 = 50 %, 2 = 75 %
#define SETTINGS_METER_MEM_MAX      3       // v0.72: počet pamätí hodnôt 0..3 = 24 / 48 / 72 / 96
#define SETTINGS_METER_MEM_STEP     24
#define SETTINGS_HV_WARN_MAX        2       // v0.74: HV warning 0 = OFF, 1 = ON (blesk), 2 = ON+BEEP
enum { HV_WARN_OFF = 0, HV_WARN_ON, HV_WARN_BEEP };

// Probe reset prahy (index 0 = OFF)
#define SETTINGS_PROBE_C_MAX        3       // OFF, 100, 200, 300 pF
#define SETTINGS_PROBE_C_STEP_PF    100
#define SETTINGS_PROBE_VA_MAX       5       // OFF, 10, 20, 30, 40, 50 digitov
#define SETTINGS_PROBE_VA_STEP      10
#define SETTINGS_PROBE_VAC_MAX      7       // OFF, 200, 400 ... 1400 digitov
#define SETTINGS_PROBE_VAC_STEP     200

typedef struct {
    uint8_t auto_off_min;   // čas do automatického vypnutia v minútach (1..99), predvolene 10
    bool    auto_reset;     // ON = tlačidlo / prepínač reštartuje odpočet; v0.78: OFF = nereštartuje ani Probe reset
    bool    probe_reset;    // ON = v Ω / dióde / pípaku prechod z OL na hodnotu reštartuje odpočet
    uint8_t probe_c;        // Cx: prah (index, 0 = OFF, inak x 100 pF)
    uint8_t probe_va;       // DC V, DC mV, DC A, AC A: prah v digitoch displeja (index, 0 = OFF, x 10)
    uint8_t probe_vac;      // AC V, AC mV: prah v digitoch displeja (index, 0 = OFF, x 200)
    bool    quick_rdc;      // ON = v polohe Ω / dióda / pípak volia F2–F4 režim priamo (ikony)
    uint8_t def_sub;        // predvolené režimy polôh (dlhé SAVE), bity viď SETTINGS_DEF_*
    uint8_t gen_off_min;    // auto vypnutie v generátore / osciloskope (0 = OFF, 5..120 po 5 min)
    // Posledné nastavenie generátora (indexy do zoznamov v gen.c); výstup ON/OFF sa neukladá
    uint8_t gen_wave;
    uint8_t gen_freq;
    uint8_t gen_duty;
    uint8_t gen_volt;
    // Nastavenia generátora (dlhé F1 na obrazovke generátora)
    bool    gen_auto_on;    // ON = pri otvorení generátora sa výstup zapne sám
    bool    gen_on_change;  // ON = pri zmene parametra sa výstup zapne sám
    int8_t  gen_cal;        // korekcia amplitúdy v 0.1 % (-100 .. +100 = -10.0 .. +10.0 %)
    uint8_t long_press;     // dĺžka dlhého stlačenia v stovkách ms (4..14 = 400..1400 ms)
    bool    beep;           // ON = pípnutie pri chybe (dlhé) a upozornení (krátke), pripomienka 5 min
    uint8_t battery_cap;    // kapacita batérie v stovkách mAh (20..35 = 2000..3500 mAh)
    uint8_t start_bl;       // jas pri tvrdom štarte: 0 = MIN (25 %), 1 = 50 %, 2 = MAX (75 %)
    bool    hide_zeros;     // v0.66: ON = na veľkom displeji multimetra sa nekreslia nuly zľava (000.0 -> 0.0)
    bool    save_range;     // v0.70: ON = dlhé SAVE ukladá aj rozsah a po zapnutí sa uložené rozsahy načítajú
    uint8_t meter_mem;      // v0.72: počet pamätí nameraných hodnôt (index 0..3 = 24 / 48 / 72 / 96)
    uint8_t hv_warn;        // v0.74: výstraha nebezpečného napätia v polohe V (HV_WARN_*), predvolene ON+BEEP
    bool    diode_beep;     // v0.78: ON = v polohe dióda nepretržité pípanie pri úbytku < 0.05 V (skrat), predvolene ON
    uint8_t minmax_pos;     // v0.76: bit = poloha prepínača (SwitchPosition_t), MIN/MAX zapnuté po zapnutí / otočení (dlhé SAVE)
} AppSettings_t;

extern AppSettings_t g_settings;

// v0.70: uložené rozsahy multimetra (dlhé SAVE pri Save range ON), samostatný záznam v EEPROM
// (viď eeprom_map.h, EE_RANGES_*). Index = kombinácia poloha + režim (mapovanie v main.c),
// hodnota = číslo rozsahu čipu alebo 0xFF = AUTO (aj prázdna EEPROM). Pri Save range OFF sa
// nemažú – iba sa nepoužívajú.
#define SETTINGS_RANGE_SLOTS  16
#define SETTINGS_RANGE_AUTO   0xFFu
extern uint8_t g_saved_range[SETTINGS_RANGE_SLOTS];

// v0.72: tri samostatné záznamy (každý kópia A / B, viď eeprom_map.h):
//   nastavenia prístroja (EE_SETTINGS_*), Save range + rozsahy + def_sub (EE_RANGES_*),
//   nastavenia generátora (EE_GEN_*). Settings_Save zapíše iba tie, ktoré sa zmenili.
void Settings_Load(void);    // pri neplatných dátach nastaví predvolené hodnoty
bool Settings_Save(void);    // zapíše iba ak sa líšia od uložených; true = OK

// v0.72 migrácia (ee_migrate.c):
//   Settings_LoadV071   – načíta nastavenia a rozsahy v starom formáte v0.71 do g_settings / g_saved_range
//   Settings_EncodeNew  – zakóduje g_settings / g_saved_range do nových záznamov (3 x 32 B, poradie
//                         prístroj, generátor, rozsahy), počítadlo 1, s CRC
#define SETTINGS_REC_SIZE  32u
void Settings_LoadV071(void);
void Settings_EncodeNew(uint8_t dev[SETTINGS_REC_SIZE], uint8_t gen[SETTINGS_REC_SIZE], uint8_t rng[SETTINGS_REC_SIZE]);

// Počet pamätí nameraných hodnôt (24 / 48 / 72 / 96)
static inline uint8_t Settings_MeterMemCount(void) {
    return (uint8_t)(SETTINGS_METER_MEM_STEP * (1u + (g_settings.meter_mem % (SETTINGS_METER_MEM_MAX + 1u))));
}

// Predvolené režimy polôh v g_settings.def_sub (dlhé SAVE na meracej obrazovke):
//   bit 0 V, 1 mV, 2 µA, 3 mA, 4 10A: 0 = DC, 1 = AC
//   bit 5 Hz/%: 0 = Hz, 1 = DUTY
//   bity 6–7 Ω: 0 = odpor, 1 = dióda, 2 = pípak
#define SETTINGS_DEF_V      0
#define SETTINGS_DEF_mV     1
#define SETTINGS_DEF_uA     2
#define SETTINGS_DEF_mA     3
#define SETTINGS_DEF_10A    4
#define SETTINGS_DEF_HZ     5
#define SETTINGS_DEF_OHM    6       // 2 bity

// Dĺžka dlhého stlačenia (F1, BL, AUTO, SAVE) v ms
static inline uint32_t Settings_LongPressMs(void) { return 100u * g_settings.long_press; }

// Jas podsvietenia v % pre index 0..2 (MIN / 50 % / MAX)
static inline uint8_t Settings_BlPercent(uint8_t i) { return (i == 0) ? 25u : (i == 1) ? 50u : 75u; }

// Prahy Probe reset z indexov (0 = vypnuté)
static inline uint32_t Settings_ProbeCPf(uint8_t i)      { return (uint32_t)i * SETTINGS_PROBE_C_STEP_PF; }
static inline uint32_t Settings_ProbeVaCounts(uint8_t i)  { return (uint32_t)i * SETTINGS_PROBE_VA_STEP; }
static inline uint32_t Settings_ProbeVacCounts(uint8_t i) { return (uint32_t)i * SETTINGS_PROBE_VAC_STEP; }

// "Posledné nastavenie" generátora:
//   Settings_GenToRam – g_settings -> g_gen (pri štarte prístroja)
//   Settings_GenSave  – g_gen -> g_settings + zápis do EEPROM (dlhé SAVE v generátore).
//                       Ak sa nič nezmenilo, nezapisuje sa. true = OK
void Settings_GenToRam(void);
bool Settings_GenSave(void);

#endif /* SETTINGS_H_ */
