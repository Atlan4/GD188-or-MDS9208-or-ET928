/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stddef.h>
#include <stdio.h>

#include "lcd.h"   
#include "meas_chip.h"
#include "rotary_switch.h"
#include "button.h"
#include "settings.h"
#include "eeprom.h"
#include "battery.h"
#include "menu.h"
#include "gen.h"
#include "gen_menu.h"
#include "scope_test.h"
#include "scope.h"
#include "beep.h"
#include "meter_mem.h"
#include "ee_migrate.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Ladiaca obrazovka: 1 = stavový riadok (STATUS: ...) a RAW sniffer bajtov z čipu,
//                    0 = bežná prevádzka (nadpis veličiny hore)
#define DEBUG_SCREEN  0

// Pri Cx malý riadok so surovými dátami z čipu (hodnota a bajt rozsahu) – na overenie
// rozsahov kapacity. 1 = zobraziť, 0 = vypnuté.
#define CX_RAW_INFO   0

// --- Automatické vypnutie ---
// Čas sa nastavuje v menu (g_settings.auto_off_min). Odpočet reštartuje tlačidlo / prepínač
// (ak je zapnuté "Auto reset time") a prechod z OL na hodnotu v Ω / dióde / pípaku
// (ak je zapnuté "Probe reset").
// V generátore (a neskôr osciloskope) platí samostatný čas g_settings.gen_off_min, 0 = OFF.
static uint32_t last_activity_tick = 0;
static bool     auto_off_done = false;

// Dlhé stlačenie F1 otvára menu, krátke = REL (vykoná sa pri pustení)
static bool     f1_pending = false;
static uint32_t f1_tick = 0;
static bool     menu_open = false;

// V menu odpočet automatického vypnutia stojí. Po MENU_IDLE_MS bez dotyku menu
// samo skončí (s uložením, ako EXIT) a začne bežný odpočet.
#define MENU_IDLE_MS      (2u * 60u * 1000u)
static uint32_t menu_input_tick = 0;

// Tlačidlo podsvietenia: krátke stlačenie = jas (pri pustení), dlhé = generátor (iba v polohe V)
static bool     bl_pending = false;
static uint32_t bl_tick = 0;
static bool     gen_open = false;          // obrazovka generátora
static bool     scope_open = false;        // osciloskop (dlhé AUTO v polohe V / mV)
static bool     mem_open = false;          // v0.71: pamäť nameraných hodnôt (krátke SAVE na meracej obrazovke)
static bool     scope_test = false;        // v osciloskope beží testovací režim (dlhé F1)
static bool     gen_from_scope = false;    // generátor otvorený z osciloskopu -> dlhé BL vráti do osciloskopu
static bool     cal_forced = false;        // kalibrácia generátora prepla polohu V na AC
static SubMode_t cal_saved_sub;
static bool     scal_forced = false;       // kalibrácia osciloskopu prepla polohu V na DC
static SubMode_t scal_saved_sub;
static uint32_t gen_hint_until = 0;        // do tohto času nadpis hint_text (GEN / SCOPE: SWITCH TO V)
static const char *hint_text = "GEN: SWITCH TO V";
static bool     bat_low = false;            // napätie článku pod 3.30 V (symbol batérie bliká)
static uint32_t bat_msg_until = 0;          // do tohto času nadpis BATTERY LOW
// v0.71: krátke BL -> v hornom riadku BRIGHTNESS MIN / 50% / MAX na 1.5 s od posledného stlačenia (všade)
#define BL_MSG_MS       1500u
static uint32_t bl_msg_until = 0;
static bool     auto_pending = false;      // AUTO na meracej obrazovke: krátke = AUTO rozsah, dlhé = osciloskop
static uint32_t auto_tick = 0;
// Obrazovky s rámikom: horný riadok (čas, nadpis, batéria) na y = 8 – v strede medzi čiarami y = 0 a y = 30
#define FRAME_SCREEN()  (gen_open || menu_open || scope_open || mem_open)
#define TOP_Y()         (FRAME_SCREEN() ? 7 : 10)            // v0.62: čiara y = 27, text 6 / 6 px
#define GEN_HINT_MS       2000
static bool     gen_resume = false;        // výstup bol zapnutý pri odchode z polohy V -> po návrate zapnúť
static uint8_t  bl_level = 50;             // aktuálny jas podsvietenia v % (25 / 50 / 75)
static bool     pos_known = false;         // poloha prepínača už bola raz prečítaná
static uint32_t gen_beep_tick = 0;         // pri vypnutom auto-vypnutí pípnutie každých 5 min
#define GEN_BEEP_PERIOD_MS  (5u * 60u * 1000u)

// Odpočet pre aktuálny režim v sekundách, 0 = automatické vypnutie zakázané
static uint32_t AutoOffSeconds(void) {
    if (gen_open || scope_open) return (uint32_t)g_settings.gen_off_min * 60u;
    return (uint32_t)g_settings.auto_off_min * 60u;
}

// Nastavenie generátora ("posledné nastavenie" v EEPROM):
//   - načíta sa iba pri štarte prístroja (Settings_GenToRam),
//   - uloží sa dlhým SAVE v generátore (gen_menu.c, Settings_GenSave – iba ak sa líši od uloženého).
//     Pred spánkom sa do EEPROM neukladá; po zobudení sa obnoví z RAM (Resume_*).
//   Medzitým platí nastavenie v RAM (g_gen) – aj po odchode z generátora a návrate.

// Pípanie: beep.c (na pozadí, bez čakania). Bzučiak je aktívny (log. 1 na PB10 = pípa).

// Zvýši sa pri návrate z menu -> všetky prvky obrazovky sa prekreslia
static uint32_t screen_epoch = 1;
// v0.66: dlhé časované hlásenie (nad TOP_TXT_MAX znakov) zaberá celý horný riadok – časomiera a batéria
// sa vtedy nekreslia. Platí iba v epoche, v ktorej sa zobrazilo (nová obrazovka ho zruší).
static bool     top_wide = false;
static uint32_t top_wide_epoch = 0;
#define TOP_WIDE()  (top_wide && top_wide_epoch == screen_epoch)
static uint32_t mode_epoch   = 1;      // zvýši sa pri každej zmene módu (pre Probe reset)

static void NoteActivity(void) {
    if (g_settings.auto_reset) last_activity_tick = HAL_GetTick();
}

// Dlhé SAVE na meracej obrazovke = aktuálny režim polohy ako predvolený (EEPROM, viď settings.h).
// Potvrdenie na 1 s v hornom riadku. v0.71: krátke SAVE = pamäť nameraných hodnôt (meter_mem.c).
#define DEF_MSG_MS          1000
static bool     save_pending = false;
static bool     f2_pending = false;        // v0.76: F2 = HOLD (krátke) / MIN/MAX (dlhé)
static uint32_t f2_tick;
static uint32_t save_tick = 0;
static uint32_t def_msg_until = 0;         // do tohto času "DEFAULT SAVED" / "SAVE ERROR"
static bool     def_msg_ok = true;


uint8_t rx_byte;

#if DEBUG_SCREEN
// --- Debug sniffer (posledných 16 bajtov z čipu) – zapisuje RX prerušenie ---
static volatile uint8_t  debug_rx_buffer[16];
static volatile uint8_t  debug_rx_index = 0;
static volatile uint32_t debug_rx_count = 0;
#endif

// HOLD – číta ho aj LCD_DrawBottomMenu() v lcd.c
volatile bool is_hold_active = false;

// --- Stav prístroja (iba hlavná slučka, ISR sa ich nedotýka) ---
static SwitchPosition_t cur_pos      = SWITCH_POS_V;
static SubMode_t        cur_sub      = SUB_MODE_DC;
static bool             menu_dirty   = true;
static int8_t           pressed_cell = -1;          // políčko menu pod držaným tlačidlom
static ButtonType_t     pending_btn  = BTN_NONE;     // RNG stlačené počas inicializácie čipu

// --- Pamäť nastavení v RAM (platí do vypnutia prístroja) ---
// Pre každú polohu prepínača posledný sub-mód (SEL, Hz/%),
// pre každú kombináciu poloha + sub-mód posledný rozsah (AUTO alebo manuálny rozsah).
#define NUM_POSITIONS  8
#define NUM_SUBMODES   8
static SubMode_t pos_sub[NUM_POSITIONS];
static uint8_t   pos_range[NUM_POSITIONS][NUM_SUBMODES];   // MEAS_RANGE_AUTO alebo číslo rozsahu
static bool             display_dirty = true;   // vynúti prekreslenie jednotiek, bodky aj číslic

// --- Rozsahy ---
// decimals = počet desatinných miest pri 4-miestnom zobrazení (0 = bez bodky)
// exp      = 1 digit tohto rozsahu = 10^exp základných jednotiek polohy.
//            Slúži pre REL: offset sa ukladá v základných jednotkách, takže platí
//            aj keď auto-rozsah medzitým prepne na iný rozsah.
// ol_above = surová hodnota, nad ktorou sa zobrazí OL (0 = bez limitu)
typedef struct {
    uint8_t     decimals;
    uint8_t     exp;
    const char *label;
    const char *unit;
    int32_t     ol_above;
} RangeInfo_t;

// V: základ 0.1 mV
// 1000 V: najvyšší rozsah, výrobca udáva max. 1000 V -> nad 1000 digitov OL (aj v AUTO)
static const RangeInfo_t RANGES_V[]   = { {1, 0, "600.0", "mV"}, {3, 1, "6.000", "V"}, {2, 2, "60.00", "V"},
                                          {1, 3, "600.0", "V"},  {0, 4, "1000",  "V", 1000} };
// mV: základ 0.01 mV
static const RangeInfo_t RANGES_mV[]  = { {2, 0, "60.00", "mV"}, {1, 1, "600.0", "mV"} };
// uA: základ 0.1 uA
static const RangeInfo_t RANGES_uA[]  = { {1, 0, "600.0", "uA"}, {0, 1, "6000",  "uA"} };
// mA: základ 0.01 mA
static const RangeInfo_t RANGES_mA[]  = { {2, 0, "60.00", "mA"}, {1, 1, "600.0", "mA"} };
// 10A: základ 1 mA
// 10.00 A: najvyšší rozsah, max. 10 A -> nad 1000 digitov OL (aj v AUTO)
static const RangeInfo_t RANGES_10A[] = { {3, 0, "6.000", "A"},  {2, 1, "10.00", "A", 1000} };

// Odpor (58 07): overené na prístroji, 00 = 600 Ω ... 05 = 60 MΩ. "\x7f" = znak Ω vo fonte.
// Základ 0.1 Ω
static const RangeInfo_t RANGES_OHM[] = { {1, 0, "600.0", "\x7f"},  {3, 1, "6.000", "k\x7f"},
                                          {2, 2, "60.00", "k\x7f"}, {1, 3, "600.0", "k\x7f"},
                                          {3, 4, "6.000", "M\x7f"}, {2, 5, "60.00", "M\x7f", 6000} };

// Dióda (58 0A): napätie x.xxx V, naprázdno čip ukazuje ~3237 -> nad 3.200 V = OL
static const RangeInfo_t RANGES_DIODE[] = { {3, 0, "", "V", 3200} };

// Pípak (58 09): iba AUTO 600.0 Ω, nad 600 Ω (rozpojené hroty) = OL
static const RangeInfo_t RANGES_CONT[]  = { {1, 0, "600.0", "\x7f", 6000} };

// Kapacita (58 0B): iba AUTO, základ 1 pF.
// Bajt rozsahu z čipu určuje váhu jedného digitu (overené na prístroji):
//   range 0: 1 pF   (100 nF -> 103325)
//   range 1: 1 nF   (4.7 uF -> 5062)
//   range 2: 10 nF  (210 uF -> 20749)
//   range 3: 1 uF   (3300 uF -> 3357)   – skok o 100x, nie o 10x
//   range 4: 10 uF  (podľa špecifikácie výrobcu, neoverené meraním)
// Tabuľka RANGES_CX_CHIP sa používa len na prepočet hodnoty z čipu na pF.
static const RangeInfo_t RANGES_CX_CHIP[] = { {3, 0, "", "nF"}, {3, 3, "", "uF"},
                                              {2, 4, "", "uF"}, {0, 6, "", "uF"},
                                              {2, 7, "", "mF"} };

// Zobrazovacie rozsahy kapacity – 9999 digitov podľa výrobcu (99.99 nF ... 99.99 mF),
// navyše rozsah "9999 uF", aby sa do 9999 uF ukazovalo v uF (čip má tam krok 1 uF,
// takže sa nič nestráca). exp = 1 digit tohto rozsahu = 10^exp pF.
static const RangeInfo_t RANGES_CAP[] = { {3, 0, "9.999", "nF"},     // jemný rozsah 1 pF (nad rámec výrobcu)
                                          {2, 1, "99.99", "nF"}, {1, 2, "999.9", "nF"},
                                          {3, 3, "9.999", "uF"}, {2, 4, "99.99", "uF"},
                                          {1, 5, "999.9", "uF"}, {0, 6, "9999",  "uF"},
                                          {2, 7, "99.99", "mF"} };

// Manuálny rozsah V / A / Ω: čip meria aj nad rozsahom (do ~10 000 digitov, napr. 9 V na rozsahu
// 6.000 V = 9.445), výrobca však udáva 6000 digitov -> nad touto hodnotou OL
#define MANUAL_OL_COUNTS  6000

// Hysteréza prepínania rozsahov Cx: nahor hneď keď hodnota prekročí 9999 digitov,
// nadol až keď sa do nižšieho rozsahu zmestí s rezervou (pod 9000 digitov nižšieho rozsahu).
#define CX_DOWN_COUNTS  9000

// Probe reset s prahom (Cx, V/A, V-AC – viď ProbeLevel_Task): prah z menu nastavení,
// opätovné "nabitie" až po poklese pod 70 % prahu, potvrdenie viacerými meraniami po sebe
#define PROBE_REARM_PCT      70
#define PROBE_CONFIRM         2
static uint8_t cx_disp_range = 0;          // aktuálny zobrazovací rozsah Cx (reset pri zmene módu)


// Formát jednej vzorky pre aktuálnu polohu
typedef struct {
    int8_t      decimals;
    uint8_t     exp;
    const char *label;
    const char *unit;
    const char *acdc;
    uint8_t     icon;        // ikona namiesto AC/DC (ICON_DIODE, ICON_BEEP)
    int32_t     ol_above;
    char        label_buf[8];
} Format_t;

// --- REL (relatívne meranie) ---
static bool         rel_active = false;
static int64_t      rel_offset = 0;          // v základných jednotkách polohy

// REL pre kapacitu sa pamätá aj po otočení prepínača preč a späť (parazitná kapacita káblov).
// Odčítava sa iba v polohe Cx, v ostatných polohách je REL vypnuté.
static bool         cx_rel_active = false;
static int64_t      cx_rel_offset = 0;

// Text korekcie zobrazený nad tlačidlom REL, napr. "-0.050 nF"
static char         rel_text[32] = "";
static char         cx_rel_text[32] = "";
static MeasSample_t last_sample;             // posledná platná vzorka (pre zachytenie REL)
static bool         last_sample_ok = false;
static bool         redraw_last = false;     // prekresliť last_sample (po zapnutí/vypnutí REL)
// v0.71: hodnota práve zobrazená na veľkom displeji (pri HOLD zmrazená) – ukladá sa do pamäte hodnôt
static MeterMemVal_t cur_disp;

// v0.76: MIN/MAX na meracej obrazovke (dlhé HOLD = F2). Sleduje sa zobrazená hodnota (pri HOLD stojí),
// porovnáva sa v základných jednotkách (rozsah AUTO môže meniť desatinné miesta a predponu jednotky).
// Uložená je celá zobrazená hodnota (s jednotkou), ako bola na displeji pri dosiahnutí MAX / MIN.
static bool          mm_on = false;
static MeterMemVal_t mm_max, mm_min;
static float         mm_vmax, mm_vmin;
static uint32_t      mm_epoch = 1;              // zmena -> Display_MinMax prekreslí
static int8_t        mm_pos_last = -1;          // poloha pri poslednom ApplyMode

static float MemValBase(const MeterMemVal_t *v) {
    float x = (float)v->mag;
    for (uint8_t i = 0; i < v->decimals; i++) x /= 10.0f;
    if (v->unit[0] != '\0' && v->unit[1] != '\0') {
        switch (v->unit[0]) {
            case 'n': x *= 1e-9f; break;
            case 'u': x *= 1e-6f; break;
            case 'm': x *= 1e-3f; break;
            case 'k': x *= 1e3f;  break;
            case 'M': x *= 1e6f;  break;
            default: break;
        }
    }
    return v->neg ? -x : x;
}

static void MinMax_Reset(void) {
    mm_max.used = false;
    mm_min.used = false;
    mm_epoch++;
}

static void MinMax_Update(const MeterMemVal_t *v) {
    if (!mm_on || !v->used) return;
    if (v->ol) {                                     // OL = nad každou hodnotou -> MAX OL
        if (!mm_max.used || !mm_max.ol) { mm_max = *v; mm_epoch++; }
        return;
    }
    float x = MemValBase(v);
    if (!mm_max.used || (!mm_max.ol && x > mm_vmax)) { mm_max = *v; mm_vmax = x; mm_epoch++; }
    if (!mm_min.used || x < mm_vmin)                 { mm_min = *v; mm_vmin = x; mm_epoch++; }
}

static bool MinMax_Saved(int pos) {
    return pos >= 0 && pos < 8 && ((g_settings.minmax_pos >> pos) & 1u);
}

#define COUNT_OF(a) (sizeof(a) / sizeof((a)[0]))

// --- Rozloženie meracej obrazovky (výškovo) ---
// Horný riadok "AUTO ..." je na y = 45..60, riadok REL korekcie na y = 188..203.
// Veľké číslice (72 px) sú v strede medzi nimi: (61 + 188) / 2 - 36 = 88.
#define DIGITS_Y   88
#define ACDC_Y     (DIGITS_Y + 5)     // AC / DC / ikona vpravo hore pri číslach
#define UNIT_Y     (DIGITS_Y + 55)    // jednotka vpravo dole pri číslach

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Rozsahová tabuľka pre polohu a sub-mód (NULL = škálovanie nie je známe)
static const RangeInfo_t *GetRangeTable(SwitchPosition_t pos, SubMode_t sub, uint8_t *count) {
    switch (pos) {
        case SWITCH_POS_OHM_DIODE_CONT:
            if (sub == SUB_MODE_RESISTANCE) { *count = COUNT_OF(RANGES_OHM);   return RANGES_OHM; }
            if (sub == SUB_MODE_DIODE)      { *count = COUNT_OF(RANGES_DIODE); return RANGES_DIODE; }
            if (sub == SUB_MODE_CONTINUITY) { *count = COUNT_OF(RANGES_CONT);  return RANGES_CONT; }
            *count = 0; return NULL;
        case SWITCH_POS_Cx:  *count = COUNT_OF(RANGES_CX_CHIP); return RANGES_CX_CHIP;
        case SWITCH_POS_V:   *count = COUNT_OF(RANGES_V);   return RANGES_V;
        case SWITCH_POS_mV:  *count = COUNT_OF(RANGES_mV);  return RANGES_mV;
        case SWITCH_POS_uA:  *count = COUNT_OF(RANGES_uA);  return RANGES_uA;
        case SWITCH_POS_mA:  *count = COUNT_OF(RANGES_mA);  return RANGES_mA;
        case SWITCH_POS_10A: *count = COUNT_OF(RANGES_10A); return RANGES_10A;
        default:             *count = 0;                    return NULL;
    }
}

// Poloha/sub-mód, kde má zmysel tlačidlo RNG (manuálny rozsah)
static bool HasRangeButton(SwitchPosition_t pos, SubMode_t sub) {
    return pos == SWITCH_POS_V  || pos == SWITCH_POS_mV || pos == SWITCH_POS_uA ||
           pos == SWITCH_POS_mA || pos == SWITCH_POS_10A ||
           (pos == SWITCH_POS_OHM_DIODE_CONT && sub == SUB_MODE_RESISTANCE);
}

// Polohy, kde sa zobrazuje AC/DC
static bool IsVoltCurrentPos(SwitchPosition_t pos) {
    return pos == SWITCH_POS_V  || pos == SWITCH_POS_mV || pos == SWITCH_POS_uA ||
           pos == SWITCH_POS_mA || pos == SWITCH_POS_10A;
}

static int64_t Pow10(uint8_t e) {
    int64_t p = 1;
    while (e--) p *= 10;
    return p;
}

// Delenie so zaokrúhlením na najbližšie celé číslo (aj pre záporné hodnoty)
static int64_t DivRound(int64_t n, int64_t d) {
    return (n >= 0) ? (n + d / 2) / d : -((-n + d / 2) / d);
}

// Výber zobrazovacieho rozsahu pre hodnotu v pF (s hysterézou okolo *state).
// Vráti false, ak je hodnota nad najvyšším rozsahom (OL).
static bool CxSelectRange(int64_t pf, uint8_t *state) {
    int64_t apf = (pf < 0) ? -pf : pf;
    uint8_t r = *state;
    if (r >= COUNT_OF(RANGES_CAP)) r = 0;
    while (r + 1u < COUNT_OF(RANGES_CAP) && DivRound(apf, Pow10(RANGES_CAP[r].exp)) > 9999) r++;
    while (r > 0 && apf < (int64_t)CX_DOWN_COUNTS * Pow10(RANGES_CAP[r - 1].exp)) r--;
    *state = r;
    return DivRound(apf, Pow10(RANGES_CAP[r].exp)) <= 9999;
}

static bool IsAcMode(uint8_t cmd) {
    return cmd == CMD_CHIP_6V_AC || cmd == CMD_CHIP_600mV_AC || cmd == CMD_CHIP_uA_AC ||
           cmd == CMD_CHIP_mA_AC || cmd == CMD_CHIP_10A_AC;
}

// Uložený rozsah pre aktuálnu polohu a sub-mód
static uint8_t *CurRangeMem(void) {
    return &pos_range[cur_pos % NUM_POSITIONS][cur_sub % NUM_SUBMODES];
}

static bool IsManual(void) {
    return *CurRangeMem() != MEAS_RANGE_AUTO;
}

static void Settings_Init(void) {
    for (int p = 0; p < NUM_POSITIONS; p++) {
        pos_sub[p] = ROTARY_GetDefaultSubMode((SwitchPosition_t)p);
        for (int m = 0; m < NUM_SUBMODES; m++) {
            pos_range[p][m] = MEAS_RANGE_AUTO;
        }
    }
}

// Bit predvoleného režimu pre polohu (0xFF = poloha bez voľby, napr. Cx)
static uint8_t DefBit(SwitchPosition_t pos) {
    switch (pos) {
        case SWITCH_POS_V:      return SETTINGS_DEF_V;
        case SWITCH_POS_mV:     return SETTINGS_DEF_mV;
        case SWITCH_POS_uA:     return SETTINGS_DEF_uA;
        case SWITCH_POS_mA:     return SETTINGS_DEF_mA;
        case SWITCH_POS_10A:    return SETTINGS_DEF_10A;
        case SWITCH_POS_Hz_Pct: return SETTINGS_DEF_HZ;
        case SWITCH_POS_OHM_DIODE_CONT: return SETTINGS_DEF_OHM;
        default:                return 0xFF;
    }
}

// Predvolený režim polohy podľa uloženého nastavenia (volať po Settings_Load)
static SubMode_t DefaultSub(SwitchPosition_t pos) {
    uint8_t b = DefBit(pos);
    if (b == 0xFF) return ROTARY_GetDefaultSubMode(pos);
    uint8_t ds = g_settings.def_sub;
    if (pos == SWITCH_POS_OHM_DIODE_CONT) {
        uint8_t v = (ds >> b) & 3u;
        return (v == 1u) ? SUB_MODE_DIODE : (v == 2u) ? SUB_MODE_CONTINUITY : SUB_MODE_RESISTANCE;
    }
    bool set = (ds >> b) & 1u;
    if (pos == SWITCH_POS_Hz_Pct) return set ? SUB_MODE_DUTY : SUB_MODE_FREQ;
    return set ? SUB_MODE_AC : SUB_MODE_DC;
}

static void ApplyDefaultSubs(void) {
    for (int p = 0; p < NUM_POSITIONS; p++) pos_sub[p] = DefaultSub((SwitchPosition_t)p);
}

// v0.70: index v g_saved_range pre kombináciu poloha + režim s voľbou rozsahu (RNG),
// 0xFF = bez rozsahu (dióda, pípak, Cx, Hz / %).
static uint8_t RangeSlot(SwitchPosition_t pos, SubMode_t sub) {
    if (!HasRangeButton(pos, sub)) return 0xFF;
    uint8_t ac = (sub == SUB_MODE_AC) ? 1u : 0u;
    switch (pos) {
        case SWITCH_POS_V:   return (uint8_t)(0u + ac);
        case SWITCH_POS_mV:  return (uint8_t)(2u + ac);
        case SWITCH_POS_uA:  return (uint8_t)(4u + ac);
        case SWITCH_POS_mA:  return (uint8_t)(6u + ac);
        case SWITCH_POS_10A: return (uint8_t)(8u + ac);
        case SWITCH_POS_OHM_DIODE_CONT: return 10u;      // iba odpor
        default:             return 0xFF;
    }
}

// v0.70: po zapnutí prepínačom (Save range ON) uložené rozsahy do pamäte rozsahov v RAM.
// Pri otočení prepínača sa potom použije rozsah z RAM (uložený, alebo posledný zvolený).
static void ApplySavedRanges(void) {
    if (!g_settings.save_range) return;                  // OFF -> všetko AUTO (uložené ostávajú v EEPROM)
    for (int p = 0; p < NUM_POSITIONS; p++) {
        for (int m = 0; m < NUM_SUBMODES; m++) {
            uint8_t sl = RangeSlot((SwitchPosition_t)p, (SubMode_t)m);
            if (sl < SETTINGS_RANGE_SLOTS) pos_range[p][m] = g_saved_range[sl];
        }
    }
}

// Uloží aktuálny režim polohy ako predvolený. Cx (bez voľby) -> nič, bez správy.
// v0.70: pri Save range ON aj aktuálny rozsah (AUTO / manuálny) tejto polohy a režimu.
static void SaveDefSubBits(uint8_t b);
// v0.76: aj MIN/MAX zap. / vyp. pre túto polohu (všetky polohy vrátane Cx)
static void SaveDefaultSub(void) {
    uint8_t b = DefBit(cur_pos);
    if ((unsigned)cur_pos < 8u) {
        uint8_t m = (uint8_t)(1u << (unsigned)cur_pos);
        g_settings.minmax_pos = mm_on ? (uint8_t)(g_settings.minmax_pos | m) : (uint8_t)(g_settings.minmax_pos & ~m);
    } else if (b == 0xFF) {
        return;
    }
    if (b != 0xFF) SaveDefSubBits(b);
    if (b != 0xFF && g_settings.save_range) {
        uint8_t sl = RangeSlot(cur_pos, cur_sub);
        if (sl < SETTINGS_RANGE_SLOTS) g_saved_range[sl] = *CurRangeMem();
    }
    def_msg_ok    = Settings_Save();             // zapíše iba pri zmene
    def_msg_until = HAL_GetTick() + DEF_MSG_MS;
    if (!def_msg_ok) Beep_Error();                 // SAVE ERROR
}

static void SaveDefSubBits(uint8_t b) {
    uint8_t ds = g_settings.def_sub;
    if (cur_pos == SWITCH_POS_OHM_DIODE_CONT) {
        uint8_t v = (cur_sub == SUB_MODE_DIODE) ? 1u : (cur_sub == SUB_MODE_CONTINUITY) ? 2u : 0u;
        ds = (uint8_t)((ds & ~(3u << b)) | (v << b));
    } else {
        bool set = (cur_sub == SUB_MODE_AC || cur_sub == SUB_MODE_DUTY);
        ds = (uint8_t)(set ? (ds | (1u << b)) : (ds & ~(1u << b)));
    }
    g_settings.def_sub = ds;
}

// Odošle čipu mód podľa aktuálnej polohy a sub-módu vrátane uloženého rozsahu.
// Dáta, ktoré prídu medzitým, sa zahodia (vzorky sa berú iba v stave READY).
static void ApplyMode(void) {
    uint8_t cmd = ROTARY_GetChipCommand(cur_pos, cur_sub);
    MeasChip_SetMode(cmd, ROTARY_GetQuery(cur_pos, cur_sub), *CurRangeMem());
    pending_btn   = BTN_NONE;
    mode_epoch++;
    cx_disp_range = 0;
    if (mm_pos_last != (int8_t)cur_pos) {            // v0.76: iná poloha -> MIN/MAX podľa dlhého SAVE
        mm_pos_last = (int8_t)cur_pos;
        mm_on = MinMax_Saved((int)cur_pos);
    }
    MinMax_Reset();                                  // nová veličina -> MIN/MAX od začiatku
    if (cur_pos == SWITCH_POS_Cx) {
        rel_active = cx_rel_active;   // Cx: obnoviť REL, ak ho užívateľ zapol
        rel_offset = cx_rel_offset;
        strcpy(rel_text, cx_rel_text);
    } else {
        rel_active = false;           // offset patrí k meranej veličine -> pri zmene sa ruší
    }
    last_sample_ok = false;
    cur_disp.used  = false;           // v0.71: nová veličina -> zatiaľ žiadna hodnota na uloženie
    display_dirty = true;
    menu_dirty    = true;
}

// Zmena sub-módu užívateľom: zapamätá sa pre túto polohu a pošle do čipu
static void SelectSubMode(SubMode_t sub) {
    if (sub == cur_sub) return;
    cur_sub = sub;
    pos_sub[cur_pos % NUM_POSITIONS] = sub;
    ApplyMode();
}

// Určí formát vzorky (bodka, popis, jednotka, exponent) pre aktuálnu polohu.
// Vráti false, ak vzorka do aktuálneho módu nepatrí alebo má neplatný rozsah.
static bool GetFormat(const MeasSample_t *s, Format_t *f) {
    uint8_t count;
    const RangeInfo_t *tbl = GetRangeTable(cur_pos, cur_sub, &count);

    f->decimals = 0;
    f->exp      = 0;
    f->label    = "";
    f->unit     = "  ";
    f->acdc     = "  ";
    f->icon     = ICON_NONE;
    f->ol_above = 0;

    if (tbl != NULL) {
        if (s->packet_type != MEAS_QUERY_MAIN) return false;
        if (s->range >= count) return false;      // neplatný / prechodný rozsah (pôvodný mV filter)
        f->decimals = tbl[s->range].decimals;
        f->exp      = tbl[s->range].exp;
        f->label    = tbl[s->range].label;
        f->unit     = tbl[s->range].unit;
        f->ol_above = tbl[s->range].ol_above;
        // Manuálny rozsah: OL nad 6000 digitov (ak tabuľka nemá nižšiu hranicu, napr. 1000 V)
        if (IsManual() && (IsVoltCurrentPos(cur_pos) ||
                           (cur_pos == SWITCH_POS_OHM_DIODE_CONT && cur_sub == SUB_MODE_RESISTANCE))) {
            if (f->ol_above == 0 || f->ol_above > MANUAL_OL_COUNTS) f->ol_above = MANUAL_OL_COUNTS;
        }
        if (cur_pos == SWITCH_POS_OHM_DIODE_CONT) {
            if (cur_sub == SUB_MODE_DIODE)      f->icon = ICON_DIODE;
            if (cur_sub == SUB_MODE_CONTINUITY) f->icon = ICON_BEEP;
        }
        if (IsVoltCurrentPos(cur_pos)) {
            f->acdc = IsAcMode(ROTARY_GetChipCommand(cur_pos, cur_sub)) ? "AC" : "DC";
        }
    }
    else if (cur_pos == SWITCH_POS_Hz_Pct && cur_sub == SUB_MODE_FREQ) {
        // PREDPOKLAD: 0x61 posiela frekvenciu v mHz (sieť 50 Hz = 50000)
        if (s->packet_type != MEAS_QUERY_FREQ) return false;
        f->decimals = 3;
        f->label = "";
        f->unit  = "Hz";
    }
    else if (cur_pos == SWITCH_POS_Hz_Pct && cur_sub == SUB_MODE_DUTY) {
        // PREDPOKLAD: 0x62 posiela striedu v 0.01 %
        if (s->packet_type != MEAS_QUERY_DUTY) return false;
        f->decimals = 2;
        f->label = "";
        f->unit  = "% ";
    }
    else {
        // dióda / pípak – škálovanie ešte nie je známe, zobrazíme surové číslo
        if (s->packet_type != MEAS_QUERY_MAIN) return false;
        snprintf(f->label_buf, sizeof(f->label_buf), "RNG %u", s->range);
        f->label = f->label_buf;
    }
    return true;
}

// Text korekcie: zachytená hodnota sa odčíta, takže korekcia má opačné znamienko.
// Napr. parazitná kapacita +0.050 nF -> "-0.050 nF".
static void FormatRelText(char *buf, size_t n, int32_t v, int8_t decimals, const char *unit) {
    char sign = (v > 0) ? '-' : (v < 0) ? '+' : ' ';
    uint32_t a = (v < 0) ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
    char num[24];                                // 10 + 1 + 10 číslic + koniec
    if (decimals > 6) decimals = 6;
    if (decimals > 0) {
        uint32_t p = (uint32_t)Pow10((uint8_t)decimals);
        snprintf(num, sizeof(num), "%lu.%0*lu", (unsigned long)(a / p), decimals, (unsigned long)(a % p));
    } else {
        snprintf(num, sizeof(num), "%lu", (unsigned long)a);
    }
    snprintf(buf, n, "%c%.12s %.4s", sign, num, unit);   // max. 1 + 12 + 1 + 4 znakov
    // odrezať medzery na konci (napr. jednotka "% ")
    size_t len = strlen(buf);
    while (len > 0 && buf[len - 1] == ' ') buf[--len] = '\0';
}

// Je vzorka mimo rozsahu (OL)? Rovnaké pravidlá ako pri zobrazení.
// Hodnota nad hranicou OL (absolútna hodnota – záporné DC napätie / prúd tiež)
static bool OverLimit(int32_t v, const Format_t *f) {
    int64_t a = (v < 0) ? -(int64_t)v : v;
    return f->ol_above && a > f->ol_above;
}

static bool SampleIsOL(const MeasSample_t *smp, const Format_t *f) {
    return smp->overload || OverLimit(smp->value, f);
}

// REL: prvé stlačenie zachytí aktuálnu hodnotu ako nulu, druhé REL vypne
// Frekvencia (mHz) -> 4-miestne zobrazenie s automatickou jednotkou:
//   < 1 kHz: Hz (50.00, 999.9), < 1 MHz: kHz (1.000 .. 999.9), inak MHz (1.000 .. 99.99)
//   v = hodnota s 'dec' desatinnými miestami (ďalej sa skracuje na 4 číslice), unit "Hz" / "kHz" / "MHz"
static void FreqScale(uint64_t mhz, int32_t *v, int8_t *dec, const char **unit) {
    if (mhz < 1000000ULL) {                    // pod 1 kHz: mHz, 3 desatinné miesta v Hz
        *v = (int32_t)mhz;          *dec = 3; *unit = "Hz";
    } else {
        uint64_t hz = mhz / 1000u;
        if (hz > 0x7FFFFFFFULL) hz = 0x7FFFFFFFULL;
        if (mhz < 1000000000ULL) { *v = (int32_t)hz; *dec = 3; *unit = "kHz"; }   // Hz -> kHz
        else                     { *v = (int32_t)hz; *dec = 6; *unit = "MHz"; }   // Hz -> MHz
    }
}

static bool IsFreqMode(void) {
    return cur_pos == SWITCH_POS_Hz_Pct && cur_sub == SUB_MODE_FREQ;
}

static void ToggleRel(void) {
    if (rel_active) {
        rel_active = false;
        if (cur_pos == SWITCH_POS_Cx) cx_rel_active = false;
    } else if (last_sample_ok && !last_sample.overload) {
        Format_t f;
        if (GetFormat(&last_sample, &f) &&
            !OverLimit(last_sample.value, &f)) {                 // z OL sa REL nezapína
            rel_offset = (int64_t)last_sample.value * Pow10(f.exp);
            if (IsFreqMode()) rel_offset = (int64_t)last_sample.freq_mhz;   // celá 40-bitová hodnota
            rel_active = true;
            if (IsFreqMode()) {
                int32_t fv; int8_t fd; const char *fu;
                FreqScale((uint64_t)rel_offset, &fv, &fd, &fu);
                while (fv > 9999 && fd > 0) { fv /= 10; fd--; }
                FormatRelText(rel_text, sizeof(rel_text), fv, fd, fu);
            } else if (cur_pos == SWITCH_POS_Cx) {
                // Kapacita: text korekcie v rovnakom formáte ako zobrazenie (napr. "-00.03 nF")
                uint8_t r = 0;
                CxSelectRange(rel_offset, &r);
                const RangeInfo_t *ri = &RANGES_CAP[r];
                FormatRelText(rel_text, sizeof(rel_text),
                              (int32_t)DivRound(rel_offset, Pow10(ri->exp)), ri->decimals, ri->unit);
            } else {
                FormatRelText(rel_text, sizeof(rel_text), last_sample.value, f.decimals, f.unit);
            }
            if (cur_pos == SWITCH_POS_Cx) {           // zapamätať pre Cx
                cx_rel_active = true;
                cx_rel_offset = rel_offset;
                strcpy(cx_rel_text, rel_text);
            }
        }
    }
    display_dirty = true;
    redraw_last   = true;
    MinMax_Reset();                                  // v0.76: REL mení zobrazené hodnoty
}

// v0.76: dlhé HOLD (F2) = MIN/MAX zap. / vyp. (vypnutie = aj vynulovanie). F2 je HOLD všade okrem
// odporu s Quick R/D/C (tam F2 = dióda a MIN/MAX sa cez F2 nezapína).
static bool F2IsHold(void) {
    return !(g_settings.quick_rdc && cur_pos == SWITCH_POS_OHM_DIODE_CONT && cur_sub == SUB_MODE_RESISTANCE);
}
static void ToggleMinMax(void) {
    mm_on = !mm_on;
    MinMax_Reset();
    hint_text = mm_on ? "MIN/MAX ON" : "MIN/MAX OFF";
    gen_hint_until = HAL_GetTick() + GEN_HINT_MS;
}

// Spracovanie tlačidiel – všetko v kontexte hlavnej slučky
// Rýchle tlačidlá R/D/C (menu "Quick R/D/C keys" = ON, poloha Ω / dióda / pípak):
//   odpor:  F2 = dióda, F3 = pípak, F4 = RANGE
//   dióda:  F2 = HOLD,  F3 = pípak, F4 = odpor
//   pípak:  F2 = HOLD,  F3 = dióda, F4 = odpor
static bool QuickRdc(void) {
    return g_settings.quick_rdc && cur_pos == SWITCH_POS_OHM_DIODE_CONT;
}

static void HandleButton(ButtonType_t btn) {
    bool range_pos = HasRangeButton(cur_pos, cur_sub);
    bool ready = (MeasChip.init_state == INIT_STEP_READY);

    if (QuickRdc() && (btn == BTN_F2 || btn == BTN_F3 ||
                       (btn == BTN_F4 && cur_sub != SUB_MODE_RESISTANCE))) {
        bool res = (cur_sub == SUB_MODE_RESISTANCE);
        if (btn == BTN_F2 && !res) {
            is_hold_active = !is_hold_active;                     // HOLD (dióda, pípak)
        } else {
            SubMode_t to;
            if (btn == BTN_F2)      to = SUB_MODE_DIODE;          // iba z odporu
            else if (btn == BTN_F3) to = (cur_sub == SUB_MODE_CONTINUITY) ? SUB_MODE_DIODE : SUB_MODE_CONTINUITY;
            else                    to = SUB_MODE_RESISTANCE;     // F4 z diódy / pípaku
            is_hold_active = false;
            SelectSubMode(to);
        }
        menu_dirty = true;
        return;
    }

    switch (btn) {
        case BTN_F1:                       // REL – vynuluje zobrazenie na aktuálnu hodnotu
            is_hold_active = false;
            ToggleRel();
            break;

        case BTN_F2:                       // HOLD
            is_hold_active = !is_hold_active;
            break;

        case BTN_F3:
            is_hold_active = false;
            if (cur_pos == SWITCH_POS_Hz_Pct) {
                SelectSubMode(SUB_MODE_FREQ);                         // F3 = Hz
            } else {
                SelectSubMode(ROTARY_GetNextSubMode(cur_pos, cur_sub)); // SEL
            }
            break;

        case BTN_F4:
            if (cur_pos == SWITCH_POS_Hz_Pct) {                       // F4 = %
                is_hold_active = false;
                SelectSubMode(SUB_MODE_DUTY);
            }
            else if (cur_pos == SWITCH_POS_OHM_DIODE_CONT && cur_sub == SUB_MODE_DIODE) {
                is_hold_active = false;
                SelectSubMode(SUB_MODE_RESISTANCE);                   // skratka dióda -> odpor
            }
            else if (cur_pos == SWITCH_POS_OHM_DIODE_CONT && cur_sub == SUB_MODE_CONTINUITY) {
                is_hold_active = false;
                SelectSubMode(SUB_MODE_DIODE);                        // skratka pípak -> dióda
            }
            else if (range_pos) {                                     // RNG
                is_hold_active = false;
                if (ready) {
                    // Každé stlačenie = ďalší rozsah. Číslo rozsahu sa uloží z odpovede čipu.
                    MeasChip_SetRange(CMD_CHIP_MANUAL);
                } else {
                    // Čip sa práve nastavuje -> vykonáme hneď ako bude pripravený
                    pending_btn = BTN_F4;
                }
            }
            break;

        case BTN_AUTO:                     // návrat do AUTO (zapamätá sa)
            is_hold_active = false;
            pending_btn = BTN_NONE;
            if (IsManual()) {
                *CurRangeMem() = MEAS_RANGE_AUTO;
                if (ready) MeasChip_SetRange(CMD_CHIP_AUTO);
                else       ApplyMode();    // inicializácia beží -> začneme ju znova už s AUTO
            }
            break;

        case BTN_BL: {                     // jas podsvietenia 25 -> 50 -> 75 -> 25 % (od aktuálneho)
            if (bl_level == 25)      bl_level = 50;
            else if (bl_level == 50) bl_level = 75;
            else                     bl_level = 25;
            LCD_BL_SetBrightness(bl_level);
            bl_msg_until = HAL_GetTick() + BL_MSG_MS;   // v0.71: BRIGHTNESS ... v hornom riadku
            return;                        // menu sa nemení
        }

        default:
            return;
    }
    menu_dirty = true;
}

// V stave READY v každom prechode slučky porovná uložený rozsah s rozsahom potvrdeným čipom
// (po 5B = potvrdený manuálny rozsah, po 5A = AUTO) a prípadne vykoná odložené stlačenie RANGE.
// Pozn.: do v0.47 sa porovnávalo iba pri prechode do READY – odpoveď čipu (5–30 ms) však zvyčajne
// prišla ešte počas toho istého prechodu slučky, prechod sa nezachytil a ostávalo "AUTO".
static void SyncRangeState(void) {
    if (MeasChip.init_state != INIT_STEP_READY) return;

    uint8_t target = MeasChip.manual_target;
    if (*CurRangeMem() != target) {
        *CurRangeMem() = target;
        menu_dirty    = true;
        display_dirty = true;
    }
    if (pending_btn == BTN_F4) {
        pending_btn = BTN_NONE;
        MeasChip_SetRange(CMD_CHIP_MANUAL);
    }
}

// Probe reset s prahom: keď veličina stúpne nad prah (PROBE_CONFIRM meraní po sebe), reštartuje
// odpočet. Ďalší reštart je možný až keď veličina klesne pod PROBE_REARM_PCT % prahu
// (napr. hroty odpojené). Pri zmene módu sa stav nuluje – hodnota, ktorá už je nad prahom,
// odpočet nereštartuje, kým raz neklesne.
//   level = porovnávaná veličina (pF alebo digity displeja), threshold 0 = vypnuté
static void ProbeLevel_Task(uint64_t level, uint32_t threshold) {
    static bool     armed = false;
    static uint8_t  above = 0;
    static uint32_t epoch = 0;
    if (epoch != mode_epoch) { epoch = mode_epoch; armed = false; above = 0; }
    // v0.78: Auto reset time OFF = odpočet sa nereštartuje ani Probe resetom (prahy ostávajú uložené)
    if (threshold == 0 || !g_settings.auto_reset) { armed = false; above = 0; return; }

    uint64_t rearm = (uint64_t)threshold * PROBE_REARM_PCT / 100u;
    if (level < rearm) {
        armed = true;                       // pod 70 % prahu -> pripravené na ďalší reset
        above = 0;
    } else if (level > threshold) {
        if (armed && ++above >= PROBE_CONFIRM) {
            last_activity_tick = HAL_GetTick();
            armed = false;
            above = 0;
        }
    } else {
        above = 0;                          // medzi prahmi – nič
    }
}

#if !DEBUG_SCREEN
// Nadpis meranej veličiny v hornom riadku. Pri výpadku komunikácie sa namiesto neho
// zobrazí červené upozornenie, aby bolo jasné, prečo hodnota stojí.
static const char *GetTitle(void) {
    if (gen_open) return GenMenu_Title();          // "GENERATOR" / "GEN MEMORY"
    if (scope_open) return scope_test ? "SCOPE TEST" : Scope_TopText();
    if (mem_open) return MeterMem_Title();          // v0.72: "METER MEMORY 2/4" pri viacerých stránkach
    switch (cur_pos) {
        case SWITCH_POS_V:
        case SWITCH_POS_mV:   return "VOLTAGE";
        case SWITCH_POS_uA:
        case SWITCH_POS_mA:
        case SWITCH_POS_10A:  return "CURRENT";
        case SWITCH_POS_Cx:   return "CAPACITANCE";
        case SWITCH_POS_Hz_Pct:
            return (cur_sub == SUB_MODE_DUTY) ? "DUTY CYCLE" : "FREQUENCY";
        case SWITCH_POS_OHM_DIODE_CONT:
            if (cur_sub == SUB_MODE_DIODE)      return "DIODE";
            if (cur_sub == SUB_MODE_CONTINUITY) return "CONTINUITY";
            return "RESISTANCE";
        default:              return "";
    }
}

static void Display_Title(void) {
    static char     last[28] = "?";
    static uint32_t epoch = 0;
    if (epoch != screen_epoch) { epoch = screen_epoch; strcpy(last, "?"); }
    bool other = gen_open || scope_open;                        // generátor / osciloskop: čip nezaujíma
    bool batm = (int32_t)(bat_msg_until - HAL_GetTick()) > 0;              // slabá batéria (všade)
    bool blm  = !batm && (int32_t)(bl_msg_until - HAL_GetTick()) > 0;      // v0.71: jas (všade)
    bool defm = !batm && !blm && !other && (int32_t)(def_msg_until - HAL_GetTick()) > 0;   // dlhé SAVE
    bool hint = batm || (!blm && !defm && !gen_open && (int32_t)(gen_hint_until - HAL_GetTick()) > 0);   // aj v osciloskope
    bool err  = MeasChip.comm_error && !other && !hint && !defm && !blm;
    const char *bltxt = (bl_level <= 25) ? "BRIGHTNESS MIN" : (bl_level >= 75) ? "BRIGHTNESS MAX" : "BRIGHTNESS 50%";
    const char *txt = batm ? "BATTERY LOW" : blm ? bltxt : defm ? (def_msg_ok ? "DEFAULT SAVED" : "SAVE ERROR")
                    : hint ? hint_text : err ? "CHIP NO RESPONSE" : GetTitle();
    uint16_t color  = blm ? LCD_YELLOW : defm ? (def_msg_ok ? LCD_GREEN : LCD_RED)
                    : hint ? LCD_ORANGE : err ? LCD_RED : LCD_CYAN;   // upozornenie oranžové, chyba červená
    if (defm || blm) err = true;                                // rovnaké umiestnenie ako hlásenia

    // v0.66: dlhé časované hlásenie (main.c hint alebo hlásenie osciloskopu) -> celý horný riadok
    const char *wide = NULL;
    uint16_t    wcol = color;
    if (hint && strlen(txt) > TOP_TXT_MAX) {
        wide = txt;
    } else if (scope_open && !scope_test && !hint && !blm) {
        wide = Scope_WideMsg(&wcol);
    } else if (mem_open && !hint && !blm) {
        wide = MeterMem_TopMsg(&wcol);                         // v0.72: hlásenia pamäte hodnôt vždy cez celý riadok
    }
    if (wide) {
        if (!TOP_WIDE() || strcmp(wide, last) != 0) {
            strncpy(last, wide, sizeof(last) - 1);
            last[sizeof(last) - 1] = '\0';
            top_wide = true;
            top_wide_epoch = screen_epoch;
            int wl = (int)strlen(wide);
            if (wl > TOP_WIDE_MAX) wl = TOP_WIDE_MAX;
            int ww = wl * 12 - 2;
            char wb[TOP_WIDE_MAX + 1];
            snprintf(wb, sizeof(wb), "%.*s", wl, wide);
            LCD_FillRect(2, (uint16_t)(TOP_Y() - 2), 316, 20, LCD_BLACK);   // celý riadok (čas, nadpis, batéria)
            LCD_DrawString((uint16_t)((320 - ww) / 2), (uint16_t)TOP_Y(), wb, wcol, LCD_BLACK, 2);
        }
        return;
    }
    if (TOP_WIDE()) {                                           // hlásenie skončilo -> obnoviť celý riadok
        top_wide = false;
        LCD_FillRect(2, (uint16_t)(TOP_Y() - 2), 316, 20, LCD_BLACK);
        screen_epoch++;                                         // časomiera, nadpis, batéria znova
        return;                                                 // nakreslia sa v ďalšom priechode
    }

    // Osciloskop: farebný horný riadok kreslí scope.c (mimo upozornení main.c)
    if (scope_open && !scope_test && !hint && !blm) {
        bool force = strcmp(last, "\x01SCOPE") != 0;
        if (force) strcpy(last, "\x01SCOPE");
        Scope_DrawTitle((uint16_t)TOP_Y(), force);
        return;
    }

    if (strcmp(txt, last) == 0) return;
    strncpy(last, txt, sizeof(last) - 1);
    last[sizeof(last) - 1] = '\0';

    // Šírka textu: 12 px na znak, posledný znak bez 2 px medzery
    int len = (int)strlen(txt);
    if (len > TOP_TXT_MAX) len = TOP_TXT_MAX;
    int w = len * 12 - 2;
    int x;
    if (err || hint || (mem_open && len >= TOP_TXT_MAX)) {
        // Rovnaká medzera od časomiery (koniec x = 63) aj od batérie (začiatok x = 279 pri rámiku)
        // (v0.72: aj "METER MEMORY 2/4" – 16 znakov by v strede displeja splynulo s časomierou)
        x = (TOP_FREE_X0 + TOP_FREE_X1 - w) / 2;
    } else {
        // Názov veličiny v strede displeja
        x = (320 - w) / 2;
    }

    char buf[20];
    snprintf(buf, sizeof(buf), "%.*s", len, txt);
    LCD_FillRect(64, (uint16_t)(TOP_Y() - 2), 214, 18, LCD_BLACK);   // zmazať priestor medzi časovačom a batériou (batéria od x = 279)
    LCD_DrawString((uint16_t)x, (uint16_t)TOP_Y(), buf, color, LCD_BLACK, 2);
}

#endif // !DEBUG_SCREEN

// Pin do analógového režimu – v spánku neodoberá žiadny prúd (žiadny pull-up, žiadny vstupný stupeň)
static void PinToAnalog(GPIO_TypeDef *port, uint16_t pins) {
    GPIO_InitTypeDef gi = {0};
    gi.Pin  = pins;
    gi.Mode = GPIO_MODE_ANALOG;
    gi.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(port, &gi);
}

// ---------------------------------------------------------------------------
// Pokračovanie po zobudení zo spánku
//   Pred spánkom sa stav uloží do RAM sekcie .noinit (štartovací kód ju nenuluje). STOP režim RAM
//   zachová a softvérový reset po zobudení ju nezmaže. Pri zapnutí prepínačom (odpojené napájanie)
//   má RAM náhodný obsah -> značka ani kontrolný súčet nesedia -> načíta sa EEPROM.
// ---------------------------------------------------------------------------
#define RESUME_MAGIC   0x52534D38u      // "RSM8" (v0.62: + panel meraní)
enum { RES_SCR_MEAS = 0, RES_SCR_GEN, RES_SCR_SCOPE };

// Osciloskop: otvorenie / zatvorenie podľa toho, či beží testovací režim
static void ScopeEnter(void) {
    if (scope_test) ScopeTest_Open();
    else            Scope_Open(cur_pos == SWITCH_POS_mV);
}
static void ScopeLeave(bool keep_state) {        // keep_state = návrat z generátora
    if (scope_test) ScopeTest_Close(keep_state);
    else            Scope_Close();
    if (!keep_state) scope_test = false;
}

typedef struct {
    uint32_t    magic;
    uint8_t     pos_sub[NUM_POSITIONS];
    uint8_t     pos_range[NUM_POSITIONS][NUM_SUBMODES];
    uint8_t     bl_level;
    uint8_t     screen;                 // RES_SCR_*
    uint8_t     gen_from_scope;
    GenParams_t gen;                    // nastavenie generátora vrátane výstupu ON/OFF
    ScopeState_t scope;                 // rozsahy osciloskopu
    ScopeExt_t  scope_ext;              // v0.76: Interp / Peak det / View
    uint32_t    sum;                    // kontrolný súčet cez predchádzajúce bajty
} Resume_t;

static Resume_t resume_ram __attribute__((section(".noinit")));

static uint8_t  resume_screen = RES_SCR_MEAS;   // obrazovka na obnovenie po zobudení (po prvom prečítaní prepínača)
static bool     resume_from_scope = false;
static bool     resume_gen_on = false;

static uint32_t ResumeSum(const Resume_t *r) {
    const uint8_t *b = (const uint8_t *)r;
    uint32_t s = 0x1234u;
    for (size_t i = 0; i < offsetof(Resume_t, sum); i++) s = (s << 5) + s + b[i];   // djb2
    return s;
}

static void Resume_Save(void) {
    memset(&resume_ram, 0, sizeof(resume_ram));
    resume_ram.magic = RESUME_MAGIC;
    for (int p = 0; p < NUM_POSITIONS; p++) {
        resume_ram.pos_sub[p] = (uint8_t)pos_sub[p];
        for (int m = 0; m < NUM_SUBMODES; m++) resume_ram.pos_range[p][m] = pos_range[p][m];
    }
    resume_ram.bl_level       = bl_level;
    resume_ram.screen         = gen_open ? RES_SCR_GEN : scope_open ? RES_SCR_SCOPE : RES_SCR_MEAS;
    resume_ram.gen_from_scope = gen_open && gen_from_scope;
    resume_ram.gen            = g_gen;
    Scope_GetState(&resume_ram.scope);
    Scope_GetExt(&resume_ram.scope_ext);
    resume_ram.sum            = ResumeSum(&resume_ram);
}

// Volať pri štarte po načítaní EEPROM. true = zobudenie so stavom v RAM (stav je obnovený).
static bool Resume_Restore(bool woke) {
    bool ok = woke && resume_ram.magic == RESUME_MAGIC && resume_ram.sum == ResumeSum(&resume_ram);
    resume_ram.magic = 0;                       // použiť iba raz
    if (!ok) return false;
    for (int p = 0; p < NUM_POSITIONS; p++) {
        pos_sub[p] = (SubMode_t)resume_ram.pos_sub[p];
        for (int m = 0; m < NUM_SUBMODES; m++) pos_range[p][m] = resume_ram.pos_range[p][m];
    }
    if (resume_ram.bl_level == 25 || resume_ram.bl_level == 50 || resume_ram.bl_level == 75)
        bl_level = resume_ram.bl_level;
    g_gen = resume_ram.gen;
    if (g_gen.wave >= GEN_WAVE_COUNT || g_gen.freq_idx >= GEN_NUM_FREQ ||
        g_gen.duty_idx >= GEN_NUM_DUTY || g_gen.volt_idx >= GEN_NUM_VOLT) {
        Settings_GenToRam();                    // poškodené -> posledné uložené
        g_gen.on = false;
    }
    Scope_SetState(&resume_ram.scope);         // kontroluje rozsahy
    Scope_SetExt(&resume_ram.scope_ext);       // v0.76
    resume_gen_on     = g_gen.on;
    resume_screen     = resume_ram.screen;
    resume_from_scope = resume_ram.gen_from_scope != 0;
    return true;
}

// Automatické vypnutie: odpojí LCD a merací čip, MCU uspí do STOP režimu.
// Zobudí ho stlačenie ľubovoľného tlačidla (EXTI) -> reset -> štart s obnovením stavu z RAM.
// Do EEPROM sa pred spánkom nič neukladá (od v0.55): po zapnutí prepínačom platí to,
// čo užívateľ uložil dlhým SAVE.
void AutoOff_PowerDown(void) {
    // Generátor: zavrieť nastavenia / kalibráciu (obnoví nastavenie spred kalibračného signálu)
    if (gen_open) GenMenu_Close();
    // 0. Stav na pokračovanie do RAM, potom vypnúť DAC, DMA, TIM6
    Resume_Save();
    Gen_Stop();

    // 1. Podsvietenie a komunikácia s čipom
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_4);
    HAL_UART_DeInit(&huart6);
    PinToAnalog(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);      // USART6 TX/RX k vypnutému čipu

    // 1b. ADC a interná referencia VREFINT – inak by ich analógová časť brala prúd aj v STOP
    ADC1->CR2 &= ~ADC_CR2_ADON;
    ADC->CCR  &= ~ADC_CCR_TSVREFE;

    // 2. Napájanie LCD + čipu vypnúť (všetky piny k LCD najprv do 0)
    LCD_Power_Off();

    // 3. Bzučiak do 0, otočný prepínač bez pull-upov (zopnutý kontakt by inak stále odoberal prúd)
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
    PinToAnalog(SW1_GPIO_Port, SW1_Pin | SW2_Pin | SW3_Pin);

    // 4. Tlačidlá ostávajú ako budenie: výstupy matice v 0, vstupy s pull-upom a EXTI na zostupnú hranu
    HAL_GPIO_WritePin(OUT_K246_GPIO_Port, OUT_K246_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OUT_K357_GPIO_Port, OUT_K357_Pin, GPIO_PIN_RESET);
    HAL_Delay(300);                                      // nech stihne doznieť prípadné stlačenie

    // 5. Všetky ostatné piny do analógového režimu (najnižší únik). Ostávajú iba:
    //    PA0, PA1, PA3 (vstupy matice), PA4, PA6 (výstupy matice v 0), PA2 (napájanie v 0),
    //    PA13, PA14 (SWD – aby sa dal procesor znova naprogramovať), PC9 (tlačidlo BL)
    PinToAnalog(GPIOA, (uint16_t)(GPIO_PIN_All & ~(IN_K6_K7_Pin | IN_K4_K5_Pin | IN_K2_K3_Pin |
                                                   OUT_K357_Pin | OUT_K246_Pin | A3V_Pin |
                                                   GPIO_PIN_13 | GPIO_PIN_14)));
    // GPIOB: LCD zbernica a I2C k EEPROM (PB8, PB9) do analógu; bzučiak PB10 a podsvietenie PB11 ostávajú výstupy v 0,
    //        aby budiace tranzistory nemali plávajúci vstup
    PinToAnalog(GPIOB, (uint16_t)(GPIO_PIN_All & ~(Buzzer_Pin | GPIO_PIN_11)));
    PinToAnalog(GPIOC, (uint16_t)(GPIO_PIN_All & ~IN_K1_Pin));
    PinToAnalog(GPIOH, GPIO_PIN_0 | GPIO_PIN_1);
    __HAL_GPIO_EXTI_CLEAR_IT(IN_K1_Pin | IN_K2_K3_Pin | IN_K4_K5_Pin | IN_K6_K7_Pin);

    // 6. STOP režim s najúspornejším nastavením F410:
    //    nízkoodberový regulátor v nízkom napätí (LPLVDS), flash vypnutá,
    //    ladenie v STOP vypnuté (inak by debug blok držal hodiny bežať)
    DBGMCU->CR &= ~(DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STANDBY);
    HAL_SuspendTick();
    HAL_PWREx_EnableFlashPowerDown();
    HAL_PWREx_EnableLowRegulatorLowVoltage();
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

    // 7. Zobudenie tlačidlom -> čistý reštart celého firmvéru
    NVIC_SystemReset();
}

// Odpočet automatického vypnutia (beží vždy, aj v menu a pri ladiacej obrazovke)
static uint32_t AutoOff_Remaining(void) {
    uint32_t total = AutoOffSeconds();
    if (total == 0) return UINT32_MAX;                 // vypnutie zakázané
    uint32_t elapsed = (HAL_GetTick() - last_activity_tick) / 1000u;
    return (elapsed >= total) ? 0 : (total - elapsed);
}

static void AutoOff_Task(void) {
    // Zakázané auto vypnutie (generátor / osciloskop): krátke pípnutie každých 5 min ako pripomienka
    if (AutoOffSeconds() == 0) {
        if (HAL_GetTick() - gen_beep_tick >= GEN_BEEP_PERIOD_MS) {
            gen_beep_tick = HAL_GetTick();
            if (g_settings.beep) Beep_Start(80);   // pripomienka iba pri Beep = ON
        }
        return;
    }
    if (AutoOff_Remaining() == 0 && !auto_off_done) {
        auto_off_done = true;
        AutoOff_PowerDown();
    }
}

#if !DEBUG_SCREEN

// Odpočet do vypnutia vľavo hore "MM:SS", dvojbodka bliká každých 0.5 s.
// V menu nastavení a v GEN MEMORY sa namiesto neho ukazuje tyrkysovo odpočet do zatvorenia menu
// (2 min bez tlačidla) – odpočet vypnutia tam stojí.
#define IDLE_TIMER_COLOR  LCD_CYAN
// Čas pre časomieru: 5 znakov, bez úvodnej nuly, zarovnané vpravo (" 5:12", "12:30"); od 100 min "1h40m"
static void FmtTimer(char *buf, size_t n, uint32_t sec, char colon) {
    if (sec >= 6000u) snprintf(buf, n, "%luh%02lum", (unsigned long)(sec / 3600u), (unsigned long)((sec / 60u) % 60u));
    else              snprintf(buf, n, "%2lu%c%02lu", (unsigned long)(sec / 60u), colon, (unsigned long)(sec % 60u));
}

#define TIMER_X  6                                     // v0.64: o 2 px doprava

static void Display_Timer(void) {
    static char     last[16] = "?";
    static uint32_t epoch = 0;
    if (epoch != screen_epoch) { epoch = screen_epoch; last[0] = '?'; last[1] = '\0'; }
    if (TOP_WIDE()) return;                                   // dlhé hlásenie cez celý riadok

    uint32_t now = HAL_GetTick();

    // Odpočet do zatvorenia menu
    bool     idle = menu_open || (gen_open && GenMenu_IsMenuOpen());
    if (idle) {
        uint32_t left_ms = menu_open ? ((now - menu_input_tick >= MENU_IDLE_MS) ? 0 : MENU_IDLE_MS - (now - menu_input_tick))
                                     : GenMenu_MemoryIdleLeftMs();
        uint32_t left = (left_ms + 999u) / 1000u;              // zaokrúhlené nahor (02:00 hneď po stlačení)
        char colon = ((now / 500u) & 1u) ? ' ' : ':';
        char buf[16];
        FmtTimer(buf, sizeof(buf), left, colon);
        if (strcmp(buf, last) == 0) return;
        if (strcmp(last, "OFF") == 0) LCD_FillRect(2, 5, 64, 21, LCD_BLACK);   // zmazať "AUTO OFF / DISABLED"
        strcpy(last, buf);
        LCD_DrawString(TIMER_X, (uint16_t)TOP_Y(), buf, IDLE_TIMER_COLOR, LCD_BLACK, 2);
        return;
    }

    // Auto vypnutie zakázané -> namiesto času nápis
    if (AutoOffSeconds() == 0) {
        if (strcmp(last, "OFF") == 0) return;
        strcpy(last, "OFF");
        LCD_FillRect(2, 5, 64, 21, LCD_BLACK);
        LCD_DrawString(8, (uint16_t)(TOP_Y() - 1), "AUTO OFF", LCD_ORANGE, LCD_BLACK, 1);   // x = 8 ako WAVE pod tým
        LCD_DrawString(8, (uint16_t)(TOP_Y() + 9), "DISABLED", LCD_ORANGE, LCD_BLACK, 1);
        return;
    }
    if (strcmp(last, "OFF") == 0) LCD_FillRect(2, 5, 64, 21, LCD_BLACK);   // zmazať malý nápis

    uint32_t remain = AutoOff_Remaining();

    char colon = ((now / 500u) & 1u) ? ' ' : ':';
    char buf[16];
    FmtTimer(buf, sizeof(buf), remain, colon);

    if (strcmp(buf, last) == 0) return;
    strcpy(last, buf);
    LCD_DrawString(TIMER_X, (uint16_t)TOP_Y(), buf, (remain < 60u) ? LCD_RED : LCD_WHITE, LCD_BLACK, 2);
}

// Batéria vpravo hore – prekreslí sa iba pri zmene percent (a pri novej obrazovke)
#define BAT_X        284
#define BAT_X_FRAME  279          // rámik na x = 319 -> medzera 6 px
// Slabá batéria: pod 3.30 V (priemer ~30 s) raz text BATTERY LOW + pípnutie, symbol bliká;
// znova až po nabití nad 3.40 V
#define BAT_LOW_MV    3300u
#define BAT_OK_MV     3400u
#define BAT_MSG_MS    3000u

static void BatteryLowCheck(void) {
    uint16_t mv = Battery_GetAvgMillivolts();
    if (mv == 0) return;                                   // ešte nemerané
    if (!bat_low && mv <= BAT_LOW_MV) {
        bat_low = true;
        bat_msg_until = HAL_GetTick() + BAT_MSG_MS;
        Beep_Warning();                                    // iba ak je Beep povolené
    } else if (bat_low && mv >= BAT_OK_MV) {
        bat_low = false;
    }
}

static void Display_Battery(void) {
    static uint16_t last = 0xFFFF;
    static uint32_t epoch = 0;
    if (epoch != screen_epoch) { epoch = screen_epoch; last = 0xFFFF; }
    BatteryLowCheck();
    if (TOP_WIDE()) return;                                   // dlhé hlásenie cez celý riadok
    uint8_t pct = Battery_GetPercent();
    bool hide = bat_low && ((HAL_GetTick() / 500u) & 1u);  // blikanie 1x za sekundu
    uint16_t key = (uint16_t)(pct | (hide ? 0x100u : 0u));
    if (key == last) return;
    last = key;
    // Obrazovky s rámikom (generátor, GEN MEMORY, menu nastavení, osciloskop): ďalej od pravej čiary
    // rámika a na y = 8 (zarovnané s nápisom MENU)
    uint16_t x = FRAME_SCREEN() ? BAT_X_FRAME : BAT_X;
    uint16_t by = (uint16_t)(TOP_Y() - (FRAME_SCREEN() ? 1 : 0));   // batéria 16 px: 5 / 5 px v pruhu
    if (hide) LCD_FillRect(x, by, 34, 16, LCD_BLACK);
    else      LCD_DrawBattery(x, by, pct);
}

// Hodnota REL korekcie nad tlačidlom REL (prázdne, keď REL nie je aktívne)
// v0.75: riadok GEN (dole y 182..188) a riadok REL (y 186..203) sa pri prepnutí REL striedajú
static bool relinfo_redraw = false;   // GEN riadok zmazal časť riadku REL -> REL nakresliť znova
static bool geninfo_redraw = false;   // REL riadok bol zmazaný -> GEN riadok nakresliť znova
static void Display_RelInfo(void) {
    static char     last[32] = "?";
    static uint32_t epoch = 0;
    if (epoch != screen_epoch) { epoch = screen_epoch; strcpy(last, "?"); }
    if (relinfo_redraw) { relinfo_redraw = false; strcpy(last, "?"); }

    const char *txt = rel_active ? rel_text : "";
    if (strcmp(txt, last) == 0) return;
    strcpy(last, txt);
    LCD_FillRect(2, 186, 230, 18, LCD_BLACK);
    geninfo_redraw = true;
    if (txt[0]) LCD_DrawString(4, 188, txt, LCD_CYAN, LCD_BLACK, 2);
}
// Na obrazovke voltmetra informácia, že beží výstup generátora (s jeho nastavením)
// v0.75: riadok v strede medzi spodkom veľkých číslic (y = 160) a vrchom spodného menu (y = 212):
// stred 186, text 8 px -> y = 182. Pri aktívnom REL je na y 188..203 hodnota REL korekcie,
// vtedy ostáva riadok na pôvodnom mieste y = 170 (v strede medzi číslicami a riadkom REL).
#define GEN_INFO_Y      182
#define GEN_INFO_Y_REL  170
static void Display_GenIndicator(void) {
    static char     last[64] = "?";
    static uint32_t epoch = 0;
    static uint16_t last_y = GEN_INFO_Y;
    if (epoch != screen_epoch) { epoch = screen_epoch; strcpy(last, "?"); }
    if (geninfo_redraw) { geninfo_redraw = false; strcpy(last, "?"); }

    uint16_t y = rel_active ? GEN_INFO_Y_REL : GEN_INFO_Y;
    char buf[64] = "";
    if (Gen_IsOn()) {
        char f[16], v[16];
        Gen_FormatFreq(Gen_FreqHz(g_gen.freq_idx), f, sizeof(f));
        Gen_FormatVolt(Gen_VoltMv(g_gen.volt_idx), v, sizeof(v));
        if (Gen_DutyUsed(g_gen.wave))
            snprintf(buf, sizeof(buf), "GEN OUTPUT ON: %s %s %u%% %s",
                     Gen_WaveName(g_gen.wave), f, Gen_DutyPct(g_gen.duty_idx), v);
        else
            snprintf(buf, sizeof(buf), "GEN OUTPUT ON: %s %s %s", Gen_WaveName(g_gen.wave), f, v);
    }
    if (y == last_y && strcmp(buf, last) == 0) return;
    if (y != last_y) {
        // presun (REL zapnuté/vypnuté): zmazať starý riadok; dolná poloha (182..189) zasahuje
        // do riadku REL (od y = 186) -> REL text nakresliť znova
        LCD_FillRect(2, last_y, 316, 8, LCD_BLACK);
        if (last_y == GEN_INFO_Y) relinfo_redraw = true;
    }
    strcpy(last, buf);
    last_y = y;
    LCD_FillRect(2, y, 316, 8, LCD_BLACK);
    if (buf[0]) LCD_DrawString((uint16_t)((320 - (int)strlen(buf) * 6 + 1) / 2), y, buf,
                               LCD_GREEN, LCD_BLACK, 1);                           // vodorovne v strede
}
#endif // !DEBUG_SCREEN

// Návrat na meraciu obrazovku (z menu alebo generátora): prekresliť všetko
static void ReturnToMeasScreen(void) {
    LCD_Clear(LCD_BLACK);
    screen_epoch++;
    display_dirty = true;
    menu_dirty    = true;
    redraw_last   = true;
}

#if DEBUG_SCREEN
// Stavový riadok – prekreslí sa iba pri zmene
static void Display_Status(void) {
    static InitState_t last_state = (InitState_t)0xFF;
    static bool        last_err   = false;
    static uint8_t     last_mode  = 0xFF;

    InitState_t st  = MeasChip.init_state;
    bool        err = MeasChip.comm_error;
    uint8_t     mode = MeasChip.mode_cmd;

    if (st == last_state && err == last_err && mode == last_mode) return;
    last_state = st;
    last_err   = err;
    last_mode  = mode;

    char buf[26];
    uint16_t color = LCD_WHITE;

    if (err) {
        snprintf(buf, sizeof(buf), "%-25s", "STATUS: CHIP NEODPOVEDA");
        color = LCD_RED;
    } else switch (st) {
        case INIT_STEP_SET_MODE:
        case INIT_STEP_SEND_AUTO:
            snprintf(buf, sizeof(buf), "STATUS: WAIT FOR 58 %02X  ", mode);
            break;
        case INIT_STEP_SET_AUTO:
            snprintf(buf, sizeof(buf), "%-25s", "STATUS: WAIT FOR 5A 00");
            break;
        case INIT_STEP_SET_MAN:
            snprintf(buf, sizeof(buf), "%-25s", "STATUS: WAIT FOR 5B REQ");
            break;
        case INIT_STEP_READY:
            snprintf(buf, sizeof(buf), "%-25s", "STATUS: TELEMETRY ACTIVE");
            color = LCD_GREEN;
            break;
        default:
            snprintf(buf, sizeof(buf), "%-25s", "STATUS: IDLE");
            break;
    }
    LCD_DrawString(10, 10, buf, color, LCD_BLACK, 2);
}

// RAW sniffer – iba keď prišli nové bajty, najviac každých 100 ms
static void Display_Sniffer(void) {
    static uint32_t last_count = 0xFFFFFFFF;
    static uint32_t last_tick  = 0;
    uint8_t snap[16];

    if (debug_rx_count == last_count || (HAL_GetTick() - last_tick) < 100) return;
    last_tick = HAL_GetTick();

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    last_count = debug_rx_count;
    for (int i = 0; i < 16; i++) snap[i] = debug_rx_buffer[i];
    __set_PRIMASK(primask);

    char hex_string[32];
    snprintf(hex_string, sizeof(hex_string), "%02X %02X %02X %02X %02X %02X %02X %02X",
             snap[0], snap[1], snap[2], snap[3], snap[4], snap[5], snap[6], snap[7]);
    LCD_DrawString(25, 165, hex_string, LCD_YELLOW, LCD_BLACK, 2);
    snprintf(hex_string, sizeof(hex_string), "%02X %02X %02X %02X %02X %02X %02X %02X",
             snap[8], snap[9], snap[10], snap[11], snap[12], snap[13], snap[14], snap[15]);
    LCD_DrawString(25, 185, hex_string, LCD_YELLOW, LCD_BLACK, 2);
}

#endif // DEBUG_SCREEN

// Text sa prekreslí iba ak sa zmenil (cache je pole aspoň 24 znakov)
static void DrawTextIfChanged(char *cache, uint16_t x, uint16_t y, const char *txt, uint16_t color) {
    if (!display_dirty && strcmp(cache, txt) == 0) return;
    snprintf(cache, 24, "%.23s", txt);
    LCD_DrawString(x, y, txt, color, LCD_BLACK, 2);
}

// v0.71: výpočet zobrazovanej hodnoty (bez kreslenia) – pre veľký displej aj pamäť hodnôt.
// Pri Cx posúva zobrazovací rozsah s hysterézou (cx_disp_range) -> volať raz na vzorku.
typedef struct {
    bool        ol, neg;
    uint32_t    a;               // absolútna hodnota, max. 4 číslice
    int8_t      decimals;
    const char *label, *unit, *acdc;
    uint8_t     icon;
} Disp_t;

static bool ComputeDisp(const MeasSample_t *s, Disp_t *d) {
    Format_t    f;
    if (!GetFormat(s, &f)) return false;

    int8_t      decimals = f.decimals;
    const char *label = f.label;
    const char *unit  = f.unit;
    const char *acdc  = f.acdc;
    int32_t     v  = s->value;
    bool        ol = s->overload;

    // Mimo rozsahu čipu (napr. odpor > 60 MΩ, dióda naprázdno, rozpojený pípak)
    if (OverLimit(v, &f)) ol = true;

    // --- Kapacita: automatická voľba jednotky nF / uF / mF podľa veľkosti hodnoty ---
    // Hodnota sa prepočíta na pF podľa rozsahu z čipu, odčíta sa REL a vyberie sa
    // zobrazovací rozsah (9999 digitov, s hysterézou).
    if (cur_pos == SWITCH_POS_Cx && !ol) {
        int64_t pf = (int64_t)v * Pow10(f.exp);
        if (rel_active) pf -= rel_offset;

        if (!CxSelectRange(pf, &cx_disp_range)) {
            ol = true;                                // nad 99.99 mF
        } else {
            const RangeInfo_t *ri = &RANGES_CAP[cx_disp_range];
            v        = (int32_t)DivRound(pf, Pow10(ri->exp));
            decimals = ri->decimals;
            label    = ri->label;
            unit     = ri->unit;
        }
    }
    // --- Frekvencia: 40-bitová hodnota v mHz, jednotka Hz / kHz / MHz ---
    else if (IsFreqMode() && !ol) {
        int64_t x   = (int64_t)s->freq_mhz - (rel_active ? rel_offset : 0);
        bool    xn  = (x < 0);
        int32_t fv; int8_t fd; const char *fu;
        FreqScale((uint64_t)(xn ? -x : x), &fv, &fd, &fu);
        v        = xn ? -fv : fv;
        decimals = fd;
        unit     = fu;
    }
    // --- REL: odčítanie offsetu v základných jednotkách, výsledok v rozlíšení aktuálneho rozsahu ---
    else if (rel_active && !ol) {
        int64_t p    = Pow10(f.exp);
        int64_t diff = DivRound((int64_t)v * p - rel_offset, p);
        if (diff > 99999999LL || diff < -99999999LL) ol = true;
        else v = (int32_t)diff;
    }

    // --- Zmenšenie na 4 číslice (ubúdajú desatinné miesta, potom OL) ---
    bool neg = (v < 0);
    uint32_t a = neg ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
    while (!ol && a > 9999) {
        if (decimals > 0) {
            a /= 10;
            decimals--;
        } else {
            ol = true;                            // viac ako 4 číslice -> mimo rozsah
        }
    }


    d->ol = ol; d->neg = neg; d->a = a; d->decimals = decimals;
    d->label = label; d->unit = unit; d->acdc = acdc; d->icon = f.icon;
    return true;
}

// v0.71: režim aktuálnej polohy pre pamäť hodnôt
static uint8_t MemModeNow(void) {
    bool ac = IsVoltCurrentPos(cur_pos) && IsAcMode(ROTARY_GetChipCommand(cur_pos, cur_sub));
    switch (cur_pos) {
        case SWITCH_POS_V:   return ac ? MM_V_AC  : MM_V_DC;
        case SWITCH_POS_mV:  return ac ? MM_mV_AC : MM_mV_DC;
        case SWITCH_POS_uA:  return ac ? MM_uA_AC : MM_uA_DC;
        case SWITCH_POS_mA:  return ac ? MM_mA_AC : MM_mA_DC;
        case SWITCH_POS_10A: return ac ? MM_A_AC  : MM_A_DC;
        case SWITCH_POS_Cx:  return MM_CAP;
        case SWITCH_POS_Hz_Pct: return (cur_sub == SUB_MODE_DUTY) ? MM_DUTY : MM_FREQ;
        case SWITCH_POS_OHM_DIODE_CONT:
            return (cur_sub == SUB_MODE_DIODE) ? MM_DIODE : (cur_sub == SUB_MODE_CONTINUITY) ? MM_CONT : MM_OHM;
        default:             return MM_NONE;
    }
}

// v0.71: zapamätať zobrazenú hodnotu (cur_disp) pre pamäť hodnôt
static void NoteDisp(const Disp_t *d) {
    MeterMemVal_t *m = &cur_disp;
    m->mode     = MemModeNow();
    m->used     = (m->mode != MM_NONE);
    m->ol       = d->ol;
    m->neg      = d->neg && !d->ol;
    m->rel      = rel_active;
    m->decimals = d->ol ? 0u : (uint8_t)((d->decimals < 0) ? 0 : (d->decimals > 4) ? 4 : d->decimals);
    m->mag      = d->ol ? 0u : (uint16_t)((d->a > 9999u) ? 9999u : d->a);
    memset(m->unit, 0, sizeof(m->unit));
    for (int i = 0; i < 3 && d->unit[i] != '\0'; i++) m->unit[i] = d->unit[i];
    for (int i = 2; i >= 0 && (m->unit[i] == ' ' || m->unit[i] == '\0'); i--) m->unit[i] = '\0';   // "% " -> "%"
    m->tag      = MM_TAG_NONE;
    MinMax_Update(m);                                // v0.76
}

// Veľké číslice + bodka + jednotky. Prekresľuje sa iba to, čo sa zmenilo.
static void Display_Measurement(const MeasSample_t *s) {
    static char    last_digits[6] = "?????";
    static uint8_t last_dot = 0xFF;
    static char    last_top[24], last_acdc[24], last_unit[24];

    const uint16_t txt_color  = LCD_YELLOW;
    const uint16_t digit_step = FONT_7SEG_WIDTH + 12;   // krok pre 45 px široké číslice
    // Číslice od x = 0 (prvá pozícia je znamienko, väčšinou prázdna), jednotka od x = 280:
    // zmestia sa 3 znaky (kHz, MHz) – koniec na x = 314
    const uint16_t dig_x0     = 0;
    const uint16_t unit_x     = 280;
    const uint16_t dot_size   = 7;
    const uint16_t dot_y      = DIGITS_Y + (FONT_7SEG_HEIGHT - dot_size);

    const char *range_mode_str = IsManual() ? "MAN " : "AUTO";
    Disp_t d;
    if (!ComputeDisp(s, &d)) return;
    NoteDisp(&d);
    int8_t      decimals = d.decimals;
    const char *label = d.label;
    const char *unit  = d.unit;
    const char *acdc  = d.acdc;
    bool        ol  = d.ol;
    bool        neg = d.neg;
    uint32_t    a   = d.a;

    // --- Bodka ---
    uint8_t dot = (ol || decimals == 0) ? 0 : (uint8_t)(5 - decimals);
    if (dot != last_dot || display_dirty) {
        for (uint16_t r = 0; r < dot_size; r++) {
            for (uint8_t k = 2; k <= 4; k++) {
                LCD_DrawFastHLine(dig_x0 + (k * digit_step) - 10, dot_y + r, 10, LCD_BLACK);
            }
        }
        last_dot = dot;
    }

    // --- Veľké číslice (pri pretečení "OL" tým istým fontom, prekresľujú sa iba zmenené) ---
    char digits[16];
    if (ol) {
        strcpy(digits, "  0L ");             // v strede: 2. a 3. pozícia zo 4 číslic (0 namiesto O)
    } else {
        snprintf(digits, sizeof(digits), "%c%04lu", neg ? '-' : ' ', (unsigned long)a);
        // v0.66 Hide zeros: nuly zľava sa nekreslia, pred bodkou ostane vždy jedna číslica
        // (000.0 -> 0.0, 0005 -> 5); mínus sa posunie hneď pred prvú číslicu (-0.5).
        // Číslice, bodka aj jednotka ostávajú na svojich miestach.
        if (g_settings.hide_zeros) {
            int last_int = 4 - decimals;             // index poslednej celej číslice (bodka je za ňou)
            int i = 1;
            while (i < last_int && digits[i] == '0') digits[i++] = ' ';
            if (neg && i > 1) {
                digits[0]     = ' ';
                digits[i - 1] = '-';
            }
        }
    }
    for (int i = 0; i < 5; i++) {
        if (display_dirty || digits[i] != last_digits[i]) {
            LCD_Draw7SegDigit(dig_x0 + i * digit_step, DIGITS_Y, digits[i], txt_color, LCD_BLACK);
            last_digits[i] = digits[i];
        }
    }
    if (dot) {
        LCD_Draw7SegDot(dig_x0 + (dot * digit_step) - 9, DIGITS_Y, txt_color, LCD_BLACK);
    }

    // --- Texty: horný riadok, AC/DC a jednotka na okraji ---
    char tmp[24], buf[24];
    if (label[0] != '\0') {
        snprintf(tmp, sizeof(tmp), "%s%s %s %s", range_mode_str, rel_active ? " REL" : "", label, unit);
    } else {
        // dióda, Hz, % – iba AUTO (+ REL), jednotka je vpravo pri číslach
        snprintf(tmp, sizeof(tmp), "%s%s", range_mode_str, rel_active ? " REL" : "");
    }
    snprintf(buf, sizeof(buf), "%-18s", tmp);       // v0.76: 18 znakov (x 20..235), vpravo MIN/MAX
    DrawTextIfChanged(last_top, 20, 45, buf, LCD_WHITE);

    // AC/DC alebo ikona (dióda, pípak) na tom istom mieste
    char acdc_key[4];
    if (d.icon) { acdc_key[0] = (char)d.icon; acdc_key[1] = '\0'; }
    else        { strncpy(acdc_key, acdc, 3); acdc_key[3] = '\0'; }
    if (display_dirty || strcmp(last_acdc, acdc_key) != 0) {
        strcpy(last_acdc, acdc_key);
        LCD_FillRect(unit_x, ACDC_Y, 36, 16, LCD_BLACK);   // zmazať celé miesto (text nekreslí medzery medzi znakmi)
        if (d.icon) LCD_DrawIcon(unit_x, ACDC_Y, d.icon, LCD_CYAN, LCD_BLACK, 2);
        else        LCD_DrawString(unit_x, ACDC_Y, acdc, (acdc[0] == 'A') ? LCD_ORANGE : LCD_GREEN, LCD_BLACK, 2);
    }

    // Jednotka: až 3 znaky (kHz, MHz); pri zmene celé miesto zmazať (aj medzery medzi znakmi)
    if (display_dirty || strcmp(last_unit, unit) != 0) {
        snprintf(last_unit, sizeof(last_unit), "%s", unit);
        LCD_FillRect(unit_x, UNIT_Y, 36, 16, LCD_BLACK);
        LCD_DrawString(unit_x, UNIT_Y, unit, txt_color, LCD_BLACK, 2);
    }

    display_dirty = false;
}

// ---------------------------------------------------------------------------
// v0.76: MIN/MAX – dva malé riadky vpravo od riadku rozsahu "AUTO 6.000 V" (x 240..311, y 45 a 54)
// ---------------------------------------------------------------------------
static void MinMaxText(const MeterMemVal_t *v, char *b, size_t n) {
    if (!v->used) { snprintf(b, n, "----"); return; }
    if (v->ol)    { snprintf(b, n, "OL"); return; }
    char r[12], t[12];
    uint8_t k = 0, dec = (v->decimals > 4u) ? 4u : v->decimals;
    uint16_t m = (v->mag > 9999u) ? 9999u : v->mag;
    for (uint8_t i = 0; i < dec; i++) { r[k++] = (char)('0' + m % 10u); m = (uint16_t)(m / 10u); }
    if (dec) r[k++] = '.';
    do { r[k++] = (char)('0' + m % 10u); m = (uint16_t)(m / 10u); } while (m);
    if (v->neg) r[k++] = '-';
    for (uint8_t i = 0; i < k; i++) t[i] = r[k - 1u - i];
    t[k] = '\0';
    snprintf(b, n, "%.7s%.3s", t, v->unit);
}

#define MINMAX_X  240
#define MINMAX_Y  45
static void Display_MinMax(void) {
    static uint32_t se = 0, ep = 0;
    static bool     shown = false;
    if (se != screen_epoch) { se = screen_epoch; ep = 0; shown = false; }
    if (!mm_on) {
        if (shown) { LCD_FillRect(MINMAX_X - 2, MINMAX_Y - 1, 78, 18, LCD_BLACK); shown = false; }
        return;
    }
    if (shown && ep == mm_epoch) return;
    ep = mm_epoch;
    shown = true;
    char v[16], t[16];
    MinMaxText(&mm_max, v, sizeof(v));
    snprintf(t, sizeof(t), "%-8.8s", v);
    LCD_DrawString(MINMAX_X, MINMAX_Y, "MAX", LCD_ORANGE, LCD_BLACK, 1);
    LCD_DrawString(MINMAX_X + 24, MINMAX_Y, t, LCD_WHITE, LCD_BLACK, 1);
    MinMaxText(&mm_min, v, sizeof(v));
    snprintf(t, sizeof(t), "%-8.8s", v);
    LCD_DrawString(MINMAX_X, MINMAX_Y + 9, "MIN", LCD_CYAN, LCD_BLACK, 1);
    LCD_DrawString(MINMAX_X + 24, MINMAX_Y + 9, t, LCD_WHITE, LCD_BLACK, 1);
}

// ---------------------------------------------------------------------------
// v0.74: HV warning (menu F1 "HV warning: OFF / ON / ON+BEEP")
// V polohe nad 30 V AC rms alebo 60 V DC (hranice dotykovo bezpečného napätia podľa IEC 61010)
// svieti červený blesk, pri ON+BEEP pípne každé 2 s. Vyhodnocuje sa aj pri HOLD a REL
// (skutočné napätie z čipu).
// v0.75: blesk menší (16x24 px), plný, bez blikania (svieti, kým je napätie nad hranicou).
// Je vpravo od veľkých číslic v stĺpci AC/DC a jednotky (x 280..315): voľný pás medzi spodkom
// AC/DC (y = ACDC_Y + 13) a vrchom jednotky (y = UNIT_Y) má 36 riadkov, blesk je presne v strede
// (6 riadkov nad aj pod), vodorovne v strede pod nápisom "AC"/"DC" (x 280..301).
// Nič iné sa tu nekreslí – hodnota ani texty sa neposúvajú ani neprekrývajú.
// ---------------------------------------------------------------------------
#define HV_AC_LIMIT    300000LL      // 30 V v 0.1 mV
#define HV_DC_LIMIT    600000LL      // 60 V
#define HV_OFF_PCT     95            // hysteréza: výstraha zhasne pod 95 % hranice
#define HV_STALE_MS    1500u         // bez novej vzorky -> výstraha preč
#define HV_BEEP_PERIOD 2000u         // ON+BEEP: pípnutie každé 2 s
#define HV_BEEP_MS     150u
#define HV_W           16
#define HV_H           24
#define HV_X           282                                                // tvar x 2..15 -> 284..297, stred pod "AC" (280..301)
#define HV_Y           ((ACDC_Y + 14 + UNIT_Y - HV_H) / 2)                // 113: AC/DC kreslí y 93..106, jednotka od 143

static bool     hv_on;               // napätie nad hranicou
static uint32_t hv_t0;               // prvá vzorka nad hranicou (začiatok 2 s cyklov pípania)
static uint32_t hv_seen;             // posledná vzorka nad hranicou

static void Hv_Sample(const MeasSample_t *s, const Format_t *f) {
    if (cur_pos != SWITCH_POS_V || g_settings.hv_warn == HV_WARN_OFF) { hv_on = false; return; }
    bool ac  = IsAcMode(ROTARY_GetChipCommand(cur_pos, cur_sub));
    int64_t lim = ac ? HV_AC_LIMIT : HV_DC_LIMIT;
    bool over;
    if (s->overload) {
        over = !IsManual() || f->exp >= 2;           // AUTO: nad 1000 V; manuálny 60 V a vyšší rozsah
    } else {
        int64_t a = (s->value < 0) ? -(int64_t)s->value : s->value;
        a *= Pow10(f->exp);                          // 0.1 mV
        over = hv_on ? (a * 100 >= lim * HV_OFF_PCT) : (a >= lim);
    }
    uint32_t now = HAL_GetTick();
    if (over) {
        if (!hv_on) { hv_on = true; hv_t0 = now; }
        hv_seen = now;
    } else {
        hv_on = false;
    }
}

// Blesk 16x24: plný mnohouholník, každý riadok = jedna vodorovná čiara (od-do)
static void Hv_DrawBolt(uint16_t color) {
    static const uint8_t L[HV_H][2] = {
        { 7, 15}, { 7, 15}, { 6, 14}, { 6, 13}, { 5, 12}, { 5, 11}, { 4, 11}, { 4, 10},
        { 3,  9}, { 3, 15}, { 2, 15}, { 2, 14}, { 8, 13}, { 8, 12}, { 7, 11}, { 7, 10},
        { 6,  9}, { 6,  9}, { 5,  8}, { 5,  7}, { 4,  7}, { 4,  6}, { 3,  5}, { 3,  4},
    };
    for (int r = 0; r < HV_H; r++)
        LCD_DrawFastHLine((uint16_t)(HV_X + L[r][0]), (uint16_t)(HV_Y + r),
                          (uint16_t)(L[r][1] - L[r][0] + 1), color);
}

// Volať v každom prechode meracej obrazovky (po Display_Measurement)
static void Display_HvWarn(void) {
    static uint32_t epoch = 0;
    static bool     shown = false;
    static uint32_t beep_cycle = 0xFFFFFFFFu;
    uint32_t now = HAL_GetTick();

    if (epoch != screen_epoch) { epoch = screen_epoch; shown = false; }    // obrazovka zmazaná

    if (hv_on && (cur_pos != SWITCH_POS_V || g_settings.hv_warn == HV_WARN_OFF || now - hv_seen > HV_STALE_MS))
        hv_on = false;

    if (hv_on && !shown) {
        Hv_DrawBolt(LCD_RED);
        shown = true;
    } else if (!hv_on && shown) {
        LCD_FillRect(HV_X, HV_Y, HV_W, HV_H, LCD_BLACK);   // iba pás blesku (nič iné tu nie je)
        shown = false;
    }

    if (hv_on && g_settings.hv_warn == HV_WARN_BEEP) {
        uint32_t cyc = (now - hv_t0) / HV_BEEP_PERIOD;
        if (cyc != beep_cycle) { beep_cycle = cyc; Beep_Start(HV_BEEP_MS); }   // nezávisle od "Beep"
    } else {
        beep_cycle = 0xFFFFFFFFu;
    }
}

// ---------------------------------------------------------------------------
// v0.78: Diode beep – v polohe dióda nepretržité pípanie, kým je úbytok < 0.05 V (skrat;
// skratované hroty ukazujú 0.001 V). Normálna dióda (~0.2 V a viac) ani OL nepípa.
// Nastavenie "Diode beep" (menu F1), predvolene ON, nezávisle od "Beep".
// ---------------------------------------------------------------------------
#define DIODE_SHORT_MV     50        // RANGES_DIODE: 3 desatinné miesta -> 1 digit = 1 mV
#define DIODE_STALE_MS     600u      // bez novej vzorky pod hranicou -> ticho
#define DIODE_BEEP_MS      40u       // Beep_Start sa obnovuje v každom prechode slučky -> súvislý tón

static bool     diode_short;
static uint32_t diode_seen;
static uint32_t diode_epoch;

static bool DiodeBeepActive(void) {
    return g_settings.diode_beep && cur_pos == SWITCH_POS_OHM_DIODE_CONT && cur_sub == SUB_MODE_DIODE;
}

static void Diode_Sample(const MeasSample_t *s, const Format_t *f) {
    if (!DiodeBeepActive() || diode_epoch != mode_epoch) {
        diode_epoch = mode_epoch;
        diode_short = false;
        if (!DiodeBeepActive()) return;
    }
    int32_t v = s->value;
    if (v < 0) v = -v;
    diode_short = !SampleIsOL(s, f) && v < DIODE_SHORT_MV;
    if (diode_short) diode_seen = HAL_GetTick();
}

static void Diode_BeepTask(void) {
    if (!diode_short) return;
    if (!DiodeBeepActive() || diode_epoch != mode_epoch || HAL_GetTick() - diode_seen > DIODE_STALE_MS) {
        diode_short = false;
        return;
    }
    Beep_Start(DIODE_BEEP_MS);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  // Príznaky resetu (zobudenie = softvérový reset, prepínač = zapnutie napájania), potom vymazať
  uint32_t reset_flags = RCC->CSR;
  RCC->CSR |= RCC_CSR_RMVF;

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  ScopeTest_InitPins();                  // relé a prepínače osciloskopu do 0, ADC MS9280 v STBY
  MX_USART6_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  LCD_PWR_PORT->BSRR = (1 << (LCD_PWR_PIN + 16));
  HAL_Delay(250);

  // Napájanie a inicializácia displeja
  LCD_Power_Accessories();
  LCD_Init_Hardware();

  // Stav ovládača čipu + spustenie príjmu cez prerušenie
  MeasChip_Init();
  HAL_UART_Receive_IT(&huart6, &rx_byte, 1);

  LCD_Init();
  BUTTON_Init();
  Settings_Init();
  Battery_Init();                        // ADC1: PA7 (U_bat, delič 10k/10k) + VREFINT
  EEPROM_Init();                         // I2C1 + EEPROM (po zapnutí PA2, ak je napájaná z neho)
  if (EeMigrate_Needed()) {              // v0.72: jednorazový presun dát z rozloženia v0.71 (ee_migrate.c)
      const char *t = "EEPROM UPDATE...";
      LCD_DrawString((uint16_t)((320 - ((int)strlen(t) * 12 - 2)) / 2), 40, t, LCD_ORANGE, LCD_BLACK, 2);
      bool mig_ok = EeMigrate_Run();
      t = mig_ok ? " EEPROM UPDATED " : "EEPROM UPD ERROR";
      LCD_DrawString((uint16_t)((320 - ((int)strlen(t) * 12 - 2)) / 2), 40, t, mig_ok ? LCD_GREEN : LCD_RED, LCD_BLACK, 2);
  }
  Settings_Load();                       // užívateľské nastavenia z EEPROM (pri chybe predvolené)
  ApplyDefaultSubs();                    // predvolené režimy polôh (dlhé SAVE)
  ApplySavedRanges();                    // v0.70: uložené rozsahy (Save range ON)
  Settings_GenToRam();                   // posledné nastavenie generátora (výstup vypnutý)
  Gen_SetCal(g_settings.gen_cal);        // kalibrácia amplitúdy generátora (nastavenia generátora)

  // Typ štartu: zobudenie zo spánku (softvérový reset) alebo zapnutie prepínačom (napájanie)
  bool woke = (reset_flags & RCC_CSR_SFTRSTF) && !(reset_flags & (RCC_CSR_PORRSTF | RCC_CSR_BORRSTF));
  bl_level = Settings_BlPercent(g_settings.start_bl);   // jas po zapnutí prepínačom
  bool resumed = Resume_Restore(woke);   // po zobudení: režimy, rozsahy, jas, generátor z RAM
  Scope_LoadEeprom(!resumed);            // kalibrácia osciloskopu; po zapnutí aj posledné nastavenie (dlhé SAVE)
  LCD_BL_SetBrightness(bl_level);
  if (resumed && resume_gen_on) Gen_SetOutput(true);    // mimo polohy V ho vypne prvé čítanie prepínača

  LCD_DrawString(85, 200, "Firmware v0.78", LCD_YELLOW, LCD_BLACK, 2);
  {   // Typ štartu, vodorovne v strede: veľký nadpis + menší popis
      const char *t1 = resumed ? "WAKE UP" : "POWER ON";
      const char *t2 = resumed ? "last state restored" : "saved settings loaded";
      int w1 = (int)strlen(t1) * 12 - 2, w2 = (int)strlen(t2) * 6 - 1;
      LCD_DrawString((uint16_t)((320 - w1) / 2), 136, t1, LCD_CYAN, LCD_BLACK, 2);
      LCD_DrawString((uint16_t)((320 - w2) / 2), 158, t2, LCD_LIGHTGREY, LCD_BLACK, 1);
  }
  LCD_DrawString(10, 100, "INITIALIZING MEAS CHIP...", LCD_WHITE, LCD_BLACK, 2);

  // Čakanie na uvítací paket A0 01 A1
  while (!MeasChip.chip_present) {
      HAL_Delay(100);
  }
  LCD_DrawString(10, 100, "   MEAS CHIP BOOTED OK     ", LCD_GREEN, LCD_BLACK, 2);
  HAL_Delay(900);

  LCD_Clear(LCD_BLACK);
  last_activity_tick = HAL_GetTick();   // štart odpočtu automatického vypnutia

  /* USER CODE END 2 */



  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      BUTTON_Poll();                            // pustenie tlačidiel (zvýraznenie, dlhé F1, opakovanie)
      Beep_Task();                              // koniec pípnutia (beží na pozadí)

      // Čip prestal odpovedať -> raz dlhé pípnutie (iba na meracej obrazovke, kde sa hláška ukáže)
      {
          static bool prev_err = false;
          bool err_now = MeasChip.comm_error && !gen_open && !scope_open && !menu_open;
          if (err_now && !prev_err) Beep_Error();
          prev_err = err_now;
      }
      ButtonType_t btn = BUTTON_GetEvent();
      if (btn != BTN_NONE) {
          NoteActivity();                       // reštart odpočtu (ak je zapnutý Auto reset time)
      }
      // Tlačidlo podsvietenia funguje vždy: krátke = jas, dlhé = generátor
      bool bl_long = false;
      if (btn == BTN_BL) {
          bl_pending = true;
          bl_tick = HAL_GetTick();
          btn = BTN_NONE;
      }
      if (bl_pending) {
          if (BUTTON_GetHeld() == BTN_BL) {
              if (HAL_GetTick() - bl_tick >= Settings_LongPressMs()) {
                  bl_pending = false;
                  bl_long = true;
              }
          } else {
              bl_pending = false;
              HandleButton(BTN_BL);                 // krátke stlačenie -> jas
          }
      }
      // Dlhé BL: generátor (iba poloha V). Z osciloskopu sa otvorí generátor a dlhé BL v ňom
      // vráti späť do osciloskopu; generátor otvorený z voltmetra sa vracia na voltmeter.
      if (bl_long && !menu_open && !mem_open) {      // v0.71: v pamäti hodnôt dlhé BL nerobí nič
          if (gen_open) {                           // odchod z generátora (výstup beží ďalej)
              gen_open = false;
              GenMenu_Close();                      // nastavenie ostáva v RAM (do EEPROM iba SAVE)
              if (gen_from_scope) {
                  gen_from_scope = false;
                  scope_open = true;
                  ScopeEnter();
                  screen_epoch++;
              } else {
                  ReturnToMeasScreen();
              }
              last_activity_tick = HAL_GetTick();   // nový odpočet
          } else if (cur_pos == SWITCH_POS_V) {     // generátor iba v polohe V
              gen_from_scope = scope_open;
              if (scope_open) {
                  scope_open = false;
                  ScopeLeave(true);                 // stav osciloskopu sa zachová
              }
              gen_open = true;                      // nastavenie z RAM (EEPROM sa číta iba pri štarte)
              f1_pending = false;
              auto_pending = false;
              GenMenu_Open(gen_from_scope);
              screen_epoch++;                       // horný riadok (čas, nadpis, batéria) znova
              last_activity_tick = HAL_GetTick();   // odpočet podľa času generátora
              gen_beep_tick = HAL_GetTick();
          } else {
              hint_text = (cur_pos == SWITCH_POS_mV) ? "GEN: NOT IN mV" : "GEN: SWITCH TO V";
              gen_hint_until = HAL_GetTick() + GEN_HINT_MS;
              Beep_Warning();                       // upozornenie: krátke pípnutie
          }
      }

      // --- Komunikácia s čipom beží vždy (aj v menu) ---
      MeasChip_Task();
      SyncRangeState();
      AutoOff_Task();
      Battery_Task();                           // meranie batérie každé 2 s

      // ===================== MENU =====================
      if (menu_open) {
          uint32_t now_m = HAL_GetTick();
          last_activity_tick = now_m;              // odpočet vypnutia v menu stojí
          if (btn != BTN_NONE || BUTTON_GetHeld() != BTN_NONE) {
              menu_input_tick = now_m;
          } else if (now_m - menu_input_tick >= MENU_IDLE_MS) {
              btn = BTN_F1;                        // 2 min bez dotyku -> ako EXIT (uloží nastavenia)
          }
          if (!Menu_Task(btn)) {
              // Návrat z menu: prekresliť celú meraciu obrazovku, odpočet podľa nového nastavenia
              menu_open = false;
              // Rýchle tlačidlá R/D/C: v odpore nie je HOLD -> zrušiť, ak ostal zapnutý
              if (QuickRdc() && cur_sub == SUB_MODE_RESISTANCE) is_hold_active = false;
              ReturnToMeasScreen();
              last_activity_tick = HAL_GetTick();
          }
#if !DEBUG_SCREEN
          else {
              Display_Timer();                     // odpočet do zatvorenia menu
              Display_Battery();
          }
#endif
          MeasSample_t dummy;
          if (MeasChip_GetSample(&dummy) && MeasChip.init_state == INIT_STEP_READY) {
              Format_t chk;
              if (GetFormat(&dummy, &chk)) { last_sample = dummy; last_sample_ok = true; }
          }
          HAL_Delay(2);
          continue;
      }

      // --- 1. Otočný prepínač (odrušený) ---
      // Výstup generátora je na svorke iba v polohe V -> mimo nej sa vypne
      // a obrazovka generátora sa zavrie.
      SwitchPosition_t new_pos;
      if (ROTARY_PollStable(&new_pos)) {
          pos_known = true;
          cur_pos = new_pos;
          cur_sub = pos_sub[cur_pos % NUM_POSITIONS];   // posledná voľba užívateľa v tejto polohe
          is_hold_active = false;          // otočenie vždy ruší HOLD
          f1_pending = false;
          NoteActivity();
          ApplyMode();
          if (cur_pos != SWITCH_POS_V) {
              if (gen_open) GenMenu_Close();        // najprv ukončiť prípadnú kalibráciu (obnoví nastavenie)
              if (Gen_IsOn()) {                     // vypnúť, ale zapamätať (RAM) -> po návrate do V zapnúť
                  Gen_SetOutput(false);
                  gen_resume = true;
              }
              if (gen_open) {
                  gen_open = false;
                  GenMenu_Close();                  // nastavenie ostáva v RAM
                  if (gen_from_scope && cur_pos == SWITCH_POS_mV) {   // generátor z osciloskopu -> späť
                      scope_open = true;
                      ScopeEnter();
                      screen_epoch++;
                  } else {
                      ReturnToMeasScreen();
                  }
                  gen_from_scope = false;
                  last_activity_tick = HAL_GetTick();
                  btn = BTN_NONE;                   // stlačenie patrilo generátoru
              }
              if (scope_open && cur_pos != SWITCH_POS_mV) {   // osciloskop (test) iba v polohe V alebo mV
                  scope_open = false;
                  ScopeLeave(false);
                  ReturnToMeasScreen();
                  last_activity_tick = HAL_GetTick();
                  btn = BTN_NONE;
              }
          } else if (gen_resume) {                  // späť v polohe V: výstup znova zapnúť
              gen_resume = false;
              Gen_SetOutput(true);
          }
      }

      // Po zobudení: obnoviť obrazovku generátora / osciloskopu (keď je poloha prepínača známa)
      if (resume_screen != RES_SCR_MEAS && pos_known) {
          if (resume_screen == RES_SCR_GEN && cur_pos == SWITCH_POS_V) {
              gen_from_scope = resume_from_scope;
              gen_open = true;
              GenMenu_Open(gen_from_scope);
              if (Gen_IsOn() != resume_gen_on) Gen_SetOutput(resume_gen_on);   // nie "Auto output ON"
              screen_epoch++;
              last_activity_tick = HAL_GetTick();
              gen_beep_tick = HAL_GetTick();
          } else if (resume_screen == RES_SCR_SCOPE &&
                     (cur_pos == SWITCH_POS_V || cur_pos == SWITCH_POS_mV)) {
              scope_open = true;
              scope_test = false;
              ScopeEnter();
              screen_epoch++;
              last_activity_tick = HAL_GetTick();
              gen_beep_tick = HAL_GetTick();
          }
          resume_screen = RES_SCR_MEAS;
          continue;
      }

      // Kalibrácia generátora skončila (alebo sa generátor zavrel) -> vrátiť pôvodný režim polohy V
      if (cal_forced && !(gen_open && GenMenu_CalActive())) {
          cal_forced = false;
          if (cur_pos == SWITCH_POS_V) SelectSubMode(cal_saved_sub);
          else                         pos_sub[SWITCH_POS_V] = cal_saved_sub;
      }

      // Kalibrácia osciloskopu (1:10 / 1:100) skončila -> vrátiť pôvodný režim polohy V
      if (scal_forced && !(scope_open && !scope_test && Scope_MeterWanted())) {
          scal_forced = false;
          if (cur_pos == SWITCH_POS_V) SelectSubMode(scal_saved_sub);
          else                         pos_sub[SWITCH_POS_V] = scal_saved_sub;
      }

      // ================== GENERÁTOR ==================
      // Meranie beží ďalej na pozadí (vzorky sa iba ukladajú), odpočet vypnutia beží normálne.
      if (gen_open) {
          GenMenu_Task(btn);
          // Menu MEMORY: odpočet vypnutia stojí (ako menu nastavení). Po zatvorení (EXIT, LOAD
          // alebo 2 min nečinnosti) tak odpočet generátora beží od začiatku.
          if (GenMenu_IsMenuOpen()) last_activity_tick = HAL_GetTick();
#if !DEBUG_SCREEN
          Display_Timer();
          Display_Title();
          Display_Battery();
#endif
          // Kalibrácia (SETTINGS, CAL): čip meria AC V na vstupe V, hodnota ide do generátora
          if (GenMenu_CalActive() && cur_sub != SUB_MODE_AC) {
              if (!cal_forced) { cal_forced = true; cal_saved_sub = cur_sub; }
              SelectSubMode(SUB_MODE_AC);
          }
          MeasSample_t gs;
          if (MeasChip_GetSample(&gs) && MeasChip.init_state == INIT_STEP_READY) {
              Format_t chk;
              if (GetFormat(&gs, &chk)) {
                  last_sample = gs; last_sample_ok = true;
                  if (GenMenu_CalActive() && cur_sub == SUB_MODE_AC) {
                      bool ok = !SampleIsOL(&gs, &chk) && gs.value >= 0;
                      int64_t v01 = (int64_t)gs.value * Pow10(chk.exp);       // V: základ 0.1 mV
                      GenMenu_SetCalReading(ok && v01 < 0x7FFFFFFF, (int32_t)v01);
                  }
              }
          }
          HAL_Delay(2);
          continue;
      }

      // ============ OSCILOSKOP ============
      // Generátor môže bežať ďalej (vlastný signál na vstup V). Odpočet podľa Gen/Scope OFF.
      if (scope_open) {
          bool stay = true;
          if (scope_test) {
              if (!ScopeTest_Task(btn)) {           // dlhé AUTO v teste = späť do osciloskopu
                  ScopeTest_Close(false);
                  scope_test = false;
                  Scope_Open(cur_pos == SWITCH_POS_mV);
                  screen_epoch++;
              }
          } else {
              ScopeAction_t a = Scope_Task(btn, cur_pos == SWITCH_POS_mV);
              if (a == SCOPE_EXIT) {                // dlhé AUTO = odchod na voltmeter
                  Scope_Close();
                  scope_open = false;
                  ReturnToMeasScreen();
                  last_activity_tick = HAL_GetTick();
                  stay = false;
              } else if (a == SCOPE_TEST) {         // dlhé F1 = testovací režim
                  Scope_Close();
                  scope_test = true;
                  ScopeTest_Open();
                  screen_epoch++;
              }
          }
          if (stay) {
#if !DEBUG_SCREEN
              Display_Timer();
              Display_Title();
              Display_Battery();
#endif
          }
          // Kalibrácia deliča 1:10 / 1:100: čip meria DC V na vstupe V, hodnota ide do osciloskopu
          bool want_dc = !scope_test && Scope_MeterWanted() && cur_pos == SWITCH_POS_V;
          if (want_dc && cur_sub != SUB_MODE_DC) {
              if (!scal_forced) { scal_forced = true; scal_saved_sub = cur_sub; }
              SelectSubMode(SUB_MODE_DC);
          }
          MeasSample_t ss;
          if (MeasChip_GetSample(&ss) && MeasChip.init_state == INIT_STEP_READY) {
              Format_t chk;
              if (GetFormat(&ss, &chk)) {
                  last_sample = ss; last_sample_ok = true;
                  if (want_dc && cur_sub == SUB_MODE_DC) {
                      bool ok = !SampleIsOL(&ss, &chk);
                      int64_t v01 = (int64_t)ss.value * Pow10(chk.exp);       // V: základ 0.1 mV
                      Scope_SetMeterReading(ok && v01 < 0x7FFFFFFF && v01 > -0x7FFFFFFF, (int32_t)v01);
                  }
              }
          }
          HAL_Delay(2);
          continue;
      }

      // ============ PAMÄŤ NAMERANÝCH HODNÔT (v0.71) ============
      // Meranie beží ďalej (NOW = hodnota, ktorá by bola na veľkom displeji; pri HOLD zmrazená).
      if (mem_open) {
          MeasSample_t ms;
          if (MeasChip_GetSample(&ms) && MeasChip.init_state == INIT_STEP_READY) {
              Format_t chk;
              if (GetFormat(&ms, &chk)) {
                  last_sample = ms; last_sample_ok = true;
                  if (!is_hold_active) {
                      Disp_t d;
                      if (ComputeDisp(&ms, &d)) NoteDisp(&d);
                  }
              }
          }
          if (!MeterMem_Task(btn, &cur_disp, is_hold_active, mm_on ? &mm_max : NULL, mm_on ? &mm_min : NULL)) {
              mem_open = false;                     // F1 EXIT
              ReturnToMeasScreen();
              continue;
          }
#if !DEBUG_SCREEN
          Display_Timer();
          Display_Title();
          Display_Battery();
#endif
          HAL_Delay(2);
          continue;
      }

      // --- 2. Tlačidlá ---
      // AUTO: krátke (pri pustení) = automatický rozsah, dlhé (1 s) = osciloskop (poloha V alebo mV)
      if (btn == BTN_AUTO) {
          auto_pending = true;
          auto_tick = HAL_GetTick();
          btn = BTN_NONE;
      }
      if (auto_pending) {
          if (BUTTON_GetHeld() == BTN_AUTO) {
              if (HAL_GetTick() - auto_tick >= Settings_LongPressMs()) {
                  auto_pending = false;
                  if (cur_pos == SWITCH_POS_V || cur_pos == SWITCH_POS_mV) {
                      scope_open = true;
                      scope_test = false;
                      f1_pending = false;
                      save_pending = false;
                      is_hold_active = false;
                      ScopeEnter();
                      screen_epoch++;               // horný riadok (čas, nadpis, batéria) znova
                      last_activity_tick = HAL_GetTick();
                      gen_beep_tick = HAL_GetTick();
                      continue;                     // z osciloskopu dlhé AUTO vráti na voltmeter
                  }
                  hint_text = "SCOPE: SWITCH TO V/mV";
                  gen_hint_until = HAL_GetTick() + GEN_HINT_MS;
                  Beep_Warning();                       // upozornenie: krátke pípnutie
              }
          } else {
              auto_pending = false;
              HandleButton(BTN_AUTO);               // krátke stlačenie -> AUTO rozsah
              menu_dirty = true;
          }
      }

      // SAVE: dlhé = predvolený režim polohy, krátke = v0.71 pamäť nameraných hodnôt (pri pustení)
      if (btn == BTN_SAVE) {
          save_pending = true;
          save_tick = HAL_GetTick();
          btn = BTN_NONE;
      }
      if (save_pending) {
          if (BUTTON_GetHeld() == BTN_SAVE) {
              if (HAL_GetTick() - save_tick >= Settings_LongPressMs()) {
                  save_pending = false;
                  SaveDefaultSub();
              }
          } else {
              save_pending = false;                 // krátke SAVE -> pamäť nameraných hodnôt
              mem_open = true;
              f1_pending = false;
              auto_pending = false;
              MeterMem_Open();
              screen_epoch++;                       // horný riadok (čas, nadpis, batéria) znova
              continue;
          }
      }

      // v0.76: F2 (HOLD) pri pustení, dlhé = MIN/MAX zap. / vyp.
      if (btn == BTN_F2 && F2IsHold()) {
          f2_pending = true;
          f2_tick = HAL_GetTick();
          btn = BTN_NONE;
      }
      if (f2_pending) {
          if (BUTTON_GetHeld() == BTN_F2) {
              if (HAL_GetTick() - f2_tick >= Settings_LongPressMs()) {
                  f2_pending = false;
                  ToggleMinMax();
              }
          } else {
              f2_pending = false;
              HandleButton(BTN_F2);                 // krátke = HOLD
          }
      }

      if (btn == BTN_F1) {
          // F1: zatiaľ nič – krátke stlačenie = REL pri pustení, dlhé = menu
          f1_pending = true;
          f1_tick = HAL_GetTick();
      } else if (btn != BTN_NONE) {
          HandleButton(btn);
      }

      if (f1_pending) {
          if (BUTTON_GetHeld() == BTN_F1) {
              if (HAL_GetTick() - f1_tick >= Settings_LongPressMs()) {
                  f1_pending = false;
                  menu_open = true;
                  menu_input_tick = HAL_GetTick();
                  Menu_Open();
                  screen_epoch++;                  // odpočet vľavo hore nakresliť znova
                  continue;
              }
          } else {
              f1_pending = false;
              HandleButton(BTN_F1);         // krátke stlačenie -> REL
          }
      }

      // Zvýraznenie SELECT / RANGE iba kým je tlačidlo držané
      {
          static int8_t last_pressed = -1;
          ButtonType_t held = BUTTON_GetHeld();
          int8_t pressed = (held == BTN_F3) ? 2 : (held == BTN_F4) ? 3 :
                           (held == BTN_F2 && QuickRdc() && cur_sub == SUB_MODE_RESISTANCE) ? 1 : -1;
          if (pressed != last_pressed) {
              last_pressed = pressed;
              pressed_cell = pressed;
              menu_dirty = true;
          }
      }

      // --- 3. Displej ---
      if (menu_dirty) {
          menu_dirty = false;
          LCD_DrawBottomMenu(cur_pos, cur_sub, rel_active, pressed_cell);
      }

#if DEBUG_SCREEN
      Display_Status();
      Display_Sniffer();
#else
      Display_Timer();
      Display_Title();
      Display_Battery();
      Display_RelInfo();
      Display_GenIndicator();
      if (relinfo_redraw) Display_RelInfo();   // v0.75: GEN riadok sa presunul cez riadok REL
      Display_MinMax();                        // v0.76
#endif

      MeasSample_t sample;
      if (MeasChip_GetSample(&sample)) {
          // Vzorky mimo READY môžu byť ešte zo starého módu -> zahodiť.
          if (MeasChip.init_state == INIT_STEP_READY) {
              Format_t chk;
              if (GetFormat(&sample, &chk)) {
                  last_sample    = sample;       // pre REL aj počas HOLD
                  last_sample_ok = true;
                  Hv_Sample(&sample, &chk);      // v0.74: HV warning (aj pri HOLD)
                  Diode_Sample(&sample, &chk);   // v0.78: Diode beep (aj pri HOLD)

                  // Probe reset: v Ω / dióde / pípaku priloženie hrotov (OL -> hodnota) reštartuje odpočet
                  static bool prev_ol = false;
                  static uint32_t prev_epoch_mode = 0;
                  if (prev_epoch_mode != mode_epoch) { prev_epoch_mode = mode_epoch; prev_ol = false; }
                  bool ol = SampleIsOL(&sample, &chk);
                  if (cur_pos == SWITCH_POS_OHM_DIODE_CONT && g_settings.auto_reset && g_settings.probe_reset &&
                      prev_ol && !ol) {                                   // v0.78: iba pri Auto reset time ON
                      last_activity_tick = HAL_GetTick();
                  }
                  prev_ol = ol;

                  // Probe reset s prahom (z hodnoty čipu pred REL; OL = nad každým prahom):
                  //   Cx:                   kapacita v pF nad "Probe C reset"
                  //                         (rozpojené hroty ukazujú len kapacitu káblov ~30 pF)
                  //   AC V, AC mV:          digity displeja nad "Probe V-AC reset"
                  //   DC V/mV, DC/AC A:     digity displeja nad "Probe V,A reset" (absolútna hodnota)
                  {
                      uint64_t level = 0;
                      uint32_t thr   = 0;
                      if (cur_pos == SWITCH_POS_Cx) {
                          thr   = Settings_ProbeCPf(g_settings.probe_c);
                          level = (uint64_t)((sample.value < 0) ? 0 : sample.value) * (uint64_t)Pow10(chk.exp);
                      } else if (IsVoltCurrentPos(cur_pos)) {
                          bool vac = (cur_pos == SWITCH_POS_V || cur_pos == SWITCH_POS_mV) &&
                                     IsAcMode(ROTARY_GetChipCommand(cur_pos, cur_sub));
                          thr   = vac ? Settings_ProbeVacCounts(g_settings.probe_vac)
                                      : Settings_ProbeVaCounts(g_settings.probe_va);
                          level = (uint64_t)((sample.value < 0) ? -(int64_t)sample.value : sample.value);
                      }
                      if (ol) level = UINT64_MAX;
                      ProbeLevel_Task(level, thr);
                  }
              }
              if (!is_hold_active) {
                  Display_Measurement(&sample);
              }
#if CX_RAW_INFO && !DEBUG_SCREEN
              {
                  static char last_raw[40] = "";
                  char raw[40] = "";
                  if (cur_pos == SWITCH_POS_Cx) {
                      snprintf(raw, sizeof(raw), "RAW: value=%ld range=%u%s     ",
                               (long)sample.value, sample.range, sample.overload ? " OL" : "");
                  }
                  if (strcmp(raw, last_raw) != 0) {
                      strcpy(last_raw, raw);
                      LCD_FillRect(2, 172, 250, 10, LCD_BLACK);
                      if (raw[0]) LCD_DrawString(10, 172, raw, LCD_LIGHTGREY, LCD_BLACK, 1);
                  }
              }
#endif
          }
      }

      // Po stlačení REL / návrate z menu hneď prekresliť poslednú hodnotu
      if (redraw_last) {
          redraw_last = false;
          if (last_sample_ok && !is_hold_active) {
              Display_Measurement(&last_sample);
          }
      }
      Display_HvWarn();                  // v0.74
      Diode_BeepTask();                  // v0.78

      HAL_Delay(2);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;   // 50 MHz (TIM6 generátora 100 MHz)
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART6) {
        MeasChip_ProcessByte(rx_byte);

#if DEBUG_SCREEN
        // Debug sniffer – posledných 16 bajtov
        if (debug_rx_index < 16) {
            debug_rx_buffer[debug_rx_index++] = rx_byte;
        } else {
            for (int i = 0; i < 15; i++) {
                debug_rx_buffer[i] = debug_rx_buffer[i + 1];
            }
            debug_rx_buffer[15] = rx_byte;
        }
        debug_rx_count++;
#endif

        HAL_UART_Receive_IT(&huart6, &rx_byte, 1);
    }
}

// Pri pretečení (ORE) HAL ukončí príjem cez IT a zavolá tento callback.
// Bez neho by sa príjem z čipu natrvalo zastavil.
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART6) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        MeasChip_ResetParser();
        HAL_UART_Receive_IT(&huart6, &rx_byte, 1);   // pri neblokujúcej chybe vráti BUSY – nevadí
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
