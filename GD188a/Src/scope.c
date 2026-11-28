/*
 * scope.c
 *
 *  Osciloskop (viď scope.h).
 *
 *  Zachytávanie:
 *    Hodiny ADC: TIM11_CH1 na PC12 (PWM 50 %). DMA požiadavky: TIM1_UP -> DMA2 Stream5 kanál 6,
 *    ktoré číta GPIOC->IDR (dolný bajt = PC0..PC7). Oba časovače bežia zo 100 MHz.
 *      - perióda vzorky P = (čas/div) / 25 px; najmenej 20 taktov (5 Msps)
 *      - P <= 100 taktov: hodiny ADC = vzorkovanie (TIM11 aj TIM1 s periódou P)
 *      - P  > 100 taktov: hodiny ADC bežia 1 MHz, TIM1 (preddelič 100) číta každú (P/100)-tu vzorku.
 *        P je vtedy vždy násobok 100, takže čítanie je stále v rovnakej fáze hodín ADC.
 *    TIM11 sa štartuje s počítadlom v polovici periódy + 1, takže DMA číta tesne po zostupnej hrane
 *    hodín – dáta ADC sú vtedy ustálené (menia sa po nábežnej hrane).
 *    Pri rýchlych časových základniach (pod 5 us/div) je menej vzoriek ako pixelov -> interpolácia
 *    (v0.76 SETUP "Interp": LINE = lineárna, SIN(X)/X = okno Lanczos 8 vzoriek, DOTS = iba vzorky).
 *
 *  v0.76 Peak det (SETUP "Peak det: ON", iba pri P > 100 taktov, t. j. od 50 us/div):
 *    TIM1 číta KAŽDÚ vzorku ADC (1 MHz) do kruhového zásobníka pk_ring (512 B, DMA circular, prerušenie
 *    v polovici a na konci = každých 256 us). Prerušenie zbiera min / max za čas jedného pixelu (P/100
 *    vzoriek) a zapisuje do buf tri polia po PK_MAX: stred (min+max)/2 – s ním pracuje trigger, merania,
 *    FFT a uloženie – a pásmo min / max, ktoré sa kreslí. Záťaž CPU asi 8 %. Pamäť: zachytenie je
 *    obmedzené na PK_MAX = 2720 pixelov (bez Peak det 8192) -> menší rozsah Trig delay / posunu.
 *    ROLL: prerušenie drží priebežné min / max, RollTask ho na hranici pixelu prevezme (namiesto priemeru).
 *    Pod 50 us/div sa číta každá vzorka aj bez Peak det -> nastavenie nemá vplyv.
 *
 *  v0.76 View FFT: 512 vzoriek od začiatku obrazovky (menej = doplnené nulami), Hannovo okno, reálna FFT
 *    cez komplexnú 256-bodovú. x = 0..fs/2 (fs podľa časovej základne), y = 10 dB/dielik, horná čiara =
 *    REF (dBV, podľa V/div zaokrúhlené na 10 dB). "PK" = najsilnejšia frekvencia (parabolická interpolácia).
 *
 *  Spúšťanie (od v0.57):
 *    Zachytí sa asi dvojnásobok šírky obrazovky, hrana sa hľadá programom (s hysterézou asi 1/10 dielika)
 *    a obraz sa posunie tak, aby bola hrana v bode spustenia (1/4, stred, 3/4 šírky).
 *    Poloha hrany sa dopočíta medzi vzorkami (rýchle časové základne bez poskakovania).
 *    AUTO: bez hrany sa obraz obnovuje nespustený; NORM: iba po hrane; SINGLE: po hrane STOP.
 *
 *  Zvislá os:
 *    Mriežka 12 x 8 dielikov po 25 x 22 px. Pre každý rozsah V/div sa zvolí hardvérová kombinácia
 *    (delič 1:1 / 1:10 / 1:100, zosilnenie x1 / x2 / x4). Zosilnenie a nula podľa meraní v0.50.
 *    Kalibrácia príde v kroku 3.
 */

#include <stdio.h>
#include <string.h>

#include "scope.h"
#include "main.h"
#include "lcd.h"
#include "settings.h"
#include "beep.h"
#include "eeprom.h"
#include "gen.h"
#include <stddef.h>

// --- Rozloženie ---
#define PX0        10                    // ľavá čiara mriežky
#define PW         300                   // 12 dielikov po 25 px
#define DIV_X      25
#define PY0        32                    // horná čiara mriežky (v0.63: nad ňou pásik y 28..31 pre značku triggera)
#define PH         176                   // 8 dielikov po 22 px (y 32..208)
#define DIV_Y      22
#define YC         (PY0 + PH / 2)        // stred (0 V pri polohe 0)
#define Y_MIN      (PY0 + 1)
#define Y_MAX      (PY0 + PH - 1)
#define TMARK_X    312                   // značka úrovne triggera vpravo od mriežky

// Menu osciloskopu (rovnaké rozloženie ako nastavenia generátora)
#define SET_ITEMS     9                  // v0.76: 2 stránky (6 + 3)
#define SET_PAGE      6
#define SET_ROW_Y     42
#define SET_ROW_H     20
#define SET_TXT_X     22
#define SET_SEP_Y     168
#define SET_HINT_Y    178
#define SET_HINT_STEP 14

// --- Zachytávanie ---
#define CAP_MAX    8192                  // vzoriek: oneskorenie triggera až asi 25 obrazoviek
#define WARMUP     4                     // prvé vzorky zahodiť (pipeline MS9280 = 3 takty)
#define P_MIN      20u                   // najkratšia perióda vzorky v taktoch (5 Msps)
#define ADC_SLOW   100u                  // hodiny ADC pri pomalom vzorkovaní (1 MHz)
#define SETTLE_RELAY_MS 60u              // po prepnutí relé
#define SETTLE_MS       15u              // po prepnutí U6 / U7 / optočlenov

// --- Piny (rovnaké ako scope_test.c) ---
#define PIN_REL_10   8u                  // PA8
#define PIN_REL_ACDC 9u                  // PA9
#define PIN_STBY     10u                 // PA10
#define PIN_O1O2     15u                 // PA15
#define PIN_U6       10u                 // PC10
#define PIN_U7       11u                 // PC11
#define PIN_ADCCLK   12u                 // PC12 = TIM11_CH1 (AF3)

#define CH_RISE  "\x80"                  // znak nábežnej hrany (font5x7)
#define CH_FALL  "\x81"
#define CH_CHECK "\x82"                  // fajka (menu meraní)
#define COL_RISE  LCD_RED                // farba nábežnej hrany
#define COL_FALL  0x34DF                 // svetlomodrá – dobežná hrana
#define HUNT_MS   25u                    // hľadanie hrany: opakované zachytenia v jednom prechode
#define AUTO_FREE_MS 150u                // AUTO: bez hrany dlhšie -> nespustený obraz
#define MEAS_REFRESH_MS 250u

// Merania (poradie = poradie v menu a v paneli)
enum { MEAS_FREQ = 0, MEAS_PER, MEAS_VPP, MEAS_VMAX, MEAS_VMIN, MEAS_VAVG, MEAS_VRMS,
       MEAS_DUTYP, MEAS_DUTYN, MEAS_RISE, MEAS_FALL, MEAS_COUNT };
static const char *const MEAS_NAME[MEAS_COUNT] = {     // menu
    "Freq", "Period", "Vpp", "Vmax", "Vmin", "Vavg", "Vrms", "Duty+", "Duty-", "Rise", "Fall" };
static const char *const MEAS_LBL[MEAS_COUNT] = {      // na obrazovke (krátko)
    "F", "T", "Vpp", "Max", "Min", "Avg", "Rms", "D+", "D-", "Tr", "Tf" };
#define MEAS_MAX_SEL  4
#define MEAS_ITEMS    (MEAS_COUNT + 1)   // + "Show" na začiatku
#define MEAS_DEFAULT  ((1u << MEAS_FREQ) | (1u << MEAS_VPP) | (1u << MEAS_VAVG) | (1u << MEAS_VRMS))

// Plochy meraní v mriežke
#define PANEL_H    32                    // v0.67: panel všetkých meraní – 3 riadky po 10 px, dole v mriežke
#define PANEL_Y0   (PY0 + PH - PANEL_H)
#define ROW_H      13                    // riadok vybraných meraní, dole v mriežke
#define ROW_Y      (PY0 + PH - ROW_H)
#define MARK_Y0    28                    // pásik nad mriežkou: značka bodu spustenia (y 28..31)

// --- Hardvérové kombinácie: index = delič * 3 + zosilnenie (0 = x1, 1 = x2, 2 = x4) ---
// Kódy ADC na 1 V (x1000) v polohe V a nula (kód x10) pri skratovaných hrotoch
// Predvolené hodnoty (merania v0.50); po kalibrácii z EEPROM (0x0180 / 0x01C0)
static const uint32_t CPV_DEF[9]  = { 12200, 24300, 48800,   1250, 2470, 4900,   125, 250, 500 };
static const uint16_t ZERO_DEF[9] = { 1210, 1207, 1199,   1210, 1205, 1192,   1210, 1203, 1191 };

typedef struct {                         // presne 64 B = 1 stránka EEPROM
    uint16_t magic;
    uint16_t zero_v[9];                  // nula (kód x10) v polohe V
    uint16_t zero_mv[3];                 // nula v polohe mV (delič 1:1, x1 / x2 / x4)
    uint32_t cpv[9];                     // kódy na 1 V x1000 v polohe V (mV = 10x delič 1:1)
    uint16_t crc;
} __attribute__((packed)) ScopeCal_t;
static ScopeCal_t cal;
_Static_assert(sizeof(ScopeCal_t) == 64, "ScopeCal_t = 1 stránka EEPROM");
#define CAL_MAGIC   0x5343u              // "SC"
#define SETUP_MAGIC 0x5353u              // "SS"
// v0.72: adresy v eeprom_map.h (EE_SCOPE_SETUP_A/B, EE_SCOPE_CAL_A/B, EE_WAVE_ADDR = 0x0600)
#define WAVE_SLOT_SIZE    EE_WAVE_SLOT_SIZE
#define WAVE_SLOTS        EE_WAVE_SLOTS
#define WAVE_MAGIC        0x5358u        // "SX" (v0.61: okno okolo obrazovky, posun)

typedef struct { uint32_t mv; uint8_t combo; const char *label; } VRange_t;
static const VRange_t VR_V[] = {
    {    500, 2, "500mV" }, {   1000, 1, "1V"   }, {   2000, 0, "2V"   },
    {   5000, 5, "5V"    }, {  10000, 4, "10V"  }, {  20000, 3, "20V"  },
    {  50000, 8, "50V"   }, { 100000, 7, "100V" }, { 200000, 6, "200V" },
};
static const VRange_t VR_MV[] = {
    { 50, 2, "50mV" }, { 100, 1, "100mV" }, { 200, 0, "200mV" },
};
#define N_VR_V   (sizeof(VR_V)  / sizeof(VR_V[0]))
#define N_VR_MV  (sizeof(VR_MV) / sizeof(VR_MV[0]))

typedef struct { uint64_t ns; const char *label; } TBase_t;
static const TBase_t TB[] = {
    { 200, "200ns" }, { 500, "500ns" }, { 1000, "1us" }, { 2000, "2us" }, { 5000, "5us" },
    { 10000, "10us" }, { 20000, "20us" }, { 50000, "50us" }, { 100000, "100us" },
    { 200000, "200us" }, { 500000, "500us" }, { 1000000, "1ms" }, { 2000000, "2ms" },
    { 5000000, "5ms" }, { 10000000, "10ms" }, { 20000000, "20ms" }, { 50000000, "50ms" },
    { 100000000, "100ms" }, { 200000000, "200ms" },
    { 500000000, "500ms" }, { 1000000000, "1s" }, { 2000000000, "2s" },   // iba ROLL
    { 5000000000ull, "5s" }, { 10000000000ull, "10s" },
};
#define N_TB  (sizeof(TB) / sizeof(TB[0]))
#define TB_DEFAULT 11                    // 1 ms/div
#define TB_ROLL_MIN 17                   // 100 ms/div: od tejto základne sa dá zapnúť ROLL
#define TB_CLASSIC_MAX 18                // 200 ms/div: nad ňou iba ROLL

static const char *const MODE_TXT[3] = { "AUTO", "NORM", "SINGLE" };
static const uint16_t    MODE_COL[3] = { LCD_GREEN, LCD_WHITE, LCD_YELLOW };
static const uint16_t    TPOS_PX[3]  = { PW / 4, PW / 2, 3 * PW / 4 };
static const char *const TPOS_TXT[3] = { "1/4", "CENTER", "3/4" };

// Oneskorenie záznamu po spustení (Trig delay), 1-2-5
typedef struct { uint64_t ns; const char *label; } Delay_t;
static const Delay_t DLY[] = {
    { 0, "OFF" },
    { 1000ull, "1us" }, { 2000ull, "2us" }, { 5000ull, "5us" }, { 10000ull, "10us" }, { 20000ull, "20us" },
    { 50000ull, "50us" }, { 100000ull, "100us" }, { 200000ull, "200us" }, { 500000ull, "500us" },
    { 1000000ull, "1ms" }, { 2000000ull, "2ms" }, { 5000000ull, "5ms" }, { 10000000ull, "10ms" },
    { 20000000ull, "20ms" }, { 50000000ull, "50ms" }, { 100000000ull, "100ms" }, { 200000000ull, "200ms" },
    { 500000000ull, "500ms" }, { 1000000000ull, "1s" }, { 2000000000ull, "2s" }, { 5000000000ull, "5s" },
    { 10000000000ull, "10s" },
};
#define N_DLY   (sizeof(DLY) / sizeof(DLY[0]))
#define HPOS_MAX 600                     // vodorovný posun najviac +-2 obrazovky (ďalej obmedzí zásobník)
#define HPOS_STEP 5                      // 1/5 dielika

// --- Stav ---
typedef enum { M_MAIN = 0, M_VDIV, M_TIME, M_TRIG, M_SETUP, M_MEAS, M_WAVE, M_CAL } Menu_t;

static ScopeState_t st = { .vdiv_v = 1, .vdiv_mv = 1, .tb = TB_DEFAULT, .ac = 0, .offset = 0,
                           .slope = 0, .mode = 0, .tpos = 1, .trig_mv = 0,
                           .meas_mask = MEAS_DEFAULT, .meas_show = 1, .meas_all = 0,
                           .meas_panel = 1 };   // TRANSP   // 1 V, 100 mV, 1 ms, DC
static bool     mv;                      // poloha mV
static bool     run = true;
static Menu_t   menu = M_MAIN;
static bool     capturing = false;
static uint32_t settle_until;
static uint16_t cap_n;                   // zachytených vzoriek (bez zahodených)
static uint16_t cap_s;                   // vzoriek na šírku obrazovky
static uint32_t cap_q16;                 // vzoriek na pixel (Q16, <= 1.0)
static uint32_t cap_ms;                  // trvanie jedného zachytenia
static uint32_t cap_lo, cap_hi;          // okno hľadania hrany (index vzorky)
static int64_t  cap_off;                 // začiatok obrazovky voči hrane (Q16 vzoriek)
static bool     delay_clamped;           // oneskorenie obmedzené zásobníkom
static int16_t  grid_mark_x = -32768;    // poloha značky bodu spustenia pri poslednej mriežke
static int64_t  frame_anchor;            // poloha hrany posledného obrazu (Q16), STOP: posun v dátach
static uint8_t  buf[CAP_MAX + WARMUP];
static uint8_t  prev_top[PW], prev_bot[PW];
static bool     have_prev;
static int16_t  last_zero_y = -1, last_trig_y = -1;

// Spúšťanie
static bool     trig_seen;               // od RUN / zmeny prišla hrana
static uint32_t last_trig_tick;
static uint8_t  misses;
static uint32_t auto_ref;                // AUTO: posledná hrana alebo zmena

// Merania
typedef struct { bool ok[MEAS_COUNT]; float v[MEAS_COUNT]; } Meas_t;
static Meas_t   meas;
static bool     meas_have;
static uint32_t meas_tick;
static uint8_t  layout = 0xFF;           // 0 nič, 1 riadok vybraných, 2 panel všetkých
static char     meas_last[MEAS_COUNT][16];
static int16_t  clip0 = Y_MIN, clip1 = Y_MAX;
static bool     panel_force;
static int16_t  pend_t, pend_b;          // v0.70: časť stĺpca stopy v priehľadnom paneli (kreslí PanelColumn)
static int16_t  reg_y0 = 999, reg_y1 = -1;   // plocha meraní v mriežke (riadok alebo panel)  // priehľadný panel: stopa prešla cez text / prekresliť všetko
enum { PANEL_SOLID = 0, PANEL_TRANSP, PANEL_PLAIN };    // st.meas_panel   // zvislý rozsah kreslenia stopy (mimo plôch meraní)
static uint8_t  meas_sel;                // kurzor v menu meraní
static char     meas_menu_last[MEAS_ITEMS][16];
static uint32_t msg_until;               // hlásenie v hornom riadku
static const char *msg_text = "";
static uint16_t msg_col = LCD_ORANGE;

// Posledný nakreslený obraz (STOP, uloženie do pamäte)
static int64_t  frame_start;
static bool     frame_valid;

// ROLL
#define ROLL_N  (PW - 1)
static bool     rolling;
static uint8_t  roll_buf[ROLL_N];
static uint16_t roll_cnt, roll_head;
static uint32_t roll_acc, roll_acc_n, roll_draw_tick;
static uint64_t roll_next_us;
static bool     roll_new;
// v0.69: trigger v ROLL (NORM / SINGLE). Obraz sa kreslí z roll_view (snímka), aby pri čakaní na
// hranu ostal posledný obraz aj po prekreslení mriežky.
enum { RS_FREE = 0, RS_ARMED, RS_POST };
static uint8_t  roll_view[ROLL_N];      // zobrazené vzorky, od najstaršej po najnovšiu
static uint16_t roll_view_cnt;
static uint8_t  roll_state;              // RS_FREE = AUTO (roluje stále)
static bool     roll_edge_armed;          // hysteréza: signál bol na druhej strane úrovne
static uint16_t roll_post;               // počet pixelov po hrane, kým sa obraz zastaví
static bool     roll_redraw;             // nakresliť roll_view (aj pri čakaní)

// ---------------------------------------------------------------------------
// v0.76: Interp / Peak det / View
// ---------------------------------------------------------------------------
static ScopeExt_t sx = { .interp = INTERP_SINC, .peak = 0, .view = VIEW_WAVE, .rsv = 0 };
static const char *const INTERP_TXT[3] = { "LINE", "SIN(X)/X", "DOTS" };

// v0.76: SIN(X)/X interpolácia – okno Lanczos (a = 4), 32 fáz x 8 odbočiek (vzorky i-3..i+4), Q14, súčet = 16384
static const int16_t SINC_TAB[32][8] = {
    {     0,      0,      0,  16384,      0,      0,      0,      0 },
    {   -49,    158,   -443,  16356,    478,   -168,     53,     -1 },
    {   -93,    304,   -850,  16271,    990,   -345,    111,     -4 },
    {  -132,    438,  -1220,  16130,   1533,   -529,    173,     -9 },
    {  -165,    560,  -1551,  15933,   2105,   -719,    238,    -17 },
    {  -193,    668,  -1845,  15685,   2703,   -913,    305,    -26 },
    {  -216,    762,  -2100,  15385,   3326,  -1110,    374,    -37 },
    {  -234,    842,  -2316,  15034,   3970,  -1307,    445,    -50 },
    {  -247,    908,  -2495,  14638,   4631,  -1502,    516,    -65 },
    {  -255,    961,  -2638,  14196,   5308,  -1693,    586,    -81 },
    {  -258,   1000,  -2744,  13711,   5995,  -1877,    655,    -98 },
    {  -258,   1025,  -2816,  13193,   6689,  -2053,    721,   -117 },
    {  -253,   1039,  -2856,  12635,   7388,  -2218,    784,   -135 },
    {  -246,   1040,  -2864,  12050,   8086,  -2370,    842,   -154 },
    {  -235,   1030,  -2842,  11435,   8780,  -2506,    894,   -172 },
    {  -222,   1009,  -2794,  10797,   9466,  -2623,    941,   -190 },
    {  -207,    979,  -2720,  10140,  10140,  -2720,    979,   -207 },
    {  -190,    941,  -2623,   9465,  10798,  -2794,   1009,   -222 },
    {  -172,    894,  -2506,   8779,  11436,  -2842,   1030,   -235 },
    {  -154,    842,  -2370,   8086,  12050,  -2864,   1040,   -246 },
    {  -135,    784,  -2218,   7386,  12637,  -2856,   1039,   -253 },
    {  -117,    721,  -2053,   6689,  13193,  -2816,   1025,   -258 },
    {   -98,    655,  -1877,   5993,  13713,  -2744,   1000,   -258 },
    {   -81,    586,  -1693,   5308,  14196,  -2638,    961,   -255 },
    {   -65,    516,  -1502,   4632,  14637,  -2495,    908,   -247 },
    {   -50,    445,  -1307,   3970,  15034,  -2316,    842,   -234 },
    {   -37,    374,  -1110,   3327,  15384,  -2100,    762,   -216 },
    {   -26,    305,   -913,   2703,  15685,  -1845,    668,   -193 },
    {   -17,    238,   -719,   2104,  15934,  -1551,    560,   -165 },
    {    -9,    173,   -529,   1533,  16130,  -1220,    438,   -132 },
    {    -4,    111,   -345,    990,  16271,   -850,    304,    -93 },
    {    -1,     53,   -168,    478,  16356,   -443,    158,    -49 },
};
// v0.76: sin(2*pi*j/512) pre j = 0..128 (štvrtina periódy) – FFT a Hannovo okno
static const float SIN_Q[129] = {
    0.0000000f, 0.0122715f, 0.0245412f, 0.0368072f, 0.0490677f, 0.0613207f, 0.0735646f, 0.0857973f,
    0.0980171f, 0.1102222f, 0.1224107f, 0.1345807f, 0.1467305f, 0.1588581f, 0.1709619f, 0.1830399f,
    0.1950903f, 0.2071114f, 0.2191012f, 0.2310581f, 0.2429802f, 0.2548657f, 0.2667128f, 0.2785197f,
    0.2902847f, 0.3020059f, 0.3136817f, 0.3253103f, 0.3368899f, 0.3484187f, 0.3598950f, 0.3713172f,
    0.3826834f, 0.3939920f, 0.4052413f, 0.4164296f, 0.4275551f, 0.4386162f, 0.4496113f, 0.4605387f,
    0.4713967f, 0.4821838f, 0.4928982f, 0.5035384f, 0.5141027f, 0.5245897f, 0.5349976f, 0.5453250f,
    0.5555702f, 0.5657318f, 0.5758082f, 0.5857979f, 0.5956993f, 0.6055110f, 0.6152316f, 0.6248595f,
    0.6343933f, 0.6438315f, 0.6531728f, 0.6624158f, 0.6715590f, 0.6806010f, 0.6895405f, 0.6983762f,
    0.7071068f, 0.7157308f, 0.7242471f, 0.7326543f, 0.7409511f, 0.7491364f, 0.7572088f, 0.7651673f,
    0.7730105f, 0.7807372f, 0.7883464f, 0.7958369f, 0.8032075f, 0.8104572f, 0.8175848f, 0.8245893f,
    0.8314696f, 0.8382247f, 0.8448536f, 0.8513552f, 0.8577286f, 0.8639729f, 0.8700870f, 0.8760701f,
    0.8819213f, 0.8876396f, 0.8932243f, 0.8986745f, 0.9039893f, 0.9091680f, 0.9142098f, 0.9191139f,
    0.9238795f, 0.9285061f, 0.9329928f, 0.9373390f, 0.9415441f, 0.9456073f, 0.9495282f, 0.9533060f,
    0.9569403f, 0.9604305f, 0.9637761f, 0.9669765f, 0.9700313f, 0.9729400f, 0.9757021f, 0.9783174f,
    0.9807853f, 0.9831055f, 0.9852776f, 0.9873014f, 0.9891765f, 0.9909026f, 0.9924795f, 0.9939070f,
    0.9951847f, 0.9963126f, 0.9972905f, 0.9981181f, 0.9987955f, 0.9993224f, 0.9996988f, 0.9999247f,
    1.0000000f,
};

// Peak det: buf = stred [0..PK_MAX) | min [PK_MAX..) | max [2*PK_MAX..)
#define PK_MAX   2720u
#define PK_RING  512u
_Static_assert(3u * PK_MAX <= CAP_MAX, "Peak det: 3 polia v buf");
#define PK_MN    (&buf[WARMUP + PK_MAX])
#define PK_MX    (&buf[WARMUP + 2u * PK_MAX])
static uint8_t           pk_ring[PK_RING];  // DMA circular, 1 MHz
static volatile bool     pk_on;             // prerušenie: zachytenie po pixeloch
static volatile bool     pk_done;
static volatile bool     pk_roll;           // prerušenie: ROLL, iba priebežné min / max
static volatile uint8_t  pk_rmn = 255, pk_rmx = 0;
static uint16_t          pk_n, pk_slots, pk_slot, pk_cnt;
static uint8_t           pk_cmn = 255, pk_cmx = 0, pk_skip;
static bool              cap_peak;          // bežiace / posledné zachytenie je Peak det
static bool              pk_valid;          // buf obsahuje pásmo min / max
static bool              cap_plain;         // kalibrácia, autoset: vždy bežné zachytenie
static bool              roll_pk;           // ROLL s Peak det
static uint8_t           roll_bmn[ROLL_N], roll_bmx[ROLL_N], roll_vmn[ROLL_N], roll_vmx[ROLL_N];

// FFT
#define FFT_N    512u
#define FFT_M    256u
static float    fft_re[FFT_M + 1u], fft_im[FFT_M];
static int16_t  fft_ref = 10;               // horná čiara mriežky v dBV
static float    fft_span = 0.0f;            // fs / 2 v Hz
static float    fft_pk = -1.0f;             // najsilnejšia frekvencia, < 0 = nie je

// SAVE: krátke = pamäte priebehov, dlhé = posledné nastavenie
static bool     save_pending;
static uint32_t save_tick;

static void Msg(const char *t, uint16_t col, uint32_t ms);
static void RollClock(bool on);
static void RollReset(void);
static void RollEnd(void);
static void RollArm(void);
static void SetRun(bool on);
static void Restart(void);
static void OpenCal(void);
static bool RollWanted(void);
#define WAVE_SLOTS_N 10u
static uint8_t  wave_sel;
static bool     wave_used[WAVE_SLOTS_N];
static uint8_t  wave_open;               // v0.73: zobrazený uložený priebeh (číslo pamäte 1..10), 0 = živý obraz
static ScopeState_t live_st;             // v0.74: živé nastavenie odložené pri OPEN (platí, kým wave_open != 0)
static void LiveState(ScopeState_t *s);
static void GridTags(void);
static void WaveClose(void);
static uint8_t  cal_sel;
static void PanelOpen(uint8_t m);
static void PanelClose(void);
static char     title_last[48];

static uint8_t  hw_combo = 0xFF, hw_ac = 0xFF, hw_mv = 0xFF;

// Tlačidlá
static bool         auto_pending, adj_pending, lvl_pending;
static ButtonType_t key_pending = BTN_NONE, adj_btn, lvl_btn;   // key_pending: F1..F3 v hlavnom menu
static uint32_t     key_tick, auto_tick, adj_tick, adj_rep, lvl_rep;
static char         last_cells[4][8];
static uint16_t     last_fg[4], last_bg[4];
static char         top_text[24];

// Menu osciloskopu
static uint8_t      set_sel;
static char         last_set[SET_ITEMS][42];     // v0.68: rovnako ako key (bez varovania)
static int8_t       last_hint = -1;

// ---------------------------------------------------------------------------
static const VRange_t *CurRange(void) {
    return mv ? &VR_MV[st.vdiv_mv % N_VR_MV] : &VR_V[st.vdiv_v % N_VR_V];
}
// v0.73: kódy na 1 V a nula pre ľubovoľný rozsah (aj uložený priebeh v náhľade)
static uint32_t RangeCpv(const VRange_t *r, bool m) {   // kódy ADC na 1 V x1000
    uint8_t c = r->combo;
    return m ? cal.cpv[c % 3] * 10u : cal.cpv[c];
}
static int32_t RangeZero10(const VRange_t *r, bool m) { uint8_t c = r->combo; return m ? cal.zero_mv[c % 3] : cal.zero_v[c]; }
static uint32_t CurCpv(void) { return RangeCpv(CurRange(), mv); }
static int32_t Zero10(void) { return RangeZero10(CurRange(), mv); }

// Popis V/div so sondou, napr. "500mV", "5V"
static const char *VdivLabel(void) {
    static char b[10];
    uint32_t v = CurRange()->mv;
    if (v >= 1000u) snprintf(b, sizeof(b), "%luV",  (unsigned long)(v / 1000u));
    else            snprintf(b, sizeof(b), "%lumV", (unsigned long)v);
    return b;
}

// mV na vstupe -> kód ADC x256
static int32_t MvToCode256(int32_t mvv) {
    return Zero10() * 256 / 10 + (int32_t)((int64_t)mvv * (int64_t)CurCpv() * 256 / 1000000);
}
// kód ADC x10 -> mV na vstupe
static int32_t Code10ToMv(int32_t c10) {
    return (int32_t)((int64_t)(c10 - Zero10()) * 100000 / (int64_t)CurCpv());
}

static void PinSet(GPIO_TypeDef *p, uint32_t pin, bool on) { p->BSRR = on ? (1u << pin) : (1u << (pin + 16)); }

// Relé a prepínače podľa rozsahu; pri zmene čakanie na ustálenie pred ďalším zachytením
static void ApplyHw(void) {
    uint8_t c = CurRange()->combo;
    if (c == hw_combo && st.ac == hw_ac && (uint8_t)mv == hw_mv) return;
    bool relay = (hw_combo == 0xFF) || ((c / 3) != (hw_combo / 3)) || st.ac != hw_ac;
    uint8_t att = c / 3, g = c % 3;
    PinSet(GPIOA, PIN_REL_10,   att >= 1);
    PinSet(GPIOA, PIN_O1O2,     att == 2);
    PinSet(GPIOC, PIN_U6,       g == 2);
    PinSet(GPIOC, PIN_U7,       g == 1);
    PinSet(GPIOA, PIN_REL_ACDC, st.ac != 0);
    hw_combo = c; hw_ac = st.ac; hw_mv = (uint8_t)mv;
    settle_until = HAL_GetTick() + (relay ? SETTLE_RELAY_MS : SETTLE_MS);
}

// Úroveň triggera: na obrazovke (+-4 dieliky okolo stredu podľa posunu stopy)
static void ClampLevel(void) {
    int32_t vd = (int32_t)CurRange()->mv;
    int32_t lo = -(4 * vd) - (int32_t)st.offset * vd / 5;
    int32_t hi =  (4 * vd) - (int32_t)st.offset * vd / 5;
    if (st.trig_mv < lo) st.trig_mv = lo;
    if (st.trig_mv > hi) st.trig_mv = hi;
}

// ---------------------------------------------------------------------------
// Časovače a DMA
// ---------------------------------------------------------------------------
// Začiatok obrazovky voči hrane v Q16 vzoriek: bod spustenia, vodorovný posun, oneskorenie
static uint64_t TdivNs(void) { return TB[(st.tb > TB_CLASSIC_MAX ? TB_CLASSIC_MAX : st.tb) % N_TB].ns; }
static int32_t DelayPx(void) {
    if (!st.delay_idx) return 0;
    uint64_t px = DLY[st.delay_idx % N_DLY].ns * 25u / TdivNs();
    return px > 2000000u ? 2000000 : (int32_t)px;
}
static int64_t OffQ16(uint32_t q16) {
    int64_t px = (int64_t)st.hpos - (int64_t)TPOS_PX[st.tpos % 3] + DelayPx();
    return px * (int64_t)q16;
}
// Poloha značky bodu spustenia na obrazovke (px); mimo 0..PW-1 = šípka na okraji
static int16_t MarkX(void) {
    int32_t x = (int32_t)TPOS_PX[st.tpos % 3] - st.hpos - (rolling ? 0 : DelayPx());   // v0.69: ROLL bez Trig delay
    if (x < -1000) x = -1000;
    if (x > 1000) x = 1000;
    return (int16_t)x;
}

static void CaptureStop(void) {
    TIM1->CR1  = 0;
    TIM1->DIER = 0;
    TIM11->CR1 = 0;
    pk_on = false;
    pk_roll = false;
    DMA2_Stream5->CR &= ~(DMA_SxCR_EN | DMA_SxCR_HTIE | DMA_SxCR_TCIE);
    while (DMA2_Stream5->CR & DMA_SxCR_EN) { }
    DMA2->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;
    NVIC_ClearPendingIRQ(DMA2_Stream5_IRQn);
    capturing = false;
}

// v0.76 Peak det: časovače + DMA circular do pk_ring (každá vzorka ADC 1 MHz), prerušenie HT / TC
static void PeakHwStart(void) {
    TIM11->PSC   = 0;
    TIM11->ARR   = ADC_SLOW - 1u;
    TIM11->CCR1  = ADC_SLOW / 2u;
    TIM11->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM11->CCER  = TIM_CCER_CC1E;
    TIM11->EGR   = TIM_EGR_UG;
    TIM11->CNT   = ADC_SLOW / 2u + 1u;                      // rovnaká fáza ako CaptureStart
    TIM1->PSC  = 0;
    TIM1->ARR  = ADC_SLOW - 1u;
    TIM1->RCR  = 0;
    TIM1->EGR  = TIM_EGR_UG;
    TIM1->SR   = 0;
    TIM1->CNT  = 0;
    TIM1->DIER = TIM_DIER_UDE;
    DMA2->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;
    DMA2_Stream5->PAR  = (uint32_t)&GPIOC->IDR;
    DMA2_Stream5->M0AR = (uint32_t)pk_ring;
    DMA2_Stream5->NDTR = PK_RING;
    DMA2_Stream5->FCR  = 0;
    DMA2_Stream5->CR   = (6u << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL_1 | DMA_SxCR_MINC | DMA_SxCR_CIRC |
                         DMA_SxCR_HTIE | DMA_SxCR_TCIE;
    NVIC_SetPriority(DMA2_Stream5_IRQn, 2);                 // pod EXTI (0) a USART6 (1)
    NVIC_ClearPendingIRQ(DMA2_Stream5_IRQn);
    NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    DMA2_Stream5->CR  |= DMA_SxCR_EN;
    __disable_irq();
    TIM11->CR1 = TIM_CR1_CEN;
    TIM1->CR1  = TIM_CR1_CEN;
    __enable_irq();
}

static void PeakHwStopIsr(void) {
    TIM1->CR1 = 0;
    TIM1->DIER = 0;
    TIM11->CR1 = 0;
    DMA2_Stream5->CR &= ~(DMA_SxCR_EN | DMA_SxCR_HTIE | DMA_SxCR_TCIE);
}

// Polovica kruhového zásobníka: min / max za pixel (pk_n vzoriek) -> buf (stred, min, max)
static void PeakChunk(const uint8_t *p, uint32_t n) {
    uint32_t i = 0;
    while (pk_skip && i < n) { pk_skip--; i++; }
    uint8_t mn = pk_cmn, mx = pk_cmx;
    uint32_t cnt = pk_cnt, N = pk_n;
    uint8_t *dm = &buf[WARMUP], *dn = PK_MN, *dx = PK_MX;
    for (; i < n; i++) {
        uint8_t c = p[i];
        if (c < mn) mn = c;
        if (c > mx) mx = c;
        if (++cnt >= N) {
            uint32_t k = pk_slot;
            dn[k] = mn; dx[k] = mx; dm[k] = (uint8_t)(((uint32_t)mn + mx + 1u) >> 1);
            cnt = 0; mn = 255; mx = 0;
            if (++pk_slot >= pk_slots) { PeakHwStopIsr(); pk_on = false; pk_done = true; return; }
        }
    }
    pk_cmn = mn; pk_cmx = mx; pk_cnt = (uint16_t)cnt;
}

void DMA2_Stream5_IRQHandler(void);
void DMA2_Stream5_IRQHandler(void) {
    uint32_t isr = DMA2->HISR;
    DMA2->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;
    const uint8_t *p;
    if (isr & DMA_HISR_TCIF5)      p = &pk_ring[PK_RING / 2u];
    else if (isr & DMA_HISR_HTIF5) p = pk_ring;
    else return;
    if (pk_roll) {
        uint8_t mn = pk_rmn, mx = pk_rmx;
        for (uint32_t i = 0; i < PK_RING / 2u; i++) { uint8_t c = p[i]; if (c < mn) mn = c; if (c > mx) mx = c; }
        pk_rmn = mn; pk_rmx = mx;
        return;
    }
    if (pk_on) PeakChunk(p, PK_RING / 2u);
}

static void CaptureStart(void) {
    CaptureStop();
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM11EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    (void)RCC->APB2ENR;

    uint32_t tdiv = (uint32_t)TB[(st.tb > TB_CLASSIC_MAX ? TB_CLASSIC_MAX : st.tb) % N_TB].ns;
    uint32_t P    = tdiv / 250u;                            // takty na pixel (100 MHz, 25 px/div)
    if (P < P_MIN) P = P_MIN;
    bool fast     = (P <= ADC_SLOW);
    uint32_t clkp = fast ? P : ADC_SLOW;
    bool peak     = sx.peak && !fast && !cap_plain;             // v0.76
    uint32_t cmax = peak ? PK_MAX : CAP_MAX;

    cap_q16 = (uint32_t)(((uint64_t)tdiv << 16) / (250ull * P));
    if (cap_q16 > 65536u) cap_q16 = 65536u;
    uint32_t sw = (uint32_t)(((uint64_t)(PW - 1) * cap_q16) >> 16) + 2u;   // vzoriek na obrazovku
    // Okno hľadania hrany W, pred ním priestor pre začiatok obrazovky (posun doľava), za ním oneskorenie + obrazovka
    int64_t off = OffQ16(cap_q16);
    int32_t off_i = (int32_t)(off >> 16);
    uint32_t W = sw < 100u ? 100u : sw;
    int32_t off_min = -(int32_t)(cmax - W - sw - 8u);
    int32_t off_max = (int32_t)(cmax - W - sw - 8u);
    bool clamped = false;
    if (off_i < off_min) { off_i = off_min; clamped = true; }
    if (off_i > off_max) { off_i = off_max; clamped = true; }
    if (clamped) off = (int64_t)off_i << 16;
    uint32_t lo = (off_i < 0) ? (uint32_t)(-off_i) + 1u : 1u;
    uint32_t after = (off_i + (int32_t)sw + 2 > 0) ? (uint32_t)(off_i + (int32_t)sw + 2) : 0u;
    uint32_t n = lo + W + after;
    if (n < 2u * sw + 4u) n = 2u * sw + 4u;
    if (n < 200u) n = 200u;
    if (sx.view == VIEW_FFT && !cap_plain) {                // v0.76: FFT = 512 vzoriek od začiatku obrazovky
        uint32_t need = lo + W + (off_i > 0 ? (uint32_t)off_i : 0u) + FFT_N;
        if (n < need) n = need;
    }
    if (n > cmax) n = cmax;
    if (clamped && !delay_clamped && st.delay_idx) Msg("DELAY MAX", LCD_ORANGE, 1500);
    delay_clamped = clamped;
    cap_off = off;
    cap_lo = lo;
    cap_hi = lo + W;
    cap_s  = (uint16_t)sw;
    cap_n  = (uint16_t)n;
    cap_ms = (uint32_t)(((uint64_t)(n + WARMUP) * P) / 100000u) + 1u;
    cap_peak = peak;
    pk_valid = peak;
    if (peak) {
        pk_n = (uint16_t)(P / ADC_SLOW);
        pk_slots = (uint16_t)n;
        pk_slot = 0; pk_cnt = 0; pk_cmn = 255; pk_cmx = 0; pk_skip = WARMUP;
        pk_done = false; pk_roll = false; pk_on = true;
        PeakHwStart();
        capturing = true;
        return;
    }

    // TIM11: hodiny ADC, PWM1 50 %, nábežná hrana pri CNT = 0
    TIM11->PSC   = 0;
    TIM11->ARR   = clkp - 1u;
    TIM11->CCR1  = clkp / 2u;
    TIM11->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM11->CCER  = TIM_CCER_CC1E;
    TIM11->EGR   = TIM_EGR_UG;
    TIM11->CNT   = clkp / 2u + 1u;                          // DMA číta tesne po zostupnej hrane

    // TIM1: DMA požiadavka pri každom pretečení
    TIM1->PSC  = fast ? 0u : (ADC_SLOW - 1u);
    TIM1->ARR  = fast ? (P - 1u) : (P / ADC_SLOW - 1u);
    TIM1->RCR  = 0;
    TIM1->EGR  = TIM_EGR_UG;                                // načítať PSC (DIER ešte bez UDE)
    TIM1->SR   = 0;
    TIM1->CNT  = 0;
    TIM1->DIER = TIM_DIER_UDE;

    // DMA2 Stream5 kanál 6: GPIOC->IDR (bajt) -> buf
    DMA2->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;
    DMA2_Stream5->PAR  = (uint32_t)&GPIOC->IDR;
    DMA2_Stream5->M0AR = (uint32_t)buf;
    DMA2_Stream5->NDTR = n + WARMUP;
    DMA2_Stream5->FCR  = 0;                                 // priamy režim
    DMA2_Stream5->CR   = (6u << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL_1 | DMA_SxCR_MINC;
    DMA2_Stream5->CR  |= DMA_SxCR_EN;

    __disable_irq();
    TIM11->CR1 = TIM_CR1_CEN;
    TIM1->CR1  = TIM_CR1_CEN;
    __enable_irq();
    capturing = true;
}

static bool CaptureDone(void) {
    if (cap_peak) {                                         // v0.76: časovače zastaví prerušenie
        if (!pk_done) return false;
        capturing = false;
        return true;
    }
    if (!(DMA2->HISR & (DMA_HISR_TCIF5 | DMA_HISR_TEIF5))) return false;
    TIM1->CR1 = 0;
    TIM1->DIER = 0;
    TIM11->CR1 = 0;
    capturing = false;
    return true;
}

// Zachytenie s čakaním (autoset); false = časový limit
static bool CaptureBlocking(uint32_t timeout_ms) {
    ApplyHw();
    while ((int32_t)(HAL_GetTick() - settle_until) < 0) { }
    cap_plain = true;                                       // v0.76: autoset bez Peak det a FFT
    CaptureStart();
    cap_plain = false;
    uint32_t t0 = HAL_GetTick();
    while (!CaptureDone()) {
        if (HAL_GetTick() - t0 > timeout_ms) { CaptureStop(); return false; }
    }
    return true;
}

static void MinMax(uint32_t cnt, uint8_t *mn, uint8_t *mx) {
    const uint8_t *s = &buf[WARMUP];
    uint8_t a = 255, b = 0;
    for (uint32_t i = 0; i < cnt; i++) { if (s[i] < a) a = s[i]; if (s[i] > b) b = s[i]; }
    *mn = a; *mx = b;
}

// ---------------------------------------------------------------------------
// Hľadanie hrany. true = nájdená; *t_q16 = poloha hrany vo vzorkách (Q16)
// ---------------------------------------------------------------------------
static bool FindTrigger(uint32_t *t_q16) {
    const uint8_t *s = &buf[WARMUP];
    int32_t lvl = MvToCode256(st.trig_mv);
    int32_t cpd256 = (int32_t)((uint64_t)CurCpv() * CurRange()->mv * 256u / 1000000u);   // kódy na dielik x256
    int32_t hys = cpd256 / 10;
    if (hys < 384) hys = 384;                                // najmenej 1.5 kódu
    uint32_t pre   = cap_lo;                                 // pred hranou: začiatok obrazovky
    uint32_t i_max = cap_hi < cap_n ? cap_hi : cap_n;
    bool rise = (st.slope == 0);
    bool armed = false;
    for (uint32_t i = 1; i < i_max; i++) {
        int32_t a = (int32_t)s[i - 1] * 256, b = (int32_t)s[i] * 256;
        if (rise) {
            if (a < lvl - hys) armed = true;
            if (armed && a < lvl && b >= lvl && i - 1u >= pre) {
                uint32_t f = (uint32_t)(((int64_t)(lvl - a) << 16) / (b - a));
                *t_q16 = ((i - 1u) << 16) + (f > 65535u ? 65535u : f);
                return true;
            }
        } else {
            if (a > lvl + hys) armed = true;
            if (armed && a > lvl && b <= lvl && i - 1u >= pre) {
                uint32_t f = (uint32_t)(((int64_t)(a - lvl) << 16) / (a - b));
                *t_q16 = ((i - 1u) << 16) + (f > 65535u ? 65535u : f);
                return true;
            }
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// Kreslenie
// ---------------------------------------------------------------------------
static void DrawFrame(void) {
    LCD_DrawFastHLine(0, 0, 320, LCD_WHITE);
    LCD_DrawFastHLine(0, 239, 320, LCD_WHITE);
    LCD_DrawFastVLine(0, 0, 240, LCD_WHITE);
    LCD_DrawFastVLine(319, 0, 240, LCD_WHITE);
    LCD_DrawFastHLine(0, 27, 320, LCD_WHITE);
}

// Mriežka v stĺpci x medzi y0 a y1 (vnútro, bez obrysu) vrátane značky bodu spustenia hore
static void GridColumn(uint16_t x, uint16_t y0, uint16_t y1) {
    uint16_t rx = (uint16_t)(x - PX0);
    bool vline  = (rx % DIV_X) == 0;
    bool vmid   = (rx == PW / 2);
    for (uint16_t y = y0; y <= y1; y++) {
        uint16_t ry = (uint16_t)(y - PY0);
        bool hline = (ry % DIV_Y) == 0;
        bool hmid  = (y == YC);
        if ((vmid && (y % 2u) == 0u) || (hmid && (x % 2u) == 0u))
            LCD_DrawPixel(x, y, LCD_LIGHTGREY);
        else if ((vline && (y % 4u) == 0u) || (hline && (x % 4u) == 0u))
            LCD_DrawPixel(x, y, LCD_GREY);
    }
}

// Značka bodu spustenia v pásiku nad mriežkou (y 28..31): trojuholník nadol, mimo obrazovky šípka na okraji
static void DrawTrigMark(void) {
    int16_t mx = grid_mark_x;
    LCD_FillRect(1, MARK_Y0, 318, 4, LCD_BLACK);
    if (mx >= 0 && mx < PW) {
        for (int16_t r = 0; r < 4; r++) {
            int16_t h = (int16_t)(3 - r);
            LCD_DrawFastHLine((uint16_t)(PX0 + mx - h), (uint16_t)(MARK_Y0 + r), (uint16_t)(2 * h + 1), LCD_ORANGE);
        }
    } else if (mx < 0) {                                       // hrana pred obrazovkou: šípka vľavo
        LCD_DrawPixel(PX0, MARK_Y0 + 2, LCD_ORANGE);
        LCD_DrawFastVLine(PX0 + 1, MARK_Y0 + 1, 3, LCD_ORANGE);
        LCD_DrawFastVLine(PX0 + 2, MARK_Y0 + 1, 3, LCD_ORANGE);
        LCD_DrawFastVLine(PX0 + 3, MARK_Y0 + 1, 3, LCD_ORANGE);
    } else {                                                   // za obrazovkou: šípka vpravo
        LCD_DrawPixel(PX0 + PW, MARK_Y0 + 2, LCD_ORANGE);
        LCD_DrawFastVLine(PX0 + PW - 1, MARK_Y0 + 1, 3, LCD_ORANGE);
        LCD_DrawFastVLine(PX0 + PW - 2, MARK_Y0 + 1, 3, LCD_ORANGE);
        LCD_DrawFastVLine(PX0 + PW - 3, MARK_Y0 + 1, 3, LCD_ORANGE);
    }
}

static void DrawGrid(void) {
    grid_mark_x = MarkX();
    LCD_FillRect(1, 28, 318, 184, LCD_BLACK);
    DrawTrigMark();
    uint16_t fc = wave_open ? LCD_ORANGE : LCD_LIGHTGREY;    // v0.73: uložený priebeh = oranžový rámik
    LCD_DrawFastHLine(PX0, PY0, PW + 1, fc);
    LCD_DrawFastHLine(PX0, PY0 + PH, PW + 1, fc);
    LCD_DrawFastVLine(PX0, PY0, PH + 1, fc);
    LCD_DrawFastVLine(PX0 + PW, PY0, PH + 1, fc);
    for (uint16_t x = PX0 + 1; x < PX0 + PW; x++) GridColumn(x, Y_MIN, Y_MAX);
    have_prev = false;
    last_zero_y = -1;
    last_trig_y = -1;
    layout = 0xFF;                       // plochy meraní znova (ApplyLayout)
    GridTags();                          // v0.74: aj keď ešte nie je stopa (NORM / SINGLE čaká)
}

static int16_t OffsetPx(void) { return (int16_t)(st.offset * DIV_Y / 5); }

static int16_t ClampMarkY(int32_t y) {
    if (y < Y_MIN + 3) y = Y_MIN + 3;
    if (y > Y_MAX - 3) y = Y_MAX - 3;
    return (int16_t)y;
}

// Značka nuly vľavo (žltá, smerom k mriežke) a úrovne triggera vpravo (oranžová)
static void DrawMarkers(void) {
    int16_t y = ClampMarkY(YC - OffsetPx());
    if (y != last_zero_y) {
        if (last_zero_y >= 0) LCD_FillRect(2, (uint16_t)(last_zero_y - 3), 8, 7, LCD_BLACK);
        for (int16_t i = 0; i < 8; i++) {
            int16_t h = (int16_t)(3 - i / 2);
            LCD_DrawFastVLine((uint16_t)(2 + i), (uint16_t)(y - h), (uint16_t)(2 * h + 1), LCD_YELLOW);
        }
        last_zero_y = y;
    }
    int32_t vd = (int32_t)CurRange()->mv;
    int16_t ty = ClampMarkY(YC - OffsetPx() - (int32_t)((int64_t)st.trig_mv * DIV_Y / vd));
    if (ty != last_trig_y) {
        if (last_trig_y >= 0) LCD_FillRect(TMARK_X, (uint16_t)(last_trig_y - 3), 7, 7, LCD_BLACK);
        for (int16_t i = 0; i < 7; i++) {
            int16_t h = (int16_t)(i / 2);
            if (h > 3) h = 3;
            LCD_DrawFastVLine((uint16_t)(TMARK_X + i), (uint16_t)(ty - h), (uint16_t)(2 * h + 1), LCD_ORANGE);
        }
        last_trig_y = ty;
    }
}

// Kód ADC (x256) -> y na displeji pre rozsah r, polohu m (mV) a posun stopy off (v0.73: aj pre náhľady)
static int16_t CodeToYFor(int32_t c256, const VRange_t *r, bool m, int8_t off) {
    int32_t zero256 = RangeZero10(r, m) * 256 / 10;
    uint64_t cpd1000 = (uint64_t)RangeCpv(r, m) * r->mv / 1000u;      // kódy na dielik x1000
    if (cpd1000 == 0) cpd1000 = 1;
    int64_t dy = (int64_t)(c256 - zero256) * DIV_Y * 1000 / (int64_t)(cpd1000 * 256u);
    int32_t y = YC - (int32_t)dy - (int32_t)(off * DIV_Y / 5);
    if (y < Y_MIN) y = Y_MIN;
    if (y > Y_MAX) y = Y_MAX;
    return (int16_t)y;
}
static int16_t CodeToY(int32_t c256) { return CodeToYFor(c256, CurRange(), mv, st.offset); }

static void PanelColumn(uint16_t px, int16_t y0, int16_t y1, uint16_t ttop, uint16_t tbot);

// v0.70: časť segmentu v priehľadnom paneli sa nekreslí hneď – zapamätá sa (pend_t..pend_b) a stĺpec
// panelu sa potom nakreslí naraz (mriežka + stopa + texty) cez PanelSegFlush. Texty tak medzi
// kreslením stopy a kreslením textov nezmiznú (predtým ich stopa prepísala a obnovili sa až neskôr).
static bool PanelTransp(void) {
    return st.meas_panel != PANEL_SOLID && reg_y0 <= reg_y1;
}

// Orezanie na rozsah kreslenia; časť v priehľadnom paneli -> pend_t..pend_b. false = nič mimo panelu
static bool SegClip(uint16_t top, uint16_t bot, int16_t *t, int16_t *b) {
    *t = (int16_t)top; *b = (int16_t)bot;
    if (*t < clip0) *t = clip0;
    if (*b > clip1) *b = clip1;
    if (*t > *b) return false;
    if (PanelTransp() && *b >= reg_y0) {
        int16_t pt = (*t > reg_y0) ? *t : reg_y0;
        int16_t pb = (*b < reg_y1) ? *b : reg_y1;
        if (pt <= pb) {
            if (pt < pend_t) pend_t = pt;
            if (pb > pend_b) pend_b = pb;
        }
        if (*b > reg_y0 - 1) *b = (int16_t)(reg_y0 - 1);
        if (*t > *b) return false;
    }
    return true;
}

// Zvislá čiara stopy / mazanie orezané na rozsah kreslenia (mimo plôch meraní)
static void SegDraw(uint16_t px, uint16_t top, uint16_t bot) {
    int16_t t, b;
    if (!SegClip(top, bot, &t, &b)) return;
    LCD_DrawFastVLine(px, (uint16_t)t, (uint16_t)(b - t + 1), LCD_YELLOW);
}
static void SegErase(uint16_t px, uint16_t top, uint16_t bot) {
    int16_t t, b;
    if (!SegClip(top, bot, &t, &b)) return;
    LCD_DrawFastVLine(px, (uint16_t)t, (uint16_t)(b - t + 1), LCD_BLACK);
    GridColumn(px, (uint16_t)t, (uint16_t)b);
}
static void PanelSegStart(void) { pend_t = 999; pend_b = -1; }
static void PanelSegFlush(uint16_t px, uint16_t top, uint16_t bot) {   // top/bot = nová stopa v stĺpci
    if (pend_t <= pend_b) PanelColumn(px, pend_t, pend_b, top, bot);
}

// v0.76: hodnota v polohe i + f/65536 (kód x256): LINE = lineárne, SIN(X)/X = 8 vzoriek (iba ak je menej
// vzoriek ako pixelov, inak lineárne)
static int32_t Smp(const uint8_t *s, int32_t i) {
    if (i < 0) i = 0;
    if (i >= (int32_t)cap_n) i = (int32_t)cap_n - 1;
    return s[i];
}
static int32_t InterpC256(const uint8_t *s, uint32_t i, uint32_t f) {
    if (sx.interp == INTERP_SINC && cap_q16 < 65536u) {
        uint32_t ph = (f + 1024u) >> 11;                     // 32 fáz
        if (ph >= 32u) { ph = 0; i++; }
        const int16_t *h = SINC_TAB[ph];
        int32_t acc = 0;
        for (int32_t k = 0; k < 8; k++) acc += Smp(s, (int32_t)i + k - 3) * h[k];
        return acc / 64;                                     // Q14 -> x256
    }
    return (int32_t)s[i] * 256 + (((int32_t)s[i + 1] - (int32_t)s[i]) * (int32_t)f) / 256;
}

// Jeden stĺpec stopy: top..bot (top 0 = nič), iba zmenené stĺpce
static void TraceColumn(uint32_t x, uint16_t top, uint16_t bot) {
    uint16_t px = (uint16_t)(PX0 + 1 + x);
    if (have_prev && prev_top[x] == top && prev_bot[x] == bot) return;
    PanelSegStart();
    if (have_prev && prev_top[x]) SegErase(px, prev_top[x], prev_bot[x]);
    if (top) SegDraw(px, top, bot);
    PanelSegFlush(px, top, bot);
    prev_top[x] = (uint8_t)top;
    prev_bot[x] = (uint8_t)bot;
}

static void DrawFft(const uint8_t *s, uint32_t cnt, int64_t start_q16);

// start_q16 = poloha prvého pixelu vo vzorkách (Q16)
static void DrawTrace(int64_t start_q16) {
    const uint8_t *s = &buf[WARMUP];
    if (sx.view == VIEW_FFT) { DrawFft(s, cap_n, start_q16); return; }     // v0.76
    bool dots = (sx.interp == INTERP_DOTS);
    int16_t yp = 0, bt = 0, bb = 0;
    uint32_t pps = 65536u / (cap_q16 ? cap_q16 : 1u);        // pixelov na vzorku (>= 1)
    for (uint32_t x = 0; x + 1u < PW; x++) {                  // posledný pixel je obrys
        int64_t pos = start_q16 + (int64_t)x * cap_q16;
        if (pos < 0) pos = 0;
        uint32_t i = (uint32_t)(pos >> 16), f = (uint32_t)(pos & 0xFFFF);
        if (i + 1u >= cap_n) { i = cap_n - 2u; f = 65535u; }
        uint16_t top = 0, bot = 0;
        if (pk_valid) {                                      // v0.76 Peak det: pásmo min..max
            uint32_t k = i + (f >= 32768u ? 1u : 0u);
            if (k >= cap_n) k = cap_n - 1u;
            int16_t yt = CodeToY((int32_t)PK_MX[k] * 256), yb = CodeToY((int32_t)PK_MN[k] * 256);
            int16_t ot = yt, ob = yb;
            if (!dots && x > 0) {                            // spojiť so susedným pásmom
                if (yt > bb) yt = bb;
                if (yb < bt) yb = bt;
            }
            bt = ot; bb = ob;
            top = (uint16_t)yt; bot = (uint16_t)yb;
        } else if (dots) {                                   // v0.76 DOTS: iba skutočné vzorky
            uint32_t j = (uint32_t)((pos + 32768) >> 16);
            if (j >= cap_n) j = cap_n - 1u;
            int64_t d = pos - ((int64_t)j << 16);
            if (d < 0) d = -d;
            uint64_t dpx = (uint64_t)d * 65536u / (cap_q16 ? cap_q16 : 1u);     // vzdialenosť v px (Q16)
            int16_t y = CodeToY((int32_t)s[j] * 256);
            if (pps >= 3u && dpx <= 98304u)  { top = (uint16_t)(y - 1); bot = (uint16_t)(y + 1); }   // bod 3 x 3 (+-1.5 px)
            else if (dpx < 32768u)           { top = (uint16_t)y; bot = (uint16_t)y; }
        } else {
            int16_t y = CodeToY(InterpC256(s, i, f));
            if (x == 0) yp = y;
            top = (uint16_t)((y < yp) ? y : yp); bot = (uint16_t)((y < yp) ? yp : y);
            yp = y;
        }
        TraceColumn(x, top, bot);
    }
    have_prev = true;
    GridTags();                                              // v0.73: "WAVE n" vpravo hore, v0.74: "DLY ..." vľavo hore
}

// ---------------------------------------------------------------------------
// v0.76: FFT (512 bodov reálne = 256 komplexne), Hannovo okno, y v dBV
// ---------------------------------------------------------------------------
static float SinI(uint32_t j) {                              // sin(2*pi*j/512)
    j &= 511u;
    if (j <= 128u) return SIN_Q[j];
    if (j <= 256u) return SIN_Q[256u - j];
    if (j <= 384u) return -SIN_Q[j - 256u];
    return -SIN_Q[512u - j];
}
static float CosI(uint32_t j) { return SinI(j + 128u); }

static float Db10(float p) {                                 // 10*log10(p), presnosť asi 0.02 dB
    if (p < 1e-20f) return -200.0f;
    union { float f; uint32_t u; } v = { p };
    int32_t e = (int32_t)((v.u >> 23) & 255u) - 128;   // polynóm vracia 1 + log2(m)
    v.u = (v.u & 0x007FFFFFu) | 0x3F800000u;
    float m = v.f;
    float l2 = (float)e + (-0.34484843f * m + 2.02466578f) * m - 0.67487759f;
    return l2 * 3.0103f;
}

static void Fft256(float *re, float *im) {
    for (uint32_t i = 1, j = 0; i < FFT_M; i++) {
        uint32_t b = FFT_M >> 1;
        for (; j & b; b >>= 1) j ^= b;
        j ^= b;
        if (i < j) { float t = re[i]; re[i] = re[j]; re[j] = t; t = im[i]; im[i] = im[j]; im[j] = t; }
    }
    for (uint32_t len = 2; len <= FFT_M; len <<= 1) {
        uint32_t step = FFT_N / len, h = len >> 1;
        for (uint32_t k = 0; k < h; k++) {
            float wr = CosI(k * step), wi = -SinI(k * step);
            for (uint32_t a = k; a < FFT_M; a += len) {
                uint32_t b = a + h;
                float tr = re[b] * wr - im[b] * wi, ti = re[b] * wi + im[b] * wr;
                re[b] = re[a] - tr; im[b] = im[a] - ti;
                re[a] += tr;        im[a] += ti;
            }
        }
    }
}

// X[k] z Z[k] = a, Z[M-k] = b (reálna FFT z komplexnej polovičnej dĺžky): výkon |X[k]|^2
static float SplitPow(float ar, float ai, float br, float bi, uint32_t k) {
    float er = 0.5f * (ar + br), ei = 0.5f * (ai - bi);
    float orr = 0.5f * (ai + bi), oi = -0.5f * (ar - br);
    float c = CosI(k), sn = SinI(k);
    float xr = er + c * orr + sn * oi, xi = ei + c * oi - sn * orr;
    return xr * xr + xi * xi;
}

// Výkonové spektrum v dBV do fft_re[0..FFT_M]; n vzoriek (<= FFT_N, zvyšok nuly)
static bool FftCompute(const uint8_t *s, uint32_t n) {
    if (n < 16u) return false;
    if (n > FFT_N) n = FFT_N;
    uint32_t sum = 0;
    for (uint32_t i = 0; i < n; i++) sum += s[i];
    float mean = (float)sum / (float)n, wsum = 0.0f;
    for (uint32_t i = 0; i < FFT_N; i++) {
        float v = 0.0f;
        if (i < n) {
            float w = 0.5f - 0.5f * CosI((i * 512u + n / 2u) / n);
            wsum += w;
            v = ((float)s[i] - mean) * w;
        }
        if (i & 1u) fft_im[i >> 1] = v; else fft_re[i >> 1] = v;
    }
    Fft256(fft_re, fft_im);
    // Vrms^2 = 2 |X|^2 / wsum^2 * (V na kód)^2, V na kód = 1000 / cpv
    float vpc = 1000.0f / (float)CurCpv();
    float sc = 2.0f * vpc * vpc / (wsum * wsum);
    for (uint32_t k = 0; k <= FFT_M / 2u; k++) {
        uint32_t m = FFT_M - k;                                // k = 0: Z[M] = Z[0]
        float ar = fft_re[k], ai = fft_im[k];
        float br = fft_re[m % FFT_M], bi = fft_im[m % FFT_M];
        float pk = SplitPow(ar, ai, br, bi, k);
        float pm = SplitPow(br, bi, ar, ai, m);
        fft_re[k] = Db10(pk * sc);
        fft_re[m] = Db10(pm * sc);
    }
    return true;
}

static void Fmt3(char *b, size_t n, float v, const char *unit) {   // 3 platné číslice, napr. "1.00kHz"
    static const char *const PFX[5] = { "m", "", "k", "M", "G" };
    int e = 1;
    if (v < 0.0f) v = 0.0f;
    while (v >= 999.5f && e < 4) { v /= 1000.0f; e++; }
    while (v < 1.0f && e > 0 && v > 0.0f) { v *= 1000.0f; e--; }
    uint32_t q;
    if (v < 9.995f)      { q = (uint32_t)(v * 100.0f + 0.5f); snprintf(b, n, "%lu.%02lu%s%s", (unsigned long)(q / 100u), (unsigned long)(q % 100u), PFX[e], unit); }
    else if (v < 99.95f) { q = (uint32_t)(v * 10.0f + 0.5f);  snprintf(b, n, "%lu.%01lu%s%s", (unsigned long)(q / 10u), (unsigned long)(q % 10u), PFX[e], unit); }
    else                 { q = (uint32_t)(v + 0.5f);          snprintf(b, n, "%lu%s%s", (unsigned long)q, PFX[e], unit); }
}

// Spektrum vzoriek s (cnt) od polohy start_q16; x = 0..fs/2, 10 dB/dielik od REF
static void DrawFft(const uint8_t *s, uint32_t cnt, int64_t start_q16) {
    int64_t i0l = start_q16 >> 16;
    if (i0l < 0) i0l = 0;
    uint32_t n = cnt < FFT_N ? cnt : FFT_N;
    uint32_t i0 = (uint32_t)i0l;
    if (i0 + n > cnt) i0 = cnt - n;
    // REF: sínus cez celú výšku (8 dielikov) v dBV, zaokrúhlené nahor na 10 dB
    float vfs = 4.0f * (float)CurRange()->mv / 1000.0f;
    int32_t r = (int32_t)(Db10(vfs * vfs * 0.5f) + 1000.0f);
    fft_ref = (int16_t)(((r + 9) / 10) * 10 - 1000);
    float sample_ns = (float)TB[st.tb % N_TB].ns * 65536.0f / (25.0f * (float)(cap_q16 ? cap_q16 : 65536u));
    fft_span = 0.5e9f / sample_ns;
    fft_pk = -1.0f;
    if (!FftCompute(s + i0, n)) {
        for (uint32_t x = 0; x + 1u < PW; x++) TraceColumn(x, 0, 0);
        have_prev = true;
        GridTags();
        return;
    }
    // najsilnejšia zložka (bez DC), parabolická interpolácia
    uint32_t kb = 2;
    for (uint32_t k = 3; k + 1u < FFT_M; k++) if (fft_re[k] > fft_re[kb]) kb = k;
    if (fft_re[kb] > (float)fft_ref - 75.0f) {
        float a = fft_re[kb - 1u], b = fft_re[kb], c = fft_re[kb + 1u];
        float den = a - 2.0f * b + c, d = 0.0f;
        if (den < -1e-6f) d = 0.5f * (a - c) / den;
        if (d > 0.5f) d = 0.5f;
        if (d < -0.5f) d = -0.5f;
        fft_pk = ((float)kb + d) * fft_span / (float)FFT_M;
    }
    int16_t yp = 0;
    for (uint32_t x = 0; x + 1u < PW; x++) {
        uint32_t fb = x * FFT_M * 256u / (PW - 2u);          // bin x256
        uint32_t k = fb >> 8, fr = fb & 255u;
        float db = fft_re[k];
        if (k < FFT_M) db += (fft_re[k + 1u] - db) * (float)fr / 256.0f;
        int32_t y = PY0 + (int32_t)(((float)fft_ref - db) * (float)DIV_Y / 10.0f);
        if (y < Y_MIN) y = Y_MIN;
        if (y > Y_MAX) y = Y_MAX;
        if (x == 0) yp = (int16_t)y;
        uint16_t top = (uint16_t)((y < yp) ? y : yp), bot = (uint16_t)((y < yp) ? yp : y);
        yp = (int16_t)y;
        TraceColumn(x, top, bot);
    }
    have_prev = true;
    GridTags();
}

// ---------------------------------------------------------------------------
// Merania z údajov na obrazovke
// ---------------------------------------------------------------------------
static uint32_t ISqrt64(uint64_t v) {
    uint64_t r = 0, b = 1ull << 62;
    while (b > v) b >>= 2;
    while (b) {
        if (v >= r + b) { v -= r + b; r = (r >> 1) + b; }
        else            { r >>= 1; }
        b >>= 2;
    }
    return (uint32_t)r;
}

static void Measure(int64_t start_q16) {
    const uint8_t *s = &buf[WARMUP];
    int64_t i0l = start_q16 >> 16;
    if (i0l < 0) i0l = 0;
    uint32_t i0 = (uint32_t)i0l;
    uint32_t cnt = cap_s;
    if (i0 + cnt > cap_n) cnt = cap_n - i0;
    memset(&meas, 0, sizeof(meas));
    meas_have = true;
    if (cnt < 4) return;
    s += i0;

    float k = 100000.0f / (float)CurCpv();                  // mV na (kód x10)
    float z = (float)Zero10();
    float sample_ns = (float)TB[st.tb % N_TB].ns * 65536.0f / (25.0f * (float)cap_q16);

    uint8_t mn = 255, mx = 0;
    uint32_t sum = 0;
    int64_t sq = 0;
    int32_t z10 = Zero10();
    for (uint32_t i = 0; i < cnt; i++) {
        if (s[i] < mn) mn = s[i];
        if (s[i] > mx) mx = s[i];
        sum += s[i];
        int32_t d = (int32_t)s[i] * 10 - z10;
        sq += (int64_t)d * d;
    }
    uint8_t pmn = mn, pmx = mx;                              // v0.76 Peak det: Max / Min / Vpp z pásma
    if (pk_valid) {
        const uint8_t *a = PK_MN + i0, *b = PK_MX + i0;
        pmn = 255; pmx = 0;
        for (uint32_t i = 0; i < cnt; i++) { if (a[i] < pmn) pmn = a[i]; if (b[i] > pmx) pmx = b[i]; }
    }
    meas.v[MEAS_VMAX] = ((float)pmx * 10.0f - z) * k;  meas.ok[MEAS_VMAX] = true;
    meas.v[MEAS_VMIN] = ((float)pmn * 10.0f - z) * k;  meas.ok[MEAS_VMIN] = true;
    meas.v[MEAS_VPP]  = (float)(pmx - pmn) * 10.0f * k; meas.ok[MEAS_VPP]  = true;
    meas.v[MEAS_VAVG] = ((float)sum * 10.0f / (float)cnt - z) * k;  meas.ok[MEAS_VAVG] = true;
    meas.v[MEAS_VRMS] = (float)ISqrt64((uint64_t)(sq / cnt)) * k;   meas.ok[MEAS_VRMS] = true;

    int32_t pp = mx - mn;
    if (pp < 4) return;                                      // bez signálu: časové merania nie

    // Perióda a strieda: nábežné prechody stredom s hysterézou (poloha medzi vzorkami)
    float mid = ((float)mn + (float)mx) * 0.5f;
    float hys = (float)pp * 0.1f;
    if (hys < 1.0f) hys = 1.0f;
    bool armed = false;
    float first = -1.0f, last = -1.0f;
    uint32_t edges = 0, first_i = 0, last_i = 0;
    for (uint32_t i = 1; i < cnt; i++) {
        float a = s[i - 1], b = s[i];
        if (a < mid - hys) armed = true;
        if (armed && a < mid && b >= mid) {
            float t = (float)(i - 1) + (mid - a) / (b - a);
            if (edges == 0) { first = t; first_i = i; }
            last = t; last_i = i;
            edges++;
            armed = false;
        }
    }
    if (edges >= 2) {
        float per = (last - first) / (float)(edges - 1) * sample_ns;   // ns
        meas.v[MEAS_PER]  = per * 1e-9f;          meas.ok[MEAS_PER]  = true;
        meas.v[MEAS_FREQ] = 1e9f / per;           meas.ok[MEAS_FREQ] = true;
        uint32_t hi = 0;
        for (uint32_t i = first_i; i < last_i; i++) if ((float)s[i] >= mid) hi++;
        float d = (float)hi * 100.0f / (float)(last_i - first_i);
        meas.v[MEAS_DUTYP] = d;                   meas.ok[MEAS_DUTYP] = true;
        meas.v[MEAS_DUTYN] = 100.0f - d;          meas.ok[MEAS_DUTYN] = true;
    }

    // Nábeh / dobeh 10 % -> 90 %
    if (pp >= 8) {
        float lo = (float)mn + (float)pp * 0.1f, hiv = (float)mn + (float)pp * 0.9f;
        float t_lo = -1.0f, t_hi = -1.0f, r_sum = 0.0f, f_sum = 0.0f;
        uint32_t r_n = 0, f_n = 0;
        for (uint32_t i = 1; i < cnt; i++) {
            float a = s[i - 1], b = s[i];
            if (a < lo && b >= lo) t_lo = (float)(i - 1) + (lo - a) / (b - a);
            if (a < hiv && b >= hiv && t_lo >= 0.0f) {
                r_sum += (float)(i - 1) + (hiv - a) / (b - a) - t_lo; r_n++; t_lo = -1.0f;
            }
            if (a > hiv && b <= hiv) t_hi = (float)(i - 1) + (a - hiv) / (a - b);
            if (a > lo && b <= lo && t_hi >= 0.0f) {
                f_sum += (float)(i - 1) + (a - lo) / (a - b) - t_hi; f_n++; t_hi = -1.0f;
            }
        }
        if (r_n) { meas.v[MEAS_RISE] = r_sum / (float)r_n * sample_ns * 1e-9f; meas.ok[MEAS_RISE] = true; }
        if (f_n) { meas.v[MEAS_FALL] = f_sum / (float)f_n * sample_ns * 1e-9f; meas.ok[MEAS_FALL] = true; }
    }
}

// Hodnota s predponou (n, u, m, -, k, M) a 4 platnými číslicami, napr. "1.000kHz", "-450.0mV"
static void FmtSI(char *b, size_t n, float v, const char *unit) {
    static const char *const PFX[6] = { "n", "u", "m", "", "k", "M" };
    const char *sg = (v < 0.0f) ? "-" : "";
    float a = (v < 0.0f) ? -v : v;
    int e = 3;                                               // index predpony (3 = bez)
    if (a < 1e-12f) { snprintf(b, n, "0.000%s", unit); return; }
    while (a < 1.0f && e > 0)     { a *= 1000.0f; e--; }
    while (a >= 1000.0f && e < 5) { a /= 1000.0f; e++; }
    uint32_t q;
    if (a < 10.0f) {
        q = (uint32_t)(a * 1000.0f + 0.5f);
        if (q < 10000u) { snprintf(b, n, "%s%lu.%03lu%s%s", sg, (unsigned long)(q / 1000u), (unsigned long)(q % 1000u), PFX[e], unit); return; }
        a = 10.0f;
    }
    if (a < 100.0f) {
        q = (uint32_t)(a * 100.0f + 0.5f);
        if (q < 10000u) { snprintf(b, n, "%s%lu.%02lu%s%s", sg, (unsigned long)(q / 100u), (unsigned long)(q % 100u), PFX[e], unit); return; }
        a = 100.0f;
    }
    q = (uint32_t)(a * 10.0f + 0.5f);
    snprintf(b, n, "%s%lu.%01lu%s%s", sg, (unsigned long)(q / 10u), (unsigned long)(q % 10u), PFX[e], unit);
}

static void MeasValue(uint8_t m, char *b, size_t n) {
    if (!meas_have || !meas.ok[m]) { snprintf(b, n, "---"); return; }
    float v = meas.v[m];
    switch (m) {
        case MEAS_FREQ:  FmtSI(b, n, v, "Hz"); break;
        case MEAS_PER: case MEAS_RISE: case MEAS_FALL: FmtSI(b, n, v, "s"); break;
        case MEAS_DUTYP: case MEAS_DUTYN: {
            uint32_t q = (uint32_t)(v * 10.0f + 0.5f);
            snprintf(b, n, "%lu.%lu%%", (unsigned long)(q / 10u), (unsigned long)(q % 10u));
            break;
        }
        default: FmtSI(b, n, v / 1000.0f, "V"); break;     // mV -> V
    }
}

// Plochy meraní: 0 nič, 1 riadok vybraných (dole), 2 panel všetkých (hore)
static void ApplyLayout(void) {
    uint8_t want = st.meas_all ? 2u : (st.meas_show && st.meas_mask) ? 1u : 0u;
    if (want == layout) return;
    if (layout != 0xFF) DrawGrid();                          // obnoviť mriežku pod starou plochou
    layout = want;
    clip0 = Y_MIN;
    clip1 = Y_MAX;
    reg_y0 = 999; reg_y1 = -1;
    if (want) {                                              // merania dole v mriežke
        reg_y0 = (want == 2) ? PANEL_Y0 : ROW_Y;
        reg_y1 = Y_MAX;
        if (st.meas_panel == PANEL_SOLID) {
            LCD_FillRect(PX0 + 1, (uint16_t)reg_y0, PW - 1, (uint16_t)(reg_y1 - reg_y0 + 1), LCD_BLACK);
            clip1 = (int16_t)(reg_y0 - 1);
        }
    }
    panel_force = true;
    memset(meas_last, 0, sizeof(meas_last));
    meas_tick = HAL_GetTick() - MEAS_REFRESH_MS;
    if (!run && frame_valid && cap_n >= 2) DrawTrace(frame_start);   // STOP: obraz znova
}

// v0.67: panel všetkých meraní (3 riadky, x od PX0 + 3):
//   riadok 1: F, T, Vpp, Rms   (4 stĺpce po 75 px, 12 znakov – hodnoty nie sú záporné)
//   riadok 2: D+, D-, Tr, Tf   (4 stĺpce po 75 px, 12 znakov)          – v0.69: prehodené s riadkom 3
//   riadok 3: Min, Max, Avg    (3 stĺpce po 100 px, 15 znakov – môžu byť záporné)
static const uint8_t PANEL_POS[MEAS_COUNT][2] = {        // { riadok, stĺpec }
    [MEAS_FREQ]  = {0, 0}, [MEAS_PER]   = {0, 1}, [MEAS_VPP]  = {0, 2}, [MEAS_VRMS] = {0, 3},
    [MEAS_DUTYP] = {1, 0}, [MEAS_DUTYN] = {1, 1}, [MEAS_RISE] = {1, 2}, [MEAS_FALL] = {1, 3},
    [MEAS_VMIN]  = {2, 0}, [MEAS_VMAX]  = {2, 1}, [MEAS_VAVG] = {2, 2},
};

static void PanelPos(uint8_t m, uint16_t *x, uint16_t *y, uint8_t *w_chars) {
    uint8_t r = PANEL_POS[m][0], c = PANEL_POS[m][1];
    bool four = (r != 2);                                // v0.69: 3 merania iba v poslednom riadku
    *x = (uint16_t)(PX0 + 3 + c * (four ? 75u : 100u));
    *y = (uint16_t)(PANEL_Y0 + 2 + r * 10u);
    if (w_chars) *w_chars = four ? 12u : 15u;
}

static void DrawMeasSlot(uint8_t m, uint16_t x, uint16_t y, uint8_t w_chars) {
    char v[16], t[24];
    MeasValue(m, v, sizeof(v));
    if (strcmp(v, meas_last[m]) == 0) return;
    snprintf(meas_last[m], sizeof(meas_last[m]), "%s", v);
    size_t ll = strlen(MEAS_LBL[m]);
    snprintf(t, sizeof(t), "%-*.*s", (int)(w_chars - ll - 1u), (int)(w_chars - ll - 1u), v);
    LCD_DrawString(x, y, MEAS_LBL[m], LCD_LIGHTGREY, LCD_BLACK, 1);
    LCD_DrawString((uint16_t)(x + (ll + 1u) * 6u), y, t, LCD_WHITE, LCD_BLACK, 1);
}

static void DrawMeas(uint32_t now) {
    if (layout == 0 || layout == 0xFF) return;
    if (st.meas_panel != PANEL_SOLID) return;                 // priehľadné merania: PanelRefresh
    // v0.67: kontrola režimu pred časovačom – inak DrawMeas "zjedol" meas_tick a v TRANSP sa
    // hodnoty nikdy neobnovili
    if (now - meas_tick < MEAS_REFRESH_MS) return;
    meas_tick = now;
    if (layout == 2) {                                       // v0.67: 3 riadky (4 / 3 / 4 merania)
        for (uint8_t m = 0; m < MEAS_COUNT; m++) {
            uint16_t x, y; uint8_t w;
            PanelPos(m, &x, &y, &w);
            DrawMeasSlot(m, x, y, w);
        }
    } else {                                                 // najviac 4 vybrané v riadku
        uint8_t k = 0;
        for (uint8_t m = 0; m < MEAS_COUNT && k < MEAS_MAX_SEL; m++) {
            if (!(st.meas_mask & (1u << m))) continue;
            DrawMeasSlot(m, (uint16_t)(PX0 + 3 + k * 75u), (uint16_t)(ROW_Y + 3), 12);
            k++;
        }
    }
}

// --- Priehľadný panel všetkých meraní (TRANSP s čiernym obrysom, PLAIN bez obrysu) ---
// v0.70: bez blikania. Pixely panelu sa skladajú po stĺpcoch priamo vo výslednej farbe
// (text > čierny obrys > stopa > mriežka > čierna) a zapíšu sa naraz – nič sa najprv nemaže
// na čierno. Pri zmene hodnoty sa prekreslí iba obdĺžnik tejto hodnoty, pri prechode stopy
// iba zmenená časť stĺpca (PanelSegFlush).
static bool MeasPos(uint8_t m, uint8_t *k, uint16_t *x, uint16_t *y, uint8_t *w) {   // poloha merania v riadku / paneli
    if (layout == 2) {
        PanelPos(m, x, y, w);
        return true;
    }
    if (!(st.meas_mask & (1u << m)) || *k >= MEAS_MAX_SEL) return false;
    *x = (uint16_t)(PX0 + 3 + *k * 75u);
    *y = (uint16_t)(ROW_Y + 3);
    *w = 12;
    (*k)++;
    return true;
}

// Stĺpec reťazca v stĺpci displeja px (bity = riadky od reg_y0), 0 = mimo textu
static uint32_t StrColBits(const char *t, size_t len, uint16_t x, uint16_t y, uint16_t px) {
    if (px < x || px >= x + len * 6u) return 0;
    uint16_t dx = (uint16_t)(px - x);
    uint8_t col = (uint8_t)(dx % 6u);
    if (col >= 5u) return 0;
    uint32_t line = LCD_FontColumn(t[dx / 6u], col);
    int16_t sh = (int16_t)(y - reg_y0);
    return (sh >= 0) ? (line << sh) : (line >> -sh);
}

// Masky textov v stĺpci px: popisy (sivé) a hodnoty (biele)
static void PanelMasks(uint16_t px, uint32_t *lbl, uint32_t *val) {
    *lbl = 0; *val = 0;
    if (px <= PX0 || px >= PX0 + PW) return;
    uint8_t k = 0;
    for (uint8_t m = 0; m < MEAS_COUNT; m++) {
        uint16_t x, y; uint8_t w;
        if (!MeasPos(m, &k, &x, &y, &w)) continue;
        size_t ll = strlen(MEAS_LBL[m]);
        if (px + 1u < x || px > x + w * 6u) continue;            // mimo obdĺžnika merania
        *lbl |= StrColBits(MEAS_LBL[m], ll, x, y, px);
        size_t vl = strlen(meas_last[m]), vmax = (size_t)(w - ll - 1u);
        if (vl > vmax) vl = vmax;                                 // ako SOLID: hodnota v šírke stĺpca
        *val |= StrColBits(meas_last[m], vl, (uint16_t)(x + (ll + 1u) * 6u), y, px);
    }
}

static uint16_t GridColor(uint16_t x, uint16_t y) {               // rovnaká logika ako GridColumn
    uint16_t rx = (uint16_t)(x - PX0), ry = (uint16_t)(y - PY0);
    bool vline = (rx % DIV_X) == 0, vmid = (rx == PW / 2);
    bool hline = (ry % DIV_Y) == 0, hmid = (y == YC);
    if ((vmid && (y % 2u) == 0u) || (hmid && (x % 2u) == 0u)) return LCD_LIGHTGREY;
    if ((vline && (y % 4u) == 0u) || (hline && (x % 4u) == 0u)) return LCD_GREY;
    return LCD_BLACK;
}

// Stĺpec px panelu v riadkoch y0..y1 naraz vo výsledných farbách. ttop..tbot = stopa (ttop 0 = bez)
static void PanelColumn(uint16_t px, int16_t y0, int16_t y1, uint16_t ttop, uint16_t tbot) {
    if (y0 < reg_y0) y0 = reg_y0;
    if (y1 > reg_y1) y1 = reg_y1;
    if (y0 > y1 || px <= PX0 || px >= PX0 + PW) return;
    uint32_t l0, v0, lm, vm, lp, vp;
    PanelMasks(px, &l0, &v0);
    uint32_t fg = l0 | v0, ol = 0;
    if (st.meas_panel == PANEL_TRANSP) {                       // obrys: okolie textu o 1 px
        PanelMasks((uint16_t)(px - 1u), &lm, &vm);
        PanelMasks((uint16_t)(px + 1u), &lp, &vp);
        uint32_t n = lm | vm | fg | lp | vp;
        ol = (n | (n << 1) | (n >> 1)) & ~fg;
    }
    LCD_SetAddressWindow(px, (uint16_t)y0, px, (uint16_t)y1);
    for (int16_t y = y0; y <= y1; y++) {
        uint32_t bit = 1ul << (y - reg_y0);
        uint16_t c;
        if (v0 & bit)       c = LCD_WHITE;
        else if (l0 & bit)  c = LCD_LIGHTGREY;
        else if (ol & bit)  c = LCD_BLACK;
        else if (ttop && y >= (int16_t)ttop && y <= (int16_t)tbot) c = LCD_YELLOW;
        else                c = GridColor(px, (uint16_t)y);
        LCD_WriteData((uint8_t)(c >> 8));
        LCD_WriteData((uint8_t)(c & 0xFF));
    }
}

// Obdĺžnik x0..x1, y0..y1 panelu znova (stopa z posledného obrazu)
static void PanelArea(int16_t x0, int16_t x1, int16_t y0, int16_t y1) {
    if (x0 < PX0 + 1) x0 = PX0 + 1;
    if (x1 > PX0 + PW - 1) x1 = PX0 + PW - 1;                // vnútro mriežky (PX0 + PW = obrys)
    for (int16_t x = x0; x <= x1; x++) {
        uint32_t i = (uint32_t)(x - PX0 - 1);
        uint16_t t = have_prev ? prev_top[i] : 0, b = have_prev ? prev_bot[i] : 0;
        PanelColumn((uint16_t)x, y0, y1, t, b);
    }
}

static void PanelRefresh(uint32_t now) {
    if (layout == 0 || layout == 0xFF || st.meas_panel == PANEL_SOLID) { panel_force = false; return; }
    if (panel_force) {                                       // nová plocha: celá naraz
        panel_force = false;
        meas_tick = now;
        for (uint8_t m = 0; m < MEAS_COUNT; m++) MeasValue(m, meas_last[m], sizeof(meas_last[m]));
        PanelArea(PX0 + 1, PX0 + PW - 1, reg_y0, reg_y1);
        return;
    }
    if (now - meas_tick < MEAS_REFRESH_MS) return;
    meas_tick = now;
    uint8_t k = 0;
    for (uint8_t m = 0; m < MEAS_COUNT; m++) {
        uint16_t x, y; uint8_t w;
        if (!MeasPos(m, &k, &x, &y, &w)) continue;
        char v[16];
        MeasValue(m, v, sizeof(v));
        if (strcmp(v, meas_last[m]) == 0) continue;
        snprintf(meas_last[m], sizeof(meas_last[m]), "%s", v);
        // iba obdĺžnik tejto hodnoty (s obrysom), popis sa nemení
        uint16_t vx = (uint16_t)(x + (strlen(MEAS_LBL[m]) + 1u) * 6u);
        PanelArea((int16_t)(vx - 1), (int16_t)(x + w * 6u), (int16_t)(y - 1), (int16_t)(y + 8));
    }
}

static void DrawCell(uint8_t idx, const char *txt, uint16_t fg, uint16_t bg) {
    if (strcmp(txt, last_cells[idx]) == 0 && last_fg[idx] == fg && last_bg[idx] == bg) return;
    snprintf(last_cells[idx], sizeof(last_cells[idx]), "%s", txt);
    last_fg[idx] = fg;
    last_bg[idx] = bg;
    LCD_DrawMenuCell(idx, txt, fg, bg);
}

static bool Waiting(uint32_t now) {      // NORM / SINGLE čaká na hranu
    if (!run || st.mode == 0) return false;
    if (rolling) return roll_state == RS_ARMED;           // v0.69: ROLL čaká na hranu
    if (!trig_seen) return true;
    return (now - last_trig_tick) > 500u + 2u * cap_ms;
}

static void DrawCells(uint32_t now) {
    switch (menu) {
        case M_MAIN:
            DrawCell(0, "V/DIV", LCD_WHITE, LCD_BLACK);
            DrawCell(1, "TIME",  LCD_WHITE, LCD_BLACK);
            DrawCell(2, "TRIG",  LCD_WHITE, LCD_BLACK);
            if (!run)              DrawCell(3, "STOP", LCD_WHITE,  LCD_RED);
            else if (Waiting(now)) DrawCell(3, "WAIT", LCD_BLACK,  LCD_ORANGE);
            else                   DrawCell(3, "RUN",  LCD_SEL_FG, LCD_SEL_BG);
            break;
        // v0.73: pri uloženom priebehu sú funkcie, ktoré naň nemajú vplyv, sivé
        // (držané F2 / F3 ďalej posúvajú stopu zvislo / vodorovne)
        case M_VDIV:
            DrawCell(0, "EXIT",  LCD_CYAN,  LCD_BLACK);
            DrawCell(1, "SENS-", wave_open ? LCD_GREY : LCD_WHITE, LCD_BLACK);
            DrawCell(2, "SENS+", wave_open ? LCD_GREY : LCD_WHITE, LCD_BLACK);
            if (wave_open) DrawCell(3, st.ac ? "AC" : "DC", LCD_GREY, LCD_BLACK);
            else           DrawCell(3, st.ac ? "AC" : "DC", st.ac ? LCD_ORANGE : LCD_GREEN, LCD_BLACK);
            break;
        case M_TIME:
            DrawCell(0, "EXIT",  LCD_CYAN,  LCD_BLACK);
            DrawCell(1, "TIME-", wave_open ? LCD_GREY : LCD_WHITE, LCD_BLACK);
            DrawCell(2, "TIME+", wave_open ? LCD_GREY : LCD_WHITE, LCD_BLACK);
            if (st.tb < TB_ROLL_MIN || wave_open) DrawCell(3, "ROLL", LCD_GREY, LCD_BLACK);
            else if (RollWanted())   DrawCell(3, "ROLL", LCD_SEL_FG, LCD_SEL_BG);
            else                     DrawCell(3, "ROLL", LCD_WHITE, LCD_BLACK);
            break;
        case M_TRIG:
            DrawCell(0, "EXIT",   LCD_CYAN,   LCD_BLACK);
            DrawCell(1, "LEVEL-", wave_open ? LCD_GREY : LCD_WHITE,  LCD_BLACK);
            DrawCell(2, "LEVEL+", wave_open ? LCD_GREY : LCD_WHITE,  LCD_BLACK);
            DrawCell(3, st.slope ? CH_FALL : CH_RISE, wave_open ? LCD_GREY : st.slope ? COL_FALL : COL_RISE, LCD_BLACK);
            break;
        case M_WAVE:
            DrawCell(0, "EXIT", LCD_CYAN,  LCD_BLACK);
            DrawCell(1, "UP",   LCD_WHITE, LCD_BLACK);
            DrawCell(2, "DOWN", LCD_WHITE, LCD_BLACK);
            DrawCell(3, "OPEN", wave_used[wave_sel] ? LCD_WHITE : LCD_GREY, LCD_BLACK);
            break;
        case M_CAL:
            DrawCell(0, "EXIT", LCD_CYAN,  LCD_BLACK);
            DrawCell(1, "NEXT", LCD_WHITE, LCD_BLACK);
            DrawCell(2, cal_sel == 3 ? "CALC" : "", LCD_YELLOW, LCD_BLACK);
            DrawCell(3, "START", LCD_YELLOW, LCD_BLACK);
            break;
        case M_MEAS:
            DrawCell(0, "EXIT", LCD_CYAN,  LCD_BLACK);
            DrawCell(1, "DOWN", LCD_WHITE, LCD_BLACK);
            DrawCell(2, "UP",   LCD_WHITE, LCD_BLACK);
            DrawCell(3, "ON/OFF", LCD_YELLOW, LCD_BLACK);
            break;
        default:   // M_SETUP
            DrawCell(0, "EXIT", LCD_CYAN, LCD_BLACK);
            DrawCell(1, "NEXT", LCD_WHITE, LCD_BLACK);
            if (set_sel <= 6) {
                DrawCell(2, "<", LCD_YELLOW, LCD_BLACK);
                DrawCell(3, ">", LCD_YELLOW, LCD_BLACK);
            } else {
                DrawCell(2, "",      LCD_WHITE, LCD_BLACK);
                DrawCell(3, "START", LCD_YELLOW, LCD_BLACK);
            }
            break;
    }
}

// ---------------------------------------------------------------------------
// Menu osciloskopu (dlhé F1)
// ---------------------------------------------------------------------------
static const char *const SET_HINTS[SET_ITEMS][2] = {
    { "Position of the trigger point on screen:", "1/4, center or 3/4 of the width." },
    { "TIME change: RESET = position back to 0,", "KEEP = the same time shift as before." },
    { "Screen starts this time after trigger.", "Longest delay = about 25 screens." },
    { "All measurements (hold F3): SOLID back,", "TRANSP = see-through, PLAIN = no outline." },
    { "Below 5us/div fewer samples than pixels:", "SIN(X)/X smooth, LINE straight, DOTS samples." },
    { "From 50us/div reads every ADC sample (1MHz)", "and draws min..max band: spikes visible." },
    { "FFT: spectrum 0..fs/2, 10 dB/div from REF.", "TIME -/+ = span, PK = strongest frequency." },
    { "Zero and gain of the scope input,", "saved in EEPROM. F4 = start." },
    { "Manual control of relays and gain (test).", "Hold AUTO in test mode = back to scope." },
};
static uint8_t set_page_drawn = 0xFF;

static void DrawSetup(void) {
    uint8_t page = (uint8_t)(set_sel / SET_PAGE);
    if (page != set_page_drawn) {                    // v0.76: 2 stránky
        set_page_drawn = page;
        LCD_FillRect(1, 28, 318, SET_SEP_Y - 28, LCD_BLACK);
        memset(last_set, 0, sizeof(last_set));
        char pg[12];
        snprintf(pg, sizeof(pg), "PAGE %u/%u", (unsigned)(page + 1u), (unsigned)((SET_ITEMS + SET_PAGE - 1u) / SET_PAGE));
        LCD_DrawString(262, 31, pg, LCD_GREY, LCD_BLACK, 1);
    }
    uint8_t i_end = (uint8_t)((page + 1u) * SET_PAGE);
    if (i_end > SET_ITEMS) i_end = SET_ITEMS;
    for (uint8_t i = (uint8_t)(page * SET_PAGE); i < i_end; i++) {
        char t[40];
        if (i == 0)      snprintf(t, sizeof(t), "%-17s%7s", "Trig position:", TPOS_TXT[st.tpos % 3]);
        else if (i == 1) snprintf(t, sizeof(t), "%-17s%7s", "H-pos on TIME:", st.hkeep ? "KEEP" : "RESET");
        else if (i == 2) snprintf(t, sizeof(t), "%-17s%7s", "Trig delay:", DLY[st.delay_idx % N_DLY].label);
        else if (i == 3) snprintf(t, sizeof(t), "%-11s%13s", "Meas panel:",
                                  st.meas_panel == PANEL_SOLID ? "SOLID" : st.meas_panel == PANEL_TRANSP ? "TRANSP" : "TRANSP PLAIN");
        else if (i == 4) snprintf(t, sizeof(t), "%-16s%8s", "Interp:", INTERP_TXT[sx.interp % 3]);
        else if (i == 5) snprintf(t, sizeof(t), "%-17s%7s", "Peak det:", sx.peak ? "ON" : "OFF");
        else if (i == 6) snprintf(t, sizeof(t), "%-17s%7s", "View:", sx.view == VIEW_FFT ? "FFT" : "WAVE");
        else if (i == 7) snprintf(t, sizeof(t), "%-17s%7s", "Calibration", "");
        else             snprintf(t, sizeof(t), "%-17s%7s", "Test mode", "");
        char key[42];
        snprintf(key, sizeof(key), "%c%s", (i == set_sel) ? '>' : ' ', t);
        if (strcmp(key, last_set[i]) == 0) continue;
        snprintf(last_set[i], sizeof(last_set[i]), "%s", key);
        uint16_t y = (uint16_t)(SET_ROW_Y + (i % SET_PAGE) * SET_ROW_H);
        LCD_DrawString(6, y, (i == set_sel) ? ">" : " ", LCD_YELLOW, LCD_BLACK, 2);
        LCD_DrawString(SET_TXT_X, y, t, (i == set_sel) ? LCD_YELLOW : LCD_WHITE, LCD_BLACK, 2);
    }
    if (last_hint != (int8_t)set_sel) {
        last_hint = (int8_t)set_sel;
        LCD_FillRect(1, SET_SEP_Y + 1, 318, 210 - SET_SEP_Y - 1, LCD_BLACK);
        LCD_DrawString(10, SET_HINT_Y,                 SET_HINTS[set_sel][0], LCD_LIGHTGREY, LCD_BLACK, 1);
        LCD_DrawString(10, SET_HINT_Y + SET_HINT_STEP, SET_HINTS[set_sel][1], LCD_LIGHTGREY, LCD_BLACK, 1);
    }
}

static void OpenSetup(void) {
    PanelOpen(M_SETUP);
    set_sel = 0;
    memset(last_set, 0, sizeof(last_set));
    set_page_drawn = 0xFF;
    last_hint = -1;
    LCD_DrawFastHLine(0, SET_SEP_Y, 320, LCD_WHITE);
    DrawSetup();
}

static void CloseSetup(void) {
    PanelClose();                        // aj nová značka bodu spustenia
}

// ---------------------------------------------------------------------------
// Menu meraní (dlhé F2): 12 položiek v 2 stĺpcoch po 6, prvá = Show (riadok vybraných meraní)
// ---------------------------------------------------------------------------
#define MM_COL_X0   4
#define MM_COL_X1   162
#define MM_ROW_Y    37
#define MM_ROW_H    21

static uint8_t MeasCount(uint16_t mask) {
    uint8_t c = 0;
    for (uint8_t m = 0; m < MEAS_COUNT; m++) if (mask & (1u << m)) c++;
    return c;
}

static void DrawMeasMenu(void) {
    for (uint8_t i = 0; i < MEAS_ITEMS; i++) {
        bool on = (i == 0) ? (st.meas_show != 0) : ((st.meas_mask >> (i - 1)) & 1u);
        const char *name = (i == 0) ? "Show" : MEAS_NAME[i - 1];
        char key[16];
        snprintf(key, sizeof(key), "%c%c%s", (i == meas_sel) ? '>' : ' ', on ? '1' : '0', name);
        if (strcmp(key, meas_menu_last[i]) == 0) continue;
        snprintf(meas_menu_last[i], sizeof(meas_menu_last[i]), "%s", key);
        uint16_t x = (i < 6) ? MM_COL_X0 : MM_COL_X1;
        uint16_t y = (uint16_t)(MM_ROW_Y + (i % 6u) * MM_ROW_H);
        uint16_t c = (i == meas_sel) ? LCD_YELLOW : (i == 0) ? LCD_CYAN : LCD_WHITE;
        char nm[10];
        snprintf(nm, sizeof(nm), "%-6s", name);                // názvy rôznej dĺžky -> fajky v stĺpci
        LCD_DrawString(x, y, (i == meas_sel) ? ">" : " ", LCD_YELLOW, LCD_BLACK, 2);
        LCD_DrawString((uint16_t)(x + 12), y, nm, c, LCD_BLACK, 2);
        LCD_DrawString((uint16_t)(x + 12 + 7 * 12), y, on ? CH_CHECK : " ", LCD_GREEN, LCD_BLACK, 2);
    }
}

static void OpenMeasMenu(void) {
    PanelOpen(M_MEAS);
    meas_sel = 0;
    memset(meas_menu_last, 0, sizeof(meas_menu_last));
    LCD_DrawFastHLine(0, SET_SEP_Y, 320, LCD_WHITE);
    LCD_DrawString(10, SET_HINT_Y,                 "F4 = on / off. Show = row of up to 4 values.", LCD_LIGHTGREY, LCD_BLACK, 1);
    LCD_DrawString(10, SET_HINT_Y + SET_HINT_STEP, "Hold F3 on scope: selected > all > none.", LCD_LIGHTGREY, LCD_BLACK, 1);
    DrawMeasMenu();
}

static void MeasMenuKey(ButtonType_t ev, uint32_t now) {
    if (ev == BTN_F1) { PanelClose(); return; }
    if (ev == BTN_F2) meas_sel = (uint8_t)((meas_sel + 1u) % MEAS_ITEMS);
    if (ev == BTN_F3) meas_sel = (uint8_t)((meas_sel + MEAS_ITEMS - 1u) % MEAS_ITEMS);
    if (ev == BTN_F4) {
        if (meas_sel == 0) {
            st.meas_show = !st.meas_show;
        } else {
            uint16_t bit = (uint16_t)(1u << (meas_sel - 1u));
            if (st.meas_mask & bit)                          st.meas_mask &= (uint16_t)~bit;
            else if (MeasCount(st.meas_mask) < MEAS_MAX_SEL) st.meas_mask |= bit;
            else { Msg("MAX 4 MEASUREMENTS", LCD_ORANGE, 1500); Beep_Warning(); }
        }
    }
    DrawMeasMenu();
}

// ---------------------------------------------------------------------------
// Autoset: zvislý rozsah podľa amplitúdy, časová základňa podľa periódy (asi 3 periódy na obrazovke),
// úroveň triggera do stredu signálu. Hrana a režim spúšťania ostávajú.
// ---------------------------------------------------------------------------
static void Autoset(void) {
    if (rolling) RollEnd();                                  // Task ho podľa základne znova spustí
    uint8_t *v = mv ? &st.vdiv_mv : &st.vdiv_v;
    uint8_t  n = mv ? N_VR_MV : N_VR_V;
    st.offset = 0;
    st.hpos = 0;
    st.tb = 12;                                              // 2 ms/div (aj 50 Hz)

    // 1. Od najmenej citlivého rozsahu: najcitlivejší, kde signál nie je orezaný a je do 3.5 dielika
    int16_t best = -1;
    uint8_t mn = 0, mx = 0, bmn = 0, bmx = 0;
    for (int16_t k = (int16_t)n - 1; k >= 0; k--) {
        *v = (uint8_t)k;
        if (!CaptureBlocking(200)) break;
        MinMax(cap_n, &mn, &mx);
        if (mn <= 2 || mx >= 253) break;                     // orezané -> ostáva predchádzajúci
        int32_t z = Zero10();
        int32_t a1 = (int32_t)mx * 10 - z, a2 = z - (int32_t)mn * 10;
        int32_t amp10 = (a1 > a2) ? a1 : a2;                 // kód x10 od nuly
        uint64_t cpd10 = (uint64_t)CurCpv() * CurRange()->mv / 100000u;   // kódy na dielik x10
        if (cpd10 == 0) cpd10 = 1;
        if ((uint64_t)amp10 * 10u > cpd10 * 35u) break;      // viac ako 3.5 dielika
        best = k; bmn = mn; bmx = mx;
    }
    if (best < 0) { best = (int16_t)n - 1; bmn = mn; bmx = mx; }
    *v = (uint8_t)best;

    // Úroveň triggera do stredu signálu (krok 1/10 dielika)
    int32_t step = (int32_t)CurRange()->mv / 10;
    if (step < 1) step = 1;
    if (bmx - bmn >= 4) {
        int32_t mid = Code10ToMv(((int32_t)bmn + (int32_t)bmx) * 5);
        st.trig_mv = (mid >= 0 ? (mid + step / 2) : (mid - step / 2)) / step * step;
    } else {
        st.trig_mv = 0;
    }
    ClampLevel();

    // 2. Perióda: 20 us/div, 2 ms/div, 20 ms/div – počet prechodov stredom s hysterézou
    static const uint8_t tries[3] = { 6, 12, 15 };
    for (uint8_t t = 0; t < 3; t++) {
        st.tb = tries[t];
        if (!CaptureBlocking(800)) break;
        const uint8_t *s = &buf[WARMUP];
        uint32_t cnt_n = cap_s;                              // jedna obrazovka
        MinMax(cnt_n, &mn, &mx);
        if (mx - mn < 6) break;                              // bez signálu
        int32_t mid = (mn + mx) / 2, hys = (mx - mn) / 4;
        int32_t first = -1, last = -1, cnt = 0;
        bool high = s[0] > mid;
        for (uint32_t i = 1; i < cnt_n; i++) {
            if (!high && s[i] > mid + hys) {
                high = true;
                if (first < 0) first = (int32_t)i;
                last = (int32_t)i;
                cnt++;
            } else if (high && s[i] < mid - hys) {
                high = false;
            }
        }
        if (cnt >= 3) {
            // vzorky na periódu -> ns (vzorka = cap_q16 / 65536 pixelu... pixel = tdiv / 25)
            uint64_t T_ns = (uint64_t)(last - first) * TB[tries[t]].ns * 65536u
                          / ((uint64_t)(cnt - 1) * DIV_X * cap_q16);
            uint8_t k = 0;
            while (k < TB_CLASSIC_MAX && (uint64_t)TB[k].ns * 4u < T_ns) k++;      // asi 3 periódy na 12 dielikov
            st.tb = k;
            return;
        }
    }
    st.tb = TB_DEFAULT;
}

// ---------------------------------------------------------------------------
// Tlačidlá
// ---------------------------------------------------------------------------
// ===========================================================================
// Spoločné: CRC, hlásenie v hornom riadku, panelové menu
// ===========================================================================
static uint16_t Crc16(const uint8_t *d, uint32_t n) {
    uint16_t c = 0xFFFFu;
    while (n--) {
        c ^= (uint16_t)(*d++) << 8;
        for (uint8_t b = 0; b < 8; b++) c = (c & 0x8000u) ? (uint16_t)((c << 1) ^ 0x1021u) : (uint16_t)(c << 1);
    }
    return c;
}

static void Msg(const char *t, uint16_t col, uint32_t ms) {
    msg_text = t;
    msg_col = col;
    msg_until = HAL_GetTick() + ms;
}

static bool PanelMenu(void) { return menu == M_SETUP || menu == M_MEAS || menu == M_WAVE || menu == M_CAL; }

static void PanelOpen(uint8_t m) {
    CaptureStop();
    RollEnd();                           // ROLL po návrate začne znova
    menu = (Menu_t)m;
    LCD_FillRect(1, 28, 318, 184, LCD_BLACK);
}

// Po zatvorení panelového menu: mriežka a pri STOP posledný obraz
static void PanelClose(void) {
    menu = M_MAIN;
    DrawGrid();
    ApplyLayout();
    if (!run && frame_valid && cap_n >= 2) Measure(frame_start);    // obraz nakreslí ApplyLayout
}

// ===========================================================================
// Kalibrácia (EEPROM 0x0180 / 0x01C0, obe kópie rovnaké)
// ===========================================================================
static void CalDefaults(void) {
    memset(&cal, 0, sizeof(cal));
    cal.magic = CAL_MAGIC;
    for (uint8_t i = 0; i < 9; i++) { cal.zero_v[i] = ZERO_DEF[i]; cal.cpv[i] = CPV_DEF[i]; }
    for (uint8_t i = 0; i < 3; i++) cal.zero_mv[i] = ZERO_DEF[i];
}

static bool CalValid(const ScopeCal_t *c) {
    if (c->magic != CAL_MAGIC) return false;
    if (Crc16((const uint8_t *)c, offsetof(ScopeCal_t, crc)) != c->crc) return false;
    for (uint8_t i = 0; i < 9; i++) if (c->cpv[i] == 0 || c->zero_v[i] > 2550) return false;
    return true;
}

static void CalLoad(void) {
    ScopeCal_t t;
    if (EEPROM_Read(EE_SCOPE_CAL_A, (uint8_t *)&t, sizeof(t)) && CalValid(&t)) { cal = t; return; }
    if (EEPROM_Read(EE_SCOPE_CAL_B, (uint8_t *)&t, sizeof(t)) && CalValid(&t)) { cal = t; return; }
    CalDefaults();
}

static bool CalSave(void) {
    cal.magic = CAL_MAGIC;
    cal.crc = Crc16((const uint8_t *)&cal, offsetof(ScopeCal_t, crc));
    bool a = EEPROM_Write(EE_SCOPE_CAL_A, (const uint8_t *)&cal, sizeof(cal));
    bool b = EEPROM_Write(EE_SCOPE_CAL_B, (const uint8_t *)&cal, sizeof(cal));
    return a && b;
}

// Posledné nastavenie osciloskopu (dlhé SAVE)
typedef struct {
    uint16_t     magic;
    ScopeState_t s;
    uint16_t     crc;
} __attribute__((packed)) ScopeSetupRec_t;

// v0.76: záznam v2 (magic "S2") = v1 + ScopeExt_t (32 B, jedna stránka). Záznam v1 ("SS", 28 B) sa
// stále načíta (Interp / Peak det / View potom predvolené).
#define SETUP2_MAGIC 0x5332u
typedef struct {
    uint16_t     magic;
    ScopeState_t s;
    ScopeExt_t   x;
    uint16_t     crc;
} __attribute__((packed)) ScopeSetupRec2_t;
_Static_assert(sizeof(ScopeSetupRec2_t) <= EE_PAGE_SIZE, "setup v2 = 1 stránka");
_Static_assert(sizeof(ScopeSetupRec_t) == 2u + sizeof(ScopeState_t) + 2u, "setup v1");

static bool SetupSave(void) {
    ScopeSetupRec2_t r;
    memset(&r, 0, sizeof(r));
    r.magic = SETUP2_MAGIC;
    ScopeState_t ls;
    LiveState(&ls);                      // v0.74: pri otvorenom uloženom priebehu živé nastavenie
    memcpy((uint8_t *)&r + offsetof(ScopeSetupRec2_t, s), &ls, sizeof(ls));
    memcpy((uint8_t *)&r + offsetof(ScopeSetupRec2_t, x), &sx, sizeof(sx));
    r.crc = Crc16((const uint8_t *)&r, offsetof(ScopeSetupRec2_t, crc));
    bool a = EEPROM_Write(EE_SCOPE_SETUP_A, (const uint8_t *)&r, sizeof(r));
    bool b = EEPROM_Write(EE_SCOPE_SETUP_B, (const uint8_t *)&r, sizeof(r));
    return a && b;
}

static bool SetupLoadFrom(uint16_t addr) {
    uint8_t raw[sizeof(ScopeSetupRec2_t)];
    if (!EEPROM_Read(addr, raw, sizeof(raw))) return false;
    uint16_t magic = (uint16_t)(raw[0] | (raw[1] << 8));
    size_t n;
    if (magic == SETUP2_MAGIC)     n = offsetof(ScopeSetupRec2_t, crc);
    else if (magic == SETUP_MAGIC) n = offsetof(ScopeSetupRec_t, crc);      // v0.75 a staršie
    else return false;
    if (Crc16(raw, (uint32_t)n) != (uint16_t)(raw[n] | (raw[n + 1u] << 8))) return false;
    ScopeState_t t;
    memcpy(&t, raw + 2, sizeof(t));
    Scope_SetState(&t);
    ScopeExt_t x = { .interp = INTERP_SINC, .peak = 0, .view = VIEW_WAVE, .rsv = 0 };
    if (magic == SETUP2_MAGIC) memcpy(&x, raw + offsetof(ScopeSetupRec2_t, x), sizeof(x));
    Scope_SetExt(&x);
    return true;
}

void Scope_LoadEeprom(bool setup) {
    CalLoad();
    if (setup && !SetupLoadFrom(EE_SCOPE_SETUP_A)) (void)SetupLoadFrom(EE_SCOPE_SETUP_B);
}

// --- Kalibračné meranie ---
static void HwCombo(uint8_t c) {         // priamo, bez ApplyHw; DC väzba
    uint8_t att = c / 3, g = c % 3;
    PinSet(GPIOA, PIN_REL_10,   att >= 1);
    PinSet(GPIOA, PIN_O1O2,     att == 2);
    PinSet(GPIOC, PIN_U6,       g == 2);
    PinSet(GPIOC, PIN_U7,       g == 1);
    PinSet(GPIOA, PIN_REL_ACDC, false);
    hw_combo = 0xFF;                     // po kalibrácii znova podľa rozsahu
}

typedef struct { int32_t mean10; int32_t rms100; uint8_t mn, mx; } CalCap_t;

// Zachytenie pri 5 ms/div (~120 ms, 600 vzoriek): priemer (kód x10) a efektívna hodnota striedavej zložky (kód x100)
static bool CalCapture(uint8_t combo, uint32_t settle_ms, CalCap_t *r) {
    HwCombo(combo);
    HAL_Delay(settle_ms);
    uint8_t tb_save = st.tb;
    st.tb = 13;
    cap_plain = true;
    CaptureStart();
    cap_plain = false;
    st.tb = tb_save;
    uint32_t t0 = HAL_GetTick();
    while (!CaptureDone()) if (HAL_GetTick() - t0 > 600u) { CaptureStop(); return false; }
    const uint8_t *s = &buf[WARMUP];
    uint32_t sum = 0;
    uint8_t mn = 255, mx = 0;
    for (uint32_t i = 0; i < cap_n; i++) { sum += s[i]; if (s[i] < mn) mn = s[i]; if (s[i] > mx) mx = s[i]; }
    int32_t mean100 = (int32_t)((sum * 100u + cap_n / 2u) / cap_n);
    int64_t sq = 0;
    for (uint32_t i = 0; i < cap_n; i++) { int32_t d = (int32_t)s[i] * 100 - mean100; sq += (int64_t)d * d; }
    r->mean10 = (mean100 + 5) / 10;
    r->rms100 = (int32_t)ISqrt64((uint64_t)(sq / cap_n));
    r->mn = mn; r->mx = mx;
    return true;
}

static bool NearDefault(uint32_t v, uint32_t def, uint32_t pct) {   // v v rozsahu def +- pct %
    return (uint64_t)v * 100u >= (uint64_t)def * (100u - pct) && (uint64_t)v * 100u <= (uint64_t)def * (100u + pct);
}

#define CAL_ITEMS 4
static int32_t      meter_hist[4];               // posledné napätia z meracieho čipu (mV, absolútne)
static uint32_t     meter_tick;
static uint8_t      meter_n;
static bool         zero_v_ok, zero_mv_ok;       // v tomto sedení
static bool         cal_done[CAL_ITEMS];
static const char  *cal_res = "";
static uint16_t     cal_res_col = LCD_WHITE;
static char         cal_last[CAL_ITEMS][42];     // v0.68: rovnako ako key (bez varovania)
static int8_t       cal_last_hint = -1;
static const char  *cal_last_res = (const char *)1;
static bool         cal_popup;

// Napätie z meracieho čipu (main.c): iba v kalibrácii deliča 1:10 / 1:100, čip je prepnutý na DC V
bool Scope_MeterWanted(void) { return menu == M_CAL && (cal_sel == 2 || cal_sel == 3) && !mv; }

void Scope_SetMeterReading(bool ok, int32_t v01mv) {
    if (!ok) { meter_n = 0; return; }
    int32_t m = (v01mv < 0 ? -v01mv : v01mv) / 10;   // 0.1 mV -> mV
    memmove(&meter_hist[1], &meter_hist[0], sizeof(meter_hist) - sizeof(meter_hist[0]));
    meter_hist[0] = m;
    if (meter_n < 4) meter_n++;
    meter_tick = HAL_GetTick();
}

// Ustálené napätie: aspoň 3 čerstvé hodnoty, rozptyl do 0.5 %
static bool MeterStable(int32_t *mv_out) {
    if (meter_n < 3 || HAL_GetTick() - meter_tick > 1500u) return false;
    int32_t mn = meter_hist[0], mx = meter_hist[0], sum = 0;
    for (uint8_t i = 0; i < 3; i++) {
        if (meter_hist[i] < mn) mn = meter_hist[i];
        if (meter_hist[i] > mx) mx = meter_hist[i];
        sum += meter_hist[i];
    }
    if ((mx - mn) * 200 > mx) return false;
    *mv_out = sum / 3;
    return true;
}

static const char *const CAL_HINTS[CAL_ITEMS][2] = {
    { "Short the probe tips, F4 = start.",           "Do it in position V AND in position mV!" },
    { "Cable OUT gen -> V input, position V.",       "F4 = start (generator on / off itself)." },
    { "Apply 9..30 V DC (e.g. 9 V battery).",        "F4 = start, voltage measured by meter." },
    { "F3 = calculate from 1:10 (no voltage).",      "F4 = measure, apply 20..60 V DC." },
};

static void CalRowText(uint8_t i, char *t, size_t n) {
    char v[16] = "";                                   // v0.68: 16 B (celý rozsah int32 bez varovania)
    if ((i == 2 || i == 3) && i == cal_sel) {         // napätie z čipu pri vybranom kroku
        if (meter_n && HAL_GetTick() - meter_tick < 1500u)
            snprintf(v, sizeof(v), "%ld.%02ldV", (long)(meter_hist[0] / 1000), (long)((meter_hist[0] % 1000) / 10));
        else
            snprintf(v, sizeof(v), "--.--V");
    }
    switch (i) {
        case 0:  snprintf(t, n, "%-6s V:%-2s  mV:%-2s", "Zero", zero_v_ok ? "OK" : "--", zero_mv_ok ? "OK" : "--"); break;
        case 1:  snprintf(t, n, "%-14s%10s", "Gain 1:1",   cal_done[1] ? "OK" : ""); break;
        case 2:  snprintf(t, n, "%-14s%10s", "Gain 1:10",  v[0] ? v : (cal_done[2] ? "OK" : "")); break;
        default: snprintf(t, n, "%-14s%10s", "Gain 1:100", v[0] ? v : (cal_done[3] ? "OK" : "")); break;
    }
}

static void CalDraw(void) {
    if (cal_popup) return;
    for (uint8_t i = 0; i < CAL_ITEMS; i++) {
        char t[40];
        CalRowText(i, t, sizeof(t));
        char key[42];
        snprintf(key, sizeof(key), "%c%s", (i == cal_sel) ? '>' : ' ', t);
        if (strcmp(key, cal_last[i]) == 0) continue;
        snprintf(cal_last[i], sizeof(cal_last[i]), "%s", key);
        uint16_t y = (uint16_t)(SET_ROW_Y + i * SET_ROW_H);
        bool sel = (i == cal_sel);
        char pad[40];
        snprintf(pad, sizeof(pad), "%-24s", t);
        LCD_DrawString(6, y, sel ? ">" : " ", LCD_YELLOW, LCD_BLACK, 2);
        LCD_DrawString(SET_TXT_X, y, pad, sel ? LCD_YELLOW : LCD_WHITE, LCD_BLACK, 2);
    }
    if (cal_last_res != cal_res) {                       // výsledok posledného kroku
        cal_last_res = cal_res;
        LCD_FillRect(1, 144, 318, 20, LCD_BLACK);
        int w = (int)strlen(cal_res) * 12 - 2;
        if (cal_res[0]) LCD_DrawString((uint16_t)((320 - w) / 2), 146, cal_res, cal_res_col, LCD_BLACK, 2);
    }
    if (cal_last_hint != (int8_t)cal_sel) {
        cal_last_hint = (int8_t)cal_sel;
        LCD_FillRect(1, SET_SEP_Y + 1, 318, 210 - SET_SEP_Y - 1, LCD_BLACK);
        LCD_DrawString(10, SET_HINT_Y, CAL_HINTS[cal_sel][0], LCD_LIGHTGREY, LCD_BLACK, 1);
        LCD_DrawString(10, SET_HINT_Y + SET_HINT_STEP, CAL_HINTS[cal_sel][1],
                       cal_sel == 0 ? LCD_YELLOW : LCD_LIGHTGREY, LCD_BLACK, 1);   // V aj mV výrazne
    }
}

static void CalRedrawAll(void) {
    LCD_FillRect(1, 28, 318, SET_SEP_Y - 28, LCD_BLACK);
    memset(cal_last, 0, sizeof(cal_last));
    cal_last_hint = -1;
    cal_last_res = (const char *)1;
    CalDraw();
}

// Okno v strede: pripomenutie druhej polohy pre Zero (zavrie ho ľubovoľné tlačidlo)
static void CalPopup(const char *l1, const char *l2, const char *l3) {
    cal_popup = true;
    LCD_FillRect(30, 50, 260, 100, LCD_BLACK);
    for (uint16_t k = 0; k < 2; k++) {                    // rámik 2 px
        LCD_DrawFastHLine((uint16_t)(30 + k), (uint16_t)(50 + k), (uint16_t)(260 - 2 * k), LCD_YELLOW);
        LCD_DrawFastHLine((uint16_t)(30 + k), (uint16_t)(149 - k), (uint16_t)(260 - 2 * k), LCD_YELLOW);
        LCD_DrawFastVLine((uint16_t)(30 + k), (uint16_t)(50 + k), (uint16_t)(100 - 2 * k), LCD_YELLOW);
        LCD_DrawFastVLine((uint16_t)(289 - k), (uint16_t)(50 + k), (uint16_t)(100 - 2 * k), LCD_YELLOW);
    }
    const char *l[3] = { l1, l2, l3 };
    for (uint8_t i = 0; i < 3; i++) {
        int w = (int)strlen(l[i]) * 12 - 2;
        LCD_DrawString((uint16_t)((320 - w) / 2), (uint16_t)(62 + i * 26), l[i], i == 0 ? LCD_GREEN : LCD_YELLOW, LCD_BLACK, 2);
    }
}

static void OpenCal(void) {
    PanelOpen(M_CAL);
    if (wave_open) SetRun(true);         // v0.73: kalibrácia prepíše zásobník -> po návrate živý obraz
    frame_valid = false;
    pk_valid = false;
    cal_sel = 0;
    cal_res = "";
    cal_popup = false;
    meter_n = 0;
    LCD_DrawFastHLine(0, SET_SEP_Y, 320, LCD_WHITE);
    CalRedrawAll();
}

static void CalResult(bool ok, const char *t) {
    cal_res = t;
    cal_res_col = ok ? LCD_GREEN : LCD_RED;
    if (!ok) Beep_Warning();
}

static void CalZero(void) {
    uint8_t n = mv ? 3u : 9u;
    uint16_t z[9];
    for (uint8_t c = 0; c < n; c++) {
        CalCap_t r;
        if (!CalCapture(c, (c % 3u == 0u) ? SETTLE_RELAY_MS + 40u : SETTLE_MS + 20u, &r)) { CalResult(false, "CAPTURE ERROR"); return; }
        if (r.mean10 < ZERO_DEF[c] - 250 || r.mean10 > ZERO_DEF[c] + 250 || r.mx - r.mn > 20) {
            CalResult(false, "SHORT THE TIPS");
            return;
        }
        z[c] = (uint16_t)r.mean10;
    }
    for (uint8_t c = 0; c < n; c++) { if (mv) cal.zero_mv[c] = z[c]; else cal.zero_v[c] = z[c]; }
    if (mv) zero_mv_ok = true; else zero_v_ok = true;
    cal_done[0] = zero_v_ok && zero_mv_ok;
    if (!CalSave()) { CalResult(false, "EEPROM ERROR"); return; }
    CalResult(true, mv ? "ZERO mV DONE" : "ZERO V DONE");
    if (mv && !zero_v_ok)  CalPopup("ZERO mV DONE", "Switch to V and", "run Zero again.");
    if (!mv && !zero_mv_ok) CalPopup("ZERO V DONE", "Switch to mV and", "run Zero again.");
}

static void CalGain1(void) {
    if (mv) { CalResult(false, "SWITCH TO V"); return; }
    GenParams_t saved = g_gen;                         // signál z vlastného generátora: SINE 100 Hz 3.0 Vpp
    g_gen.wave     = GEN_WAVE_SINE;
    g_gen.freq_idx = 0;
    g_gen.volt_idx = GEN_NUM_VOLT - 1u;
    Gen_SetOutput(true);
    HAL_Delay(300);
    uint32_t cpv[3];
    bool ok = true;
    const char *err = "";
    for (uint8_t g = 0; g < 3 && ok; g++) {
        CalCap_t r;
        if (!CalCapture(g, g == 0 ? SETTLE_RELAY_MS + 40u : SETTLE_MS + 20u, &r)) { ok = false; err = "CAPTURE ERROR"; break; }
        if (r.mx - r.mn < 20)            { ok = false; err = "NO SIGNAL"; break; }
        if (r.mn <= 2 || r.mx >= 253)    { ok = false; err = "SIGNAL CLIPPED"; break; }
        cpv[g] = (uint32_t)(((uint64_t)r.rms100 * 10000u + 530u) / 1061u);   // 3.0 Vpp sínus = 1.0607 V ef.
        if (!NearDefault(cpv[g], CPV_DEF[g], 30)) { ok = false; err = "CAL OUT OF RANGE"; break; }
    }
    bool on = saved.on;
    g_gen = saved;
    Gen_SetOutput(on);                                  // pôvodné nastavenie generátora
    if (!ok) { CalResult(false, err); return; }
    for (uint8_t g = 0; g < 3; g++) cal.cpv[g] = cpv[g];
    cal_done[1] = true;
    if (!CalSave()) CalResult(false, "EEPROM ERROR");
    else            CalResult(true, "GAIN 1:1 DONE");
}

// Delič att (1 = 1:10, 2 = 1:100): meria sa pri najvyššom zosilnení, kde signál nie je orezaný; ostatné
// zosilnenia podľa pomeru z deliča 1:1 (zosilňovač je za deličom). ref_mv = napätie z meracieho čipu.
static const char *CalAtt(uint8_t att, int32_t ref_mv, uint32_t *out) {
    for (int8_t g = 2; g >= 0; g--) {
        uint8_t c = (uint8_t)(att * 3u + (uint8_t)g);
        CalCap_t r;
        if (!CalCapture(c, (g == 2) ? SETTLE_RELAY_MS + 40u : SETTLE_MS + 20u, &r)) return "CAPTURE ERROR";
        if (r.mn <= 2 || r.mx >= 253) continue;         // orezané -> menšie zosilnenie
        int32_t d10 = r.mean10 - (int32_t)cal.zero_v[c];
        if (d10 < 0) d10 = -d10;                        // prehodené hroty
        if (d10 < 40) return "TOO LOW VOLTAGE";         // menej ako 4 kódy
        uint32_t meas = (uint32_t)((uint64_t)d10 * 100000u / (uint32_t)ref_mv);
        for (uint8_t k = 0; k < 3; k++)
            out[k] = (uint32_t)((uint64_t)meas * cal.cpv[k] / cal.cpv[(uint8_t)g]);
        return NearDefault(out[2], CPV_DEF[att * 3u + 2u], 40) ? NULL : "CAL OUT OF RANGE";
    }
    return "SIGNAL CLIPPED";
}

static void CalGainAtt(uint8_t att) {
    if (mv) { CalResult(false, "SWITCH TO V"); return; }
    int32_t ref;
    if (!MeterStable(&ref))               { CalResult(false, "VOLTAGE UNSTABLE"); return; }
    if (att == 1 && ref < 5000)           { CalResult(false, "APPLY 9..30 V DC"); return; }
    if (att == 2 && ref < 20000)          { CalResult(false, "APPLY 20..60 V DC"); return; }
    uint32_t v[3];
    const char *e = CalAtt(att, ref, v);
    if (e) { CalResult(false, e); return; }
    for (uint8_t k = 0; k < 3; k++) cal.cpv[att * 3u + k] = v[k];
    cal_done[att + 1u] = true;
    if (!CalSave()) { CalResult(false, "EEPROM ERROR"); return; }
    CalResult(true, att == 1 ? "GAIN 1:10 DONE" : "GAIN 1:100 DONE");
}

static void CalCalc100(void) {                          // 1:100 = 1:10 / 10 (bez napätia)
    for (uint8_t k = 0; k < 3; k++) cal.cpv[6 + k] = (cal.cpv[3 + k] + 5u) / 10u;
    cal_done[3] = true;
    if (!CalSave()) CalResult(false, "EEPROM ERROR");
    else            CalResult(true, "1:100 = 1:10/10 DONE");
}

static void CalRun(uint8_t what) {                      // 0..3 = krok, 4 = CALC 1:100
    cal_res = "MEASURING...";
    cal_res_col = LCD_CYAN;
    CalDraw();
    switch (what) {
        case 0:  CalZero();       break;
        case 1:  CalGain1();      break;
        case 2:  CalGainAtt(1);   break;
        case 3:  CalGainAtt(2);   break;
        default: CalCalc100();    break;
    }
    hw_combo = 0xFF;
    CalDraw();
}

static void CalKey(ButtonType_t ev, uint32_t now) {
    (void)now;
    if (cal_popup) {                                     // ľubovoľné tlačidlo zavrie okno
        if (ev == BTN_F1 || ev == BTN_F2 || ev == BTN_F3 || ev == BTN_F4 || ev == BTN_SAVE) {
            cal_popup = false;
            CalRedrawAll();
        }
        return;
    }
    if (ev == BTN_F1) { PanelClose(); return; }
    if (ev == BTN_F2) { cal_sel = (uint8_t)((cal_sel + 1u) % CAL_ITEMS); meter_n = 0; }
    if (ev == BTN_F3 && cal_sel == 3) CalRun(4);
    if (ev == BTN_F4) CalRun(cal_sel);
    CalDraw();
}

// ===========================================================================
// Pamäte priebehov (krátke SAVE): 10 x 512 B od EE_WAVE_ADDR (0x0600)
// ===========================================================================
typedef struct {
    uint16_t     magic;
    uint16_t     crc;                    // cez zvyšok hlavičky a vzorky
    ScopeState_t s;
    uint8_t      mv;
    uint8_t      rsv;
    uint16_t     count;                  // vzoriek
    uint16_t     rsv2;
    uint32_t     disp;                   // začiatok obrazovky v uložených vzorkách (Q16)
    uint32_t     q16;                    // vzoriek na pixel
} __attribute__((packed)) WaveHdr_t;
#define WAVE_MAX_SAMPLES (WAVE_SLOT_SIZE - sizeof(WaveHdr_t))

static char     wave_info[WAVE_SLOTS][20];
static uint8_t  wave_rec[WAVE_SLOT_SIZE];    // v0.73: jeden spoločný zásobník (uloženie, otvorenie, náhľady)

static uint16_t WaveAddr(uint8_t i) { return (uint16_t)(EE_WAVE_ADDR + i * WAVE_SLOT_SIZE); }

static const VRange_t *WaveRange(const WaveHdr_t *h) {
    return h->mv ? &VR_MV[h->s.vdiv_mv % N_VR_MV] : &VR_V[h->s.vdiv_v % N_VR_V];
}

static void WaveInfo(uint8_t i, const WaveHdr_t *h) {
    snprintf(wave_info[i], sizeof(wave_info[i]), "%s %s %s", WaveRange(h)->label, TB[h->s.tb % N_TB].label,
             h->s.ac ? "AC" : "DC");
}

// Pamäť i do wave_rec: 0 = prázdna, 1 = v poriadku, 2 = chyba čítania / CRC
enum { WS_EMPTY = 0, WS_OK, WS_ERR };
static uint8_t WaveRead(uint8_t i, WaveHdr_t *h) {
    if (!EEPROM_Read(WaveAddr(i), wave_rec, sizeof(WaveHdr_t))) return WS_ERR;
    memcpy(h, wave_rec, sizeof(*h));
    if (h->magic != WAVE_MAGIC || h->count < 2 || h->count > WAVE_MAX_SAMPLES) return WS_EMPTY;
    if (!EEPROM_Read((uint16_t)(WaveAddr(i) + sizeof(*h)), wave_rec + sizeof(*h), h->count) ||
        Crc16(wave_rec + 4, sizeof(*h) - 4u + h->count) != h->crc) return WS_ERR;
    return WS_OK;
}

// --- v0.73: WAVE MEMORY s náhľadmi: mriežka 2 x 5 (vľavo 1..5, vpravo 6..10), bunka 155 x 35 px ---
#define WC_X0   3
#define WC_DX   158
#define WC_Y0   29
#define WC_DY   36
#define WC_W    155
#define WC_H    35
#define WV_DX   3                        // plocha náhľadu v bunke
#define WV_DY   11
#define WV_W    149
#define WV_H    22

static void WaveCellXY(uint8_t i, uint16_t *x, uint16_t *y) {
    *x = (uint16_t)(WC_X0 + (i / 5u) * WC_DX);
    *y = (uint16_t)(WC_Y0 + (i % 5u) * WC_DY);
}

// Rámik a riadok s textom (číslo pamäte, V/div, čas/div, väzba); vybraná bunka žltý rámik 2 px
static void WaveCellFrame(uint8_t i) {
    uint16_t x, y;
    WaveCellXY(i, &x, &y);
    bool sel = (i == wave_sel);
    uint16_t c1 = sel ? LCD_YELLOW : LCD_GREY, c2 = sel ? LCD_YELLOW : LCD_BLACK;
    LCD_DrawFastHLine(x, y, WC_W, c1);
    LCD_DrawFastHLine(x, (uint16_t)(y + WC_H - 1), WC_W, c1);
    LCD_DrawFastVLine(x, y, WC_H, c1);
    LCD_DrawFastVLine((uint16_t)(x + WC_W - 1), y, WC_H, c1);
    LCD_DrawFastHLine((uint16_t)(x + 1), (uint16_t)(y + 1), WC_W - 2, c2);
    LCD_DrawFastHLine((uint16_t)(x + 1), (uint16_t)(y + WC_H - 2), WC_W - 2, c2);
    LCD_DrawFastVLine((uint16_t)(x + 1), (uint16_t)(y + 1), WC_H - 2, c2);
    LCD_DrawFastVLine((uint16_t)(x + WC_W - 2), (uint16_t)(y + 1), WC_H - 2, c2);
    char n[4];
    snprintf(n, sizeof(n), "%2u", (unsigned)(i + 1u));
    LCD_DrawString((uint16_t)(x + 4), (uint16_t)(y + 3), n, sel ? LCD_YELLOW : LCD_CYAN, LCD_BLACK, 1);
    char t[20];
    snprintf(t, sizeof(t), "%-19s", wave_used[i] ? wave_info[i] : "");
    LCD_DrawString((uint16_t)(x + 22), (uint16_t)(y + 3), t, sel ? LCD_YELLOW : LCD_WHITE, LCD_BLACK, 1);
}

// Náhľad z wave_rec (práve prečítaná / uložená pamäť): obrazovka, ktorá bola pri uložení, zmenšená
// na WV_W x WV_H. Zvisle sa zobrazí iba pásmo so signálom (najmenej 2 dieliky, zväčšenie najviac 4x),
// aby bol tvar čitateľný. Každý stĺpec = min..max zo zodpovedajúcich pixelov obrazovky.
static int16_t WaveSampleY(const WaveHdr_t *h, const uint8_t *s, uint32_t q16, uint32_t sx) {
    int64_t pos = (int64_t)h->disp + (int64_t)sx * q16;
    uint32_t k = (uint32_t)(pos >> 16), f = (uint32_t)(pos & 0xFFFF);
    if (k + 1u >= h->count) { k = h->count - 2u; f = 65535u; }
    int32_t c256 = (int32_t)s[k] * 256 + (((int32_t)s[k + 1] - (int32_t)s[k]) * (int32_t)f) / 256;
    return CodeToYFor(c256, WaveRange(h), h->mv != 0, h->s.offset);
}

static void WaveCellWave(uint8_t i, uint8_t ws, const WaveHdr_t *h) {
    uint16_t x, y;
    WaveCellXY(i, &x, &y);
    uint16_t wx = (uint16_t)(x + WV_DX), wy = (uint16_t)(y + WV_DY);
    LCD_FillRect(wx, wy, WV_W, WV_H, LCD_BLACK);
    if (ws != WS_OK) {
        const char *t = (ws == WS_EMPTY) ? "EMPTY" : "DATA ERROR";
        int w = (int)strlen(t) * 6 - 1;
        LCD_DrawString((uint16_t)(wx + (WV_W - w) / 2), (uint16_t)(wy + (WV_H - 7) / 2), t,
                       ws == WS_EMPTY ? LCD_GREY : LCD_RED, LCD_BLACK, 1);
        return;
    }
    const uint8_t *s = wave_rec + sizeof(WaveHdr_t);
    uint32_t q16 = h->q16 ? h->q16 : 65536u;
    int16_t y0 = Y_MAX, y1 = Y_MIN;                          // pásmo signálu na obrazovke
    for (uint32_t sx = 0; sx + 1u < PW; sx++) {
        int16_t sy = WaveSampleY(h, s, q16, sx);
        if (sy < y0) y0 = sy;
        if (sy > y1) y1 = sy;
    }
    int32_t span = (int32_t)(y1 - y0) * 23 / 20 + 2;         // +15 % okraj
    if (span < 2 * DIV_Y) span = 2 * DIV_Y;
    if (span > Y_MAX - Y_MIN) span = Y_MAX - Y_MIN;
    int32_t w0 = (y0 + y1) / 2 - span / 2;
    if (w0 < Y_MIN) w0 = Y_MIN;
    if (w0 + span > Y_MAX) w0 = Y_MAX - span;
    int16_t last = -1;
    for (uint16_t j = 0; j < WV_W; j++) {
        uint32_t xa = (uint32_t)j * (PW - 1u) / WV_W, xb = (uint32_t)(j + 1u) * (PW - 1u) / WV_W;
        if (xb <= xa) xb = xa + 1u;
        int16_t mn = (last >= 0) ? last : 999, mx = (last >= 0) ? last : -1;
        for (uint32_t sx = xa; sx < xb; sx++) {
            int32_t d = WaveSampleY(h, s, q16, sx) - w0;
            if (d < 0) d = 0;
            if (d > span) d = span;
            int16_t py = (int16_t)(wy + d * (WV_H - 1) / span);
            if (py < mn) mn = py;
            if (py > mx) mx = py;
            last = py;
        }
        LCD_DrawFastVLine((uint16_t)(wx + j), (uint16_t)mn, (uint16_t)(mx - mn + 1), LCD_YELLOW);
    }
}

static void OpenWave(void) {
    PanelOpen(M_WAVE);
    if (wave_open) wave_sel = (uint8_t)(wave_open - 1u);        // kurzor na otvorenom priebehu
    for (uint8_t i = 0; i < WAVE_SLOTS; i++) {
        WaveHdr_t h;
        uint8_t ws = WaveRead(i, &h);
        wave_used[i] = (ws == WS_OK);
        if (wave_used[i]) WaveInfo(i, &h);
        WaveCellFrame(i);
        WaveCellWave(i, ws, &h);
    }
}

static void WaveSave(uint8_t i) {
    if (!frame_valid || cap_n < 2) { Msg("NO WAVEFORM", LCD_RED, 1500); Beep_Warning(); return; }
    uint8_t *rec = wave_rec;
    WaveHdr_t h;
    memset(&h, 0, sizeof(h));
    // Okno do WAVE_MAX_SAMPLES vzoriek okolo zobrazenej obrazovky (aby sa dalo v pamäti posúvať)
    int64_t fs = frame_start < 0 ? 0 : frame_start;
    uint32_t sw = (uint32_t)(((uint64_t)(PW - 1) * cap_q16) >> 16) + 2u;
    uint32_t first = (uint32_t)(fs >> 16);
    uint32_t margin = (WAVE_MAX_SAMPLES > sw) ? (uint32_t)(WAVE_MAX_SAMPLES - sw) / 2u : 0u;
    uint32_t i0 = (first > margin) ? first - margin : 0u;
    if (i0 >= cap_n) i0 = 0;
    uint32_t cnt = cap_n - i0;
    if (cnt > WAVE_MAX_SAMPLES) cnt = WAVE_MAX_SAMPLES;
    h.magic = WAVE_MAGIC;
    h.s = st;
    h.mv = mv;
    h.count = (uint16_t)cnt;
    h.disp = (uint32_t)(fs - ((int64_t)i0 << 16));
    h.q16 = cap_q16;
    memcpy(rec + sizeof(h), &buf[WARMUP + i0], cnt);
    memcpy(rec, &h, sizeof(h));
    h.crc = Crc16(rec + 4, sizeof(h) - 4u + cnt);
    memcpy(rec, &h, sizeof(h));
    if (EEPROM_Write(WaveAddr(i), rec, (uint16_t)(sizeof(h) + cnt))) {
        static char m[20];
        wave_used[i] = true;
        WaveInfo(i, &h);
        WaveCellFrame(i);
        WaveCellWave(i, WS_OK, &h);                              // nový náhľad hneď
        snprintf(m, sizeof(m), "SAVED TO WAVE %u", (unsigned)(i + 1u));
        Msg(m, LCD_GREEN, 1500);
    } else {
        Msg("SAVE ERROR", LCD_RED, 1500);
        Beep_Warning();
    }
}

static void WaveLoad(uint8_t i) {
    WaveHdr_t h;
    uint8_t ws = WaveRead(i, &h);
    if (ws == WS_EMPTY) { Msg("EMPTY", LCD_ORANGE, 1500); return; }
    if (ws != WS_OK)    { Msg("DATA ERROR", LCD_RED, 1500); Beep_Warning(); return; }
    if ((h.mv != 0) != mv) { Msg(h.mv ? "SAVED IN mV" : "SAVED IN V", LCD_ORANGE, 2000); Beep_Warning(); return; }
    // v0.74: živé nastavenie odložiť (iba pri prvom OPEN; pri OPEN ďalšej pamäte ostáva pôvodné živé)
    if (!wave_open) live_st = st;
    // Nastavenie z pamäte (výber meraní ostáva), obraz ako STOP
    uint16_t mm = st.meas_mask; uint8_t ms = st.meas_show, ma = st.meas_all;
    ScopeState_t ls;
    memcpy(&ls, wave_rec + offsetof(WaveHdr_t, s), sizeof(ls));
    Scope_SetState(&ls);
    st.meas_mask = mm; st.meas_show = ms; st.meas_all = ma;
    run = false;
    memcpy(&buf[WARMUP], wave_rec + sizeof(h), h.count);
    pk_valid = false;                                   // v0.76: uložený priebeh bez pásma Peak det
    cap_n = h.count;
    cap_q16 = h.q16 ? h.q16 : 65536u;
    cap_s = (uint16_t)((((uint64_t)(PW - 1) * cap_q16) >> 16) + 2u);
    if (cap_s > cap_n) cap_s = cap_n;
    frame_start = h.disp;
    frame_anchor = (int64_t)h.disp - OffQ16(cap_q16);   // posun ďalej od tejto polohy
    frame_valid = true;
    rolling = false;
    wave_open = (uint8_t)(i + 1u);                      // v0.73: oranžový rámik, "WAVE n", sivé funkcie
    title_last[0] = '\0';
    ApplyHw();
    PanelClose();
    static char m[20];
    snprintf(m, sizeof(m), "WAVE %u OPENED", (unsigned)(i + 1u));
    Msg(m, LCD_GREEN, 1500);
}

// Malé sivé texty hore v mriežke (panel meraní je dole):
//   v0.73 vpravo "WAVE n" (uložený priebeh), v0.74 vľavo "DLY 1ms" pri zapnutom Trig delay (živý aj uložený)
//   v0.76 vľavo "PEAK" (kreslí sa pásmo Peak det); View FFT: vľavo "REF +10dBV 10dB/div" a "SPAN 12.5kHz",
//   vpravo "PK 1.00kHz" (pod ním "WAVE n")
static void GridTags(void) {
    uint16_t yl = (uint16_t)(PY0 + 3), yr = (uint16_t)(PY0 + 3);
    char t[32], v[16];
    if (sx.view == VIEW_FFT) {
        snprintf(t, sizeof(t), "REF %+ddBV 10dB/div", (int)fft_ref);
        LCD_DrawString((uint16_t)(PX0 + 3), yl, t, LCD_LIGHTGREY, LCD_BLACK, 1);
        yl = (uint16_t)(yl + 10u);
        Fmt3(v, sizeof(v), fft_span, "Hz");
        snprintf(t, sizeof(t), "SPAN %-8s", v);
        LCD_DrawString((uint16_t)(PX0 + 3), yl, t, LCD_LIGHTGREY, LCD_BLACK, 1);
        yl = (uint16_t)(yl + 10u);
        if (fft_pk >= 0.0f) Fmt3(v, sizeof(v), fft_pk, "Hz"); else snprintf(v, sizeof(v), "---");
        char pk[20];
        snprintf(pk, sizeof(pk), "PK %s", v);
        snprintf(t, sizeof(t), "%13s", pk);                  // pevná šírka (zmaže dlhší starý text)
        int w = (int)strlen(t) * 6 - 1;
        LCD_DrawString((uint16_t)(PX0 + PW - 3 - w), yr, t, LCD_YELLOW, LCD_BLACK, 1);
        yr = (uint16_t)(yr + 10u);
    } else if (st.delay_idx % N_DLY) {
        snprintf(t, sizeof(t), "DLY %s", DLY[st.delay_idx % N_DLY].label);
        LCD_DrawString((uint16_t)(PX0 + 3), yl, t, LCD_LIGHTGREY, LCD_BLACK, 1);
        yl = (uint16_t)(yl + 10u);
    }
    if (pk_valid && sx.view != VIEW_FFT) LCD_DrawString((uint16_t)(PX0 + 3), yl, "PEAK", LCD_LIGHTGREY, LCD_BLACK, 1);
    if (!wave_open) return;
    snprintf(t, sizeof(t), "WAVE %u", (unsigned)wave_open);
    int w = (int)strlen(t) * 6 - 1;
    LCD_DrawString((uint16_t)(PX0 + PW - 3 - w), yr, t, LCD_LIGHTGREY, LCD_BLACK, 1);
}

// v0.74: živé nastavenie (pri otvorenom uloženom priebehu odložené pri OPEN). Výber a zobrazenie
// meraní sú spoločné pre živý aj uložený obraz -> vždy aktuálne.
static void LiveState(ScopeState_t *s) {
    *s = wave_open ? live_st : st;
    s->meas_mask  = st.meas_mask;
    s->meas_show  = st.meas_show;
    s->meas_all   = st.meas_all;
    s->meas_panel = st.meas_panel;
}

// v0.74: obnoviť živé nastavenie (bez kreslenia); volá WaveClose a Scope_Close
static void WaveRestoreLive(void) {
    if (!wave_open) return;
    ScopeState_t s;
    LiveState(&s);
    st = s;
    wave_open = 0;
    title_last[0] = '\0';
    ClampLevel();
}

// Späť na živý obraz (RUN, prepnutie V / mV, kalibrácia, nový vstup do osciloskopu).
// v0.74: obnoví živé nastavenie spred OPEN (rozsah, základňa, trigger, posuny...)
static void WaveClose(void) {
    if (!wave_open) return;
    WaveRestoreLive();
    Restart();                                           // rozsah / väzba späť na živé, nové zachytenie
    if (!PanelMenu()) DrawGrid();                        // sivý rámik, bez "WAVE n"
}

static void WaveKey(ButtonType_t ev) {
    uint8_t old = wave_sel;
    if (ev == BTN_F1)   { PanelClose(); return; }
    if (ev == BTN_F2)   wave_sel = (uint8_t)((wave_sel + WAVE_SLOTS - 1u) % WAVE_SLOTS);   // UP
    if (ev == BTN_F3)   wave_sel = (uint8_t)((wave_sel + 1u) % WAVE_SLOTS);                // DOWN
    if (ev == BTN_SAVE) WaveSave(wave_sel);
    if (ev == BTN_F4)   { if (wave_used[wave_sel]) { WaveLoad(wave_sel); return; } Beep_Warning(); }
    if (old != wave_sel) { WaveCellFrame(old); WaveCellFrame(wave_sel); }
}

// ===========================================================================
// ROLL: hodiny ADC bežia stále (1 MHz), program číta port a priemeruje za čas jedného pixelu
// ===========================================================================
static bool RollWanted(void) { return (st.roll && st.tb >= TB_ROLL_MIN) || st.tb > TB_CLASSIC_MAX; }

static void RollClock(bool on) {
    if (!on) { TIM11->CR1 = 0; return; }
    RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
    (void)RCC->APB2ENR;
    TIM11->PSC   = 0;
    TIM11->ARR   = ADC_SLOW - 1u;
    TIM11->CCR1  = ADC_SLOW / 2u;
    TIM11->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM11->CCER  = TIM_CCER_CC1E;
    TIM11->EGR   = TIM_EGR_UG;
    TIM11->CR1   = TIM_CR1_CEN;
}

static void RollReset(void) {
    roll_cnt = 0;
    roll_head = 0;
    roll_acc = 0;
    roll_acc_n = 0;
    roll_next_us = (uint64_t)HAL_GetTick() * 1000u;
    roll_new = true;
    roll_view_cnt = 0;                   // nový rozsah / základňa: starý obraz neplatí
    roll_redraw = true;
    RollArm();
}

// v0.69: AUTO = voľné rolovanie, NORM / SINGLE = čakať na hranu (posledný obraz ostane)
static void RollArm(void) {
    roll_state = (st.mode == 0) ? RS_FREE : RS_ARMED;
    roll_edge_armed = false;
    roll_post = 0;
}

// Kópia kruhového bufra do zobrazenia (od najstaršej vzorky)
static void RollSnapshot(void) {
    uint16_t c = roll_cnt;
    for (uint16_t k = 0; k < c; k++) {
        uint16_t j = (uint16_t)((roll_head + ROLL_N - c + k) % ROLL_N);
        roll_view[k] = roll_buf[j];
        roll_vmn[k] = roll_bmn[j];                           // v0.76 Peak det
        roll_vmx[k] = roll_bmx[j];
    }
    roll_view_cnt = c;
}

static void RollBegin(void) {
    CaptureStop();
    roll_pk = (sx.peak != 0);                                // v0.76: Peak det -> všetky vzorky cez DMA
    if (roll_pk) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM11EN;
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
        (void)RCC->APB2ENR;
        pk_rmn = 255; pk_rmx = 0;
        pk_on = false; pk_roll = true;
        PeakHwStart();
    } else {
        RollClock(true);
    }
    RollReset();
    rolling = true;
}

static void RollEnd(void) {
    if (roll_pk) CaptureStop();                              // zastaví aj TIM11
    roll_pk = false;
    RollClock(false);
    rolling = false;
}

// Poradie od najstaršej vzorky do buf (merania, uloženie)
static void RollToBuf(void) {
    uint16_t c = roll_view_cnt;                              // v0.69: zobrazený obraz
    for (uint16_t k = 0; k < c; k++) buf[WARMUP + k] = roll_view[k];
    if (roll_pk) for (uint16_t k = 0; k < c; k++) { PK_MN[k] = roll_vmn[k]; PK_MX[k] = roll_vmx[k]; }
    pk_valid = roll_pk;
    cap_n = c; cap_s = c; cap_q16 = 65536u;
    frame_start = 0;
    frame_anchor = -OffQ16(65536u);
    frame_valid = (c >= 2);
}

static void DrawRoll(void) {
    uint16_t c = roll_view_cnt;                              // v0.69: zo snímky
    bool dots = (sx.interp == INTERP_DOTS);                  // v0.76
    int16_t yp = -1, bt = 0, bb = 0;
    for (uint32_t x = 0; x + 1u < PW; x++) {
        uint32_t from_right = (PW - 2u) - x;                // 0 = najnovšia vzorka vpravo
        uint16_t top = 0, bot = 0;
        if (from_right < c) {
            uint32_t k = c - 1u - from_right;
            if (roll_pk) {                                   // v0.76 Peak det: pásmo min..max
                int16_t yt = CodeToY((int32_t)roll_vmx[k] * 256), yb = CodeToY((int32_t)roll_vmn[k] * 256);
                int16_t ot = yt, ob = yb;
                if (!dots && yp >= 0) { if (yt > bb) yt = bb; if (yb < bt) yb = bt; }
                bt = ot; bb = ob; yp = 0;
                top = (uint16_t)yt; bot = (uint16_t)yb;
            } else {
                int16_t y = CodeToY((int32_t)roll_view[k] * 256);
                if (yp < 0 || dots) yp = y;
                top = (uint16_t)((y < yp) ? y : yp);
                bot = (uint16_t)((y < yp) ? yp : y);
                yp = y;
            }
        }
        TraceColumn(x, top, bot);
    }
    have_prev = true;
    GridTags();
}

static void RollDrawNow(uint32_t now) {
    roll_draw_tick = now;
    if (sx.view == VIEW_FFT) {                               // v0.76: spektrum zobrazených vzoriek ROLL
        RollToBuf();
        DrawFft(&buf[WARMUP], cap_n, 0);
        if (layout == 1 || layout == 2) Measure(0);
        return;
    }
    DrawRoll();
    RollToBuf();
    if (layout == 1 || layout == 2) Measure(0);
}

// v0.69: hrana v surovej vzorke (kód 0..255) s hysterézou ako pri bežnom triggeri
static bool RollEdge(uint8_t code) {
    int32_t lvl = MvToCode256(st.trig_mv);
    int32_t cpd256 = (int32_t)((uint64_t)CurCpv() * CurRange()->mv * 256u / 1000000u);
    int32_t hys = cpd256 / 10;
    if (hys < 384) hys = 384;
    int32_t v = (int32_t)code * 256;
    if (st.slope == 0) {
        if (v < lvl - hys) roll_edge_armed = true;
        return roll_edge_armed && v >= lvl;
    }
    if (v > lvl + hys) roll_edge_armed = true;
    return roll_edge_armed && v <= lvl;
}

static void RollTask(uint32_t now) {
    if (!have_prev || roll_redraw) {                         // mriežka prekreslená / nový rozsah
        roll_redraw = false;
        RollDrawNow(now);
    }
    if (!run) return;
    if ((st.mode == 0) != (roll_state == RS_FREE)) RollArm();   // zmena AUTO <-> NORM / SINGLE

    uint8_t code = (uint8_t)(GPIOC->IDR & 0xFFu);
    roll_acc += code;
    roll_acc_n++;

    // NORM / SINGLE: čakanie na hranu – vzorky sa zbierajú (pre-trigger), obraz stojí
    if (roll_state == RS_ARMED && RollEdge(code)) {
        int16_t pre = (int16_t)(MarkX() - 1);                // bod spustenia (značka na PX0 + x, stopa na PX0 + 1 + x)
        if (pre < 0) pre = 0;
        if (pre > (int16_t)(PW - 2)) pre = (int16_t)(PW - 2);
        roll_post = (uint16_t)((PW - 2) - pre + 1);          // pixel s hranou + pixely za ním
        roll_state = RS_POST;
    }

    uint64_t now_us = (uint64_t)now * 1000u;
    uint64_t per_us = TB[st.tb % N_TB].ns / 25000u;        // čas pixelu
    if (now_us >= roll_next_us) {
        roll_buf[roll_head] = (uint8_t)((roll_acc + roll_acc_n / 2u) / roll_acc_n);
        if (roll_pk) {                                       // v0.76: min / max zo všetkých vzoriek 1 MHz
            __disable_irq();
            uint8_t mn = pk_rmn, mx = pk_rmx;
            pk_rmn = 255; pk_rmx = 0;
            __enable_irq();
            if (mn > mx) { mn = roll_buf[roll_head]; mx = mn; }
            roll_bmn[roll_head] = mn;
            roll_bmx[roll_head] = mx;
            roll_buf[roll_head] = (uint8_t)(((uint32_t)mn + mx + 1u) >> 1);
        }
        roll_head = (uint16_t)((roll_head + 1u) % ROLL_N);
        if (roll_cnt < ROLL_N) roll_cnt++;
        roll_acc = 0; roll_acc_n = 0;
        roll_next_us += per_us;
        if (now_us > roll_next_us + per_us) roll_next_us = now_us + per_us;   // oneskorenie: znova od teraz
        roll_new = true;
        if (roll_state == RS_POST && roll_post && --roll_post == 0) {
            // Hrana je na značke: obraz zastaviť
            RollSnapshot();
            roll_new = false;
            RollDrawNow(now);
            if (st.mode == 2) SetRun(false);                 // SINGLE -> STOP
            RollArm();                                       // NORM: čakať na ďalšiu hranu
            return;
        }
    }
    if (roll_state == RS_ARMED) return;                      // obraz stojí
    if (roll_new && now - roll_draw_tick >= 40u) {
        roll_new = false;
        RollSnapshot();
        RollDrawNow(now);
    }
}

static void Restart(void) {           // nový rozsah / základňa -> zahodiť rozbehnuté zachytenie
    if (capturing) CaptureStop();
    ApplyHw();
    trig_seen = false;
    misses = 0;
    auto_ref = HAL_GetTick();
    if (rolling) RollReset();
}

static void StepSens(int dir) {       // dir > 0 = citlivejšie (menej V/div)
    uint8_t *v = mv ? &st.vdiv_mv : &st.vdiv_v;
    uint8_t n = mv ? N_VR_MV : N_VR_V;
    if (dir > 0 && *v > 0) (*v)--;
    if (dir < 0 && *v + 1u < n) (*v)++;
    ClampLevel();
    Restart();
}

static void StepLevel(int dir) {
    int32_t step = (int32_t)CurRange()->mv / 10;
    if (step < 1) step = 1;
    st.trig_mv += dir * step;
    ClampLevel();
    trig_seen = false;
}

static void SetRun(bool on) {
    if (on) WaveClose();                   // v0.73: RUN = späť z uloženého priebehu na živý obraz
    run = on;
    if (on && rolling) RollArm();          // v0.69: NORM / SINGLE v ROLL znova čaká na hranu
    trig_seen = false;
    misses = 0;
    auto_ref = HAL_GetTick();
}

// Časová základňa; vodorovný posun podľa voľby H-pos on TIME (RESET / KEEP rovnaký čas)
static void StepTime(int dir) {
    uint8_t old = st.tb;
    if (dir < 0 && st.tb > 0) st.tb--;
    if (dir > 0 && st.tb + 1u < N_TB) st.tb++;
    if (st.tb == old) return;
    if (st.hkeep) {
        int64_t h = (int64_t)st.hpos * (int64_t)TB[old % N_TB].ns / (int64_t)TB[st.tb % N_TB].ns;
        if (h > HPOS_MAX) h = HPOS_MAX;
        if (h < -HPOS_MAX) h = -HPOS_MAX;
        st.hpos = (int16_t)h;
    } else {
        st.hpos = 0;
    }
    Restart();
}

static void StepHpos(int d) {
    int32_t h = st.hpos + d;
    if (h > HPOS_MAX) h = HPOS_MAX;
    if (h < -HPOS_MAX) h = -HPOS_MAX;
    if (h == st.hpos) return;
    st.hpos = (int16_t)h;
    static char m[20];
    char v[14];
    FmtSI(v, sizeof(v), (float)((double)st.hpos * (double)TdivNs() / 25.0 * 1e-9), "s");
    snprintf(m, sizeof(m), "POS %s%s", st.hpos > 0 ? "+" : "", v);
    Msg(m, LCD_CYAN, 1000);
    if (!run && frame_valid && cap_n >= 2) {               // STOP / pamäť: listovanie v zachytených dátach
        int64_t fs = frame_anchor + OffQ16(cap_q16);
        int64_t smax = ((int64_t)cap_n - cap_s) << 16;
        if (fs > smax) fs = smax;
        if (fs < 0) fs = 0;
        frame_start = fs;
        Measure(fs);
        DrawGrid();                                         // nová značka; ApplyLayout obraz nakreslí
    }
}

static ScopeAction_t Buttons(ButtonType_t ev, uint32_t now) {
    uint32_t lp = Settings_LongPressMs();

    // AUTO: krátke = autoset (v TRIG = režim spúšťania), dlhé = odchod
    if (ev == BTN_AUTO) { auto_pending = true; auto_tick = now; ev = BTN_NONE; }
    if (auto_pending) {
        if (BUTTON_GetHeld() == BTN_AUTO) {
            if (now - auto_tick >= lp) { auto_pending = false; return SCOPE_EXIT; }
        } else {
            auto_pending = false;
            if (wave_open && !PanelMenu()) {
                Beep_Warning();                               // v0.73: na uložený priebeh nie (režim, autoset)
            } else if (menu == M_TRIG) {
                st.mode = (uint8_t)((st.mode + 1u) % 3u);
                SetRun(true);                                 // SINGLE: čakať na hranu
            } else if (!PanelMenu()) {
                Autoset();
                Restart();
                DrawGrid();
            }
        }
    }

    // SAVE: krátke = pamäte priebehov, dlhé = uložiť posledné nastavenie (v pamätiach SAVE = uložiť priebeh)
    if (ev == BTN_SAVE && !PanelMenu()) { save_pending = true; save_tick = now; ev = BTN_NONE; }
    if (save_pending) {
        if (BUTTON_GetHeld() == BTN_SAVE) {
            if (now - save_tick >= lp) {
                save_pending = false;
                if (SetupSave()) Msg("SETUP SAVED", LCD_GREEN, 1500);
                else           { Msg("SAVE ERROR", LCD_RED, 1500); Beep_Warning(); }
            }
        } else {
            save_pending = false;
            OpenWave();
            return SCOPE_STAY;
        }
    }

    // F1..F3 v hlavnom menu: krátke pri pustení, dlhé = menu osciloskopu / menu meraní / všetky merania
    if (menu == M_MAIN && (ev == BTN_F1 || ev == BTN_F2 || ev == BTN_F3)) { key_pending = ev; key_tick = now; ev = BTN_NONE; }
    if (key_pending != BTN_NONE) {
        if (BUTTON_GetHeld() == key_pending) {
            if (now - key_tick >= lp) {
                ButtonType_t k = key_pending;
                key_pending = BTN_NONE;
                if (k == BTN_F1) OpenSetup();
                if (k == BTN_F2) OpenMeasMenu();
                if (k == BTN_F3) {                             // v0.77: vybrané -> všetky -> žiadne -> ...
                    if (st.meas_all) {                          // všetky -> žiadne
                        st.meas_all = 0;
                        st.meas_show = 0;
                    } else if (st.meas_show && st.meas_mask) {  // vybrané -> všetky
                        st.meas_all = 1;
                    } else if (st.meas_mask) {                  // žiadne -> vybrané
                        st.meas_show = 1;
                    } else {                                    // žiadne vybrané: žiadne -> všetky
                        st.meas_all = 1;
                    }
                }                                               // ApplyLayout prekreslí
            }
        } else {
            ButtonType_t k = key_pending;
            key_pending = BTN_NONE;
            if (k == BTN_F1) menu = M_VDIV;
            if (k == BTN_F2) menu = M_TIME;
            if (k == BTN_F3) menu = M_TRIG;
        }
    }

    switch (menu) {
        case M_MAIN:
            if (ev == BTN_F4) SetRun(!run);
            break;

        case M_MEAS:
            MeasMenuKey(ev, now);
            break;

        case M_WAVE:
            WaveKey(ev);
            break;

        case M_CAL:
            CalKey(ev, now);
            break;

        case M_VDIV:
            if (ev == BTN_F1) menu = M_MAIN;
            if (ev == BTN_F4) { if (wave_open) Beep_Warning(); else { st.ac = !st.ac; Restart(); } }
            // F2 / F3: krátke = citlivosť (F2 menej, F3 viac), držané = posun stopy (F2 dole, F3 hore)
            if (ev == BTN_F2 || ev == BTN_F3) { adj_pending = true; adj_btn = ev; adj_tick = now; adj_rep = now + lp; }
            if (adj_pending) {
                if (BUTTON_GetHeld() == adj_btn) {
                    if ((int32_t)(now - adj_rep) >= 0) {
                        adj_rep = now + 120u;
                        if (adj_btn == BTN_F3 && st.offset <  20) st.offset++;
                        if (adj_btn == BTN_F2 && st.offset > -20) st.offset--;
                        ClampLevel();
                        if (!run && !rolling && frame_valid && cap_n >= 2) DrawTrace(frame_start);   // v0.73: STOP / pamäť
                    }
                } else {
                    adj_pending = false;
                    if (now - adj_tick < lp) {
                        if (wave_open) Beep_Warning();       // v0.73: rozsah uloženého priebehu sa nemení
                        else           StepSens(adj_btn == BTN_F3 ? +1 : -1);
                    }
                }
            }
            break;

        case M_TIME:
            if (ev == BTN_F1) menu = M_MAIN;
            // F2 / F3: krátke = TIME- / TIME+, držané = vodorovný posun (F2 skôr, F3 neskôr)
            if (ev == BTN_F2 || ev == BTN_F3) { adj_pending = true; adj_btn = ev; adj_tick = now; adj_rep = now + lp; }
            if (adj_pending) {
                if (BUTTON_GetHeld() == adj_btn) {
                    if ((int32_t)(now - adj_rep) >= 0) {
                        adj_rep = now + 120u;
                        StepHpos(adj_btn == BTN_F3 ? +HPOS_STEP : -HPOS_STEP);
                    }
                } else {
                    adj_pending = false;
                    if (now - adj_tick < lp) {
                        if (wave_open) Beep_Warning();       // v0.73: základňa uloženého priebehu sa nemení
                        else           StepTime(adj_btn == BTN_F3 ? +1 : -1);
                    }
                }
            }
            if (ev == BTN_F4 && wave_open) Beep_Warning();
            else if (ev == BTN_F4 && st.tb >= TB_ROLL_MIN && st.tb <= TB_CLASSIC_MAX) { st.roll = !st.roll; Restart(); }
            break;

        case M_TRIG:
            if (ev == BTN_F1) menu = M_MAIN;
            if (wave_open && (ev == BTN_F2 || ev == BTN_F3 || ev == BTN_F4)) { Beep_Warning(); break; }   // v0.73
            if (ev == BTN_F4) { st.slope = !st.slope; trig_seen = false; }
            // F2 / F3: krok pri stlačení, držané = opakovanie
            if (ev == BTN_F2 || ev == BTN_F3) {
                lvl_pending = true; lvl_btn = ev; lvl_rep = now + lp;
                StepLevel(ev == BTN_F3 ? +1 : -1);
            }
            if (lvl_pending) {
                if (BUTTON_GetHeld() == lvl_btn) {
                    if ((int32_t)(now - lvl_rep) >= 0) {
                        lvl_rep = now + 80u;
                        StepLevel(lvl_btn == BTN_F3 ? +1 : -1);
                    }
                } else {
                    lvl_pending = false;
                }
            }
            break;

        default:   // M_SETUP
            if (ev == BTN_F1) { CloseSetup(); break; }
            if (ev == BTN_F2) set_sel = (uint8_t)((set_sel + 1u) % SET_ITEMS);
            if (set_sel == 0) {                              // < > dookola
                if (ev == BTN_F3) st.tpos = (uint8_t)((st.tpos + 2u) % 3u);
                if (ev == BTN_F4) st.tpos = (uint8_t)((st.tpos + 1u) % 3u);
            } else if (set_sel == 1) {
                if (ev == BTN_F3 || ev == BTN_F4) st.hkeep = !st.hkeep;
            } else if (set_sel == 2) {
                if (ev == BTN_F3) st.delay_idx = (uint8_t)((st.delay_idx + N_DLY - 1u) % N_DLY);
                if (ev == BTN_F4) st.delay_idx = (uint8_t)((st.delay_idx + 1u) % N_DLY);
            } else if (set_sel == 3) {
                if (ev == BTN_F3) st.meas_panel = (uint8_t)((st.meas_panel + 2u) % 3u);
                if (ev == BTN_F4) st.meas_panel = (uint8_t)((st.meas_panel + 1u) % 3u);
                layout = 0xFF;                               // po návrate plocha panelu podľa voľby
            } else if (set_sel == 4) {                       // v0.76 Interp
                if (ev == BTN_F3) sx.interp = (uint8_t)((sx.interp + 2u) % 3u);
                if (ev == BTN_F4) sx.interp = (uint8_t)((sx.interp + 1u) % 3u);
            } else if (set_sel == 5) {                       // v0.76 Peak det
                if (ev == BTN_F3 || ev == BTN_F4) sx.peak = !sx.peak;
            } else if (set_sel == 6) {                       // v0.76 View
                if (ev == BTN_F3 || ev == BTN_F4) sx.view = (sx.view == VIEW_FFT) ? VIEW_WAVE : VIEW_FFT;
            } else if (set_sel == 7) {
                if (ev == BTN_F4) { OpenCal(); break; }
            } else if (ev == BTN_F4) {
                menu = M_MAIN;                               // po návrate z testu hlavné menu
                return SCOPE_TEST;
            }
            DrawSetup();
            break;
    }
    return SCOPE_STAY;
}

// ---------------------------------------------------------------------------
void Scope_Open(bool mv_pos) {
    mv = mv_pos;
    menu = M_MAIN;
    SetRun(true);
    auto_pending = adj_pending = lvl_pending = false;
    key_pending = BTN_NONE;
    save_pending = false;
    title_last[0] = '\0';
    rolling = false;
    frame_valid = false;
    wave_open = 0;
    memset(last_cells, 0, sizeof(last_cells));
    hw_combo = hw_ac = hw_mv = 0xFF;

    // Hodiny ADC na PC12 ako TIM11_CH1 (AF3), ADC beží
    GPIOC->AFR[1] = (GPIOC->AFR[1] & ~(0xFu << ((PIN_ADCCLK - 8u) * 4u))) | (3u << ((PIN_ADCCLK - 8u) * 4u));
    GPIOC->MODER  = (GPIOC->MODER & ~(3u << (PIN_ADCCLK * 2u))) | (2u << (PIN_ADCCLK * 2u));
    GPIOC->OSPEEDR |= (3u << (PIN_ADCCLK * 2u));
    PinSet(GPIOA, PIN_STBY, false);

    LCD_Clear(LCD_BLACK);
    DrawFrame();
    LCD_DrawMenuFrame();
    DrawGrid();
    ClampLevel();
    ApplyHw();
    DrawCells(HAL_GetTick());
}

void Scope_Close(void) {
    CaptureStop();
    // PC12 späť ako výstup v 0, relé a prepínače do 0, ADC do STBY
    GPIOC->BSRR  = 1u << (PIN_ADCCLK + 16u);
    GPIOC->MODER = (GPIOC->MODER & ~(3u << (PIN_ADCCLK * 2u))) | (1u << (PIN_ADCCLK * 2u));
    PinSet(GPIOA, PIN_REL_10, false);
    PinSet(GPIOA, PIN_O1O2, false);
    PinSet(GPIOC, PIN_U6, false);
    PinSet(GPIOC, PIN_U7, false);
    PinSet(GPIOA, PIN_REL_ACDC, false);
    PinSet(GPIOA, PIN_STBY, true);
    hw_combo = hw_ac = hw_mv = 0xFF;
    menu = M_MAIN;
    rolling = false;
    roll_pk = false;
    WaveRestoreLive();                   // v0.74: odchod z uloženého priebehu = živé nastavenie
}

// Hotové zachytenie: hľadanie hrany, pri neúspechu ďalšie zachytenia (najviac HUNT_MS), potom kreslenie
static void Process(uint32_t now) {
    uint32_t t;
    bool found = FindTrigger(&t);
    if (!found && cap_ms < 10u) {
        uint32_t t0 = HAL_GetTick();
        while (!found && HAL_GetTick() - t0 < HUNT_MS) {
            CaptureStart();
            while (!CaptureDone()) {
                if (HAL_GetTick() - t0 > HUNT_MS + 20u) { CaptureStop(); return; }
            }
            found = FindTrigger(&t);
        }
        now = HAL_GetTick();
    }
    if (found) {
        misses = 0;
        trig_seen = true;
        last_trig_tick = now;
        auto_ref = now;
        int64_t start = (int64_t)t + cap_off;
        frame_anchor = (int64_t)t;
        DrawTrace(start);
        Measure(start);
        frame_start = start;
        frame_valid = true;
        if (st.mode == 2) {                               // SINGLE: zastaviť
            SetRun(false);
            trig_seen = true;
        }
    } else {
        if (misses < 255) misses++;
        // AUTO: bez hrany dlhšie ako AUTO_FREE_MS (pomalé základne hneď) -> nespustený obraz
        if (st.mode == 0 && (now - auto_ref >= AUTO_FREE_MS || cap_ms >= 100u)) {
            int64_t start = ((int64_t)cap_lo << 16) + cap_off;
            int64_t smax = ((int64_t)cap_n - cap_s) << 16;
            if (start > smax) start = smax;
            if (start < 0) start = 0;
            DrawTrace(start);
            Measure(start);
            frame_start = start;
            frame_anchor = start - cap_off;
            frame_valid = true;
        }
    }
}

ScopeAction_t Scope_Task(ButtonType_t ev, bool mv_pos) {
    uint32_t now = HAL_GetTick();

    if (mv_pos != mv) {                       // prepínač V <-> mV: iná sada rozsahov
        mv = mv_pos;
        if (wave_open) SetRun(true);          // v0.73: uložený priebeh patrí k inej polohe -> živý obraz
        ClampLevel();
        Restart();
        if (!PanelMenu()) DrawGrid();
    }

    ScopeAction_t a = Buttons(ev, now);
    if (a != SCOPE_STAY) return a;

    if (PanelMenu()) {                        // v menu sa nezachytáva
        if (menu == M_CAL) CalDraw();         // napätie z meracieho čipu
        DrawCells(now);
        return SCOPE_STAY;
    }

    if (MarkX() != grid_mark_x) DrawGrid();   // značka bodu spustenia (posun, oneskorenie, základňa)
    ApplyLayout();
    ApplyHw();
    bool settled = (int32_t)(now - settle_until) >= 0;
    bool want_roll = RollWanted() && !wave_open;   // v0.73: uložený priebeh z ROLL sa zobrazí ako STOP
    if (want_roll != rolling) {
        if (want_roll) RollBegin(); else RollEnd();
        DrawGrid();
        ApplyLayout();
    }
    if (rolling) {
        if (settled) RollTask(now);
    } else if (run) {
        if (!capturing && settled) {
            CaptureStart();
        } else if (capturing && CaptureDone()) {
            Process(now);
            now = HAL_GetTick();
            if (run && (int32_t)(now - settle_until) >= 0) CaptureStart();
        }
    } else if (capturing) {
        CaptureStop();
    }
    DrawMarkers();
    DrawMeas(now);
    PanelRefresh(now);
    DrawCells(now);
    return SCOPE_STAY;
}

static void FmtMv(char *b, size_t n, int32_t v) {
    int32_t a = v < 0 ? -v : v;
    const char *sg = v < 0 ? "-" : "";
    if (a < 0 || a > 999999) a = 999999;              // v0.68: obmedzenie pre buffer (reálne max. stovky V)
    if (a >= 1000) snprintf(b, n, "%s%ld.%02ldV", sg, (long)(a / 1000), (long)((a % 1000) / 10));
    else           snprintf(b, n, "%s%ldmV", sg, (long)a);
}

const char *Scope_TopText(void) {
    const char *edge = st.slope ? CH_FALL : CH_RISE;
    if (menu == M_SETUP) return "SCOPE SETTINGS";
    if (menu == M_MEAS)  return "MEASUREMENTS";
    if (menu == M_WAVE)  return "WAVE MEMORY";
    if (menu == M_CAL)   return "SCOPE CAL";
    if (menu == M_TRIG) {
        char lv[12];
        FmtMv(lv, sizeof(lv), st.trig_mv);
        snprintf(top_text, sizeof(top_text), "%s %s %s", MODE_TXT[st.mode % 3], edge, lv);
    } else {
        snprintf(top_text, sizeof(top_text), "%s %s %s %s", VdivLabel(), TB[st.tb % N_TB].label,
                 st.ac ? "AC" : "DC", edge);
    }
    return top_text;
}

// Horný riadok farebne: časti textu s vlastnou farbou, v strede medzi časovačom (do x 62) a batériou (od x 284)
// Horný riadok: hodnoty v pevných poliach (vycentrované v poli), aby sa pri zmene nič neposúvalo.
// Priestor medzi časomierou (končí na x 64) a batériou (od x 279).
typedef struct { const char *t; uint16_t c; int16_t x; int16_t w; } TitleField_t;   // w = šírka poľa v px

static void TitleCentered(TitleField_t *f, const char *t, uint16_t c, int16_t x, int16_t w) {
    f->t = t; f->c = c; f->x = x; f->w = w;
}

const char *Scope_WideMsg(uint16_t *col) {
    if ((int32_t)(msg_until - HAL_GetTick()) <= 0) return NULL;
    if (strlen(msg_text) <= TOP_TXT_MAX) return NULL;
    if (col) *col = msg_col;
    return msg_text;
}

void Scope_DrawTitle(uint16_t y, bool force) {
    TitleField_t f[4];
    uint8_t      n = 0;
    char         lv[12];
    const char  *edge = st.slope ? CH_FALL : CH_RISE;
    uint16_t     ecol = st.slope ? COL_FALL : COL_RISE;
    bool msg = (int32_t)(msg_until - HAL_GetTick()) > 0;
    bool free_text = false;

    if (msg) {
        TitleCentered(&f[n++], msg_text, msg_col, TOP_FREE_X0, TOP_FREE_X1 - TOP_FREE_X0); free_text = true;
    } else if (PanelMenu()) {
        TitleCentered(&f[n++], (menu == M_SETUP) ? "SCOPE SETTINGS" : (menu == M_MEAS) ? "MEASUREMENTS" :
                      (menu == M_WAVE) ? "WAVE MEMORY" : "SCOPE CAL", LCD_CYAN, TOP_FREE_X0, TOP_FREE_X1 - TOP_FREE_X0);
        free_text = true;
    } else if (menu == M_TRIG) {
        // [režim, pole ako "SINGLE"] [úroveň, pole 7 znakov] [hrana]: x 79-148, 161-242, 255-264
        FmtMv(lv, sizeof(lv), st.trig_mv);
        TitleCentered(&f[n++], MODE_TXT[st.mode % 3], MODE_COL[st.mode % 3], 79, 70);
        TitleCentered(&f[n++], lv, LCD_ORANGE, 161, 82);
        TitleCentered(&f[n++], edge, ecol, 255, 10);
    } else {
        // [V/div] [čas/div] [AC/DC] [hrana D]: x 76-133, 145-202, 214-235, 247-268 (medzery 12 px)
        TitleCentered(&f[n++], VdivLabel(), LCD_CYAN, 76, 58);
        TitleCentered(&f[n++], TB[st.tb % N_TB].label, MODE_COL[st.mode % 3], 145, 58);   // farba = režim triggera
        TitleCentered(&f[n++], st.ac ? "AC" : "DC", LCD_CYAN, 214, 22);
        TitleCentered(&f[n++], edge, ecol, 247, 10);
    }

    char key[48];
    int  p = 0;
    for (uint8_t i = 0; i < n; i++) {
        p += snprintf(key + p, sizeof(key) - (size_t)p, "%s|%04X|", f[i].t, f[i].c);
        if (p >= (int)sizeof(key)) p = (int)sizeof(key) - 1;
    }
    bool show_d = !free_text && menu != M_TRIG && st.delay_idx;
    if (p < (int)sizeof(key) - 2) { key[p++] = show_d ? 'D' : '-'; key[p] = '\0'; }
    if (!force && strcmp(key, title_last) == 0) return;
    snprintf(title_last, sizeof(title_last), "%s", key);

    LCD_FillRect(65, 5, 214, 18, LCD_BLACK);
    for (uint8_t i = 0; i < n; i++) {
        int len = (int)strlen(f[i].t);
        int maxc = (f[i].w + 2) / 12;
        if (len > maxc) len = maxc;
        if (len > TOP_TXT_MAX) len = TOP_TXT_MAX;        // v0.66: dlhšie texty idú cez celý riadok (main.c)
        int w = len * 12 - 2;
        char b[20];
        snprintf(b, sizeof(b), "%.*s", len, f[i].t);
        LCD_DrawString((uint16_t)(f[i].x + (f[i].w - w) / 2), y, b, f[i].c, LCD_BLACK, 2);
    }
    if (show_d) LCD_DrawString(259, y, "D", LCD_ORANGE, LCD_BLACK, 2);   // Trig delay zapnuté
}

void Scope_GetState(ScopeState_t *s) { LiveState(s); }   // v0.74: nikdy nastavenie uloženého priebehu

void Scope_SetState(const ScopeState_t *s) {
    st = *s;
    if (st.vdiv_v >= N_VR_V)   st.vdiv_v = 1;
    if (st.vdiv_mv >= N_VR_MV) st.vdiv_mv = 1;
    if (st.tb >= N_TB)         st.tb = TB_DEFAULT;
    if (st.ac > 1)             st.ac = 0;
    if (st.offset > 20 || st.offset < -20) st.offset = 0;
    if (st.slope > 1)          st.slope = 0;
    if (st.mode > 2)           st.mode = 0;
    if (st.tpos > 2)           st.tpos = 1;
    if (st.trig_mv > 1000000 || st.trig_mv < -1000000) st.trig_mv = 0;
    st.meas_mask &= (uint16_t)((1u << MEAS_COUNT) - 1u);
    if (MeasCount(st.meas_mask) > MEAS_MAX_SEL) st.meas_mask = MEAS_DEFAULT;
    if (st.meas_show > 1) st.meas_show = 1;
    if (st.meas_all > 1)  st.meas_all = 0;
    if (st.roll > 1)      st.roll = 0;
    if (st.hpos > HPOS_MAX || st.hpos < -HPOS_MAX) st.hpos = 0;
    if (st.hkeep > 1)     st.hkeep = 0;
    if (st.delay_idx >= N_DLY) st.delay_idx = 0;
    if (st.meas_panel > 2)  st.meas_panel = PANEL_TRANSP;
}

// v0.76: Interp / Peak det / View (zobudenie zo spánku: Resume_t v main.c)
void Scope_GetExt(ScopeExt_t *x) { *x = sx; }

void Scope_SetExt(const ScopeExt_t *x) {
    sx = *x;
    if (sx.interp > INTERP_DOTS) sx.interp = INTERP_SINC;
    if (sx.peak > 1)             sx.peak = 0;
    if (sx.view > VIEW_FFT)      sx.view = VIEW_WAVE;
    sx.rsv = 0;
}
