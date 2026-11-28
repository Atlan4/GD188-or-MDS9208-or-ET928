/*
 * gen_menu.c
 *
 *  Obrazovka generátora:
 *
 *    hlavné ovládanie:  F1 WAVE   F2 FREQ   F3 DUTY   F4 VOLT   -> otvorí zoznam hodnôt
 *    v zozname:         F1 EXIT   F2 dole   F3 hore             (výber platí hneď)
 *    AUTO:              výstup ON / OFF
 *    SAVE krátko:       menu MEMORY – 10 pamätí + pevná položka 11
 *    SAVE dlho (1 s):   uloží aktuálne nastavenie do EEPROM ako "posledné nastavenie"
 *                       (načíta sa pri štarte prístroja), správa SAVED na 1 s
 *    F1 krátko (pri pustení): zoznam WAVE, F1 dlho (1 s): nastavenia generátora (SETTINGS)
 *    dlhé BL:           odchod z generátora (rieši main.c – na voltmeter alebo späť do osciloskopu)
 *
 *  Nastavenia generátora (SETTINGS): F1 EXIT (uloží), F2 NEXT, F3/F4 OFF/ON alebo - / +
 *    Auto output ON: pri otvorení generátora sa výstup zapne sám
 *    ON at change:   pri zmene parametra (aj LOAD z pamäte) sa výstup zapne sám
 *    CAL:            korekcia amplitúdy -10.0 .. +10.0 %. Kým je vybraná, beží kalibračný signál
 *                    SINE 100 Hz 3.0 Vpp (1.061 V AC). Pri prepojení OUT gen so vstupom V ukazuje
 *                    nameranú hodnotu; AUTO vypočíta korekciu sama.
 *
 *  Menu MEMORY (pamäť 1 celkom dole, čísla rastú nahor, pevná 11 hore):
 *    F1 EXIT   F2 DOWN   F3 UP (výber dokola)   F4 LOAD (nastaví generátor a vráti sa späť)
 *    SAVE krátko: uloží aktuálne nastavenie generátora do vybranej pamäte (hneď, bez potvrdenia)
 *    LOAD mení iba RAM – do EEPROM ako posledné nastavenie až po dlhom SAVE na obrazovke generátora.
 *    Kým je MEMORY alebo SETTINGS otvorené, odpočet vypnutia stojí (main.c, GenMenu_IsMenuOpen).
 *    Po 2 min bez tlačidla sa zavrie ako EXIT a odpočet generátora začne od začiatku.
 *
 *  Obe obrazovky majú biely rámik okolo displeja a čiaru pod horným riadkom (ako menu nastavení).
 *
 *  Rozloženie hlavnej obrazovky: vľavo parametre, vpravo náhľad tvaru vlny (zoznam hodnôt sa otvára
 *  na jeho mieste), pod parametrami stav výstupu a nápoveda (na jej mieste sa ukazuje správa).
 *  Horný riadok (čas, nadpis, batéria) kreslí main.c – táto obrazovka sa ho nedotýka (y < 32).
 */

#include <stdio.h>
#include <string.h>

#include "gen_menu.h"
#include "gen.h"
#include "gen_preset.h"
#include "settings.h"
#include "lcd.h"
#include "beep.h"

// --- Rozloženie ---
#define AREA_Y       29                     // pracovná plocha pod horným riadkom
#define AREA_H       (212 - AREA_Y)         // po spodné menu (y = 212)
#define AREA_X       1                      // vnútri rámika (x = 0 a 319)
#define AREA_W       318
#define TOP_LINE_Y   27                     // čiara pod horným riadkom

#define PAR_X        8
#define PAR_VAL_X    (PAR_X + 5 * 12)       // hodnota za "WAVE "
#define PAR_Y0       41
#define PAR_STEP     26
#define PAR_VAL_LEN  8                      // znakov hodnoty (vrátane medzier)

#define OUT_Y        152
#define HINT_Y       175                    // malé písmo, krok 12 px: vľavo 2 riadky, vpravo 3 (Hold …)
#define HINT_STEP    12
#define HINT_R_X     176                    // pravý stĺpec poznámok (pod náhľadom vlny)
#define MSG_Y        172                    // správa (SAVED ...) na mieste poznámok, v strede
#define MSG_H        37
#define MSG_TXT_Y    182

#define PV_X         176                    // náhľad vlny (rámik)
#define PV_Y         37
#define PV_W         136
#define PV_H         96

#define POP_X        176                    // zoznam hodnôt (na mieste náhľadu)
#define POP_Y        33
#define POP_W        136
#define POP_H        164
#define POP_ROWS     7
#define POP_ROW_Y    39
#define POP_ROW_H    22

// Menu MEMORY
#define MEM_ROWS     6
#define MEM_ROW_Y    35
#define MEM_ROW_H    24
#define MEM_TXT_X    8
#define MEM_INFO_Y   184                    // nápoveda / správa pod zoznamom
#define MEM_INFO_H   25
#define MEM_IDLE_MS  (2u * 60u * 1000u)     // MEMORY / SETTINGS bez tlačidla -> zavrieť ako EXIT

// Nastavenia generátora
#define SET_ITEMS    3
#define SET_ROW_Y    42
#define SET_ROW_H    24
#define SET_TXT_X    22
#define SET_MEAS_Y   121                    // nameraná / očakávaná hodnota pri CAL
#define SET_SEP_Y    168                    // čiara pod riadkom Target (ten končí na y 159)
#define SET_HINT_Y   174                    // v0.77: 3 riadky v strede medzi čiarou a spodným menu (169..209)
#define SET_HINT_STEP 12
#define SET_HINT_LINES 3
#define SET_MSG_Y    181                    // správa (CAL DONE …) na mieste nápovedy, v strede
#define CAL_TARGET   7071                   // v0.77: 2.0 Vpp sínus = 0.7071 V AC (v 0.1 mV)
#define CAL_VOLT_IDX 8u                     // v0.77: 2.0 Vpp (rezerva DAC pre korekciu +-10 %)
#define CAL_SETTLE_MS 2000u                 // v0.77: po štarte / zmene korekcie sa meranie ignoruje
#define CAL_AVG_N    4u                     // v0.77: priemer zo 4 vzoriek čipu
#define CAL_MIN_READ 2000                   // pod 0.2 V = signál nie je pripojený

#define REPEAT_DELAY_MS   400
#define REPEAT_PERIOD_MS  150
#define MSG_TIME_MS       1000              // ako dlho svieti správa

typedef enum { PAR_WAVE = 0, PAR_FREQ, PAR_DUTY, PAR_VOLT, PAR_NONE = 0xFF } Param_t;
typedef enum { SCR_MAIN = 0, SCR_MEMORY, SCR_SETUP } Screen_t;

static Screen_t     screen = SCR_MAIN;
static Param_t      popup = PAR_NONE;       // práve otvorený zoznam
static ButtonType_t rep_btn = BTN_NONE;
static uint32_t     rep_tick;

// SAVE: krátke / dlhé stlačenie sa rozlíši podľa času držania
static bool         save_pending = false;
static uint32_t     save_tick;

// Správa na 1 s (SAVED, SAVE ERROR ...)
static bool         msg_active = false;
static uint32_t     msg_until;
static char         msg_text[24];
static uint16_t     msg_color;

// Posledné stlačenie / držanie tlačidla v MEMORY / SETTINGS (zatvorenie po nečinnosti)
static uint32_t     mem_input_tick;

// Generátor otvorený z osciloskopu (dlhé BL sa vráti do osciloskopu) – len pre nápovedu
static bool         from_scope = false;

// F1 na hlavnej obrazovke: krátke = WAVE, dlhé = SETTINGS
static bool         f1_pending = false;
static uint32_t     f1_tick;

// SETTINGS
static uint8_t      set_sel = 0;
static bool         set_auto_on, set_on_change;
static int16_t      set_cal;
static bool         cal_running = false;     // kalibračný signál beží (vybraná položka CAL)
static GenParams_t  cal_saved;               // nastavenie generátora pred kalibráciou
static bool         cal_valid = false;       // meranie AC V z čipu (main.c)
static int32_t      cal_read = 0;            // 0.1 mV (priemer CAL_AVG_N vzoriek)
static uint32_t     cal_settle_tick;         // v0.77: do tohto času sa vzorky ignorujú
static int64_t      cal_sum = 0;             // v0.77: súčet vzoriek pre priemer
static uint8_t      cal_cnt = 0;
static char         last_set[SET_ITEMS][48];
static char         last_meas[2][32];
static int8_t       last_hint = -1;

// Menu MEMORY – vybraná položka sa pamätá aj po zatvorení (do vypnutia).
// mem_sel = index pamäte (0 = pamäť 1), mem_top = prvý viditeľný riadok (0 = horný riadok zoznamu).
// Zoznam je obrátený: riadok r zodpovedá pamäti GEN_PRESET_COUNT - 1 - r (pamäť 1 dole).
static uint8_t      mem_sel = 0;
static uint8_t      mem_top = 0;
#define MEM_ROW_OF(idx)  ((uint8_t)(GEN_PRESET_COUNT - 1u - (idx)))   // index <-> riadok (symetrické)

// Cache prekresľovania
static char     last_par[4][24];
static int8_t   last_out = -1;
static char     last_pv[16];
static char     last_cells[4][8];
static char     last_pop[POP_ROWS][16];
static uint8_t  last_pop_top = 0xFF;
static char     last_mem[MEM_ROWS][40];
static uint8_t  last_mem_top = 0xFF;

// ---------------------------------------------------------------------------
static uint8_t ParCount(Param_t p) {
    switch (p) {
        case PAR_WAVE: return GEN_WAVE_COUNT;
        case PAR_FREQ: return Gen_FreqCount(g_gen.wave);
        case PAR_DUTY: return GEN_NUM_DUTY;
        case PAR_VOLT: return GEN_NUM_VOLT;
        default:       return 0;
    }
}

static uint8_t *ParIdx(Param_t p) {
    switch (p) {
        case PAR_WAVE: return &g_gen.wave;
        case PAR_FREQ: return &g_gen.freq_idx;
        case PAR_DUTY: return &g_gen.duty_idx;
        default:       return &g_gen.volt_idx;
    }
}

// Číselné zoznamy sa zobrazujú od najväčšej hodnoty hore (UP = vyššia hodnota),
// tvar vlny v prirodzenom poradí. Poloha = riadok v zozname (0 = hore).
static bool ParReversed(Param_t p) { return p != PAR_WAVE; }
static uint8_t PosToIdx(Param_t p, uint8_t pos) {
    return ParReversed(p) ? (uint8_t)(ParCount(p) - 1u - pos) : pos;
}
#define IdxToPos PosToIdx      // zobrazenie je symetrické

// Text hodnoty pre položku idx daného parametra
static void ValueText(Param_t p, uint8_t idx, char *buf, size_t n) {
    switch (p) {
        case PAR_WAVE: snprintf(buf, n, "%s", Gen_WaveName(idx)); break;
        case PAR_FREQ: Gen_FormatFreq(Gen_FreqHz(idx), buf, n); break;
        case PAR_DUTY: snprintf(buf, n, "%u %%", Gen_DutyPct(idx)); break;
        default:       Gen_FormatVolt(Gen_VoltMv(idx), buf, n); break;
    }
}

static void DrawCell(uint8_t idx, const char *txt, uint16_t fg, uint16_t bg) {
    char key[8];
    snprintf(key, sizeof(key), "%.5s%c", txt, (fg == LCD_WHITE) ? 'w' : (fg == LCD_GREY) ? 'g' : 'c');
    if (strcmp(key, last_cells[idx]) == 0) return;
    strcpy(last_cells[idx], key);
    LCD_DrawMenuCell(idx, txt, fg, bg);
}

// ---------------------------------------------------------------------------
// Parametre vľavo
// ---------------------------------------------------------------------------
static void DrawParams(void) {
    static const char *const labels[4] = { "WAVE", "FREQ", "DUTY", "VOLT" };
    for (uint8_t p = 0; p < 4; p++) {
        char val[16], line[16], key[24];
        bool dimmed = (p == PAR_DUTY && !Gen_DutyUsed(g_gen.wave));
        if (dimmed) strcpy(val, "--");
        else        ValueText((Param_t)p, *ParIdx((Param_t)p), val, sizeof(val));
        snprintf(line, sizeof(line), "%-*.*s", PAR_VAL_LEN, PAR_VAL_LEN, val);

        bool edit = (popup == (Param_t)p);
        snprintf(key, sizeof(key), "%c%c%s", edit ? 'E' : '-', dimmed ? 'D' : '-', line);
        if (strcmp(key, last_par[p]) == 0) continue;
        strcpy(last_par[p], key);

        uint16_t y = PAR_Y0 + p * PAR_STEP;
        LCD_DrawString(PAR_X, y, labels[p], dimmed ? LCD_GREY : LCD_WHITE, LCD_BLACK, 2);
        LCD_DrawString(PAR_VAL_X, y, line, dimmed ? LCD_GREY : LCD_YELLOW, LCD_BLACK, 2);

        // Rámik okolo hodnoty, ktorá sa práve nastavuje (inak rámik zmazať)
        uint16_t fc = edit ? LCD_SEL_BG : LCD_BLACK;
        const uint16_t fx = PAR_VAL_X - 4, fy = y - 4;
        const uint16_t fw = PAR_VAL_LEN * 12 + 6, fh = 16 + 7;
        LCD_DrawFastHLine(fx, fy, fw, fc);
        LCD_DrawFastHLine(fx, fy + fh - 1, fw, fc);
        LCD_DrawFastVLine(fx, fy, fh, fc);
        LCD_DrawFastVLine(fx + fw - 1, fy, fh, fc);
    }
}

// ---------------------------------------------------------------------------
// Stav výstupu a nápoveda
// ---------------------------------------------------------------------------
static void DrawOutput(void) {
    int8_t on = g_gen.on ? 1 : 0;
    if (on == last_out) return;
    last_out = on;
    LCD_FillRect(PAR_X, OUT_Y, 160, 16, LCD_BLACK);
    LCD_DrawString(PAR_X, OUT_Y, "OUTPUT", LCD_WHITE, LCD_BLACK, 2);
    if (on) LCD_DrawString(PAR_X + 7 * 12, OUT_Y, "ON",  LCD_GREEN, LCD_BLACK, 2);
    else    LCD_DrawString(PAR_X + 7 * 12, OUT_Y, "OFF", LCD_RED,   LCD_BLACK, 2);
}

// Poznámky dole: vľavo AUTO / SAVE, vpravo pod sebou Hold BL / Hold SAVE / Hold F1.
// Pravý stĺpec je pod zoznamom hodnôt -> kým je zoznam otvorený, nekreslí sa (F1 je tam EXIT).
static void DrawHintsRight(void) {
    LCD_DrawString(HINT_R_X, HINT_Y, from_scope ? "Hold BL   = to scope" : "Hold BL   = exit",
                   LCD_LIGHTGREY, LCD_BLACK, 1);
    LCD_DrawString(HINT_R_X, HINT_Y + HINT_STEP,     "Hold SAVE = save setup", LCD_LIGHTGREY, LCD_BLACK, 1);
    LCD_DrawString(HINT_R_X, HINT_Y + 2 * HINT_STEP, "Hold F1   = settings",   LCD_LIGHTGREY, LCD_BLACK, 1);
}

static void DrawHints(void) {
    if (popup == PAR_NONE) LCD_FillRect(AREA_X + 1, MSG_Y, AREA_W - 2, MSG_H, LCD_BLACK);
    else                   LCD_FillRect(AREA_X + 1, MSG_Y, POP_X - AREA_X - 2, MSG_H, LCD_BLACK);
    LCD_DrawString(PAR_X, HINT_Y,             "AUTO = output ON/OFF", LCD_LIGHTGREY, LCD_BLACK, 1);
    LCD_DrawString(PAR_X, HINT_Y + HINT_STEP, "SAVE = memory",        LCD_LIGHTGREY, LCD_BLACK, 1);
    if (popup == PAR_NONE) DrawHintsRight();
}

// Nápoveda pod zoznamom v menu MEMORY
static void DrawMemInfo(void) {
    LCD_FillRect(AREA_X + 1, MEM_INFO_Y, AREA_W - 2, MEM_INFO_H, LCD_BLACK);
    LCD_DrawString(MEM_TXT_X, MEM_INFO_Y + 4, "SAVE = store current settings to the slot",
                   LCD_LIGHTGREY, LCD_BLACK, 1);
    LCD_DrawString(MEM_TXT_X, MEM_INFO_Y + 14, "LOAD = set generator from the slot",
                   LCD_LIGHTGREY, LCD_BLACK, 1);
}

// ---------------------------------------------------------------------------
// Správa na 1 s (na mieste nápovedy aktuálnej obrazovky, vodorovne v strede)
// Na hlavnej obrazovke pri otvorenom zozname hodnôt v strede ľavej časti (vpravo je zoznam).
// ---------------------------------------------------------------------------
static void MsgDraw(void) {
    uint16_t w = (uint16_t)(strlen(msg_text) * 12u - 2u);
    if (screen == SCR_MAIN) {
        uint16_t x0 = AREA_X + 1;
        uint16_t aw = (popup == PAR_NONE) ? (uint16_t)(AREA_W - 2) : (uint16_t)(POP_X - AREA_X - 2);
        LCD_FillRect(x0, MSG_Y, aw, MSG_H, LCD_BLACK);
        uint16_t x = (w < aw) ? (uint16_t)(x0 + (aw - w) / 2u) : x0;
        LCD_DrawString(x, MSG_TXT_Y, msg_text, msg_color, LCD_BLACK, 2);
    } else if (screen == SCR_MEMORY) {
        LCD_FillRect(AREA_X + 1, MEM_INFO_Y, AREA_W - 2, MEM_INFO_H, LCD_BLACK);
        uint16_t x = (w < 320u) ? (uint16_t)((320u - w) / 2u) : 0;
        LCD_DrawString(x, MEM_INFO_Y + 5, msg_text, msg_color, LCD_BLACK, 2);
    } else {                                   // SETTINGS: na mieste nápovedy pod čiarou
        LCD_FillRect(AREA_X + 1, SET_SEP_Y + 1, AREA_W - 2, 210 - SET_SEP_Y - 1, LCD_BLACK);
        uint16_t x = (w < 320u) ? (uint16_t)((320u - w) / 2u) : 0;
        LCD_DrawString(x, SET_MSG_Y, msg_text, msg_color, LCD_BLACK, 2);
    }
}

static void MsgShow(const char *txt, uint16_t color) {
    if (color == LCD_RED) Beep_Error();        // chyba: dlhé pípnutie (podľa nastavenia Beep)
    strncpy(msg_text, txt, sizeof(msg_text) - 1);
    msg_text[sizeof(msg_text) - 1] = '\0';
    msg_color  = color;
    msg_active = true;
    msg_until  = HAL_GetTick() + MSG_TIME_MS;
    MsgDraw();
}

static void MsgTask(void) {
    if (!msg_active || (int32_t)(HAL_GetTick() - msg_until) < 0) return;
    msg_active = false;
    if (screen == SCR_MAIN)        DrawHints();
    else if (screen == SCR_MEMORY) DrawMemInfo();
    else last_hint = -1;                       // SETTINGS: nápoveda sa znova nakreslí v DrawSetup
}

// ---------------------------------------------------------------------------
// Náhľad tvaru vlny (2 periódy). Výška podľa amplitúdy, sivá pri vypnutom výstupe.
// ---------------------------------------------------------------------------
static void DrawPreview(bool force) {
    if (popup != PAR_NONE) return;
    char key[16];
    snprintf(key, sizeof(key), "%u%u%u%u", g_gen.wave, g_gen.duty_idx, g_gen.volt_idx, g_gen.on ? 1 : 0);
    if (!force && strcmp(key, last_pv) == 0) return;
    strcpy(last_pv, key);

    const uint16_t ix = PV_X + 2, iy = PV_Y + 2;
    const uint16_t iw = PV_W - 4, ih = PV_H - 4;
    const int16_t  mid = (int16_t)(iy + ih / 2);

    LCD_FillRect(PV_X, PV_Y, PV_W, PV_H, LCD_BLACK);
    LCD_DrawFastHLine(PV_X, PV_Y, PV_W, LCD_GREY);
    LCD_DrawFastHLine(PV_X, PV_Y + PV_H - 1, PV_W, LCD_GREY);
    LCD_DrawFastVLine(PV_X, PV_Y, PV_H, LCD_GREY);
    LCD_DrawFastVLine(PV_X + PV_W - 1, PV_Y, PV_H, LCD_GREY);
    for (uint16_t x = ix; x < ix + iw; x += 4) LCD_DrawPixel(x, (uint16_t)mid, LCD_GREY);  // bodkovaná nula

    uint16_t mv  = Gen_VoltMv(g_gen.volt_idx);
    int32_t  amp = (int32_t)((ih / 2 - 2) * (uint32_t)mv / 3000u);
    if (mv > 0 && amp < 3) amp = 3;
    uint16_t color = g_gen.on ? LCD_YELLOW : LCD_LIGHTGREY;
    uint8_t  duty  = Gen_DutyPct(g_gen.duty_idx);
    uint32_t per   = iw / 2;                         // 2 periódy na šírku

    int16_t prev_y = 0;
    for (uint16_t x = 0; x < iw; x++) {
        int32_t s = Gen_Shape(g_gen.wave, duty, x % per, per);
        int16_t y = (int16_t)(mid - (s * amp) / 32767);
        if (x == 0) prev_y = y;
        // zvislá spojnica s predchádzajúcim bodom (hrany obdĺžnika, strmá rampa)
        int16_t y0 = (prev_y < y) ? prev_y : y;
        int16_t y1 = (prev_y < y) ? y : prev_y;
        LCD_DrawFastVLine(ix + x, (uint16_t)y0, (uint16_t)(y1 - y0 + 1), color);
        prev_y = y;
    }
}

// ---------------------------------------------------------------------------
// Zoznam hodnôt (7 viditeľných riadkov, posúva sa s výberom)
// ---------------------------------------------------------------------------
static void DrawPopup(bool force) {
    if (popup == PAR_NONE) return;
    uint8_t cnt = ParCount(popup);
    uint8_t sel = IdxToPos(popup, *ParIdx(popup));     // poloha vybranej položky v zozname
    uint8_t top = 0;
    if (cnt > POP_ROWS) {
        top = (sel > POP_ROWS / 2) ? (uint8_t)(sel - POP_ROWS / 2) : 0;
        if (top > cnt - POP_ROWS) top = (uint8_t)(cnt - POP_ROWS);
    }

    if (force) {
        LCD_FillRect(POP_X, POP_Y, POP_W, POP_H, LCD_BLACK);
        LCD_DrawFastHLine(POP_X, POP_Y, POP_W, LCD_WHITE);
        LCD_DrawFastHLine(POP_X, POP_Y + POP_H - 1, POP_W, LCD_WHITE);
        LCD_DrawFastVLine(POP_X, POP_Y, POP_H, LCD_WHITE);
        LCD_DrawFastVLine(POP_X + POP_W - 1, POP_Y, POP_H, LCD_WHITE);
        memset(last_pop, 0, sizeof(last_pop));
        last_pop_top = 0xFF;
    }

    for (uint8_t r = 0; r < POP_ROWS; r++) {
        uint8_t idx = (uint8_t)(top + r);              // poloha v zozname
        char txt[16], key[16];
        if (idx < cnt) ValueText(popup, PosToIdx(popup, idx), txt, sizeof(txt));
        else           txt[0] = '\0';
        snprintf(key, sizeof(key), "%c%.13s", (idx == sel) ? '>' : ' ', txt);
        if (strcmp(key, last_pop[r]) == 0) continue;
        strcpy(last_pop[r], key);

        uint16_t y = POP_ROW_Y + r * POP_ROW_H;
        uint16_t bg = (idx == sel && idx < cnt) ? LCD_SEL_BG : LCD_BLACK;
        uint16_t fg = (idx == sel && idx < cnt) ? LCD_SEL_FG : LCD_WHITE;
        LCD_FillRect(POP_X + 4, y, POP_W - 16, POP_ROW_H - 2, bg);
        if (txt[0]) LCD_DrawString(POP_X + 10, y + 3, txt, fg, bg, 2);
    }

    // Značky, že sú ďalšie položky nad / pod viditeľnými
    if (top != last_pop_top) {
        last_pop_top = top;
        LCD_DrawString(POP_X + POP_W - 10, POP_ROW_Y + 2, (top > 0) ? "^" : " ", LCD_WHITE, LCD_BLACK, 1);
        LCD_DrawString(POP_X + POP_W - 10, POP_ROW_Y + (POP_ROWS - 1) * POP_ROW_H + 10,
                       (top + POP_ROWS < cnt) ? "v" : " ", LCD_WHITE, LCD_BLACK, 1);
    }
}

static void ClosePopup(void) {
    if (popup == PAR_NONE) return;
    popup = PAR_NONE;
    rep_btn = BTN_NONE;
    LCD_FillRect(POP_X, POP_Y, POP_W, POP_H, LCD_BLACK);
    DrawPreview(true);
    if (msg_active) MsgDraw();          // správa znova v strede celej šírky
    else            DrawHintsRight();   // pravé poznámky boli pod zoznamom
}

static void OpenPopup(Param_t p) {
    if (p == PAR_DUTY && !Gen_DutyUsed(g_gen.wave)) return;   // sínus: strieda sa nepoužíva
    popup = p;
    rep_btn = BTN_NONE;
    DrawPopup(true);
    if (msg_active) MsgDraw();          // správa sa presunie do ľavej časti
}

// Posun výberu v zozname (dir +1 = o riadok nižšie), zmena platí hneď
// v0.66: wrap = true pri novom stlačení -> z konca zoznamu na začiatok a naopak.
// Pri opakovaní (držanie F2 / F3) wrap = false -> na konci zoznamu zastaví.
static void Step(int8_t dir, bool wrap) {
    uint8_t cnt = ParCount(popup);
    if (cnt == 0) return;
    uint8_t *idx = ParIdx(popup);
    uint8_t pos = IdxToPos(popup, *idx);
    if (dir > 0 && pos + 1u < cnt)  pos++;
    else if (dir < 0 && pos > 0)    pos--;
    else if (wrap && cnt > 1)       pos = (dir > 0) ? 0 : (uint8_t)(cnt - 1u);
    else return;
    *idx = PosToIdx(popup, pos);
    Gen_Update();          // pri zmene tvaru aj obmedzenie frekvencie (100 kHz iba sínus)
    if (g_settings.gen_on_change && !g_gen.on) Gen_SetOutput(true);   // "ON at change"
}

// ---------------------------------------------------------------------------
// Menu MEMORY
// ---------------------------------------------------------------------------

// Krátky zápis frekvencie bez medzery: "100Hz", "1.00kHz", "10.0kHz", "100kHz"
static void FreqShort(uint32_t hz, char *buf, size_t n) {
    if (hz < 1000u)        snprintf(buf, n, "%luHz", (unsigned long)hz);
    else if (hz < 10000u)  snprintf(buf, n, "%lu.%02lukHz", (unsigned long)(hz / 1000u), (unsigned long)((hz % 1000u) / 10u));
    else if (hz < 100000u) snprintf(buf, n, "%lu.%lukHz",   (unsigned long)(hz / 1000u), (unsigned long)((hz % 1000u) / 100u));
    else                   snprintf(buf, n, "%lukHz", (unsigned long)(hz / 1000u));
}

// Text položky, napr. " 3 SQR 1.00kHz 50% 3.0Vpp" (25 znakov = 300 px)
static void MemText(uint8_t idx, char *buf, size_t n) {
    static const char *const WAVE_SHORT[GEN_WAVE_COUNT] = { "SIN", "SQR", "RMP" };
    const GenPreset_t *p = GenPreset_Get(idx);
    if (!p->used) {
        snprintf(buf, n, "%2u EMPTY", idx + 1u);
        return;
    }
    char f[12], d[6];
    FreqShort(Gen_FreqHz(p->freq_idx), f, sizeof(f));
    if (Gen_DutyUsed(p->wave)) snprintf(d, sizeof(d), "%u%%", Gen_DutyPct(p->duty_idx));
    else                       strcpy(d, "--");
    uint16_t mv = Gen_VoltMv(p->volt_idx);
    snprintf(buf, n, "%2u %-3.3s %7.7s %-3.4s %u.%uVpp", (idx + 1u) % 100u,
             WAVE_SHORT[(p->wave < GEN_WAVE_COUNT) ? p->wave : 0], f, d, (mv / 1000u) % 10u, (mv % 1000u) / 100u);
}

// Vybraná položka vždy viditeľná (pri prechode dokola skočí zoznam na začiatok / koniec)
static void MemScroll(void) {
    uint8_t row = MEM_ROW_OF(mem_sel);
    if (row < mem_top) mem_top = row;
    if (row >= mem_top + MEM_ROWS) mem_top = (uint8_t)(row - (MEM_ROWS - 1u));
}

static void DrawMemList(void) {
    for (uint8_t r = 0; r < MEM_ROWS; r++) {
        uint8_t row = (uint8_t)(mem_top + r);
        uint8_t idx = (row < GEN_PRESET_COUNT) ? MEM_ROW_OF(row) : 0xFF;
        char txt[40], key[40];
        if (idx != 0xFF) MemText(idx, txt, sizeof(txt));
        else             txt[0] = '\0';
        bool sel = (idx == mem_sel);
        snprintf(key, sizeof(key), "%c%.29s", sel ? '>' : ' ', txt);
        if (strcmp(key, last_mem[r]) == 0) continue;
        strcpy(last_mem[r], key);

        uint16_t y  = MEM_ROW_Y + r * MEM_ROW_H;
        uint16_t bg = sel ? LCD_SEL_BG : LCD_BLACK;
        uint16_t fg = sel ? LCD_SEL_FG
                    : (idx == GEN_PRESET_FIXED) ? LCD_CYAN                 // pevná položka
                    : GenPreset_Get(idx)->used ? LCD_WHITE : LCD_GREY;     // prázdna sivá
        LCD_FillRect(4, y, 306, MEM_ROW_H - 2, bg);
        if (txt[0]) LCD_DrawString(MEM_TXT_X, y + 3, txt, fg, bg, 2);
    }

    // Značky, že sú ďalšie položky nad / pod viditeľnými
    if (mem_top != last_mem_top) {
        last_mem_top = mem_top;
        LCD_DrawString(312, MEM_ROW_Y + 2, (mem_top > 0) ? "^" : " ", LCD_WHITE, LCD_BLACK, 1);
        LCD_DrawString(312, MEM_ROW_Y + (MEM_ROWS - 1) * MEM_ROW_H + 12,
                       (mem_top + MEM_ROWS < GEN_PRESET_COUNT) ? "v" : " ", LCD_WHITE, LCD_BLACK, 1);
    }
}

// dir +1 = o riadok nižšie (nižšie číslo pamäte), -1 = vyššie; dokola
static void MemStep(int8_t dir) {
    if (dir > 0) mem_sel = (uint8_t)((mem_sel + GEN_PRESET_COUNT - 1u) % GEN_PRESET_COUNT);
    else         mem_sel = (uint8_t)((mem_sel + 1u) % GEN_PRESET_COUNT);
    MemScroll();
}

// ---------------------------------------------------------------------------
// Celé obrazovky (horný riadok y < 32 ostáva)
// ---------------------------------------------------------------------------
static void ResetCaches(void) {
    memset(last_par, 0, sizeof(last_par));
    memset(last_cells, 0, sizeof(last_cells));
    memset(last_mem, 0, sizeof(last_mem));
    last_pv[0] = '\0';
    last_out = -1;
    last_mem_top = 0xFF;
    msg_active = false;
    rep_btn = BTN_NONE;
}

// Biely rámik okolo displeja a čiara pod horným riadkom (ako menu nastavení)
static void DrawFrame(void) {
    LCD_DrawFastHLine(0, 0, 320, LCD_WHITE);
    LCD_DrawFastHLine(0, 239, 320, LCD_WHITE);
    LCD_DrawFastVLine(0, 0, 240, LCD_WHITE);
    LCD_DrawFastVLine(319, 0, 240, LCD_WHITE);
    LCD_DrawFastHLine(0, TOP_LINE_Y, 320, LCD_WHITE);
}

static void ShowMain(void) {
    screen = SCR_MAIN;
    popup  = PAR_NONE;
    ResetCaches();
    LCD_FillRect(AREA_X, AREA_Y, AREA_W, AREA_H, LCD_BLACK);
    LCD_DrawMenuFrame();
    DrawHints();
    DrawParams();
    DrawOutput();
    DrawPreview(true);
}

static void ShowMemory(void) {
    screen = SCR_MEMORY;
    popup  = PAR_NONE;
    GenPreset_ReadAll();
    ResetCaches();
    MemScroll();
    mem_input_tick = HAL_GetTick();
    LCD_FillRect(AREA_X, AREA_Y, AREA_W, AREA_H, LCD_BLACK);
    LCD_DrawMenuFrame();
    DrawMemList();
    DrawMemInfo();
}

// ---------------------------------------------------------------------------
// SETTINGS – nastavenia generátora
// ---------------------------------------------------------------------------
static const char *const SET_HINTS[SET_ITEMS][SET_HINT_LINES] = {
    { "", "ON: the output switches on by itself",    "when the generator screen opens." },
    { "", "ON: the output switches on by itself",    "when a parameter is changed or loaded." },
    { "Signal on OUT gen: SINE 100 Hz 2.0 Vpp.",     "Connect OUT gen to the V input (cable).",
      "F3 / F4 = -/+ 0.1 %, AUTO = automatic CAL." },
};

// v0.77: nové meranie po zmene signálu – počkať na ustálenie čipu a potom priemerovať
static void CalRestart(void) {
    cal_valid = false;
    cal_sum = 0;
    cal_cnt = 0;
    cal_settle_tick = HAL_GetTick() + CAL_SETTLE_MS;
}

// Kalibračný signál: SINE 100 Hz 3.0 Vpp, kým je vybraná položka CAL
static void CalStart(void) {
    if (cal_running) return;
    cal_saved = g_gen;
    g_gen.wave     = GEN_WAVE_SINE;
    g_gen.freq_idx = 0;                       // 100 Hz
    g_gen.volt_idx = CAL_VOLT_IDX;            // 2.0 Vpp
    cal_running = true;
    CalRestart();
    Gen_SetOutput(true);
}

static void CalStop(void) {
    if (!cal_running) return;
    cal_running = false;
    bool on = cal_saved.on;
    g_gen = cal_saved;
    Gen_SetOutput(on);                        // pôvodné nastavenie a stav výstupu
}

static void SetLineText(uint8_t item, char *buf, size_t n) {
    switch (item) {
        case 0:  snprintf(buf, n, "%-17s%7s", "Auto output ON:", set_auto_on   ? "ON" : "OFF"); break;
        case 1:  snprintf(buf, n, "%-17s%7s", "ON at change:",   set_on_change ? "ON" : "OFF"); break;
        default: {
            int16_t c = set_cal;
            unsigned a = (unsigned)((c < 0) ? -c : c) % 1000u;      // max. 99.9 %
            char v[12];
            snprintf(v, sizeof(v), "%c%u.%u %%", (c < 0) ? '-' : '+', a / 10u, a % 10u);
            snprintf(buf, n, "%-17s%7s", "CAL:", v);
            break;
        }
    }
}

static void DrawSetup(void) {
    for (uint8_t i = 0; i < SET_ITEMS; i++) {
        char t[40], key[48];
        SetLineText(i, t, sizeof(t));
        snprintf(key, sizeof(key), "%c%s", (i == set_sel) ? '>' : ' ', t);
        if (strcmp(key, last_set[i]) == 0) continue;
        strcpy(last_set[i], key);
        uint16_t y = SET_ROW_Y + i * SET_ROW_H;
        uint16_t c = (i == set_sel) ? LCD_YELLOW : LCD_WHITE;
        LCD_DrawString(6, y, (i == set_sel) ? ">" : " ", LCD_YELLOW, LCD_BLACK, 2);
        LCD_DrawString(SET_TXT_X, y, t, c, LCD_BLACK, 2);
    }

    // Pri CAL: nameraná a očakávaná hodnota
    char m0[32] = "", m1[32] = "";
    if (set_sel == 2) {
        if (cal_valid) snprintf(m0, sizeof(m0), "Measured: %ld.%03ld V", (long)(cal_read / 10000),
                                (long)((cal_read % 10000) / 10));
        else           snprintf(m0, sizeof(m0), "Measured: ---");
        snprintf(m1, sizeof(m1), "Target:   %ld.%03ld V AC", (long)(CAL_TARGET / 10000),
                 (long)((CAL_TARGET % 10000 + 5) / 10));
    }
    if (strcmp(m0, last_meas[0]) != 0) {
        strcpy(last_meas[0], m0);
        LCD_FillRect(AREA_X + 1, SET_MEAS_Y, AREA_W - 2, 16, LCD_BLACK);
        if (m0[0]) LCD_DrawString(SET_TXT_X, SET_MEAS_Y, m0, cal_valid ? LCD_GREEN : LCD_LIGHTGREY, LCD_BLACK, 2);
    }
    if (strcmp(m1, last_meas[1]) != 0) {
        strcpy(last_meas[1], m1);
        LCD_FillRect(AREA_X + 1, SET_MEAS_Y + 20, AREA_W - 2, 16, LCD_BLACK);
        if (m1[0]) LCD_DrawString(SET_TXT_X, SET_MEAS_Y + 20, m1, LCD_LIGHTGREY, LCD_BLACK, 2);
    }

    if (last_hint != (int8_t)set_sel && !msg_active) {     // počas správy je na jej mieste správa
        last_hint = (int8_t)set_sel;
        LCD_FillRect(AREA_X + 1, SET_SEP_Y + 1, AREA_W - 2, 210 - SET_SEP_Y - 1, LCD_BLACK);
        for (uint8_t i = 0; i < SET_HINT_LINES; i++)
            LCD_DrawString(10, (uint16_t)(SET_HINT_Y + i * SET_HINT_STEP), SET_HINTS[set_sel][i], LCD_LIGHTGREY, LCD_BLACK, 1);
    }
}

static void ShowSetup(void) {
    screen = SCR_SETUP;
    popup  = PAR_NONE;
    ResetCaches();
    set_sel       = 0;
    set_auto_on   = g_settings.gen_auto_on;
    set_on_change = g_settings.gen_on_change;
    set_cal       = Gen_GetCal();
    memset(last_set, 0, sizeof(last_set));
    memset(last_meas, 0, sizeof(last_meas));
    last_hint = -1;
    mem_input_tick = HAL_GetTick();
    LCD_FillRect(AREA_X, AREA_Y, AREA_W, AREA_H, LCD_BLACK);
    LCD_DrawFastHLine(0, SET_SEP_Y, 320, LCD_WHITE);
    LCD_DrawMenuFrame();
    DrawSetup();
}

// EXIT (aj po 2 min nečinnosti): uložiť do EEPROM (iba pri zmene), ukončiť kalibráciu
static void CloseSetup(void) {
    CalStop();
    g_settings.gen_auto_on   = set_auto_on;
    g_settings.gen_on_change = set_on_change;
    g_settings.gen_cal       = (int8_t)set_cal;
    Settings_Save();
    ShowMain();
}

static void SetAdjust(ButtonType_t b) {
    switch (set_sel) {
        case 0: set_auto_on   = (b == BTN_F4); break;
        case 1: set_on_change = (b == BTN_F4); break;
        default:
        {
            int16_t old = set_cal;
            if (b == BTN_F3 && set_cal > -SETTINGS_GEN_CAL_MAX) set_cal--;
            if (b == BTN_F4 && set_cal <  SETTINGS_GEN_CAL_MAX) set_cal++;
            Gen_SetCal(set_cal);                          // hneď platí – vidno to na meraní
            if (set_cal != old) CalRestart();             // v0.77: staré meranie už neplatí
            break;
        }
    }
}

// AUTO pri CAL: korekcia z pomeru očakávanej a nameranej hodnoty
static void CalAuto(void) {
    if (!cal_valid || cal_read < CAL_MIN_READ) {
        MsgShow("NO SIGNAL", LCD_RED);
        return;
    }
    int32_t c = (int32_t)(((int64_t)(1000 + set_cal) * CAL_TARGET + cal_read / 2) / cal_read) - 1000;
    if (c > SETTINGS_GEN_CAL_MAX || c < -SETTINGS_GEN_CAL_MAX) {
        MsgShow("CAL OUT OF RANGE", LCD_RED);
        return;
    }
    set_cal = (int16_t)c;
    Gen_SetCal(set_cal);
    CalRestart();                                         // v0.77: počkať na ustálené nové meranie
    MsgShow("CAL DONE", LCD_GREEN);
}

static void SetupButtons(ButtonType_t ev, uint32_t now) {
    switch (ev) {
        case BTN_F1:  CloseSetup(); return;
        case BTN_F2:  set_sel = (uint8_t)((set_sel + 1u) % SET_ITEMS); break;
        case BTN_F3:
        case BTN_F4:  SetAdjust(ev); rep_btn = ev; rep_tick = now + REPEAT_DELAY_MS; break;
        case BTN_AUTO: if (set_sel == 2) CalAuto(); break;
        default: break;
    }
    if (set_sel == 2) CalStart(); else CalStop();

    // Opakovanie pri držaní - / + (iba CAL)
    if (rep_btn != BTN_NONE) {
        if (BUTTON_GetHeld() != rep_btn || set_sel != 2) {
            rep_btn = BTN_NONE;
        } else if ((int32_t)(now - rep_tick) >= 0) {
            SetAdjust(rep_btn);
            rep_tick = now + REPEAT_PERIOD_MS;
        }
    }
}

// ---------------------------------------------------------------------------
// Tlačidlá
// ---------------------------------------------------------------------------

// Dlhé SAVE na hlavnej obrazovke: posledné nastavenie do EEPROM
static void SaveLastSetup(void) {
    MsgShow("SAVING...", LCD_YELLOW);
    if (Settings_GenSave()) MsgShow("SAVED", LCD_GREEN);
    else                    MsgShow("SAVE ERROR", LCD_RED);
}

// Krátke SAVE v MEMORY: aktuálne nastavenie do vybranej pamäte
static void SaveShort(void) {
    MsgShow("SAVING...", LCD_YELLOW);
    if (mem_sel == GEN_PRESET_FIXED) {
        MsgShow("FIXED - READ ONLY", LCD_RED);
    } else {
        char b[24];
        if (GenPreset_Store(mem_sel)) {
            snprintf(b, sizeof(b), "SAVED TO %u", mem_sel + 1u);
            MsgShow(b, LCD_GREEN);
            last_mem[MEM_ROW_OF(mem_sel) - mem_top][0] = '\0';   // prekresliť zmenený riadok
        } else {
            MsgShow("SAVE ERROR", LCD_RED);
        }
    }
}

// SAVE: udalosť prichádza pri stlačení, rozhodne sa podľa času držania
static void SaveButtonTask(ButtonType_t ev, uint32_t now) {
    if (ev == BTN_SAVE) {
        save_pending = true;
        save_tick = now;
        return;
    }
    if (!save_pending) return;

    if (BUTTON_GetHeld() == BTN_SAVE) {
        if (now - save_tick >= Settings_LongPressMs()) {
            save_pending = false;
            if (screen == SCR_MAIN) SaveLastSetup();   // v MEMORY / SETTINGS dlhé SAVE nerobí nič
        }
    } else {
        save_pending = false;
        if (screen == SCR_MAIN)        ShowMemory();
        else if (screen == SCR_MEMORY) SaveShort();
    }
}

static void MainButtons(ButtonType_t ev, uint32_t now) {
    // F1 bez otvoreného zoznamu: krátke (pri pustení) = WAVE, dlhé = SETTINGS
    if (f1_pending) {
        if (BUTTON_GetHeld() == BTN_F1) {
            if (now - f1_tick >= Settings_LongPressMs()) {
                f1_pending = false;
                ShowSetup();
                return;
            }
        } else {
            f1_pending = false;
            if (popup == PAR_NONE) OpenPopup(PAR_WAVE);
        }
    }

    if (ev == BTN_AUTO) {
        Gen_SetOutput(!g_gen.on);
    } else if (popup == PAR_NONE) {
        switch (ev) {
            case BTN_F1: f1_pending = true; f1_tick = now; break;
            case BTN_F2: OpenPopup(PAR_FREQ); break;
            case BTN_F3: OpenPopup(PAR_DUTY); break;
            case BTN_F4: OpenPopup(PAR_VOLT); break;
            default: break;
        }
    } else {
        switch (ev) {
            case BTN_F1: ClosePopup(); break;
            case BTN_F2: Step(+1, true); rep_btn = BTN_F2; rep_tick = now + REPEAT_DELAY_MS; break;
            case BTN_F3: Step(-1, true); rep_btn = BTN_F3; rep_tick = now + REPEAT_DELAY_MS; break;
            default: break;
        }
    }

    // Opakovanie pri držaní F2 / F3 v zozname
    if (rep_btn != BTN_NONE) {
        if (BUTTON_GetHeld() != rep_btn || popup == PAR_NONE) {
            rep_btn = BTN_NONE;
        } else if ((int32_t)(now - rep_tick) >= 0) {
            Step((rep_btn == BTN_F2) ? +1 : -1, false);
            rep_tick = now + REPEAT_PERIOD_MS;
        }
    }
}

static void MemoryButtons(ButtonType_t ev, uint32_t now) {
    switch (ev) {
        case BTN_F1:                                   // EXIT
            ShowMain();
            return;
        case BTN_F2: MemStep(+1); rep_btn = BTN_F2; rep_tick = now + REPEAT_DELAY_MS; break;
        case BTN_F3: MemStep(-1); rep_btn = BTN_F3; rep_tick = now + REPEAT_DELAY_MS; break;
        case BTN_F4:                                   // LOAD – iba RAM, späť na generátor
            if (GenPreset_Apply(mem_sel)) {
                if (g_settings.gen_on_change && !g_gen.on) Gen_SetOutput(true);   // "ON at change"
                ShowMain();
                return;
            }
            break;                                     // prázdna pamäť – nič
        default: break;                                // AUTO v MEMORY nič nerobí
    }

    if (rep_btn != BTN_NONE) {
        if (BUTTON_GetHeld() != rep_btn) {
            rep_btn = BTN_NONE;
        } else if ((int32_t)(now - rep_tick) >= 0) {
            MemStep((rep_btn == BTN_F2) ? +1 : -1);
            rep_tick = now + REPEAT_PERIOD_MS;
        }
    }
}

// ---------------------------------------------------------------------------
void GenMenu_Open(bool opened_from_scope) {
    from_scope   = opened_from_scope;
    save_pending = false;
    f1_pending   = false;
    if (g_settings.gen_auto_on && !g_gen.on) Gen_SetOutput(true);   // "Auto output ON"
    LCD_Clear(LCD_BLACK);
    DrawFrame();
    ShowMain();
    GenMenu_Task(BTN_NONE);
}

void GenMenu_Close(void) {
    if (screen == SCR_SETUP) {                 // odchod priamo z nastavení: uložiť ako pri EXIT
        CalStop();
        g_settings.gen_auto_on   = set_auto_on;
        g_settings.gen_on_change = set_on_change;
        g_settings.gen_cal       = (int8_t)set_cal;
        Settings_Save();
    }
    screen = SCR_MAIN;
    popup = PAR_NONE;
    rep_btn = BTN_NONE;
    save_pending = false;
    f1_pending = false;
    msg_active = false;
}

bool GenMenu_IsMenuOpen(void) {
    return screen == SCR_MEMORY || screen == SCR_SETUP;
}

bool GenMenu_CalActive(void) {
    return screen == SCR_SETUP && cal_running;
}

// v0.77: vzorky počas ustálenia sa zahodia, potom sa zobrazuje priemer CAL_AVG_N vzoriek
// (obnovuje sa po každých CAL_AVG_N vzorkách). Neplatná vzorka (OL, bez signálu) začne znova.
void GenMenu_SetCalReading(bool valid, int32_t v01mv) {
    if ((int32_t)(HAL_GetTick() - cal_settle_tick) < 0) return;
    if (!valid) {
        cal_valid = false;
        cal_sum = 0;
        cal_cnt = 0;
        return;
    }
    cal_sum += v01mv;
    if (++cal_cnt >= CAL_AVG_N) {
        cal_read  = (int32_t)((cal_sum + CAL_AVG_N / 2u) / CAL_AVG_N);
        cal_valid = true;
        cal_sum = 0;
        cal_cnt = 0;
    }
}

uint32_t GenMenu_MemoryIdleLeftMs(void) {
    uint32_t el = HAL_GetTick() - mem_input_tick;
    return (el >= MEM_IDLE_MS) ? 0 : MEM_IDLE_MS - el;
}

const char *GenMenu_Title(void) {
    return (screen == SCR_MEMORY) ? "GEN MEMORY" : (screen == SCR_SETUP) ? "GEN SETTINGS" : "GENERATOR";
}

void GenMenu_Task(ButtonType_t ev) {
    uint32_t now = HAL_GetTick();

    // MEMORY / SETTINGS: 2 min bez tlačidla -> zavrieť ako EXIT (odpočet generátora potom od začiatku)
    if (screen != SCR_MAIN) {
        if (ev != BTN_NONE || BUTTON_GetHeld() != BTN_NONE) {
            mem_input_tick = now;
        } else if (now - mem_input_tick >= MEM_IDLE_MS) {
            save_pending = false;
            if (screen == SCR_SETUP) CloseSetup();
            else                     ShowMain();
        }
    }

    SaveButtonTask(ev, now);
    if (ev == BTN_SAVE) ev = BTN_NONE;

    if (screen == SCR_MAIN)        MainButtons(ev, now);
    else if (screen == SCR_MEMORY) MemoryButtons(ev, now);
    else                           SetupButtons(ev, now);

    MsgTask();

    if (screen == SCR_MAIN) {
        DrawParams();
        DrawOutput();
        DrawPreview(false);
        DrawPopup(false);

        // Spodné políčka
        if (popup == PAR_NONE) {
            DrawCell(0, "WAVE", LCD_WHITE, LCD_BLACK);
            DrawCell(1, "FREQ", LCD_WHITE, LCD_BLACK);
            DrawCell(2, "DUTY", Gen_DutyUsed(g_gen.wave) ? LCD_WHITE : LCD_GREY, LCD_BLACK);
            DrawCell(3, "VOLT", LCD_WHITE, LCD_BLACK);
        } else {
            DrawCell(0, "EXIT", LCD_CYAN,  LCD_BLACK);
            DrawCell(1, "DOWN", LCD_WHITE, LCD_BLACK);
            DrawCell(2, "UP",   LCD_WHITE, LCD_BLACK);
            DrawCell(3, "",     LCD_WHITE, LCD_BLACK);
        }
    } else if (screen == SCR_MEMORY) {
        DrawMemList();
        DrawCell(0, "EXIT", LCD_CYAN,  LCD_BLACK);
        DrawCell(1, "DOWN", LCD_WHITE, LCD_BLACK);
        DrawCell(2, "UP",   LCD_WHITE, LCD_BLACK);
        DrawCell(3, "LOAD", GenPreset_Get(mem_sel)->used ? LCD_WHITE : LCD_GREY, LCD_BLACK);
    } else {
        DrawSetup();
        DrawCell(0, "EXIT", LCD_CYAN,  LCD_BLACK);
        DrawCell(1, "NEXT", LCD_WHITE, LCD_BLACK);
        if (set_sel == 2) {
            DrawCell(2, "-", LCD_YELLOW, LCD_BLACK);
            DrawCell(3, "+", LCD_YELLOW, LCD_BLACK);
        } else {
            bool on = (set_sel == 0) ? set_auto_on : set_on_change;
            if (on) { DrawCell(2, "OFF", LCD_WHITE,  LCD_BLACK);  DrawCell(3, "ON", LCD_SEL_FG, LCD_SEL_BG); }
            else    { DrawCell(2, "OFF", LCD_SEL_FG, LCD_SEL_BG); DrawCell(3, "ON", LCD_WHITE,  LCD_BLACK);  }
        }
    }
}
