/*
 * menu.c
 *
 *  Menu nastavení (17 položiek, viditeľných 5, F2 NEXT posúva dokola). v0.78 poradie:
 *    Meter auto OFF:    xx:00   F3 -1 min, F4 +1 min (podržanie = opakovanie) – iba multimeter
 *    Gen/Scope OFF:     OFF, 05:00 .. 120:00 po 5 min – auto vypnutie v generátore a osciloskope
 *    Auto reset time:   ON/OFF  tlačidlá a prepínač reštartujú odpočet. v0.78: OFF = odpočet sa
 *                       nereštartuje vôbec (ani Probe reset), 4 položky Probe sú sivé a kurzor ich
 *                       preskakuje; ich hodnoty ostávajú uložené a platia znova pri ON
 *    Probe Ω/dióda/pípak ON/OFF v Ω / dióde / pípaku prechod z OL na hodnotu reštartuje odpočet
 *    Probe C reset:     OFF, 100..300 pF – kapacita nad prahom reštartuje odpočet
 *    Probe V,A reset:   OFF, 10..50 digitov – DC V, DC mV, DC A, AC A
 *    Probe V-AC reset:  OFF, 200..1400 digitov – AC V, AC mV
 *    Quick R/D/C keys:  ON/OFF  v polohe Ω / dióda / pípak volia F2–F4 režim priamo (ikony)
 *    Diode beep:        ON/OFF  v0.78: v polohe dióda nepretržité pípanie pri úbytku < 0.05 V (skrat)
 *    Beep:              ON/OFF  pípnutie pri chybe (dlhé) a upozornení (krátke)
 *    Save range:        ON/OFF  v0.70: dlhé SAVE ukladá aj rozsah (AUTO / manuálny) režimu, načíta sa po zapnutí
 *    Meter memory:      24 / 48 / 72 / 96 – v0.72: počet pamätí nameraných hodnôt (krátke SAVE)
 *    Hide zeros:        ON/OFF  v0.66: nuly zľava na veľkom displeji sa nekreslia (000.0 -> 0.0)
 *    HV warning:        OFF / ON / ON+BEEP – v0.74: blesk (a pípanie) v polohe V nad 30 V AC / 60 V DC
 *    Start bright:      MIN (25 %) / 50 % / MAX (75 %) – jas po zapnutí prepínačom
 *    Long press:        400 .. 1400 ms po 100 ms – dĺžka dlhého stlačenia (F1, BL, AUTO, SAVE)
 *    Battery:           2000 .. 3500 mAh po 100 mAh – kapacita pre odhad času chodu
 *
 *  Pod položkami deliaca čiara, 2 riadky nápovedy, medzera a úplne dole napätie batérie
 *  s odhadom zostávajúceho času chodu.
 *  Odpočet do zatvorenia menu (2 min bez tlačidla) kreslí main.c vľavo hore.
 *  Texty na displeji sú po anglicky (font 5x7 nemá diakritiku).
 */

#include <stdio.h>
#include <string.h>

#include "menu.h"
#include "lcd.h"
#include "settings.h"
#include "battery.h"

typedef enum {
    ITEM_AUTO_OFF = 0,
    ITEM_GEN_OFF,
    ITEM_AUTO_RESET,
    ITEM_PROBE_OHM,
    ITEM_PROBE_C,
    ITEM_PROBE_VA,
    ITEM_PROBE_VAC,
    ITEM_QUICK_RDC,
    ITEM_DIODE_BEEP,
    ITEM_BEEP,
    ITEM_SAVE_RANGE,
    ITEM_METER_MEM,
    ITEM_HIDE_ZEROS,
    ITEM_HV_WARN,
    ITEM_START_BL,
    ITEM_LONG_PRESS,
    ITEM_BATTERY,
    ITEM_COUNT
} Item_t;

// Odber podľa jasu podsvietenia (namerané: 25 % -> 75 mA, 75 % -> 180 mA, lineárne)
#define CUR_25_MA10       750    // 75.0 mA
#define CUR_SLOPE_MA10    21     // 2.1 mA na 1 % jasu

#define MENU_ROWS         5      // viditeľné riadky
#define LINE_Y0           42
#define LINE_STEP         24
#define TEXT_X            22
#define MARK_X            6
#define SCROLL_X          312    // značky ^ / v
#define SEP_Y             164    // deliaca čiara medzi nastaveniami a nápovedou
#define HINT_Y            170
#define BAT_INFO_Y        199    // tesne nad spodným menu (y = 210), nad ním medzera

#define MENU_GRAY         0x7BEF // v0.78: neaktívne položky Probe (Auto reset time OFF)

#define REPEAT_DELAY_MS   500    // po tomto čase držania sa hodnota začne meniť sama
#define REPEAT_PERIOD_MS  100    // perióda opakovania počas držania

static uint8_t      sel_item;
static uint8_t      top_item;        // prvá viditeľná položka
static AppSettings_t edit;           // upravované hodnoty (do EEPROM až pri EXIT)
static ButtonType_t rep_btn;
static uint32_t     rep_tick;

// Cache pre prekresľovanie iba zmien
static char         last_line[MENU_ROWS][32];
static char         last_cells[4][8];
static uint8_t      last_hint_item;
static uint8_t      last_top;
static char         last_bat[48];

static const char *const hints[ITEM_COUNT][2] = {
    [ITEM_AUTO_OFF]   = { "Meter: time until auto power off (1 - 99 min).",
                          "Hold F3 / F4 to change the value faster." },
    [ITEM_AUTO_RESET] = { "ON: every button press or rotary switch turn",
                          "restarts the countdown. OFF: no Probe reset." },
    [ITEM_GEN_OFF]    = { "Power off time in generator / oscilloscope",
                          "(5 min steps). OFF = never, beep every 5 min." },
    [ITEM_PROBE_OHM]  = { "ON: in R / diode / continuity mode, probe",
                          "contact (0L -> value) restarts the countdown." },
    [ITEM_PROBE_C]    = { "Capacitance above this value restarts the",
                          "countdown (again after a drop below 70 %)." },
    [ITEM_PROBE_VA]   = { "DC V, DC mV, DC A, AC A: display digits (5.0 =",
                          "50) above this value restart the countdown." },
    [ITEM_PROBE_VAC]  = { "AC V, AC mV: display digits (5.0 = 50) above",
                          "this value restart the countdown." },
    [ITEM_QUICK_RDC]  = { "ON: in the R / diode / continuity position the",
                          "keys F2 - F4 select the mode directly (icons)." },
    [ITEM_DIODE_BEEP] = { "ON: in the diode position a continuous beep",
                          "while the drop is below 0.05 V (short)." },
    [ITEM_SAVE_RANGE] = { "ON: long SAVE also stores the range (AUTO or",
                          "manual) of the mode, used after power on." },
    [ITEM_HV_WARN]    = { "V position: red bolt above 30 V AC / 60 V DC.",
                          "ON+BEEP: also beep every 2 s. Default: ON+BEEP." },
    [ITEM_METER_MEM]  = { "Number of meter memory slots (short SAVE).",
                          "Fewer slots only hide the rest, data stays." },
    [ITEM_LONG_PRESS] = { "How long a key must be held for its second",
                          "function (F1, BL, AUTO, SAVE). 100 ms steps." },
    [ITEM_BEEP]       = { "ON: long beep on an error, short beep on a",
                          "warning (also the 5 min reminder)." },
    [ITEM_START_BL]   = { "Backlight brightness after switching the meter",
                          "on with the rotary switch (BL changes it later)." },
    [ITEM_HIDE_ZEROS] = { "ON: leading zeros on the main display are",
                          "hidden (000.0mV -> 0.0mV). Default: ON." },
    [ITEM_BATTERY]    = { "Battery capacity for the run time estimate",
                          "shown below. Default: 1500 mAh." },
};

// Položky s hodnotou OFF / ON (ostatné sa menia - / +)
static bool IsOnOff(uint8_t item) {
    return item == ITEM_AUTO_RESET || item == ITEM_PROBE_OHM || item == ITEM_QUICK_RDC || item == ITEM_BEEP ||
           item == ITEM_HIDE_ZEROS || item == ITEM_SAVE_RANGE || item == ITEM_DIODE_BEEP;
}

// v0.78: položky Probe sú pri Auto reset time OFF neaktívne (sivé, kurzor ich preskakuje)
static bool IsProbe(uint8_t item) {
    return item >= ITEM_PROBE_OHM && item <= ITEM_PROBE_VAC;
}
static bool IsDisabled(uint8_t item) {
    return IsProbe(item) && !edit.auto_reset;
}

static void LineText(uint8_t item, char *buf, size_t n) {
    char val[8];
    const char *label = "";
    switch (item) {
        case ITEM_AUTO_OFF:
            label = "Meter auto OFF:";        snprintf(val, sizeof(val), "%02u:00", edit.auto_off_min); break;
        case ITEM_AUTO_RESET:
            label = "Auto reset time:"; snprintf(val, sizeof(val), "%s", edit.auto_reset  ? "ON" : "OFF"); break;
        case ITEM_PROBE_OHM:
            // Ω je znak fontu (0x7F), za ním sa na voľné miesta kreslia ikony diódy a reproduktora
            label = "Probe \x7f        :"; snprintf(val, sizeof(val), "%s", edit.probe_reset ? "ON" : "OFF"); break;
        case ITEM_PROBE_C:
            label = "Probe C reset:";
            if (edit.probe_c == 0) snprintf(val, sizeof(val), "OFF");
            else snprintf(val, sizeof(val), "%lupF", (unsigned long)Settings_ProbeCPf(edit.probe_c));
            break;
        case ITEM_PROBE_VA:
            label = "Probe V,A reset:";
            if (edit.probe_va == 0) snprintf(val, sizeof(val), "OFF");
            else snprintf(val, sizeof(val), "%lu", (unsigned long)Settings_ProbeVaCounts(edit.probe_va));
            break;
        case ITEM_PROBE_VAC:
            label = "Probe V-AC reset:";
            if (edit.probe_vac == 0) snprintf(val, sizeof(val), "OFF");
            else snprintf(val, sizeof(val), "%lu", (unsigned long)Settings_ProbeVacCounts(edit.probe_vac));
            break;
        case ITEM_QUICK_RDC:
            label = "Quick R/D/C keys:"; snprintf(val, sizeof(val), "%s", edit.quick_rdc ? "ON" : "OFF"); break;
        case ITEM_SAVE_RANGE:
            label = "Save range:";       snprintf(val, sizeof(val), "%s", edit.save_range ? "ON" : "OFF"); break;
        case ITEM_HV_WARN:
            label = "HV warning:";
            snprintf(val, sizeof(val), "%s", (edit.hv_warn == HV_WARN_OFF) ? "OFF" :
                                             (edit.hv_warn == HV_WARN_ON)  ? "ON" : "ON+BEEP");
            break;
        case ITEM_METER_MEM:
            label = "Meter memory:";
            snprintf(val, sizeof(val), "%u", (unsigned)(SETTINGS_METER_MEM_STEP * (1u + edit.meter_mem % 4u)));
            break;
        case ITEM_LONG_PRESS:
            label = "Long press:";
            snprintf(val, sizeof(val), "%ums", (unsigned)(100u * (edit.long_press % 100u)));
            break;
        case ITEM_DIODE_BEEP:
            label = "Diode beep:";       snprintf(val, sizeof(val), "%s", edit.diode_beep ? "ON" : "OFF"); break;
        case ITEM_BEEP:
            label = "Beep:";             snprintf(val, sizeof(val), "%s", edit.beep ? "ON" : "OFF"); break;
        case ITEM_START_BL:
            label = "Start bright:";
            snprintf(val, sizeof(val), "%s", (edit.start_bl == 0) ? "MIN" : (edit.start_bl == 1) ? "50%" : "MAX");
            break;
        case ITEM_HIDE_ZEROS:
            label = "Hide zeros:";       snprintf(val, sizeof(val), "%s", edit.hide_zeros ? "ON" : "OFF"); break;
        case ITEM_BATTERY:
            label = "Battery:";
            snprintf(val, sizeof(val), "%umAh", (unsigned)(100u * (edit.battery_cap % 100u)));
            break;
        default:
            label = "Gen/Scope OFF:";
            if (edit.gen_off_min == 0) snprintf(val, sizeof(val), "OFF");
            else                       snprintf(val, sizeof(val), "%02u:00", edit.gen_off_min);
            break;
    }
    snprintf(buf, n, "%-17s%7s", label, val);
}

static void DrawCell(uint8_t idx, const char *txt, uint16_t fg, uint16_t bg) {
    // Kľúč cache obsahuje aj farbu pozadia, aby sa prekreslilo pri zmene zvýraznenia
    char key[8];
    snprintf(key, sizeof(key), "%.5s%c", txt, (bg == LCD_BLACK) ? '0' : (bg == LCD_YELLOW) ? '1' : '2');
    if (strcmp(key, last_cells[idx]) == 0) return;
    strcpy(last_cells[idx], key);
    LCD_DrawMenuCell(idx, txt, fg, bg);
}

// Vybraná položka vždy viditeľná (pri prechode dokola skočí zoznam na začiatok)
static void Scroll(void) {
    if (sel_item < top_item) top_item = sel_item;
    if (sel_item >= top_item + MENU_ROWS) top_item = (uint8_t)(sel_item - (MENU_ROWS - 1u));
}

static void Redraw(void) {
    ButtonType_t held = BUTTON_GetHeld();

    // Riadky nastavení: vybraný žltý so šípkou
    for (uint8_t r = 0; r < MENU_ROWS; r++) {
        uint8_t item = (uint8_t)(top_item + r);
        char buf[28];
        LineText(item, buf, sizeof(buf));
        buf[24] = '\0';
        char key[32];
        bool gray = IsDisabled(item);
        snprintf(key, sizeof(key), "%c%c%c%s", (item == sel_item) ? '>' : ' ', 'A' + item, gray ? 'g' : 'n', buf);
        if (strcmp(key, last_line[r]) == 0) continue;
        uint16_t y = LINE_Y0 + r * LINE_STEP;
        // v0.75: písmo nekreslí stĺpce medzi znakmi (6. stĺpec), ikony diódy a reproduktora áno ->
        // po posunutí zoznamu ostávali z nich v riadku body. Ak bol v riadku "Probe Ω", zmazať miesto ikon.
        if (last_line[r][1] == (char)('A' + ITEM_PROBE_OHM) && item != ITEM_PROBE_OHM)
            LCD_FillRect(TEXT_X + 8 * 12, y, 5 * 12, 16, LCD_BLACK);
        strcpy(last_line[r], key);

        uint16_t color = (item == sel_item) ? LCD_YELLOW : gray ? MENU_GRAY : LCD_WHITE;
        LCD_DrawString(MARK_X, y, (item == sel_item) ? ">" : " ", LCD_YELLOW, LCD_BLACK, 2);
        LCD_DrawString(TEXT_X, y, buf, color, LCD_BLACK, 2);
        if (item == ITEM_PROBE_OHM) {
            // "Probe Ω" + dióda (znaky 8-9) + reproduktor (znaky 11-12)
            LCD_DrawIcon(TEXT_X + 8 * 12,  y, ICON_DIODE, color, LCD_BLACK, 2);
            LCD_DrawIcon(TEXT_X + 11 * 12, y, ICON_BEEP,  color, LCD_BLACK, 2);
        }
        if (item == ITEM_BATTERY && edit.battery_cap == SETTINGS_BAT_CAP_DEF)
            LCD_DrawString(TEXT_X + 17 * 12, y, buf + 17, LCD_GREEN, LCD_BLACK, 2);   // predvolená hodnota zelená
    }

    // Značky, že sú ďalšie položky nad / pod viditeľnými
    if (top_item != last_top) {
        last_top = top_item;
        LCD_DrawString(SCROLL_X, LINE_Y0 + 2, (top_item > 0) ? "^" : " ", LCD_WHITE, LCD_BLACK, 1);
        LCD_DrawString(SCROLL_X, LINE_Y0 + (MENU_ROWS - 1) * LINE_STEP + 8,
                       (top_item + MENU_ROWS < ITEM_COUNT) ? "v" : " ", LCD_WHITE, LCD_BLACK, 1);
    }

    // Nápoveda k vybranej položke (malé písmo)
    if (last_hint_item != sel_item) {
        last_hint_item = sel_item;
        LCD_FillRect(2, HINT_Y, 316, 18, LCD_BLACK);
        LCD_DrawString(10, HINT_Y,      hints[sel_item][0], LCD_LIGHTGREY, LCD_BLACK, 1);
        LCD_DrawString(10, HINT_Y + 10, hints[sel_item][1], LCD_LIGHTGREY, LCD_BLACK, 1);
    }

    // Informácia: napätie batérie a odhad zostávajúceho času chodu.
    // Čas = zostávajúca kapacita (percentá z priemeru ~30 s x nastavená kapacita) / odber podľa jasu.
    // Je to hrubý odhad: percentá sú z napätia (Li-ion krivka), preto zaokrúhlené po 10 min.
    {
        uint16_t mv   = Battery_GetMillivolts();
        uint32_t pct  = Battery_GetAvgPercent();
        uint32_t cap  = 100u * edit.battery_cap;                       // mAh (aj neuložená zmena)
        int32_t  bl   = LCD_BL_GetBrightness();
        int32_t  ma10 = CUR_25_MA10 + (bl - 25) * CUR_SLOPE_MA10;      // 0.1 mA
        if (ma10 < 100) ma10 = 100;
        uint32_t min  = (uint32_t)((uint64_t)pct * cap * 600u / 100u / (uint32_t)ma10);   // minúty
        min = (min + 5u) / 10u * 10u;
        char b[48];
        snprintf(b, sizeof(b), "Battery %u.%02u V  %u %%   ~%lu h %02lu min",
                 mv / 1000u, (mv % 1000u) / 10u, Battery_GetPercent(),
                 (unsigned long)(min / 60u), (unsigned long)(min % 60u));
        if (strcmp(b, last_bat) != 0) {
            strcpy(last_bat, b);
            LCD_FillRect(2, BAT_INFO_Y, 316, 9, LCD_BLACK);
            LCD_DrawString(10, BAT_INFO_Y, b, LCD_LIGHTGREY, LCD_BLACK, 1);
        }
    }

    // Spodné políčka s funkciami tlačidiel
    DrawCell(0, "EXIT", LCD_CYAN,  LCD_BLACK);
    DrawCell(1, "NEXT", LCD_WHITE, LCD_BLACK);
    if (!IsOnOff(sel_item)) {
        // - / + : žlté pozadie, kým je tlačidlo držané
        if (held == BTN_F3) DrawCell(2, "-", LCD_BLACK,  LCD_YELLOW);
        else                DrawCell(2, "-", LCD_YELLOW, LCD_BLACK);
        if (held == BTN_F4) DrawCell(3, "+", LCD_BLACK,  LCD_YELLOW);
        else                DrawCell(3, "+", LCD_YELLOW, LCD_BLACK);
    } else {
        // OFF / ON : aktívna voľba má zelené pozadie (ako Hz / DUTY)
        bool on = (sel_item == ITEM_AUTO_RESET) ? edit.auto_reset :
                  (sel_item == ITEM_PROBE_OHM)   ? edit.probe_reset :
                  (sel_item == ITEM_BEEP)        ? edit.beep :
                  (sel_item == ITEM_HIDE_ZEROS)  ? edit.hide_zeros :
                  (sel_item == ITEM_SAVE_RANGE)  ? edit.save_range :
                  (sel_item == ITEM_DIODE_BEEP)  ? edit.diode_beep : edit.quick_rdc;
        if (on) { DrawCell(2, "OFF", LCD_WHITE,  LCD_BLACK);  DrawCell(3, "ON", LCD_SEL_FG, LCD_SEL_BG); }
        else    { DrawCell(2, "OFF", LCD_SEL_FG, LCD_SEL_BG); DrawCell(3, "ON", LCD_WHITE,  LCD_BLACK);  }
    }
}

// Zmena hodnoty o krok v rozsahu 0..max (bez pretočenia)
static void StepIdx(uint8_t *v, uint8_t max, ButtonType_t b) {
    if (b == BTN_F3 && *v > 0)   (*v)--;
    if (b == BTN_F4 && *v < max) (*v)++;
}

static void Adjust(ButtonType_t b) {
    switch (sel_item) {
        case ITEM_AUTO_OFF:
            if (b == BTN_F3 && edit.auto_off_min > SETTINGS_AUTO_OFF_MIN_MIN) edit.auto_off_min--;
            if (b == BTN_F4 && edit.auto_off_min < SETTINGS_AUTO_OFF_MIN_MAX) edit.auto_off_min++;
            break;
        case ITEM_AUTO_RESET: edit.auto_reset  = (b == BTN_F4); break;
        case ITEM_PROBE_OHM:  edit.probe_reset = (b == BTN_F4); break;
        case ITEM_QUICK_RDC:  edit.quick_rdc   = (b == BTN_F4); break;
        case ITEM_SAVE_RANGE: edit.save_range  = (b == BTN_F4); break;
        case ITEM_LONG_PRESS:
            if (b == BTN_F3 && edit.long_press > SETTINGS_LONG_PRESS_MIN) edit.long_press--;
            if (b == BTN_F4 && edit.long_press < SETTINGS_LONG_PRESS_MAX) edit.long_press++;
            break;
        case ITEM_BEEP:       edit.beep = (b == BTN_F4); break;
        case ITEM_DIODE_BEEP: edit.diode_beep = (b == BTN_F4); break;
        case ITEM_HIDE_ZEROS: edit.hide_zeros = (b == BTN_F4); break;
        case ITEM_START_BL:   StepIdx(&edit.start_bl, SETTINGS_START_BL_MAX, b); break;
        case ITEM_HV_WARN:    StepIdx(&edit.hv_warn, SETTINGS_HV_WARN_MAX, b); break;
        case ITEM_METER_MEM:  StepIdx(&edit.meter_mem, SETTINGS_METER_MEM_MAX, b); break;
        case ITEM_BATTERY:
            if (b == BTN_F3 && edit.battery_cap > SETTINGS_BAT_CAP_MIN) edit.battery_cap--;
            if (b == BTN_F4 && edit.battery_cap < SETTINGS_BAT_CAP_MAX) edit.battery_cap++;
            break;
        case ITEM_PROBE_C:    StepIdx(&edit.probe_c,   SETTINGS_PROBE_C_MAX,   b); break;
        case ITEM_PROBE_VA:   StepIdx(&edit.probe_va,  SETTINGS_PROBE_VA_MAX,  b); break;
        case ITEM_PROBE_VAC:  StepIdx(&edit.probe_vac, SETTINGS_PROBE_VAC_MAX, b); break;
        case ITEM_GEN_OFF:
            if (b == BTN_F3 && edit.gen_off_min >= SETTINGS_GEN_OFF_STEP) edit.gen_off_min -= SETTINGS_GEN_OFF_STEP;
            if (b == BTN_F4 && edit.gen_off_min + SETTINGS_GEN_OFF_STEP <= SETTINGS_GEN_OFF_MAX) edit.gen_off_min += SETTINGS_GEN_OFF_STEP;
            break;
        default: break;
    }
}

void Menu_Open(void) {
    edit = g_settings;
    sel_item = 0;
    top_item = 0;
    rep_btn = BTN_NONE;
    memset(last_line, 0, sizeof(last_line));
    memset(last_cells, 0, sizeof(last_cells));
    last_hint_item = 0xFF;
    last_top = 0xFF;
    last_bat[0] = '\0';

    LCD_Clear(LCD_BLACK);

    // Biely rámik okolo celého displeja
    LCD_DrawFastHLine(0, 0, 320, LCD_WHITE);
    LCD_DrawFastHLine(0, 239, 320, LCD_WHITE);
    LCD_DrawFastVLine(0, 0, 240, LCD_WHITE);
    LCD_DrawFastVLine(319, 0, 240, LCD_WHITE);

    // Nadpis a deliace čiary (pod nadpisom, medzi nastaveniami a nápovedou)
    LCD_DrawString((320 - (4 * 12 - 2)) / 2, 7, "MENU", LCD_CYAN, LCD_BLACK, 2);   // y ako časomiera (TOP_Y)
    LCD_DrawFastHLine(0, 27, 320, LCD_WHITE);      // horný pruh 6 / 6 px (v0.62)
    LCD_DrawFastHLine(0, SEP_Y, 320, LCD_WHITE);

    LCD_DrawMenuFrame();
    Redraw();
}

bool Menu_Task(ButtonType_t ev) {
    uint32_t now = HAL_GetTick();

    switch (ev) {
        case BTN_F1:                                   // EXIT + uloženie
            g_settings = edit;
            Settings_Save();                           // zapíše iba ak sa niečo zmenilo
            return false;

        case BTN_F2:                                   // ďalšia položka dokola
            do {                                       // v0.78: neaktívne položky preskočiť
                sel_item = (uint8_t)((sel_item + 1) % ITEM_COUNT);
            } while (IsDisabled(sel_item));
            Scroll();
            break;

        case BTN_F3:
        case BTN_F4:
            Adjust(ev);
            rep_btn  = ev;
            rep_tick = now + REPEAT_DELAY_MS;
            break;

        default:
            break;
    }

    // Opakovanie počas držania F3 / F4
    ButtonType_t held = BUTTON_GetHeld();
    if (rep_btn != BTN_NONE) {
        if (held != rep_btn) {
            rep_btn = BTN_NONE;
        } else if ((int32_t)(now - rep_tick) >= 0) {
            Adjust(rep_btn);
            rep_tick = now + REPEAT_PERIOD_MS;
        }
    }

    Redraw();
    return true;
}
