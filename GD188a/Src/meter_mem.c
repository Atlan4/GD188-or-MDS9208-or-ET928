/*
 * meter_mem.c
 *
 *  v0.71: pamäť nameraných hodnôt multimetra – otvára sa krátkym SAVE na meracej obrazovke.
 *  v0.72: 24 / 48 / 72 / 96 pamätí (menu "Meter memory"), po 24 na stránku; hlásenia v hornom riadku.
 *
 *  Obrazovka (biely rámik ako generátor / menu, horný riadok kreslí main.c:
 *  "METER MEMORY" a pri viac ako 24 pamätiach aj stránka, napr. "METER MEMORY 2/4"):
 *    y  33      NOW  aktuálna hodnota z displeja (HOLD = zmrazená), jednotka, AC/DC alebo ikona
 *    y  50..193 24 pamätí stránky: 3 stĺpce x 8 riadkov (1–8, 9–16, 17–24 zhora nadol; na 2. stránke
 *               25–32 ...), políčko 104 x 17 px: číslo pamäte malým písmom, hodnota veľkým (6 znakov),
 *               vpravo malým jednotka a pod ňou AC / DC alebo ikona diódy / pípaku.
 *               Vybraná pamäť má zelené pozadie.
 *    y 197      nápoveda
 *    y 212      spodné menu: F1 EXIT  F2 UP  F3 DOWN  F4 COL >
 *  Hlásenia (SAVED TO 5, DELETED 5, NO VALUE ...) sú 1.5 s v hornom riadku cez celú šírku
 *  (main.c, MeterMem_TopMsg) – časomiera a batéria sa vtedy nekreslia.
 *
 *  Ovládanie:
 *    F1          späť na meraciu obrazovku
 *    F2 / F3     predchádzajúca / ďalšia pamäť (dokola cez stĺpce aj stránky), podržanie = opakovanie
 *    F4          rovnaký riadok v ďalšom stĺpci; z 3. stĺpca na 1. stĺpec ďalšej stránky (dokola)
 *    SAVE krátko aktuálnu hodnotu do vybranej pamäte (obsadenú prepíše), výber sa posunie na ďalšiu
 *                v0.76: pri zapnutom MIN/MAX 3 pamäte za sebou: hodnota, MAX (značka ▲, oranžové číslo),
 *                MIN (značka ▼, azúrové číslo); za poslednou pamäťou pokračuje od 1 (ako posun výberu)
 *    AUTO dlho   vymaže vybranú pamäť (krátke AUTO iba pripomenie, že treba podržať)
 *    BL          jas (rieši main.c); dlhé BL a dlhé SAVE tu nerobia nič
 *  Meranie beží ďalej, otočenie prepínača obrazovku nezatvára (NOW ukáže novú veličinu).
 *
 *  Pri znížení počtu pamätí sa vyššie pamäte nemažú – iba sa nezobrazujú (po zvýšení sú späť).
 *
 *  EEPROM (EE_METER_MEM_ADDR = 0x0100, 96 x 8 B = 12 stránok, pamäť nikdy neprechádza cez hranicu stránky):
 *    [0] režim MeterMemMode_t (1..16), 0xFF = prázdna
 *    [1] bity 0–2 desatinné miesta, bit 3 značka MAX (v0.76), bit 4 záporné, bit 5 OL, bit 6 REL,
 *        bit 7 značka MIN (v0.76; staršie verzie bity 3 a 7 ignorujú)
 *    [2] [3] hodnota 0..9999 (little endian)
 *    [4] [5] [6] jednotka (doplnená nulami)
 *    [7] kontrolný súčet = ~(súčet bajtov 0..6)
 *  Vymazanie zapíše 8 x 0xFF. Zle zapísaná pamäť (výpadok napájania) má zlý súčet -> prázdna.
 */

#include <stdio.h>
#include <string.h>

#include "meter_mem.h"
#include "eeprom.h"
#include "settings.h"
#include "lcd.h"
#include "beep.h"

// --- Rozloženie ---
#define TOP_LINE_Y     27
#define NOW_Y          33
#define GRID_Y         50
#define ROW_H          18
#define CELL_H         17
#define COL_X0         2
#define COL_PITCH      106
#define CELL_W         104
#define NUM_DX         1          // číslo pamäte (malé písmo, 2 znaky)
#define VAL_DX         14         // hodnota (size 2, 6 znakov = 70 px)
#define UNIT_DX        86         // jednotka / AC DC (malé písmo, 3 znaky = 17 px)
#define INFO_Y         197
#define MSG_MS         1500u
#define PAGE_SLOTS     (METER_MEM_ROWS * METER_MEM_COLS)

#define REPEAT_DELAY_MS   400u
#define REPEAT_PERIOD_MS  150u

#define SLOT_SIZE      8u

static MeterMemVal_t slots[PAGE_SLOTS];      // pamäte zobrazenej stránky
static uint8_t       page = 0xFF;             // stránka v slots[] (0xFF = nenačítaná)
static uint8_t       count = METER_MEM_SLOTS_MIN;   // počet pamätí podľa nastavenia
static uint8_t       sel = 0;                 // vybraná pamäť (0..count-1) – pamätá sa do vypnutia
static char          last_cell[PAGE_SLOTS][28];
static char          last_now[40];
static bool          msg_active = false;
static uint32_t      msg_until;
static char          msg_text[24];
static uint16_t      msg_col;
static char          title[24];
static bool          save_pending = false;
static uint32_t      save_tick;
static bool          auto_pending = false;
static uint32_t      auto_tick;
static ButtonType_t  rep_btn = BTN_NONE;
static int8_t        last_info = -1;          // v0.76: nápoveda s MIN/MAX / bez
static uint32_t      rep_tick;

// ---------------------------------------------------------------------------
// EEPROM
// ---------------------------------------------------------------------------
static uint16_t SlotAddr(uint8_t i) { return (uint16_t)(EE_METER_MEM_ADDR + i * SLOT_SIZE); }

static uint8_t SlotSum(const uint8_t *b) {
    uint8_t s = 0;
    for (uint8_t i = 0; i < SLOT_SIZE - 1u; i++) s = (uint8_t)(s + b[i]);
    return (uint8_t)~s;
}

static void Decode(const uint8_t *b, MeterMemVal_t *v) {
    memset(v, 0, sizeof(*v));
    if (b[0] == 0u || b[0] >= MM_COUNT || b[7] != SlotSum(b)) return;
    uint16_t mag = (uint16_t)(b[2] | (b[3] << 8));
    if (mag > 9999u || (b[1] & 7u) > 4u) return;
    v->used     = true;
    v->mode     = b[0];
    v->decimals = (uint8_t)(b[1] & 7u);
    v->neg      = (b[1] & 0x10u) != 0;
    v->ol       = (b[1] & 0x20u) != 0;
    v->rel      = (b[1] & 0x40u) != 0;
    v->tag      = (b[1] & 0x08u) ? MM_TAG_MAX : (b[1] & 0x80u) ? MM_TAG_MIN : MM_TAG_NONE;
    v->mag      = mag;
    for (uint8_t i = 0; i < 3u; i++) {
        char c = (char)b[4 + i];
        v->unit[i] = (c >= ' ' && c <= 0x7f) ? c : '\0';
    }
    v->unit[3] = '\0';
}

static void Encode(const MeterMemVal_t *v, uint8_t *b) {
    b[0] = v->mode;
    b[1] = (uint8_t)((v->decimals & 7u) | (v->neg ? 0x10u : 0u) | (v->ol ? 0x20u : 0u) | (v->rel ? 0x40u : 0u) |
                     (v->tag == MM_TAG_MAX ? 0x08u : 0u) | (v->tag == MM_TAG_MIN ? 0x80u : 0u));
    b[2] = (uint8_t)(v->mag & 0xFFu);
    b[3] = (uint8_t)(v->mag >> 8);
    for (uint8_t i = 0; i < 3u; i++) b[4 + i] = (uint8_t)v->unit[i];
    b[7] = SlotSum(b);
}

// Načíta stránku p (24 pamätí) do slots[]
static void ReadPage(uint8_t p) {
    uint8_t buf[PAGE_SLOTS * SLOT_SIZE];
    bool ok = EEPROM_Read(SlotAddr((uint8_t)(p * PAGE_SLOTS)), buf, sizeof(buf));
    for (uint8_t i = 0; i < PAGE_SLOTS; i++) {
        if (ok) Decode(&buf[i * SLOT_SIZE], &slots[i]);
        else    memset(&slots[i], 0, sizeof(slots[i]));     // EEPROM neodpovedá -> prázdne
    }
    page = p;
}

static uint8_t SelPage(void)  { return (uint8_t)(sel / PAGE_SLOTS); }
static uint8_t Pages(void)    { return (uint8_t)(count / PAGE_SLOTS); }
static MeterMemVal_t *SelSlot(void) { return &slots[sel % PAGE_SLOTS]; }

// Zápis 8 B s kontrolou spätným čítaním
static bool WriteSlot(uint8_t i, const uint8_t *b) {
    if (!EEPROM_Write(SlotAddr(i), b, SLOT_SIZE)) return false;
    uint8_t chk[SLOT_SIZE];
    return EEPROM_Read(SlotAddr(i), chk, SLOT_SIZE) && memcmp(chk, b, SLOT_SIZE) == 0;
}

// ---------------------------------------------------------------------------
// Texty
// ---------------------------------------------------------------------------

// Hodnota bez núl zľava, zarovnaná doprava na 6 znakov: "-12.34", " 0.500", "  1000", "    OL"
static void ValueText(const MeterMemVal_t *v, char *buf, size_t n) {
    char t[12];
    if (v->ol) {
        strcpy(t, "OL");
    } else {
        // číslice odzadu, bodka za 'decimals' číslicami, aspoň jedna číslica pred bodkou
        char r[12];
        uint8_t  k   = 0;
        uint8_t  dec = (v->decimals > 4u) ? 4u : v->decimals;
        uint16_t m   = (v->mag > 9999u) ? 9999u : v->mag;
        for (uint8_t i = 0; i < dec; i++) { r[k++] = (char)('0' + m % 10u); m = (uint16_t)(m / 10u); }
        if (dec) r[k++] = '.';
        do { r[k++] = (char)('0' + m % 10u); m = (uint16_t)(m / 10u); } while (m);
        if (v->neg) r[k++] = '-';
        for (uint8_t i = 0; i < k; i++) t[i] = r[k - 1u - i];
        t[k] = '\0';
    }
    snprintf(buf, n, "%6.6s", t);
}

static bool ModeIsAc(uint8_t m) {
    return m == MM_V_AC || m == MM_mV_AC || m == MM_uA_AC || m == MM_mA_AC || m == MM_A_AC;
}
static bool ModeIsDc(uint8_t m) {
    return m == MM_V_DC || m == MM_mV_DC || m == MM_uA_DC || m == MM_mA_DC || m == MM_A_DC;
}

// AC / DC (text) alebo ikona (dióda, pípak) na pozícii x, y; size 1 alebo 2
static void DrawTag(uint16_t x, uint16_t y, uint8_t mode, bool sel_bg, uint16_t bg, uint8_t size) {
    if (ModeIsAc(mode))      LCD_DrawString(x, y, "AC", sel_bg ? LCD_SEL_FG : LCD_ORANGE, bg, size);
    else if (ModeIsDc(mode)) LCD_DrawString(x, y, "DC", sel_bg ? LCD_SEL_FG : LCD_GREEN,  bg, size);
    else if (mode == MM_DIODE) LCD_DrawIcon(x, y, ICON_DIODE, sel_bg ? LCD_SEL_FG : LCD_CYAN, bg, size);
    else if (mode == MM_CONT)  LCD_DrawIcon(x, y, ICON_BEEP,  sel_bg ? LCD_SEL_FG : LCD_CYAN, bg, size);
}

// ---------------------------------------------------------------------------
// Kreslenie
// ---------------------------------------------------------------------------
// i = políčko na stránke (0..23), číslo pamäte = stránka * 24 + i
static void DrawCell(uint8_t i) {
    const MeterMemVal_t *v = &slots[i];
    uint8_t n = (uint8_t)(page * PAGE_SLOTS + i);
    bool s = (n == sel);
    char val[12], key[28];
    if (v->used) ValueText(v, val, sizeof(val));
    else         strcpy(val, "   ---");
    snprintf(key, sizeof(key), "%c%02u%u%u%02u%.6s%.3s", s ? '>' : ' ', (unsigned)n, v->used, v->tag, v->mode, val, v->unit);
    if (strcmp(key, last_cell[i]) == 0) return;
    strcpy(last_cell[i], key);

    uint8_t  c  = (uint8_t)(i / METER_MEM_ROWS), r = (uint8_t)(i % METER_MEM_ROWS);
    uint16_t x  = (uint16_t)(COL_X0 + c * COL_PITCH);
    uint16_t y  = (uint16_t)(GRID_Y + r * ROW_H);
    uint16_t bg = s ? LCD_SEL_BG : LCD_BLACK;
    LCD_FillRect(x, y, CELL_W, CELL_H, bg);

    char num[4];
    snprintf(num, sizeof(num), "%2u", (unsigned)(n + 1u));
    uint16_t tc = (v->used && v->tag == MM_TAG_MAX) ? LCD_ORANGE : (v->used && v->tag == MM_TAG_MIN) ? LCD_CYAN : LCD_LIGHTGREY;
    LCD_DrawString((uint16_t)(x + NUM_DX), (uint16_t)(y + 5), num, s ? LCD_SEL_FG : tc, bg, 1);
    if (v->used && v->tag != MM_TAG_NONE) {             // v0.76: ▲ nad číslom (MAX), ▼ pod číslom (MIN)
        uint16_t mc = s ? LCD_SEL_FG : tc;
        for (uint16_t r = 0; r < 3u; r++) {
            uint16_t yy = (v->tag == MM_TAG_MAX) ? (uint16_t)(y + 1u + r) : (uint16_t)(y + 15u - r);
            LCD_DrawFastHLine((uint16_t)(x + 7u - r), yy, (uint16_t)(2u * r + 1u), mc);
        }
    }

    uint16_t vc = s ? LCD_SEL_FG : !v->used ? LCD_GREY : v->ol ? LCD_RED : LCD_WHITE;
    LCD_DrawString((uint16_t)(x + VAL_DX), (uint16_t)(y + 1), val, vc, bg, 2);   // 8 riadkov fontu = 16 px
    if (!v->used) return;
    LCD_DrawString((uint16_t)(x + UNIT_DX), (uint16_t)(y + 1), v->unit, s ? LCD_SEL_FG : LCD_YELLOW, bg, 1);
    DrawTag((uint16_t)(x + UNIT_DX), (uint16_t)(y + 9), v->mode, s, bg, 1);
}

static void DrawGrid(void) {
    if (page != SelPage()) ReadPage(SelPage());      // výber prešiel na inú stránku
    for (uint8_t i = 0; i < PAGE_SLOTS; i++) DrawCell(i);
}

// Riadok NOW: aktuálna hodnota, ktorá sa uloží
static void DrawNow(const MeterMemVal_t *now, bool hold, bool minmax) {
    char val[12], key[40];
    if (now->used) ValueText(now, val, sizeof(val));
    else           strcpy(val, "  ----");
    snprintf(key, sizeof(key), "%u%u%u%u%u%s%.3s", hold, minmax, now->used, now->mode, now->rel, val, now->unit);
    if (strcmp(key, last_now) == 0) return;
    strcpy(last_now, key);

    LCD_FillRect(2, NOW_Y - 1, 316, 16, LCD_BLACK);
    LCD_DrawString(8, NOW_Y, hold ? "HOLD" : "NOW", hold ? LCD_ORANGE : LCD_LIGHTGREY, LCD_BLACK, 2);
    LCD_DrawString(62, NOW_Y, val, !now->used ? LCD_GREY : now->ol ? LCD_RED : LCD_YELLOW, LCD_BLACK, 2);
    if (!now->used) return;
    LCD_DrawString(146, NOW_Y, now->unit, LCD_YELLOW, LCD_BLACK, 2);
    DrawTag(194, NOW_Y, now->mode, false, LCD_BLACK, 2);
    if (now->rel) LCD_DrawString(236, NOW_Y, "REL", LCD_CYAN, LCD_BLACK, 2);
    if (minmax) {                                         // v0.76: SAVE uloží aj MAX a MIN
        LCD_DrawString(278, NOW_Y - 1, "+MAX", LCD_ORANGE, LCD_BLACK, 1);
        LCD_DrawString(278, NOW_Y + 7, "+MIN", LCD_CYAN, LCD_BLACK, 1);
    }
}

static void DrawInfo(bool minmax) {
    last_info = minmax ? 1 : 0;
    LCD_FillRect(2, INFO_Y - 1, 316, 15, LCD_BLACK);
    const char *t = minmax ? "SAVE = value + MAX + MIN   hold AUTO = delete" : "SAVE = store   hold AUTO = delete";
    LCD_DrawString((uint16_t)((320 - (int)strlen(t) * 6 + 1) / 2), INFO_Y + 4, t, LCD_LIGHTGREY, LCD_BLACK, 1);
}

// v0.72: hlásenie v hornom riadku (kreslí main.c cez MeterMem_TopMsg)
static void MsgShow(const char *txt, uint16_t color) {
    if (color == LCD_RED) Beep_Error();
    snprintf(msg_text, sizeof(msg_text), "%s", txt);
    msg_col    = color;
    msg_active = true;
    msg_until  = HAL_GetTick() + MSG_MS;
}

static void DrawFrame(void) {
    LCD_DrawFastHLine(0, 0, 320, LCD_WHITE);
    LCD_DrawFastHLine(0, 239, 320, LCD_WHITE);
    LCD_DrawFastVLine(0, 0, 240, LCD_WHITE);
    LCD_DrawFastVLine(319, 0, 240, LCD_WHITE);
    LCD_DrawFastHLine(0, TOP_LINE_Y, 320, LCD_WHITE);
    // deliace čiary medzi stĺpcami
    for (uint8_t c = 1; c < METER_MEM_COLS; c++) {
        LCD_DrawFastVLine((uint16_t)(COL_X0 + c * COL_PITCH - 2), GRID_Y, METER_MEM_ROWS * ROW_H - 1, LCD_GREY);
    }
    LCD_DrawMenuFrame();
    LCD_DrawMenuCell(0, "EXIT",  LCD_WHITE, LCD_BLACK);
    LCD_DrawMenuCell(1, "UP",    LCD_WHITE, LCD_BLACK);
    LCD_DrawMenuCell(2, "DOWN",  LCD_WHITE, LCD_BLACK);
    LCD_DrawMenuCell(3, "COL >", LCD_WHITE, LCD_BLACK);
}

// ---------------------------------------------------------------------------
// Akcie
// ---------------------------------------------------------------------------
// v0.76: hodnota + MAX + MIN do 3 pamätí za sebou (sel, sel+1, sel+2; za poslednou od 1)
static void Store3(const MeterMemVal_t *now, const MeterMemVal_t *mx, const MeterMemVal_t *mn) {
    if (!now->used || now->mode == MM_NONE) { MsgShow("NO VALUE", LCD_RED); return; }
    const MeterMemVal_t *src[3] = { now, mx, mn };
    uint8_t n[3];
    for (uint8_t k = 0; k < 3u; k++) {
        MeterMemVal_t v = *src[k];
        if (!v.used) { v = *now; v.ol = true; v.mag = 0; v.neg = false; }   // ešte bez MAX / MIN (nemá nastať)
        v.mode = now->mode;
        v.tag  = (k == 1u) ? MM_TAG_MAX : (k == 2u) ? MM_TAG_MIN : MM_TAG_NONE;
        n[k] = (uint8_t)((sel + k) % count);
        uint8_t b[SLOT_SIZE];
        Encode(&v, b);
        if (!WriteSlot(n[k], b)) { MsgShow("SAVE ERROR", LCD_RED); return; }
        if (n[k] / PAGE_SLOTS == page) Decode(b, &slots[n[k] % PAGE_SLOTS]);
    }
    char t[24];
    snprintf(t, sizeof(t), "SAVED TO %u,%u,%u", (unsigned)(n[0] + 1u), (unsigned)(n[1] + 1u), (unsigned)(n[2] + 1u));
    MsgShow(t, LCD_GREEN);
    sel = (uint8_t)((sel + 3u) % count);
    if (SelPage() != page) ReadPage(SelPage());          // výber prešiel na inú stránku (DrawGrid)
    else                   memset(last_cell, 0, sizeof(last_cell));
}

static void Store(const MeterMemVal_t *now) {
    if (!now->used || now->mode == MM_NONE) { MsgShow("NO VALUE", LCD_RED); return; }
    uint8_t b[SLOT_SIZE];
    MeterMemVal_t v = *now;
    v.tag = MM_TAG_NONE;
    Encode(&v, b);
    char t[24];
    if (WriteSlot(sel, b)) {
        Decode(b, SelSlot());
        snprintf(t, sizeof(t), "SAVED TO %u", (unsigned)(sel + 1u));
        MsgShow(t, LCD_GREEN);
        sel = (uint8_t)((sel + 1u) % count);               // ďalšia pamäť pre ďalšie meranie
    } else {
        MsgShow("SAVE ERROR", LCD_RED);
    }
}

static void Delete(void) {
    char t[24];
    if (!SelSlot()->used) { MsgShow("SLOT IS EMPTY", LCD_ORANGE); return; }
    uint8_t b[SLOT_SIZE];
    memset(b, 0xFF, sizeof(b));
    if (WriteSlot(sel, b)) {
        memset(SelSlot(), 0, sizeof(MeterMemVal_t));
        snprintf(t, sizeof(t), "DELETED %u", (unsigned)(sel + 1u));
        MsgShow(t, LCD_YELLOW);
    } else {
        MsgShow("DELETE ERROR", LCD_RED);
    }
}

static void Step(ButtonType_t b) {
    if (b == BTN_F2)      sel = (uint8_t)((sel + count - 1u) % count);
    else if (b == BTN_F3) sel = (uint8_t)((sel + 1u) % count);
}

// F4: rovnaký riadok v ďalšom stĺpci, z 3. stĺpca na 1. stĺpec ďalšej stránky (dokola)
static void NextCol(void) {
    uint8_t in = (uint8_t)(sel % PAGE_SLOTS);
    if (in / METER_MEM_ROWS < METER_MEM_COLS - 1u) {
        sel = (uint8_t)(sel + METER_MEM_ROWS);
    } else {
        uint8_t p = (uint8_t)((SelPage() + 1u) % Pages());
        sel = (uint8_t)(p * PAGE_SLOTS + in % METER_MEM_ROWS);
    }
}

// ---------------------------------------------------------------------------
// Verejné funkcie
// ---------------------------------------------------------------------------
void MeterMem_Open(void) {
    count = Settings_MeterMemCount();
    if (count < METER_MEM_SLOTS_MIN || count > METER_MEM_SLOTS) count = METER_MEM_SLOTS_MIN;
    if (sel >= count) sel = 0;                          // počet sa znížil -> vyššie pamäte skryté
    ReadPage(SelPage());
    memset(last_cell, 0, sizeof(last_cell));
    last_now[0]  = '\0';
    msg_active   = false;
    save_pending = false;
    auto_pending = false;
    rep_btn      = BTN_NONE;
    LCD_Clear(LCD_BLACK);
    DrawFrame();
    last_info    = -1;
    DrawGrid();
}

bool MeterMem_Task(ButtonType_t ev, const MeterMemVal_t *now, bool hold,
                   const MeterMemVal_t *mmax, const MeterMemVal_t *mmin) {
    uint32_t t = HAL_GetTick();
    bool minmax = (mmax != NULL && mmin != NULL);
    if (last_info != (minmax ? 1 : 0)) DrawInfo(minmax);
    switch (ev) {
        case BTN_F1:   return false;
        case BTN_F2:
        case BTN_F3:   Step(ev); rep_btn = ev; rep_tick = t + REPEAT_DELAY_MS; break;
        case BTN_F4:   NextCol(); break;
        case BTN_SAVE: save_pending = true; save_tick = t; break;
        case BTN_AUTO: auto_pending = true; auto_tick = t; break;
        default: break;
    }
    // F2 / F3 podržané = opakovanie
    if (rep_btn != BTN_NONE) {
        if (BUTTON_GetHeld() != rep_btn) {
            rep_btn = BTN_NONE;
        } else if ((int32_t)(t - rep_tick) >= 0) {
            Step(rep_btn);
            rep_tick = t + REPEAT_PERIOD_MS;
        }
    }
    // SAVE: uloženie pri pustení (krátke); dlhé SAVE tu nerobí nič
    if (save_pending) {
        if (BUTTON_GetHeld() == BTN_SAVE) {
            if (t - save_tick >= Settings_LongPressMs()) save_pending = false;   // dlhé -> nič
        } else {
            save_pending = false;
            if (minmax) Store3(now, mmax, mmin);
            else        Store(now);
        }
    }
    // AUTO: dlhé = vymazať, krátke = pripomenúť podržanie
    if (auto_pending) {
        if (BUTTON_GetHeld() == BTN_AUTO) {
            if (t - auto_tick >= Settings_LongPressMs()) {
                auto_pending = false;
                Delete();
            }
        } else {
            auto_pending = false;
            MsgShow("HOLD AUTO TO DELETE", LCD_ORANGE);
        }
    }

    if (msg_active && (int32_t)(t - msg_until) >= 0) msg_active = false;
    DrawNow(now, hold, minmax);
    DrawGrid();
    return true;
}

const char *MeterMem_Title(void) {
    if (Pages() > 1u) snprintf(title, sizeof(title), "METER MEMORY %u/%u", (unsigned)(SelPage() + 1u), (unsigned)Pages());
    else              snprintf(title, sizeof(title), "METER MEMORY");
    return title;
}

const char *MeterMem_TopMsg(uint16_t *col) {
    if (!msg_active || (int32_t)(HAL_GetTick() - msg_until) >= 0) return NULL;
    if (col) *col = msg_col;
    return msg_text;
}
