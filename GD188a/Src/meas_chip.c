/*
 * meas_chip.c
 *
 *  Driver pre merací čip (klon DTM0660) na USART6.
 *
 *  Pravidlá:
 *   - MeasChip_ProcessByte() beží v RX prerušení: iba skladá pakety a mení stav,
 *     NIKDY nevysiela (vysielanie z ISR blokovalo príjem a kolidovalo s main).
 *   - Všetko vysielanie robí MeasChip_Task() / SetMode() / SetRange() z hlavnej slučky.
 *   - Nameraná hodnota sa odovzdáva cez MeasChip_GetSample() – atomická kópia,
 *     takže hodnota a rozsah sú vždy z toho istého paketu.
 */

#include <string.h>

#include "meas_chip.h"
#include "rotary_switch.h"

// --- Časovanie (ms) ---
#define MEAS_POLL_PERIOD_MS     200   // perióda dotazu 60/61/62 (ako pôvodný FW)
#define MEAS_CMD_TIMEOUT_MS     500   // čakanie na echo / odpoveď príkazu
#define MEAS_DATA_TIMEOUT_MS    2000  // v READY: ak tak dlho nepríde dátový paket -> re-init
#define MEAS_INTERBYTE_GAP_MS   5     // medzera v pakete väčšia ako toto = začiatok nového paketu
#define MEAS_MAX_RETRIES        3     // po toľkých neúspechoch sa nastaví comm_error
#define MEAS_MAX_MAN_STEPS      8     // max. počet 5B pri obnove manuálneho rozsahu

MeasChip_t MeasChip;

// --- Stav parsera (iba ISR) ---
static uint8_t  rx_buffer[8];
static uint8_t  rx_index = 0;
static uint32_t last_byte_tick = 0;

// --- Odovzdávanie vzorky ISR -> main ---
static volatile MeasSample_t latest_sample;
static volatile bool         sample_ready = false;
static volatile uint32_t     last_data_tick = 0;
static volatile bool         chip_rebooted = false;

// --- Stav hlavnej slučky ---
static uint32_t last_poll_tick = 0;


// Kontrolný súčet = súčet bajtov mod 256
static uint8_t CalculateChecksum(const uint8_t *data, uint8_t length) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return sum;
}

// Odoslanie paketu s automaticky dopočítaným CS (len z hlavnej slučky!)
static void SendPacket(const uint8_t *payload, uint8_t payload_len) {
    uint8_t tx_buf[8];
    if (payload_len > 7) return;

    memcpy(tx_buf, payload, payload_len);
    tx_buf[payload_len] = CalculateChecksum(payload, payload_len);

    HAL_UART_Transmit(&huart6, tx_buf, payload_len + 1, 50);
}

static void SendModeCmd(void) {
    uint8_t payload[2] = { 0x58, MeasChip.mode_cmd };
    MeasChip.last_cmd_tick = HAL_GetTick();
    SendPacket(payload, sizeof(payload));
}

static void SendSingle(uint8_t cmd) {
    MeasChip.last_cmd_tick = HAL_GetTick();
    SendPacket(&cmd, 1);
}

// Zaznamená neúspešný pokus; po MEAS_MAX_RETRIES nahlási chybu komunikácie
static void CountRetry(void) {
    if (MeasChip.retries < 255) MeasChip.retries++;
    if (MeasChip.retries >= MEAS_MAX_RETRIES) MeasChip.comm_error = true;
}


void MeasChip_Init(void) {
    memset((void *)&MeasChip, 0, sizeof(MeasChip));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    rx_index = 0;
    sample_ready = false;
    chip_rebooted = false;
    MeasChip.init_state = INIT_STEP_IDLE;
    MeasChip.query = MEAS_QUERY_MAIN;
    MeasChip.manual_target = MEAS_RANGE_AUTO;
}

void MeasChip_ResetParser(void) {
    rx_index = 0;
}

// Nastavenie meracieho módu (58 mm) -> 5A -> prípadne obnova manuálneho rozsahu (5B ...).
// query = ktorý dotaz sa potom periodicky posiela.
void MeasChip_SetMode(uint8_t mode_cmd, uint8_t query, uint8_t manual_target) {
    if (mode_cmd == CMD_CHIP_UNKNOWN) return;

    MeasChip.mode_cmd      = mode_cmd;
    MeasChip.query         = query;
    MeasChip.manual_target = manual_target;
    MeasChip.restoring     = false;
    MeasChip.retries       = 0;
    MeasChip.init_state = INIT_STEP_SET_MODE;
    SendModeCmd();
}

// Prepnutie AUTO (5A) / MANUAL (5B) rozsahu. Volať iba v stave READY.
void MeasChip_SetRange(uint8_t range_cmd) {
    MeasChip.restoring = false;
    if (range_cmd == CMD_CHIP_AUTO) {
        MeasChip.manual_target = MEAS_RANGE_AUTO;
        MeasChip.init_state = INIT_STEP_SET_AUTO;
    } else if (range_cmd == CMD_CHIP_MANUAL) {
        MeasChip.init_state = INIT_STEP_SET_MAN;
    } else {
        return;
    }
    MeasChip.retries = 0;
    SendSingle(range_cmd);
}

// Stavový automat – volať v každom prechode hlavnej slučky
void MeasChip_Task(void) {
    uint32_t now = HAL_GetTick();

    // Čip sa sám reštartoval (prišlo A0 01 A1 počas behu) -> znovu nastavíme mód
    if (chip_rebooted) {
        chip_rebooted = false;
        if (MeasChip.mode_cmd != CMD_CHIP_UNKNOWN) {
            MeasChip_SetMode(MeasChip.mode_cmd, MeasChip.query, MeasChip.manual_target);
            return;
        }
    }

    switch (MeasChip.init_state) {
        case INIT_STEP_SET_MODE:
            // Echo 58 neprišlo -> zopakujeme
            if (now - MeasChip.last_cmd_tick > MEAS_CMD_TIMEOUT_MS) {
                CountRetry();
                SendModeCmd();
            }
            break;

        case INIT_STEP_SEND_AUTO:
            // Parser potvrdil echo 58 -> tu (mimo ISR) pošleme 5A
            MeasChip.init_state = INIT_STEP_SET_AUTO;
            SendSingle(CMD_CHIP_AUTO);
            break;

        case INIT_STEP_SET_AUTO:
            if (now - MeasChip.last_cmd_tick > MEAS_CMD_TIMEOUT_MS) {
                CountRetry();
                SendSingle(CMD_CHIP_AUTO);
            }
            break;

        case INIT_STEP_SEND_MAN:
            // Obnova manuálneho rozsahu: pošleme ďalšie 5B a počkáme na odpoveď s rozsahom
            MeasChip.man_steps++;
            MeasChip.init_state = INIT_STEP_SET_MAN;
            SendSingle(CMD_CHIP_MANUAL);
            break;

        case INIT_STEP_SET_MAN:
            if (now - MeasChip.last_cmd_tick > MEAS_CMD_TIMEOUT_MS) {
                CountRetry();
                if (MeasChip.restoring && MeasChip.man_steps < MEAS_MAX_MAN_STEPS) {
                    // Pri obnove sa porovnáva odpoveď s cieľom, takže ďalšie 5B je bezpečné
                    MeasChip.init_state = INIT_STEP_SEND_MAN;
                } else {
                    // Stlačenie RNG užívateľom: 5B NEopakujeme (posunulo by rozsah dvakrát).
                    // Skutočný rozsah prinesie najbližší dátový paket.
                    MeasChip.restoring  = false;
                    MeasChip.init_state = INIT_STEP_READY;
                }
            }
            break;

        case INIT_STEP_READY:
            if (now - last_poll_tick >= MEAS_POLL_PERIOD_MS) {
                last_poll_tick = now;
                SendSingle(MeasChip.query);
            }
            // Dlho žiadne dáta -> čip asi stratil nastavenie, inicializujeme znova
            if (now - last_data_tick > MEAS_DATA_TIMEOUT_MS) {
                MeasChip.comm_error = true;
                last_data_tick = now;
                MeasChip_SetMode(MeasChip.mode_cmd, MeasChip.query, MeasChip.manual_target);
            }
            break;

        case INIT_STEP_IDLE:
        default:
            break;
    }
}

// Atomická kópia poslednej vzorky. Vráti true, ak je nová.
bool MeasChip_GetSample(MeasSample_t *out) {
    bool fresh = false;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (sample_ready) {
        out->packet_type = latest_sample.packet_type;
        out->value       = latest_sample.value;
        out->range       = latest_sample.range;
        out->overload    = latest_sample.overload;
        out->freq_mhz    = latest_sample.freq_mhz;
        for (int i = 0; i < 5; i++) out->raw[i] = latest_sample.raw[i];
        sample_ready = false;
        fresh = true;
    }
    __set_PRIMASK(primask);
    return fresh;
}

// Uloží vzorku z ISR
static void StoreSample(uint8_t type, int32_t value, uint8_t range, bool overload) {
    latest_sample.packet_type = type;
    latest_sample.value       = value;
    latest_sample.range       = range;
    latest_sample.overload    = overload;
    sample_ready   = true;
    last_data_tick = HAL_GetTick();
    MeasChip.range = range;
    MeasChip.comm_error = false;
}

// Očakávaná dĺžka paketu podľa hlavičky (0 = neznáma hlavička)
static uint8_t PacketLength(uint8_t header) {
    switch (header) {
        case 0xA0: return 3;   // A0 01 CS
        case 0x58: return 3;   // 58 mm CS
        case 0x5A: return 3;   // 5A rr CS
        case 0x5B: return 3;   // 5B rr CS
        case 0x60: return 6;   // 60 v0 v1 v2 rr CS
        case 0x61: return 7;   // 61 v0 v1 v2 v3 v4 CS (v4 = najvyšší bajt 40-bitovej hodnoty)
        case 0x62: return 4;   // 62 v0 v1 CS
        default:   return 0;
    }
}

// Parser – volá sa z RX prerušenia pre každý prijatý bajt
void MeasChip_ProcessByte(uint8_t byte) {
    uint32_t now = HAL_GetTick();

    // Medzera uprostred paketu -> zvyšok starého paketu zahodíme a synchronizujeme znova
    if (rx_index != 0 && (now - last_byte_tick) > MEAS_INTERBYTE_GAP_MS) {
        rx_index = 0;
    }
    last_byte_tick = now;

    // Prvý bajt musí byť známa hlavička
    if (rx_index == 0 && PacketLength(byte) == 0) {
        return;
    }

    rx_buffer[rx_index++] = byte;

    uint8_t expected = PacketLength(rx_buffer[0]);
    if (rx_index < expected) {
        return;                       // paket ešte nie je kompletný
    }
    rx_index = 0;                     // paket kompletný, ďalší bajt začína nový

    if (rx_buffer[expected - 1] != CalculateChecksum(rx_buffer, expected - 1)) {
        return;                       // zlý CS -> zahodiť
    }

    switch (rx_buffer[0]) {
        case 0xA0:                    // uvítací paket čipu
            if (MeasChip.chip_present) {
                chip_rebooted = true; // čip sa reštartoval počas behu
            }
            MeasChip.chip_present = true;
            break;

        case 0x58:                    // echo nastavenia módu
            if (MeasChip.init_state == INIT_STEP_SET_MODE && rx_buffer[1] == MeasChip.mode_cmd) {
                MeasChip.init_state = INIT_STEP_SEND_AUTO;
            }
            break;

        case 0x5A:                    // potvrdenie AUTO, rx_buffer[1] = aktuálny rozsah
            if (MeasChip.init_state == INIT_STEP_SET_AUTO) {
                MeasChip.range      = rx_buffer[1];
                MeasChip.retries    = 0;
                MeasChip.comm_error = false;
                last_data_tick      = now;
                if (MeasChip.manual_target != MEAS_RANGE_AUTO) {
                    // Užívateľ mal v tomto móde manuálny rozsah -> obnovíme ho krokovaním 5B
                    MeasChip.restoring  = true;
                    MeasChip.man_steps  = 0;
                    MeasChip.init_state = INIT_STEP_SEND_MAN;
                } else {
                    MeasChip.init_state = INIT_STEP_READY;
                }
            }
            break;

        case 0x5B:                    // potvrdenie MANUAL, rx_buffer[1] = nový rozsah
            if (MeasChip.init_state == INIT_STEP_SET_MAN) {
                uint8_t r = rx_buffer[1];
                MeasChip.range   = r;
                MeasChip.retries = 0;
                last_data_tick   = now;
                if (MeasChip.restoring && r != MeasChip.manual_target &&
                    MeasChip.man_steps < MEAS_MAX_MAN_STEPS) {
                    MeasChip.init_state = INIT_STEP_SEND_MAN;   // ešte nie sme na cieli
                } else {
                    MeasChip.restoring     = false;
                    MeasChip.manual_target = r;                 // tento rozsah teraz platí
                    MeasChip.init_state    = INIT_STEP_READY;
                }
            }
            break;

        case 0x60: {                  // hlavné meranie: 24-bit so znamienkom, little endian
            uint32_t raw = (uint32_t)rx_buffer[1] |
                           ((uint32_t)rx_buffer[2] << 8) |
                           ((uint32_t)rx_buffer[3] << 16);
            bool ol = (raw == MEAS_OL_VALUE);
            int32_t value = (raw & 0x800000UL) ? (int32_t)(raw | 0xFF000000UL) : (int32_t)raw;
            StoreSample(0x60, value, rx_buffer[4], ol);
            break;
        }

        case 0x61: {                  // frekvencia: 40-bit bez znamienka v mHz, little endian
            // Do v0.51 sa 5. bajt bral ako rozsah (pri 50 Hz vždy 00). 20 MHz = 2*10^10 mHz
            // sa do 32 bitov nezmestí (max. 4,29 MHz) -> 5. bajt je najvyšší bajt hodnoty.
            uint64_t f = 0;
            for (int i = 5; i >= 1; i--) f = (f << 8) | rx_buffer[i];
            latest_sample.freq_mhz = f;
            for (int i = 0; i < 5; i++) latest_sample.raw[i] = rx_buffer[1 + i];
            StoreSample(0x61, (int32_t)(f > 0x7FFFFFFFu ? 0x7FFFFFFF : f), 0, false);
            break;
        }

        case 0x62: {                  // strieda: 16-bit bez znamienka
            uint32_t raw = (uint32_t)rx_buffer[1] | ((uint32_t)rx_buffer[2] << 8);
            StoreSample(0x62, (int32_t)raw, 0, false);
            break;
        }

        default:
            break;
    }
}
