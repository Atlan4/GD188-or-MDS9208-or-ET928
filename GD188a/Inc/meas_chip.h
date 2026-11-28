/*
 * meas_chip.h
 *
 *  Driver pre merací čip (klon DTM0660) na USART6, 9600 8N1.
 *
 *  Protokol (overený sigrok záznamami, D1 = MCU -> čip, D0 = čip -> MCU):
 *    Štart čipu:       čip pošle  A0 01 A1
 *    Nastavenie módu:  MCU 58 mm CS        -> čip echo 58 mm CS
 *    Auto rozsah:      MCU 5A 5A           -> čip 5A rr CS   (rr = aktuálny rozsah)
 *    Manuálny rozsah:  MCU 5B 5B           -> čip 5B rr CS   (rr = nový rozsah; opakované 5B
 *                                                              posúva rozsah dokola)
 *    Dotaz meranie:    MCU 60 60           -> čip 60 v0 v1 v2 rr CS   (v = 24-bit so znamienkom, LE)
 *    Dotaz frekvencia: MCU 61 61           -> čip 61 v0 v1 v2 v3 rr CS (v = 32-bit bez znamienka)
 *    Dotaz strieda:    MCU 62 62           -> čip 62 v0 v1 CS          (v = 16-bit bez znamienka)
 *    Pretečenie (OL):  v0 v1 v2 = FF FF 7F
 *    CS = súčet všetkých predchádzajúcich bajtov (mod 256)
 */

#ifndef MEAS_CHIP_H
#define MEAS_CHIP_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

extern UART_HandleTypeDef huart6;

// Dotazové príkazy (ktorý typ dát chceme čítať)
#define MEAS_QUERY_MAIN   0x60   // napätie, prúd, odpor, dióda, kapacita
#define MEAS_QUERY_FREQ   0x61   // frekvencia
#define MEAS_QUERY_DUTY   0x62   // strieda

// Hodnota 24-bit, ktorou čip hlási pretečenie rozsahu
#define MEAS_OL_VALUE     0x7FFFFF

typedef enum {
    INIT_STEP_IDLE = 0,
    INIT_STEP_SET_MODE,   // Poslané 58 mm, čaká sa na echo
    INIT_STEP_SEND_AUTO,  // Echo prišlo, hlavná slučka má poslať 5A (parser v ISR nevysiela)
    INIT_STEP_SET_AUTO,   // Poslané 5A, čaká sa na odpoveď 5A rr
    INIT_STEP_SEND_MAN,   // Obnova uloženého manuálneho rozsahu: hlavná slučka má poslať 5B
    INIT_STEP_SET_MAN,    // Poslané 5B, čaká sa na odpoveď 5B rr
    INIT_STEP_READY       // Hotovo, periodicky sa posiela dotaz 60/61/62
} InitState_t;

// Interný stav ovládača – zapisuje ho ISR aj hlavná slučka, preto volatile
typedef struct {
    volatile InitState_t init_state;
    volatile bool     chip_present;   // prišlo A0 01 A1
    volatile bool     comm_error;     // čip neodpovedá ani po opakovaniach
    volatile uint32_t last_cmd_tick;
    volatile uint8_t  mode_cmd;       // posledný požadovaný mód (pre opakovanie)
    volatile uint8_t  query;          // 0x60 / 0x61 / 0x62
    volatile uint8_t  range;          // posledný známy rozsah
    volatile uint8_t  retries;
    volatile uint8_t  manual_target;  // 0xFF = AUTO, inak manuálny rozsah, ktorý sa má nastaviť
    volatile bool     restoring;      // práve sa krokuje 5B na manual_target
    volatile uint8_t  man_steps;      // počet 5B pri obnove (poistka proti zacykleniu)
} MeasChip_t;

#define MEAS_RANGE_AUTO   0xFF

// Jedna nameraná vzorka – kópia urobená atomicky pre hlavnú slučku
typedef struct {
    uint8_t  packet_type;  // 0x60 / 0x61 / 0x62
    int32_t  value;
    uint8_t  range;
    bool     overload;
    uint64_t freq_mhz;     // 0x61: 40-bitová frekvencia v mHz (bajty v0..v4, little-endian)
    uint8_t  raw[5];       // 0x61: dátové bajty paketu (na overenie výkladu)
} MeasSample_t;

extern MeasChip_t MeasChip;

void MeasChip_Init(void);
void MeasChip_ProcessByte(uint8_t byte);      // volá sa IBA z RX prerušenia
void MeasChip_ResetParser(void);              // volá sa z RX error callbacku

// Nasledujúce funkcie volať IBA z hlavnej slučky
// manual_target: MEAS_RANGE_AUTO alebo číslo rozsahu, ktorý sa po nastavení módu obnoví
void MeasChip_SetMode(uint8_t mode_cmd, uint8_t query, uint8_t manual_target);
void MeasChip_SetRange(uint8_t range_cmd);    // CMD_CHIP_AUTO alebo CMD_CHIP_MANUAL
void MeasChip_Task(void);                     // stavový automat, timeouty, periodický dotaz
bool MeasChip_GetSample(MeasSample_t *out);   // true = prišla nová vzorka

#endif // MEAS_CHIP_H
