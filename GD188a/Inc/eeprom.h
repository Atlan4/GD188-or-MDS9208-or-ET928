/*
 * eeprom.h
 *
 *  I2C EEPROM 24C256 (BL24C256A) na I2C1: PB8 = SCL, PB9 = SDA, 100 kHz.
 *  Vlastný registrový ovládač – nepotrebuje HAL I2C modul.
 *  Volať iba z hlavnej slučky (blokujúce, s časovými limitmi).
 */

#ifndef EEPROM_H_
#define EEPROM_H_

#include <stdint.h>
#include <stdbool.h>
#include "eeprom_map.h"

// Inicializácia I2C1 + vyhľadanie EEPROM na adresách 0x50..0x57. true = EEPROM odpovedá.
bool EEPROM_Init(void);
bool EEPROM_IsPresent(void);

// Čítanie / zápis ľubovoľnej dĺžky (zápis sa sám delí po stránkach a čaká na dokončenie)
bool EEPROM_Read(uint16_t mem_addr, uint8_t *buf, uint16_t len);
bool EEPROM_Write(uint16_t mem_addr, const uint8_t *data, uint16_t len);

#endif /* EEPROM_H_ */
