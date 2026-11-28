/*
 * eeprom.c
 *
 *  Registrový I2C master (STM32F4 I2C v1) pre EEPROM 24C256.
 *  Postupy čítania (N = 1, 2, >2) podľa referenčného manuálu RM0401, kap. I2C master receiver.
 *
 *  Pozn.: ak sa v CubeMX vygeneruje aj HAL I2C (MX_I2C1_Init), nevadí to –
 *  EEPROM_Init si I2C1 nastaví sám znova rovnako.
 */

#include "eeprom.h"
#include "stm32f4xx_hal.h"

#define EE_I2C              I2C1
#define EE_BUS_HZ           100000u
#define EE_TIMEOUT_MS       10u     // limit na jeden krok prenosu
#define EE_WRITE_CYCLE_MS   10u     // max. čas interného zápisu EEPROM (typ. 5 ms)

#define SCL_PIN             GPIO_PIN_8
#define SDA_PIN             GPIO_PIN_9

static uint8_t dev_addr = 0xA0;     // 8-bitová adresa (0x50 << 1), nájde sa pri Init
static bool    present  = false;

// ---------------------------------------------------------------------------
// Pomocné funkcie
// ---------------------------------------------------------------------------

static void DelayUs(uint32_t us) {
    // hrubé čakanie pri 100 MHz (stačí na obnovu zbernice)
    for (volatile uint32_t i = 0; i < us * 25u; i++) { }
}

// Čaká na príznak v SR1. Pri NACK (AF) alebo vypršaní času vráti false.
static bool WaitSR1(uint32_t mask) {
    uint32_t t0 = HAL_GetTick();
    while ((EE_I2C->SR1 & mask) == 0) {
        if (EE_I2C->SR1 & I2C_SR1_AF) return false;
        if (HAL_GetTick() - t0 > EE_TIMEOUT_MS) return false;
    }
    return true;
}

static void Stop(void) {
    EE_I2C->CR1 |= I2C_CR1_STOP;
}

static void ClearAddr(void) {
    (void)EE_I2C->SR1;
    (void)EE_I2C->SR2;
}

// Ukončenie prenosu po chybe
static void Abort(void) {
    EE_I2C->SR1 &= ~I2C_SR1_AF;
    Stop();
    EE_I2C->CR1 &= ~I2C_CR1_POS;
    EE_I2C->CR1 |= I2C_CR1_ACK;
}

// Ak niektorá EEPROM drží SDA v 0 (napr. reset MCU uprostred čítania), 9 taktov SCL ju uvoľní
static void BusRecover(void) {
    GPIO_InitTypeDef gi = {0};
    gi.Pin   = SCL_PIN | SDA_PIN;
    gi.Mode  = GPIO_MODE_OUTPUT_OD;
    gi.Pull  = GPIO_PULLUP;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_WritePin(GPIOB, SCL_PIN | SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_Init(GPIOB, &gi);
    DelayUs(10);

    for (int i = 0; i < 9 && HAL_GPIO_ReadPin(GPIOB, SDA_PIN) == GPIO_PIN_RESET; i++) {
        HAL_GPIO_WritePin(GPIOB, SCL_PIN, GPIO_PIN_RESET); DelayUs(5);
        HAL_GPIO_WritePin(GPIOB, SCL_PIN, GPIO_PIN_SET);   DelayUs(5);
    }
    // STOP: SDA 0 -> 1 pri SCL = 1
    HAL_GPIO_WritePin(GPIOB, SDA_PIN, GPIO_PIN_RESET); DelayUs(5);
    HAL_GPIO_WritePin(GPIOB, SCL_PIN, GPIO_PIN_SET);   DelayUs(5);
    HAL_GPIO_WritePin(GPIOB, SDA_PIN, GPIO_PIN_SET);   DelayUs(5);
}

static void PeriphInit(void) {
    GPIO_InitTypeDef gi = {0};
    gi.Pin       = SCL_PIN | SDA_PIN;
    gi.Mode      = GPIO_MODE_AF_OD;
    gi.Pull      = GPIO_PULLUP;
    gi.Speed     = GPIO_SPEED_FREQ_HIGH;
    gi.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gi);

    EE_I2C->CR1 = I2C_CR1_SWRST;            // čistý reset periférie
    EE_I2C->CR1 = 0;

    uint32_t pclk = HAL_RCC_GetPCLK1Freq(); // 50 MHz v tomto projekte (APB1 = HCLK / 2)
    uint32_t mhz  = pclk / 1000000u;
    if (mhz < 2u) mhz = 2u;
    uint32_t ccr  = pclk / (2u * EE_BUS_HZ);
    if (ccr < 4u) ccr = 4u;

    EE_I2C->CR2   = mhz;                    // FREQ
    EE_I2C->CCR   = ccr;                    // štandardný režim 100 kHz
    EE_I2C->TRISE = mhz + 1u;               // max. 1000 ns
    EE_I2C->CR1   = I2C_CR1_PE | I2C_CR1_ACK;
}

// Zbernica obsadená (napr. po chybe) -> reset periférie a obnova
static bool EnsureIdle(void) {
    uint32_t t0 = HAL_GetTick();
    while (EE_I2C->SR2 & I2C_SR2_BUSY) {
        if (HAL_GetTick() - t0 > EE_TIMEOUT_MS) {
            BusRecover();
            PeriphInit();
            return (EE_I2C->SR2 & I2C_SR2_BUSY) == 0;
        }
    }
    return true;
}

static bool Start(void) {
    EE_I2C->CR1 &= ~I2C_CR1_POS;
    EE_I2C->CR1 |= I2C_CR1_START;
    return WaitSR1(I2C_SR1_SB);
}

// Pošle adresu; false pri NACK (zariadenie neodpovedá alebo ešte zapisuje)
static bool SendAddr(uint8_t addr8) {
    EE_I2C->DR = addr8;
    if (!WaitSR1(I2C_SR1_ADDR)) {
        Abort();
        return false;
    }
    return true;
}

static bool SendByte(uint8_t b) {
    if (!WaitSR1(I2C_SR1_TXE)) { Abort(); return false; }
    EE_I2C->DR = b;
    return true;
}

static bool Probe(uint8_t addr8) {
    if (!EnsureIdle() || !Start()) { Abort(); return false; }
    if (!SendAddr(addr8)) return false;
    ClearAddr();
    Stop();
    return true;
}

// Čakanie na koniec interného zápisu: EEPROM na svoju adresu neodpovedá, kým zapisuje
static bool WaitWriteDone(void) {
    uint32_t t0 = HAL_GetTick();
    do {
        if (Start() && SendAddr(dev_addr)) {
            ClearAddr();
            Stop();
            return true;
        }
    } while (HAL_GetTick() - t0 <= EE_WRITE_CYCLE_MS);
    return false;
}

// ---------------------------------------------------------------------------
// Verejné funkcie
// ---------------------------------------------------------------------------

bool EEPROM_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    BusRecover();
    PeriphInit();

    // Adresa podľa zapojenia A0..A2 nie je istá -> vyskúšať 0x50..0x57
    present = false;
    for (uint8_t a = 0x50; a <= 0x57; a++) {
        if (Probe((uint8_t)(a << 1))) {
            dev_addr = (uint8_t)(a << 1);
            present  = true;
            break;
        }
    }
    return present;
}

bool EEPROM_IsPresent(void) {
    return present;
}

bool EEPROM_Read(uint16_t mem_addr, uint8_t *buf, uint16_t len) {
    if (!present || len == 0 || (uint32_t)mem_addr + len > EE_SIZE) return false;
    if (!EnsureIdle()) return false;

    // Fáza zápisu adresy v pamäti
    if (!Start())                          { Abort(); return false; }
    if (!SendAddr(dev_addr))               return false;
    ClearAddr();
    if (!SendByte((uint8_t)(mem_addr >> 8)))   return false;
    if (!SendByte((uint8_t)(mem_addr & 0xFF))) return false;
    if (!WaitSR1(I2C_SR1_BTF))             { Abort(); return false; }

    // Opakovaný START a čítanie
    if (!Start())                          { Abort(); return false; }

    if (len == 1) {
        EE_I2C->DR = dev_addr | 1u;
        if (!WaitSR1(I2C_SR1_ADDR))        { Abort(); return false; }
        EE_I2C->CR1 &= ~I2C_CR1_ACK;
        __disable_irq();                   // ClearAddr + STOP musia ísť hneď za sebou
        ClearAddr();
        Stop();
        __enable_irq();
        if (!WaitSR1(I2C_SR1_RXNE))        { Abort(); return false; }
        buf[0] = (uint8_t)EE_I2C->DR;
    }
    else if (len == 2) {
        EE_I2C->DR = dev_addr | 1u;
        if (!WaitSR1(I2C_SR1_ADDR))        { Abort(); return false; }
        EE_I2C->CR1 &= ~I2C_CR1_ACK;
        EE_I2C->CR1 |= I2C_CR1_POS;
        ClearAddr();
        if (!WaitSR1(I2C_SR1_BTF))         { Abort(); return false; }
        Stop();
        buf[0] = (uint8_t)EE_I2C->DR;
        buf[1] = (uint8_t)EE_I2C->DR;
    }
    else {
        EE_I2C->CR1 |= I2C_CR1_ACK;
        EE_I2C->DR = dev_addr | 1u;
        if (!WaitSR1(I2C_SR1_ADDR))        { Abort(); return false; }
        ClearAddr();

        uint16_t i = 0;
        while (len - i > 3) {
            if (!WaitSR1(I2C_SR1_RXNE))    { Abort(); return false; }
            buf[i++] = (uint8_t)EE_I2C->DR;
        }
        // Zostávajú 3 bajty: N-2 v DR, N-1 v posuvnom registri (SCL podržaný)
        if (!WaitSR1(I2C_SR1_BTF))         { Abort(); return false; }
        EE_I2C->CR1 &= ~I2C_CR1_ACK;
        buf[i++] = (uint8_t)EE_I2C->DR;     // N-2
        if (!WaitSR1(I2C_SR1_BTF))         { Abort(); return false; }
        Stop();
        buf[i++] = (uint8_t)EE_I2C->DR;     // N-1
        if (!WaitSR1(I2C_SR1_RXNE))        { Abort(); return false; }
        buf[i++] = (uint8_t)EE_I2C->DR;     // N
    }

    EE_I2C->CR1 &= ~I2C_CR1_POS;
    EE_I2C->CR1 |= I2C_CR1_ACK;
    return true;
}

bool EEPROM_Write(uint16_t mem_addr, const uint8_t *data, uint16_t len) {
    if (!present || (uint32_t)mem_addr + len > EE_SIZE) return false;

    while (len > 0) {
        // Zápis nesmie prekročiť hranicu stránky (inak by sa adresa v stránke pretočila)
        uint16_t chunk = (uint16_t)(EE_PAGE_SIZE - (mem_addr % EE_PAGE_SIZE));
        if (chunk > len) chunk = len;

        if (!EnsureIdle())                 return false;
        if (!Start())                      { Abort(); return false; }
        if (!SendAddr(dev_addr))           return false;
        ClearAddr();
        if (!SendByte((uint8_t)(mem_addr >> 8)))   return false;
        if (!SendByte((uint8_t)(mem_addr & 0xFF))) return false;
        for (uint16_t k = 0; k < chunk; k++) {
            if (!SendByte(data[k]))        return false;
        }
        if (!WaitSR1(I2C_SR1_BTF))         { Abort(); return false; }
        Stop();

        if (!WaitWriteDone())              return false;

        mem_addr += chunk;
        data     += chunk;
        len      -= chunk;
    }
    return true;
}
