/*
 * button.c
 *
 *  Created on: Nov 10, 2026
 *      Author: 42190
 */


// button.c
#include "button.h"


// Posledná stlačená klávesa, zapisuje EXTI, číta a nuluje BUTTON_GetEvent()
static volatile ButtonType_t button_event = BTN_NONE;

// Práve držané tlačidlo (nastaví EXTI, uvoľní BUTTON_Poll po pustení)
static volatile ButtonType_t held_btn = BTN_NONE;
static volatile uint32_t     release_tick = 0;

#define BTN_RELEASE_STABLE_MS  30   // pin musí byť v 1 aspoň toľko, aby sa tlačidlo bralo ako pustené
#define BTN_AFTER_RELEASE_MS   50   // zákmity po pustení sa ignorujú

// Vstupný pin, na ktorom je dané tlačidlo. V kľude sú OUT_K246 aj OUT_K357 v 0,
// takže držané tlačidlo drží svoj vstup v 0 bez potreby skenovať maticu.
static GPIO_PinState ButtonPin(ButtonType_t b) {
    switch (b) {
        case BTN_F1: case BTN_F2:     return HAL_GPIO_ReadPin(IN_K2_K3_GPIO_Port, IN_K2_K3_Pin);
        case BTN_F3: case BTN_F4:     return HAL_GPIO_ReadPin(IN_K4_K5_GPIO_Port, IN_K4_K5_Pin);
        case BTN_AUTO: case BTN_SAVE: return HAL_GPIO_ReadPin(IN_K6_K7_GPIO_Port, IN_K6_K7_Pin);
        case BTN_BL:                  return HAL_GPIO_ReadPin(IN_K1_GPIO_Port, IN_K1_Pin);
        default:                      return GPIO_PIN_SET;
    }
}

// Volať v každom prechode hlavnej slučky: zistí pustenie držaného tlačidla
void BUTTON_Poll(void) {
    static bool     high = false;
    static uint32_t high_since = 0;
    ButtonType_t h = held_btn;
    uint32_t now = HAL_GetTick();

    if (h == BTN_NONE) { high = false; return; }

    if (ButtonPin(h) == GPIO_PIN_SET) {
        if (!high) { high = true; high_since = now; }
        else if (now - high_since >= BTN_RELEASE_STABLE_MS) {
            release_tick = now;
            held_btn = BTN_NONE;
            high = false;
        }
    } else {
        high = false;                 // stále držané (alebo zákmit)
    }
}

ButtonType_t BUTTON_GetHeld(void) {
    return held_btn;
}

ButtonType_t BUTTON_GetEvent(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    ButtonType_t ev = button_event;
    button_event = BTN_NONE;
    __set_PRIMASK(primask);
    return ev;
}

// Inicializácia tlačidiel
void BUTTON_Init(void) {
    // V KĽUDE DRŽÍME OBA VÝSTUPY NA LOGICKEJ 0!
    // Vďaka tomu stlačenie AKÉHOKOĽVEK tlačidla okamžite vyvolá EXTI prerušenie.
    HAL_GPIO_WritePin(OUT_K246_GPIO_Port, OUT_K246_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OUT_K357_GPIO_Port, OUT_K357_Pin, GPIO_PIN_RESET);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    // 1. Čistý softvérový filter na báze ticks bez blokovania SysTicku
    static uint32_t last_exti_tick = 0;
    uint32_t current_tick = HAL_GetTick();

    // Kým je tlačidlo držané a krátko po pustení sa ďalšie hrany ignorujú (zákmity gumičky,
    // falošné hrany zo skenu matice). Prvá hrana nového stlačenia sa spracuje okamžite.
    if (held_btn != BTN_NONE ||
        (current_tick - release_tick) < BTN_AFTER_RELEASE_MS ||
        (current_tick - last_exti_tick) < BTN_RELEASE_STABLE_MS) {
        // Okamžite čistíme Pending príznak hardvéru, aby sa nevytváral rad falošných prerušení
        EXTI->PR = GPIO_Pin;
        return;
    }

    ButtonType_t detected_btn = BTN_NONE;

    // --- SAMOTNÝ BLESKOVÝ HARDVÉROVÝ SKEN MATICE ---
    if (GPIO_Pin == IN_K1_Pin) {
        detected_btn = BTN_BL;
    }
    else {
        HAL_GPIO_WritePin(OUT_K246_GPIO_Port, OUT_K246_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(OUT_K357_GPIO_Port, OUT_K357_Pin, GPIO_PIN_SET);
        for(volatile int i=0; i<50; i++); // Krátka pauza v inštrukciách procesora (bezpečné nanosekundy)

        if (HAL_GPIO_ReadPin(IN_K2_K3_GPIO_Port, IN_K2_K3_Pin) == GPIO_PIN_RESET) detected_btn = BTN_F1;
        if (HAL_GPIO_ReadPin(IN_K4_K5_GPIO_Port, IN_K4_K5_Pin) == GPIO_PIN_RESET) detected_btn = BTN_F3;
        if (HAL_GPIO_ReadPin(IN_K6_K7_GPIO_Port, IN_K6_K7_Pin) == GPIO_PIN_RESET) detected_btn = BTN_AUTO;

        if (detected_btn == BTN_NONE) {
            HAL_GPIO_WritePin(OUT_K246_GPIO_Port, OUT_K246_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(OUT_K357_GPIO_Port, OUT_K357_Pin, GPIO_PIN_RESET);
            for(volatile int i=0; i<50; i++);

            if (HAL_GPIO_ReadPin(IN_K2_K3_GPIO_Port, IN_K2_K3_Pin) == GPIO_PIN_RESET) detected_btn = BTN_F2;
            if (HAL_GPIO_ReadPin(IN_K4_K5_GPIO_Port, IN_K4_K5_Pin) == GPIO_PIN_RESET) detected_btn = BTN_F4;
            if (HAL_GPIO_ReadPin(IN_K6_K7_GPIO_Port, IN_K6_K7_Pin) == GPIO_PIN_RESET) detected_btn = BTN_SAVE;
        }
    }

    // --- ODOVZDANIE UDALOSTI HLAVNEJ SLUČKE ---
    // V prerušení sa nič nekreslí ani neposiela – LCD a UART obsluhuje iba main.
    // Blokovanie sa spustí až keď sa tlačidlo naozaj našlo – ak sken trafil zákmit
    // a nič nenašiel, ďalšia hrana toho istého stlačenia sa skúsi znova.
    if (detected_btn != BTN_NONE) {
        button_event = detected_btn;
        held_btn = detected_btn;
        last_exti_tick = current_tick;
    }

    // --- RE-INICIALIZÁCIA PRE ĎALŠIE BUDENIE ---
    // Vraciame budiace linky matice trvalo na logickú 0
    HAL_GPIO_WritePin(OUT_K246_GPIO_Port, OUT_K246_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OUT_K357_GPIO_Port, OUT_K357_Pin, GPIO_PIN_RESET);

    // [OPRAVA]: ZMAZANÝ NEBEZPEČNÝ HAL_DELAY
    // Okamžite čistíme hardvérový register čakajúcich prerušení (Pending register)
    EXTI->PR = GPIO_Pin;
}
