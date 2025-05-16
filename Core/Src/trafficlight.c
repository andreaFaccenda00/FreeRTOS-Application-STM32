// trafficlight.c
#include "trafficlight.h"
#include "stm32g4xx_hal.h"  // o stm32g4xx_hal_gpio.h

/**
 * @brief  Inizializza i pin di un semaforo veicolare (tutti OFF)
 */
void TL_Init(const TrafficLight_t* tl) {
    HAL_GPIO_WritePin(tl->port,
                      tl->pin_red | tl->pin_yellow | tl->pin_green,
                      GPIO_PIN_RESET);
}

/**
 * @brief  Imposta lo stato di un semaforo veicolare
 */
void TL_SetState(const TrafficLight_t* tl, TL_State_t state) {
    switch (state) {
        case TL_RED:
            HAL_GPIO_WritePin(tl->port, tl->pin_red,    GPIO_PIN_SET);
            HAL_GPIO_WritePin(tl->port, tl->pin_yellow, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(tl->port, tl->pin_green,  GPIO_PIN_RESET);
            break;
        case TL_YELLOW:
            HAL_GPIO_WritePin(tl->port, tl->pin_red,    GPIO_PIN_RESET);
            HAL_GPIO_WritePin(tl->port, tl->pin_yellow, GPIO_PIN_SET);
            HAL_GPIO_WritePin(tl->port, tl->pin_green,  GPIO_PIN_RESET);
            break;
        case TL_GREEN:
            HAL_GPIO_WritePin(tl->port, tl->pin_red,    GPIO_PIN_RESET);
            HAL_GPIO_WritePin(tl->port, tl->pin_yellow, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(tl->port, tl->pin_green,  GPIO_PIN_SET);
            break;
        case TL_OFF:  // spegne tutte le luci
            HAL_GPIO_WritePin(tl->port,
                              tl->pin_red | tl->pin_yellow | tl->pin_green,
                              GPIO_PIN_RESET);
            break;
    }
}

/**
 * @brief  Inizializza il LED pedonale (OFF)
 */
void PL_Init(const PedLight_t* pl) {
    HAL_GPIO_WritePin(pl->port, pl->pin, GPIO_PIN_RESET);
}

/**
 * @brief  Accende il LED pedonale
 */
void PL_On(const PedLight_t* pl) {
    HAL_GPIO_WritePin(pl->port, pl->pin, GPIO_PIN_SET);
}

/**
 * @brief  Spegne il LED pedonale
 */
void PL_Off(const PedLight_t* pl) {
    HAL_GPIO_WritePin(pl->port, pl->pin, GPIO_PIN_RESET);
}

/**
 * @brief  Toggle del LED pedonale
 */
void PL_Toggle(const PedLight_t* pl) {
    HAL_GPIO_TogglePin(pl->port, pl->pin);
}
