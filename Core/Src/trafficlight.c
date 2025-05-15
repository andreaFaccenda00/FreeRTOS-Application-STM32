#include "trafficlight.h"
#include "stm32g4xx_hal.h"  // Assicurati che corrisponda alla tua serie STM32

/**
 * @brief  Inizializza i pin di un semaforo veicolare (rossi, gialli, verdi) in OFF
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
    HAL_GPIO_WritePin(tl->port, tl->pin_red,    (state == TL_RED)    ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(tl->port, tl->pin_yellow, (state == TL_YELLOW) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(tl->port, tl->pin_green,  (state == TL_GREEN)  ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief  Inizializza il LED pedonale in OFF
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
