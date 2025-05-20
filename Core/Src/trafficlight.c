#include "trafficlight.h"
#include "stm32g4xx_hal.h"

void TL_Init(const TrafficLight_t* tl) {
    HAL_GPIO_WritePin(tl->port,
                      tl->pin_red | tl->pin_yellow | tl->pin_green,
                      GPIO_PIN_RESET);
}

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
        case TL_OFF:
            HAL_GPIO_WritePin(tl->port,
                              tl->pin_red | tl->pin_yellow | tl->pin_green,
                              GPIO_PIN_RESET);
            break;
    }
}

void PL_Init(const PedLight_t* pl) {
    HAL_GPIO_WritePin(pl->port, pl->pin, GPIO_PIN_RESET);
}

void PL_On(const PedLight_t* pl) {
    HAL_GPIO_WritePin(pl->port, pl->pin, GPIO_PIN_SET);
}

void PL_Off(const PedLight_t* pl) {
    HAL_GPIO_WritePin(pl->port, pl->pin, GPIO_PIN_RESET);
}

void PL_Toggle(const PedLight_t* pl) {
    HAL_GPIO_TogglePin(pl->port, pl->pin);
}
