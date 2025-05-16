#ifndef PINOUT_H
#define PINOUT_H

#include "stm32g4xx_hal.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} GPIO_Pin_t;

/* South GPIO (CN8) */
static const GPIO_Pin_t TL_SOUTH_RED    = { GPIOA, GPIO_PIN_0 };
static const GPIO_Pin_t TL_SOUTH_YELLOW = { GPIOA, GPIO_PIN_1 };
static const GPIO_Pin_t TL_SOUTH_GREEN  = { GPIOA, GPIO_PIN_4 };

/* Nord GPIO (CN7) */
static const GPIO_Pin_t TL_NORTH_RED    = { GPIOC, GPIO_PIN_10 }; // CN7 Pin 1
static const GPIO_Pin_t TL_NORTH_YELLOW = { GPIOC, GPIO_PIN_11 }; // CN7 Pin 2
static const GPIO_Pin_t TL_NORTH_GREEN  = { GPIOC, GPIO_PIN_12 }; // CN7 Pin 3

/* East GPIO (CN10) */
static const GPIO_Pin_t TL_EAST_RED     = { GPIOC, GPIO_PIN_9 };  // CN10 Pin 1
static const GPIO_Pin_t TL_EAST_YELLOW  = { GPIOC, GPIO_PIN_8 };  // CN10 Pin 2
static const GPIO_Pin_t TL_EAST_GREEN   = { GPIOC, GPIO_PIN_6 };  // CN10 Pin 4

/* Pedestrian South (PB5 → CN9-5 / D4) */
static const GPIO_Pin_t PED_SOUTH       = { GPIOB, GPIO_PIN_5 };
/* Pedestrian East  (PB7 → CN7-21)     */
static const GPIO_Pin_t PED_EAST        = { GPIOB, GPIO_PIN_7 };

/*  Button ( PC13 )  */
static const GPIO_Pin_t BUTTON_PED      = { GPIOC, GPIO_PIN_13 };

#endif // PINOUT_H
