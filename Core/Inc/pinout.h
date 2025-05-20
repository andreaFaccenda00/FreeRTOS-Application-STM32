#ifndef PINOUT_H
#define PINOUT_H

#include "stm32g4xx_hal.h"

/*============================================================================*/
/* 📍 Pin Mapping Configuration                                               */
/*----------------------------------------------------------------------------*/
/*  Defines logical mappings for traffic light and pedestrian GPIO pins.      */
/*  All pins are wrapped in GPIO_Pin_t structures for abstraction.            */
/*============================================================================*/

/*----------------------------------------------------------
 * 🧩 GPIO Pin Structure
 * Holds a reference to a GPIO port and pin number
 *---------------------------------------------------------*/
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} GPIO_Pin_t;

/*----------------------------------------------------------
 * 🚦 Traffic Lights — South Direction (CN8)
 * PA0  → RED
 * PA1  → YELLOW
 * PA4  → GREEN
 *---------------------------------------------------------*/
static const GPIO_Pin_t TL_SOUTH_RED    = { GPIOA, GPIO_PIN_0 };
static const GPIO_Pin_t TL_SOUTH_YELLOW = { GPIOA, GPIO_PIN_1 };
static const GPIO_Pin_t TL_SOUTH_GREEN  = { GPIOA, GPIO_PIN_4 };

/*----------------------------------------------------------
 * 🚦 Traffic Lights — North Direction (CN7)
 * PC10 → RED   (CN7 Pin 1)
 * PC11 → YELLOW(CN7 Pin 2)
 * PC12 → GREEN (CN7 Pin 3)
 *---------------------------------------------------------*/
static const GPIO_Pin_t TL_NORTH_RED    = { GPIOC, GPIO_PIN_10 };
static const GPIO_Pin_t TL_NORTH_YELLOW = { GPIOC, GPIO_PIN_11 };
static const GPIO_Pin_t TL_NORTH_GREEN  = { GPIOC, GPIO_PIN_12 };

/*----------------------------------------------------------
 * 🚦 Traffic Lights — East Direction (CN10)
 * PC9  → RED   (CN10 Pin 1)
 * PC8  → YELLOW(CN10 Pin 2)
 * PC6  → GREEN (CN10 Pin 4)
 *---------------------------------------------------------*/
static const GPIO_Pin_t TL_EAST_RED     = { GPIOC, GPIO_PIN_9 };
static const GPIO_Pin_t TL_EAST_YELLOW  = { GPIOC, GPIO_PIN_8 };
static const GPIO_Pin_t TL_EAST_GREEN   = { GPIOC, GPIO_PIN_6 };

/*----------------------------------------------------------
 * 🚶 Pedestrian Lights
 * PB5  → South Pedestrian Light (CN9-5 / D4)
 * PB7  → East Pedestrian Light  (CN7-21)
 *---------------------------------------------------------*/
static const GPIO_Pin_t PED_SOUTH       = { GPIOB, GPIO_PIN_5 };
static const GPIO_Pin_t PED_EAST        = { GPIOB, GPIO_PIN_7 };

/*----------------------------------------------------------
 * 🔘 Pedestrian Button Input
 * PC13 → User Button (Pedestrian request trigger)
 *---------------------------------------------------------*/
static const GPIO_Pin_t BUTTON_PED      = { GPIOC, GPIO_PIN_13 };

#endif // PINOUT_H
