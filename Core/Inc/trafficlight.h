#ifndef TRAFFIC_LIGHT_H
#define TRAFFIC_LIGHT_H

#include "main.h"

/*============================================================================*/
/* 🚦 Traffic & Pedestrian Light Control Interface                            */
/*----------------------------------------------------------------------------*/
/*  Provides data structures and function prototypes for controlling          */
/*  vehicle traffic lights and pedestrian lights.                             */
/*============================================================================*/

/*----------------------------------------------------------
 * 🎨 Traffic Light State Enum
 * Represents the current state of a traffic light:
 *  • TL_RED     → Red only
 *  • TL_YELLOW  → Yellow only
 *  • TL_GREEN   → Green only
 *  • TL_OFF     → All lights off
 *---------------------------------------------------------*/
typedef enum { TL_RED, TL_YELLOW, TL_GREEN, TL_OFF } TL_State_t;

/*----------------------------------------------------------
 * 🚦 Traffic Light Structure
 * Defines a traffic light group with shared port and
 * individual pin numbers for red, yellow, and green LEDs.
 *---------------------------------------------------------*/
typedef struct {
  GPIO_TypeDef* port;
  uint16_t      pin_red;
  uint16_t      pin_yellow;
  uint16_t      pin_green;
} TrafficLight_t;

/*----------------------------------------------------------
 * 🛠️ Traffic Light Functions
 * TL_Init       → Initializes the GPIOs (if needed)
 * TL_SetState   → Sets the active light (red/yellow/green)
 *---------------------------------------------------------*/
void TL_Init(const TrafficLight_t* tl);
void TL_SetState(const TrafficLight_t* tl, TL_State_t state);

/*----------------------------------------------------------
 * 🚶 Pedestrian Light Structure
 * Simple on/off LED with one GPIO pin
 *---------------------------------------------------------*/
typedef struct {
  GPIO_TypeDef* port;
  uint16_t      pin;
} PedLight_t;

/*----------------------------------------------------------
 * 👣 Pedestrian Light Functions
 * PL_Init    → Initializes the GPIO (if needed)
 * PL_On      → Turns the light on
 * PL_Off     → Turns the light off
 * PL_Toggle  → Toggles the light (used for blinking)
 *---------------------------------------------------------*/
void PL_Init(const PedLight_t* pl);
void PL_On  (const PedLight_t* pl);
void PL_Off (const PedLight_t* pl);
void PL_Toggle(const PedLight_t* pl);

#endif // TRAFFIC_LIGHT_H
