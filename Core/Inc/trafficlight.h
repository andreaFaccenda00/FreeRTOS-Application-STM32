// trafficlight.h (estendi il file)
#ifndef TRAFFIC_LIGHT_H
#define TRAFFIC_LIGHT_H

#include "main.h"

// --- semafori veicolari ---
typedef enum { TL_RED, TL_YELLOW, TL_GREEN } TL_State_t;

typedef struct {
  GPIO_TypeDef* port;
  uint16_t      pin_red;
  uint16_t      pin_yellow;
  uint16_t      pin_green;
} TrafficLight_t;

void TL_Init(const TrafficLight_t* tl);
void TL_SetState(const TrafficLight_t* tl, TL_State_t state);

// --- luci pedonali (on/off/blink) ---
typedef struct {
  GPIO_TypeDef* port;
  uint16_t      pin;
} PedLight_t;

void PL_Init(const PedLight_t* pl);
void PL_On  (const PedLight_t* pl);
void PL_Off (const PedLight_t* pl);
void PL_Toggle(const PedLight_t* pl);

#endif // TRAFFIC_LIGHT_H
