# STM32 Traffic Light Controller with Pedestrian Signal (FreeRTOS)

## Overview

This project implements a **traffic light control system** using an STM32 microcontroller and **FreeRTOS**. It features:

- Three traffic lights (Sem1, Sem2, Sem3) controlled via GPIOs
- A pedestrian crossing with a final **blinking warning phase**
- RTOS task synchronization using **semaphores**

The traffic lights follow a coordinated sequence ensuring safe vehicle and pedestrian flow.

---

## System Architecture

### Traffic Lights

- **Sem1Task** – Controls the first traffic light on GPIOA (PA0, PA1, PA4)
- **Sem3Task** – Controls the third traffic light on GPIOC (PC10, PC11, PC12)
- **Sem2Task** – Controls the central traffic light on GPIOC (PC6, PC8, PC9) and a pedestrian LED on GPIOB (PB5)

### Sequence Logic

1. **Sem1 and Sem3** alternate their green/yellow/red cycles independently.
2. Once **both Sem1 and Sem3** have completed their cycles, **Sem2** runs.
3. Sem2 enables pedestrian crossing:
   - Pedestrian LED is solid ON for 4 seconds
   - Then **blinks for 1 second**
4. At the end, control is returned to Sem1 and Sem3, and the cycle repeats.

---

## Timing Configuration

| Phase                | Duration (ms) |
|----------------------|---------------|
| Green Light          | 4000          |
| Yellow Light         | 1500          |
| Pedestrian Solid     | 4000          |
| Pedestrian Blinking  | 1000          |
| Blink Interval       | 250           |

---

## FreeRTOS Synchronization

- `sem1SemHandle` – Binary semaphore to trigger Sem1
- `sem3SemHandle` – Binary semaphore to trigger Sem3
- `sem2SemHandle` – **Counting semaphore** used to ensure both Sem1 and Sem3 have completed before Sem2 starts

Each task runs in a loop and waits for its semaphore before proceeding.

---

## GPIO Pin Mapping

| Component      | Port | Pins             |
|----------------|------|------------------|
| Semaforo 1     | A    | PA0, PA1, PA4     |
| Semaforo 2     | C    | PC6, PC8, PC9     |
| Semaforo 3     | C    | PC10, PC11, PC12  |
| Pedestrian LED | B    | PB5               |

---

## Development Environment

- STM32CubeIDE or compatible STM32 development environment
- FreeRTOS (CMSIS-OS abstraction layer)
- STM32 HAL drivers
- Compatible STM32 Nucleo board with GPIO outputs

---

## Notes

- Button and onboard LED initialization is included, though unused in the core logic.
- The system can be extended to include interrupt-based pedestrian button or UART commands.

---

## License

© 2025 STMicroelectronics. All rights reserved. For educational and non-commercial use.
