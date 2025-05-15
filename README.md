# 🚦 STM32 Traffic Light Controller with Pedestrian Signal (CMSIS-RTOS2)

## 📚 Table of Contents

- [📝 Overview](#-overview)
- [🧱 System Architecture](#-system-architecture)
- [🔍 System Behavior](#-system-behavior)
- [⏱️ Timing Configuration](#️-timing-configuration)
- [🔐 Synchronization (CMSIS-RTOS2)](#-synchronization-cmsis-rtos2)
- [⚙️ GPIO Pin Mapping](#-gpio-pin-mapping)
- [🧰 Development Environment](#-development-environment)
- [🚀 Future Improvements](#-future-improvements)
- [📄 License](#-license)

---

## 📝 Overview

This project implements a **real-time traffic light control system** on an STM32 microcontroller using **CMSIS-RTOS2** (based on FreeRTOS). Key features include:

- 🚘 Management of three traffic light units (North-South, East)
- 🚶‍♂️ Pedestrian crossing support with LED indication and blink phase
- 🔁 Task-based control architecture with inter-task synchronization
- 📊 Dynamic traffic monitoring with green light extension
- ⏱️ Timing control and task notification using RTOS primitives

---

## 🧱 System Architecture

### 🧵 RTOS Task Structure

| Task Name  | Description                                              |
|------------|----------------------------------------------------------|
| `NSTask`   | Manages North-South lights (PA0, PA1, PA4 + PC10–12)     |
| `EstTask`  | Manages East traffic light (PC6, PC8, PC9)               |
| `PedTask`  | Controls pedestrian LEDs (PB5, PB7) and pedestrian logic |

Each task operates independently but is triggered based on semaphores and inter-task signaling.

---

## 🔍 System Behavior

The application uses a **cyclic task schedule** driven by semaphores, with real-time responsiveness to external events (button presses). Key behaviors include:

### 🔁 Traffic Light Cycle

1. **`NSTask` (North-South Phase)**:  
   - Triggered via `semNS` binary semaphore.
   - Simulates a number of vehicles in South and North directions using `rand()`.
   - If the number of vehicles on either direction exceeds a `PRIORITY_THRESHOLD`, the **green time is extended** from the default `T_GREEN_MS` (4s) to `T_GREEN_EXTENSION_MS` (6s).
   - If a **pedestrian button** is pressed during this phase, green time is **cut in half** to prioritize pedestrians.

2. **`EstTask` (East Phase)**:  
   - Requires **two tokens** from the counting semaphore `semEst`, released by `NSTask`.
   - Manages the East light with the same dynamic timing logic:
     - Extended green if traffic is heavy (more than 15 vehicles).
     - Shortened green if pedestrian flag is active.

3. **`PedTask` (Pedestrian Crossing)**:  
   - Triggered via `semPed`.
   - Pedestrian LEDs (PB5, PB7) are:
     - ON for 4 seconds
     - Blinking for 1 second (at 250 ms intervals)
   - After execution, it clears the pedestrian event flag and resumes the North-South phase.

### 🔘 Button Interrupt Logic

- An **external interrupt** on pin `PC13` handles the pedestrian button press.
- The handler uses:
  - `osEventFlagsSet()` to signal that a pedestrian has requested crossing.
  - `osThreadFlagsSet()` to notify the currently active task (`NSTask` or `EstTask`) to **end green early**.

---

## ⏱️ Timing Configuration

| Phase                | Duration (ms) |
|----------------------|---------------|
| 🟢 Green Light        | 4000          |
| 🟡 Yellow Light       | 1500          |
| 🟢 Extended Green     | 6000          |
| 🚶 Pedestrian Steady  | 4000          |
| 🚨 Pedestrian Blinking| 1000          |
| 🔁 Blink Interval     | 250           |

---

## 🔐 Synchronization (CMSIS-RTOS2)

This project demonstrates the practical use of **CMSIS-RTOS2 primitives**:

| RTOS2 API              | Purpose                                      | Usage Example                               |
|------------------------|----------------------------------------------|---------------------------------------------|
| `osThreadNew()`        | Creates RTOS threads                         | `NSTask`, `EstTask`, `PedTask`              |
| `osSemaphoreNew()`     | Creates binary/counting semaphores           | `semNS`, `semEst`, `semPed`                 |
| `osSemaphoreAcquire()` | Blocks a task until the semaphore is released| Controls task execution order               |
| `osSemaphoreRelease()` | Signals another task to run                  | Task-to-task progression                    |
| `osEventFlagsNew()`    | Creates an event flag group                  | `pedFlags` for button state                 |
| `osEventFlagsSet()`    | Signals an event (e.g., from ISR)            | Called in `HAL_GPIO_EXTI_Callback()`        |
| `osThreadFlagsSet()`   | Notifies a specific task (non-blocking)      | Used to interrupt ongoing task              |
| `osThreadFlagsWait()`  | Waits for a notification                     | Allows task to respond immediately to events|
| `osDelay()`            | Suspends a task for given ms                 | Implements timed delays (green, yellow, etc.)|
| `osKernelGetTickCount()`| Gets system time (ms tick)                  | Used for logging and timestamp generation   |

The real-time design ensures **non-blocking, deterministic transitions** between traffic phases and pedestrian requests. By decoupling responsibilities across RTOS tasks, the system is modular and scalable.

---

## ⚙️ GPIO Pin Mapping

| Component            | Port | Pins                |
|--------------------- |------|---------------------|
| 🚦 North-South Light | A    | PA0, PA1, PA4       |
| 🚦 North (duplicate) | C    | PC10, PC11, PC12    |
| 🚦 East Light        | C    | PC6, PC8, PC9       |
| 🚶 Pedestrian LEDs   | B    | PB5 (South), PB7 (East) |
| 🔘 Pedestrian Button | C    | PC13                |

---

## 🧰 Development Environment

- 💻 **STM32CubeIDE**  
- 🧵 **CMSIS-RTOS2** (FreeRTOS-based)  
- 🔌 **STM32 HAL Drivers**  
- 📦 **STM32G4 Nucleo Board**  
- 🪛 Serial output via `printf()` (for logging traffic events)

---

## 🚀 Future Improvements

- 📲 UART command support for remote control or debug console
- 📷 Replace vehicle simulation with real sensors (IR, radar)
- 🔔 Audio feedback for visually impaired pedestrians
- 🌐 Web dashboard using Ethernet or Wi-Fi

---

## 📄 License

© 2025 STMicroelectronics  
Released for **educational and non-commercial use** only.
