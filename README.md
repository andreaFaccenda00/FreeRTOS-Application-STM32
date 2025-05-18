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
- ⏱️ Preemption-based emergency interruption using `osThreadFlags`

---

## 🧱 System Architecture

### 🧵 RTOS Task Structure

| Task Name         | Description                                                                 |
|------------------|-----------------------------------------------------------------------------|
| `NSTask`          | Manages North-South lights (PA0, PA1, PA4 + PC10–12)                        |
| `EstTask`         | Manages East traffic light (PC6, PC8, PC9)                                  |
| `PedTask`         | Controls pedestrian LEDs (PB5, PB7) and logic                               |
| `EmergencyTask`   | Periodically interrupts all tasks to simulate emergency preemption via LED |

Each task uses **`osThreadFlagsWait`** for real-time responsiveness to events like emergency or pedestrian requests.

---

## 🔍 System Behavior

### 🔁 Traffic Cycle Overview

1. **North-South Phase (`NSTask`)**
   - Activated via `semNS` (binary semaphore).
   - Uses `rand()` to simulate vehicle count on South and North.
   - If count exceeds a threshold, green is extended.
   - If a **pedestrian request** occurs (`PED_FLAG`), green time is halved.
   - At the end of green or upon pedestrian/emergency, transitions to yellow, then red.

2. **East Phase (`EstTask`)**
   - Needs **two tokens** from `semEst`.
   - Same dynamic timing logic as `NSTask`.
   - Transitions to pedestrian phase via `semPed`.

3. **Pedestrian Phase (`PedTask`)**
   - Activated via `semPed`.
   - LEDs ON steady for 4s, then blink for 1s (250ms interval).
   - On emergency, phase is immediately aborted.
   - Releases `semNS` to restart cycle.

4. **Emergency Phase (`EmergencyTask`)**
   - Every 15 seconds, sets `EMG_FLAG` via `osThreadFlagsSet` to **preempt all tasks**.
   - Forces all lights to red and turns on an emergency LED (`PC7`) for 5s.
   - Clears semaphores to reset the traffic cycle to North-South.

### 🔘 Pedestrian Button

- External interrupt on `PC13` triggers pedestrian request.
- Sets `PED_FLAG` and notifies active task (`NSTask` or `EstTask`) via `osThreadFlagsSet`.

### ⏸️ Inter-Phase Delay and Watchdog

- Each phase has a **1-second all-red delay** (`osDelay(1000)`) before releasing the next phase.
- An **Independent Watchdog Timer (IWDG)** resets the system in case of deadlock.

---

## ⏱️ Timing Configuration

| Phase                 | Duration (ms) |
|-----------------------|---------------|
| 🟢 Green Light        | 4000          |
| 🟡 Yellow Light       | 1500          |
| 🟢 Extended Green     | 6000          |
| 🚶 Pedestrian Steady  | 4000          |
| 🚨 Pedestrian Blinking| 1000          |
| 🔁 Blink Interval     | 250           |
| 🚨 Emergency Interval | 15000         |
| 🚨 Emergency Duration | 5000          |

---

## 🔐 Synchronization (CMSIS-RTOS2)

| API                   | Use Case                                |
|-----------------------|------------------------------------------|
| `osThreadNew()`       | Thread creation                          |
| `osSemaphoreNew()`    | Phase synchronization                    |
| `osSemaphoreAcquire()`| Await phase                              |
| `osSemaphoreRelease()`| Transition to next phase                 |
| `osThreadFlagsSet()`  | Notify individual threads (e.g. EMG, PED)|
| `osThreadFlagsWait()` | Handle preemption and external signals   |
| `osDelay()`           | Time-controlled transitions              |

> The use of `osThreadFlags` eliminates the need for global shared flags. Preemption becomes **task-specific** and deterministic.

---

## ⚙️ GPIO Pin Mapping

| Component            | Port | Pins                        |
|----------------------|------|-----------------------------|
| North-South Lights   | A    | PA0, PA1, PA4               |
| North Duplicate      | C    | PC10, PC11, PC12            |
| East Light           | C    | PC6, PC8, PC9               |
| Pedestrian LEDs      | B    | PB5 (South), PB7 (East)     |
| Pedestrian Button    | C    | PC13                        |
| Emergency LED        | C    | PC7                         |

---

## 🧰 Development Environment

- 💻 **STM32CubeIDE**
- ⚙️ **STM32 HAL Drivers**
- 🧵 **CMSIS-RTOS2 (FreeRTOS)**
- 🔌 **STM32G4 Nucleo Board**
- 🖨️ **Serial debug via `printf()`**
- 🖥️ **OLED Display (SSD1306 via I2C)**

---

## 🚀 Future Improvements

- 📡 Replace random vehicle input with IR sensors
- 📲 UART interface for remote control
- 🌐 Real-time web dashboard with ESP32 bridge
- 🔔 Add buzzer for visually impaired pedestrians

---

## 📄 License

© 2025 STMicroelectronics  
Released under **educational and non-commercial use only**.
