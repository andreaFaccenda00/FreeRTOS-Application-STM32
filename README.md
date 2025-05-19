# 🚦 STM32 Traffic Light Controller with Pedestrian Signal (CMSIS-RTOS2)

## 📚 Table of Contents

- [📝 Overview](#-overview)
- [🧱 System Architecture](#-system-architecture)
- [🔍 System Behavior](#-system-behavior)
- [⏱️ Timing Configuration](#️-timing-configuration)
- [🔐 Synchronization (CMSIS-RTOS2)](#-synchronization-cmsis-rtos2)
- [📊 RTOS Profiling & Logger](#-rtos-profiling--logger)
- [⚙️ GPIO Pin Mapping](#-gpio-pin-mapping)
- [🧰 Development Environment](#-development-environment)
- [🚀 Future Improvements](#-future-improvements)
- [📄 License](#-license)

---

## 📝 Overview

This project implements a **real-time traffic light control system** on an STM32 microcontroller using **CMSIS-RTOS2** (based on FreeRTOS). Key features include:

- 🚘 Management of three traffic light units (North-South, East, Emergency)
- 🚶‍♂️ Pedestrian crossing support with LED indication and blink phase
- 🔁 Task-based control architecture with inter-task synchronization
- 📊 Dynamic traffic monitoring with green light extension
- 📺 Real-time **OLED traffic monitor** synchronized via mutex
- 🚨 Emergency priority lane handling with exclusive green light
- ⏱️ Preemption-based event handling using `osThreadFlags`

---

## 🧱 System Architecture

### 🧵 RTOS Task Structure

| Task Name       | Description                                                                 |
|----------------|-----------------------------------------------------------------------------|
| `NSTask`        | Manages North-South lights (PA0, PA1, PA4 + PC10–12)                        |
| `EstTask`       | Manages East traffic light (PC6, PC8, PC9)                                  |
| `PedTask`       | Controls pedestrian LEDs (PB5, PB7) and logic                               |
| `EmergencyTask` | Grants green to **emergency lane only**, interrupts other tasks             |
| `LoggerTask`    | Records timing metrics for real-time performance analysis                   |

All tasks accessing the OLED display use a **mutex** to ensure mutual exclusion during data output.

---

## 🔍 System Behavior

### 🔁 Traffic Cycle Overview

1. **North-South Phase (`NSTask`)**
   - Triggered by `semNS` (binary semaphore).
   - Uses `rand()` to simulate vehicle count on both sides.
   - High count → extended green; pedestrian request → halved green.
   - OLED displays vehicle info and current state using **mutex** protection.

2. **East Phase (`EstTask`)**
   - Requires two tokens from `semEst`.
   - Same dynamic duration and display logic as `NSTask`.

3. **Pedestrian Phase (`PedTask`)**
   - Triggered via `semPed`.
   - LEDs ON for 4s, blink for 1s.
   - Emergency interrupts this phase.
   - OLED shows pedestrian countdown (via mutex).

4. **Emergency Phase (`EmergencyTask`)**
   - Every 15s, sets `EMG_FLAG` to preempt all tasks.
   - Forces **green to emergency lane only**, red to others.
   - Activates emergency LED (`PC7`) and resets semaphores.
   - Also logs event to OLED using mutex.

5. **Logger Task**
   - Collects runtime statistics:
     - **Wake-up latency** = `Task_START - ISR`
     - **Task duration** = `Task_END - Task_START`
     - **Wake-up jitter** = variation in successive latencies
     - **Execution jitter** = variation in successive durations

### 🔘 Pedestrian Button

- External interrupt on `PC13` triggers pedestrian request.
- Sets `PED_FLAG` and notifies the current phase using `osThreadFlagsSet`.

### ⏸️ Inter-Phase Delay and Watchdog

- All-red delay (`osDelay(1000)`) ensures safe transitions.
- **IWDG watchdog** protects against task deadlocks.

---

## ⏱️ Timing Configuration

| Phase                  | Duration (ms) |
|------------------------|---------------|
| 🟢 Green Light         | 4000          |
| 🟡 Yellow Light        | 1500          |
| 🟢 Extended Green      | 6000          |
| 🚶 Pedestrian Steady   | 4000          |
| 🚨 Pedestrian Blinking | 1000          |
| 🔁 Blink Interval      | 250           |
| 🚨 Emergency Interval  | 15000         |
| 🚨 Emergency Duration  | 5000          |

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
| `osMutexNew()`        | **Exclusive OLED access**                |
| `osMutexAcquire()`    | Lock OLED display                        |
| `osMutexRelease()`    | Unlock OLED display                      |
| `osDelay()`           | Time-controlled transitions              |

---

## 📊 RTOS Profiling & Logger

To assess **real-time performance**, the system logs:

- **Wake-up Latency**: `Task_START - ISR`
- **Task Duration**: `Task_END - Task_START`
- **Jitter (Latency)**: Δ between successive wake-ups
- **Jitter (Duration)**: Δ between successive executions

Timestamps are derived from the DWT cycle counter and printed via `printf()` on the serial monitor.

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
| OLED (SSD1306 I2C)   | B    | PB6 (SCL), PB9 (SDA)        |

---

## 🧰 Development Environment

- 💻 **STM32CubeIDE**
- ⚙️ **STM32 HAL Drivers**
- 🧵 **CMSIS-RTOS2 (FreeRTOS)**
- 🔌 **STM32G4 Nucleo Board**
- 📺 **OLED Display SSD1306 via I2C**
- 🖨️ **Serial debug via `printf()` and DWT timer**

---

## 🚀 Future Improvements

- 📡 Replace random vehicle input with IR sensors
- 📲 UART interface for remote control
- 🌐 Real-time web dashboard with ESP32 bridge
- 🔔 Add buzzer for visually impaired pedestrians
- 📈 Export logger data via USB/UART for PC analysis

---

## 📄 License

© 2025 STMicroelectronics  
Released under **educational and non-commercial use only**.
