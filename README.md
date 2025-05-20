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

- 🚘 Management of three traffic light units: North-South, East, and Emergency
- 🚶‍♂️ Pedestrian crossing support with steady and blinking light phases
- 🔁 Fully task-based logic with inter-task signaling
- 📺 OLED display integration to show live traffic and pedestrian states
- 🚨 Emergency vehicle preemption interrupting normal traffic
- 📊 Logger for timing metrics and system performance
- 🧠 Safe concurrent access to shared resources via `osMutex`

---

## 🧱 System Architecture

### 🧵 RTOS Task Structure

| Task Name       | Description                                                                 |
|----------------|-----------------------------------------------------------------------------|
| `NSTask`        | Controls North-South traffic flow with dynamic timing and pedestrian logic |
| `EstTask`       | Manages East traffic light using the same logic as NS                      |
| `PedTask`       | Controls pedestrian lights and blinking timing                             |
| `EmergencyTask` | Interrupts all phases and prioritizes East emergency lane                  |
| `LoggerTask`    | Collects and logs timing metrics using timestamps                          |

---

## 🔍 System Behavior

### 🔁 Traffic Phase Cycle

1. **North-South Phase (`NSTask`)**
   - Triggered by a binary semaphore `semNS`
   - Random number of vehicles per direction (`rand()`)
   - ⬆️ High traffic count → extended green
   - 🚶 Pedestrian request → green/yellow durations halved
   - Displays vehicle info on OLED (protected by mutex)
   - Preemptible via emergency or pedestrian event flags

2. **East Phase (`EstTask`)**
   - Triggered by acquiring `semEst` **twice**
   - Shares same logic and protections as `NSTask`

3. **Pedestrian Phase (`PedTask`)**
   - Activated via `semPed` from East phase
   - LED steady ON for 4s, then blinks for 1s
   - Interruptible at any moment by emergency flag
   - Updates OLED (uses mutex)

4. **Emergency Phase (`EmergencyTask`)**
   - Wakes every 15 seconds
   - Sends `EMG_FLAG` to preempt current task
   - Forces RED on NS, GREEN on East
   - Flushes pending semaphores to cancel queued traffic
   - Activates emergency indicator (LED on PC7)
   - Logs event and clears flag after cycle ends

5. **Pedestrian Button**
   - External interrupt on `PC13`
   - Sets `PED_FLAG_NS` and `PED_FLAG_EST`
   - Wakes any active traffic phase via `osEventFlags`

6. **Inter-Phase Delay**
   - 1-second all-red delay (`osDelay(1000)`) ensures safe phase transitions

7. **Watchdog**
   - IWDG is refreshed via `vApplicationIdleHook`
   - Prevents system lock-up in case of scheduler failure

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
| `osSemaphoreNew()`    | Synchronize task phases                  |
| `osSemaphoreAcquire()`| Wait for semaphore to begin execution    |
| `osSemaphoreRelease()`| Signal next task or phase                |
| `osEventFlagsSet()`   | Set emergency/pedestrian flags           |
| `osEventFlagsGet()`   | Read flag status                         |
| `osEventFlagsClear()` | Manually clear active flags              |
| `osEventFlagsWait()`  | Wait on multiple interruptible events    |
| `osMutexNew()`        | Protect OLED (shared resource)           |
| `osMutexAcquire()`    | Enter critical section                   |
| `osMutexRelease()`    | Exit critical section                    |
| `osDelay()`           | Timed delays for traffic/phases          |

### 🧠 Critical Sections & Concurrency Notes

- 🖥️ **OLED Display Access** is mutex-protected to prevent concurrent updates.
- 🚨 **Interrupt routines do not use mutexes**, to remain ISR-safe.
- 🧹 **EmergencyTask flushes semaphores** (`while(...)`) to forcibly interrupt queued tasks.
- 🧯 Tasks clear event flags manually after handling (no auto-clear).
- 🧩 Tasks wait on **multiple flags** (`osFlagsWaitAny`) to support simultaneous event detection (e.g., EMG + PED).
- 🔁 Transitions are atomic and safe from race conditions thanks to `semaphore` + `flag` handling.

---

## 📊 RTOS Profiling & Logger

| Metric               | Description                              |
|----------------------|------------------------------------------|
| **Wake-up Latency**  | `Task_START - ISR`                       |
| **Task Duration**    | `Task_END - Task_START`                  |
| **Jitter (Latency)** | Δ between successive ISR-to-task delays  |
| **Jitter (Duration)**| Δ between execution durations            |

- Logged using `LogEventTS()` with DWT-based timestamps
- Output printed via UART/Serial for analysis

---

## ⚙️ GPIO Pin Mapping

| Component            | Port | Pins                        |
|----------------------|------|-----------------------------|
| North-South Lights   | A    | PA0 (Red), PA1 (Yellow), PA4 (Green) |
| North Duplicate      | C    | PC10 (Red), PC11 (Yellow), PC12 (Green) |
| East Light           | C    | PC6 (Green), PC8 (Yellow), PC9 (Red) |
| Pedestrian LEDs      | B    | PB5 (South), PB7 (East)     |
| Pedestrian Button    | C    | PC13                        |
| Emergency LED        | C    | PC7                         |
| OLED (SSD1306 I2C)   | B    | PB6 (SCL), PB9 (SDA)        |

---

## 🧰 Development Environment

- 💻 **STM32CubeIDE** (G4 Series)
- 📦 **STM32 HAL Drivers**
- 🧵 **CMSIS-RTOS2** (FreeRTOS-based)
- 🔌 **Nucleo-G4 board**
- 📺 **OLED SSD1306** via I2C (mutex-protected)
- 🖨️ **Serial debugging** with `printf()` and DWT timer
- 🔐 **IWDG Watchdog** enabled, refreshed in idle hook

---

## 🚀 Future Improvements

- 📡 Replace `rand()` with real-time IR sensors
- 📲 Add UART-based configuration interface
- 🌐 ESP32 bridge for web dashboard and MQTT control
- 🔔 Buzzer module for accessibility (blind pedestrians)
- 📈 Export logs via USB or UART to external PC tools

---

## 📄 License

© 2025 STMicroelectronics  
Released under **educational and non-commercial use only**.

