/*============================================================================*/
/* 🚦 Traffic Light Control System                                            */
/*----------------------------------------------------------------------------*/
/*  Manages synchronized traffic light phases with pedestrian crossings       */
/*  using CMSIS-RTOS2 (Real-Time Operating System) API.                       */
/*                                                                            */
/*  Features:                                                                 */
/*   • Coordinated North-South and East-West traffic flow                     */
/*   • Pedestrian crossing requests with visual feedback                      */
/*   • Emergency vehicle preemption with traffic override                     */
/*   • OLED display updates for real-time traffic status                      */
/*                                                                            */
/*  RTOS Components:                                                          */
/*   • Threads for each direction and function                                */
/*   • Semaphores for task synchronization                                    */
/*   • Event flags for pedestrian/emergency signaling                         */
/*   • Mutex for safe OLED display access                                     */
/*============================================================================*/

#include "main.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "trafficlight.h"
#include "iwdg.h"
#include "logger.h"
#include "app_config.h"
#include "pinout.h"
#include "FreeRTOS.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "core_cm4.h"

static const TrafficLight_t trafficLightSouth = {
    .port       = TL_SOUTH_RED.port,
    .pin_red    = TL_SOUTH_RED.pin,
    .pin_yellow = TL_SOUTH_YELLOW.pin,
    .pin_green  = TL_SOUTH_GREEN.pin
};

static const TrafficLight_t trafficLightNorth = {
    .port       = TL_NORTH_RED.port,
    .pin_red    = TL_NORTH_RED.pin,
    .pin_yellow = TL_NORTH_YELLOW.pin,
    .pin_green  = TL_NORTH_GREEN.pin
};

static const TrafficLight_t trafficLightEast = {
    .port       = TL_EAST_RED.port,
    .pin_red    = TL_EAST_RED.pin,
    .pin_yellow = TL_EAST_YELLOW.pin,
    .pin_green  = TL_EAST_GREEN.pin
};

static const PedLight_t pedestrianLightSouth = {
    .port = PED_SOUTH.port,
    .pin  = PED_SOUTH.pin
};

static const PedLight_t pedestrianLightEast = {
    .port = PED_EAST.port,
    .pin  = PED_EAST.pin
};

COM_InitTypeDef       serialConfig;
I2C_HandleTypeDef  hi2c1;
osSemaphoreId_t       northSouthSemaphore, eastSemaphore, pedestrianSemaphore;
osEventFlagsId_t      eventFlagsId;
osThreadId_t          northSouthTaskHandle, eastTrafficTaskHandle, pedestrianTaskHandle, emergencyTaskHandle;
osMutexId_t           oledDisplayMutex;

typedef enum { PHASE_NS, PHASE_EST, PHASE_PED } Phase_t;
static volatile Phase_t activeTrafficPhase = PHASE_NS;

const osThreadAttr_t northSouthTaskAttributes  = { .name = "NorthSouthTask",  .priority = osPriorityAboveNormal,    .stack_size = 128*4 };
const osThreadAttr_t eastTrafficTaskAttributes = { .name = "EastSemTask",     .priority = osPriorityAboveNormal,    .stack_size = 128*4 };
const osThreadAttr_t pedestrianTaskAttributes  = { .name = "PedTask",         .priority = osPriorityAboveNormal1,  .stack_size = 128*4 };
const osThreadAttr_t emergencyTaskAttributes   = { .name = "EmergencyTask",   .priority = osPriorityRealtime,      .stack_size = 128*4 };

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void NorthSouthTask(void *argument);
void EastTrafficTask(void *argument);
void PedestrianTask(void *argument);
void EmergencyTask(void *argument);
static void MX_I2C1_Init(void);

int main(void)
{
    /*----------------------------------------------------------
     * 🍏 HAL Initialization
     * Initializes the Hardware Abstraction Layer (HAL),
     * sets up the Flash interface and Systick timer.
     *---------------------------------------------------------*/
	HAL_Init();

    /*----------------------------------------------------------
     * 🕒 System Clock Configuration
     * Configures the main system clocks (HCLK, PCLK1, PCLK2)
     * for optimal performance and timing accuracy.
     *---------------------------------------------------------*/
    SystemClock_Config();

    /*----------------------------------------------------------
     * 🌐 GPIO & I2C Setup
     * Prepares all General-Purpose I/O pins and the I2C1 bus
     * used for external peripherals (e.g., sensors, displays).
     *---------------------------------------------------------*/
    MX_GPIO_Init();
    MX_I2C1_Init();

    /*----------------------------------------------------------
     * 🖥️ SSD1306 OLED Initialization
     * Boots the OLED display driver, clears the screen,
     * and readies it for drawing text and graphics.
     *---------------------------------------------------------*/
    ssd1306_Init();

    /*----------------------------------------------------------
     * 🔄 Watchdog Reset Check
     * If the Independent Watchdog (IWDG) caused a reset,
     * clear the flag and notify via serial output.
     *---------------------------------------------------------*/
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
        __HAL_RCC_CLEAR_RESET_FLAGS();
        printf("*** Reset caused by IWDG ***\r\n");
    }

    /*----------------------------------------------------------
     * ⏱️ Independent Watchdog Initialization
     * Sets up the hardware watchdog to ensure system recovery
     * in case of software lock-ups or hangs.
     *---------------------------------------------------------*/
    MX_IWDG_Init();

    /*----------------------------------------------------------
     * 🧮 Random Seed from CPU Cycle Counter
     * Enables the DWT cycle counter and uses it to seed
     * the standard C pseudo-random number generator.
     *---------------------------------------------------------*/
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t seed = DWT->CYCCNT;
    srand(seed);

    /*----------------------------------------------------------
     * 🚀 RTOS Kernel Initialization
     * Prepares the CMSIS-RTOS2 kernel, clears old objects,
     * and gets ready to create threads and synchronization primitives.
     *---------------------------------------------------------*/
    osKernelInitialize();

    /*----------------------------------------------------------
     * 🎯 Event Flags Creation
     * Creates an event-flag object for inter-thread signaling.
     *---------------------------------------------------------*/
    eventFlagsId = osEventFlagsNew(NULL);
    if (eventFlagsId == NULL) Error_Handler();

    /*----------------------------------------------------------
     * 🔒 Mutex for OLED Display
     * Ensures that only one thread at a time can draw
     * on the shared OLED display to avoid collisions.
     *---------------------------------------------------------*/
    oledDisplayMutex = osMutexNew(NULL);
    if (oledDisplayMutex == NULL) Error_Handler();

    /*----------------------------------------------------------
     * 🚦 Semaphore Setup for Traffic Control
     * - northSouthSemaphore: Controls NS traffic (count = 1)
     * - eastSemaphore: Controls E-W traffic (count = 2)
     * - pedestrianSemaphore: Controls pedestrian crossing (count = 1)
     *---------------------------------------------------------*/
    northSouthSemaphore = osSemaphoreNew(1, 1, NULL);
    eastSemaphore = osSemaphoreNew(2, 0, NULL);
    pedestrianSemaphore = osSemaphoreNew(1, 0, NULL);
    if (!northSouthSemaphore || !eastSemaphore || !pedestrianSemaphore) Error_Handler();

    /*----------------------------------------------------------
     * 📡 UART Serial Configuration
     * Sets up serial port parameters (baud, word length,
     * stop bits, parity, hardware flow control) for console I/O.
     *---------------------------------------------------------*/
    serialConfig.BaudRate   = 115200;
    serialConfig.WordLength = COM_WORDLENGTH_8B;
    serialConfig.StopBits   = COM_STOPBITS_1;
    serialConfig.Parity     = COM_PARITY_NONE;
    serialConfig.HwFlowCtl  = COM_HWCONTROL_NONE;
    if (BSP_COM_Init(COM1, &serialConfig) != BSP_ERROR_NONE) Error_Handler();

    /*----------------------------------------------------------
     * 🧵 Thread Creation
     * Spawns the RTOS threads for:
     *  • North-South traffic control
     *  • East-West traffic control
     *  • Pedestrian signals
     *  • Emergency vehicle preemption
     *---------------------------------------------------------*/
    northSouthTaskHandle    = osThreadNew(NorthSouthTask, NULL, &northSouthTaskAttributes);
    eastTrafficTaskHandle   = osThreadNew(EastTrafficTask, NULL, &eastTrafficTaskAttributes);
    pedestrianTaskHandle    = osThreadNew(PedestrianTask, NULL, &pedestrianTaskAttributes);
    emergencyTaskHandle     = osThreadNew(EmergencyTask, NULL, &emergencyTaskAttributes);

    /*----------------------------------------------------------
     * ▶️ Start the RTOS Scheduler
     * Transfers control to the RTOS: threads will now run
     * according to their priorities and synchronization.
     *---------------------------------------------------------*/
    osKernelStart();

    /*----------------------------------------------------------
     * 🔁 Infinite Loop (Should Never Reach Here)
     * If we do, something went wrong—halt in a safe state.
     *---------------------------------------------------------*/
    while (1) {}
}

void NorthSouthTask(void *argument)
{
    uint32_t vehiclesSouth, vehiclesNorth;
    for (;;) {

        /*----------------------------------------------------------
         * 🚦 Semaphore Wait
         * Waits for permission to run North-South traffic cycle
         *---------------------------------------------------------*/
        osSemaphoreAcquire(northSouthSemaphore, osWaitForever);
        LogEventTS("NorthSouthTask_start");

        /*----------------------------------------------------------
         * 🚨 Emergency Check (Pre-Cycle)
         * If an emergency is detected, cancel current cycle
         *---------------------------------------------------------*/
        if (osEventFlagsGet(eventFlagsId) & EMG_FLAG) {
            osEventFlagsClear(eventFlagsId, EMG_FLAG);
            LogEventTS("NorthSouthTask_end_by_emergency");
            continue;
        }

        /*----------------------------------------------------------
         * 🚗 Vehicle Simulation
         * Randomly generates car counts for South and North
         *---------------------------------------------------------*/
        vehiclesSouth = (rand() % MAX_VEHICLES) + 1;
        vehiclesNorth = (rand() % MAX_VEHICLES) + 1;

        /*----------------------------------------------------------
         * 🖥️ OLED Display Update
         * Shows the vehicle count on the OLED screen
         *---------------------------------------------------------*/
        char buf[24];
        osMutexAcquire(oledDisplayMutex, osWaitForever);
        ssd1306_Fill(Black);
        sprintf(buf, "S: %2lu Cars", (unsigned long)vehiclesSouth);
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString(buf, Font_16x15, White);
        sprintf(buf, "N: %2lu Cars", (unsigned long)vehiclesNorth);
        ssd1306_SetCursor(0, 22);
        ssd1306_WriteString(buf, Font_16x15, White);
        ssd1306_UpdateScreen();
        osMutexRelease(oledDisplayMutex);

        /*----------------------------------------------------------
         * 🚶 Pedestrian Check
         * Checks if a pedestrian requested crossing (N-S)
         *---------------------------------------------------------*/
        bool pedestrianRequest = false;
        if (osEventFlagsGet(eventFlagsId) & PED_FLAG_NS) {
            pedestrianRequest = true;
            osEventFlagsClear(eventFlagsId, PED_FLAG_NS);
        }

        /*----------------------------------------------------------
         * ⏱️ Timing Calculation
         * Adjusts green and yellow times based on:
         *  • Pedestrian request
         *  • Traffic volume threshold
         *---------------------------------------------------------*/
        uint32_t greenTime  = pedestrianRequest ? (T_GREEN_MS/2) :
                              ((vehiclesSouth > PRIORITY_THRESHOLD || vehiclesNorth > PRIORITY_THRESHOLD) ?
                              T_GREEN_EXTENSION_MS : T_GREEN_MS);
        uint32_t yellowTime = pedestrianRequest ? (T_YELLOW_MS/2) : T_YELLOW_MS;

        /*----------------------------------------------------------
         * 🟢 Traffic Light Phase: GREEN
         * North-South: GREEN | East: RED
         *---------------------------------------------------------*/
        TL_SetState(&trafficLightSouth, TL_GREEN);
        TL_SetState(&trafficLightNorth, TL_GREEN);
        TL_SetState(&trafficLightEast, TL_RED);

        /*----------------------------------------------------------
         * ⏳ Wait During GREEN
         * Listen for emergency/pedestrian while lights are green
         *---------------------------------------------------------*/
        uint32_t flags = osEventFlagsWait(eventFlagsId, EMG_FLAG | PED_FLAG_NS, osFlagsWaitAny, greenTime);
        if (flags & EMG_FLAG) {
            LogEventTS("NorthSouthTask_end_by_emergency");
            continue;
        }
        if (flags & PED_FLAG_NS) osEventFlagsClear(eventFlagsId, PED_FLAG_NS);

        /*----------------------------------------------------------
         * 🟡 Traffic Light Phase: YELLOW
         * Transition North-South to yellow phase
         *---------------------------------------------------------*/
        TL_SetState(&trafficLightSouth, TL_YELLOW);
        TL_SetState(&trafficLightNorth, TL_YELLOW);

        /*----------------------------------------------------------
         * ⏳ Wait During YELLOW
         * Interruptible by emergency flag
         *---------------------------------------------------------*/
        flags = osEventFlagsWait(eventFlagsId, EMG_FLAG | PED_FLAG_NS, osFlagsWaitAny, yellowTime);
        if (flags & EMG_FLAG) {
            LogEventTS("NorthSouthTask_end_by_emergency");
            continue;
        }
        if (flags & PED_FLAG_NS) osEventFlagsClear(eventFlagsId, PED_FLAG_NS);

        /*----------------------------------------------------------
         * 🔴 Traffic Light Phase: RED
         * Ends North-South movement, prepares for East traffic
         *---------------------------------------------------------*/
        TL_SetState(&trafficLightSouth, TL_RED);
        TL_SetState(&trafficLightNorth, TL_RED);

        /*----------------------------------------------------------
         * ⏳ Final Wait (Before releasing East)
         * Brief pause to allow full stop, interruptible by EMG
         *---------------------------------------------------------*/
        flags = osEventFlagsWait(eventFlagsId, EMG_FLAG, osFlagsWaitAny, 1000U);
        if (flags & EMG_FLAG) {
            LogEventTS("NorthSouthTask_end_by_emergency");
            continue;
        }

        /*----------------------------------------------------------
         * 🔄 End of Cycle
         * Logs completion and gives turn to East direction
         *---------------------------------------------------------*/
        LogEventTS("NorthSouthTask_end");
        osSemaphoreRelease(eastSemaphore);
        osSemaphoreRelease(eastSemaphore);
    }
}

void EastTrafficTask(void *argument)
{
    uint32_t vehiclesEast;
    for (;;) {

        /*----------------------------------------------------------
         * ⏳ Wait for Turn
         * Waits for two semaphore releases from NorthSouthTask
         *---------------------------------------------------------*/
        osSemaphoreAcquire(eastSemaphore, osWaitForever);
        osSemaphoreAcquire(eastSemaphore, osWaitForever);
        LogEventTS("EastTrafficTask_start");

        /*----------------------------------------------------------
         * 🚨 Emergency Check
         * If an emergency is active, abort current cycle
         *---------------------------------------------------------*/
        if (osEventFlagsGet(eventFlagsId) & EMG_FLAG) {
            osEventFlagsClear(eventFlagsId, EMG_FLAG);
            LogEventTS("EastTrafficTask_end_by_emergency");
            continue;
        }

        /*----------------------------------------------------------
         * 🚗 Vehicle Simulation
         * Randomly simulates car count for East direction
         *---------------------------------------------------------*/
        vehiclesEast = (rand() % MAX_VEHICLES) + 1;
        char buf[24];

        /*----------------------------------------------------------
         * 🖥️ OLED Display Update
         * Displays number of cars approaching from the East
         *---------------------------------------------------------*/
        osMutexAcquire(oledDisplayMutex, osWaitForever);
        ssd1306_Fill(Black);
        sprintf(buf, "E: %2lu Cars", (unsigned long)vehiclesEast);
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString(buf, Font_16x15, White);
        ssd1306_UpdateScreen();
        osMutexRelease(oledDisplayMutex);

        /*----------------------------------------------------------
         * 🚶 Pedestrian Check
         * Checks for pedestrian request on East crosswalk
         *---------------------------------------------------------*/
        bool pedestrianRequest = false;
        if (osEventFlagsGet(eventFlagsId) & PED_FLAG_EST) {
            pedestrianRequest = true;
            osEventFlagsClear(eventFlagsId, PED_FLAG_EST);
        }

        /*----------------------------------------------------------
         * ⏱️ Timing Calculation
         * Adjusts green/yellow durations based on:
         *  • Pedestrian request
         *  • Traffic volume threshold
         *---------------------------------------------------------*/
        uint32_t greenTime  = pedestrianRequest ? (T_GREEN_MS/2) :
                              ((vehiclesEast > PRIORITY_THRESHOLD) ?
                              T_GREEN_EXTENSION_MS : T_GREEN_MS);
        uint32_t yellowTime = pedestrianRequest ? (T_YELLOW_MS/2) : T_YELLOW_MS;

        /*----------------------------------------------------------
         * 🟢 Traffic Light Phase: GREEN
         * East direction is given green signal
         *---------------------------------------------------------*/
        TL_SetState(&trafficLightEast, TL_GREEN);

        /*----------------------------------------------------------
         * ⏳ Wait During GREEN
         * Monitors emergency and pedestrian flags during green
         *---------------------------------------------------------*/
        uint32_t flags = osEventFlagsWait(eventFlagsId, EMG_FLAG | PED_FLAG_EST, osFlagsWaitAny, greenTime);
        if (flags & EMG_FLAG) {
            LogEventTS("EastTrafficTask_end_by_emergency");
            continue;
        }
        if (flags & PED_FLAG_EST) osEventFlagsClear(eventFlagsId, PED_FLAG_EST);

        /*----------------------------------------------------------
         * 🟡 Traffic Light Phase: YELLOW
         * Transition East signal to yellow
         *---------------------------------------------------------*/
        TL_SetState(&trafficLightEast, TL_YELLOW);

        /*----------------------------------------------------------
         * ⏳ Wait During YELLOW
         * Interruptible by emergency
         *---------------------------------------------------------*/
        flags = osEventFlagsWait(eventFlagsId, EMG_FLAG | PED_FLAG_EST, osFlagsWaitAny, yellowTime);
        if (flags & EMG_FLAG) {
            LogEventTS("EastTrafficTask_end_by_emergency");
            continue;
        }
        if (flags & PED_FLAG_EST) osEventFlagsClear(eventFlagsId, PED_FLAG_EST);

        /*----------------------------------------------------------
         * 🔴 Traffic Light Phase: RED
         * Ends East traffic flow
         *---------------------------------------------------------*/
        TL_SetState(&trafficLightEast, TL_RED);

        /*----------------------------------------------------------
         * ⏳ Delay Before Handing Off
         * Brief pause before releasing pedestrian task
         *---------------------------------------------------------*/
        flags = osEventFlagsWait(eventFlagsId, EMG_FLAG, osFlagsWaitAny, 1000U);
        if (flags & EMG_FLAG) {
            LogEventTS("EastTrafficTask_end_by_emergency");
            continue;
        }

        /*----------------------------------------------------------
         * 🔄 End of Cycle
         * Logs end and releases pedestrian semaphore
         *---------------------------------------------------------*/
        LogEventTS("EastTrafficTask_end");
        osSemaphoreRelease(pedestrianSemaphore);
    }
}

void PedestrianTask(void *argument)
{
    for (;;) {

        /*----------------------------------------------------------
         * ⏳ Wait for Semaphore
         * Triggered by EastTrafficTask after its cycle ends
         *---------------------------------------------------------*/
        osSemaphoreAcquire(pedestrianSemaphore, osWaitForever);
        LogEventTS("PedestrianTask_start");

        /*----------------------------------------------------------
         * 🧹 Clear Pedestrian Flags
         * Resets NS and EST pedestrian request flags
         *---------------------------------------------------------*/
        osEventFlagsClear(eventFlagsId, PED_FLAG_NS | PED_FLAG_EST);

        /*----------------------------------------------------------
         * 🚨 Emergency Check
         * Abort pedestrian sequence if emergency is active
         *---------------------------------------------------------*/
        if (osEventFlagsGet(eventFlagsId) & EMG_FLAG) {
            osEventFlagsClear(eventFlagsId, EMG_FLAG);
            LogEventTS("PedestrianTask_end_by_emergency");
            continue;
        }

        /*----------------------------------------------------------
         * 🚦 Activate Pedestrian Lights
         * Turns on steady walk signal for both directions
         *---------------------------------------------------------*/
        PL_On(&pedestrianLightSouth);
        PL_On(&pedestrianLightEast);

        /*----------------------------------------------------------
         * ⏱️ Wait Before Blinking
         * Duration of steady walk phase (non-blinking)
         *---------------------------------------------------------*/
        uint32_t t1 = T_PED_TOTAL_MS - T_PED_BLINK_MS;
        uint32_t flags = osEventFlagsWait(eventFlagsId, EMG_FLAG, osFlagsWaitAny, t1);
        if (flags & EMG_FLAG) {
            LogEventTS("PedestrianTask_end_by_emergency");
            continue;
        }

        /*----------------------------------------------------------
         * ✨ Blinking Phase
         * Blinks pedestrian lights at set intervals
         *---------------------------------------------------------*/
        for (uint32_t i = 0; i < T_PED_BLINK_MS / T_PED_BLINK_INTERVAL_MS; ++i) {
            if (osEventFlagsGet(eventFlagsId) & EMG_FLAG) break;
            PL_Toggle(&pedestrianLightSouth);
            PL_Toggle(&pedestrianLightEast);
            osDelay(T_PED_BLINK_INTERVAL_MS);
        }

        /*----------------------------------------------------------
         * 🔴 Turn Off Pedestrian Lights
         * Ends walk/blink phase
         *---------------------------------------------------------*/
        PL_Off(&pedestrianLightSouth);
        PL_Off(&pedestrianLightEast);

        /*----------------------------------------------------------
         * ⏳ Delay Before Handoff
         * Short pause before resuming North-South traffic
         *---------------------------------------------------------*/
        flags = osEventFlagsWait(eventFlagsId, EMG_FLAG, osFlagsWaitAny, 1000U);
        if (flags & EMG_FLAG) {
            LogEventTS("PedestrianTask_end_by_emergency");
            continue;
        }

        /*----------------------------------------------------------
         * 🔄 End of Cycle
         * Logs completion and resumes North-South traffic
         *---------------------------------------------------------*/
        LogEventTS("PedestrianTask_end");
        osSemaphoreRelease(northSouthSemaphore);
    }
}

void EmergencyTask(void *argument)
{
    const uint32_t period_ms = 30000;  // ⏲️ Interval between emergencies

    for (;;) {

        /*----------------------------------------------------------
         * 📝 Task Start Log
         * Indicates that the task has entered its loop
         *---------------------------------------------------------*/
        LogEventTS("EmergencyTask_start");

        /*----------------------------------------------------------
         * ⏳ Wait Before Triggering Emergency
         * Simulates time between emergency events
         *---------------------------------------------------------*/
        osDelay(period_ms);

        /*----------------------------------------------------------
         * ⏰ Emergency Trigger
         * Indicates that an emergency has occurred
         *---------------------------------------------------------*/
        LogEventTS("EmergencyTask_wakeup");

        /*----------------------------------------------------------
         * 🖥️ Display Emergency Message
         * Clears screen and shows "Emergency" alert
         *---------------------------------------------------------*/
        osMutexAcquire(oledDisplayMutex, osWaitForever);
        ssd1306_Fill(Black);
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("Emergency", Font_16x15, White);
        ssd1306_UpdateScreen();
        osMutexRelease(oledDisplayMutex);

        /*----------------------------------------------------------
         * 🚩 Set Emergency Flag
         * Notifies all tasks that emergency is active
         *---------------------------------------------------------*/
        osEventFlagsSet(eventFlagsId, EMG_FLAG);

        /*----------------------------------------------------------
         * 🚦 Override Traffic Lights
         *  • All directions RED except East
         *  • Pedestrian lights turned OFF
         *---------------------------------------------------------*/
        TL_SetState(&trafficLightSouth, TL_RED);
        TL_SetState(&trafficLightNorth, TL_RED);
        TL_SetState(&trafficLightEast, TL_GREEN);
        PL_Off(&pedestrianLightSouth);
        PL_Off(&pedestrianLightEast);

        /*----------------------------------------------------------
         * 💡 Emergency Indicator (GPIO)
         * Blinks LED to signal active emergency state
         *---------------------------------------------------------*/
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
        osDelay(5000);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

        /*----------------------------------------------------------
         * 🧹 Flush Semaphores
         * Clears any queued North-South traffic operations
         *---------------------------------------------------------*/
        while (osSemaphoreAcquire(northSouthSemaphore, 0) == osOK);

        /*----------------------------------------------------------
         * 🔄 End of Emergency
         * Releases North-South semaphore and clears flag
         *---------------------------------------------------------*/
        LogEventTS("EmergencyTask_end");
        osSemaphoreRelease(northSouthSemaphore);
        osEventFlagsClear(eventFlagsId, EMG_FLAG);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13)
    {
        /*----------------------------------------------------------
         * 📝 Log Button Press
         * Marks the ISR trigger for debugging or analysis
         *---------------------------------------------------------*/
        LogEventTS("ISR_PED");

        /*----------------------------------------------------------
         * 🖥️ OLED Notification
         * Displays a "Pedestrian Wait" message to the user
         *---------------------------------------------------------*/
        osMutexAcquire(oledDisplayMutex, osWaitForever);
        ssd1306_Fill(Black);
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("Pedestrian Wait", Font_16x15, White);
        ssd1306_UpdateScreen();
        osMutexRelease(oledDisplayMutex);

        /*----------------------------------------------------------
         * 🚶 Set Pedestrian Flags
         * Informs both directions of pending pedestrian request
         *---------------------------------------------------------*/
        osEventFlagsSet(eventFlagsId, PED_FLAG_NS | PED_FLAG_EST);
    }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM            = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN            = 85;
  RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK |
                                     RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    Error_Handler();
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x40B285C2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = TL_SOUTH_RED.pin
                        | TL_SOUTH_YELLOW.pin
                        | TL_SOUTH_GREEN.pin;
    HAL_GPIO_Init(TL_SOUTH_RED.port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = TL_NORTH_RED.pin
                        | TL_NORTH_YELLOW.pin
                        | TL_NORTH_GREEN.pin;
    HAL_GPIO_Init(TL_NORTH_RED.port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = TL_EAST_RED.pin
                        | TL_EAST_YELLOW.pin
                        | TL_EAST_GREEN.pin;
    HAL_GPIO_Init(TL_EAST_RED.port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PED_SOUTH.pin;
    HAL_GPIO_Init(PED_SOUTH.port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PED_EAST.pin;
    HAL_GPIO_Init(PED_EAST.port, &GPIO_InitStruct);

    TL_Init(&trafficLightSouth);
    TL_Init(&trafficLightNorth);
    TL_Init(&trafficLightEast);
    PL_Init(&pedestrianLightSouth);
    PL_Init(&pedestrianLightEast);

    HAL_GPIO_WritePin(PED_SOUTH.port, PED_SOUTH.pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PED_EAST.port,  PED_EAST.pin,  GPIO_PIN_RESET);

    /* Configure GPIO pins :PC8 PC9 */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins :PC7 */
    GPIO_InitStruct.Pin   = GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


    GPIO_InitStruct.Pin  = BUTTON_PED.pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BUTTON_PED.port, &GPIO_InitStruct);


    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
