/*============================================================================*/
/*       Traffic lights with pedestrian crossing and synchronized NS phase    */
/*                       using CMSIS-RTOS2 API                                */
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
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();

    ssd1306_Init();

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
        __HAL_RCC_CLEAR_RESET_FLAGS();
        printf("*** Reset caused by IWDG ***\r\n");
    }
    MX_IWDG_Init();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t seed = DWT->CYCCNT;
    srand(seed);
    osKernelInitialize();

    eventFlagsId = osEventFlagsNew(NULL);
    if (eventFlagsId == NULL) Error_Handler();

    oledDisplayMutex = osMutexNew(NULL);
    if (oledDisplayMutex == NULL) Error_Handler();

    northSouthSemaphore = osSemaphoreNew(1, 1, NULL);
    eastSemaphore = osSemaphoreNew(2, 0, NULL);
    pedestrianSemaphore = osSemaphoreNew(1, 0, NULL);
    if (!northSouthSemaphore || !eastSemaphore || !pedestrianSemaphore) Error_Handler();

    serialConfig.BaudRate   = 115200;
    serialConfig.WordLength = COM_WORDLENGTH_8B;
    serialConfig.StopBits   = COM_STOPBITS_1;
    serialConfig.Parity     = COM_PARITY_NONE;
    serialConfig.HwFlowCtl  = COM_HWCONTROL_NONE;
    if (BSP_COM_Init(COM1, &serialConfig) != BSP_ERROR_NONE) Error_Handler();

    northSouthTaskHandle    = osThreadNew(NorthSouthTask, NULL, &northSouthTaskAttributes);
    eastTrafficTaskHandle   = osThreadNew(EastTrafficTask, NULL, &eastTrafficTaskAttributes);
    pedestrianTaskHandle    = osThreadNew(PedestrianTask, NULL, &pedestrianTaskAttributes);
    emergencyTaskHandle     = osThreadNew(EmergencyTask, NULL, &emergencyTaskAttributes);

    osKernelStart();
    while (1) {}
}

/*============================================================================*/
/*                                 NorthSouthTask                             */
/*============================================================================*/
void NorthSouthTask(void *argument)
{
    uint32_t vehiclesSouth, vehiclesNorth;
    for (;;) {
        osSemaphoreAcquire(northSouthSemaphore, osWaitForever);
        LogEventTS("NorthSouthTask_start");

        if (osEventFlagsGet(eventFlagsId) & EMG_FLAG) {
            osEventFlagsClear(eventFlagsId, EMG_FLAG);
            LogEventTS("NorthSouthTask_end_by_emergency");
            continue;
        }

        vehiclesSouth = (rand() % MAX_VEHICLES) + 1;
        vehiclesNorth = (rand() % MAX_VEHICLES) + 1;

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

        bool pedestrianRequest = false;
        if (osEventFlagsGet(eventFlagsId) & PED_FLAG_NS) {
            pedestrianRequest = true;
            osEventFlagsClear(eventFlagsId, PED_FLAG_NS);
        }

        uint32_t greenTime  = pedestrianRequest ? (T_GREEN_MS/2) : ((vehiclesSouth > PRIORITY_THRESHOLD || vehiclesNorth > PRIORITY_THRESHOLD) ? T_GREEN_EXTENSION_MS : T_GREEN_MS);
        uint32_t yellowTime = pedestrianRequest ? (T_YELLOW_MS/2) : T_YELLOW_MS;

        TL_SetState(&trafficLightSouth, TL_GREEN);
        TL_SetState(&trafficLightNorth, TL_GREEN);
        TL_SetState(&trafficLightEast, TL_RED);

        uint32_t flags = osEventFlagsWait(eventFlagsId, EMG_FLAG | PED_FLAG_NS, osFlagsWaitAny, greenTime);
        if (flags & EMG_FLAG) {
        	LogEventTS("NorthSouthTask_end_by_emergency");
        	continue;
        }
        if (flags & PED_FLAG_NS) osEventFlagsClear(eventFlagsId, PED_FLAG_NS);

        TL_SetState(&trafficLightSouth, TL_YELLOW);
        TL_SetState(&trafficLightNorth, TL_YELLOW);

        flags = osEventFlagsWait(eventFlagsId, EMG_FLAG | PED_FLAG_NS, osFlagsWaitAny, yellowTime);
        if (flags & EMG_FLAG) {
        	LogEventTS("NorthSouthTask_end_by_emergency");
        	continue;
        }
        if (flags & PED_FLAG_NS) osEventFlagsClear(eventFlagsId, PED_FLAG_NS);

        TL_SetState(&trafficLightSouth, TL_RED);
        TL_SetState(&trafficLightNorth, TL_RED);

        flags = osEventFlagsWait(eventFlagsId, EMG_FLAG, osFlagsWaitAny, 1000U);
        if (flags & EMG_FLAG) {
        	LogEventTS("NorthSouthTask_end_by_emergency");
        	continue;
        }

        LogEventTS("NorthSouthTask_end");
        osSemaphoreRelease(eastSemaphore);
        osSemaphoreRelease(eastSemaphore);
    }
}

/*============================================================================*/
/*                                 EastTrafficTask                            */
/*============================================================================*/
void EastTrafficTask(void *argument)
{
    uint32_t vehiclesEast;
    for (;;) {
        osSemaphoreAcquire(eastSemaphore, osWaitForever);
        osSemaphoreAcquire(eastSemaphore, osWaitForever);
        LogEventTS("EastTrafficTask_start");

        if (osEventFlagsGet(eventFlagsId) & EMG_FLAG) {
            osEventFlagsClear(eventFlagsId, EMG_FLAG);
            LogEventTS("EastTrafficTask_end_by_emergency");
            continue;
        }

        vehiclesEast = (rand() % MAX_VEHICLES) + 1;
        char buf[24];

        osMutexAcquire(oledDisplayMutex, osWaitForever);
        ssd1306_Fill(Black);
        sprintf(buf, "E: %2lu Cars", (unsigned long)vehiclesEast);
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString(buf, Font_16x15, White);
        ssd1306_UpdateScreen();
        osMutexRelease(oledDisplayMutex);

        bool pedestrianRequest = false;
        if (osEventFlagsGet(eventFlagsId) & PED_FLAG_EST) {
            pedestrianRequest = true;
            osEventFlagsClear(eventFlagsId, PED_FLAG_EST);
        }

        uint32_t greenTime  = pedestrianRequest ? (T_GREEN_MS/2) : ((vehiclesEast > PRIORITY_THRESHOLD) ? T_GREEN_EXTENSION_MS : T_GREEN_MS);
        uint32_t yellowTime = pedestrianRequest ? (T_YELLOW_MS/2) : T_YELLOW_MS;

        TL_SetState(&trafficLightEast, TL_GREEN);

        uint32_t flags = osEventFlagsWait(eventFlagsId, EMG_FLAG | PED_FLAG_EST, osFlagsWaitAny, greenTime);
        if (flags & EMG_FLAG) {
        	LogEventTS("EastTrafficTask_end_by_emergency");
        	continue;
        }

        if (flags & PED_FLAG_EST) osEventFlagsClear(eventFlagsId, PED_FLAG_EST);

        TL_SetState(&trafficLightEast, TL_YELLOW);

        flags = osEventFlagsWait(eventFlagsId, EMG_FLAG | PED_FLAG_EST, osFlagsWaitAny, yellowTime);
        if (flags & EMG_FLAG) {
        	LogEventTS("EastTrafficTask_end_by_emergency");
        	continue;
        }
        if (flags & PED_FLAG_EST) osEventFlagsClear(eventFlagsId, PED_FLAG_EST);

        TL_SetState(&trafficLightEast, TL_RED);

        flags = osEventFlagsWait(eventFlagsId, EMG_FLAG, osFlagsWaitAny, 1000U);
        if (flags & EMG_FLAG) {
        	LogEventTS("EastTrafficTask_end_by_emergency");
        	continue;
        }

        LogEventTS("EastTrafficTask_end");
        osSemaphoreRelease(pedestrianSemaphore);
    }
}

/*============================================================================*/
/*                                 PedestrianTask                             */
/*============================================================================*/
void PedestrianTask(void *argument)
{
    for (;;) {
        osSemaphoreAcquire(pedestrianSemaphore, osWaitForever);
        LogEventTS("PedestrianTask_start");

        osEventFlagsClear(eventFlagsId, PED_FLAG_NS | PED_FLAG_EST);

        if (osEventFlagsGet(eventFlagsId) & EMG_FLAG) {
            osEventFlagsClear(eventFlagsId, EMG_FLAG);
            LogEventTS("PedestrianTask_end_by_emergency");
            continue;
        }

        PL_On(&pedestrianLightSouth);
        PL_On(&pedestrianLightEast);

        uint32_t t1 = T_PED_TOTAL_MS - T_PED_BLINK_MS;
        uint32_t flags = osEventFlagsWait(eventFlagsId, EMG_FLAG, osFlagsWaitAny, t1);
        if (flags & EMG_FLAG) {
        	LogEventTS("PedestrianTask_end_by_emergency");
        	continue;
        }

        for (uint32_t i = 0; i < T_PED_BLINK_MS / T_PED_BLINK_INTERVAL_MS; ++i) {
            if (osEventFlagsGet(eventFlagsId) & EMG_FLAG) break;
            PL_Toggle(&pedestrianLightSouth);
            PL_Toggle(&pedestrianLightEast);
            osDelay(T_PED_BLINK_INTERVAL_MS);
        }

        PL_Off(&pedestrianLightSouth);
        PL_Off(&pedestrianLightEast);

        flags = osEventFlagsWait(eventFlagsId, EMG_FLAG, osFlagsWaitAny, 1000U);
        if (flags & EMG_FLAG) {
        	LogEventTS("PedestrianTask_end_by_emergency");
        	continue;
        }

        LogEventTS("PedestrianTask_end");
        osSemaphoreRelease(northSouthSemaphore);
    }
}

/*============================================================================*/
/*                                 EmergencyTask                              */
/*============================================================================*/
void EmergencyTask(void *argument)
{
    const uint32_t period_ms = 30000;
    for (;;) {
        LogEventTS("EmergencyTask_start");

        osDelay(period_ms);

        LogEventTS("EmergencyTask_wakeup");

        osMutexAcquire(oledDisplayMutex, osWaitForever);
        ssd1306_Fill(Black);
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("Emergency", Font_16x15, White);
        ssd1306_UpdateScreen();
        osMutexRelease(oledDisplayMutex);

        osEventFlagsSet(eventFlagsId, EMG_FLAG);

        TL_SetState(&trafficLightSouth, TL_RED);
        TL_SetState(&trafficLightNorth, TL_RED);
        TL_SetState(&trafficLightEast, TL_GREEN);
        PL_Off(&pedestrianLightSouth);
        PL_Off(&pedestrianLightEast);

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
        osDelay(5000);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

        while (osSemaphoreAcquire(northSouthSemaphore, 0) == osOK);
        LogEventTS("EmergencyTask_end");
        osSemaphoreRelease(northSouthSemaphore);
        osEventFlagsClear(eventFlagsId, EMG_FLAG);
    }
}

/*============================================================================*/
/*                                 EXTI_Callback                              */
/*============================================================================*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13)
    {
        LogEventTS("ISR_PED");
        osMutexAcquire(oledDisplayMutex, osWaitForever);
        ssd1306_Fill(Black);
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("Pedestrian Wait", Font_16x15, White);
        ssd1306_UpdateScreen();
        osMutexRelease(oledDisplayMutex);

        osEventFlagsSet(eventFlagsId, PED_FLAG_NS | PED_FLAG_EST);
    }
}

/*============================================================================*/
/*                                 SystemClock_Config                         */
/*============================================================================*/
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

/*============================================================================*/
/*                                 I2C1_Init                         		  */
/*============================================================================*/
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

/*============================================================================*/
/*                                 GPIO_Init                                  */
/*============================================================================*/
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
