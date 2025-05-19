/*============================================================================*/
/*       Traffic lights with pedestrian crossing and synchronized NS phase    */
/*                       using CMSIS-RTOS2 API                                */
/*============================================================================*/

/*============================================================================*/
/*                                  INCLUDES                                  */
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
/*============================================================================*/
/*                                  GLOBAL                                    */
/*============================================================================*/
static const TrafficLight_t tlSouth = {
    .port       = TL_SOUTH_RED.port,
    .pin_red    = TL_SOUTH_RED.pin,
    .pin_yellow = TL_SOUTH_YELLOW.pin,
    .pin_green  = TL_SOUTH_GREEN.pin
};

static const TrafficLight_t tlNorth = {
    .port       = TL_NORTH_RED.port,
    .pin_red    = TL_NORTH_RED.pin,
    .pin_yellow = TL_NORTH_YELLOW.pin,
    .pin_green  = TL_NORTH_GREEN.pin
};

static const TrafficLight_t tlEast = {
    .port       = TL_EAST_RED.port,
    .pin_red    = TL_EAST_RED.pin,
    .pin_yellow = TL_EAST_YELLOW.pin,
    .pin_green  = TL_EAST_GREEN.pin
};

static const PedLight_t plSouth = {
    .port = PED_SOUTH.port,
    .pin  = PED_SOUTH.pin
};

static const PedLight_t plEast = {
    .port = PED_EAST.port,
    .pin  = PED_EAST.pin
};

COM_InitTypeDef       BspCOMInit;
I2C_HandleTypeDef hi2c1;
osSemaphoreId_t       semNS, semEst, semPed;
static osEventFlagsId_t flagsId;
osThreadId_t          nsTaskHandle, estTaskHandle, pedTaskHandle, emergencyTaskHandle;
osMutexId_t           oledMutex;

typedef enum { PHASE_NS, PHASE_EST, PHASE_PED } Phase_t;
static volatile Phase_t currentPhase = PHASE_NS;

const osThreadAttr_t nsTask_attributes  = { .name = "NSTask",  .priority = osPriorityAboveNormal,    .stack_size = 128*4 };
const osThreadAttr_t estTask_attributes = { .name = "EstTask", .priority = osPriorityAboveNormal,    .stack_size = 128*4 };
const osThreadAttr_t pedTaskAttr        = { .name = "PedTask", .priority = osPriorityAboveNormal1,  .stack_size = 128*4 };
const osThreadAttr_t emergencyTaskAttr  = { .name = "EmergencyTask", .priority = osPriorityRealtime,   .stack_size = 128*4 };

/*============================================================================*/
/*                                  PROTOTYPES                                */
/*============================================================================*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void NSTask(void *argument);
void EstTask(void *argument);
void PedTask(void *argument);
void Error_Handler(void);
void EmergencyTask(void *argument);
static void MX_I2C1_Init(void);

/*============================================================================*/
/*                                  MAIN                                      */
/*============================================================================*/
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();

    ssd1306_Init();    /* ←  driver hi2c1 */

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
        __HAL_RCC_CLEAR_RESET_FLAGS();
        printf("*** Reset causato da IWDG ***\r\n");
    }
    MX_IWDG_Init(); 	/* ← inizialize watchdog */

    CoreDebug->DEMCR   |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL          |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t seed = DWT->CYCCNT;
    srand(seed);
    osKernelInitialize();

    flagsId = osEventFlagsNew(NULL);
    if (flagsId == NULL) Error_Handler();

    oledMutex = osMutexNew(NULL);
    if (oledMutex == NULL) {
        Error_Handler();
    }

    semNS  = osSemaphoreNew(1, 1, NULL);
    semEst = osSemaphoreNew(2, 0, NULL);
    semPed = osSemaphoreNew(1, 0, NULL);
    if (!semNS || !semEst || !semPed) {
        Error_Handler();
    }

    BspCOMInit.BaudRate   = 115200;
    BspCOMInit.WordLength = COM_WORDLENGTH_8B;
    BspCOMInit.StopBits   = COM_STOPBITS_1;
    BspCOMInit.Parity     = COM_PARITY_NONE;
    BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
    if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE) {
        Error_Handler();
    }

    nsTaskHandle  = osThreadNew(NSTask,  NULL, &nsTask_attributes);
    estTaskHandle = osThreadNew(EstTask, NULL, &estTask_attributes);
    pedTaskHandle = osThreadNew(PedTask, NULL, &pedTaskAttr);
    emergencyTaskHandle = osThreadNew(EmergencyTask, NULL, &emergencyTaskAttr);

    osKernelStart();
    while (1) {  }
}

/*============================================================================*/
/*                                  EmergencyTask                             */
/*============================================================================*/
void EmergencyTask(void *argument)
{
    const uint32_t period_ms = 25000;
    for (;;)
    {
        osDelay(period_ms);

		osMutexAcquire(oledMutex, osWaitForever);
		ssd1306_Fill(Black);
		ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("Emergency", Font_16x15, White);
		ssd1306_UpdateScreen();
	    osMutexRelease(oledMutex);

        osEventFlagsSet(flagsId, EMG_FLAG);

        TL_SetState(&tlSouth, TL_RED);
        TL_SetState(&tlNorth, TL_RED);
        TL_SetState(&tlEast,   TL_GREEN);

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
        osDelay(5000);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

        while (osSemaphoreAcquire(semNS, 0) == osOK);
        osSemaphoreRelease(semNS);

        osEventFlagsClear(flagsId, EMG_FLAG);

    }
}

/*============================================================================*/
/*                          EXTI CALLBACK                                     */
/*============================================================================*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13)
    {
		osMutexAcquire(oledMutex, osWaitForever);
		ssd1306_Fill(Black);
		ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("Pedestrian Wait", Font_16x15, White);
		ssd1306_UpdateScreen();
	    osMutexRelease(oledMutex);

        LogEvent("Button", "PED_PRIO", 0, 0, 0);

    }
}

/*============================================================================*/
/*                                 NSTask                                     */
/*============================================================================*/
void NSTask(void *argument)
{
    uint32_t vehiclesS, vehiclesN;
    for (;;)
    {
        osSemaphoreAcquire(semNS, osWaitForever);

        if (osEventFlagsGet(flagsId) & EMG_FLAG) {
            osEventFlagsClear(flagsId, EMG_FLAG);
            continue;
        }

        vehiclesS = (rand() % MAX_VEHICLES) + 1;
        vehiclesN = (rand() % MAX_VEHICLES) + 1;

        char buf[24];

		osMutexAcquire(oledMutex, osWaitForever);
		ssd1306_Fill(Black);
        sprintf(buf, "S: %2lu Cars", (unsigned long)vehiclesS);
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString(buf, Font_16x15, White);
        sprintf(buf, "N: %2lu Cars", (unsigned long)vehiclesN);
        ssd1306_SetCursor(0, 22);
        ssd1306_WriteString(buf, Font_16x15, White);
		ssd1306_UpdateScreen();
	    osMutexRelease(oledMutex);

	    bool pedReq = false;
        uint32_t greenMs  = pedReq
                           ? (T_GREEN_MS/2)
                           : ((vehiclesS > PRIORITY_THRESHOLD || vehiclesN > PRIORITY_THRESHOLD)
                              ? T_GREEN_EXTENSION_MS
                              : T_GREEN_MS);
        uint32_t yellowMs = pedReq ? (T_YELLOW_MS/2) : T_YELLOW_MS;

        TL_SetState(&tlSouth, TL_GREEN);
        TL_SetState(&tlNorth, TL_GREEN);
        TL_SetState(&tlEast,   TL_RED);

        uint32_t flags = osEventFlagsWait(
            flagsId,
            EMG_FLAG, //| PED_FLAG,
            osFlagsWaitAny,
            greenMs
        );
        if (flags & EMG_FLAG) {
            continue;
        }

        LogEvent("NS","GIALLO",vehiclesS,vehiclesN,0);
        TL_SetState(&tlSouth, TL_YELLOW);
        TL_SetState(&tlNorth, TL_YELLOW);

        flags = osEventFlagsWait(
            flagsId,
            EMG_FLAG,
            osFlagsWaitAny,
            yellowMs
        );
        if (flags & EMG_FLAG) { continue; }

        LogEvent("NS","ROSSO",vehiclesS,vehiclesN,0);
        TL_SetState(&tlSouth, TL_RED);
        TL_SetState(&tlNorth, TL_RED);

        flags = osEventFlagsWait(
            flagsId,
            EMG_FLAG,
            osFlagsWaitAny,
            1000U
        );
        if (flags & EMG_FLAG) { continue; }

        osSemaphoreRelease(semEst);
        osSemaphoreRelease(semEst);
    }
}

/*============================================================================*/
/*                                 EstTask                                    */
/*============================================================================*/
void EstTask(void *argument)
{
    uint32_t vehiclesE;
    for (;;)
    {
        osSemaphoreAcquire(semEst, osWaitForever);
        osSemaphoreAcquire(semEst, osWaitForever);

        if (osEventFlagsGet(flagsId) & EMG_FLAG) {
            osEventFlagsClear(flagsId, EMG_FLAG);
            continue;
        }

        vehiclesE = (rand() % MAX_VEHICLES) + 1;

        char buf[24];

		osMutexAcquire(oledMutex, osWaitForever);
		ssd1306_Fill(Black);
        sprintf(buf, "E: %2lu Cars", (unsigned long)vehiclesE);
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString(buf, Font_16x15, White);
		ssd1306_UpdateScreen();
	    osMutexRelease(oledMutex);

        bool pedReq = false;
        uint32_t greenMs  = pedReq
                           ? (T_GREEN_MS/2)
                           : ((vehiclesE > PRIORITY_THRESHOLD)
                              ? T_GREEN_EXTENSION_MS
                              : T_GREEN_MS);
        uint32_t yellowMs = pedReq ? (T_YELLOW_MS/2) : T_YELLOW_MS;

        TL_SetState(&tlEast, TL_GREEN);

        uint32_t flags = osEventFlagsWait(
            flagsId,
            EMG_FLAG ,//| PED_FLAG,
            osFlagsWaitAny,
            greenMs
        );
        if (flags & EMG_FLAG) {
            continue;
        }

        LogEvent("Est","GIALLO",0,0,vehiclesE);
        TL_SetState(&tlEast, TL_YELLOW);

        flags = osEventFlagsWait(
            flagsId,
            EMG_FLAG,
            osFlagsWaitAny,
            yellowMs
        );
        if (flags & EMG_FLAG) { continue; }

        LogEvent("Est","ROSSO",0,0,vehiclesE);
        TL_SetState(&tlEast, TL_RED);

        flags = osEventFlagsWait(
            flagsId,
            EMG_FLAG,
            osFlagsWaitAny,
            1000U
        );
        if (flags & EMG_FLAG) { continue; }

        osSemaphoreRelease(semPed);
    }
}

/*============================================================================*/
/*                                 PedTask                                    */
/*============================================================================*/
void PedTask(void *argument)
{
    for (;;)
    {
        osSemaphoreAcquire(semPed, osWaitForever);

        if (osEventFlagsGet(flagsId) & EMG_FLAG) {
            osEventFlagsClear(flagsId, EMG_FLAG);
            continue;
        }

        LogEvent("Ped","ON",0,0,0);
        PL_On(&plSouth);
        PL_On(&plEast);

        uint32_t t1 = T_PED_TOTAL_MS - T_PED_BLINK_MS;
        uint32_t flags = osEventFlagsWait(
            flagsId,
            EMG_FLAG,
            osFlagsWaitAny,
            t1
        );
        if (flags & EMG_FLAG) { continue; }

        LogEvent("Ped","BLINK",0,0,0);
        for (uint32_t i = 0; i < T_PED_BLINK_MS / T_PED_BLINK_INTERVAL_MS; ++i) {
            if (osEventFlagsGet(flagsId) & EMG_FLAG) {
                break;
            }
            PL_Toggle(&plSouth);
            PL_Toggle(&plEast);
            osDelay(T_PED_BLINK_INTERVAL_MS);
        }

        PL_Off(&plSouth);
        PL_Off(&plEast);
        LogEvent("Ped","OFF",0,0,0);

        flags = osEventFlagsWait(
            flagsId,
            EMG_FLAG,
            osFlagsWaitAny,
            1000U
        );
        if (flags & EMG_FLAG) { continue; }

        osSemaphoreRelease(semNS);
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

    TL_Init(&tlSouth);
    TL_Init(&tlNorth);
    TL_Init(&tlEast);
    PL_Init(&plSouth);
    PL_Init(&plEast);

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
