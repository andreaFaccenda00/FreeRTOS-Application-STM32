/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Traffic lights with pedestrian crossing and synchronized NS phase
  *                   using CMSIS-RTOS2 API
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "trafficlight.h"

/* Defines -------------------------------------------------------------------*/
#define MAX_VEHICLES            20U
#define PRIORITY_THRESHOLD      15U

#define T_GREEN_MS              4000U
#define T_YELLOW_MS             1500U
#define T_GREEN_EXTENSION_MS    (T_GREEN_MS * 3/2)
#define T_PED_TOTAL_MS          5000U
#define T_PED_BLINK_MS          1000U
#define T_PED_BLINK_INTERVAL_MS 250U

#define PED_FLAG    (1U << 0)
#define IRQ_FLAG    (1U << 0)

/* South GPIO (CN8) */
#define S_RED_PIN       GPIO_PIN_0
#define S_YELLOW_PIN    GPIO_PIN_1
#define S_GREEN_PIN     GPIO_PIN_4
#define S_PORT          GPIOA

// North GPIO (CN7)
#define N_RED_PIN       GPIO_PIN_10 // CN7 Pin 1
#define N_YELLOW_PIN    GPIO_PIN_11 // CN7 Pin 2
#define N_GREEN_PIN     GPIO_PIN_12 // CN7 Pin 3
#define N_PORT          GPIOC

// East GPIO
#define E_RED_PIN       GPIO_PIN_9 // CN10 Pin 1
#define E_YELLOW_PIN    GPIO_PIN_8 // CN10 Pin 2
#define E_GREEN_PIN     GPIO_PIN_6 // CN10 Pin 4
#define E_PORT          GPIOC

/* Pedestrian South (PB5 → CN9-5 / D4) */
#define S_PED_PIN       GPIO_PIN_5
#define S_PED_PORT      GPIOB

/* Pedestrian East (PB7 → CN7-21) */
#define E_PED_PIN       GPIO_PIN_7
#define E_PED_PORT      GPIOB

/* Globals -------------------------------------------------------------------*/
static const TrafficLight_t tlSouth = { S_PORT, S_RED_PIN,    S_YELLOW_PIN,   S_GREEN_PIN   };
static const TrafficLight_t tlNorth = { N_PORT, N_RED_PIN,    N_YELLOW_PIN,   N_GREEN_PIN   };
static const TrafficLight_t tlEast  = { E_PORT, E_RED_PIN,    E_YELLOW_PIN,   E_GREEN_PIN   };
static const PedLight_t plSouth    = { S_PED_PORT, S_PED_PIN };
static const PedLight_t plEast     = { E_PED_PORT, E_PED_PIN  };

static const TrafficLight_t allLights[] = {
    { S_PORT, S_RED_PIN,    S_YELLOW_PIN,   S_GREEN_PIN   },
    { N_PORT, N_RED_PIN,    N_YELLOW_PIN,   N_GREEN_PIN   },
    { E_PORT, E_RED_PIN,    E_YELLOW_PIN,   E_GREEN_PIN   },
};

COM_InitTypeDef       BspCOMInit;
osSemaphoreId_t       semNS, semEst, semPed;
static osEventFlagsId_t pedFlags;
osThreadId_t          nsTaskHandle, estTaskHandle, pedTaskHandle;

typedef enum { PHASE_NS, PHASE_EST, PHASE_PED } Phase_t;
static volatile Phase_t currentPhase = PHASE_NS;

const osThreadAttr_t nsTask_attributes  = { .name = "NSTask",  .priority = osPriorityAboveNormal,    .stack_size = 128*4 };
const osThreadAttr_t estTask_attributes = { .name = "EstTask", .priority = osPriorityAboveNormal,    .stack_size = 128*4 };
const osThreadAttr_t pedTaskAttr        = { .name = "PedTask", .priority = osPriorityAboveNormal1,  .stack_size = 128*4 };

/* Prototypes ----------------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void NSTask(void *argument);
void EstTask(void *argument);
void PedTask(void *argument);
void Error_Handler(void);

/*============================================================================*/
/*                                  MAIN                                      */
/*============================================================================*/
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    srand(HAL_GetTick());
    osKernelInitialize();

    pedFlags = osEventFlagsNew(NULL);
    if (pedFlags == NULL) {
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

    osKernelStart();
    while (1) {  }
}

/*============================================================================*/
/*                          EXTI CALLBACK                                     */
/*============================================================================*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13)
    {
        osEventFlagsSet(pedFlags, PED_FLAG);

        if (currentPhase == PHASE_NS) {
            osThreadFlagsSet(nsTaskHandle, IRQ_FLAG);
        } else if (currentPhase == PHASE_EST) {
            osThreadFlagsSet(estTaskHandle, IRQ_FLAG);
        }

        uint32_t sec = osKernelGetTickCount()/1000;
        printf("%4lus | Button   |      -    | PED_PRIO\r\n", sec);
    }
}

/*============================================================================*/
/*                                 NSTask                                      */
/*============================================================================*/
void NSTask(void *argument)
{
    uint32_t vehiclesS, vehiclesN, sec;
    for (;;)
    {
        osSemaphoreAcquire(semNS, osWaitForever);

        vehiclesS = (rand() % MAX_VEHICLES) + 1;
        vehiclesN = (rand() % MAX_VEHICLES) + 1;
        sec = osKernelGetTickCount() / 1000;

        currentPhase = PHASE_NS;
        osThreadFlagsClear(IRQ_FLAG);

        bool ped = (osEventFlagsGet(pedFlags) & PED_FLAG) != 0;
        uint32_t greenMs = ped ? (T_GREEN_MS / 2)
                               : ((vehiclesS > PRIORITY_THRESHOLD || vehiclesN > PRIORITY_THRESHOLD)
                                  ? T_GREEN_EXTENSION_MS
                                  : T_GREEN_MS);
        uint32_t yellowMs = ped ? (T_YELLOW_MS / 2) : T_YELLOW_MS;

        if (ped) {
            printf("%4lus | NS       | S=%2lu N=%2lu | VERDE(PRIO)\r\n", sec, vehiclesS, vehiclesN);
        } else if (greenMs > T_GREEN_MS) {
            printf("%4lus | NS       | S=%2lu N=%2lu | EXTEND\r\n", sec, vehiclesS, vehiclesN);
        } else {
            printf("%4lus | NS       | S=%2lu N=%2lu | VERDE\r\n", sec, vehiclesS, vehiclesN);
        }

        TL_SetState(&tlSouth, TL_GREEN);
        TL_SetState(&tlNorth, TL_GREEN);
        TL_SetState(&tlEast,   TL_RED);

        uint32_t flags = osThreadFlagsWait(IRQ_FLAG, osFlagsWaitAny, greenMs);
        if (flags & IRQ_FLAG) {
            sec = osKernelGetTickCount() / 1000;
            printf("%4lus | NS       | S=%2lu N=%2lu | GIALLO(INT)\r\n", sec, vehiclesS, vehiclesN);
            TL_SetState(&tlSouth, TL_YELLOW);
            TL_SetState(&tlNorth, TL_YELLOW);
            osDelay(yellowMs);
            sec = osKernelGetTickCount() / 1000;
            printf("%4lus | NS       | S=%2lu N=%2lu | ROSSO\r\n", sec, vehiclesS, vehiclesN);
            TL_SetState(&tlSouth, TL_RED);
            TL_SetState(&tlNorth, TL_RED);
            osSemaphoreRelease(semPed);
            continue;
        }

        sec = osKernelGetTickCount() / 1000;
        printf("%4lus | NS       | S=%2lu N=%2lu | GIALLO\r\n", sec, vehiclesS, vehiclesN);
        TL_SetState(&tlSouth, TL_YELLOW);
        TL_SetState(&tlNorth, TL_YELLOW);
        osDelay(yellowMs);
        sec = osKernelGetTickCount() / 1000;
        printf("%4lus | NS       | S=%2lu N=%2lu | ROSSO\r\n", sec, vehiclesS, vehiclesN);
        TL_SetState(&tlSouth, TL_RED);
        TL_SetState(&tlNorth, TL_RED);
        osSemaphoreRelease(semEst);
        osSemaphoreRelease(semEst);
    }
}

/*============================================================================*/
/*                                 EstTask                                     */
/*============================================================================*/
void EstTask(void *argument)
{
    uint32_t vehiclesE, sec;
    for (;;)
    {
        osSemaphoreAcquire(semEst, osWaitForever);
        osSemaphoreAcquire(semEst, osWaitForever);

        vehiclesE = (rand() % MAX_VEHICLES) + 1;
        sec = osKernelGetTickCount() / 1000;

        currentPhase = PHASE_EST;
        osThreadFlagsClear(IRQ_FLAG);

        bool ped = (osEventFlagsGet(pedFlags) & PED_FLAG) != 0;
        uint32_t greenMs = ped ? (T_GREEN_MS / 2)
                              : (vehiclesE > PRIORITY_THRESHOLD
                                 ? T_GREEN_EXTENSION_MS
                                 : T_GREEN_MS);
        uint32_t yellowMs = ped ? (T_YELLOW_MS / 2) : T_YELLOW_MS;

        if (ped) {
            printf("%4lus | Est      |     %2lu | VERDE(PRIO)\r\n", sec, vehiclesE);
        } else if (greenMs > T_GREEN_MS) {
            printf("%4lus | Est      |     %2lu | EXTEND\r\n", sec, vehiclesE);
        } else {
            printf("%4lus | Est      |     %2lu | VERDE\r\n", sec, vehiclesE);
        }

        TL_SetState(&tlEast, TL_GREEN);

        uint32_t flags = osThreadFlagsWait(IRQ_FLAG, osFlagsWaitAny, greenMs);
        if (flags & IRQ_FLAG) {
            sec = osKernelGetTickCount() / 1000;
            printf("%4lus | Est      |     %2lu | GIALLO(INT)\r\n", sec, vehiclesE);
            TL_SetState(&tlEast, TL_YELLOW);
            osDelay(yellowMs);
            sec = osKernelGetTickCount() / 1000;
            printf("%4lus | Est      |     %2lu | ROSSO\r\n", sec, vehiclesE);
            TL_SetState(&tlEast, TL_RED);
            osSemaphoreRelease(semPed);
            continue;
        }

        sec = osKernelGetTickCount() / 1000;
        printf("%4lus | Est      |     %2lu | GIALLO\r\n", sec, vehiclesE);
        TL_SetState(&tlEast, TL_YELLOW);
        osDelay(yellowMs);
        sec = osKernelGetTickCount() / 1000;
        printf("%4lus | Est      |     %2lu | ROSSO\r\n", sec, vehiclesE);
        TL_SetState(&tlEast, TL_RED);
        osSemaphoreRelease(semPed);
    }
}

/*============================================================================*/
/*                                 PedTask                                     */
/*============================================================================*/
void PedTask(void *argument)
{
    for (;;)
    {
        osSemaphoreAcquire(semPed, osWaitForever);

        uint32_t sec = osKernelGetTickCount() / 1000;
        printf("%4lus | Ped      |      -  | ON\r\n", sec);
        PL_On(&plSouth);
        PL_On(&plEast);
        osDelay(T_PED_TOTAL_MS - T_PED_BLINK_MS);

        printf("%4lus | Ped      |      -  | BLINK\r\n", osKernelGetTickCount() / 1000);
        for (uint32_t i = 0; i < T_PED_BLINK_MS / T_PED_BLINK_INTERVAL_MS; ++i) {
            PL_Toggle(&plSouth);
            PL_Toggle(&plEast);
            osDelay(T_PED_BLINK_INTERVAL_MS);
        }

        PL_Off(&plSouth);
        PL_Off(&plEast);
        printf("%4lus | Ped      |      -  | OFF\r\n", osKernelGetTickCount() / 1000);

        osEventFlagsClear(pedFlags, PED_FLAG);
        currentPhase = PHASE_PED;
        osSemaphoreRelease(semNS);
    }
}

/**
  * @brief  System Clock Configuration
  */
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

/**
  * @brief  GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = S_RED_PIN | S_YELLOW_PIN | S_GREEN_PIN;
  HAL_GPIO_Init(S_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = N_RED_PIN | N_YELLOW_PIN | N_GREEN_PIN;
  HAL_GPIO_Init(N_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = E_RED_PIN | E_YELLOW_PIN | E_GREEN_PIN;
  HAL_GPIO_Init(E_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = S_PED_PIN;
  HAL_GPIO_Init(S_PED_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = E_PED_PIN;
  HAL_GPIO_Init(E_PED_PORT, &GPIO_InitStruct);

  for (size_t i = 0; i < sizeof(allLights)/sizeof(allLights[0]); ++i) {
    TL_Init(&allLights[i]);
  }
  PL_Init(&plSouth);
  PL_Init(&plEast);

  HAL_GPIO_WritePin(S_PED_PORT, S_PED_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(E_PED_PORT, E_PED_PIN, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin  = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports file name and line number
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* Optional: insert debug print here */
}
#endif
