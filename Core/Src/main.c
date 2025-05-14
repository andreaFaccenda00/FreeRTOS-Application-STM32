/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Traffic lights with pedestrian crossing and synchronized NS phase
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "semphr.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

/* Defines -------------------------------------------------------------------*/
#define MAX_VEHICLES            20U
#define PRIORITY_THRESHOLD      17U

#define T_GREEN_MS              4000U
#define T_YELLOW_MS             1500U
#define T_GREEN_EXTENSION_MS    (T_GREEN_MS * 3/2)
#define T_PED_TOTAL_MS          5000U
#define T_PED_BLINK_MS          1000U
#define T_PED_BLINK_INTERVAL_MS 250U

// South GPIO (CN8)
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

// Emergency GPIO (CN)
#define EMERGENCY_LED_PORT GPIOB
#define EMERGENCY_LED_PIN  GPIO_PIN_7
#define PERIOD_EMERGENCY_MS       45000U
#define LED_ON_EMERGENCY_MS       2000U

// Pedestrian LED (PB5 → CN9-5 / D4)
#define P_LED_PIN       GPIO_PIN_5
#define P_LED_PORT      GPIOB

/* Globals -------------------------------------------------------------------*/
COM_InitTypeDef BspCOMInit;
SemaphoreHandle_t semNS;      // South+North phase
SemaphoreHandle_t semEst;     // East
static volatile bool pedPriority = false;

/* Task handles */
osThreadId_t nsTaskHandle;
osThreadId_t estTaskHandle;

const osThreadAttr_t nsTask_attributes = { .name = "NSTask", .priority = osPriorityAboveNormal, .stack_size = 128*4 };
const osThreadAttr_t estTask_attributes = { .name = "EstTask", .priority = osPriorityAboveNormal, .stack_size = 128*4 };
/* Emergency task handle e attributi */
osThreadId_t emergencyTaskHandle;
const osThreadAttr_t emergencyTask_attributes = {
    .name       = "EmergencyLED",
    .priority   = osPriorityRealtime,  // la più alta
    .stack_size = 128*4
};

/* Prototypes ----------------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void NSTask(void *argument);
void EstTask(void *argument);
void EmergencyTask(void *argument);
void Error_Handler(void);
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line);
#endif

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    srand(HAL_GetTick());
    osKernelInitialize();

    // Semaphores
    semNS  = xSemaphoreCreateBinary();
    semEst = xSemaphoreCreateCounting(2,0);
    if (!semNS || !semEst) Error_Handler();

    // Release initial NS phase
    xSemaphoreGive(semNS);

    // Init COM1 (unused by SWO)
    BspCOMInit.BaudRate   = 115200;
    BspCOMInit.WordLength = COM_WORDLENGTH_8B;
    BspCOMInit.StopBits   = COM_STOPBITS_1;
    BspCOMInit.Parity     = COM_PARITY_NONE;
    BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
    if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE) Error_Handler();

    // Create tasks
    nsTaskHandle  = osThreadNew(NSTask,  NULL, &nsTask_attributes);
    estTaskHandle = osThreadNew(EstTask, NULL, &estTask_attributes);
    emergencyTaskHandle = osThreadNew(
        EmergencyTask,
        NULL,
        &emergencyTask_attributes
    );

    osKernelStart();
    while(1);
}

void NSTask(void *argument)
{
    uint32_t vehiclesS, vehiclesN;
    for (;;)
    {
        // 1) attendo il via
        xSemaphoreTake(semNS, portMAX_DELAY);

        // 2) conto i veicoli
        vehiclesS = (rand() % MAX_VEHICLES) + 1;
        vehiclesN = (rand() % MAX_VEHICLES) + 1;
        uint32_t sec = osKernelGetTickCount() / 1000;

        // 3) calcolo durate (verde/giallo dimezzati in pedPriority)
        bool ped = pedPriority;
        uint32_t greenDur  = ped
                              ? (T_GREEN_MS  / 2)
                              : ((vehiclesS > PRIORITY_THRESHOLD || vehiclesN > PRIORITY_THRESHOLD)
                                  ? T_GREEN_EXTENSION_MS
                                  : T_GREEN_MS);
        uint32_t yellowDur = ped
                              ? (T_YELLOW_MS / 2)
                              : T_YELLOW_MS;

        // 4) log
        if (ped) {
            printf("%4lus | PED_PRIO | S=%2lu N=%2lu | VERDE(PRIO)\r\n",
                   sec, vehiclesS, vehiclesN);
        } else if (greenDur > T_GREEN_MS) {
            printf("%4lus | EXTEND   | S=%2lu N=%2lu | VERDE\r\n",
                   sec, vehiclesS, vehiclesN);
        } else {
            printf("%4lus | NS       | S=%2lu N=%2lu | VERDE\r\n",
                   sec, vehiclesS, vehiclesN);
        }

        // 5) fase VERDE
        HAL_GPIO_WritePin(S_PORT, S_GREEN_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(N_PORT, N_GREEN_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(S_PORT, S_RED_PIN|S_YELLOW_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(N_PORT, N_RED_PIN|N_YELLOW_PIN, GPIO_PIN_RESET);
        xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(greenDur));

        // 6) fase GIALLO
        sec = osKernelGetTickCount() / 1000;
        printf("%4lus | NS       | S=%2lu N=%2lu | GIALLO\r\n",
               sec, vehiclesS, vehiclesN);
        HAL_GPIO_WritePin(S_PORT, S_YELLOW_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(N_PORT, N_YELLOW_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(S_PORT, S_RED_PIN|S_GREEN_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(N_PORT, N_RED_PIN|N_GREEN_PIN, GPIO_PIN_RESET);
        xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(yellowDur));

        // 7) fase ROSSO + rilascio Est
        sec = osKernelGetTickCount() / 1000;
        printf("%4lus | NS       | S=%2lu N=%2lu | ROSSO\r\n",
               sec, vehiclesS, vehiclesN);
        HAL_GPIO_WritePin(S_PORT, S_RED_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(N_PORT, N_RED_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(S_PORT, S_YELLOW_PIN|S_GREEN_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(N_PORT, N_YELLOW_PIN|N_GREEN_PIN, GPIO_PIN_RESET);

        xSemaphoreGive(semEst);
        xSemaphoreGive(semEst);
    }
}

void EstTask(void *argument)
{
    uint32_t vehiclesE;
    for (;;)
    {
        // 1) aspetto i due semafori
        xSemaphoreTake(semEst, portMAX_DELAY);
        xSemaphoreTake(semEst, portMAX_DELAY);

        // 2) conto veicoli Est
        vehiclesE = (rand() % MAX_VEHICLES) + 1;
        uint32_t sec = osKernelGetTickCount() / 1000;

        // 3) calcolo durate
        bool ped = pedPriority;
        uint32_t greenDur  = ped
                              ? (T_GREEN_MS  / 2)
                              : (vehiclesE > PRIORITY_THRESHOLD
                                  ? T_GREEN_EXTENSION_MS
                                  : T_GREEN_MS);
        uint32_t yellowDur = ped
                              ? (T_YELLOW_MS / 2)
                              : T_YELLOW_MS;

        // 4) log
        if (ped) {
            printf("%4lus | PED_PRIO |     %2lu | VERDE(PRIO)\r\n",
                   sec, vehiclesE);
        } else if (greenDur > T_GREEN_MS) {
            printf("%4lus | EXTEND   |     %2lu | VERDE\r\n",
                   sec, vehiclesE);
        } else {
            printf("%4lus | Est      |     %2lu | VERDE\r\n",
                   sec, vehiclesE);
        }

        // 5) fase VERDE Est
        HAL_GPIO_WritePin(E_PORT, E_GREEN_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(E_PORT, E_RED_PIN|E_YELLOW_PIN, GPIO_PIN_RESET);
        xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(greenDur));

        // 6) fase GIALLO Est
        sec = osKernelGetTickCount() / 1000;
        printf("%4lus | Est      |     %2lu | GIALLO\r\n",
               sec, vehiclesE);
        HAL_GPIO_WritePin(E_PORT, E_YELLOW_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(E_PORT, E_RED_PIN|E_GREEN_PIN, GPIO_PIN_RESET);
        xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(yellowDur));

        // 7) fase ROSSO Est
        sec = osKernelGetTickCount() / 1000;
        printf("%4lus | Est      |     %2lu | ROSSO\r\n",
               sec, vehiclesE);
        HAL_GPIO_WritePin(E_PORT, E_RED_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(E_PORT, E_YELLOW_PIN|E_GREEN_PIN, GPIO_PIN_RESET);

        // 8) attraversamento pedonale
        sec = osKernelGetTickCount() / 1000;
        printf("%4lus | Ped      |      -  | ON\r\n", sec);
        HAL_GPIO_WritePin(P_LED_PORT, P_LED_PIN, GPIO_PIN_SET);
        osDelay(T_PED_TOTAL_MS - T_PED_BLINK_MS);

        sec = osKernelGetTickCount() / 1000;
        printf("%4lus | Ped      |      -  | BLINK\r\n", sec);
        for (uint32_t i = 0; i < (T_PED_BLINK_MS / T_PED_BLINK_INTERVAL_MS); ++i) {
            HAL_GPIO_TogglePin(P_LED_PORT, P_LED_PIN);
            osDelay(T_PED_BLINK_INTERVAL_MS);
        }
        HAL_GPIO_WritePin(P_LED_PORT, P_LED_PIN, GPIO_PIN_RESET);

        sec = osKernelGetTickCount() / 1000;
        printf("%4lus | Ped      |      -  | OFF\r\n", sec);

        // 9) fine priorità pedonale, riavvia NS
        pedPriority = false;
        xSemaphoreGive(semNS);
    }
}

void EmergencyTask(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(PERIOD_EMERGENCY_MS);

    for (;;)
    {
        // Sleep fino al prossimo intervallo fisso di 30 s
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Log ON
        uint32_t sec = osKernelGetTickCount() / 1000;
        printf("%4lus | EMERGENCY | ON\r\n", sec);

        // 1) Sospendi i task semafori
        vTaskSuspend(nsTaskHandle);
        vTaskSuspend(estTaskHandle);

        // 2) Metti tutti i semafori su ROSSO
        HAL_GPIO_WritePin(S_PORT, S_GREEN_PIN|S_YELLOW_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(S_PORT, S_RED_PIN,               GPIO_PIN_SET);

        HAL_GPIO_WritePin(N_PORT, N_GREEN_PIN|N_YELLOW_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(N_PORT, N_RED_PIN,                GPIO_PIN_SET);

        HAL_GPIO_WritePin(E_PORT, E_GREEN_PIN|E_YELLOW_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(E_PORT, E_RED_PIN,                GPIO_PIN_SET);

        // 3) Accendi LED blu
        HAL_GPIO_WritePin(EMERGENCY_LED_PORT, EMERGENCY_LED_PIN, GPIO_PIN_SET);

        // 4) Mantieni LED acceso per 2 s
        vTaskDelay(pdMS_TO_TICKS(LED_ON_EMERGENCY_MS));

        // 5) Spegni LED e log OFF
        HAL_GPIO_WritePin(EMERGENCY_LED_PORT, EMERGENCY_LED_PIN, GPIO_PIN_RESET);
        sec = osKernelGetTickCount() / 1000;
        printf("%4lus | EMERGENCY | OFF\r\n", sec);

        // 6) Ripulisci semafori e rilancia NS
        while (xSemaphoreTake(semNS,  0) == pdTRUE) {}
        while (xSemaphoreTake(semEst, 0) == pdTRUE) {}
        xSemaphoreGive(semNS);

        // 7) Riattiva i task semafori
        vTaskResume(nsTaskHandle);
        vTaskResume(estTaskHandle);
    }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_13)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    pedPriority = true;   // attiva la modalità priorità pedonale
    // Notifica entrambi i task in esecuzione per svegliarli istantaneamente
    vTaskNotifyGiveFromISR(nsTaskHandle,  &xHigherPriorityTaskWoken);
    vTaskNotifyGiveFromISR(estTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    printf("%4lus | %-6s |      -   | PED_PRIO\r\n",
           osKernelGetTickCount()/1000,
           "Button");
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

  /* Enable GPIO clocks */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Reset all outputs */
  HAL_GPIO_WritePin(S_PORT, S_RED_PIN|S_YELLOW_PIN|S_GREEN_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(P_LED_PORT, P_LED_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(E_PORT, E_RED_PIN|E_YELLOW_PIN|E_GREEN_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(N_PORT,  N_RED_PIN|N_YELLOW_PIN|N_GREEN_PIN, GPIO_PIN_RESET);

  /* Semaforo 1 (PA0, PA1, PA4) */
  GPIO_InitStruct.Pin   = S_RED_PIN|S_YELLOW_PIN|S_GREEN_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(S_PORT, &GPIO_InitStruct);

  /* LED pedonale (PB5) */
  GPIO_InitStruct.Pin = P_LED_PIN;
  HAL_GPIO_Init(P_LED_PORT, &GPIO_InitStruct);

  /* Semaforo 2 (PC6, PC8, PC9) */
  GPIO_InitStruct.Pin = E_RED_PIN|E_YELLOW_PIN|E_GREEN_PIN;
  HAL_GPIO_Init(E_PORT, &GPIO_InitStruct);

  /* Semaforo 3 (PC10, PC11, PC12) */
  GPIO_InitStruct.Pin = N_RED_PIN|N_YELLOW_PIN|N_GREEN_PIN;
  HAL_GPIO_Init(N_PORT, &GPIO_InitStruct);

  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin   = EMERGENCY_LED_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EMERGENCY_LED_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(EMERGENCY_LED_PORT, EMERGENCY_LED_PIN, GPIO_PIN_RESET);

  /* Button USER PC13 in EXTI mode */
  __HAL_RCC_GPIOC_CLK_ENABLE();              // già presente
  GPIO_InitStruct.Pin  = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // il pulsante tira PC13 a GND
  GPIO_InitStruct.Pull = GPIO_NOPULL;          // o PULLUP se preferisci
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


  /* NVIC for EXTI line[15:10] */
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
