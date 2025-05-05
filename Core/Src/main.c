/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (semafori + pedonale con lampeggio finale)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private defines -----------------------------------------------------------*/
// Semaforo principale (porta A, CN8)
#define RED_PIN        GPIO_PIN_0
#define YELLOW_PIN     GPIO_PIN_1
#define GREEN_PIN      GPIO_PIN_4

// Semaforo secondario (porta C, CN10)
#define RED2_PIN       GPIO_PIN_9
#define YELLOW2_PIN    GPIO_PIN_8
#define GREEN2_PIN     GPIO_PIN_6
#define SECOND_PORT    GPIOC

// Terzo semaforo (porta C, CN7)
#define RED3_PIN       GPIO_PIN_10
#define YELLOW3_PIN    GPIO_PIN_11
#define GREEN3_PIN     GPIO_PIN_12
#define THIRD_PORT     GPIOC

// LED pedonale (porta B, PB5 → CN9-5 / D4)
#define PED_PORT                GPIOB
#define PED_PIN                 GPIO_PIN_5

// Durate pedonale
#define T_PED_TOTAL_MS          5000U      // durata complessiva 5 s
#define T_PED_BLINK_MS          1000U      // ultimi 1 s lampeggio
#define T_PED_BLINK_INTERVAL_MS 250U       // intervallo lampeggio 250 ms

// Durata fasi veicolari in ms
#define T_GREEN_MS     4000U
#define T_YELLOW_MS    1500U

/* Private variables ---------------------------------------------------------*/
COM_InitTypeDef BspCOMInit;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name       = "defaultTask",
  .priority   = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* Definitions for semaforoTask */
osThreadId_t semaforoTaskHandle;
const osThreadAttr_t semaforoTask_attributes = {
  .name       = "semaforoTask",
  .priority   = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void *argument);
void SemaforoTask(void *argument);

/**
  * @brief  The application entry point.
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  /* Init RTOS */
  osKernelInitialize();

  /* Create threads */
  defaultTaskHandle  = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  semaforoTaskHandle = osThreadNew(SemaforoTask,    NULL, &semaforoTask_attributes);

  /* Init onboard LED and user button */
  BSP_LED_Init(LED_GREEN);
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Init COM1 */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE) {
    Error_Handler();
  }

  /* Start scheduler */
  osKernelStart();
  /* Should never get here */
  while (1) {}
}

/**
  * @brief  Task che gestisce i tre semafori + pedonale (5 fasi)
  */
void SemaforoTask(void *argument)
{
  TickType_t lastWake = xTaskGetTickCount();

  for (;;)
  {
    // Fase 1: sem1+3 VERDI, sem2 ROSSO
    HAL_GPIO_WritePin(GPIOA, GREEN_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, RED_PIN|YELLOW_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SECOND_PORT, RED2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SECOND_PORT, YELLOW2_PIN|GREEN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(THIRD_PORT, GREEN3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(THIRD_PORT, RED3_PIN|YELLOW3_PIN, GPIO_PIN_RESET);
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(T_GREEN_MS));

    // Fase 2: sem1+3 GIALLO, sem2 ROSSO
    HAL_GPIO_WritePin(GPIOA, YELLOW_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, RED_PIN|GREEN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SECOND_PORT, RED2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SECOND_PORT, YELLOW2_PIN|GREEN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(THIRD_PORT, YELLOW3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(THIRD_PORT, RED3_PIN|GREEN3_PIN, GPIO_PIN_RESET);
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(T_YELLOW_MS));

    // Fase 3: sem1+3 ROSSI, sem2 VERDI
    HAL_GPIO_WritePin(GPIOA, RED_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, YELLOW_PIN|GREEN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SECOND_PORT, GREEN2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SECOND_PORT, RED2_PIN|YELLOW2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(THIRD_PORT, RED3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(THIRD_PORT, YELLOW3_PIN|GREEN3_PIN, GPIO_PIN_RESET);
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(T_GREEN_MS));

    // Fase 4: sem1+3 ROSSI, sem2 GIALLO
    HAL_GPIO_WritePin(GPIOA, RED_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, YELLOW_PIN|GREEN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SECOND_PORT, YELLOW2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SECOND_PORT, RED2_PIN|GREEN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(THIRD_PORT, RED3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(THIRD_PORT, YELLOW3_PIN|GREEN3_PIN, GPIO_PIN_RESET);
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(T_YELLOW_MS));

    // Fase 5: ALL-RED + Pedonale con lampeggio finale
    // — tutti i veicoli rossi —
    HAL_GPIO_WritePin(GPIOA, RED_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, YELLOW_PIN|GREEN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SECOND_PORT, RED2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SECOND_PORT, YELLOW2_PIN|GREEN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(THIRD_PORT, RED3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(THIRD_PORT, YELLOW3_PIN|GREEN3_PIN, GPIO_PIN_RESET);

    // — pedonale acceso fisso per T_PED_TOTAL_MS - T_PED_BLINK_MS —
    HAL_GPIO_WritePin(PED_PORT, PED_PIN, GPIO_PIN_SET);
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(T_PED_TOTAL_MS - T_PED_BLINK_MS));

    // — lampeggio negli ultimi T_PED_BLINK_MS —
    {
      const uint32_t toggles = T_PED_BLINK_MS / T_PED_BLINK_INTERVAL_MS;
      for (uint32_t i = 0; i < toggles; ++i) {
        HAL_GPIO_TogglePin(PED_PORT, PED_PIN);
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(T_PED_BLINK_INTERVAL_MS));
      }
    }
    // assicurarsi pedonale spento
    HAL_GPIO_WritePin(PED_PORT, PED_PIN, GPIO_PIN_RESET);
  }
}

/**
  * @brief  Function implementing the defaultTask thread.
  */
void StartDefaultTask(void *argument)
{
  for (;;)
  {
    osDelay(1);
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
  HAL_GPIO_WritePin(GPIOA, RED_PIN|YELLOW_PIN|GREEN_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PED_PORT, PED_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SECOND_PORT, RED2_PIN|YELLOW2_PIN|GREEN2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(THIRD_PORT,  RED3_PIN|YELLOW3_PIN|GREEN3_PIN, GPIO_PIN_RESET);

  /* Semaforo 1 (PA0, PA1, PA4) */
  GPIO_InitStruct.Pin   = RED_PIN|YELLOW_PIN|GREEN_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* LED pedonale (PB5) */
  GPIO_InitStruct.Pin = PED_PIN;
  HAL_GPIO_Init(PED_PORT, &GPIO_InitStruct);

  /* Semaforo 2 (PC6, PC8, PC9) */
  GPIO_InitStruct.Pin = RED2_PIN|YELLOW2_PIN|GREEN2_PIN;
  HAL_GPIO_Init(SECOND_PORT, &GPIO_InitStruct);

  /* Semaforo 3 (PC10, PC11, PC12) */
  GPIO_InitStruct.Pin = RED3_PIN|YELLOW3_PIN|GREEN3_PIN;
  HAL_GPIO_Init(THIRD_PORT, &GPIO_InitStruct);

  /* USER button (EXTI) */
  GPIO_InitStruct.Pin   = B1_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief  HAL tick callback (TIM6)
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
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
  * @brief  Reports the name of the source file and the source line number
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add debug printing here */
}
#endif
