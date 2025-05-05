/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (solo semafori, senza logica ambulanza)
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

/* Private define ------------------------------------------------------------*/
// Semaforo principale (porta A, CN8)
#define RED_PIN        GPIO_PIN_0   // PA0 (CN8-A0)
#define YELLOW_PIN     GPIO_PIN_1   // PA1 (CN8-A1)
#define GREEN_PIN      GPIO_PIN_4   // PA4 (CN8-A2)

// Semaforo secondario (porta C, CN10)
#define RED2_PIN       GPIO_PIN_9   // PC9 (CN10-1)
#define YELLOW2_PIN    GPIO_PIN_8   // PC8 (CN10-2)
#define GREEN2_PIN     GPIO_PIN_6   // PC6 (CN10-4)
#define SECOND_PORT    GPIOC

// Terzo semaforo (porta C, CN7)
#define RED3_PIN       GPIO_PIN_10  // PC10 (CN7-1)
#define YELLOW3_PIN    GPIO_PIN_11  // PC11 (CN7-2)
#define GREEN3_PIN     GPIO_PIN_12  // PC12 (CN7-3)
#define THIRD_PORT     GPIOC

// Durata fasi in ms
#define T_GREEN_MS     4000U
#define T_YELLOW_MS    1500U

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
COM_InitTypeDef BspCOMInit;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void *argument);
void SemaforoTask(void *argument);

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  /* Init scheduler */
  osKernelInitialize();

  /* Create tasks */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  xTaskCreate(SemaforoTask, "Semaforo", 128, NULL, tskIDLE_PRIORITY+1, NULL);

  /* Initialize onboard LED e button user (non usati per logica) */
  BSP_LED_Init(LED_GREEN);
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);

  /* Initialize COM port */
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

  /* Non dovrebbe mai arrivare qui */
  while (1) {}
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM       = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN       = 85;
  RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ       = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR       = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                   |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Enable GPIO Ports Clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Reset all traffic-light outputs */
  HAL_GPIO_WritePin(GPIOA, RED_PIN|YELLOW_PIN|GREEN_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SECOND_PORT, RED2_PIN|YELLOW2_PIN|GREEN2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(THIRD_PORT, RED3_PIN|YELLOW3_PIN|GREEN3_PIN, GPIO_PIN_RESET);

  /* Configure pins: Semaforo principale (PA0, PA1, PA4) */
  GPIO_InitStruct.Pin   = RED_PIN|YELLOW_PIN|GREEN_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure pins: Semaforo secondario (PC6, PC8, PC9) */
  GPIO_InitStruct.Pin = RED2_PIN|YELLOW2_PIN|GREEN2_PIN;
  HAL_GPIO_Init(SECOND_PORT, &GPIO_InitStruct);

  /* Configure pins: Terzo semaforo (PC10, PC11, PC12) */
  GPIO_InitStruct.Pin = RED3_PIN|YELLOW3_PIN|GREEN3_PIN;
  HAL_GPIO_Init(THIRD_PORT, &GPIO_InitStruct);

  /* Configura USER button come input digitale (non usato) */
  GPIO_InitStruct.Pin  = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief Task che gestisce i tre semafori con quattro fasi
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
  }
}

/**
  * @brief Thread di default (blocco minimo)
  */
void StartDefaultTask(void *argument)
{
  for (;;) { osDelay(1); }
}

/**
  * @brief Callback TIM6 (tick HAL)
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) HAL_IncTick();
}

/**
  * @brief Gestione errore
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
/**
  * @brief Riporta file e riga in caso di assert failure
  */
void assert_failed(uint8_t *file, uint32_t line)
{ }
#endif
