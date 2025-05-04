/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "event_groups.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// semaforo principale (porta A)
#define RED_PIN        GPIO_PIN_0   // PA0
#define YELLOW_PIN     GPIO_PIN_1   // PA1
#define GREEN_PIN      GPIO_PIN_4   // PA4

// semaforo secondario (porta C)
#define RED2_PIN       GPIO_PIN_9   // PC9
#define YELLOW2_PIN    GPIO_PIN_8   // PC8
#define GREEN2_PIN     GPIO_PIN_6   // PC6
#define SECOND_PORT    GPIOC

// LED ambulanza (porta B)
#define BLUE_PIN       GPIO_PIN_0   // PB0
#define BLUE_PORT      GPIOB

// bit per modalità ambulanza
#define BIT_AMB_REQ    (1 << 0)
#define BIT_AMB_DONE   (1 << 1)

// durata fasi in ms
#define T_RED_MS       4000U
#define T_YELLOW_MS    1500U
#define T_GREEN_MS     4000U

typedef struct {
  TickType_t duration;      // durata in tick
  uint16_t   setPins;       // GPIO da accendere sul semaforo principale
  uint16_t   resetPins;     // GPIO da spegnere sul semaforo principale
  UBaseType_t nextPhase;    // indice della fase successiva
} Phase_t;

static const Phase_t phases[] = {
  { pdMS_TO_TICKS(T_RED_MS),    RED_PIN,                YELLOW_PIN|GREEN_PIN, 1 },
  { pdMS_TO_TICKS(T_YELLOW_MS), YELLOW_PIN,             RED_PIN|GREEN_PIN,    2 },
  { pdMS_TO_TICKS(T_GREEN_MS),  GREEN_PIN,              RED_PIN|YELLOW_PIN,   3 },
  { pdMS_TO_TICKS(T_GREEN_MS),  GREEN_PIN,              RED_PIN|YELLOW_PIN,   4 },
  { pdMS_TO_TICKS(T_YELLOW_MS), YELLOW_PIN,             RED_PIN|GREEN_PIN,    0 },
};
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
COM_InitTypeDef BspCOMInit;
static EventGroupHandle_t semaEventGroup;

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
void AmbulanceTask(void *argument);

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  /* Initialize scheduler */
  osKernelInitialize();
  semaEventGroup = xEventGroupCreate();

  /* Create tasks */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  xTaskCreate(SemaforoTask,   "Semaforo",   128, NULL, tskIDLE_PRIORITY + 1, NULL);
  xTaskCreate(AmbulanceTask,  "Ambulanza",  128, NULL, tskIDLE_PRIORITY + 2, NULL);

  /* Initialize onboard resources */
  BSP_LED_Init(LED_GREEN);
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

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

  /* Infinite loop */
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
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
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

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Reset outputs
  HAL_GPIO_WritePin(GPIOA, RED_PIN|YELLOW_PIN|GREEN_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, BLUE_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SECOND_PORT, RED2_PIN|YELLOW2_PIN|GREEN2_PIN, GPIO_PIN_RESET);

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  // semaforo principale
  GPIO_InitStruct.Pin = RED_PIN|YELLOW_PIN|GREEN_PIN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // LED ambulanza
  GPIO_InitStruct.Pin = BLUE_PIN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // semaforo secondario
  GPIO_InitStruct.Pin = RED2_PIN|YELLOW2_PIN|GREEN2_PIN;
  HAL_GPIO_Init(SECOND_PORT, &GPIO_InitStruct);

  // USER button
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void SemaforoTask(void *argument)
{
  UBaseType_t phase = 0;
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    EventBits_t bits = xEventGroupGetBits(semaEventGroup);
    if (bits & BIT_AMB_REQ) {
      // semaforo 1 rosso, semaforo 2 verde
      HAL_GPIO_WritePin(GPIOA, RED_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOA, YELLOW_PIN|GREEN_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SECOND_PORT, RED2_PIN|YELLOW2_PIN|GREEN2_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SECOND_PORT, GREEN2_PIN, GPIO_PIN_SET);
      xEventGroupWaitBits(semaEventGroup, BIT_AMB_DONE, pdTRUE, pdTRUE, portMAX_DELAY);
      phase = 1;
      lastWake = xTaskGetTickCount();
      continue;
    }
    // ciclo normale
    HAL_GPIO_WritePin(GPIOA, phases[phase].setPins, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, phases[phase].resetPins, GPIO_PIN_RESET);
    // secondo semaforo invertito
    HAL_GPIO_WritePin(SECOND_PORT, RED2_PIN|YELLOW2_PIN|GREEN2_PIN, GPIO_PIN_RESET);
    if (phases[phase].setPins & RED_PIN) {
      HAL_GPIO_WritePin(SECOND_PORT, GREEN2_PIN, GPIO_PIN_SET);
    } else if (phases[phase].setPins & GREEN_PIN) {
      HAL_GPIO_WritePin(SECOND_PORT, RED2_PIN, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(SECOND_PORT, YELLOW2_PIN, GPIO_PIN_SET);
    }
    vTaskDelayUntil(&lastWake, phases[phase].duration);
    phase = phases[phase].nextPhase;
  }
}

void AmbulanceTask(void *argument)
{
  for (;;) {
    xEventGroupWaitBits(semaEventGroup, BIT_AMB_REQ, pdFALSE, pdTRUE, portMAX_DELAY);
    HAL_GPIO_WritePin(BLUE_PORT, BLUE_PIN, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(2000));
    HAL_GPIO_WritePin(BLUE_PORT, BLUE_PIN, GPIO_PIN_RESET);
    xEventGroupClearBits(semaEventGroup, BIT_AMB_REQ);
    xEventGroupSetBits(semaEventGroup, BIT_AMB_DONE);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == B1_Pin) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR(semaEventGroup, BIT_AMB_REQ, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void StartDefaultTask(void *argument)
{
  for (;;) {
    osDelay(1);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
