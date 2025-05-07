/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Traffic lights with pedestrian crossing
  *                   (includes final pedestrian blinking phase)
  ******************************************************************************
  * @details
  * This application controls a system of three traffic lights using FreeRTOS.
  * Traffic lights 1 and 3 operate alternately. After both complete their cycles,
  * traffic light 2 activates, allowing vehicle passage and pedestrian crossing.
  * The pedestrian LED remains solid for a set duration, then blinks before turning off.
  * Tasks are synchronized using FreeRTOS semaphores.
  *
  * Traffic lights:
  * - Sem1Task (PA0, PA1, PA4): Red, Yellow, Green
  * - Sem3Task (PC10, PC11, PC12): Red, Yellow, Green
  * - Sem2Task (PC6, PC8, PC9 + PB5): Red, Yellow, Green + Pedestrian LED
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "semphr.h"    // per FreeRTOS Semaphore API

/* Private defines -----------------------------------------------------------*/
// Semaforo principale (porta A, CN8)
#define RED_PIN        GPIO_PIN_0
#define YELLOW_PIN     GPIO_PIN_1
#define GREEN_PIN      GPIO_PIN_4

// Semaforo secondario (porta C, CN10)
#define RED2_PIN       GPIO_PIN_9 // CN10 Pin 1
#define YELLOW2_PIN    GPIO_PIN_8 // CN10 Pin 2
#define GREEN2_PIN     GPIO_PIN_6 // CN10 Pin 4
#define SECOND_PORT    GPIOC

// Terzo semaforo (porta C, CN7)
#define RED3_PIN       GPIO_PIN_10 // CN7 Pin 1
#define YELLOW3_PIN    GPIO_PIN_11 // CN7 Pin 2
#define GREEN3_PIN     GPIO_PIN_12 // CN7 Pin 3
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

/* Private typedef -----------------------------------------------------------*/
/* (nessuno) */

/* Private variables ---------------------------------------------------------*/
COM_InitTypeDef BspCOMInit;

// Semafori software per sincronizzazione
SemaphoreHandle_t sem1SemHandle;
SemaphoreHandle_t sem2SemHandle;
SemaphoreHandle_t sem3SemHandle;

/* Task handles e attributi */
osThreadId_t sem1TaskHandle;
osThreadId_t sem2TaskHandle;
osThreadId_t sem3TaskHandle;

const osThreadAttr_t sem1Task_attributes = {
  .name       = "Sem1Task",
  .priority   = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
const osThreadAttr_t sem2Task_attributes = {
  .name       = "Sem2Task",
  .priority   = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
const osThreadAttr_t sem3Task_attributes = {
  .name       = "Sem3Task",
  .priority   = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void Sem1Task(void *argument);
void Sem2Task(void *argument);
void Sem3Task(void *argument);
void Error_Handler(void);
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line);
#endif

/**
  * @brief  The application entry point.
  */
int main(void)
{
  /* MCU e HAL init */
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  /* Init RTOS kernel */
  osKernelInitialize();

  /* Init onboard LED e user button */
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

  /* Creazione semafori software */
  sem1SemHandle = xSemaphoreCreateBinary();
  sem3SemHandle = xSemaphoreCreateBinary();
  sem2SemHandle = xSemaphoreCreateCounting(2, 0);  // max count=2, iniziale=0
  if (!sem1SemHandle || !sem2SemHandle || !sem3SemHandle) {
    Error_Handler();
  }
  /* All'avvio, rilascio subito Sem1 e Sem3 */
  xSemaphoreGive(sem1SemHandle);
  xSemaphoreGive(sem3SemHandle);

  /* Creazione dei task */
  sem1TaskHandle = osThreadNew(Sem1Task, NULL, &sem1Task_attributes);
  sem2TaskHandle = osThreadNew(Sem2Task, NULL, &sem2Task_attributes);
  sem3TaskHandle = osThreadNew(Sem3Task, NULL, &sem3Task_attributes);

  /* Avvio scheduler */
  osKernelStart();

  /* Non dovrebbe mai arrivare qui */
  while (1) {}
}

/**
  * @brief  Task che gestisce il semaforo 1 (GPIOA)
  */
void Sem1Task(void *argument)
{
  for (;;)
  {
    /* attendo il mio semaforo */
    xSemaphoreTake(sem1SemHandle, portMAX_DELAY);

    /* verde */
    HAL_GPIO_WritePin(GPIOA, GREEN_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, RED_PIN|YELLOW_PIN, GPIO_PIN_RESET);
    osDelay(T_GREEN_MS);

    /* giallo */
    HAL_GPIO_WritePin(GPIOA, YELLOW_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, RED_PIN|GREEN_PIN, GPIO_PIN_RESET);
    osDelay(T_YELLOW_MS);

    /* rosso */
    HAL_GPIO_WritePin(GPIOA, RED_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, YELLOW_PIN|GREEN_PIN, GPIO_PIN_RESET);

    /* segnalo Sem2 (1 token) */
    xSemaphoreGive(sem2SemHandle);
  }
}

/**
  * @brief  Task che gestisce il semaforo 3 (GPIOC)
  */
void Sem3Task(void *argument)
{
  for (;;)
  {
    xSemaphoreTake(sem3SemHandle, portMAX_DELAY);

    /* verde */
    HAL_GPIO_WritePin(THIRD_PORT, GREEN3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(THIRD_PORT, RED3_PIN|YELLOW3_PIN, GPIO_PIN_RESET);
    osDelay(T_GREEN_MS);

    /* giallo */
    HAL_GPIO_WritePin(THIRD_PORT, YELLOW3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(THIRD_PORT, RED3_PIN|GREEN3_PIN, GPIO_PIN_RESET);
    osDelay(T_YELLOW_MS);

    /* rosso */
    HAL_GPIO_WritePin(THIRD_PORT, RED3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(THIRD_PORT, YELLOW3_PIN|GREEN3_PIN, GPIO_PIN_RESET);

    /* segnalo Sem2 (1 token) */
    xSemaphoreGive(sem2SemHandle);
  }
}

/**
  * @brief  Task che gestisce il semaforo 2 (GPIOC) + pedonale
  */
void Sem2Task(void *argument)
{
  for (;;)
  {
    /* attendo i due token da Sem1 e Sem3 */
    xSemaphoreTake(sem2SemHandle, portMAX_DELAY);
    xSemaphoreTake(sem2SemHandle, portMAX_DELAY);

    /* verde */
    HAL_GPIO_WritePin(SECOND_PORT, GREEN2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SECOND_PORT, RED2_PIN|YELLOW2_PIN, GPIO_PIN_RESET);
    osDelay(T_GREEN_MS);

    /* giallo */
    HAL_GPIO_WritePin(SECOND_PORT, YELLOW2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SECOND_PORT, RED2_PIN|GREEN2_PIN, GPIO_PIN_RESET);
    osDelay(T_YELLOW_MS);

    /* rosso veicoli */
    HAL_GPIO_WritePin(SECOND_PORT, RED2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SECOND_PORT, YELLOW2_PIN|GREEN2_PIN, GPIO_PIN_RESET);

    /* pedonale acceso fisso */
    HAL_GPIO_WritePin(PED_PORT, PED_PIN, GPIO_PIN_SET);
    osDelay(T_PED_TOTAL_MS - T_PED_BLINK_MS);

    /* lampeggio pedonale */
    {
      const uint32_t toggles = T_PED_BLINK_MS / T_PED_BLINK_INTERVAL_MS;
      for (uint32_t i = 0; i < toggles; ++i)
      {
        HAL_GPIO_TogglePin(PED_PORT, PED_PIN);
        osDelay(T_PED_BLINK_INTERVAL_MS);
      }
    }
    HAL_GPIO_WritePin(PED_PORT, PED_PIN, GPIO_PIN_RESET);

    /* rilascio Sem1 e Sem3 per ricominciare il ciclo */
    xSemaphoreGive(sem1SemHandle);
    xSemaphoreGive(sem3SemHandle);
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
