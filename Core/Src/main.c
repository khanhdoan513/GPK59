/* USER CODE BEGIN Header */
/*
   Gyro Compass
   Programmer: K-D
   Version: GyroCp LBCQ v2.1
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>

uint32_t TIMR3;
uint32_t timr3_curr;
uint32_t timr3_pass;
uint32_t cnt;
uint8_t RxData;
uint8_t MLRxData[14];
uint8_t TxData[5] = {0x77, 0x04, 0x00, 0x04, 0x08};
// uint8_t TxData[6] = {0x77, 0x05, 0x00, 0x0B, 0x04, 0x14};
typedef enum
{
  CK_77,
  CK_0D,
  CK_00,
  CK_84,
  READ_X1,
  READ_X2,
  READ_X3,
  READ_Y1,
  READ_Y2,
  READ_Y3,
  READ_Z1,
  READ_Z2,
  READ_Z3,
  CHECKSUM
} STATE;
typedef enum
{
  READY,
  CHECK,
  DONE
} STATE_BTN;
STATE state = CK_77;
STATE_BTN state_btn = READY;

int32_t Ang_curr;
int32_t Ang_old;
int32_t Denta_Ang;
int32_t Poss_target;
int32_t Poss_real;

uint32_t lastDebounceTime;

uint8_t byte1, byte2, byte3;

uint8_t csm;

bool btn = false;
bool btn_old = true;
bool btn_flag = false;
bool reading;

bool chanA_bit_old;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void KD_delay(uint32_t tm);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if (htim->Instance == TIM4)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, TxData, 5, 10);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

    Denta_Ang = Ang_curr - Ang_old;
    if (btn == true)
      Denta_Ang = 0;
    Ang_old = Ang_curr;
    if (Denta_Ang >= -36000 && Denta_Ang < -18000)
    {
      Poss_target += (Denta_Ang + 36000);
    }
    else if (Denta_Ang >= -18000 && Denta_Ang <= 18000)
    {
      Poss_target += Denta_Ang;
    }
    else if (Denta_Ang > 18000 && Denta_Ang <= 36000)
    {
      Poss_target -= (36000 - Denta_Ang);
    }
  }

  if (htim->Instance == TIM3)
  {
    TIMR3++;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_IT(&huart1, &RxData, 1);
  //	HAL_UART_Receive_IT(&huart1, MLRxData, 14);
  //	HAL_UART_Receive_DMA(&huart1, &RxData, 1);
  switch (state)
  {
  case CK_77:
    if (RxData == 0x77)
    {
      state = CK_0D;
    }
    break;
  case CK_0D:
    if (RxData == 0x0D)
    {
      csm += RxData;
      state = CK_00;
    }
    else
    {
      state = CK_77;
    }
    break;
  case CK_00:
    if (RxData == 0x00)
    {
      csm += RxData;
      state = CK_84;
    }
    else
    {
      state = CK_77;
    }
    break;
  case CK_84:
    if (RxData == 0x84)
    {
      csm += RxData;
      state = READ_X1;
    }
    else
    {
      state = CK_77;
    }
    break;
  case READ_X1:
    state = READ_X2;
    csm += RxData;
    break;
  case READ_X2:
    state = READ_X3;
    csm += RxData;
    break;
  case READ_X3:
    state = READ_Y1;
    csm += RxData;
    break;
  case READ_Y1:
    state = READ_Y2;
    csm += RxData;
    break;
  case READ_Y2:
    state = READ_Y3;
    csm += RxData;
    break;
  case READ_Y3:
    state = READ_Z1;
    csm += RxData;
    break;
  case READ_Z1:
    byte1 = RxData;
    csm += RxData;
    //        Ang_curr = (RxData & 0x0F) * 10000;
    state = READ_Z2;
    break;
  case READ_Z2:
    byte2 = RxData;
    csm += RxData;
    //        Ang_curr += ((RxData & 0xF0) >> 4) * 1000;
    //        Ang_curr += (RxData & 0x0F) * 100;
    state = READ_Z3;
    break;
  case READ_Z3:
    byte3 = RxData;
    csm += RxData;
    //        Ang_curr += ((RxData & 0xF0) >> 4) * 10;
    //        Ang_curr += (RxData & 0x0F);
    state = CHECKSUM;
    break;
  case CHECKSUM:
    if (RxData == csm)
    {
      cnt++;
      Ang_curr = (byte1 & 0x0F) * 10000;
      Ang_curr += ((byte2 & 0xF0) >> 4) * 1000;
      Ang_curr += (byte2 & 0x0F) * 100;
      Ang_curr += ((byte3 & 0xF0) >> 4) * 10;
      Ang_curr += (byte3 & 0x0F);
      csm = 0;
    }
    state = CK_77;
    break;
  default:
    break;
  }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_UART_Receive_IT(&huart1, &RxData, 1);

  //	HAL_UART_Receive_IT(&huart1, MLRxData, 14);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //=================================================================================
    //		TIMR3 = 0;
    //		while(TIMR3 < 40){
    //			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,1);
    //		}
    //		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
    //=================================================================================
    //		if(cnt < 1600){
    //			timr3_curr = TIMR3;
    //			if(timr3_curr - timr3_pass > 100){
    //				timr3_pass = timr3_curr;
    //				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    //				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1,1);
    //				KD_delay(1);
    //				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1,0);
    //				cnt++;
    //			}
    //		}
    //		if(cnt == 1600){
    //			HAL_Delay(1000);
    //			cnt = 0;
    //		}
    //===================================================================================
    timr3_curr = TIMR3;
    if (timr3_curr - timr3_pass > 100)
    {
      timr3_pass = timr3_curr;
      if (Poss_real > (int32_t)(Poss_target * 2 / 45))
      {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
        KD_delay(1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
        Poss_real -= 1;
      }
      else if (Poss_real < (int32_t)(Poss_target * 2 / 45))
      {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
        KD_delay(1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
        Poss_real += 1;
      }
    }
    //===================================================================================
    //		reading = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9);
    //		if (reading == false && btn_old == true) {
    //      if (TIMR3 - lastDebounceTime > 50000) {
    //          btn = !btn;
    //				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,!btn);

    //          lastDebounceTime = TIMR3;
    //      }
    //    }
    //		btn_old = reading;
    reading = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
    switch (state_btn)
    {
    case READY:
      if (reading == false)
      {
        state_btn = CHECK;
        lastDebounceTime = TIMR3;
      }
      break;
    case CHECK:
      if (TIMR3 - lastDebounceTime > 100000)
      {
        if (reading == false)
        {
          btn = !btn;
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, !btn);
          state_btn = READY;
        }
        else
        {
          state_btn = READY;
        }
      }
      break;
    case DONE:
      btn = !btn;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, !btn);
      state_btn = READY;
      break;
    default:
      break;
    }

    if (btn == true)
    {
      bool chanA_bit = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
      if (chanA_bit != chanA_bit_old)
      {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) != chanA_bit)
        {
          Poss_target += 25;
        }
        else
        {
          Poss_target -= 25;
          //          Serial.println("DECRE");
        }
      }
      chanA_bit_old = chanA_bit;
    }
    //===================================================================================
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 720 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000 - 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//============================================================================================================
void KD_delay(uint32_t tm)
{
  uint32_t tm_curr = TIMR3;
  while (TIMR3 - tm_curr < tm)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
  }
}
//============================================================================================================
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
