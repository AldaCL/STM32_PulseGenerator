/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "st7735.h"
#include "fonts.h"
#include <math.h>


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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int nLoop = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM10_Init();
  MX_USART2_UART_Init();
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    //HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1);
  /* USER CODE BEGIN 2 */
  ST7735_Init();
  ST7735_FillScreen(ST7735_BLACK);
  int duty = 40;
  int freq = 200;
  int printDD = 40;
  float mess = 40;
  float cycle = 0.0;
  float pwidth = 0.0;

  htim10.Instance->CCR1 = 40;
  htim10.Instance->ARR = freq;

  float realFreq = 84000/((htim10.Instance->ARR)+1);

  //uint8_t
  ST7735_FillRectangle(5, 8, 160, 30, ST7735_BLUE);
  ST7735_WriteString(7, 10, "Frequency=        kHz", Font_7x10, ST7735_WHITE, ST7735_BLUE);
  ST7735_WriteString(7, 25, "Pulse W. =         ns", Font_7x10, ST7735_WHITE, ST7735_BLUE);

  ST7735_WriteString(1, 45, "3.3v", Font_7x10, ST7735_YELLOW, ST7735_BLACK);
  ST7735_FillRectangle(2, 60, 1, 40, ST7735_RED);
  ST7735_FillRectangle(0, 100,5, 1, ST7735_RED);
  ST7735_FillRectangle(0, 60 ,5 ,1, ST7735_RED);

  ST7735_FillRectangle(10, 60, 1, 40,ST7735_WHITE );
  ST7735_FillRectangle(10, 60, 45, 1, ST7735_WHITE);
  //ST7735_FillRectangle(55, 70, 1, 40,ST7735_WHITE);
  ST7735_FillRectangle(55, 100, 50, 1, ST7735_WHITE);
  ST7735_FillRectangle(105, 60, 1, 40,ST7735_WHITE );
  ST7735_FillRectangle(105, 60, 20, 1, ST7735_WHITE);

  //ST7735_FillRectangle(10, 115, 45, 1, ST7735_CYAN);
  //ST7735_FillRectangle(10, 113, 1, 4, ST7735_CYAN);
  //ST7735_FillRectangle(55, 113, 1, 4, ST7735_CYAN);
  //ST7735_WriteString(26, 118, "1us", Font_7x10, ST7735_WHITE, ST7735_BLACK);

  ST7735_WriteString(35, 118, "Duty=      %", Font_7x10, ST7735_WHITE, ST7735_BLACK);



//  for(int y = ST7735_HEIGHT/2; y< ){
//
//  }
  bool press = true;
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if(!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))){
		  while(!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)));

		  press = !press;

		  if(press){
			  TIM1->CNT = printDD*4;
		  		  }else{
		  			TIM1->CNT =freq*4;
		  		  }
	  }

	  int EncVal = TIM1->CNT;


//	  	  if(EncVal==0){
//	  		  EncVal = 40;
//	  	  }

	  //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // PORTA, PA5


	  if(press == true){
		  mess = EncVal/4;
		  //duty = refactorScale(mess, 1, 100, 1, 180);
		  //int mess2 = EncVal/4;
		  //duty = (100*(printDD/(freq)));
		  //duty = duty%200;

		  duty = ((mess/freq)*100);
		  		  if (duty >100){
		  			  duty = 99;
		  			  //htim10.Instance->CCR1 = 1;
		  			TIM1->CNT = 1;
		  		  }else{

		  			  htim10.Instance->CCR1 = mess;
		  		  }

		  //

		  //UPPER LINES
		  ST7735_FillRectangle(10, 60, duty*0.92, 1, ST7735_WHITE);
		  ST7735_FillRectangle(11+(duty*0.92), 60, 95-(duty*0.92) , 1, ST7735_BLACK);
		  //DOWN LINES

		  ST7735_FillRectangle(11+(duty*0.92), 100, 95-(duty*0.92), 1, ST7735_WHITE);
		  ST7735_FillRectangle(10, 100, duty*0.92, 1, ST7735_BLACK);
		  //ST7735_FillRectangle(10, 110, duty*0.9, 1, ST7735_BLACK);
		 // ST7735_FillRectangle(11+(duty*0.9), 70, 95-(duty*0.9) , 1, ST7735_WHITE);

		  //VERTICAL LINE
		  ST7735_FillRectangle(10+(duty*0.92), 60, 1, 40,ST7735_WHITE);
		  ST7735_FillRectangle(9+(duty*0.92), 61, 1, 39,ST7735_BLACK);
		  //ST7735_FillRectangle(11+(duty*0.92), 71, 1, 39,ST7735_BLACK);
		  //ST7735_FillRectangle(9+(duty*0.94), 71, 1, 39,ST7735_BLACK);
		  ST7735_FillRectangle(11+(duty*0.94), 61, 1, 39,ST7735_BLACK);

		  //ST7735_FillRectangle(9+(duty*0.92), 71, 1, 39,ST7735_BLACK);

		  //ST7735_FillRectangle(11+(duty*0.95), 71, 1, 39,ST7735_BLACK);
		  ST7735_FillRectangle(101, 61, 2, 39,ST7735_BLACK);
		  ST7735_FillRectangle(10, 60, 1, 40,ST7735_WHITE );

		//  ST7735_FillRectangle(55, 110, 50, 1, ST7735_WHITE);


		  //Time
		  //ST7735_FillRectangle(10+(duty*0.92), 115, 45, 1, ST7735_CYAN);
		  ST7735_FillRectangle(10, 105, duty*0.92, 1, ST7735_CYAN);
		  //ST7735_FillRectangle(55, 113, 1, 4, ST7735_CYAN);
		  ST7735_FillRectangle(10+(duty*0.92), 103, 1,4,ST7735_CYAN);
		  ST7735_FillRectangle(9+(duty*0.92), 103, 1, 4,ST7735_BLACK);
		  ST7735_FillRectangle(11+(duty*0.92), 103, 1, 4,ST7735_BLACK);

		  ST7735_FillRectangle(102, 103, 2, 4,ST7735_BLACK);

		  ST7735_FillRectangle(10, 103, 1, 4, ST7735_CYAN);

		  ST7735_FillRectangle(11+(duty*0.92), 105, 95-(duty*0.92), 1, ST7735_BLACK);


		  //Final line
		  ST7735_FillRectangle(105, 60, 1, 40,ST7735_WHITE );




	  }else {

		  freq = EncVal/4;
		  htim10.Instance->ARR = freq ;
	  }


//	  HAL_Delay(500);
	  //nLoop++;
	  //itoa(EncVal,snum,10);
	  //int i = 247593;
	  realFreq = 84000.0/((htim10.Instance->ARR)+1);

	  char strEnc[6];
	  //print("{:0>8.4f}".format(2.02))
	  sprintf(strEnc, "%07.1f", realFreq);
	  //sprintf(strEnc,realFreq);
	  //"{:0>6.2f}".format(strEnc);
	  ST7735_WriteString(80, 10, strEnc, Font_7x10,
	  	                         ST7735_CYAN, ST7735_BLUE);
	  cycle = ((mess/freq)*100);

	  cycle = fmodf(cycle,100);

	  char strCycle[6];
	  sprintf(strCycle, "%0.2f", cycle);
	  ST7735_WriteString(75, 118, strCycle, Font_7x10, ST7735_CYAN, ST7735_BLACK);


	  pwidth =((mess)/84000)*1000000;
	 char stringCCR[7];

	 sprintf(stringCCR, "%07.2f", pwidth);
	 ST7735_WriteString(80, 25, stringCCR, Font_7x10,
	 	  	    	  	                         ST7735_CYAN, ST7735_BLUE);

//	 char stringARR[5];
//	 sprintf(stringARR,"%04d", freq );
//	 ST7735_WriteString(95, 40, stringARR, Font_7x10,
//	 	 	  	    	  	                         ST7735_RED, ST7735_BLUE);


	  	     // HAL_Delay(20);

//	  	      for(int x = 0; x < ST7735_WIDTH; x++) {
//	  	          ST7735_DrawPixel(x, 0, ST7735_RED);
//	  	          ST7735_DrawPixel(x, ST7735_HEIGHT-1, ST7735_RED);
//	  	      }
//
//	  	      for(int y = 0; y < ST7735_HEIGHT; y++) {
//	  	          ST7735_DrawPixel(0, y, ST7735_RED);
//	  	          ST7735_DrawPixel(ST7735_WIDTH-1, y, ST7735_RED);
//	  	      }
//
//	  	      HAL_Delay(3000);
//
//	  	      // Check fonts
//	  	      ST7735_FillScreen(ST7735_BLACK);
//	  	      ST7735_WriteString(0, 0, "Font_7x10", Font_7x10,
//	  	                         ST7735_RED, ST7735_BLACK);
//	  	      ST7735_WriteString(0, 3*10, "Font_11x18", Font_11x18,
//	  	                         ST7735_GREEN, ST7735_BLACK);
//	  	      ST7735_WriteString(0, 3*10+3*18, "Font_16x26", Font_16x26,
//	  	                         ST7735_BLUE, ST7735_BLACK);
//	  	      HAL_Delay(2000);
//
//	  	      // Check color inversion
//	  	      ST7735_InvertColors(true);
//	  	      HAL_Delay(2000);
//	  	      ST7735_InvertColors(false);
//	  	      HAL_Delay(2000);
//
//	  	      // Display test image
//	  	      //ST7735_DrawImage(0, 0, ST7735_WIDTH, ST7735_HEIGHT,test_img_128x128);
//	  	      HAL_Delay(15000);
    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 1-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 5000;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DC_Pin|CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RES_Pin */
  GPIO_InitStruct.Pin = RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_Pin CS_Pin */
  GPIO_InitStruct.Pin = DC_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
