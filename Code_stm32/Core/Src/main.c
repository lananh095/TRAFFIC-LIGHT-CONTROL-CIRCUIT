/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stdbool.h"
//7447 ÐÔNG -TÂY
//HÀNG CHUC
#define A11(state) HAL_GPIO_WritePin(A11_GPIO_Port,A11_Pin,(GPIO_PinState)state)
#define B11(state) HAL_GPIO_WritePin(B11_GPIO_Port,B11_Pin,(GPIO_PinState)state)
#define C11(state) HAL_GPIO_WritePin(C11_GPIO_Port,C11_Pin,(GPIO_PinState)state)
#define D11(state) HAL_GPIO_WritePin(D11_GPIO_Port,D11_Pin,(GPIO_PinState)state)
//HANG DON VI
#define A12(state) HAL_GPIO_WritePin(A12_GPIO_Port,A12_Pin,(GPIO_PinState)state)
#define B12(state) HAL_GPIO_WritePin(B12_GPIO_Port,B12_Pin,(GPIO_PinState)state)
#define C12(state) HAL_GPIO_WritePin(C12_GPIO_Port,C12_Pin,(GPIO_PinState)state)
#define D12(state) HAL_GPIO_WritePin(D12_GPIO_Port,D12_Pin,(GPIO_PinState)state)
//7447 NAM-BAC
//HANG CHUC
#define A21(state) HAL_GPIO_WritePin(A21_GPIO_Port,A21_Pin,(GPIO_PinState)state)
#define B21(state) HAL_GPIO_WritePin(B21_GPIO_Port,B21_Pin,(GPIO_PinState)state)
#define C21(state) HAL_GPIO_WritePin(C21_GPIO_Port,C21_Pin,(GPIO_PinState)state)
#define D21(state) HAL_GPIO_WritePin(D21_GPIO_Port,D21_Pin,(GPIO_PinState)state)
//HANG DON VI
#define A22(state) HAL_GPIO_WritePin(A22_GPIO_Port,A22_Pin,(GPIO_PinState)state)
#define B22(state) HAL_GPIO_WritePin(B22_GPIO_Port,B22_Pin,(GPIO_PinState)state)
#define C22(state) HAL_GPIO_WritePin(C22_GPIO_Port,C22_Pin,(GPIO_PinState)state)
#define D22(state) HAL_GPIO_WritePin(D22_GPIO_Port,D22_Pin,(GPIO_PinState)state)
//BUTTON
#define BTV HAL_GPIO_ReadPin(BTV_GPIO_Port,BTV_Pin)
#define BTDX HAL_GPIO_ReadPin(BTDX_GPIO_Port,BTDX_Pin)
//LED D-X-V
#define R1(state) HAL_GPIO_WritePin(R1_GPIO_Port,R1_Pin,(GPIO_PinState)state)
#define Y1(state) HAL_GPIO_WritePin(Y1_GPIO_Port,Y1_Pin,(GPIO_PinState)state)
#define G1(state) HAL_GPIO_WritePin(G1_GPIO_Port,G1_Pin,(GPIO_PinState)state)

#define R2(state) HAL_GPIO_WritePin(R2_GPIO_Port,R2_Pin,(GPIO_PinState)state)
#define Y2(state) HAL_GPIO_WritePin(Y2_GPIO_Port,Y2_Pin,(GPIO_PinState)state)
#define G2(state) HAL_GPIO_WritePin(G2_GPIO_Port,G2_Pin,(GPIO_PinState)state)

#define R3(state) HAL_GPIO_WritePin(R3_GPIO_Port,R3_Pin,(GPIO_PinState)state)
#define Y3(state) HAL_GPIO_WritePin(Y3_GPIO_Port,Y3_Pin,(GPIO_PinState)state)
#define G3(state) HAL_GPIO_WritePin(G3_GPIO_Port,G3_Pin,(GPIO_PinState)state)

#define R4(state) HAL_GPIO_WritePin(R4_GPIO_Port,R4_Pin,(GPIO_PinState)state)
#define Y4(state) HAL_GPIO_WritePin(Y4_GPIO_Port,Y4_Pin,(GPIO_PinState)state)
#define G4(state) HAL_GPIO_WritePin(G4_GPIO_Port,G4_Pin,(GPIO_PinState)state)
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
int seven_seg_digits[10][4] = {  
                                { 0,0,0,0 },  // = 0//D   C  B  A//
                                { 0,0,0,1 },  // = 1//MSB       LSB//
                                { 0,0,1,0 },  // = 2
                                { 0,0,1,1 },  // = 3
                                { 0,1,0,0 },  // = 4
                                { 0,1,0,1 },  // = 5
                                { 0,1,1,0 },  // = 6
                                { 0,1,1,1 },  // = 7
                                { 1,0,0,0 },  // = 8
                                { 1,0,0,1 }   // = 9
                              };
uint8_t btn[2],stepbtnv=0,stepbtndx=0;
uint8_t step_do1=true,step_vang1=false,step_xanh1=false;
uint8_t step_do2=false,step_vang2=false,step_xanh2=true;
uint8_t cnt1=30,cnt2=25,cnt3=5;
uint8_t cnt4=30,cnt5=25,cnt6=5;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void disp1(uint8_t digit);
void disp2(uint8_t digit);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void disp1(uint8_t digit)
{
    //hangchuc
    A11(seven_seg_digits[digit%10][3]);
    B11(seven_seg_digits[digit%10][2]);
    C11(seven_seg_digits[digit%10][1]);
    D11(seven_seg_digits[digit%10][0]);
    
    A21(seven_seg_digits[digit/10][3]);
    B21(seven_seg_digits[digit/10][2]);
    C21(seven_seg_digits[digit/10][1]);
    D21(seven_seg_digits[digit/10][0]);

}
void disp2(uint8_t digit)
{
      //hangdonvi
    A12(seven_seg_digits[digit%10][3]);
    B12(seven_seg_digits[digit%10][2]);
    C12(seven_seg_digits[digit%10][1]);
    D12(seven_seg_digits[digit%10][0]);

    A22(seven_seg_digits[digit/10][3]);
    B22(seven_seg_digits[digit/10][2]);
    C22(seven_seg_digits[digit/10][1]);
    D22(seven_seg_digits[digit/10][0]);
}
void den1(void)
{
    if(step_do1==true)
    {
      R1(1);R3(1);
      Y1(0);Y3(0);
      G1(0);G3(0);
      disp1(cnt1);
      if(cnt1==0)
      {
        disp1(0);
        step_xanh1=true;
        cnt1=30;
        step_do1=false;
        
      }
      else 
      {
        cnt1--;
      }
    }
    if(step_xanh1==true)
    {
      R1(0);R3(0);
      Y1(0);Y3(0);
      G1(1);G3(1);
      disp1(cnt2);
      if(cnt2==0)
      {
        disp1(0);
        step_vang1=true;
        cnt2=25;
        step_xanh1=false;
      }
      else 
      {
        cnt2--;
      }
    }
    if(step_vang1==true)
    {
      R1(0);R3(0);
      Y1(1);Y3(1);
      G1(0);G3(0);
      disp1(cnt3);
      if(cnt3==0)
      {
        disp1(0);
        step_do1=true;
        cnt3=5;
        step_vang1=false;
      }
      else 
      {
        cnt3--;
      }
    }
}
void den2(void)
{
    if(step_do2==true)
    {
      R2(1);R4(1);
      Y2(0);Y4(0);
      G2(0);G4(0);
      disp2(cnt4);
      if(cnt4==0)
      {
        disp2(0);
        step_xanh2=true;
        cnt4=30;
        step_do2=false;
        
      }
      else 
      {
        cnt4--;
      }
    }
    if(step_xanh2==true)
    {
      R2(0);R4(0);
      Y2(0);Y4(0);
      G2(1);G4(1);
      disp2(cnt5);
      if(cnt5==0)
      {
        disp2(0);
        step_vang2=true;
        cnt5=25;
        step_xanh2=false;
      }
      else 
      {
        cnt5--;
      }
    }
    if(step_vang2==true)
    {
      R2(0);R4(0);
      Y2(1);Y4(1);
      G2(0);G4(0);
      disp2(cnt6);
      if(cnt6==0)
      {
        disp2(0);
        step_do2=true;
        cnt6=5;
        step_vang2=false;
      }
      else 
      {
        cnt6--;
      }
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance==htim3.Instance)
  {
    if(stepbtnv==0&&stepbtndx==0)
    {
      den1();
      den2();
    }
    else if(stepbtnv==1)
    {
      disp1(0);
      disp2(0);
      R1(0);R3(0);
      Y1(1);Y3(1);
      G1(0);G3(0);
      
      R2(0);R4(0);
      Y2(1);Y4(1);
      G2(0);G4(0);   
    }
    else if(stepbtndx==1)
    {
      disp1(0);
      disp2(0);
      R1(1);R3(1);
      Y1(0);Y3(0);
      G1(0);G3(0);
      
      R2(0);R4(0);
      Y2(0);Y4(0);
      G2(1);G4(1);   
    }
    else if(stepbtndx==2)
    {
      disp1(0);
      disp2(0);
      R1(0);R3(0);
      Y1(0);Y3(0);
      G1(1);G3(1);
      
      R2(1);R4(1);
      Y2(0);Y4(0);
      G2(0);G4(0);   
    }
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  disp1(0);
  disp2(0);
  HAL_Delay(1000);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    btn[0]=BTV;
    btn[1]=BTDX;
    if(BTV==0)
    {
      HAL_Delay(80);
      if(BTV==0)
      {
        stepbtnv++;
        stepbtndx=0;
      }
    }
    if(BTDX==0)
    {
      HAL_Delay(80);
      if(BTDX==0)
      {
        stepbtndx++;
        stepbtnv=0;
      }
    }
    if(stepbtnv>1)stepbtnv=0;
    if(stepbtndx>2)stepbtndx=0;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 23000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1999;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, R1_Pin|Y1_Pin|G1_Pin|R2_Pin
                          |Y2_Pin|G2_Pin|R3_Pin|Y3_Pin
                          |C11_Pin|A12_Pin|D11_Pin|B11_Pin
                          |A11_Pin|D22_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, G3_Pin|R4_Pin|Y4_Pin|G4_Pin
                          |D12_Pin|C12_Pin|B12_Pin|C22_Pin
                          |B22_Pin|A22_Pin|D21_Pin|C21_Pin
                          |B21_Pin|A21_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTV_Pin */
  GPIO_InitStruct.Pin = BTV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTV_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R1_Pin Y1_Pin G1_Pin R2_Pin
                           Y2_Pin G2_Pin R3_Pin Y3_Pin
                           C11_Pin A12_Pin D11_Pin B11_Pin
                           A11_Pin D22_Pin */
  GPIO_InitStruct.Pin = R1_Pin|Y1_Pin|G1_Pin|R2_Pin
                          |Y2_Pin|G2_Pin|R3_Pin|Y3_Pin
                          |C11_Pin|A12_Pin|D11_Pin|B11_Pin
                          |A11_Pin|D22_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : G3_Pin R4_Pin Y4_Pin G4_Pin
                           D12_Pin C12_Pin B12_Pin C22_Pin
                           B22_Pin A22_Pin D21_Pin C21_Pin
                           B21_Pin A21_Pin */
  GPIO_InitStruct.Pin = G3_Pin|R4_Pin|Y4_Pin|G4_Pin
                          |D12_Pin|C12_Pin|B12_Pin|C22_Pin
                          |B22_Pin|A22_Pin|D21_Pin|C21_Pin
                          |B21_Pin|A21_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BTDX_Pin */
  GPIO_InitStruct.Pin = BTDX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTDX_GPIO_Port, &GPIO_InitStruct);

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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
