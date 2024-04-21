/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	IDLE,
    FORWARD,
    BACK,
}State;
typedef enum {
	EXTIN,   /* have button press*/
	NOEXTIN, /* don't have button press */
}StateExti;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
State CurrentState = IDLE ;
StateExti ButtonState = NOEXTIN;
volatile uint8_t flag_debounce = 0;  		//cmt: flag start debounce button
volatile uint8_t flag_swap = 0;				//cmt: motor status change flag
static volatile uint32_t ms_counter = 0;    //cmt: time debounce of button
static uint8_t DEBOUNCE_DELAY = 20;
volatile uint16_t temp_arr2 = 1800;
uint16_t ARR_PWM = 3600; /* frequency 20k for dc motor*/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static void CheckDirMotor(void);
void ConfigTimerIOCW (void);
void ConfigTimerIOCCW (void);
extern void msSysTickIntHandler(void);
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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  if (SysTick_Config(SystemCoreClock / 1000)) {
      Error_Handler();
  }
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();

  // MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  CheckDirMotor();


    /* USER CODE END WHILE */

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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(72000000);
  LL_SetSystemCoreClock(72000000);
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	 /* GPIO Ports Clock Enable */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
		 /**/
	LL_GPIO_ResetOutputPin(GPIOA, M1_HIN_L_Pin|M1_HIN_R_Pin);
	 /* GPIO Ports Clock Enable */
	 LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);


  /* USER CODE BEGIN TIM2_Init 0 */
	RCC->APB1ENR |= (1<<0); /* enable TIM2 clock */
  /* USER CODE END TIM2_Init 0 */

  /* USER CODE BEGIN TIM2_Init 1 */
	TIM2->CCR2 = 0;

   TIM2->CCMR1 = 0X7000;	/* mode PWM2 CH2 */
  /* USER CODE END TIM2_Init 1 */

  /* USER CODE BEGIN TIM2_Init 2 */
   TIM2->CCR3 = 0;
   TIM2->CCMR2 = 0X0070;	/* mode PWM2 CH2 */
   TIM2->CCER = (0x1 << 4)|(0x1 << 8);
   TIM2->PSC = 200;
   TIM2->ARR = ARR_PWM - 1;

  /* USER CODE END TIM2_Init 2 */

	//ConfigTimer2Pin
	 GPIO_InitStruct.Pin = M1_HIN_L_Pin|M1_HIN_R_Pin ;
	 GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	 GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	 GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	 LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH2);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH3);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**TIM3 GPIO Configuration
  PA7   ------> TIM3_CH2
  PB0   ------> TIM3_CH3
  */
  GPIO_InitStruct.Pin = PWMT_2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(PWMT_2_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = PWMN_2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(PWMN_2_GPIO_Port, &GPIO_InitStruct);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, M1_LIN_L_Pin|M1_LIN_R_Pin|M2_LIN_L_Pin|M2_LIN_R_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED_DIR_GPIO_Port, LED_DIR_Pin);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTA, LL_GPIO_AF_EXTI_LINE5);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTA, LL_GPIO_AF_EXTI_LINE6);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_5;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_6;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinMode(INCR_GPIO_Port, INCR_Pin, LL_GPIO_MODE_FLOATING);

  /**/
  LL_GPIO_SetPinMode(DCR_GPIO_Port, DCR_Pin, LL_GPIO_MODE_FLOATING);

  /**/
  GPIO_InitStruct.Pin = M1_LIN_L_Pin|M1_LIN_R_Pin|M2_LIN_L_Pin|M2_LIN_R_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = SW_FW_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(SW_FW_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED_DIR_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(LED_DIR_GPIO_Port, &GPIO_InitStruct);

  NVIC_SetPriority(EXTI9_5_IRQn, 0); // Set priority (adjust as needed)
  NVIC_EnableIRQ(EXTI9_5_IRQn);      // Enable the EXTI9_5_IRQn interrupt in the NVIC
}

/* USER CODE BEGIN 4 */
void delay_ms(uint16_t t) {
	volatile unsigned long l = 0;
	for(uint16_t i = 0; i < t; i++) {
		for(l = 0; l < 72000; l++) {}
	}
}

void CheckDirMotor(void) {

	if(LL_GPIO_IsInputPinSet(GPIOA, SW_FW_Pin)) {
		if(ButtonState == EXTIN) {
		TIM2->CCR2 = temp_arr2 - 1;
		ButtonState = NOEXTIN;
		}
		if(CurrentState != FORWARD) {
			ConfigTimerIOCW();
			TIM2->CR1 = 1;
			CurrentState = FORWARD;
			//LL_GPIO_SetOutputPin(GPIOB, LED_DIR_Pin);
			}
		}
	else {
		if(ButtonState == EXTIN) {
		TIM2->CCR3 = temp_arr2 - 1;
		ButtonState = NOEXTIN;
		}
		if(CurrentState != BACK) {
			ConfigTimerIOCCW();
			TIM2->CR1 = 1;
			CurrentState = BACK;
			LL_GPIO_ResetOutputPin(GPIOB, LED_DIR_Pin);
		}
	}
}
void ConfigTimerIOCW(void) {
	TIM2->CCR3 = 0; /* Make M1_HIN_R_Pin = 0 */
	TIM2->CCR2 = 0; /* Make M1_HIN_L_Pin = 0 */
	delay_ms(2);
	TIM2->CR1 = 0;  /* Stop counter */
	LL_GPIO_ResetOutputPin(GPIOA, M1_LIN_L_Pin);
	delay_ms(2);
	LL_GPIO_SetOutputPin(GPIOA, M1_LIN_R_Pin);
	TIM2->CCR2 = temp_arr2 - 1;  /*Make M1_HIN_L_Pin =1*/
}
void ConfigTimerIOCCW(void) {
	 TIM2->CCR2 = 0; /* Make M1_HIN_L_Pin = 0 */
	 TIM2->CCR3 = 0; /* Make M1_HIN_L_Pin = 0 */
	 delay_ms(2);
	 TIM2->CR1 = 0;
	 LL_GPIO_ResetOutputPin(GPIOA, M1_LIN_R_Pin);
	 delay_ms(2);
	 LL_GPIO_SetOutputPin(GPIOA, M1_LIN_L_Pin);
	 TIM2->CCR3 = temp_arr2 - 1;  /*Make M1_HIN_L_Pin =1*/

}


/*
void IsSwapStateMotor(void) {
	LL_GPIO_ResetOutputPin(LED_DIR_GPIO_Port, LED_DIR_Pin);
	if(flag_swap == 1) {
		NVIC_DisableIRQ(EXTI3_IRQn);
		if(CurrentState == FORWARD) {
			LL_GPIO_SetOutputPin(LED_DIR_GPIO_Port, LED_DIR_Pin);
			RCC->APB1ENR &= ~(1U << 0);	 //cmt: disable timer2
			delay_ms(100);				//cmt: time to mosfet off
			RCC->APB1ENR |= (1<<0); 	// enable TIM2 clock
			GPIOA->CRL = 0x44344B34;  // PA2: alternate func. output //
			GPIOA->ODR &= ~(1 << 1);
			delay_ms(10);
			TIM2->CCR3 = temp_arr2;
			TIM2->CCER = 0x1 << 8; // CC3P = 0, CC3E = 1 //
			TIM2->CCMR2 = 0x0070; // toggle channel 3 //
			TIM2->ARR =799;
			TIM2->CR1 = 1; // start counting up //
			flag_swap = 0;
		}
		NVIC_EnableIRQ(EXTI3_IRQn);
	}

}
*/

void EXTI9_5_IRQHandler(void)
{
    if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_5) != RESET)
    {
    	flag_debounce = 1;
        if (ms_counter > DEBOUNCE_DELAY)
        {
        	LL_GPIO_SetOutputPin(GPIOB, LED_DIR_Pin);
           // EXT USER CODE BEGIN //
        	temp_arr2=2880;
        	ms_counter = 0;
        	flag_debounce = 0;
        	ButtonState = EXTIN;
        }
        	//EXT USER CODE END //
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_5);
     }

    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_6) != RESET) {
    	flag_debounce = 1;
    	        if (ms_counter > DEBOUNCE_DELAY)
    	        {
    	           // EXT USER CODE BEGIN //
    	        	temp_arr2=1260;
    	        	ms_counter = 0;
    	        	flag_debounce = 0;
    	        	ButtonState = EXTIN;
    	        }
                // Handle the interrupt
                LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_6);
                // Your interrupt handling code here
            }
}


void msSysTickIntHandler (void) {
	if(flag_debounce == 1) {
	ms_counter++;
	}
}

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
