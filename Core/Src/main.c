/**
 * @mainpage Quadcopter Flight Control System
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for a quadcopter design by Aaron Lubisnky and Dane Carroll. This project was taken on and
 * guided by Cal Poly's ME507 course (Graduate Mechanical Control Systems) and instructed by lecturer, Charlie Refvem.
 *
 * The drone has a custom designed and 3D printed frame.
 * It has been designed to operate using a custom PCB including an onboard STM2F411CEU6 MCU and BNO055 IMU. Though,
 * currently uses a Blackpill development board and Bosch BN055 development board due to in house PCB manufacturing failures.
 * The arming sequence is tailored towards ReadyTosky ESCs and the bluetooth module (HC05) is intended to interact with
 * a PC running a custom python script for drone control and plotting blackboc data.
 *
 * @section features_sec Key Features
 *
 * - PID control for roll, pitch, and yaw
 * - BNO055 IMU integration
 * - ESC motor control with safety limits
 * - Flight data logging (blackbox)
 *
 * @section hardware_sec Hardware Requirements
 *
 * - STM32F4 Development Board
 * - BNO055 IMU Module
 * - HC05 Bluetooth Module
 * - 4x Electronic Speed Controllers (ESCs)
 * - 4x 3 Phase Brushless Motors
 * - 4-channel Power Distribution Board
 * - 3s/4s Battery
 * - Status LEDs
 *
 * @section usage_sec Flight Preparation
 *
 * 1. Start program and observe LEDs.
 * 2. Solid Red LED indicates that the BNO055 is calibrating. Maneuver quadcopter in figure-8 pattern until blinking stops LED is calibrated.
 * 3. Use controller to begin arming sequence by pulling left joystick fully to the left. The Red LED should blink at 2Hz.
 * 4. Connect ESCs to battery by switching both mechanical switches. The program will already be applying 100% duty cycle. Wait for audio prompts to cycle.
 * 5. After 4 rapid ESC beeps followed by a single beep. Hold LT on Xbox controller for 5 seconds. This lowers PWM to to complete arming sequence.
 * 6. Once LT is released, wait for another audio prompt (3 beeps, pause, beep). Pull LT fully to the right to begin operational mode.
 * 2. Follow flying instructions until flight concludes. To conclude flight, any of the safety stops will suffice. see section below.
 * 3. If quadcopter no longer in use, unplug battery from power distribution board.
 *
 * @section modules_sec Modules
 *
 * - @ref ESC.c "ESC Driver" - Motor control and PID implementation
 * - @ref BNO055.c "BNO055 Driver" - IMU sensor interface
 * - @ref HC05.c "Bluetooth Driver" = Bluetooth receive and transmit, data parsing
 * * @section flight_instr Flight Instructions
 *  For use only with gaming controller in operational mode
 * - Left Trigger: decrement thrust
 * - Right Trigger: increment thrust
 * - Left Joystick (horizontal): Roll
 * - Left Joystick (vertical): Pitch
 * - Right Joystick (horizontal): decrement/increment Yaw
 *
 * * @section safety_sec Safety Stops
 *
 * - Option 1: Push right joystick fully up on controller
 * - Option 2: Press ENTER on Python Terminal
 * - Option 3: Flip mechanical switches on quadcopter
 *
 * All options will stop motors from spinning and dump 'Blackbox' data to be analyzed through Python Script communicating with the HC05.
 *
 * @author Aaron Lubinsky & Dane Carroll
 * @version 1.4
 * @date 2025
 */




/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file main.c
  * @brief Main program for quadcopter control. derived as FSM mastermind, implemented in pupre C.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <unistd.h>
#include <string.h>
#include "BNO055.h"
#include "HC05.h"
#include "ESC.h"

//#include "HC05.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PID_SCALE 100000 //100,000 allow us to use ints, not floats for Kp of 1.1 --> 1,100_000
#define true 1
#define false 0
#define BT_MSG_LEN 37
//#define BT_MSG_LEN 2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
UART_HandleTypeDef *BT_UART_ptr = &huart2;
UART_HandleTypeDef *STLink_UART_ptr = &huart1;
volatile int imu_request = 0;

//State
int state = 0;
int32_t roll_set, pitch_set, yaw_set, effort_set;                // from user control
int32_t roll_true, pitch_true, yaw_true;             // from IMU
int32_t roll_effort, pitch_effort, yaw_effort;
int stopFlag = false; //triggered when effortSet is 0

// PID vars
int32_t roll_integral, pitch_integral, yaw_integral;
int32_t roll_derivative, pitch_derivative, yaw_derivative;
int32_t roll_error, pitch_error, yaw_error;        // error for sample n-1
int32_t last_roll_error, last_pitch_error, last_yaw_error;        // error for sample n-1
int32_t last_last_roll_error, last_last_pitch_error, last_last_yaw_error; // error for sample n-2

// Gains
int32_t K_effort = 50000;
// Roll Axis
int32_t Kp_roll = 450;
int32_t Ki_roll = 30;
int32_t Kd_roll = 120000;
// Pitch Axis
int32_t Kp_pitch = 350;
int32_t Ki_pitch = 40;
int32_t Kd_pitch = 60000;

// Yaw Axis
int32_t Kp_yaw  =0; //Yaw is currently disable
int32_t Ki_yaw = 0; //Yaw is currently disable
int32_t Kd_yaw = 0; //Yaw is currently disable


//BT
uint8_t BT_RxBuf[BT_MSG_LEN]; //This buffer is filled by DMA UART and processed by HC05 driver
int     dumpFlag = 0;  //This flag is triggered by safety stop and tells program to transmit blackbox data

//IMU



//


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 if (state == 0){
		 BNO_Init(); //initialize BNO055
		 HAL_UART_Receive_DMA(&huart2, BT_RxBuf, BT_MSG_LEN-1); //allow for UART DMA Callback
		 state = 1; //Move to next state

	 }else if (state == 1){ //Arming
		 if (roll_set < -10000){ //Wait for user input (done manually after mechanical switches enabled)
			 armESC(); //begin arming sequence. Leave arming sequence after user input in arming function
			 state = 2; //move to operational state
		 }
		 effort_set = 0; //Safety precaution: eliminates posibility of user mistake: holding accelerate after arming sequence but before start signal

	 }else if(state == 2){ //State 2 is operation (flying) mode where the drone reads the BNO, updates motor PWM to the latest bluetooth DMA
		  BNO_Read(&roll_true, &pitch_true, &yaw_true); //Read BNO, log blackbox data
		  update_Motors(); //PID and update signal
		  if (dumpFlag == 1){ //if user stop detected anywhere
		  			effort_set = 0;
		  			stopFlag = true;
		  			update_Motors();//stop motors
		  			state = 3; //move to dump state
		  		 }


	 }else if(state == 3){//This state is triggered by user input. It sends blackBox data then returns to state 2
		 	dumpBlackbox(); //dump data to HC05
		 	dumpFlag = 0; //reset dumpflag
		 	HAL_UART_Receive_DMA(&huart2, BT_RxBuf, BT_MSG_LEN-1); //reenable UART DMA
		 	while (roll_set < 10000){
		 		//wait for user input
		 	}
		 	state = 2; //ESCs are armed, can return back to state 2
		 	effort_set = 0;
		 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   // Set PA0 High (go signal)

 }




	  }//while loop close


  //main loop close


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 50;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20660;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 400;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 3200;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.BaudRate = 9600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, US_ECHO_Pin|US_TRIG_Pin|GPIO_PIN_14|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : US_ECHO_Pin US_TRIG_Pin PB14 PB9 */
  GPIO_InitStruct.Pin = US_ECHO_Pin|US_TRIG_Pin|GPIO_PIN_14|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



/**
  * @brief  This callback function is triggered when UART2 (HC05 Stream) fills BT_RxBuf (Direct to Memory).
  *  It adds an end message statment, sends the buffer to the processInput function and sets the imu_request flag
  *
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//should trigger when DMA reads complete message
{    if (huart->Instance == USART2) {
        // Null-terminate just in case you're using sscanf or string functions
        BT_RxBuf[BT_MSG_LEN - 1] = '\0';

        // Process entire message

        processInput((char *)BT_RxBuf, &roll_set, &pitch_set, &yaw_set, &effort_set, &dumpFlag);
        }
        // Restart DMA to receive next message
        //HAL_UART_Receive_DMA(&huart2, BT_RxBuf, BT_MSG_LEN); //set up this function to run on next BT input
        imu_request = true; //set up IMU to run when interupt exits
        HAL_UART_Receive_DMA(&huart2, BT_RxBuf, BT_MSG_LEN-1); //set up this function to run on next BT input
}
/**
  * @brief  This function is used to send printf() statments to the ST-Link UART
  * @retval None
  */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
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
