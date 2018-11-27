/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#define NUMBER_OF_SAMPLES 32000
#include "main.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "stdlib.h"

/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "arm_common_tables.h"
#include "stm32l475e_iot01_qspi.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;

DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Filter_HandleTypeDef hdfsdm1_filter1;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;

UART_HandleTypeDef huart1;

osThreadId soundThreadTaskHandle;
osThreadId blinkThreadTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//Flags
int interrupt_flag = 0;
int countValues = 0;
int sineWave1Flag = 0;
int buttonPressed = 0;
int timer = 0;
int chooseThread = 0;

//Variables
float32_t freq1 = 261.63;
float32_t freq2 = 392.00;
float32_t Ts = 16000;
float32_t epsilon = 0.0001;
int MAX_NUMBER_OF_ITERATIONS = 1000;

//QSPI Addresses
uint32_t write_address1 = ((uint32_t)0x0);
uint32_t read_address1 = ((uint32_t)0x0);
uint32_t write_address2 = ((uint32_t)0x186A0);
uint32_t read_address2 = ((uint32_t)0x186A0);
uint32_t write_address3 = ((uint32_t)0x30D40);
uint32_t read_address3 = ((uint32_t)0x30D40);
uint32_t write_address4 = ((uint32_t)0x493E0);
uint32_t read_address4 = ((uint32_t)0x493E0);
uint32_t write_address5 = ((uint32_t)0x61A80);
uint32_t read_address5 = ((uint32_t)0x61A80);
uint32_t write_address6 = ((uint32_t)0x7A120);
uint32_t read_address6 = ((uint32_t)0x7A120);
uint32_t write_address7 = ((uint32_t)0x927C0);
uint32_t read_address7 = ((uint32_t)0x927C0);
uint32_t write_address8 = ((uint32_t)0xAAE60);
uint32_t read_address8 = ((uint32_t)0xAAE60);
uint32_t write_address9 = ((uint32_t)0xC3500);
uint32_t read_address9 = ((uint32_t)0xC3500);

	
//mixed waves	
//uint8_t x1[NUMBER_OF_SAMPLES];
//uint8_t x2[NUMBER_OF_SAMPLES];	
uint8_t x1[NUMBER_OF_SAMPLES];
uint8_t x2[NUMBER_OF_SAMPLES];
//coefficients
float32_t a11 = 0.9794;
float32_t a12 = -0.5484;
float32_t a21 = -0.2656;
float32_t a22 = -0.0963;
float32_t mean1 = 0;
float32_t mean2 = 0;

//FAST ICA//
int means[2];

//cov matrix
float32_t var1 = 0, cov1 = 0;
float32_t cov2 = 0, var2 = 0;

float32_t sols1[2];
float32_t sols2[2];

float32_t eig[2];


arm_matrix_instance_f32 eigDiagMatrix;
float32_t eigDiagMatrixData[4];

arm_matrix_instance_f32 eigVecMatrix;
float32_t eigVecMatrixData[4];

arm_matrix_instance_f32 whiteningMatrix;
float32_t whiteningMatrixData[4];

arm_matrix_instance_f32 dewhiteningMatrix;
float32_t dewhiteningMatrixData[4];

arm_matrix_instance_f32 newVectorMatrix;
float32_t newVectorMatrixData[4];

//A Matrix
arm_matrix_instance_f32 A;
float32_t Adata[4];
	
//W Matrix
arm_matrix_instance_f32 W;
float32_t Wdata[4];


uint8_t bufferValues[100];
char buffer[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_DAC1_Init(void);

void StartDefaultTask(void const * argument);
void blinkThread(void const *arg);
void soundThread(void const *arg);
void sineWave(float freq, int write_address);
void remMean(void);
void eigValues(void);
void eigVectors(void);
void mult(void);
void cov();
void squareRoot(float32_t a);
void remMean(void);
void findMean(void);
void fpica(void);
void eraseMemory();
int transmitSineWave(int readAddress);
void readMixWaves(int readAddress1, int readAddress2);
int mixWaves(int readAddress1, int readAddress2);
int unmixedWaves(uint32_t readAddress1, uint32_t readAddress2);
float32_t RandomNumber(float Max);
float32_t normalize(float32_t vector[], int length);
float32_t * simultSolve(float32_t a, float32_t b, float32_t p, float32_t q, float32_t * sols);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

int fputc(int ch, FILE *f) {
  while (HAL_OK != HAL_UART_Transmit(&huart1, (uint8_t *) &ch, 1, 30000));
  return ch;
}
int fgetc(FILE *f) {
  uint8_t ch = 0;
  while (HAL_OK != HAL_UART_Receive(&huart1, (uint8_t *)&ch, 1, 30000));
  return ch;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DFSDM1_Init();
  MX_DAC1_Init();
	BSP_QSPI_Init();
	
  /* USER CODE BEGIN 2 */
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
  /* USER CODE END 2 */

	arm_mat_init_f32(&eigDiagMatrix, 2, 2, eigDiagMatrixData);
	arm_mat_init_f32(&eigVecMatrix, 2, 2, eigVecMatrixData);
	arm_mat_init_f32(&whiteningMatrix, 2, 2, whiteningMatrixData);
	arm_mat_init_f32(&dewhiteningMatrix, 2, 2, dewhiteningMatrixData);
	arm_mat_init_f32(&newVectorMatrix, 2, 32000, newVectorMatrixData);
	arm_mat_init_f32(&A, 2, 2, Adata);
	arm_mat_init_f32(&W, 2, 2, Wdata);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
	sineWave(freq1, write_address1);
	//transmitSineWave(read_address1);
	//BSP_QSPI_Erase_Chip();
	//eraseMemory();
	sineWave(freq2, write_address2);
	//transmitSineWave(read_address2);
					
	//unmixedWaves(read_address1,read_address2);
	mixWaves(read_address1, read_address2);
	//readMixWaves(read_address3, read_address4);
	findMean();
	//readMixWaves(read_address3, read_address4);
	//HAL_Delay(10000);
	remMean();
	//HAL_Delay(10000);
	readMixWaves(read_address5, read_address6);
	//BSP_QSPI_EnableMemoryMappedMode();

  /* USER CODE BEGIN RTOS_THREADS */
	//osThreadDef(soundTask, soundThread, osPriorityNormal, 0, 128);
	//soundThreadTaskHandle = osThreadCreate(osThread(soundTask), NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  //osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//remMean();
	//cov();
	//eigValues();
	//eigVectors();
	while (1)
  {
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_SAI1
                              |RCC_PERIPHCLK_DFSDM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 32;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* DAC1 init function */
static void MX_DAC1_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT2 config 
    */
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DFSDM1 init function */
static void MX_DFSDM1_Init(void)
{

  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = DISABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC4_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 128;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  hdfsdm1_filter1.Instance = DFSDM1_Filter1;
  hdfsdm1_filter1.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter1.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter1.Init.RegularParam.DmaMode = DISABLE;
  hdfsdm1_filter1.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC4_ORDER;
  hdfsdm1_filter1.Init.FilterParam.Oversampling = 128;
  hdfsdm1_filter1.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_AUDIO;
  hdfsdm1_channel1.Init.OutputClock.Divider = 32;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_FALLING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = -1152;
  hdfsdm1_channel1.Init.RightBitShift = 0x0D;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_AUDIO;
  hdfsdm1_channel2.Init.OutputClock.Divider = 32;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = -1152;
  hdfsdm1_channel2.Init.RightBitShift = 0x0D;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_1, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter1, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Pinout Configuration
*/
static void MX_GPIO_Init(void){
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

void soundThread(void const * argument) {
		int index = 0;
		osThreadDef(blinkTask, blinkThread, osPriorityNormal, 0, 128);
		blinkThreadTaskHandle = osThreadCreate(osThread(blinkTask), NULL);
		while(1) {
			/**
			if(interrupt_flag == 1 && chooseThread != 0) {
				interrupt_flag = 0;
				if(index == 32000) index = 0;
				
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, x1[index]);
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_8B_R, x2[index]);
				index++;
			}
			**/
			if(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)){
				buttonPressed = 1;
				while(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)){}
				buttonPressed = 0;
				chooseThread = chooseThread + 1;
				/**
				if(chooseThread == 1) {
					unmixedWaves(read_address1, read_address2);
				} else if(chooseThread == 2){
					remMean();
				} else if(chooseThread == 3) {
					mixWaves(read_address1, read_address2);
				} else if(chooseThread == 4) {
					osThreadTerminate(blinkThreadTaskHandle);
					BSP_QSPI_Erase_Chip();
					osThreadCreate(osThread(blinkTask), NULL);
					while(1){};
				}
				**/
				if(chooseThread == 1) {
					remMean();
					//sineWave(freq1, write_address1);
					//transmitSineWave(read_address1);
				} else if(chooseThread == 2) {
					osThreadTerminate(blinkThreadTaskHandle);
					eraseMemory();
					osThreadCreate(osThread(blinkTask), NULL);
					while(1){};
				}
			}
			
			timer = 0;
			
			
		}
}

void eraseMemory() {
		int status = 0;
		for(int i=0; i<256; i++) {
			if(BSP_QSPI_Erase_Sector(i) == QSPI_ERROR) {
				status = -1;
			}
		}
		
		if(status == -1) {
			HAL_UART_Transmit(&huart1, (uint8_t *)"ERROR\n", 6, 30000);
		} else {
			HAL_UART_Transmit(&huart1, (uint8_t *)"OK\n", 3, 30000);
		}
}

int mixWaves(int readAddress1, int readAddress2){
	float32_t buffer1[100];
	float32_t buffer2[100];
	int temp1 = readAddress1;
	int temp2 = readAddress2;
	int temp3 = write_address3;
	int temp4 = write_address4;
	
	for(int i=0; i<16; i++) {
		memset(buffer1, 0, 400);
		memset(buffer2, 0, 400);
		BSP_QSPI_Read((uint8_t *)buffer1, temp1, 400);
		BSP_QSPI_Read((uint8_t *)buffer2, temp2, 400);
		temp1+=400;
		temp2+=400;
		//HAL_UART_Transmit(&huart1, (uint8_t *)"Running\n", 8, 30000);
		for(int j=0; j<100; j++) {
			float32_t mult_11 = a11*buffer1[j];
			float32_t mult_12 = a12*buffer2[j];
			float32_t mult_21 = a21*buffer1[j];
			float32_t mult_22 = a22*buffer2[j];
			float32_t X1 = mult_11 + mult_12;
			float32_t X2 = mult_21 + mult_22;	
			
			buffer1[j] = X1;
			buffer2[j] = X2;
		}
		BSP_QSPI_Write((uint8_t *)buffer1, temp3, 400);
		BSP_QSPI_Write((uint8_t *)buffer2, temp4, 400);
		temp3+=400;
		temp4+=400;
		memset(buffer1, 0, 400);
		memset(buffer2, 0, 400);
		//HAL_Delay(2000);
	}

	//memset(buffer, 0 ,strlen(buffer));
	//sprintf(buffer, "%d\n", x1[i]);

	return 1;
}

void readMixWaves(int readAddress1, int readAddress2) {
	float32_t buffer1[100];
	float32_t buffer2[100];
	
	int temp1 = readAddress1;
	int temp2 = readAddress2;
	
	for(int i=0; i<16; i++) {
		memset(buffer1, 0, 400);
		memset(buffer2, 0, 400);
		BSP_QSPI_Read((uint8_t *)buffer1, temp1, 400);
		BSP_QSPI_Read((uint8_t *)buffer2, temp2, 400);
		temp1+=400;
		temp2+=400;
		
		for(int j=0; j<100; j++) {
			memset(buffer, 0, strlen(buffer));
			sprintf(buffer, "value 1: %.2f value 2: %.2f\n", buffer1[j], buffer2[j]);
			HAL_UART_Transmit(&huart1, (uint8_t *)&buffer[0], strlen(buffer), 30000);
		}
	}
}

int unmixedWaves(uint32_t readAddress1, uint32_t readAddress2){	
	BSP_QSPI_Read(x1, readAddress1, 32000);
	BSP_QSPI_Read(x2, readAddress2, 32000);
	
	return 1;
}

void squareRoot(float32_t a) {
	float32_t ans;
	
	arm_sqrt_f32(a, &ans);
	memset(buffer, 0, strlen(buffer));
	sprintf(buffer, "%0.3f\n", ans);
	HAL_UART_Transmit(&huart1, (uint8_t *)&buffer[0], strlen(buffer), 30000); 
	
}

void mult(){
	
	arm_matrix_instance_f32 matrix_1;
	arm_matrix_instance_f32 matrix_2;
	arm_matrix_instance_f32 matrix_res;
	
	float32_t data_1[4];
	float32_t data_2[4];
	float32_t data_res[4];
	
/*	
	matrix_1.numCols = 2;
	matrix_1.numRows = 2;
	matrix_1.pData = data_1;
	
	matrix_2.numCols = 2;
	matrix_2.numRows = 2;
	matrix_2.pData = data_2;
	*/
	
	arm_mat_init_f32(&matrix_1, 2, 2, data_1);
	arm_mat_init_f32(&matrix_2, 2, 2, data_2);
	arm_mat_init_f32(&matrix_res, 2, 2, data_res);
	
	data_1[0] = 1;
	data_1[1] = 2;
	data_1[2] = 3;
	data_1[3] = 4;
	
	data_2[0] = 5;
	data_2[1] = 6;
	data_2[2] = 7;
	data_2[3] = 8;
	
	arm_mat_mult_f32(&matrix_1, &matrix_2, &matrix_res);
	
	for(int i = 0; i < 4; i++){
		memset(buffer, 0 ,strlen(buffer));
		sprintf(buffer, "%.2f\n", data_res[i]);
		HAL_UART_Transmit(&huart1, (uint8_t *)&buffer[0], strlen(buffer), 30000); 
	}
	
}


void sineWave(float freq, int write_address) {
	float32_t buffer[100];
	memset(buffer, 0, 400);
	int temp = write_address;
	int index = 0;
	for(int i=0; i<1600; i++) {
		buffer[index++] = arm_sin_f32(2 * PI * freq * i * (1/Ts));
		//sample = sample + 1.0;
		//sample = sample * 127.0;
		//sample_int = (uint8_t) sample;
		//buffer[index++] = sample_int;
		
		if(index == 100) {
				BSP_QSPI_Write((uint8_t *) buffer, temp, 400);
				memset(buffer, 0, 400);
				index = 0;
				temp += 400;
				HAL_Delay(2000);
		}
	}
}

int transmitSineWave(int readAddress){
	float32_t readValue[100];
	int temp = readAddress;
	for(int i=0; i<16; i++) {
		memset(readValue, 0, 400);
		BSP_QSPI_Read((uint8_t *)readValue, temp, 400);
		temp+=400;
		for(int j=0; j<100; j++) {
			memset(buffer, 0, strlen(buffer));
			sprintf(buffer, "%.3f\n", readValue[j]);
			HAL_UART_Transmit(&huart1, (uint8_t *)&buffer[0], strlen(buffer), 30000);
		}
	}
	return 1;
}

void blinkThread(void const *arg){
	while(1){
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		osDelay(1000);
	}
}



void remMean() {
	float32_t buffer1[100];
	float32_t buffer2[100];
	int temp1 = read_address3;
	int temp2 = read_address4;
	int temp3 = write_address5;
	int temp4 = write_address6;
	float32_t sum1 = 0;
	float32_t sum2 = 0;
	
	for(int i=0; i<16; i++) {
		memset(buffer1, 0, 400);
		memset(buffer2, 0, 400);
		BSP_QSPI_Read((uint8_t *)buffer1, temp1, 400);
		BSP_QSPI_Read((uint8_t *)buffer2, temp2, 400);
		temp1+=400;
		temp2+=400;
		//HAL_UART_Transmit(&huart1, (uint8_t *)"Running\n", 8, 30000);
		for(int j=0; j<100; j++) {
			float32_t mult_11 = a11*buffer1[j];
			float32_t mult_12 = a12*buffer2[j];
			float32_t mult_21 = a21*buffer1[j];
			float32_t mult_22 = a22*buffer2[j];
			float32_t X1 = mult_11 + mult_12;
			float32_t X2 = mult_21 + mult_22;	

			X1 = buffer1[j] - mean1;
			X2 = buffer2[j] - mean2;
			sum1 += X1;
			sum2 += X2;
			
			buffer1[j] = X1;
			buffer2[j] = X2;
		}
		BSP_QSPI_Write((uint8_t *)buffer1, temp3, 400);
		BSP_QSPI_Write((uint8_t *)buffer2, temp4, 400);
		temp3+=400;
		temp4+=400;
		memset(buffer1, 0, 400);
		memset(buffer2, 0, 400);
		//HAL_Delay(2000);
	}
	memset(buffer, 0, strlen(buffer));
	sprintf(buffer, "sum 1: %.4f sum 2: %.4f\n", sum1, sum2);
	HAL_UART_Transmit(&huart1, (uint8_t *)&buffer[0], strlen(buffer), 30000);
	
}

void findMean(void){
	float32_t sum1 = 0;
	float32_t sum2 = 0;
	
	float32_t buffer1[100];
	float32_t buffer2[100];
	
	int temp1 = read_address3;
	int temp2 = read_address4;
	
	for(int i=0; i<16; i++) {
		memset(buffer1, 0, 400);
		memset(buffer2, 0, 400);
		BSP_QSPI_Read((uint8_t *)buffer1, temp1, 400);
		BSP_QSPI_Read((uint8_t *)buffer2, temp2, 400);
		temp1+=400;
		temp2+=400;
		
		for(int j=0; j<100; j++) {
			sum1 += buffer1[j];
			sum2 += buffer2[j];
		}
	}
	
	mean1 = sum1/1600;
	mean2 = sum2/1600;
	
	memset(buffer, 0, strlen(buffer));
	sprintf(buffer, "mean 1: %.4f mean 2: %.4f\n", mean1, mean2);
	HAL_UART_Transmit(&huart1, (uint8_t *)&buffer[0], strlen(buffer), 30000);

	
	/**
	for(int i=0; i<16; i++) {
		memset(buffer1, 0, 400);
		memset(buffer2, 0, 400);
		BSP_QSPI_Read((uint8_t *)buffer1, temp1, 400);
		BSP_QSPI_Read((uint8_t *)buffer2, temp2, 400);
		temp1+=400;
		temp2+=400;
		for(int j=0; j<100; j++) {
			buffer1[j] = buffer1[j] - mean1;
			buffer2[j] = buffer2[j] - mean2;
			sum1 += buffer1[j];
			sum2 += buffer2[j];
		}
		BSP_QSPI_Write((uint8_t *)buffer1, temp3, 400);
		BSP_QSPI_Write((uint8_t *)buffer2, temp4, 400);
		temp3+=400;
		temp4+=400;
		HAL_Delay(2000);
	}
	
	temp3 = read_address5;
	temp4 = read_address6;
	
	for(int i=0; i<16; i++) {
		memset(buffer1, 0, 400);
		memset(buffer2, 0, 400);
		BSP_QSPI_Read((uint8_t *)buffer1, temp3, 400);
		BSP_QSPI_Read((uint8_t *)buffer2, temp4, 400);
		temp1+=400;
		temp2+=400;
		
		for(int j=0; j<100; j++) {
			memset(buffer, 0, strlen(buffer));
			sprintf(buffer, "value 1: %.2f value 2: %.2f\n", buffer1[j], buffer2[j]);
			HAL_UART_Transmit(&huart1, (uint8_t *)&buffer[0], strlen(buffer), 30000);
		}
	}
	
	memset(buffer, 0, strlen(buffer));
	sprintf(buffer, "sum 1: %.4f sum 2: %.4f\n", sum1, sum2);
	HAL_UART_Transmit(&huart1, (uint8_t *)&buffer[0], strlen(buffer), 30000);**/
	
	/**
	
	memset(removedMean1, 0, 400);
	memset(removedMean2, 0, 400);
	BSP_QSPI_Read((uint8_t *)removedMean1, read_address3, 400);
	BSP_QSPI_Read((uint8_t *)removedMean2, read_address4, 400);
	
	for(int i=0; i<100; i++) {
		memset(buffer, 0, strlen(buffer));
		sprintf(buffer, "value 1: %.2f value 2: %.2f\n", removedMean1[i], removedMean2[i]);
		HAL_UART_Transmit(&huart1, (uint8_t *)&buffer[0], strlen(buffer), 30000);
	}
	
		memset(removedMean1, 0, 400);
	memset(removedMean2, 0, 400);
	BSP_QSPI_Read((uint8_t *)removedMean1, read_address3+400, 400);
	BSP_QSPI_Read((uint8_t *)removedMean2, read_address4+400, 400);
	
	for(int i=0; i<100; i++) {
		memset(buffer, 0, strlen(buffer));
		sprintf(buffer, "value 1: %.2f value 2: %.2f\n", removedMean1[i], removedMean2[i]);
		HAL_UART_Transmit(&huart1, (uint8_t *)&buffer[0], strlen(buffer), 30000);
	}
	
			memset(removedMean1, 0, 400);
	memset(removedMean2, 0, 400);
	BSP_QSPI_Read((uint8_t *)removedMean1, read_address3+800, 400);
	BSP_QSPI_Read((uint8_t *)removedMean2, read_address4+800, 400);
	
	for(int i=0; i<100; i++) {
		memset(buffer, 0, strlen(buffer));
		sprintf(buffer, "value 1: %.2f value 2: %.2f\n", removedMean1[i], removedMean2[i]);
		HAL_UART_Transmit(&huart1, (uint8_t *)&buffer[0], strlen(buffer), 30000);
	}
	
				memset(removedMean1, 0, 400);
	memset(removedMean2, 0, 400);
	BSP_QSPI_Read((uint8_t *)removedMean1, read_address3+1200, 400);
	BSP_QSPI_Read((uint8_t *)removedMean2, read_address4+1200, 400);
	
	for(int i=0; i<100; i++) {
		memset(buffer, 0, strlen(buffer));
		sprintf(buffer, "value 1: %.2f value 2: %.2f\n", removedMean1[i], removedMean2[i]);
		HAL_UART_Transmit(&huart1, (uint8_t *)&buffer[0], strlen(buffer), 30000);
	}**/
}

/**

**/
void cov(){
	float32_t removedMean1[100]; //x1 - mean
	float32_t removedMean2[100]; //x1 - mean
	int temp1 = read_address3;
	int temp2 = read_address4;
	float32_t sum1 = 0;
	float32_t sum2 = 0;
	
	
	for(int j=0; j<16; j++) {
		memset(removedMean1, 0, 400);
		memset(removedMean2, 0, 400);
		BSP_QSPI_Read((uint8_t *)removedMean1, temp1, 400);
		BSP_QSPI_Read((uint8_t *)removedMean2, temp2, 400);
		temp1+=400;
		temp2+=400;
		for(int i=0; i<100; i++) {
			var1 += (removedMean1[i] * removedMean1[i]);
			cov1 += (removedMean1[i] * removedMean2[i]);
			cov2 = cov1;
			var2 += (removedMean2[i] * removedMean2[i]);
			//sum1+=removedMean1[i];
			//sum2+=removedMean2[i];
		}
		HAL_Delay(2000);
	}
	
	var1 = var1/1600;
	cov1 = cov1/1600;
	cov2 = cov2/1600;
	var2 = var2/1600;
	
	memset(buffer, 0, strlen(buffer));
	sprintf(buffer, "var 1: %.2f cov 1: %.2f\ncov 2: %.2f var 2: %.2f\n", var1, cov1, cov2, var2);
	//sprintf(buffer, "sum 1: %.2f sum 2: %.2f\n", sum1, sum2);
	HAL_UART_Transmit(&huart1, (uint8_t *)&buffer[0], strlen(buffer), 30000);
}

void eigValues(){
	
	float32_t sqrtExp = 0;
	float32_t sqrtVals[2];
	float32_t sumVar = var1 + var2;
	
	sqrtExp = (var1 + var2)*(var1 + var2) - 4*((var1*var2) - (cov1*cov2));
	arm_sqrt_f32(sqrtExp, &sqrtVals[0]);
	arm_sqrt_f32(sqrtExp, &sqrtVals[1]);
	
	sqrtVals[1] = -1*sqrtVals[1];
	
	eig[0] = (sumVar + sqrtVals[0])/2;
	eig[1] = (sumVar + sqrtVals[1])/2;
	
	eigDiagMatrixData[0] = eig[0];
	eigDiagMatrixData[1] = 0;
	eigDiagMatrixData[2] = 0;
	eigDiagMatrixData[3] = eig[1];
	
	memset(buffer, 0 ,strlen(buffer));
	sprintf(buffer, "eig1: %.2f eig2: %.2f\n", eig[0], eig[1]);
	HAL_UART_Transmit(&huart1, (uint8_t *)&buffer[0], strlen(buffer), 30000); 
}

void eigVectors(){
	simultSolve(var1 - eig[0], cov1, cov2, var2 - eig[0], sols1);
	simultSolve(var1 - eig[1], cov1, cov2, var2 - eig[1], sols2);
	//simultSolve(1, 1, -2, -2, sols1);
	//simultSolve(2,1,-2,-1, sols2);
	
	eigVecMatrixData[0] = sols1[0];		//x1
	eigVecMatrixData[1] = sols2[0];		//x2
	eigVecMatrixData[2] = sols1[1];		//y1
	eigVecMatrixData[3] = sols2[1];		//y2
	
	//memset(buffer, 0 ,strlen(buffer));
	//sprintf(buffer, "x1: %.2f x2: %.2f\ny1: %.2f y2: %.2f\n", sols1[0], sols2[0], sols1[1], sols2[1]);
	//HAL_UART_Transmit(&huart1, (uint8_t *)&buffer[0], strlen(buffer), 30000); 
}

float32_t * simultSolve(float32_t a, float32_t b, float32_t p, float32_t q, float32_t * sols){

	float64_t x, y;
	
	float64_t c = 0;
	float64_t r = 0;
	
	//if(((a*q-p*b)!=0)&&((b*p-q*a)!=0))
	{//In this case we have a unique solution and display x and y
		//x=(c*q-r*b)/(a*q-p*b);
		//y=(c*p-r*a)/(b*p-q*a);
	}
	//else if(((a*q-p*b)==0)&&((b*p-q*a)==0)&&((c*q-r*b)==0)&&((c*p-r*a)==0)){//In such condition we can have infinitely many solutions to the equation.
	//else {//When we have such a condition than mathematically we can choose any one unknown as free and other unknown can be calculated using the free variables's value.
	//So we choose x as free variable and then get y
	    x = 1;
	    y = (c/b) + ((-1*a/b)*x); 
	//}

	sols[0] = x;
	sols[1] = y;
	
	memset(buffer, 0 ,strlen(buffer));
	sprintf(buffer, "x1: %.4f y1: %.4f\n", x, y);
	HAL_UART_Transmit(&huart1, (uint8_t *)&buffer[0], strlen(buffer), 30000); 
	
	return sols;
}

void whiteEnv(){
	
	arm_matrix_instance_f32 eigDiagMatrix_sqrt;
	float32_t eigDiagMatrix_sqrtData[4];
	arm_mat_init_f32(&eigDiagMatrix_sqrt, 2, 2, eigDiagMatrix_sqrtData);
	
	arm_matrix_instance_f32 eigDiagMatrix_inv;
	float32_t eigDiagMatrix_invData[4];
	arm_mat_init_f32(&eigDiagMatrix_sqrt, 2, 2, eigDiagMatrix_invData);
	
	
	arm_matrix_instance_f32 eigVecMatrix_trans;
	float32_t eigVecMatrix_transData[4];	
	arm_mat_init_f32(&eigDiagMatrix_sqrt, 2, 2, eigVecMatrix_transData);
	
	
	//Calculating sqrt
	for(int i = 0; i < 4; i++){
		arm_sqrt_f32(eigDiagMatrixData[i], &eigDiagMatrix_sqrtData[i]);
	}
	
	arm_mat_inverse_f32(&eigDiagMatrix_sqrt, &eigDiagMatrix_inv);
	arm_mat_trans_f32(&eigVecMatrix, &eigVecMatrix_trans);
	
	arm_mat_mult_f32(&eigDiagMatrix_inv, &eigVecMatrix_trans, &whiteningMatrix);
	arm_mat_mult_f32(&eigVecMatrix, &eigDiagMatrix_sqrt, &dewhiteningMatrix);
	arm_mat_mult_f32(&eigDiagMatrix_inv, &eigVecMatrix_trans, &whiteningMatrix);
	
}

void storeSignals(uint32_t write_address, uint8_t * signal){
	memset(bufferValues, 0, 100);
	int index = 0;
	for(int i=0; i<32000; i++) {
		bufferValues[index++] = signal[i];
		
		if(index == 100) {
				BSP_QSPI_Write((uint8_t *) bufferValues, write_address, 100);
				memset(bufferValues, 0, 100);
				index = 0;
				write_address += 100;
		}
	}
}
void fpica() {
	int numRows = 2;
	int numCols = 32000;

	//A Matrix
	arm_matrix_instance_f32 A;
	float32_t Adata[4];
	arm_mat_init_f32(&A, numRows, numRows, Adata);
	
	//W Matrix
	arm_matrix_instance_f32 W;
	float32_t Wdata[4];
	arm_mat_init_f32(&W, numRows, numRows, Wdata);
	
	//B Matrix
	arm_matrix_instance_f32 B;
	float32_t Bdata[4] = {0.0, 0.0, 0.0, 0.0};
	arm_mat_init_f32(&B, numRows, numRows, Bdata);
	
	//B' Matrix
	arm_matrix_instance_f32 Btranspose;
	float32_t BtransposeData[4];
	arm_mat_init_f32(&Btranspose, numRows, numRows, BtransposeData);
	
	//Do the transpose
	arm_mat_trans_f32(&B, &Btranspose);
	
	
	for(int i=0; i<numRows; i++) {
		//w Matrix
		arm_matrix_instance_f32 w;
		float32_t wData[2] = {RandomNumber(4), RandomNumber(4)};
		arm_mat_init_f32(&w, 2, 1, wData);
		
		//w old
		arm_matrix_instance_f32 wOld;
		float32_t wOldData[2] = {0.0, 0.0};
		arm_mat_init_f32(&w, 2, 1, wOldData);
		
		//B*B'
		arm_matrix_instance_f32 BtimesBtranspose;
		float32_t BtimesBtransposeData[4];
		arm_mat_init_f32(&w, 2, 2, BtimesBtransposeData);
		
		//B*B'*w
		arm_matrix_instance_f32 BtimesBtransposetimesw;
		float32_t BtimesBtransposetimeswData[2];
		arm_mat_init_f32(&w, 2, 1, BtimesBtransposetimeswData);
		
		//Do the multiplication
		arm_mat_mult_f32(&B, &Btranspose, &BtimesBtranspose);
		arm_mat_mult_f32(&BtimesBtranspose, &w, &BtimesBtransposetimesw);
		arm_mat_sub_f32(&w, &BtimesBtransposetimesw, &w);
		
		//Normalize w
		float32_t wNorm = normalize(wData, 2);
		wNorm = 1 / wNorm;
		arm_mat_scale_f32(&w, wNorm, &w);
		
		for(int j=0; j<MAX_NUMBER_OF_ITERATIONS; j++) {
			arm_mat_mult_f32(&B, &Btranspose, &BtimesBtranspose);
			arm_mat_mult_f32(&BtimesBtranspose, &w, &BtimesBtransposetimesw);
			arm_mat_sub_f32(&w, &BtimesBtransposetimesw, &w);
			
			//w-wOld
			arm_matrix_instance_f32 wminuswOld;
			float32_t wminuswOldData[2];
			arm_mat_init_f32(&wminuswOld, 2, 1, wminuswOldData);
			arm_mat_sub_f32(&w, &wOld, &wminuswOld);
			float32_t wminuswOldNorm = normalize(wminuswOldData, 2);
			
			//w+wOld
			arm_matrix_instance_f32 wpluswOld;
			float32_t wpluswOldData[2];
			arm_mat_init_f32(&wpluswOld, 2, 1, wpluswOldData);
			arm_mat_add_f32(&w, &wOld, &wpluswOld);
			float32_t wpluswOldNorm = normalize(wpluswOldData, 2);
			
			if(wminuswOldNorm < epsilon || wpluswOldNorm < epsilon) {
				if(i==0) {
					Bdata[0] = wData[0];
					Bdata[2] = wData[1];
					Adata[0] = (dewhiteningMatrixData[0]*wData[0])+(dewhiteningMatrixData[1]*wData[1]);
					Adata[2] = (dewhiteningMatrixData[2]*wData[0])+(dewhiteningMatrixData[3]*wData[1]);
					Wdata[0] = (wData[0]*whiteningMatrixData[0])+(wData[1]*whiteningMatrixData[2]);
					Wdata[1] = (wData[0]*whiteningMatrixData[1])+(wData[1]*whiteningMatrixData[3]);
				} else if(i==1) {
					Bdata[1] = wData[0];
					Bdata[3] = wData[1];
					Adata[1] = (dewhiteningMatrixData[0]*wData[0])+(dewhiteningMatrixData[1]*wData[1]);
					Adata[3] = (dewhiteningMatrixData[2]*wData[0])+(dewhiteningMatrixData[3]*wData[1]);
					Wdata[2] = (wData[0]*whiteningMatrixData[0])+(wData[1]*whiteningMatrixData[2]);
					Wdata[3] = (wData[0]*whiteningMatrixData[1])+(wData[1]*whiteningMatrixData[3]);
				}
				break;
			}
			
			wOld.pData = w.pData;
			
			float32_t whiteSigBuf1[100];
			float32_t whiteSigBuf2[100];
			float32_t result[100];
			
			uint32_t temp5 = read_address5;
			uint32_t temp6 = read_address6;
			uint32_t temp7 = read_address7;
			
			for(int i=0; i<320; i++) {
				memset(whiteSigBuf1, 0, 400);
				memset(whiteSigBuf2, 0, 400);
				memset(result, 0, 400);
				BSP_QSPI_Read((uint8_t *)whiteSigBuf1, temp5, 400);
				BSP_QSPI_Read((uint8_t *)whiteSigBuf2, temp6, 400);
				temp5+=400;
				temp6+=400;
				
				for(int j=0; j<100; j++) {
					result[i] = (whiteSigBuf1[i]*wData[0])+(whiteSigBuf2[i]*wData[1]);
					result[i] = result[i]*result[i]*result[i]; 
				}
				
				BSP_QSPI_Write((uint8_t *)result, temp7, 400);
				temp7+=400;
			}
			
			temp5 = read_address5;
			temp6 = read_address6;
			temp7 = read_address7;
			float32_t sums[2];
			
			for(int i=0; i<320; i++) {
				memset(whiteSigBuf1, 0, 400);
				memset(whiteSigBuf2, 0, 400);
				memset(result, 0, 400);
				BSP_QSPI_Read((uint8_t *)whiteSigBuf1, temp5, 400);
				BSP_QSPI_Read((uint8_t *)whiteSigBuf2, temp6, 400);
				BSP_QSPI_Read((uint8_t *)result, temp7, 400);
				temp5+=400;
				temp6+=400;
				temp7+=400;
				
				for(int j=0; j<100; j++) {
					sums[0] += whiteSigBuf1[j]*result[j];
					sums[1] += whiteSigBuf2[j]*result[j];
				}
			}
			
			sums[0] = sums[0]/32000;
			sums[1] = sums[1]/32000;
			sums[0] = sums[0] - (3*wData[0]);
			sums[1] = sums[1] - (3*wData[1]);
			float32_t normSums = normalize(sums, 2);
			sums[0] = sums[0]/normSums;
			sums[1] = sums[1]/normSums;
			
			w.pData[0] = sums[0];
			w.pData[1] = sums[1];
		}
	}
}

void addMean(){
	float32_t buff1[100];
	float32_t buff2[100];
	int index = 0;
	int temp1 = write_address8;
	int temp2 = write_address9;

	float32_t newMeans[2];
		
	newMeans[0] = (Wdata[0] * means[0]) + (Wdata[1] * means[1]);
	newMeans[1] = (Wdata[2] * means[0]) + (Wdata[3] * means[1]);
	
	for(int i = 0; i < 32000; i++){
		buff1[index] = (Wdata[0] * x1[i]) + (Wdata[1] * x2[i]) + newMeans[0];
		buff2[index++] = (Wdata[2] * x1[i]) + (Wdata[3] * x2[i]) + newMeans[1];
		if(index == 100){
			BSP_QSPI_Write((uint8_t *) buff1, temp1, 400);
			BSP_QSPI_Write((uint8_t *) buff2, temp2, 400);
			index = 0;
			temp1 += 400;
			temp2 += 400;
			memset(buff1, 0 ,400);
			memset(buff2, 0 ,400);
		}
	}
}

float32_t normalize(float32_t vector[], int length) {
	float32_t wNorm;
	for(int i=0; i<length; i++) {
			wNorm += vector[i]*vector[i];
	}
	arm_sqrt_f32(wNorm, &wNorm);
	return wNorm;
}

float32_t RandomNumber(float Max){
    return (2 * (double)rand() / (double)RAND_MAX - 1)*Max;
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	interrupt_flag = 1;
	if(buttonPressed == 1) {
		timer++;
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/