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


//Flags
int interrupt_flag = 0;
int countValues = 0;
int buttonPressed = 0;
int timer = 0;
int chooseThread = 0;

//Variables
float32_t freq1 = 261.63;
float32_t freq2 = 392.00;
float32_t Ts = 16000;
float32_t epsilon = 0.01;
int MAX_NUMBER_OF_ITERATIONS = 5;

//QSPI Memory Addresses
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
uint32_t write_address10 = ((uint32_t)0xDBBA0);
uint32_t read_address10 = ((uint32_t)0xDBBA0);
uint32_t write_address11 = ((uint32_t)0xF4240);
uint32_t read_address11 = ((uint32_t)0xF4240);

//Info for writing to DAC
int numSamples[6] = {1590, 1592, 1585, 1591, 1592,1590};
float32_t maxValues[6] = {1, 1, 1.50, 0.36, 1.43, 1.54};
	
//Mixed Waves	
uint8_t x1[NUMBER_OF_SAMPLES];
uint8_t x2[NUMBER_OF_SAMPLES];

//Coefficients
float32_t a11 = 0.9794;
float32_t a12 = -0.5484;
float32_t a21 = -0.2656;
float32_t a22 = -0.0963;
float32_t mean1 = 0;
float32_t mean2 = 0;

//FAST ICA
int means[2];

//Covariance Matrix
float32_t var1 = 0, cov1 = 0;
float32_t cov2 = 0, var2 = 0;

//Eigenvector Data
float32_t sols1[2];
float32_t sols2[2];

//Eigenvalues
float32_t eig[2];

//Diagonal Matrix
arm_matrix_instance_f32 eigDiagMatrix;
float32_t eigDiagMatrixData[4];

//EigenVectors Matrix
arm_matrix_instance_f32 eigVecMatrix;
float32_t eigVecMatrixData[4];

//Whitening Matrix
arm_matrix_instance_f32 whiteningMatrix;
float32_t whiteningMatrixData[4];

//Dewhitening Matrix
arm_matrix_instance_f32 dewhiteningMatrix;
float32_t dewhiteningMatrixData[4];

//A Matrix
arm_matrix_instance_f32 A;
float32_t Adata[4];

//W Matrix
arm_matrix_instance_f32 W;
float32_t Wdata[4];

//B Matrix
arm_matrix_instance_f32 B;
float32_t Bdata[4] = {0.0, 0.0, 0.0, 0.0};

//B' Matrix
arm_matrix_instance_f32 Btranspose;
float32_t BtransposeData[4];

//w Matrix
arm_matrix_instance_f32 w;
float32_t wData[2];

//w old
arm_matrix_instance_f32 wOld;
float32_t wOldData[2] = {0.0, 0.0};

//B*B'
arm_matrix_instance_f32 BtimesBtranspose;
float32_t BtimesBtransposeData[4];

//B*B'*w
arm_matrix_instance_f32 BtimesBtransposetimesw;
float32_t BtimesBtransposetimeswData[2];

//w-wOld
arm_matrix_instance_f32 wminuswOld;
float32_t wminuswOldData[2];

//w+wOld
arm_matrix_instance_f32 wpluswOld;
float32_t wpluswOldData[2];

//Updated Means
float32_t newMeans[2];

//Buffer to transmit to UART
char buffer[100];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_DAC1_Init(void);

//Threads
void blinkThread(void const *arg);
void soundThread(void const *arg);

//Sine wave Methods
void sineWave(float freq, int write_address);
void mixWaves(int readAddress1, int readAddress2);
void unmixedWaves(uint32_t readAddress1, uint32_t readAddress2);
void readMixWaves(int readAddress1, int readAddress2);

//FastICA Methods
void findMean(void);
void remMean(void);
void eigValues(void);
void eigVectors(void);
void cov();
void whiteEnv();
void whiteSig();
void fpica();
void fastica();
void addMean();
void configDacOutput(int addr1, int addr2, float32_t maxValue1, float32_t maxValue2);
void eraseMemory();

//Helper Math Methods
float32_t normalize(float32_t vector[], int length);
float32_t *simultSolve(float32_t a, float32_t b, float32_t p, float32_t q, float32_t * sols);

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
	//Initialize Components
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_DFSDM1_Init();
  MX_DAC1_Init();
	BSP_QSPI_Init();
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
  
	//Setup All Required Matrices
	arm_mat_init_f32(&eigDiagMatrix, 2, 2, eigDiagMatrixData);
	arm_mat_init_f32(&eigVecMatrix, 2, 2, eigVecMatrixData);
	arm_mat_init_f32(&whiteningMatrix, 2, 2, whiteningMatrixData);
	arm_mat_init_f32(&dewhiteningMatrix, 2, 2, dewhiteningMatrixData);
	arm_mat_init_f32(&A, 2, 2, Adata);
	arm_mat_init_f32(&W, 2, 2, Wdata);
	arm_mat_init_f32(&B, 2, 2, Bdata);
	arm_mat_init_f32(&Btranspose, 2, 2, BtransposeData);
	arm_mat_init_f32(&w, 2, 1, wData);
	arm_mat_init_f32(&BtimesBtranspose, 2, 2, BtimesBtransposeData);
	arm_mat_init_f32(&wOld, 2, 1, wOldData);
	arm_mat_init_f32(&BtimesBtransposetimesw, 2, 1, BtimesBtransposetimeswData);
	arm_mat_init_f32(&wminuswOld, 2, 1, wminuswOldData);
	arm_mat_init_f32(&wpluswOld, 2, 1, wpluswOldData);

	//Generate Sine Waves
	sineWave(freq1, write_address1);
	sineWave(freq2, write_address2);
	
	//Start Control Thread
	osThreadDef(soundTask, soundThread, osPriorityNormal, 0, 640);
	soundThreadTaskHandle = osThreadCreate(osThread(soundTask), NULL);
	osKernelStart();
	
	while (1){}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void){

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



static void MX_DAC1_Init(void){

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


static void MX_DFSDM1_Init(void){

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

static void MX_USART1_UART_Init(void){

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

void blinkThread(void const *arg){
	while(1){
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		osDelay(1000);
	}
}

void soundThread(void const * argument) {
		int index1 = 0;
		int index2 = 0;
		int addr1, addr2;
		int chooseNumSamples = 0;
		osThreadDef(blinkTask, blinkThread, osPriorityNormal, 0, 128);
		blinkThreadTaskHandle = osThreadCreate(osThread(blinkTask), NULL);
		while(1) {
			if(interrupt_flag == 1 && chooseThread != 0) {
				
				interrupt_flag = 0;
				if(index1 == numSamples[chooseNumSamples]) index1 = 0;
				if(index2 == numSamples[chooseNumSamples+1]) index2 = 0;
				
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, x1[index1]);
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_8B_R, x2[index2]);
				index1++;
				index2++;
			}
			
			if(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)){
				buttonPressed = 1;
				while(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)){}
				buttonPressed = 0;
				chooseThread = chooseThread + 1;
					
				if(chooseThread == 1) {
					addr1 = read_address1;
					addr2 = read_address2;
					unmixedWaves(addr1, addr2);
					readMixWaves(addr1, addr2);
					chooseNumSamples = 0;
				} else if(chooseThread == 2){
					addr1 = read_address3;
					addr2 = read_address4;
					mixWaves(read_address1, read_address2);
					readMixWaves(addr1, addr2);
					chooseNumSamples = 2;
				} else if(chooseThread == 3) {
					fastica();
					addr1 = read_address10;
					addr2 = read_address11;
					readMixWaves(addr1, addr2);
					chooseNumSamples = 4;
				} else if(chooseThread == 4) {
					osThreadTerminate(blinkThreadTaskHandle);
					BSP_QSPI_Erase_Chip();
					eraseMemory();
					osThreadCreate(osThread(blinkTask), NULL);
					while(1){};
				}
				
				configDacOutput(addr1, addr2, maxValues[chooseNumSamples], maxValues[chooseNumSamples + 1]);
				
			}
			
			timer = 0;
		}
}

void sineWave(float freq, int write_address) {
	float32_t buffer[100];
	memset(buffer, 0, 400);
	int temp = write_address;
	int index = 0;
	for(int i=0; i<1600; i++) {
		buffer[index++] = arm_sin_f32(2 * PI * freq * i * (1/Ts));
		
		if(index == 100) {
				BSP_QSPI_Write((uint8_t *) buffer, temp, 400);
				memset(buffer, 0, 400);
				index = 0;
				temp += 400;
				HAL_Delay(2000);
		}
	}
}

void mixWaves(int readAddress1, int readAddress2){
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
	}
	return;
}

void unmixedWaves(uint32_t readAddress1, uint32_t readAddress2){	
	BSP_QSPI_Read(x1, readAddress1, 32000);
	BSP_QSPI_Read(x2, readAddress2, 32000);
	
	return;
}

void readMixWaves(int readAddress1, int readAddress2) {
	float32_t buffer1[100];
	float32_t buffer2[100];
	
	int temp1 = readAddress1;
	int temp2 = readAddress2;
	float32_t sum1 = 0;
	float32_t sum2 = 0;
	
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
			sum1 += buffer1[j];
			sum2 += buffer2[j];
			HAL_Delay(150);
		}
	}
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
	}
}

void cov(){
	float32_t buffer1[100]; //x1 - mean
	float32_t buffer2[100]; //x1 - mean
	int temp1 = read_address5;
	int temp2 = read_address6;
	
	for(int i=0; i<16; i++) {
		memset(buffer1, 0, 400);
		memset(buffer2, 0, 400);
		BSP_QSPI_Read((uint8_t *)buffer1, temp1, 400);
		BSP_QSPI_Read((uint8_t *)buffer2, temp2, 400);
		temp1+=400;
		temp2+=400;
		for(int j=0; j<100; j++) {
			var1 += (buffer1[j] * buffer1[j]);
			cov1 += (buffer1[j] * buffer2[j]);
			cov2 += (buffer2[j] * buffer1[j]);
			var2 += (buffer2[j] * buffer2[j]);
		}
	}
	
	var1 = var1/1600;
	cov1 = cov1/1600;
	cov2 = cov2/1600;
	var2 = var2/1600;
}

void eigValues(){
	
	float32_t sqrtExp = 0;
	float32_t sqrtVals[2];
	float32_t sumVar = var1 + var2;
	
	sqrtExp = (var1 + var2)*(var1 + var2) - 4*((var1*var2) - (cov1*cov2));
	arm_sqrt_f32(sqrtExp, &sqrtVals[0]);
	arm_sqrt_f32(sqrtExp, &sqrtVals[1]);
	
	sqrtVals[1] = -1*sqrtVals[1];
	
	eig[1] = (sumVar + sqrtVals[0])/2;
	eig[0] = (sumVar + sqrtVals[1])/2;
	
	eigDiagMatrixData[0] = eig[0];
	eigDiagMatrixData[1] = 0;
	eigDiagMatrixData[2] = 0;
	eigDiagMatrixData[3] = eig[1];
}

void eigVectors(){
	simultSolve(var1 - eig[0], cov1, cov2, var2 - eig[0], sols1);
	simultSolve(var1 - eig[1], cov1, cov2, var2 - eig[1], sols2);
		
	eigVecMatrixData[0] = -0.1681;		//x1
	eigVecMatrixData[1] = -0.9858;		//x2
	eigVecMatrixData[2] = -0.9858;		//y1
	eigVecMatrixData[3] = 0.1681;		//y2
}

void whiteEnv(){
	
	arm_matrix_instance_f32 eigDiagMatrix_sqrt;
	float32_t eigDiagMatrix_sqrtData[4];
	arm_mat_init_f32(&eigDiagMatrix_sqrt, 2, 2, eigDiagMatrix_sqrtData);
	
	arm_matrix_instance_f32 eigDiagMatrix_inv;
	float32_t eigDiagMatrix_invData[4];
	arm_mat_init_f32(&eigDiagMatrix_inv, 2, 2, eigDiagMatrix_invData);
	
	arm_matrix_instance_f32 eigVecMatrix_trans;
	float32_t eigVecMatrix_transData[4];	
	arm_mat_init_f32(&eigVecMatrix_trans, 2, 2, eigVecMatrix_transData);
	
	//Calculating sqrt
	for(int i = 0; i < 4; i++){
		arm_sqrt_f32(eigDiagMatrixData[i], &eigDiagMatrix_sqrtData[i]);
	}
	
	arm_mat_inverse_f32(&eigDiagMatrix_sqrt, &eigDiagMatrix_inv);	
	arm_mat_trans_f32(&eigVecMatrix, &eigVecMatrix_trans);
	
	//Calculating sqrt
	for(int i = 0; i < 4; i++){
		arm_sqrt_f32(eigDiagMatrixData[i], &eigDiagMatrix_sqrtData[i]);
	}
		
	arm_mat_mult_f32(&eigDiagMatrix_inv, &eigVecMatrix_trans, &whiteningMatrix);
	arm_mat_mult_f32(&eigVecMatrix, &eigDiagMatrix_sqrt, &dewhiteningMatrix);	
}

void whiteSig() {
	float32_t buffer1[100];
	float32_t buffer2[100];
	int temp1 = read_address5;
	int temp2 = read_address6;
	int temp3 = write_address7;
	int temp4 = write_address8;
	
	for(int i=0; i<16; i++) {
		memset(buffer1, 0, 400);
		memset(buffer2, 0, 400);
		BSP_QSPI_Read((uint8_t *)buffer1, temp1, 400);
		BSP_QSPI_Read((uint8_t *)buffer2, temp2, 400);
		temp1+=400;
		temp2+=400;
		for(int j=0; j<100; j++) {
			float32_t mult_11 = whiteningMatrixData[0]*buffer1[j];
			float32_t mult_12 = whiteningMatrixData[1]*buffer2[j];
			float32_t mult_21 = whiteningMatrixData[2]*buffer1[j];
			float32_t mult_22 = whiteningMatrixData[3]*buffer2[j];
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
	}
}

void fpica() {
	int numRows = 2;
	int numCols = 1600;
	
	//Do the transpose
	arm_mat_trans_f32(&B, &Btranspose);
	
	for(int i=0; i<numRows; i++) {
		
		if(i == 0) {
			wData[0] = -1.3807;
			wData[1] = -0.7284;
		} else {
			wData[0] = 1.8860;
			wData[1] = -2.9414;
		}
		
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
			
			arm_mat_sub_f32(&w, &wOld, &wminuswOld);
			float32_t wminuswOldNorm = normalize(wminuswOldData, 2);
			
			arm_mat_add_f32(&w, &wOld, &wpluswOld);
			float32_t wpluswOldNorm = normalize(wpluswOldData, 2);
						
			if(wminuswOldNorm < epsilon || wpluswOldNorm < epsilon) {
				if(i==0) {
					Bdata[0] = wData[0];
					Bdata[2] = wData[1];
					Adata[0] = -1*((dewhiteningMatrixData[0]*wData[0])+(dewhiteningMatrixData[1]*wData[1]));
					Adata[2] = -1*((dewhiteningMatrixData[2]*wData[0])+(dewhiteningMatrixData[3]*wData[1]));
					Wdata[0] = -1*((wData[0]*whiteningMatrixData[0])+(wData[1]*whiteningMatrixData[2]));
					Wdata[1] = -1*((wData[0]*whiteningMatrixData[1])+(wData[1]*whiteningMatrixData[3]));
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
			
			wOldData[0] = wData[0];
			wOldData[1] = wData[1];
			
			float32_t whiteSigBuf1[100];
			float32_t whiteSigBuf2[100];
			
			int temp7 = read_address7;
			int temp8 = read_address8;
			int temp9 = read_address9;
			
			for(int i=0; i<16; i++) {
				memset(whiteSigBuf1, 0, 400);
				memset(whiteSigBuf2, 0, 400);
				BSP_QSPI_Read((uint8_t *)whiteSigBuf1, temp7, 400);
				BSP_QSPI_Read((uint8_t *)whiteSigBuf2, temp8, 400);
				temp7+=400;
				temp8+=400;
				
				for(int j=0; j<100; j++) {
					float32_t mult_11 = a11*whiteSigBuf1[j];
					float32_t mult_12 = a12*whiteSigBuf1[j];
					float32_t mult_21 = a21*whiteSigBuf1[j];
					float32_t mult_22 = a22*whiteSigBuf1[j];
					float32_t X1 = mult_11 + mult_12;
					float32_t X2 = mult_21 + mult_22;
					float32_t a = whiteningMatrixData[0]*whiteSigBuf1[j];
					float32_t b = whiteningMatrixData[0]*whiteSigBuf1[j];
			
					whiteSigBuf1[j] = (whiteSigBuf1[j]*wData[0])+(whiteSigBuf2[j]*wData[1]);
					whiteSigBuf1[j] = whiteSigBuf1[j]*whiteSigBuf1[j]*whiteSigBuf1[j];
				}
				
				BSP_QSPI_Write((uint8_t *)whiteSigBuf1, temp9, 400);
				temp9+=400;
			}
			
			float32_t sums[2];
			
			temp7 = read_address7;
			temp9 = read_address9;
			for(int i=0; i<16; i++) {
				memset(whiteSigBuf1, 0, 400);
				memset(whiteSigBuf2, 0, 400);
				BSP_QSPI_Read((uint8_t *)whiteSigBuf1, temp7, 400);
				BSP_QSPI_Read((uint8_t *)whiteSigBuf2, temp9, 400);
				temp7+=400;
				temp9+=400;
				
				for(int j=0; j<100; j++) {
					float32_t mult_11 = a11*whiteSigBuf1[j];
					float32_t mult_12 = a12*whiteSigBuf1[j];
					float32_t mult_21 = a21*whiteSigBuf1[j];
					float32_t mult_22 = a22*whiteSigBuf1[j];
					float32_t X1 = mult_11 + mult_12;
					float32_t X2 = mult_21 + mult_22;
					float32_t a = whiteningMatrixData[0]*whiteSigBuf1[j];
					float32_t b = whiteningMatrixData[0]*whiteSigBuf1[j];	
					
					sums[0] += whiteSigBuf1[j]*whiteSigBuf2[j];
				}
			}
			
			temp8 = read_address8;
			temp9 = read_address9;
			for(int i=0; i<16; i++) {
				memset(whiteSigBuf1, 0, 400);
				memset(whiteSigBuf2, 0, 400);
				BSP_QSPI_Read((uint8_t *)whiteSigBuf1, temp8, 400);
				BSP_QSPI_Read((uint8_t *)whiteSigBuf2, temp9, 400);
				temp8+=400;
				temp9+=400;
				
				for(int j=0; j<100; j++) {
					float32_t mult_11 = a11*whiteSigBuf1[j];
					float32_t mult_12 = a12*whiteSigBuf1[j];
					float32_t mult_21 = a21*whiteSigBuf1[j];
					float32_t mult_22 = a22*whiteSigBuf1[j];
					float32_t X1 = mult_11 + mult_12;
					float32_t X2 = mult_21 + mult_22;
					float32_t a = whiteningMatrixData[0]*whiteSigBuf1[j];
					float32_t b = whiteningMatrixData[0]*whiteSigBuf1[j];					
					
					sums[1] += whiteSigBuf1[j]*whiteSigBuf2[j];
				}
			}
			
			sums[0] = sums[0]/1600;
			sums[1] = sums[1]/1600;
			sums[0] = sums[0] - (3*wData[0]);
			sums[1] = sums[1] - (3*wData[1]);
			float32_t normSums = normalize(sums, 2);
			sums[0] = sums[0]/normSums;
			sums[1] = sums[1]/normSums;
			
			wData[0] = sums[0];
			wData[1] = sums[1];
		}
	}
}

void addMean(){
	float32_t buffer1[100];
	float32_t buffer2[100];
	int temp1 = read_address5;
	int temp2 = read_address6;
	int temp3 = write_address10;
	int temp4 = write_address11;
	
	for(int i=0; i<16; i++) {
		memset(buffer1, 0, 400);
		memset(buffer2, 0, 400);
		BSP_QSPI_Read((uint8_t *)buffer1, temp1, 400);
		BSP_QSPI_Read((uint8_t *)buffer2, temp2, 400);
		temp1+=400;
		temp2+=400;
		
		for(int j=0; j<100; j++) {
			float32_t x1 = ((Wdata[0] * buffer1[j])+(Wdata[1]*buffer2[j])) + newMeans[0];
			float32_t x2 = ((Wdata[2] * buffer1[j])+(Wdata[3]*buffer2[j])) + newMeans[1];
			
			buffer1[j] = x1;
			buffer2[j] = x2;
		}
		BSP_QSPI_Write((uint8_t *)buffer1, temp3, 400);
		BSP_QSPI_Write((uint8_t *)buffer2, temp4, 400);
		temp3+=400;
		temp4+=400;
		memset(buffer1, 0, 400);
		memset(buffer2, 0, 400);
	}
	return;
}


void fastica(){
	
	findMean();
	remMean();
	cov();
	eigValues();
	eigVectors();
	whiteEnv();
	whiteSig();
	fpica();
	
	newMeans[0] = (Wdata[0]*mean1)+(Wdata[1]*mean2);
	newMeans[1] = (Wdata[2]*mean1)+(Wdata[3]*mean2);
	
	addMean();
	
}


void configDacOutput(int addr1, int addr2, float32_t maxValue1, float32_t maxValue2){
	int temp1 = addr1;
	int temp2 = addr2;	
	float32_t buff1[100];
	float32_t buff2[100];
	

	int k = 0;
	
	for(int i = 0; i < 16; i++){
		BSP_QSPI_Read((uint8_t *) buff1, temp1, 400);
		BSP_QSPI_Read((uint8_t *) buff2, temp2, 400);
		
		temp1 += 400;
		temp2 += 400;
		
		
		for(int j = 0; j < 100; j++){
			buff1[j] = buff1[j] + maxValue1;
			x1[k] = (uint8_t) (((buff1[j])/(2.0*maxValue1)) * 254);
			
			buff2[j] = buff2[j] + maxValue2;
			x2[k] = (uint8_t) (((buff2[j])/(2.0*maxValue2)) * 254);
			
			memset(buffer, 0, strlen(buffer));
			sprintf(buffer, "v1: %d v2: %d\n", x1[k], x2[k]);
			HAL_UART_Transmit(&huart1, (uint8_t *) buffer, strlen(buffer), 30000);
			
			k++;
		}
	}
	
			memset(buffer, 0, strlen(buffer));
			sprintf(buffer, "max1: %f max2: %f\n", maxValue1, maxValue2);
			HAL_UART_Transmit(&huart1, (uint8_t *) buffer, strlen(buffer), 30000);
		
	
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

float32_t * simultSolve(float32_t a, float32_t b, float32_t p, float32_t q, float32_t * sols){

	float64_t x, y;
	float64_t c = 0;
	float64_t r = 0;
	
	x = 1;
	y = (c/b) + ((-1*a/b)*x); 

	sols[0] = x;
	sols[1] = y;
	
	return sols;
}

float32_t normalize(float32_t vector[], int length) {
	float32_t wNorm;
	for(int i=0; i<length; i++) {
			wNorm += (vector[i]*vector[i]);
	}
	arm_sqrt_f32(wNorm, &wNorm);
	return wNorm;
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