
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
#include "main.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include <string.h>

#include "stm32l475e_iot01_accelero.h"
#include "stm32l475e_iot01_gyro.h"
#include "stm32l475e_iot01_magneto.h"
#include "stm32l475e_iot01_psensor.h"
#include "stm32l475e_iot01_hsensor.h"
#include "stm32l475e_iot01_tsensor.h"


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
osThreadId blinkThreadId;
osThreadId usartThreadId;
osThreadId accThreadId;
osThreadId gyroThreadId;
osThreadId btnThreadId;
osThreadId magnetoThreadId;
osThreadId pressureThreadId;
osThreadId tempThreadId;
osThreadId humidityThreadId;
osThreadId currentThreadId;

//Global Variables
osMutexId mutex_id;
int timer = 0;
int buttonPressed = 0;
int chooseThread = 0;
int isInitialized = 0;
char buffer[100];

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef handle;
static void MX_GPIO_Init(void);
void UART_init(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void accThread(void const *argument);
void gyroThread(void const *argument);
void magnetoThread(void const *argument);
void pressureThread(void const *argument);
void tempThread(void const *argument);
void humidityThread(void const *argument);
void buttonThread(void const *argument);
void transmitThread(void const *str);
void blinkThread(void const *arg);
int fputc(int ch, FILE *f);
int fgetc(FILE *f);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
  /* USER CODE BEGIN 2 */
	MX_GPIO_Init();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	//Initialize Mutex
	osMutexDef(MutexIsr);
	mutex_id = osMutexCreate(osMutex(MutexIsr));
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */

	//Start Button Control Thread
	osThreadDef(btnTask, buttonThread, osPriorityNormal, 0, 100);
	btnThreadId = osThreadCreate(osThread(btnTask), NULL);
	
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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
void SystemClock_Config(void){

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */
void accThread(void const * argument){

  /* USER CODE BEGIN 5 */
	
	BSP_ACCELERO_Init();
	BSP_ACCELERO_LowPower(1);
	int16_t XYZ[3];
	char xBuff[100];
	char yBuff[20];
	char zBuff[20];
  /* Infinite loop */
  for(;;)
  {
		
		osDelay(200);
		osMutexWait(mutex_id, osWaitForever);
		BSP_ACCELERO_AccGetXYZ(XYZ);
		//printf("X = %d, Y = %d, Z = %d\n", XYZ[0], XYZ[1], XYZ[2]);
		sprintf(xBuff, "Accelerometer X:%d ", XYZ[0]);
		sprintf(yBuff, "Y:%d ", XYZ[1]);
		sprintf(zBuff, "Z:%d \n", XYZ[2]);
		
		memset(buffer, 0, strlen(buffer));
		
		strcat(buffer, xBuff);
		strcat(buffer, yBuff);
		strcat(buffer, zBuff);
		osMutexRelease(mutex_id);
		
  }
  /* USER CODE END 5 */ 
}

void gyroThread(void const * argument){
	
	BSP_GYRO_Init();
	BSP_GYRO_LowPower(1);
	float XYZ[3];
	char xBuff[100];
	char yBuff[20];
	char zBuff[20];
  /* Infinite loop */
  for(;;)
  {
		
		osDelay(200);
		osMutexWait(mutex_id, osWaitForever);
		BSP_GYRO_GetXYZ(XYZ);
		sprintf(xBuff, "Gyroscope X:%.2f ", XYZ[0]);
		sprintf(yBuff, "Y:%.2f ", XYZ[1]);
		sprintf(zBuff, "Z:%.2f \n", XYZ[2]);
		
		memset(buffer, 0, strlen(buffer));
		
		strcat(buffer, xBuff);
		strcat(buffer, yBuff);
		strcat(buffer, zBuff);
		osMutexRelease(mutex_id);
		
  }
	

}

void magnetoThread(void const * argument){
	
	BSP_MAGNETO_Init();
	BSP_MAGNETO_LowPower(1);
	int16_t XYZ[3];
	char xBuff[100];
	char yBuff[20];
	char zBuff[20];
  /* Infinite loop */
  for(;;)
  {
		
		osDelay(200);
		osMutexWait(mutex_id, osWaitForever);
		BSP_MAGNETO_GetXYZ(XYZ);
		sprintf(xBuff, "Magnetometer X:%d ", XYZ[0]);
		sprintf(yBuff, "Y:%d ", XYZ[1]);
		sprintf(zBuff, "Z:%d \n", XYZ[2]);
		
		memset(buffer, 0, strlen(buffer));
		
		strcat(buffer, xBuff);
		strcat(buffer, yBuff);
		strcat(buffer, zBuff);
		osMutexRelease(mutex_id);
  }
}

void pressureThread(void const * argument){
	
	BSP_PSENSOR_Init();
	float pressure;
	char pBuff[20];
  /* Infinite loop */
  for(;;)
  {
		
		osDelay(200);
		osMutexWait(mutex_id, osWaitForever);
		pressure = BSP_PSENSOR_ReadPressure();
		sprintf(pBuff, "Pressure: %.2f \n", pressure);
		
		memset(buffer, 0, strlen(buffer));
		
		strcat(buffer, pBuff);
		osMutexRelease(mutex_id);
  }
}

void tempThread(void const * argument){
	
	BSP_TSENSOR_Init();
	float temp;
	char tBuff[20];
  /* Infinite loop */
  for(;;)
  {
		
		osDelay(200);
		osMutexWait(mutex_id, osWaitForever);
		temp = BSP_TSENSOR_ReadTemp();
		sprintf(tBuff, "Temperature: %.2f \n", temp);
		
		memset(buffer, 0, strlen(buffer));
		
		strcat(buffer, tBuff);
		osMutexRelease(mutex_id);
  }
}

void humidityThread(void const * argument){
	
	BSP_HSENSOR_Init();
	float humidity;
	char hBuff[20];
  /* Infinite loop */
  for(;;)
  {
		
		osDelay(200);
		osMutexWait(mutex_id, osWaitForever);
		humidity = BSP_HSENSOR_ReadHumidity();
		sprintf(hBuff, "Humidity: %.2f \n", humidity);
		
		memset(buffer, 0, strlen(buffer));
		
		strcat(buffer, hBuff);
		osMutexRelease(mutex_id);
  }
}



void buttonThread(void const * argument){
	
	//Define All Threads
	osThreadDef(magnetoTask, magnetoThread, osPriorityNormal, 0, 128);
	osThreadDef(pressureTask, pressureThread, osPriorityNormal, 0, 128);
	osThreadDef(tempTask, tempThread, osPriorityNormal, 0, 128);
	osThreadDef(accTask, accThread, osPriorityNormal, 0, 128);
	osThreadDef(humidityTask, humidityThread, osPriorityNormal, 0, 128);
	osThreadDef(gyroTask, gyroThread, osPriorityNormal, 0, 128);
	osThreadDef(usartTransmit, transmitThread, osPriorityNormal, 0, 100);
	//osThreadDef(blink, blinkThread, osPriorityNormal, 0, 128);
	
	while(1){
		if(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)){
			buttonPressed = 1;
			while(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)){}
			buttonPressed = 0;
			if(timer >= 3000){ 
				
				//Power Down Everything
				BSP_GYRO_DeInit();
				BSP_MAGNETO_DeInit();
				BSP_ACCELERO_DeInit();
				osThreadTerminate(currentThreadId);
				osThreadTerminate(usartThreadId);
				//osThreadTerminate(blinkThreadId);
				isInitialized = 0;
				chooseThread = 0;
				
			} else {
				if(chooseThread == 0) {
					if(isInitialized == 1) {
						BSP_GYRO_DeInit();
						osThreadTerminate(gyroThreadId);
					} else {
						isInitialized = 1;
						usartThreadId = osThreadCreate(osThread(usartTransmit), NULL);
						//blinkThreadId = osThreadCreate(osThread(blink), NULL);
					}		
					//Magnetometer
					magnetoThreadId = osThreadCreate(osThread(magnetoTask), NULL);
					currentThreadId = magnetoThreadId;
					chooseThread = 1;					
				} else if(chooseThread == 1) {
					//Pressure Sensor
					BSP_MAGNETO_DeInit();
					osThreadTerminate(magnetoThreadId);
					pressureThreadId = osThreadCreate(osThread(pressureTask), NULL);
					currentThreadId = pressureThreadId;
					chooseThread = 2;
				} else if(chooseThread == 2) {
					//Temperature Sensor
					osThreadTerminate(pressureThreadId);
					tempThreadId = osThreadCreate(osThread(tempTask), NULL);
					currentThreadId = tempThreadId;
					chooseThread = 3;
				} else if(chooseThread == 3) {
					//Accelerometer Sensor
					osThreadTerminate(tempThreadId);
					accThreadId = osThreadCreate(osThread(accTask), NULL);
					currentThreadId = accThreadId;
					chooseThread = 4;
				} else if(chooseThread == 4) {
					//Humidity Sensor
					BSP_ACCELERO_DeInit();
					osThreadTerminate(accThreadId);
					humidityThreadId = osThreadCreate(osThread(humidityTask), NULL);
					currentThreadId = humidityThreadId;
					chooseThread = 5;
				} else if(chooseThread == 5) {
					//Gyroscope Sensor
					osThreadTerminate(humidityThreadId);
					gyroThreadId = osThreadCreate(osThread(gyroTask), NULL);
          currentThreadId = gyroThreadId;
					chooseThread = 0;
				}
			}
			timer = 0;
		}
	}
}

void transmitThread(void const *str){
	UART_init();
	while(1){
		osMutexWait(mutex_id, osWaitForever);
		HAL_UART_Transmit(&handle, (uint8_t *) &buffer[0], strlen(buffer), 30000);
		//HAL_UART_Transmit(&handle, (uint8_t *) "\n", 1, 30000);
		osMutexRelease(mutex_id);
		osDelay(200);
	}
}
void blinkThread(void const *arg){

	while(1){
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		osDelay(1000);
	}
	
}

void UART_init() {	
	GPIO_InitTypeDef gpio_init;
	
	handle.Instance = USART1;
	handle.Init.BaudRate = 115200;
  handle.Init.WordLength = UART_WORDLENGTH_8B;
  handle.Init.StopBits = UART_STOPBITS_1;
  handle.Init.Parity = UART_PARITY_NONE;
  handle.Init.Mode = UART_MODE_TX_RX;
  handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  handle.Init.OverSampling = UART_OVERSAMPLING_16;
  handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();
	
  gpio_init.Pin = GPIO_PIN_6;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOB, &gpio_init);
	
	gpio_init.Pin = GPIO_PIN_7;
	HAL_GPIO_Init(GPIOB, &gpio_init);
	
	HAL_UART_Init(&handle);
}

int fputc(int ch, FILE *f) {
  while (HAL_OK != HAL_UART_Transmit(&handle, (uint8_t *) &ch, 1, 30000));
  return ch;
}

int fgetc(FILE *f) {
  uint8_t ch = 0;
  while (HAL_OK != HAL_UART_Receive(&handle, (uint8_t *)&ch, 1, 30000));
  return ch;
}



/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  /* USER CODE BEGIN Callback 0 */
	//HAL_UART_Transmit(&handle, (uint8_t *)"timer\n",6, 3000 );
	if(buttonPressed == 1){
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
void _Error_Handler(char *file, int line){
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
