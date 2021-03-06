#include "main.h"
#include "stm32l4xx_hal.h"
#include <stdlib.h>
#include <string.h>

ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void ADC_Init(void);
int UART_Print_String(UART_HandleTypeDef *huart1, char *string, int stringSize);
int flag;

int main(void)
{
	char input[1];
	char output[1] = {'X'};
	
	char string[18]= {'T','e','m','p','e','r','a','t','u','r','e',' ','=',' ','0','0','C','\n'};
	uint32_t tens_digit;
	uint32_t ones_digit;

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
	SystemClock_Config();
		
  /* Configure the system clock */
  /* Initialize all configured peripherals */
	ADC_Init();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
	
	uint32_t adcMeasurement = 0;

  /* Infinite loop */
  while (1)
  {
		/**
		HAL_UART_Receive(&huart1,(uint8_t *)&input[0], 1, 300);
		if(input[0] == 'Y') {
			UART_Print_String(&huart1, &output[0], 1);
		**/
		
		
		if(flag == 1) {
				HAL_ADC_Start(&hadc1);
				flag = 0;
				if(HAL_ADC_PollForConversion(&hadc1, 1000000) == HAL_OK) {
					
				adcMeasurement = (uint32_t)HAL_ADC_GetValue(&hadc1);
				adcMeasurement = __HAL_ADC_CALC_TEMPERATURE(3400,adcMeasurement, ADC_RESOLUTION_10B);
					
				//Extract the tens and ones digit from the celsius value of temperature
				tens_digit = adcMeasurement/10;
				ones_digit = adcMeasurement - (tens_digit * 10);
					
				//Cast both to chars and add to the output string
				string[14] = tens_digit + '0';
				string[15] = ones_digit + '0';
				UART_Print_String(&huart1, &string[0], 18);
			}
		}
		
  }
}

//This method sends a string over the UART connection serially
int UART_Print_String(UART_HandleTypeDef *huart1, char *string, int stringSize) {
	return HAL_UART_Transmit(huart1, (uint8_t *)string, stringSize, 30000);
}

static void ADC_Init(void) {
	__HAL_RCC_ADC_CLK_ENABLE();
	hadc1.Instance = ADC1;
	
	//Set properties for ADC
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	//Resolution set to 10 bits as required by the assignment
	hadc1.Init.Resolution = ADC_RESOLUTION_10B;
	//LSB is the right most bit
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	//Set ADC to be triggered by software only
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	
	//Initialize the ADC
	HAL_ADC_Init(&hadc1);
	
	//Struct declaration for ADC channel
	ADC_ChannelConfTypeDef adcChannel;
	
	//Set input channel of ADC to the temperature sensor
	adcChannel.Channel = ADC_CHANNEL_TEMPSENSOR;
	adcChannel.Rank = ADC_REGULAR_RANK_1;
	//Set sampling time for getting values from ADC
	adcChannel.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
	adcChannel.Offset = 0;
	
	//Initialize the channel
	if(HAL_ADC_ConfigChannel(&hadc1, &adcChannel) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
}

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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 32;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
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
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/10);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
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

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
}


void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ 
}
#endif /* USE_FULL_ASSERT */
