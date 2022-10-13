/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "lcd_n1202.h"
#include "nrf24.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* BUTTONS GPIO */
#define BTN01	GPIO_PIN_6
#define BTN02	GPIO_PIN_5
#define BTN03	GPIO_PIN_4
#define BTN04	GPIO_PIN_12
#define BTN05	GPIO_PIN_11
#define BTN06	GPIO_PIN_10
#define BTN		GPIOA

/* DATA LENGTH */
#define DLEN	3	//Data Length for TX and RX

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* RX and TX Variables */
uint8_t RFmode;
uint8_t RXaddr[] = {0xEE,0xDD,0xCC,0xBB,0xAA};	//RX Address
uint8_t RXdat[4];	//RX Data
uint8_t TXaddr[] = {0xEE,0xDD,0xCC,0xBB,0xAA};	//TX Address
uint8_t TXdat[4];	//TX Data

/* Button Variables */
uint16_t BTNseq[6] = {BTN01,BTN02,BTN03,BTN04,BTN05,BTN06}; //Buttons bit sequence/position
uint8_t btns; //for indexing Buttons

/* ADC Variable */
uint16_t adcv; //for containing ADC value

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* Check Button Function */
uint8_t checkbtn(uint16_t cbtn); //Check if any button is pressed

/* Get ADC Value Function */
uint16_t getADCvalue();

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
  MX_ADC_Init();
  MX_SPI1_Init();
  lcdn1202_init();
  NRF24_init();
  /* USER CODE BEGIN 2 */
  if(HAL_GPIO_ReadPin(BTN, BTN01)==GPIO_PIN_SET) //If Button 1 is pressed -> TX Mode
  {
	  RFmode = 1;
	  LCD_drawtext("TX MODE", 1, 16);
	  NRF24_TXmode(10, TXaddr);	//Use Channel 10
  }
  else	//If Button 1 is not pressed -> RX Mode
  {
	  RFmode = 0;
	  LCD_drawtext("RX MODE", 1, 16);
	  NRF24_RXmode(10, 1, RXaddr, DLEN); //Use Channel 10 and Pipe 1. Payload size = data length
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if(RFmode==1)
	  {
		  	  TXdat[0] = 0;
		  	  for(btns=0;btns<6;btns++) //Check all button
		  	  {
	  	  			  if(checkbtn(BTNseq[btns])==1) //Check if a button is pressed
	  	  			  {
	  	  				  TXdat[0] = TXdat[0] | 1<<btns; //Update buttons data
	  	  			  }
	  	  			  else{}
		  	  }

		  	  adcv = getADCvalue();		//Get ADC Value
		  	  TXdat[1] = adcv >> 8;		//ADC Upper Byte
		  	  TXdat[2] = adcv & 0x00FF;	//ADC Lower Byte

		  	  if(NRF24_TX(TXdat,DLEN)==1) //Send all payload data
	  	  	  {
	  	  			  LCD_drawtext("Transmit...", 3, 0);
	  	  			  LCD_drawint(TXdat[0], 5, 0);	//Display Buttons State Data
	  	  			  LCD_drawint(TXdat[1], 5, 32);	//Display ADC Upper Byte
	  	  			  LCD_drawint(TXdat[2], 5, 64);	//Display ADC Lower Byte
	  	  	  }
	  	  	  else{}

		  	  HAL_Delay(200);
		  	  LCD_clearblock(3,0,95);
		  	  LCD_clearblock(5,0,95);
	  }
	  else
	  {
	  	  	  if(NRF24_checkpipe(1)==1)	//Check if any data in Pipe 1
	  	  	  {
	  	  	  		  NRF24_RX(RXdat,DLEN);	//Receive data
	  	  	  		  LCD_drawint(RXdat[0], 3, 0);	//Display Buttons State Data
	  	  	  		  LCD_drawint(RXdat[1], 3, 32);	//Display ADC Upper Byte
	  	  	  		  LCD_drawint(RXdat[2], 3, 64);	//Display ADC Lower Byte
	  	  	  		  adcv = (RXdat[1] << 8) | RXdat[2];	//Get ADC Value

	  	  	  		  for(btns=0;btns<6;btns++)
	  	  	  		  {
	  	  	  			  if((RXdat[0])&0x01) //Check buttons state bit
	  	  	  			  {
	  	  	  				LCD_drawint((btns+1), 4, (btns*12)+12);	//Display pressed buttons
	  	  	  			  }
	  	  	  			  RXdat[0] = RXdat[0]>>1;
	  	  	  		  }

	  	  	  		LCD_drawtext("ADC : ", 6, 0);
	  	  	  		LCD_drawint(adcv, 6, 48);
	  	  	  }
	  	  	  else{}

	  	  	  HAL_Delay(200);
	  	  	  LCD_clearblock(3,0,95);
	  	  	  LCD_clearblock(4,12,83);
	  	  	  LCD_clearblock(6,0,95);
	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : BTN03_Pin BTN02_Pin BTN01_Pin BTN06_Pin
                           BTN05_Pin BTN04_Pin */
  GPIO_InitStruct.Pin = BTN03_Pin|BTN02_Pin|BTN01_Pin|BTN06_Pin
                          |BTN05_Pin|BTN04_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint8_t checkbtn(uint16_t cbtn)
{
	uint8_t sbtn;

	if(HAL_GPIO_ReadPin(BTN, cbtn)==GPIO_PIN_SET) //Check if Button is pressed
	{
		sbtn = 1; //Return 1
	}
	else sbtn = 0;

	return sbtn;
}

uint16_t getADCvalue()
{
	uint16_t adcval;

	HAL_ADC_Start(&hadc);
	if (HAL_ADC_PollForConversion(&hadc, 1000) == HAL_OK)
	{
		adcval = HAL_ADC_GetValue(&hadc);
	}
	HAL_ADC_Stop(&hadc);

	return adcval;
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

