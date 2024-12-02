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
#include <stdio.h>
#include <string.h>
#include <stdbool.h>


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
#define ADS1115_ADDRESS 0x48 //<< 1  // Address of the ADS1115 (shifted for HAL library)
#define ADS1115_POINTER_CONVERSION 0x00
#define ADS1115_POINTER_CONFIG 0x01

uint8_t ADSwrite[3];
uint8_t ADSread[2];
int16_t reading;
static float voltage[4];
const float voltageConv = 0.256 / 32768.0;  // Update this based on the FSR (prev 6.114)


/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t Binary4Bit(uint8_t num);
void selectMux(uint8_t type, uint8_t IO, uint8_t bin_pin);
void mux(uint8_t pin1, uint8_t pin2, uint8_t IO);

void ADS1115_WriteConfig(uint16_t config);
uint16_t ADS1115_ReadConversion();
float* contDetect();

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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //setup code
  // Initialize Comparator with COMP_LAT = 0b0 and COMP_QUE = 0b11
//   COMP_HandleTypeDef hcomp;
//   hcomp.Instance = COMP1; // Example using COMP1
//   hcomp.Init.InvertingInput = COMP_INVERTINGINPUT_IO1; // Adjust according to your setup
//   hcomp.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1; // Adjust according to your setup
//   hcomp.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
//   hcomp.Init.Mode = COMP_POWERMODE_HIGHSPEED;
//   hcomp.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
//   hcomp.Init.TriggerMode = COMP_TRIGGERMODE_NONE; // No interrupt generation
//
//   HAL_COMP_Init(&hcomp);
//
//   __HAL_COMP_COMP_LAT_RESET(hcomp.Instance); // Set COMP_LAT to 0b0
//   __HAL_COMP_COMP_QUE_SET(hcomp.Instance, 0b11); // Set COMP_QUE to 0b11
//
//   HAL_COMP_Start(&hcomp); // Start comparator




  //Pin defintions
  //Only doing 16 right now to test
  //uint64_t pin1 = 0x1010101010101010101010101010101010101010101010101010101010101010;
  //uint64_t pin2 = 011100;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //Testing Header 00 Level 0 MUX00 Pin X0 and Header 01 Level 0 MUX02 X0
  //Output Pin 2 Outer Mux (ABC High)
  //Check for Input pin 2 (ABC High)
  //Select mux pin, turn on output
  //Check each pin and populate array

//	 COM_InitTypeDef comlog;
//
//	 comlog.BaudRate = 115200;
//	 comlog.WordLength = COM_WORDLENGTH_8B;
//	 comlog.StopBits = COM_STOPBITS_1;
//	 comlog.Parity = COM_PARITY_NONE;
//	 comlog.HwFlowCtl = COM_HWCONTROL_NONE;
//
////	 BSP_COM_Init(COM1, &comlog);
//
//	 printf("Hello World!");
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); //output pin

	 //Current source for ADC // doing to see if this changes voltage for continuity check
	 //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	 //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	 //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	 //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
	 uint64_t array1;//just need 1 var for now, need 4 for full thing
	  //Replace with actual pin


	 //TESTCODE

//	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET); //D
//	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); //C working (Input)
//	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); //B working (Input)
//	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); //A (Input)
//	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //D
//	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); //C working
//	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); //B working
//	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); //A

	 //selectMux(1,0, 0b0010); //Mux 0, X1, Output
	 //mux(0b0010, 0b0000, 0);

	 //Small Test 01 - Mobo Only (Single connect)
	 //Connections:  O_X7 -> I_X7 (X6-X7 is still not working, trying X7)
	 float* cont_array[15];
	 bool yah = false;
	 mux(0b0110, 0b0000, 1); //X6
	 HAL_Delay(20);
	 mux(0b0110, 0b0000, 0); //X6
	 // Test reading ADS1115 channels
	 HAL_Delay(100);
     cont_array[0] = contDetect();

	 for (uint8_t i = 0; i < 15; i++) {
	   mux(Binary4Bit(i), 0b0000, 1);
	   cont_array[i] = contDetect();
	 }


	 //GPIO_PinState pinState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3); //Read input
	 //mux(0b1110, 0b0000, 0); // Test no connect
	 //pinState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3); //Read input





	 /*
	 for (int z = 0; z < 16; z++) {
		 mux(0000, Binary4Bit(z) , 0);
		 for (int i = 0; i < 16; i++) {
			 mux(0000, Binary4Bit(i) , 1);
			 GPIO_PinState pinState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3); //Read input
			 if (pinState == GPIO_PIN_SET) {
				 array1 |= (uint64_t)1 << i;
			 }
		 }

		 if (array1 ^ pin1) {
			 //do something here idk
		 }
	 }
	 */
  /* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */

}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
	  hi2c1.Init.Timing = 0x2000090E;
	  hi2c1.Init.OwnAddress1 = 0;
	  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	  hi2c1.Init.OwnAddress2 = 0;
	  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	  {
		Error_Handler();
	  }

	  /** Configure Analogue filter
	  */
	  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	  {
		Error_Handler();
	  }

	  /** Configure Digital filter
	  */
	  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	  {
		Error_Handler();
	  }


  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
/* USER CODE BEGIN MX_GPIO_Init_1 */


static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
//  COMP_HandleTypeDef hcomp;
//   hcomp.Instance = COMP1; // Example using COMP1
//   hcomp.Init.InvertingInput = COMP_INVERTINGINPUT_IO1; // Adjust according to your setup
//   hcomp.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1; // Adjust according to your setup
//   hcomp.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
//   hcomp.Init.Mode = COMP_POWERMODE_HIGHSPEED;
//   hcomp.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
//   hcomp.Init.TriggerMode = COMP_TRIGGERMODE_NONE; // No interrupt generation
//
//   HAL_COMP_Init(&hcomp);
//
//   __HAL_COMP_COMP_LAT_RESET(hcomp.Instance); // Set COMP_LAT to 0b0
//   __HAL_COMP_COMP_QUE_SET(hcomp.Instance, 0b11); // Set COMP_QUE to 0b11
//
//   HAL_COMP_Start(&hcomp); // Start comparator

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA2 PA4 PA5 PA6
                           PA7 PA8 PA9 PA10
                           PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3
                           PB5 PB6 PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB13
                           PB14 PB15 PB4 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_1 */
/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t Binary4Bit(uint8_t num) {
	uint8_t bit;
    for (int i = 3; i >= 0; i--) {
    	bit = (num >> i) & 1;
    }
    return bit;
}



void mux(uint8_t pin1, uint8_t pin2, uint8_t IO) { //selectMux abstraction
	selectMux(1, IO, pin1); //deals with inner mux first
	selectMux(0, IO, pin2);
}

void selectMux(uint8_t type, uint8_t IO, uint8_t bin_pin) {
    // Determine MUX selection and IO direction
    if (type == 1) { // 1 is Inner, 0 is Outer
        if (IO == 1) { // 1 is Input, 0 is Output
            // Reset all pins to low before setting them according to bin_pin
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_RESET);

            // Set pins based on bin_pin bits
            if (bin_pin & 0b0001) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
            if (bin_pin & 0b0010) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
            if (bin_pin & 0b0100) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
            if (bin_pin & 0b1000) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);

        } else { // If Output
            // Reset all pins to low before setting them according to bin_pin
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);

            // Set pins based on bin_pin bits
            if (bin_pin & 0b0001) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
            if (bin_pin & 0b0010) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            if (bin_pin & 0b0100) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
            if (bin_pin & 0b1000) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
        }
    } else { // If "Outer"
        if (IO == 1) { // 1 is Input, 0 is Output
            // Reset all pins to low before setting them according to bin_pin
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_RESET);

            // Set pins based on bin_pin bits
            if (bin_pin & 0b0001) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
            if (bin_pin & 0b0010) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
            if (bin_pin & 0b0100) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
            if (bin_pin & 0b1000) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);

        } else { // If Output
            // Reset all pins to low before setting them according to bin_pin
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8, GPIO_PIN_RESET);

            // Set pins based on bin_pin bits
            if (bin_pin & 0b0001) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
            if (bin_pin & 0b0010) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
            if (bin_pin & 0b0100) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
            if (bin_pin & 0b1000) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        }
    }
}

void ADS1115_WriteConfig(uint16_t config)
{
  uint8_t configData[3];
  configData[0] = ADS1115_POINTER_CONFIG;
  configData[1] = (uint8_t)(config >> 8);   // MSB
  configData[2] = (uint8_t)(config & 0xFF); // LSB
  HAL_StatusTypeDef result;
  result = HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, configData, 3, HAL_MAX_DELAY);
  if (result)
  {
    // Transmission Error
    Error_Handler();
  }
}

uint16_t ADS1115_ReadConversion()
{
  uint8_t convData[2];
  uint16_t value;

  if (HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADS1115_POINTER_CONVERSION, 1, HAL_MAX_DELAY) != HAL_OK)
  {
    // Transmission Error
    Error_Handler();
  }

  if (HAL_I2C_Master_Receive(&hi2c1, ADS1115_ADDRESS << 1, convData, 2, HAL_MAX_DELAY) != HAL_OK)
  {
    // Reception Error
    Error_Handler();
  }

  value = (convData[0] << 8) | convData[1];
  return value;
}

float* contDetect() {
	for(int i=0; i< 4; i++){
				ADSwrite[0] = 0x01;

				switch(i){
					case(0):
						ADSwrite[1] = 0xCD; //11001101 #bits 1-3 (110) correspond to PGA gain of 0.256V
					break;
					case(1):
						ADSwrite[1] = 0xDD; //11011101
					break;
					case(2):
						ADSwrite[1] = 0xED;
					break;
					case(3):
						ADSwrite[1] = 0xFD;
					break;
				}

				ADSwrite[2] = 0x83; //10000011 LSB

				 HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADSwrite, 3, 100);

				// Wait for conversion to complete by polling OS bit
				uint8_t configRegister[2];
				do {
					HAL_I2C_Mem_Read(&hi2c1, ADS1115_ADDRESS << 1, 0x01, I2C_MEMADD_SIZE_8BIT, configRegister, 2, 100);
				} while ((configRegister[0] & 0x80) == 0);

				// Read conversion results
				HAL_I2C_Mem_Read(&hi2c1, ADS1115_ADDRESS << 1, 0x00, I2C_MEMADD_SIZE_8BIT, ADSread, 2, 100);
				reading = (ADSread[0] << 8) | ADSread[1];
				if(reading < 0) {
					reading = 0;
				}
				voltage[i] = reading * voltageConv;


	}
	return voltage;
}

  //printf("Channel %d: %f V\n", channel, voltage);


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL err	or return state */
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
