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
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "ili9341.h"
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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern const uint16_t fondo[];
extern const uint16_t fondo_imagen[76800];



float offset_gx = 0.0f;
float offset_gy = 0.0f;
float offset_gz = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void MPU6050_Calibrar_Giroscopio(void);

void LCD_MostrarFondo(void);
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // Inicializar pantalla TFT
  LCD_Init();

  // Pintar pantalla de azul
  LCD_Clear(0x001F);
  HAL_Delay(1000);

  LCD_MostrarFondo();


  uint8_t despertar = 0x00;

  HAL_StatusTypeDef estado = HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x6B, I2C_MEMADD_SIZE_8BIT, &despertar, 1, 100);

  // Verificar si la escritura fue exitosa
  if (estado == HAL_OK)
  {
      char texto[] = "Sensor inicializado\r\n";

      HAL_UART_Transmit(&huart2, (uint8_t *)texto, sizeof(texto) - 1, 100);
  }
  else
  {
      char texto[] = "Error al inicializar sensor\r\n";

      HAL_UART_Transmit(&huart2, (uint8_t *)texto, sizeof(texto) - 1, 100);

      Error_Handler();
  }

  // Configurar giroscopio en +/-250 grados/s
  uint8_t gyro_config = 0x00;

  HAL_StatusTypeDef estado_config;

  estado_config = HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x1B, I2C_MEMADD_SIZE_8BIT, &gyro_config, 1, 100);

  if (estado_config != HAL_OK)
  {
      Error_Handler();
  }

  // Configurar acelerometro en +/-2 g
  uint8_t accel_config = 0x00;

  if (HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x1C, I2C_MEMADD_SIZE_8BIT, &accel_config, 1, 100) != HAL_OK)
  {
      Error_Handler();
  }

  // Dar tiempo al sensor para estabilizarse
  HAL_Delay(500);

  // Avisar que comenzara la calibracion
  char inicio[] = "Calibrando giroscopio. No mover sensor...\r\n";

  HAL_UART_Transmit(&huart2, (uint8_t *)inicio, sizeof(inicio) - 1, 100);

  // Calibrar
  MPU6050_Calibrar_Giroscopio();

  char fincalibracion[] = "Fin de la calibracion\r\n";

  	    HAL_UART_Transmit(&huart2, (uint8_t *)fincalibracion, strlen(fincalibracion), HAL_MAX_DELAY);

  	    HAL_Delay(2000);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	/*while (1) {

	    char mensaje[] = "HOLA";

	    HAL_UART_Transmit(&huart1, (uint8_t *)mensaje, strlen(mensaje), HAL_MAX_DELAY);

	    HAL_Delay(1000);



	}*/
  while (1)
  {
      uint8_t datos[6];

      int16_t Ax, Ay, Az;
      int16_t Gx, Gy, Gz;

      float ax_g, ay_g, az_g;
      float gx_dps, gy_dps, gz_dps;

      float roll, pitch;

      char mensaje[100];

      HAL_StatusTypeDef estado_lectura;

      // Leer los 6 bytes del acelerometro
      estado_lectura = HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x3B, I2C_MEMADD_SIZE_8BIT, datos, 6, 100);

      if (estado_lectura == HAL_OK)
      {
          // Combinar bytes altos y bajos
          Ax = (int16_t)((datos[0] << 8) | datos[1]);
          Ay = (int16_t)((datos[2] << 8) | datos[3]);
          Az = (int16_t)((datos[4] << 8) | datos[5]);

          // Convertir a unidades de gravedad (g)
          ax_g = Ax / 16384.0f;
          ay_g = Ay / 16384.0f;
          az_g = Az / 16384.0f;

          // Calcular angulos de inclinacion
          roll = atan2f(ay_g, az_g) * 180.0f / 3.14159265f;

          pitch = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 180.0f / 3.14159265f;

          // Preparar mensaje
          snprintf(mensaje, sizeof(mensaje),"Ax: %.2f g | Ay: %.2f g | Az: %.2f g\r\n", ax_g, ay_g, az_g);

          // Enviar al monitor serial
          HAL_UART_Transmit(&huart2, (uint8_t *)mensaje, strlen(mensaje), 100);

          snprintf(mensaje, sizeof(mensaje), "Roll: %.2f grados | Pitch: %.2f grados\r\n", roll, pitch);

          HAL_UART_Transmit(&huart2, (uint8_t *)mensaje, strlen(mensaje), 100);
      }

      // Leer los 6 bytes del giroscopio
            estado_lectura = HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x43, I2C_MEMADD_SIZE_8BIT, datos, 6, 100);

            if (estado_lectura == HAL_OK)
            {
                // Combinar bytes altos y bajos
                Gx = (int16_t)((datos[0] << 8) | datos[1]);
                Gy = (int16_t)((datos[2] << 8) | datos[3]);
                Gz = (int16_t)((datos[4] << 8) | datos[5]);

                // Convertir a grados por segundo
                gx_dps = (Gx / 131.0f) - offset_gx;
                gy_dps = (Gy / 131.0f) - offset_gy;
                gz_dps = (Gz / 131.0f) - offset_gz;

                // Preparar mensaje
                snprintf(mensaje, sizeof(mensaje),"Gx: %.2f dps | Gy: %.2f dps | Gz: %.2f dps\r\n", gx_dps, gy_dps, gz_dps);

                // Enviar al monitor serial
                HAL_UART_Transmit(&huart2, (uint8_t *)mensaje, strlen(mensaje), 100);
            }
            // Separar cada grupo de mediciones

                char separador[] = "--------------------------\r\n";

                HAL_UART_Transmit(&huart2, (uint8_t *)separador, strlen(separador), 100);


      HAL_Delay(500);
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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

  /** Initializes the CPU, AHB and APB buses clocks
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_CS_Pin|SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RESET_Pin */
  GPIO_InitStruct.Pin = LCD_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LCD_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LCD_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin SD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void LCD_MostrarFondo(void)
{
    uint16_t color;

    SetWindows(0, 0, 319, 239);

    for (uint32_t i = 0; i < 76800; i++)
    {
        color = fondo_imagen[i];

        LCD_DATA(color >> 8);
        LCD_DATA(color & 0xFF);
    }
}

void MPU6050_Calibrar_Giroscopio(void)
{
    uint8_t datos[6];

    int16_t Gx, Gy, Gz;

    int32_t suma_gx = 0;
    int32_t suma_gy = 0;
    int32_t suma_gz = 0;

    uint16_t muestras_validas = 0;

    for (int i = 0; i < 500; i++)
    {
        HAL_StatusTypeDef estado;

        estado = HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x43, I2C_MEMADD_SIZE_8BIT, datos, 6, 100);

        if (estado == HAL_OK)
        {
            Gx = (int16_t)((datos[0] << 8) | datos[1]);
            Gy = (int16_t)((datos[2] << 8) | datos[3]);
            Gz = (int16_t)((datos[4] << 8) | datos[5]);

            suma_gx += Gx;
            suma_gy += Gy;
            suma_gz += Gz;

            muestras_validas++;
        }

        HAL_Delay(5);
    }

    if (muestras_validas >= 450)
    {
        offset_gx = ((float)suma_gx / muestras_validas) / 131.0f;
        offset_gy = ((float)suma_gy / muestras_validas) / 131.0f;
        offset_gz = ((float)suma_gz / muestras_validas) / 131.0f;
    }
    else
    {
        Error_Handler();
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
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
