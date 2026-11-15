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
#include "fatfs.h"

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

static volatile uint8_t uart1RecibioByte = 0;


extern const uint16_t fondo_imagen[76800];


extern const uint16_t lateral_izquierda[4900];
extern const uint16_t lateral_centro[4900];
extern const uint16_t lateral_derecha[4900];

extern const uint16_t vertical_arriba[4900];
extern const uint16_t vertical_centro[4900];
extern const uint16_t vertical_abajo[4900];

extern const uint16_t precaucion[900];

float offset_gx = 0.0f;
float offset_gy = 0.0f;
float offset_gz = 0.0f;

static uint8_t datoN1;

static volatile char lineaN1[64];
static volatile uint8_t lineaN1Lista = 0;

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
void LCD_MovLateral(const uint16_t *imagen);
void LCD_MovVertical(const uint16_t *imagen);

void LCD_Rectangulo(uint16_t x, uint16_t y, uint16_t ancho, uint16_t alto, uint16_t color);
void LCD_DibujarPaneles(void);

void LCD_MostrarAngulos(float roll, float pitch);

void LCD_ActualizarManos(float roll, float pitch);

void LCD_MostrarPrecaucion(uint16_t x, uint16_t y);
void LCD_ActualizarPrecauciones(float roll, float pitch);

void UART_EnviarAngulos(float roll, float pitch);

void SPI1_VelocidadBaja(void);
void SPI1_VelocidadAlta(void);

static void N1_Enviar(const char *mensaje);
static void N1_Error(unsigned int id, const char *detalle);
static void N1_Ejecutar(const char *linea);
static void N1_Procesar(void);

static HAL_StatusTypeDef MPU6050_LeerAngulos(float *roll, float *pitch);
static FRESULT SD_GuardarMedicion(unsigned int id, uint32_t tiempo, float roll, float pitch);


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
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  // Inicializar pantalla TFT
  LCD_Init();

  // Dibujar fondo
  LCD_MostrarFondo();

  // Dibujar paneles de angulos
  LCD_DibujarPaneles();

  // Dibujar manos
  LCD_MovLateral(lateral_centro);
  LCD_MovVertical(vertical_centro);

  // Titulos de los paneles
  LCD_Print("ROLL", 35, 60, 1, 0x001F, 0xFFFF);

  LCD_Print("PITCH", 35, 150, 1, 0x001F, 0xFFFF);


  HAL_StatusTypeDef prueba_i2c;

  prueba_i2c = HAL_I2C_IsDeviceReady(&hi2c1, 0x68 << 1, 3, 100);

  if (prueba_i2c == HAL_OK)
  {
      char mensaje[] = "MPU6050 responde por I2C\r\n";

      HAL_UART_Transmit(&huart2, (uint8_t *)mensaje, strlen(mensaje), 100);
  }
  else
  {
      char mensaje[] = "MPU6050 NO responde por I2C\r\n";

      HAL_UART_Transmit(&huart2, (uint8_t *)mensaje, strlen(mensaje), 100);
  }

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
  HAL_Delay(100);

  // Avisar que comenzara la calibracion
  char inicio[] = "Calibrando giroscopio. No mover sensor...\r\n";

  HAL_UART_Transmit(&huart2, (uint8_t *)inicio, sizeof(inicio) - 1, 100);

  // Calibrar
  MPU6050_Calibrar_Giroscopio();

  char fincalibracion[] = "Fin de la calibracion\r\n";

  	    HAL_UART_Transmit(&huart2, (uint8_t *)fincalibracion, strlen(fincalibracion), HAL_MAX_DELAY);

  	    HAL_Delay(2000);

  	  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  	  HAL_NVIC_EnableIRQ(USART1_IRQn);

  	  if (HAL_UART_Receive_IT(&huart1, &datoN1, 1) != HAL_OK)
  	  {
  	      Error_Handler();
  	  }

  	  const char lista[] = "NUCLEO2 LISTA\r\n";

  	  HAL_UART_Transmit(
  	      &huart2,
  	      (uint8_t *)lista,
  	      sizeof(lista) - 1U,
  	      100
  	  );


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {


	  if (uart1RecibioByte)
	  {
	      uart1RecibioByte = 0;

	      char aviso[] = "UART1: byte recibido\r\n";

	      HAL_UART_Transmit(
	          &huart2,
	          (uint8_t *)aviso,
	          strlen(aviso),
	          100
	      );
	  }

	  N1_Procesar();

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

          // Actualizar angulos en la pantalla
          LCD_MostrarAngulos(roll, pitch);

          // Actualizar imagenes segun los angulos
          LCD_ActualizarManos(roll, pitch);

          // Actualizar advertencias
          LCD_ActualizarPrecauciones(roll, pitch);

          // Enviar mediciones a Nucleo 1
          //UART_EnviarAngulos(roll, pitch);

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  HAL_GPIO_WritePin(GPIOB, LCD_CS_Pin|SD_CS_Pin, GPIO_PIN_SET);

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

static void N1_Enviar(const char *mensaje)
{
    // Enviar respuesta hacia Nucleo 1
    HAL_StatusTypeDef resultado;

    resultado = HAL_UART_Transmit(
        &huart1,
        (uint8_t *)mensaje,
        (uint16_t)strlen(mensaje),
        100
    );

    // Mostrar respuesta enviada en Hercules
    char debug[130];

    snprintf(debug, sizeof(debug),
             "TX Nucleo 2: %s | HAL: %d\r\n",
             mensaje, (int)resultado);

    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)debug,
        strlen(debug),
        100
    );
}


// Informar un error asociado a una solicitud.
static void N1_Error(unsigned int id, const char *detalle)
{
    char respuesta[96];

    snprintf(
        respuesta,
        sizeof(respuesta),
        "ERROR;%u;%s\r\n",
        id,
        detalle
    );

    N1_Enviar(respuesta);
}



// Ejecutar una solicitud completa.
static void N1_Ejecutar(const char *linea)
{

	// Diagnostico: mostrar comando completo recibido
	char debug_uart[100];

	snprintf(debug_uart, sizeof(debug_uart),
	         "Comando recibido N1: %s\r\n", linea);

	HAL_UART_Transmit(&huart2,
	                  (uint8_t *)debug_uart,
	                  strlen(debug_uart),
	                  100);

    char comando[16];
    char extra;
    unsigned int id;

    if (sscanf(linea, "%15[^;];%u%c", comando, &id, &extra) != 2 ||
        id == 0U ||
        id > 65535U)
    {
        N1_Error(0, "FORMATO_INVALIDO");
        return;
    }


    if (strcmp(comando, "guardar") == 0)
    {
        float roll;
        float pitch;

        // Tomar una nueva medicion del MPU6050
        HAL_StatusTypeDef estado = MPU6050_LeerAngulos(&roll, &pitch);

        if (estado != HAL_OK)
        {
            N1_Error(id, "LECTURA_SENSOR");
            return;
        }

        // Obtener tiempo de la medicion
        uint32_t tiempo = HAL_GetTick();

        // Guardar nueva medicion en la microSD
        FRESULT resultado = SD_GuardarMedicion(id, tiempo, roll, pitch);

        if (resultado == FR_OK)
        {
            char respuesta[64];

            snprintf(respuesta, sizeof(respuesta), "GUARDADO;%u\r\n", id);

            N1_Enviar(respuesta);
        }
        else
        {
            char detalle[40];

            snprintf(detalle, sizeof(detalle), "SD_ERROR_%u", (unsigned int)resultado);

            N1_Error(id, detalle);
        }

        return;
    }


    if (strcmp(comando, "leer") != 0)
    {
        N1_Error(id, "COMANDO_INVALIDO");
        return;
    }

    char respuesta[96];

    // Nucleo 1 activa el sonido al recibir esta confirmacion.
    snprintf(
        respuesta,
        sizeof(respuesta),
        "INICIO;%u\r\n",
        id
    );

    N1_Enviar(respuesta);

    // Leer acelerometro: suficiente para calcular inclinacion.
    uint8_t datos[6];

    if (HAL_I2C_Mem_Read(
            &hi2c1,
            0x68U << 1,
            0x3B,
            I2C_MEMADD_SIZE_8BIT,
            datos,
            6,
            100
        ) != HAL_OK)
    {
        N1_Error(id, "LECTURA_SENSOR");
        return;
    }

    int16_t ax =
        (int16_t)(((uint16_t)datos[0] << 8) | datos[1]);

    int16_t ay =
        (int16_t)(((uint16_t)datos[2] << 8) | datos[3]);

    int16_t az =
        (int16_t)(((uint16_t)datos[4] << 8) | datos[5]);

    if (ax == 0 && ay == 0 && az == 0)
    {
        N1_Error(id, "VECTOR_NULO");
        return;
    }

    float ax_g = ax / 16384.0f;
    float ay_g = ay / 16384.0f;
    float az_g = az / 16384.0f;

    float roll =
        atan2f(ay_g, az_g) * 180.0f / 3.14159265f;

    float pitch =
        atan2f(
            -ax_g,
            sqrtf(ay_g * ay_g + az_g * az_g)
        ) * 180.0f / 3.14159265f;

    // Usar las funciones de pantalla de TU proyecto.
    LCD_MostrarAngulos(roll, pitch);
    LCD_ActualizarManos(roll, pitch);
    LCD_ActualizarPrecauciones(roll, pitch);


    // Transmitir angulos como enteros en centesimas de grado.
    int rollCdeg = (int)lroundf(roll * 100.0f);
    int pitchCdeg = (int)lroundf(pitch * 100.0f);

    snprintf(
        respuesta,
        sizeof(respuesta),
        "DATOS;%u;%d;%d\r\n",
        id,
        rollCdeg,
        pitchCdeg
    );

    N1_Enviar(respuesta);
}


// Procesar una linea recibida, fuera de la interrupcion.
static void N1_Procesar(void)
{
    uint8_t lista = lineaN1Lista;

    if (lista == 0U)
    {
        return;
    }

    if (lista == 2U)
    {
        lineaN1Lista = 0;
        N1_Error(0, "LINEA_INVALIDA");
        return;
    }

    // Copiar mientras la interrupcion conserva la linea.
    char copia[64];

    for (uint8_t i = 0; i < sizeof(copia); i++)
    {
        copia[i] = lineaN1[i];

        if (copia[i] == '\0')
        {
            break;
        }
    }

    copia[sizeof(copia) - 1U] = '\0';

    lineaN1Lista = 0;

    N1_Ejecutar(copia);
}


// Recibir caracteres sin hacer lecturas I2C ni dibujar en la ISR.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static uint8_t posicion = 0;
    static uint8_t descartar = 0;

    if (huart->Instance != USART1)
    {
        return;
    }

    if (lineaN1Lista == 0U)
    {
        char c = (char)datoN1;
        uart1RecibioByte = 1;

        if (c == '\n')
        {
            lineaN1[posicion] = '\0';

            if (descartar)
            {
                lineaN1Lista = 2;
            }
            else if (posicion > 0U)
            {
                lineaN1Lista = 1;
            }

            posicion = 0;
            descartar = 0;
        }
        else if (c != '\r' && !descartar)
        {
            if (c != '\0' &&
                posicion < sizeof(lineaN1) - 1U)
            {
                lineaN1[posicion++] = c;
            }
            else
            {
                descartar = 1;
            }
        }
    }

    HAL_UART_Receive_IT(&huart1, &datoN1, 1);
}

static HAL_StatusTypeDef MPU6050_LeerAngulos( float *roll, float *pitch)
{
    uint8_t datos[6];

    if (roll == NULL || pitch == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef estado = HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x3B, I2C_MEMADD_SIZE_8BIT, datos, 6, 100);

    if (estado != HAL_OK)
    {
        return estado;
    }

    int16_t Ax = (int16_t)(((uint16_t)datos[0] << 8) | datos[1]
    );

    int16_t Ay = (int16_t)(((uint16_t)datos[2] << 8) | datos[3]
    );

    int16_t Az = (int16_t)(((uint16_t)datos[4] << 8) | datos[5]
    );

    if (Ax == 0 && Ay == 0 && Az == 0)
    {
        return HAL_ERROR;
    }

    float ax_g = Ax / 16384.0f;
    float ay_g = Ay / 16384.0f;
    float az_g = Az / 16384.0f;

    *roll = atan2f(ay_g, az_g) * 180.0f / 3.14159265f;

    *pitch = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 180.0f / 3.14159265f;

    return HAL_OK;
}


static FRESULT SD_GuardarMedicion(unsigned int id, uint32_t tiempo, float roll, float pitch)
{
    FATFS fs;
    FIL archivo;
    FRESULT resultado;
    FRESULT cierre;
    UINT escritos;

    char linea[100];

    const char cabecera[] = "ID,Tiempo_ms,Roll_deg,Pitch_deg\r\n";

    // Montar tarjeta microSD
    resultado = f_mount(&fs, "", 1);

    if (resultado != FR_OK)
    {
        f_mount(NULL, "", 0);
        return resultado;
    }

    // Abrir archivo sin borrar datos anteriores
    resultado = f_open(&archivo, "DATOS.CSV", FA_OPEN_ALWAYS | FA_WRITE);

    if (resultado != FR_OK)
    {
        f_mount(NULL, "", 0);
        return resultado;
    }

    // Si el archivo esta vacio, escribir cabecera
    if (f_size(&archivo) == 0)
    {
        resultado = f_write(&archivo, cabecera, sizeof(cabecera) - 1, &escritos);

        if (resultado != FR_OK ||
            escritos != sizeof(cabecera) - 1)
        {
            if (resultado == FR_OK)
                resultado = FR_DISK_ERR;

            goto finalizar;
        }
    }

    // Colocar cursor al final
    resultado = f_lseek(&archivo, f_size(&archivo));

    if (resultado != FR_OK)
        goto finalizar;

    // Preparar medicion
    int longitud = snprintf(
        linea,
        sizeof(linea),
        "%u,%lu,%.2f,%.2f\r\n",
        id,
        (unsigned long)tiempo,
        roll,
        pitch
    );

    if (longitud <= 0 ||
        longitud >= (int)sizeof(linea))
    {
        resultado = FR_INVALID_PARAMETER;
        goto finalizar;
    }

    // Escribir medicion
    resultado = f_write(&archivo, linea, (UINT)longitud, &escritos);

    if (resultado == FR_OK &&
        escritos != (UINT)longitud)
    {
        resultado = FR_DISK_ERR;
    }

finalizar:

    // Cerrar archivo
    cierre = f_close(&archivo);

    if (resultado == FR_OK)
    {
        resultado = cierre;
    }

    // Desmontar microSD
    f_mount(NULL, "", 0);

    return resultado;
}



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


void LCD_MovLateral(const uint16_t *imagen)
{
    SetWindows(220, 50, 289, 119);

    for (uint32_t i = 0; i < 4900; i++)
    {
        uint16_t color = imagen[i];

        LCD_DATA(color >> 8);
        LCD_DATA(color & 0xFF);
    }
}

void LCD_MovVertical(const uint16_t *imagen)
{
    SetWindows(220, 140, 289, 209);

    for (uint32_t i = 0; i < 4900; i++)
    {
        uint16_t color = imagen[i];

        LCD_DATA(color >> 8);
        LCD_DATA(color & 0xFF);
    }
}

void LCD_MostrarAngulos(float roll, float pitch)
{
    char texto[20];

    // Limpiar area del angulo Roll
    LCD_Rectangulo(25, 80, 133, 32, 0xFFFF);

    snprintf(texto, sizeof(texto), "%.1f", roll);

    // Numero grande
    LCD_Print(texto, 30, 85, 2, 0x001F, 0xFFFF);

    // Unidad
    LCD_Print("deg", 115, 92, 1, 0x001F, 0xFFFF);


    // Limpiar area del angulo Pitch
    LCD_Rectangulo(25, 170, 133, 32, 0xFFFF);

    snprintf(texto, sizeof(texto), "%.1f", pitch);

    // Numero grande
    LCD_Print(texto, 30, 175, 2, 0x001F, 0xFFFF);

    // Unidad
    LCD_Print("deg", 115, 182, 1, 0x001F, 0xFFFF);
}

void LCD_ActualizarManos(float roll, float pitch)
{
    static int8_t estado_lateral = 0;
    static int8_t estado_vertical = 0;

    int8_t nuevo_lateral;
    int8_t nuevo_vertical;

    // Determinar posicion lateral
    if (roll < -15.0f)
    {
        nuevo_lateral = -1;
    }
    else if (roll > 15.0f)
    {
        nuevo_lateral = 1;
    }
    else
    {
        nuevo_lateral = 0;
    }

    // Determinar posicion vertical
    if (pitch < -15.0f)
    {
        nuevo_vertical = -1;
    }
    else if (pitch > 15.0f)
    {
        nuevo_vertical = 1;
    }
    else
    {
        nuevo_vertical = 0;
    }

    // Actualizar imagen lateral solo si cambia
    if (nuevo_lateral != estado_lateral)
    {
        if (nuevo_lateral == -1)
        {
            LCD_MovLateral(lateral_izquierda);
        }
        else if (nuevo_lateral == 1)
        {
            LCD_MovLateral(lateral_derecha);
        }
        else
        {
            LCD_MovLateral(lateral_centro);
        }

        estado_lateral = nuevo_lateral;
    }

    // Actualizar imagen vertical solo si cambia
    if (nuevo_vertical != estado_vertical)
    {
        if (nuevo_vertical == -1)
        {
            LCD_MovVertical(vertical_arriba);
        }
        else if (nuevo_vertical == 1)
        {
            LCD_MovVertical(vertical_abajo);
        }
        else
        {
            LCD_MovVertical(vertical_centro);
        }

        estado_vertical = nuevo_vertical;
    }
}

void LCD_MostrarPrecaucion(uint16_t x, uint16_t y)
{
    uint16_t color;

    SetWindows(x, y, x + 29, y + 29);

    for (uint32_t i = 0; i < 900; i++)
    {
        color = precaucion[i];

        LCD_DATA(color >> 8);
        LCD_DATA(color & 0xFF);
    }
}

void LCD_ActualizarPrecauciones(float roll, float pitch)
{
    static uint8_t alerta_roll = 0;
    static uint8_t alerta_pitch = 0;

    uint8_t nueva_alerta_roll;
    uint8_t nueva_alerta_pitch;

    // Limites de prueba
    float limite_roll = 30.0f;
    float limite_pitch = 30.0f;

    // Comprobar si se superaron los limites
    nueva_alerta_roll = (fabsf(roll) > limite_roll);
    nueva_alerta_pitch = (fabsf(pitch) > limite_pitch);

    // Actualizar advertencia Roll
    if (nueva_alerta_roll != alerta_roll)
    {
        if (nueva_alerta_roll)
        {
            LCD_MostrarPrecaucion(164, 55);
        }
        else
        {
            LCD_Rectangulo(164, 55, 30, 30, 0xFFFF);
        }

        alerta_roll = nueva_alerta_roll;
    }

    // Actualizar advertencia Pitch
    if (nueva_alerta_pitch != alerta_pitch)
    {
        if (nueva_alerta_pitch)
        {
            LCD_MostrarPrecaucion(164, 145);
        }
        else
        {
            LCD_Rectangulo(164, 145, 30, 30, 0xFFFF);
        }

        alerta_pitch = nueva_alerta_pitch;
    }
}

void UART_EnviarAngulos(float roll, float pitch)
{
    char mensaje[64];

    int longitud = snprintf(mensaje, sizeof(mensaje), "ROLL:%.1f,PITCH:%.1f\r\n", roll, pitch);

    if (longitud > 0 && longitud < sizeof(mensaje))
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)mensaje, (uint16_t)longitud, 100);
    }
}

void SPI1_VelocidadBaja(void)
{
    // Deseleccionar pantalla y SD
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port,
                      LCD_CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(SD_CS_GPIO_Port,
                      SD_CS_Pin, GPIO_PIN_SET);

    // Desactivar SPI
    HAL_SPI_DeInit(&hspi1);

    // Configurar prescaler 256
    hspi1.Init.BaudRatePrescaler =
        SPI_BAUDRATEPRESCALER_256;

    // Reiniciar SPI
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}


void SPI1_VelocidadAlta(void)
{
    // Deseleccionar pantalla y SD
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

    // Desactivar SPI
    HAL_SPI_DeInit(&hspi1);

    // Restaurar prescaler 8
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;

    // Reiniciar SPI
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
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

void LCD_Rectangulo(uint16_t x, uint16_t y, uint16_t ancho, uint16_t alto, uint16_t color)
{
    if (ancho == 0 || alto == 0 || x >= 320 || y >= 240 || ancho > 320 - x || alto > 240 - y)
    {
        return;
    }

    SetWindows(x, y, x + ancho - 1, y + alto - 1);

    for (uint32_t i = 0; i < (uint32_t)ancho * alto; i++)
    {
        LCD_DATA(color >> 8);
        LCD_DATA(color & 0xFF);
    }
}

void LCD_DibujarPaneles(void)
{
    uint16_t celeste = 0x05BF;
    uint16_t blanco = 0xFFFF;

    // Panel Roll: borde celeste
    LCD_Rectangulo(20, 50, 180, 70, celeste);

    // Interior blanco
    LCD_Rectangulo(23, 53, 174, 64, blanco);

    // Panel Pitch: borde celeste
    LCD_Rectangulo(20, 140, 180, 70, celeste);

    // Interior blanco
    LCD_Rectangulo(23, 143, 174, 64, blanco);
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
