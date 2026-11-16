/* USER CODE BEGIN Header */
// Proyecto 3 Electronica Digital 2
// Parte 7: indicador sonoro con DAC
// Parte 6: indicador visual con Neopixel
// Parte 2: comunicación UART (esp32, nucleo 2 y computadora)
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "Neopixel.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {encendido, recibiendo, enviando, guardando} EstadoSistema;
typedef enum{OPERACION_NINGUNA, OPERACION_LEER, OPERACION_GUARDAR} OperacionN2;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AUDIO1             32U
#define TIM6_CLK_HZ        84000000UL
#define TIM6_PSC           0U

// efectos de sonido
#define SONIDO_StartMedicion    0U
#define SONIDO_SaveMedicion     1U
#define SONIDO_AlertaLimite 	2U

#define ESP_BUFFER_SIZE 64U
#define N2_BUFFER_SIZE       256U
#define N2_LINEA_SIZE        96U

#define N2_TIMEOUT_MS        15000U
#define INDICACION_TX_MS     300U

// 3000 centesimas de grado = 30 grados
#define LIMITE_ANGULO_CDEG   3000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;
DMA_HandleTypeDef hdma_tim1_ch1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

static uint32_t ondaDAC[AUDIO1];
EstadoSistema estado = encendido;

// frecuencias y duraciones de efectos de sonido
static const uint16_t StartMedicion[] = {
		  262, 0,
		  294, 0,
		  262, 0,
	      311, 0,
	      392, 0,
	      466, 0,
	      440
};
static const uint16_t StartMedicionDuraciones[] = {
		    360, 40,
		    200, 50,
		    360, 40,
		    180, 30,
		    170, 20,
		    170, 30,
		    560
};
static const uint16_t SaveMedicion[] = {
		  659, 0,
		  554, 0,
		  554, 0,
		  494, 0,
		  554, 0,
		  659
};
static const uint16_t SaveMedicionDuraciones[] = {
		  100, 20,
		   90, 40,
		  380, 10,
	       90, 10,
	      120, 10,
	      180
};
static const uint16_t AlertaLimite[] = {1200, 0, 1200, 0, 1200};
static const uint16_t AlertaLimiteDuraciones[] = {100, 100, 100, 100, 250};

//variables de estado del indicador sonoro
static const uint16_t *melodiaActual = 0;
static const uint16_t *duracionesActuales = 0;
static uint16_t cantidadNotas = 0;
static uint16_t notaActual = 0;
static uint8_t dacActivo = 0;
static uint32_t inicioNota = 0;

float brilloled;

//esp32
static uint8_t datoESP;

static volatile uint8_t bufferESP[ESP_BUFFER_SIZE];
static volatile uint8_t entradaESP = 0;
static volatile uint8_t salidaESP = 0;


static uint8_t datoN2;
static volatile uint8_t bufferN2[N2_BUFFER_SIZE];
static volatile uint16_t entradaN2 = 0;
static volatile uint16_t salidaN2 = 0;
static volatile uint8_t desbordeN2 = 0;
static OperacionN2 operacionN2 = OPERACION_NINGUNA;
static unsigned int solicitudN2 = 0;
static uint32_t inicioOperacionN2 = 0;
static uint32_t inicioIndicacionTX = 0;
static uint8_t inicioMedicionConfirmado = 0;

static uint8_t alertaPendiente = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

static void generarOnda(void);
static uint32_t calcularARR(uint16_t frecuencia);
static void Audio_Tono(uint16_t frecuencia);
static void playTone(const uint16_t *notas, const uint16_t *duraciones, uint16_t cantidad);
void Audio_Reproducir(uint8_t efecto);
void Audio_Actualizar(void);
void Neopixel_Actualizar(void);

static void ESP_Responder(const char *mensaje);
static void ESP_EjecutarComando(const char *comando);
static void ESP_Procesar(void);

static void Sistema_Finalizar(const char *mensaje);
static void N2_Interpretar(const char *linea);
static void N2_Procesar(void);
static void Sistema_Actualizar(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Funciones de comunicación NUCLEO 1 con ESP32 ////////////////////////////////////////////
static void ESP_Responder(const char *mensaje)
{
    uint16_t longitud = (uint16_t)strlen(mensaje);
    HAL_UART_Transmit(&huart3, (uint8_t *)mensaje, longitud, 100);
    HAL_UART_Transmit(&huart2, (uint8_t *)mensaje, longitud, 100);
}

static void ESP_EjecutarComando(const char *comando)
{
    OperacionN2 nuevaOperacion;

    if (strcmp(comando, "leer") == 0){
        nuevaOperacion = OPERACION_LEER;
    }
    else if (strcmp(comando, "guardar") == 0){
        nuevaOperacion = OPERACION_GUARDAR;
    }
    else{
        ESP_Responder("ERROR: use leer o guardar\r\n");
        return;
    }

    if (operacionN2 != OPERACION_NINGUNA)
    {
        ESP_Responder("OCUPADO: espere la respuesta\r\n");
        return;
    }

    // Identificador entre 1 y 65535
    solicitudN2 = (solicitudN2 % 65535U) + 1U;

    operacionN2 = nuevaOperacion;
    inicioMedicionConfirmado = 0;

    estado = (nuevaOperacion == OPERACION_LEER)
           ? recibiendo
           : guardando;

    char mensaje[32];

    snprintf(mensaje, sizeof(mensaje), "%s;%u\n", comando, solicitudN2);

    inicioOperacionN2 = HAL_GetTick();

    if (HAL_UART_Transmit(&huart1, (uint8_t *)mensaje, (uint16_t)strlen(mensaje), 100) != HAL_OK)
    {
        Sistema_Finalizar("ERROR: no se pudo enviar a Nucleo 2\r\n");
    }
}

// Formar comandos con los caracteres recibidos por UART
static void ESP_Procesar(void)
{
    static char comando[32];
    static uint8_t posicion = 0;
    static uint8_t descartar = 0;

    while (salidaESP != entradaESP)
    {
        char caracter = (char)bufferESP[salidaESP];

        salidaESP = (uint8_t)((salidaESP + 1U) % ESP_BUFFER_SIZE);

        //aceptar finales de linea \n o \r\n
        if (caracter == '\r'){
            continue;
        }

        if (caracter == '\n'){
            if (descartar){
                ESP_Responder("ERROR: comando demasiado largo\r\n");
            }
            else if (posicion > 0U){
                comando[posicion] = '\0';
                ESP_EjecutarComando(comando);
            }

            posicion = 0;
            descartar = 0;
        }
        else if (!descartar){
            if (posicion < sizeof(comando) - 1U){
                comando[posicion++] = caracter;
            }
            else{
                //ignorar el resto de esta linea
                descartar = 1;
            }
        }
    }
}

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  generarOnda();
  setBrightness(70);
  estado = encendido;

  // Activar una sola vez la recepcion desde ESP32
  if (HAL_UART_Receive_IT(&huart3, &datoESP, 1) != HAL_OK)
  {
      Error_Handler();
  }

  // Habilitar interrupcion del enlace con Nucleo 2
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  // Activar una sola vez la recepcion desde Nucleo 2
  if (HAL_UART_Receive_IT(&huart1, &datoN2, 1) != HAL_OK)
  {
      Error_Handler();
  }

  // Avisar al telefono y a la computadora
  ESP_Responder("NUCLEO1 LISTA\r\n");
  Audio_Reproducir(SONIDO_StartMedicion);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

 while (1)
 {

	 Audio_Actualizar();

     if (alertaPendiente && melodiaActual == 0)
     {
         alertaPendiente = 0;
         Audio_Reproducir(SONIDO_AlertaLimite);
     }

     N2_Procesar();
     Sistema_Actualizar();
     ESP_Procesar();
     Neopixel_Actualizar();


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
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 105-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 2624;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Funciones de Buzzer DAC ////////////////////////////////////////
//llena el arreglo con un ciclo de onda cuadrada
static void generarOnda(void){
    for (uint32_t x = 0; x < AUDIO1; x++){
        if (x < (AUDIO1 / 2U)){
            ondaDAC[x] = 0U;
        }
        else{
            ondaDAC[x] = 4095U;
        }
    }
}

// Calcula el valor de ARR para la frecuencia solicitada
static uint32_t calcularARR(uint16_t frecuencia){

    if (frecuencia == 0U){  //silencio antes de iniciar el timer
        return 0U;
    }

    // fs = muestras por ciclo × frecuencia del sonido */
    uint32_t frecuenciaMuestreo = AUDIO1 * (uint32_t)frecuencia;

    // CLK después del prescaler
    uint32_t relojContador = TIM6_CLK_HZ / (TIM6_PSC + 1U);

    // Cantidad de cuentas por actualización
    uint32_t cuentas = relojContador / frecuenciaMuestreo;

    // TIM6 CLK de 16 bits
    if ((cuentas < 1U) || (cuentas > 65536U)){
        Error_Handler();
        return 0U;
    }

    return cuentas - 1U;
}

// Reproducir una frecuencia o detener el sonido
static void Audio_Tono(uint16_t frecuencia){
    HAL_TIM_Base_Stop(&htim6);

    if (dacActivo){
        HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
        dacActivo = 0;
    }

    if (frecuencia == 0U){  // Frecuencia cero para silencio
        return;
    }

    uint32_t arr = calcularARR(frecuencia);

    __HAL_TIM_SET_AUTORELOAD(&htim6, arr);
    __HAL_TIM_SET_COUNTER(&htim6, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);

    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0U);

    // Preparar el DAC y el DMA
    if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, ondaDAC, AUDIO1, DAC_ALIGN_12B_R) != HAL_OK){
        Error_Handler();
    }

    dacActivo = 1;

    if (HAL_TIM_Base_Start(&htim6) != HAL_OK){
        Error_Handler();
    }
}

// Iniciar secuencia de notas
static void playTone(const uint16_t *notas, const uint16_t *duraciones, uint16_t cantidad){
    if ((notas == 0) || (duraciones == 0) || (cantidad == 0U)){
        return;
    }

    melodiaActual = notas;
    duracionesActuales = duraciones;

    cantidadNotas = cantidad;
    notaActual = 0;

    Audio_Tono(melodiaActual[notaActual]);
    inicioNota = HAL_GetTick();
}

// Seleccionar el efecto de sonido
void Audio_Reproducir(uint8_t efecto)
{
    switch (efecto){
        case SONIDO_StartMedicion:
            playTone(StartMedicion, StartMedicionDuraciones, sizeof(StartMedicion) / sizeof(StartMedicion[0]));
            break;

        case SONIDO_SaveMedicion:
            playTone(SaveMedicion, SaveMedicionDuraciones, sizeof(SaveMedicion) / sizeof(SaveMedicion[0]));
            break;

        case SONIDO_AlertaLimite:
            playTone(AlertaLimite, AlertaLimiteDuraciones, sizeof(AlertaLimite) / sizeof(AlertaLimite[0]));
            break;
        default:
            return;
    }
}

// Avanzar de nota sin delay
void Audio_Actualizar(void)
{
    if (melodiaActual == 0){
        return;
    }

    if ((uint32_t)(HAL_GetTick() - inicioNota) < duracionesActuales[notaActual]){
        return;
    }

    notaActual++;

    if (notaActual >= cantidadNotas){ // termina la secuencia
        Audio_Tono(0);
        melodiaActual = 0;
        duracionesActuales = 0;
        return;
    }

    Audio_Tono(melodiaActual[notaActual]); // inicia la siguiente nota
    inicioNota = HAL_GetTick();
}

// Funcion de Neopixel ////////////////////////////////////////////////////////////////////////
void Neopixel_Actualizar(void)
{
    static uint32_t ultimaActualizacion = 0;
    uint32_t ahora = HAL_GetTick();

    // actualizo cada 20 ms
    if ((uint32_t)(ahora - ultimaActualizacion) < 20U){
        return;
    }

    ultimaActualizacion = ahora;
    uint32_t tiempo = ahora % 2000U;
    uint8_t nivel;

    if (tiempo < 1000U){
        nivel = (uint8_t)((tiempo * 255U) / 1000U);
    }
    else{
        nivel = (uint8_t)(((2000U - tiempo) * 255U) / 1000U);
    }

    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;

    switch (estado)
    {
        case encendido: //encendido o en espera
            //blanco con parpadeo de 1 segundo
            if (((ahora / 1000U) % 2U) == 0U)
            {
                r = 255;
                g = 255;
                b = 255;
            }
            break;

        case recibiendo: //nucleo 2 recibe un comando (cualquiera)
            // transición de morado a azul
            r = (uint8_t)((128U * (255U - nivel)) / 255U);
            g = 0;
            b = 255;
            break;

        case enviando: //nucleo 2 envia los datos
            // transición de naranja a amarillo
            r = 255;
            g = (uint8_t)(80U + (175U * nivel) / 255U);
            b = 0;
            break;

        case guardando: //se guardan mediciones en SD
            // verde que aumenta y disminuye intensidad
            r = 0;
            g = nivel;
            b = 0;
            break;

        default:
            return;
    }
    setPixelColor(0, r, g, b);
    pixelShow();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3){
        uint8_t siguiente = (uint8_t)((entradaESP + 1U) % ESP_BUFFER_SIZE);

        if (siguiente != salidaESP){
            bufferESP[entradaESP] = datoESP;
            entradaESP = siguiente;
        }

        HAL_UART_Receive_IT(&huart3, &datoESP, 1);
    }
    else if (huart->Instance == USART1){

    	uint16_t siguiente = (uint16_t)((entradaN2 + 1U) % N2_BUFFER_SIZE);

        if (siguiente != salidaN2){
            bufferN2[entradaN2] = datoN2;
            entradaN2 = siguiente;
        }
        else{
            desbordeN2 = 1;
        }

        HAL_UART_Receive_IT(&huart1, &datoN2, 1);
    }
}

// funciones de comunicacion con nucleo 2 ///////////////////////////////////////////////////////////////////////


// Publicar el resultado y terminar la operacion
static void Sistema_Finalizar(const char *mensaje)
{
    operacionN2 = OPERACION_NINGUNA;
    estado = enviando;
    inicioIndicacionTX = HAL_GetTick();

    ESP_Responder(mensaje);
}

static void N2_Procesar(void)
{
    static char linea[N2_LINEA_SIZE];
    static uint16_t posicion = 0;
    static uint8_t descartar = 0;

    while (salidaN2 != entradaN2){
        if (desbordeN2){
            desbordeN2 = 0;
            posicion = 0;
            descartar = 1;

            if (operacionN2 != OPERACION_NINGUNA){
                Sistema_Finalizar("ERROR: desborde de recepcion Nucleo 2\r\n");
            }
        }

        char caracter = (char)bufferN2[salidaN2];

        salidaN2 = (uint16_t)((salidaN2 + 1U) % N2_BUFFER_SIZE);

        if (caracter == '\r'){
            continue;
        }

        if (caracter == '\n'){
            if (!descartar && posicion > 0U){
                linea[posicion] = '\0';
                N2_Interpretar(linea);
            }

            posicion = 0;
            descartar = 0;
        }
        else if (!descartar){
            if (posicion < sizeof(linea) - 1U){
                linea[posicion++] = caracter;
            }
            else{
                posicion = 0;
                descartar = 1;

                if (operacionN2 != OPERACION_NINGUNA){
                    Sistema_Finalizar("ERROR: respuesta demasiado larga\r\n");
                }
            }
        }
    }
}

static void N2_Interpretar(const char *linea)
{
    unsigned int id;
    int roll;
    int pitch;
    char extra;
    char detalle[64];

     //Procesar errores primero
     // ID 0 significa que Nucleo 2 no pudo identificar la solicitud porque recibio una trama invalida

    if (sscanf(linea,"ERROR;%u;%63[^\r\n]",&id,detalle) == 2){
        if (id == 0U || (operacionN2 != OPERACION_NINGUNA && id == solicitudN2)){
            char respuesta[128];
            snprintf(respuesta,sizeof(respuesta),"ERROR Nucleo 2: %s\r\n",detalle);

            if (operacionN2 != OPERACION_NINGUNA){
                Sistema_Finalizar(respuesta); // muestra error y liberar la operacion pendiente
            }
            else{
                ESP_Responder(respuesta); // muestra errores de enlace aunque estemos en espera
            }
        }
        return;
    }

    // Las demas respuestas requieren una operacion pendiente.
    if (operacionN2 == OPERACION_NINGUNA){
        return;
    }

     //Nucleo 2 confirma el inicio de la lectura
    if (sscanf(linea, "INICIO;%u%c", &id, &extra) == 1){
        if (id == solicitudN2 && operacionN2 == OPERACION_LEER && !inicioMedicionConfirmado){
            inicioMedicionConfirmado = 1;
            inicioOperacionN2 = HAL_GetTick();
            estado = recibiendo;
        }
        return;
    }

     //Recibir roll y pitch en centesimas de grado
    if (sscanf(linea,"DATOS;%u;%d;%d%c",&id,&roll,&pitch,&extra) == 3){
        if (id != solicitudN2 || operacionN2 != OPERACION_LEER){
            return;
        }

        // Validar los rangos posibles de inclinacion.
        if (roll < -18000 || roll > 18000 || pitch < -9000 || pitch > 9000){
            Sistema_Finalizar("ERROR: angulos recibidos fuera de rango\r\n");
            return;
        }

        int absRoll = roll < 0 ? -roll : roll;
        int absPitch = pitch < 0 ? -pitch : pitch;

        char respuesta[128];

        // Formatear sin necesitar printf con flotantes.
        snprintf(respuesta,sizeof(respuesta),"Roll: %s%d.%02d grados | Pitch: %s%d.%02d grados\r\n",
            roll < 0 ? "-" : "",
            absRoll / 100,
            absRoll % 100,
            pitch < 0 ? "-" : "",
            absPitch / 100,
            absPitch % 100
        );

        uint8_t alerta =
            roll > LIMITE_ANGULO_CDEG ||
            roll < -LIMITE_ANGULO_CDEG ||
            pitch > LIMITE_ANGULO_CDEG ||
            pitch < -LIMITE_ANGULO_CDEG;

        estado = enviando;

        // Enviar los angulos SOLO a la computadora.
        HAL_UART_Transmit(&huart2,(uint8_t *)respuesta,(uint16_t)strlen(respuesta),100);

        // El telefono recibe solamente la confirmacion.
        Sistema_Finalizar("Lectura completada\r\n");

        if (alerta){
            // se reproduce la alarma al terminar el sonido que este en curso.
            alertaPendiente = 1;

            ESP_Responder(
                "ALERTA: inclinacion mayor de 30 grados\r\n"
            );
        }
        return;
    }

     //Nucleo 2 confirma que comenzo la escritura SD
    if (sscanf(linea,"GUARDANDO;%u%c",&id,&extra) == 1){
        if (id == solicitudN2 && operacionN2 == OPERACION_GUARDAR){
            estado = guardando;
            inicioOperacionN2 = HAL_GetTick();
        }
        return;
    }

     // Nucleo 2 confirma que termino el guardado
    if (sscanf(linea,"GUARDADO;%u%c",&id,&extra) == 1){
        if (id == solicitudN2 && operacionN2 == OPERACION_GUARDAR){
            Audio_Reproducir(SONIDO_SaveMedicion);
            Sistema_Finalizar("Medicion guardada en SD\r\n");
        }
        return;
    }
}

// Controlar tiempos sin delay
static void Sistema_Actualizar(void){
    uint32_t ahora = HAL_GetTick();

    if (operacionN2 != OPERACION_NINGUNA){
        if ((uint32_t)(ahora - inicioOperacionN2) >= N2_TIMEOUT_MS){
            Sistema_Finalizar("ERROR: Nucleo 2 no respondio a tiempo\r\n");
        }
    }
    else if (estado == enviando){
        if ((uint32_t)(ahora - inicioIndicacionTX) >= INDICACION_TX_MS){
            estado = encendido;
        }
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
