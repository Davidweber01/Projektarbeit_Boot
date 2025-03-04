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
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 		83
#define LoRa_BUFFER_SIZE 	64

#define WAKE 1					//States Definieren
#define SEND 2
#define JOIN 3
#define SLEEP 4
#define GPS 5
#define FULL_RESET 6

#define OK_ANSWER 1				//States für erwartete Antwort der LoRa Antenne definieren
#define JOIN_ANSWER 2
#define SEND_ANSWER 3


#define SEARCH_STRING "$GNRMC"  // Die zu suchende Zeichenkette in der GPS Nachricht
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
char rxBuffer[LoRa_BUFFER_SIZE]; 		//Puffer für die LoRa-Antwort
uint8_t rxBuf;
uint8_t gps_index = 0;
uint8_t recieving = 0;
uint8_t answerRecieved = 0;				//Umschaltvariable für Antwort der LoRa-Antenne
uint8_t state = WAKE;						//Zustandsvariable
uint8_t stateAnswer = 0;				//Zustandsvariable für die erwartete LoRa Antwort
uint8_t rxByte;							//Variable für aktuell eingelesenes Byte auf dem UART
uint16_t rx_index = 0;					//Variable für den aktuellen Index im Puffer

uint8_t message_buffer[BUFFER_SIZE];  // Puffer für die Nachricht zwischen "$GNRMC" und "*"
uint16_t buffer_index = 0;  // Index, um den aktuellen Pufferort zu verfolgen
uint16_t message_index = 0;  // Index für den Nachrichtenspeicher
uint16_t GpsSize = 0;	//Variable für die größe des GPS Strings
uint8_t rx_char;  // Einzelnes Zeichen für den Empfang
uint8_t checksum_phase = 0;  // Speichert, ob wir gerade die Prüfsumme empfangen
uint8_t GpsActive = 0;		//Umschaltvariable für validität der GPS-Daten
volatile char LoraPayload[sizeof(message_buffer) * 2 + 1];	//Hilfspuffer für die LoRa Payload
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void sendCommandAndWaitForOK(const char *command);
uint8_t checkResponse(char *rxBuffer, const char* strCheck);
void parse_gnrmc(char *data);
void send_ubx_message(UART_HandleTypeDef *huart, uint8_t *msg, uint16_t len);
void Process_GNRMC_Data(uint8_t* data);
void Config_GPS_Modul(void);
void Store_GPS_Data(void);
int GPS_Check_ACK(uint8_t *buffer, int size);
void Config_GPS_Modul_Iteration(uint8_t state);
void convertToHex(const char *input, char *output);
void processLoraMessage(char *rxBuffer);
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
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM5_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
/* ============== Activate Power Supply for external Components ===========*/
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
/* ========================================================================*/
  if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)		//Mit Standby-Flag prüfen ob Programm aus standby erwacht ist
  {
      __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);  // Standby-Flag löschen
      HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);  // RTC Wakeup deaktivieren
      state = WAKE;							//In Zustand "WAKE" gehen
  }



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  switch (state)
	  {
	  case 0:
		  state = WAKE;
		  break;
	  case WAKE:
		  HAL_TIM_Base_Start_IT(&htim5);			//Timer für Errortimeout im Interruptmodus starten
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);	//Stromversorgung der Antennen einschalten
		  HAL_Delay(5000);
		  Config_GPS_Modul();						//GPS-Modul konfigurieren
		  HAL_UART_Receive_IT(&huart3, &rx_char, 1);	//Auf UART3(GPS) zuhören im Interruptmodus
		  answerRecieved = 0;						//Antwortflag reseten
		  state = GPS;								//In GPS Zustand wechseln
		  break;
	  case SEND:
		  if(!answerRecieved){				//Prüfen ob bereits Antwort empfangen wurde
		  uint8_t atCommand1[sizeof(LoraPayload) + 20];		//Command Array definieren
		  snprintf(atCommand1, sizeof(atCommand1), "AT+SEND=1:1:%s\r\n", LoraPayload); //Command Array mit Send-Befehl und Payload füllen
		  memset(LoraPayload, 0, sizeof(LoraPayload));	//Hilfspuffer für Payload reseten
		  stateAnswer = SEND_ANSWER;					//Antwortszustand für Antworten vom AT+SEND command setzen
		  HAL_Delay(1000);
		  HAL_UART_Receive_IT(&huart1, &rxByte, 1);		//An UART1 (LoRa) zuhören im Interruptmodus starten
		  HAL_UART_Transmit(&huart1, atCommand1, 14 + GpsSize * 2 - 1,  HAL_MAX_DELAY);		//Command Array an UART1 (LoRa) senden
		  HAL_Delay(500);
		  }
		  break;
	  case JOIN:
		  HAL_UART_Receive_IT(&huart1, &rxByte, 1);		//An UART1 (LoRa) zuhören im Interruptmodus starten
		  HAL_UART_Receive_IT(&huart3, &rx_char, 1);	//An UART3 (GPS) zuhören im Interruptmodus starten
		  uint8_t atCommand6[] = "AT+JOIN=1\r\n";		//Command array initialisieren
		  if(!answerRecieved){							//Prüfen ob bereits eine Antwort eines vorherigen durchlaufs empfangen wurde um nicht durchgehend wieder zu senden
		  HAL_Delay(1000);
		  stateAnswer = JOIN_ANSWER;					//Antwortszustand für Antworten vom AT+JOIN Command setzen
		  HAL_UART_Receive_IT(&huart1, &rxByte, 1);			//An UART1 (LoRa) zuhören im Interruptmodus starten
		  HAL_UART_Transmit(&huart1, atCommand6, sizeof(atCommand6) - 1, HAL_MAX_DELAY);	//Command Array an UART1 (LoRa) senden
		  HAL_Delay(500);
		  }
		  break;
	  case SLEEP:
		  HAL_TIM_Base_Stop(&htim5);	//Timer für Errortimeout stopen
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);	//Stromversorgung der Antennen abschalten
		  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 61439, RTC_WAKEUPCLOCK_RTCCLK_DIV16);	//RTC Wakeuptimer starten
		  HAL_PWR_EnterSTANDBYMode();				    // Standby-Modus aktivieren
		  break;
	  case GPS:
		  HAL_UART_Receive_IT(&huart3, &rx_char, 1);		//An UART3 (GPS) zuhören im Interruptmodus starten
		  if(GpsActive){			//Prüfen ob valide GPS-Daten empfangen werden
			  state = JOIN;			//Zustand auf JOIN setzen
		  }
		  break;
	  case FULL_RESET:
		  HAL_TIM_Base_Stop(&htim5);		//Timer für Errortimeout stopen
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);	//Stromversorgung der Antennen abschalten
		  answerRecieved = 0;		//Antwortflag reseten
		  GpsActive = 0;			//GPSvalid flag reseten
		  HAL_Delay(5000);
		  state = WAKE;				//Zustand auf WAKE setzen
		  break;
	  }

	  HAL_Delay(500);
	  HAL_UART_Receive_IT(&huart1, &rxByte, 1);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 61439, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 16999;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1199999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
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
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)		//ISR der UARTS
{
    if (huart->Instance == USART3)		//Prüfen ob UART 3 (GPS) ausgelößt wurde
    {
		Store_GPS_Data();				//GPS Daten verarbeiten
        HAL_UART_Receive_IT(&huart3, &rx_char, 1);
    }

    if (huart->Instance == USART1)		//Prüfen ob UART 1 (LoRa) ausgelößt wurde
    {
    	answerRecieved = 1;				//Antwortflag setzen

			if (rxByte == '\n')			//Prüfen ob das aktuelle Byte "\n" ist
			{

				if (rx_index > 0 && rxBuffer[rx_index - 1] == '\r')		//Prüfen ob Index größer 0 und ob das verherige Byte "\r" war
				{

					if (rx_index > 7)		//Prüfen ob mindestens 7 Bytes empfangen wurden um "OK" antworten rauszufiltern
					{

						rxBuffer[rx_index - 1] = '\0';		//Puffer mit \0 abschließen
						answerRecieved = 0;					//Antwortflag reseten
						processLoraMessage(rxBuffer);		//Empfangene Nachricht verarbeiten


					}

					rx_index = 0;	//Index reseten
				}
				else
				{

					if (rx_index < LoRa_BUFFER_SIZE - 1)	//Prüfen ob Index kleiner als Puffergröße ist
					{
						rxBuffer[rx_index++] = rxByte;		//Aktuelles Byte an stelle von Index in Puffer schreiben und Index inkrementieren
					}
					else
					{
						answerRecieved = 0;					//Antwortflag reseten
						rx_index = 0;						//Index reseten
					}
				}
			}
			else
			{

				if (rx_index < LoRa_BUFFER_SIZE - 1)	//Prüfen ob Index kleiner als Puffergröße ist
				{
					rxBuffer[rx_index++] = rxByte;		//Aktuelles Byte an stelle von Index in Puffer schreiben und Index inkrementieren
				}
				else
				{
					answerRecieved = 0;					//Antwortflag reseten
					rx_index = 0;						//Index reseten
				}
			}

			HAL_UART_Receive_IT(&huart1, &rxByte, 1);

		}

}

uint8_t checkResponse(char *rxBuffer, const char* strCheck)
{
    if (strstr(rxBuffer, strCheck) != NULL)		//Prüfen ob strCheck in rxBuffer enthalten ist
    {
        memset(rxBuffer, 0, BUFFER_SIZE);		//rxBuffer reseten
        return 1;
    }
    return 0;
}

void Config_GPS_Modul(void)
{

	//UBX Commands definieren
	uint8_t ubx_cfg_nmea_RMC[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x04, 0x44}; // activate
	uint8_t ubx_cfg_nmea_GGA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x23}; //deactivate
	uint8_t ubx_cfg_nmea_GLL[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A}; //deactivate
	uint8_t ubx_cfg_nmea_GSA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31}; //deactivate
	uint8_t ubx_cfg_nmea_GSV[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38}; //deactivate
	uint8_t ubx_cfg_nmea_VTG[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46}; //deactivate


	//UBX Commands an GPS Senden
	send_ubx_message(&huart3, ubx_cfg_nmea_RMC, sizeof(ubx_cfg_nmea_RMC));
	send_ubx_message(&huart3, ubx_cfg_nmea_GGA, sizeof(ubx_cfg_nmea_GGA));
	send_ubx_message(&huart3, ubx_cfg_nmea_GLL, sizeof(ubx_cfg_nmea_GLL));
	send_ubx_message(&huart3, ubx_cfg_nmea_GSA, sizeof(ubx_cfg_nmea_GSA));
	send_ubx_message(&huart3, ubx_cfg_nmea_GSV, sizeof(ubx_cfg_nmea_GSV));
	send_ubx_message(&huart3, ubx_cfg_nmea_VTG, sizeof(ubx_cfg_nmea_VTG));

}

void send_ubx_message(UART_HandleTypeDef *huart, uint8_t *msg, uint16_t len)
{
	HAL_UART_Transmit(huart, msg, len, 200);	//msg an huart senden
}

void Store_GPS_Data(void)
{
	if (recieving)		//Prüfen ob empfangen wird
	{

		if (rx_char == '*')		//Prüfen ob aktueller Char '*' ist (* schließt RMC Nachricht ab)
		{

			checksum_phase = 1;		//Checksum_phase setzen
		}
		else if (checksum_phase && message_index < BUFFER_SIZE - 1)		//Prüfen ob checksum_phase gesetzt ist und ob der Index kleiner als die Puffergröße ist
		{

			message_buffer[message_index++] = rx_char;	//Aktuellen Char an stelle von Index in Puffer schreiben und Index inkrementieren

			if (message_index >= 3 && checksum_phase)		//Prüfen ob index größergleich 3 und checksum_phase gesetzt ist
			{
				char hexOutput[sizeof(message_buffer) * 2 + 1];		//Puffer für Hex_daten initialisieren

				message_buffer[message_index] = '\0';		//Puffer mit '\0' abschließen
				if(message_buffer[17] == 'A'){				//Prüfen ob aktueller Buffer valide ist
				convertToHex((char *)message_buffer, hexOutput);	//Puffer zu HEX konvertieren
				printf("Empfangene Nachricht: %s\n", message_buffer);
				printf("Empfangene Nachricht in HEX: %s\n",  hexOutput);
				GpsSize = message_index;		//Aktuellen index in GpsSize speichern
				GpsActive = 1;					//GpsActive setzten um empfang valider GPS Daten zu bestätigen
				strcpy(LoraPayload, hexOutput);	//hexOutput in Hilfspuffer LoraPayload kopieren
				}

				memset(message_buffer, 0, BUFFER_SIZE);		//Puffer reseten
				memset(hexOutput, 0, sizeof(hexOutput));	//Hex-Puffer reseten
				message_index = 0;							//INdex reseten
				recieving = 0;								//recieve Flag reseten
				checksum_phase = 0;							//Checksum_phase reseten
			}
		} else if (message_index < BUFFER_SIZE - 1)			//Prüfen ob noch platz im Puffer ist
		{

			message_buffer[message_index++] = rx_char;		//Aktuellen Char an stelle von Index in Puffer schreiben und Index inkrementieren
		}
	}
	else if (rx_char == '$')			//Prüfen ob startzeichen '$' empfangen wurde
	{

		message_index = 0;		//Index reseten
		recieving = 1;			//Recieve flag setzen
		checksum_phase = 0;
		memset(message_buffer, 0, BUFFER_SIZE);	//Puffer reseten
		message_buffer[message_index++] = rx_char;		//Aktuellen Char an stelle von Index in Puffer schreiben und Index inkrementieren
	}
}

void convertToHex(const char *input, char *output)
{
    while (*input) {
        sprintf(output, "%02X", (unsigned char)*input);		//Input stelle zu Hex convertieren und in Output stelle schreiben
        output += 2;		//Pointer auf Output position um 2 inkrementieren
        input++;		//Pointer auf input Stringposition inkrementieren
    }
    *output = '\0';		//Output string mit '\0' abschließen
}

void processLoraMessage(char *rxBuffer)
{

    printf("Empfangene Nachricht: %s\n", rxBuffer);

	switch(stateAnswer)
	{
	case 0:
		break;
	case JOIN_ANSWER:
		if(checkResponse(rxBuffer, "+EVT:JOIN FAILED") == 1)	//Puffer auf string überprüfen
		{
			state = JOIN;		//Zustand auf JOIN setzen
		}
		else if(checkResponse(rxBuffer, "+EVT:JOINED") == 1)	//Puffer auf string überprüfen
		{
			state = SEND;	//Zustand auf SEND setzen

		}
		else
		{
			state = GPS;	//Zustand auf GPS setzen
		}
		break;
	case SEND_ANSWER:
		if(checkResponse(rxBuffer, "AT_BUSY_ERROR") == 1)	//Puffer auf string überprüfen
		{
			HAL_Delay(5000);
			state = SEND;	//Zustand auf SEND setzen
		}
		else if(checkResponse(rxBuffer, "AT_NO_NETWORK_JOINED") == 1)	//Puffer auf string überprüfen
		{
			state = JOIN;	//Zustand auf JOIN setzen
		}
		else if(checkResponse(rxBuffer, "+EVT:SEND_CONFIRMED") == 1)	//Puffer auf string überprüfen
		{
			state = SLEEP;	//Zustand auf SLEEP setzen
		}
		else if(checkResponse(rxBuffer, "AT_PARAM_ERROR") == 1)	//Puffer auf string überprüfen
		{
			state = GPS;	//Zustand auf GPS setzen
		}
		else
		{
			state = SEND;	//Zustand auf SEND setzen
		}
		break;
	}
//	answerRecieved = 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){		//Errortimout ISR
	if(htim ->Instance == TIM5){	//Prüfen ob Timer5 Interrupt ausgelößt hat
		state = FULL_RESET;		//State auf FULL_RESET setzten
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
