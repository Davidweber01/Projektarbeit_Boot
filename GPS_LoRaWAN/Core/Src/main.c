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
#define BUFFER_SIZE 		128
#define LoRa_BUFFER_SIZE 	4
#define GPS_BUFFER_SIZE		16
//#define GPS_BUFFER_SIZE 256
#define SEARCH_STRING "$GNRMC"  // Die zu suchende Zeichenkette
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
char msg_tx1[] = "AT+DEUI=70:B3:D5:7E:D8:00:3D:8F\r";
char msg_tx2[] = "AT+APPEUI=58:38:75:E9:A9:FE:9F:A9\r";
char msg_tx3[] = "AT+NWKKEY=D4:67:24:93:1B:8C:89:BC:69:41:FC:EB:17:21:66:1F\r";
char msg_tx4[] = "AT+APPKEY=D4:67:24:93:1B:8C:89:BC:69:41:FC:EB:17:21:66:1F\r";
char msg_tx5[] = "AT+JOIN=1\r";
char msg_tx[] = "AT\r";
uint8_t rxBuffer[LoRa_BUFFER_SIZE];
uint8_t rxBuf;
uint8_t gps_index = 0;
uint8_t receiving = 0;
volatile uint8_t receivedFlag = 0;
volatile uint8_t transferFlag = 0;
volatile uint8_t state = 0;





uint8_t message_buffer[BUFFER_SIZE];  // Puffer für die Nachricht zwischen "$GNRMC" und "*"
uint16_t buffer_index = 0;  // Index, um den aktuellen Pufferort zu verfolgen
uint16_t message_index = 0;  // Index für den Nachrichtenspeicher
uint8_t rx_char;  // Einzelnes Zeichen für den Empfang
uint8_t checksum_phase = 0;  // Speichert, ob wir gerade die Prüfsumme empfangen
uint8_t gps_ack_buffer[GPS_BUFFER_SIZE];
uint8_t gps_config_stat = 0;
uint8_t wait_for_gps_ack = 0;
uint8_t send_ubx_state = 1;

enum config_gps
{
	ACTIVATE_RMC,
	DEACTIVATE_GGA,
	DEACTIVATE_GLL,
	DEACTIVATE_GSA,
	DEACTIVATE_GSV,
	DEACTIVATE_VTG,
	GPS_CONFIGURED
};
static enum config_gps config_gps_state;

volatile uint8_t gps_config_state = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void sendCommandAndWaitForOK(const char *command);
uint8_t checkResponse(const char* strCheck);
void parse_gnrmc(char *data);
void send_ubx_message(UART_HandleTypeDef *huart, uint8_t *msg, uint16_t len);
void Process_GNRMC_Data(uint8_t* data);
void Config_GPS_Modul(void);
void Store_GPS_Data(void);
int GPS_Check_ACK(uint8_t *buffer, int size);
void Config_GPS_Modul_Iteration(uint8_t state);
void convertToHex(const char *input, char *output);
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
  /* USER CODE BEGIN 2 */
/* ============== Activate Power Supply for external Components ===========*/
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
/* ========================================================================*/
  uint8_t atCommand6[] = "AT+JOIN=1\r";


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(state == 7)
	    {
		  HAL_UART_Receive_IT(&huart1, rxBuffer, sizeof(rxBuffer));
	  	  HAL_UART_Transmit(&huart1, atCommand6, sizeof(atCommand6) - 1, 10);
	  	  //HAL_Delay(100);
	  	  //HAL_UART_Receive_IT(&huart1, rxBuffer, sizeof(rxBuffer));
	  	  //HAL_UART_Receive(&huart1, rxBuffer, sizeof(rxBuffer), 4000);
//
	  	  //HAL_Delay(500);
		  HAL_UART_Receive_IT(&huart1, rxBuffer, sizeof(rxBuffer));

	  	  while(state)
	  	  {
		  	  //HAL_UART_Receive_IT(&huart1, rxBuffer, sizeof(rxBuffer));
	  	  }
	    }
//	  switch (state)
//	  {
//	  case 0:
//
//		  break;
//	  case 1:
//		  uint8_t atCommand1[] = "AT+SEND=1:1:ABCD\r";
//		  HAL_UART_Transmit(&huart1, atCommand1, sizeof(atCommand1) - 1, 100);
//		  HAL_UART_Receive_IT(&huart1, rxBuffer, sizeof(rxBuffer));
//		  HAL_Delay(500);
//		  break;
//	  case 2:
//		  uint8_t atCommand2[] = "AT+DEUI=70:B3:D5:7E:D8:00:3D:8F\r";
//		  HAL_UART_Transmit(&huart1, atCommand2, sizeof(atCommand2) - 1, HAL_MAX_DELAY);
//		  HAL_UART_Receive_IT(&huart1, rxBuffer, sizeof(rxBuffer));
//		  HAL_Delay(500);
//		  break;
//	  case 3:
//		  uint8_t atCommand3[] = "AT+APPEUI=58:38:75:E9:A9:FE:9F:A9\r";
//		  HAL_UART_Transmit(&huart1, atCommand3, sizeof(atCommand3) - 1, HAL_MAX_DELAY);
//		  HAL_UART_Receive_IT(&huart1, rxBuffer, sizeof(rxBuffer));
//		  HAL_Delay(500);
//		  break;
//	  case 4:
//		  uint8_t atCommand4[] = "AT+NWKKEY=D4:67:24:93:1B:8C:89:BC:69:41:FC:EB:17:21:66:1F\r";
//		  HAL_UART_Transmit(&huart1, atCommand4, sizeof(atCommand4) - 1, HAL_MAX_DELAY);
//		  HAL_UART_Receive_IT(&huart1, rxBuffer, sizeof(rxBuffer));
//		  HAL_Delay(500);
//		  break;
//	  case 5:
//		  uint8_t atCommand5[] = "AT+APPKEY=D4:67:24:93:1B:8C:89:BC:69:41:FC:EB:17:21:66:1F\r";
//		  HAL_UART_Transmit(&huart1, atCommand5, sizeof(atCommand5) - 1, HAL_MAX_DELAY);
//		  HAL_UART_Receive_IT(&huart1, rxBuffer, sizeof(rxBuffer));
//		  HAL_Delay(500);
//		  break;
//	  case 6:
//		  uint8_t atCommand6[] = "AT+JOIN=1\r";
//		  HAL_UART_Transmit(&huart1, atCommand6, sizeof(atCommand6) - 1, HAL_MAX_DELAY);
//		  HAL_UART_Receive_IT(&huart1, rxBuffer, sizeof(rxBuffer));
//		  HAL_Delay(500);
//		  break;
//	  case 7:
//		  Config_GPS_Modul();
//		  HAL_UART_Receive_IT(&huart3, &rx_char, 1);
////		  break;
//
//	  }

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == /*GPIO_PIN_13*/GPIO_PIN_5)
	{
	    	    state = 7;
	}
}

// USART1: LoRa-Modul
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{

	}
}

// USART3: GPS-Modul
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
//    	printf("State: %d\n", config_gps_state);
//    	Config_GPS_Modul_Iteration(config_gps_state);

		Store_GPS_Data();
        HAL_UART_Receive_IT(huart, &rx_char, 1);
    }

    if (huart->Instance == USART1)
    {
		//memset(rxBuffer, 0, sizeof(rxBuffer));

//    	if(checkResponse("OK") == 1)
//    	{
//    		state = 0;
//    	}
//    	else
//    	{
//    		state = 0;
//    	}
    }
}



void parse_gnrmc(char *data)
{
    char time[10], status[2], latitude[10], lat_dir[2], longitude[11], lon_dir[2], speed[10], date[7];

    sscanf(data, "$GNRMC,%9[^,],%1[^,],%9[^,],%1[^,],%10[^,],%1[^,],%9[^,],,,%6[^,],",
           time, status, latitude, lat_dir, longitude, lon_dir, speed, date);

    printf("Time: %s\n", time);
    printf("Status: %s\n", status);
    printf("Latitude: %s %s\n", latitude, lat_dir);
    printf("Longitude: %s %s\n", longitude, lon_dir);
    printf("Speed: %s\n", speed);
    printf("Date: %s\n", date);
}


void Process_GNRMC_Data(uint8_t* data)
{

    if (strncmp((char*)data, "$GNRMC", 6) == 0)
    {

        printf("GNRMC Data: %s\n", data);
    }
    else
    {

    }
}

uint8_t checkResponse(const char* strCheck)
{
    if (strstr((char *)rxBuffer, strCheck) != NULL)
    {
        return 1;
        memset(rxBuffer, 0, BUFFER_SIZE);
    }
    return 0;
}

void Config_GPS_Modul(void)
{
	uint8_t ubx_cfg_nmea_RMC[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x04, 0x44}; // activate
	uint8_t ubx_cfg_nmea_GGA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x23}; //deactivate
	uint8_t ubx_cfg_nmea_GLL[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A}; //deactivate
	uint8_t ubx_cfg_nmea_GSA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31}; //deactivate
	uint8_t ubx_cfg_nmea_GSV[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38}; //deactivate
	uint8_t ubx_cfg_nmea_VTG[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46}; //deactivate

	//send_ubx_state = 1;
	send_ubx_message(&huart3, ubx_cfg_nmea_RMC, sizeof(ubx_cfg_nmea_RMC));
	send_ubx_message(&huart3, ubx_cfg_nmea_GGA, sizeof(ubx_cfg_nmea_GGA));
	send_ubx_message(&huart3, ubx_cfg_nmea_GLL, sizeof(ubx_cfg_nmea_GLL));
	send_ubx_message(&huart3, ubx_cfg_nmea_GSA, sizeof(ubx_cfg_nmea_GSA));
	send_ubx_message(&huart3, ubx_cfg_nmea_GSV, sizeof(ubx_cfg_nmea_GSV));
	send_ubx_message(&huart3, ubx_cfg_nmea_VTG, sizeof(ubx_cfg_nmea_VTG));


//	while(send_ubx_state)
//	{
//		switch(config_gps_state)
//			{
//			case ACTIVATE_RMC:
//				send_ubx_message(&huart3, ubx_cfg_nmea_RMC, sizeof(ubx_cfg_nmea_RMC));
//				break;
//			case DEACTIVATE_GGA:
//				send_ubx_message(&huart3, ubx_cfg_nmea_GGA, sizeof(ubx_cfg_nmea_GGA));
//				break;
//			case DEACTIVATE_GLL:
//				send_ubx_message(&huart3, ubx_cfg_nmea_GLL, sizeof(ubx_cfg_nmea_GLL));
//				break;
//			case DEACTIVATE_GSA:
//				send_ubx_message(&huart3, ubx_cfg_nmea_GSA, sizeof(ubx_cfg_nmea_GSA));
//				break;
//			case DEACTIVATE_GSV:
//				send_ubx_message(&huart3, ubx_cfg_nmea_GSV, sizeof(ubx_cfg_nmea_GSV));
//				break;
//			case DEACTIVATE_VTG:
//				send_ubx_message(&huart3, ubx_cfg_nmea_VTG, sizeof(ubx_cfg_nmea_VTG));
//				break;
//			case GPS_CONFIGURED:
//				printf("GPS already configured! \n");
//				send_ubx_state = 0;
//				break;
//			default:
//				break;
//
//			}
//			//HAL_Delay(50);
//
//	}


}

void Config_GPS_Modul_Iteration(uint8_t state)
{
	switch (state)
	    	{
	    	case ACTIVATE_RMC:
	    		if(GPS_Check_ACK(gps_ack_buffer, sizeof(gps_ack_buffer)) == 1)
	    		{

	    			printf("ACK erhalten RMC! \n");
	    			memset(gps_ack_buffer, 0, sizeof(gps_ack_buffer));
	    			config_gps_state++;
	    		}
	    		break;
	    	case DEACTIVATE_GGA:
	    		if(GPS_Check_ACK(gps_ack_buffer, sizeof(gps_ack_buffer)) == 1)
				{
	    			config_gps_state++;
					printf("ACK erhalten GGA! \n");
					memset(gps_ack_buffer, 0, sizeof(gps_ack_buffer));
				}
	    		break;
	    	case DEACTIVATE_GLL:
	    		if(GPS_Check_ACK(gps_ack_buffer, sizeof(gps_ack_buffer)) == 1)
				{
	    			config_gps_state++;
					printf("ACK erhalten GLL! \n");
					memset(gps_ack_buffer, 0, sizeof(gps_ack_buffer));
				}
	    		break;
	    	case DEACTIVATE_GSA:
	    		if(GPS_Check_ACK(gps_ack_buffer, sizeof(gps_ack_buffer)) == 1)
				{
	    			config_gps_state++;
					printf("ACK erhalten GSA! \n");
					memset(gps_ack_buffer, 0, sizeof(gps_ack_buffer));
				}
	    		break;
	    	case DEACTIVATE_GSV:
	    		if(GPS_Check_ACK(gps_ack_buffer, sizeof(gps_ack_buffer)) == 1)
				{
	    			config_gps_state++;
					printf("ACK erhalten GSV! \n");
					memset(gps_ack_buffer, 0, sizeof(gps_ack_buffer));
				}
	    		break;
	    	case DEACTIVATE_VTG:
	    		if(GPS_Check_ACK(gps_ack_buffer, sizeof(gps_ack_buffer)) == 1)
				{
	    			config_gps_state++;
					printf("ACK erhalten VTG! \n");
					memset(gps_ack_buffer, 0, sizeof(gps_ack_buffer));
				}
	    		break;
	    	case 6:
	    		printf("GPS already configured! \n");
	    		break;
	    	default:
	    		break;


	    	}


}

void send_ubx_message(UART_HandleTypeDef *huart, uint8_t *msg, uint16_t len)
{
	HAL_UART_Transmit(huart, msg, len, 200);
	//HAL_UART_Receive_IT(&huart3, gps_ack_buffer, sizeof(gps_ack_buffer));
}
void Store_GPS_Data(void)
{
	 if (receiving)
	{
		if (rx_char == '*')
		{

			checksum_phase = 1;
		}
		else if (checksum_phase && message_index < BUFFER_SIZE - 1)
		{

			message_buffer[message_index++] = rx_char;

			if (message_index >= 3 && checksum_phase)
			{
				char hexOutput[sizeof(message_buffer) * 2 + 1];

				message_buffer[message_index] = '\0';
				convertToHex((char *)message_buffer, hexOutput);
				printf("Empfangene Nachricht: %s\n", message_buffer);
				printf("Empfangene Nachricht in HEX: %s\n",  hexOutput);
				memset(message_buffer, 0, BUFFER_SIZE);
				message_index = 0;
				receiving = 0;
				checksum_phase = 0;
			}
		} else if (message_index < BUFFER_SIZE - 1)
		{

			message_buffer[message_index++] = rx_char;
		}
	}
	else if (rx_char == '$')
	{

		message_index = 0;
		receiving = 1;
		checksum_phase = 0;
		memset(message_buffer, 0, BUFFER_SIZE);
		message_buffer[message_index++] = rx_char;
	}
}
int GPS_Check_ACK(uint8_t *buffer, int size)
{
	buffer[size] = '\0';

	for (int i = 0; i < size - 2; i++)
	{
		if (buffer[i] == 0xB5 && buffer[i + 1] == 0x62)
		{
			return 1;
		}
	}
	return 0;

}

void convertToHex(const char *input, char *output)
{
    while (*input) {
        sprintf(output, "%02X", (unsigned char)*input);
        output += 2;
        input++;
    }
    *output = '\0';
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
