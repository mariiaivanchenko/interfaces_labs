/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"
#include "string.h"
#include "parserqueue.h"
#include "ILI9341.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>

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

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t gps_rx_buffer[512];
uint8_t pc_rx_buffer[128];

uint8_t parser_buffer_gps[512];
uint8_t parser_buffer_pc[128];
uint8_t stats_buffer[128];

volatile parser_queue PQ;
volatile parser_queue CQ;

volatile bool tx_busy = false;
uint8_t cli_buf1[512];
uint8_t cli_buf2[512];

uint8_t *buf_1 = cli_buf1;
uint8_t *buf_2 = cli_buf2;

volatile uint16_t len_buf_1 = 0;
volatile uint16_t len_buf_2 = 0;


// statistic variables
volatile bool send_stats = false;
volatile bool send_raw_data = false;

volatile int portions_count = 0;
volatile uint16_t bytes_count = 0;
volatile uint16_t last_postion_size = 0;

volatile uint32_t fe_count = 0;   // framing error
volatile uint32_t ne_count = 0;   // noise error
volatile uint32_t ore_count = 0;  // overrun error
volatile uint32_t pe_count = 0;   // parity error

// parsed values
struct tm utc = {0};
char * lat;
char * lot;
char latV;
char lotV;
char status;
int fix = 0;
int satellite_num = 0;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {

	if (huart->Instance == USART1) {
		// statistic calculations
		portions_count++;
		bytes_count += Size;
		last_postion_size = Size;

		if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_FE)) { fe_count++; __HAL_UART_CLEAR_FEFLAG(&huart1); }
		if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_NE)) { ne_count++; __HAL_UART_CLEAR_NEFLAG(&huart1); }
		if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)){ ore_count++; __HAL_UART_CLEAR_OREFLAG(&huart1); }
		if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_PE)) { pe_count++; __HAL_UART_CLEAR_PEFLAG(&huart1); }

		// copy to parser buffer
		memcpy(parser_buffer_gps, gps_rx_buffer, Size);
		enqueue(&PQ, parser_buffer_gps, Size);
	}

	else if (huart->Instance == USART2) {
		// copy to parser buffer
		memcpy(parser_buffer_pc, pc_rx_buffer, Size);
		enqueue(&CQ, parser_buffer_pc, Size);
	}
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	tx_busy = false;

	if (len_buf_2 > 0) { // buf_1 here is pointing at buf_2
		tx_busy = true;

		// swap buffers
		uint8_t *tmp = buf_1;
		buf_1= buf_2;
		buf_2 = tmp;

		len_buf_1 = len_buf_2;
		len_buf_2 = 0;

		HAL_UART_Transmit_DMA(huart, buf_1, len_buf_1);
	}
}


void send_msg_uart(uint8_t * msg, uint16_t msg_len) {
	memcpy(buf_2, msg, msg_len);
	len_buf_2 = msg_len;

	if (!tx_busy) {
		tx_busy = true;

        uint8_t *tmp = buf_1;
		buf_1 = buf_2;
		buf_2 = tmp;

		len_buf_1 = len_buf_2;
		len_buf_2 = 0;

		HAL_UART_Transmit_DMA(&huart2, buf_1, len_buf_1);
	}
}


bool calculate_checksum(uint8_t *sentence) {
	const char *start = (sentence[0] == '$') ? (const char*)(sentence + 1) : (const char*)sentence;
	const char *star = strchr(start, '*');
	size_t len = (star) ? (size_t)(star - start) : strlen(start);

	int checksum;
	int ascii_values_g[85];
	sscanf(star + 1, "%2X", &checksum);
	ascii_values_g[0] = (int)sentence[1];


	for (int i = 1; i < len; i++) {
		ascii_values_g[i] = ascii_values_g[i-1] ^ (int)sentence[i+1];
	}

	return checksum == ascii_values_g[len - 1];
}


double calculate_degrees(const char *nmea, char direction, bool is_lat) {
    double raw = atof(nmea);
    int degrees;

    if (is_lat) {
		degrees = (int)(raw / 100);
	} else {
		degrees = (int)(raw / 100);
		if (degrees < 100) {
			degrees = (int)(raw / 1000);
		}
	}

    double minutes = raw - (degrees * 100);
    double decimal = degrees + minutes / 60.0;

    if (direction == 'S' || direction == 'W') {
        decimal *= -1;
    }

    return decimal;
}


uint8_t current_portion_gps[512];
uint8_t current_portion_pc[64];
uint8_t return_msg[128];

uint8_t line_gps[128];
uint8_t line_pc[32];
void NMEA_PC_parser() {
//	uint8_t current_portion_gps[512];
//	uint8_t current_portion_pc[128];

//	uint8_t line_gps[128];
//	uint8_t line_pc[64];
//	uint8_t return_msg[128];

	uint16_t start_gps = 0;
	uint16_t start_pc = 0;
	uint16_t current_size_gps = 0;
	uint16_t current_size_pc = 0;

	dequeue(&PQ, current_portion_gps, &current_size_gps);
	dequeue(&CQ, current_portion_pc, &current_size_pc);

	for (uint16_t i = 0; i + 1 <= current_size_gps; i++) {

	    if (current_portion_gps[i] == '\r' && current_portion_gps[i+1] == '\n') {

	        if (i > start_gps) {
	        	uint16_t len = i - start_gps;
	        	if (len >= sizeof(line_gps)) len = sizeof(line_gps) - 1;
	        	memcpy(line_gps, &current_portion_gps[start_gps], len);
	        	line_gps[len] = '\0';


	        	if (line_gps[0] == '$' && current_portion_gps[i+1] == '\n') {
	        		if (calculate_checksum(line_gps)) {
	        			char *token;

	        			if (strstr((char*)line_gps, "RMC") != NULL) {

							token = strtok((char*)line_gps, ",");
							int field_index = 0;

							while (token != NULL && field_index < 7) {

								switch (field_index) {
								    case 1:
								        double UTC = atof(token);
								        int hh = (int)(UTC / 10000);
										int mm = (int)((UTC - hh * 10000) / 100);
										int ss = (int)(UTC - hh * 10000 - mm * 100);

										utc.tm_year = 2025 - 1900;  // tm_year counts from 1900
										utc.tm_mon  = 8;    // tm_mon is 0-based
										utc.tm_mday = 23;
										utc.tm_hour = hh;
										utc.tm_min  = mm;
										utc.tm_sec  = ss;

								        break;

								    case 2:
								        status = token[0];
								        break;

								    case 3:
								        lat = token;
								        break;

								    case 4:
										latV = token[0];
										break;

								    case 5:
								        lot = token;
								        break;
								    case 6:
										lotV = token[0];
										break;

								    default:
								        break;
								}

								token = strtok(NULL, ",");
								field_index++;
							}
						}

						if (strstr((char*)line_gps, "GGA") != NULL) {

							token = strtok((char*)line_gps, ",");
							int field_index = 0;

							while (token != NULL && field_index < 8) {

								switch (field_index) {
									case 6:
										fix = atoi(token);
										break;

									case 7:
										satellite_num = atoi(token);
										break;

									default:
										break;
								}

								token = strtok(NULL, ",");
								field_index++;
							}
						}
					}
	        	}
			}
			start_gps = i + 2;
			i++;
		}
	}


	for (uint16_t i = start_pc; i + 1 <= current_size_pc; i++) {

		if (current_portion_pc[i] == '\r') {
			line_pc[0] = '\0';
			return_msg[0] = '\0';

			if (i > start_pc) {
				uint16_t len = i - start_pc;
				if (len >= sizeof(line_pc)) len = sizeof(line_pc) - 1;
				memcpy(line_pc, &current_portion_pc[start_pc], len);
				line_pc[len] = '\0';

				if (strstr((char*)line_pc, "GET POS") != NULL) {
					double res_lat = calculate_degrees(lat, latV, true);
					double res_lot = calculate_degrees(lot, lotV, false);
					const char *status = (fix == 0) ? "stale": "";

					time_t gps_ms = (utc.tm_hour*3600 + utc.tm_min*60 + utc.tm_sec);
					time_t now_ms = HAL_GetTick();
					double age_ms = (double)(now_ms - gps_ms);

					int len = snprintf((char *) return_msg, sizeof(return_msg),
							"\r\nGET POS\r\nPOS: lat: %.4f, lon: %.4f, fix: %d, sats: %d, age_ms: %.2f, %s\r\n",
							res_lat, res_lot, fix, satellite_num, age_ms, status);

					if (len > 0) {
						send_msg_uart((uint8_t*)return_msg, len);
					}
				}

				if (strstr((char*)line_pc, "GET TIME") != NULL) {
					struct tm local = utc;
					local.tm_hour += 3;
					mktime(&local);

					char formatted_time[32];
					strftime(formatted_time, sizeof(formatted_time), "%Y-%m-%dT%H:%M:%SZ", &local);
					int len = snprintf((char *)return_msg, sizeof(return_msg), "\r\nGET TIME\r\n%s\r\n", formatted_time);

					if (len > 0) {
						send_msg_uart((uint8_t*)return_msg, len);
					}
				}
				if (strstr((char*)line_pc, "RAW OFF") != NULL) {
					send_raw_data = false;
				}

				if (strstr((char*)line_pc, "RAW ON") != NULL) {
					send_raw_data = true;
				}

				if (strstr((char*)line_pc, "STATS ON") != NULL) {
					send_stats = true;
				}

				if (strstr((char*)line_pc, "STATS OFF") != NULL) {
					send_stats = false;
				}

				if (strstr((char*)line_pc, "VER") != NULL) {
				    struct tm local = utc;
				    local.tm_hour += 3;
				    mktime(&local); // normalize the time

				    char formatted_time[32];
				    strftime(formatted_time, sizeof(formatted_time), "%Y-%m-%dT%H:%M:%SZ", &local);

				    int len = snprintf(return_msg, sizeof(return_msg),
				                       "\r\nVER\r\n"
				                       "Firmware: %s\r\n"
				                       "Build: %s\r\n"
				                       "UART: %d baud, parity %s, oversampling %d\r\n",
				                       "1.0.0",         // firmware version
				                       formatted_time,  // build time
				                       115200,          // baud rate
				                       "8N1",           // parity/data bits/stop bits
				                       16);             // oversampling

				    if (len > 0) {
				        send_msg_uart((uint8_t*)return_msg, len);
				    }
				}

			}
//
			start_pc = i;
//			i++;
		}
	}
}

uint32_t get_device_id(void) {
    uint32_t *uid = (uint32_t*)UID_BASE;
    uint32_t id = uid[0] ^ uid[1] ^ uid[2];
    return id;
}

char buf[64];
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, gps_rx_buffer, sizeof(gps_rx_buffer));
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, pc_rx_buffer, sizeof(pc_rx_buffer));

  ILI9341_init(&hspi1);

  Screen ILI9341_Screen;
  Screen_Tiles_init(&ILI9341_Screen);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  struct tm local = utc;
  local.tm_hour += 3;
  ILI9341_fill_screen(&hspi1, &ILI9341_Screen, 0x0000);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  GPIO_PinState state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
//	  if (state == GPIO_PIN_RESET) {
//		  memcpy(message, "Pressed\n", 8);
//		  send_msg_uart(message, 8);
//	  } else {
//		  memcpy(message, "Released\n", 9);
//		  send_msg_uart(message, 9);
//	  }
//	  HAL_Delay(200);


//	  //color fill by tiles
//	  ILI9341_fill_screen(&hspi1, &ILI9341_Screen, 0x0000); // black
//	  HAL_Delay(1000);
//	  ILI9341_fill_screen(&hspi1, &ILI9341_Screen, 0x07E0); // blue
//	  HAL_Delay(1000);
//	  ILI9341_fill_screen(&hspi1, &ILI9341_Screen, 0x001F); // green
//	  HAL_Delay(1000);
//	  ILI9341_fill_screen(&hspi1, &ILI9341_Screen, 0xF800); // red
//	  HAL_Delay(1000);
//
	  char * lat = "49.000";
	  char * lot = "24.000";

	  NMEA_PC_parser();

	  int id = get_device_id();
	  mktime(&local);

	  char formatted_time[32];
	  strftime(formatted_time, sizeof(formatted_time), "%Y-%m-%d %H:%M:%S", &local);

	  snprintf(buf, sizeof(buf), "ID: 0x%d", id);
	  ILI9341_draw_text(&hspi1, &ILI9341_Screen, 220, 10, buf,0xF800);

	  snprintf(buf, sizeof(buf), "LAT: %s", lat);
	  ILI9341_draw_text(&hspi1, &ILI9341_Screen, 220, 30, buf,0xF800);

	  snprintf(buf, sizeof(buf), "LON: %s", lot);
	  ILI9341_draw_text(&hspi1, &ILI9341_Screen, 220, 50, buf,0xF800);

	  snprintf(buf, sizeof(buf), "UTC: %s", formatted_time);
	  ILI9341_draw_text(&hspi1, &ILI9341_Screen, 220, 70, buf,0xF800);

	  local.tm_sec += 1;

	  HAL_Delay(1000);


//
//	if (send_raw_data) {
//		send_msg_uart(gps_rx_buffer, strlen((char *)gps_rx_buffer));
//		HAL_Delay(500);
//	}
//
//    // to see statistics
//    if (send_stats) {
//    	int len = sprintf(stats_buffer, "\r\nportions_count: %d\r\nbytes_count: %d \r\nfe_count: %d \r\nne_count: %d \r\nore_count: %d \r\npe_count: %d \r\n", portions_count, bytes_count, fe_count, ne_count, ore_count, pe_count);
//    	send_msg_uart(stats_buffer, len);
//		HAL_Delay(500);
//    }

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 200;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 5;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  huart1.Init.BaudRate = 9600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |SPI1_CS_Pin|SPI1_D_C_Pin|SPI1_RESET_Pin|Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DATA_Ready_Pin */
  GPIO_InitStruct.Pin = DATA_Ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DATA_Ready_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_Pin INT2_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = INT1_Pin|INT2_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_button_Pin */
  GPIO_InitStruct.Pin = B1_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B1_button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           SPI1_CS_Pin SPI1_D_C_Pin SPI1_RESET_Pin Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |SPI1_CS_Pin|SPI1_D_C_Pin|SPI1_RESET_Pin|Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
