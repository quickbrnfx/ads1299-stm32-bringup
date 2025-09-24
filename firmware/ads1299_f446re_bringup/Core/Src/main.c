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
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */

/*
  ADS1299 streaming path:
  - DRDY (PB0, falling edge) -> HAL_GPIO_EXTI_Callback
  - In EXTI: assert CS (PA4 low), start SPI1 TxRx DMA for 27 bytes
      27 = 3 status + 8 * 3-byte channels (24-bit two's complement)
  - DMA complete -> HAL_SPI_TxRxCpltCallback
      de-assert CS (PA4 high), increase counter
  - Main loop prints once per second

  Rationale:
  - EXTI aligns and reads to ADS1299's data-ready timing.
  - TxRx DMA guarantees SCK clocks.
  - Keep ISRs short; do prints/work in main loop to avoid missed frames.
*/


#include <stdio.h>
#include <string.h>
/* RX buffer holds latest ADS1299 frame; zeros generate clocks */
static uint8_t frame[27];
static uint8_t zeros27[27] = {0};               // clocks SPI, tx dummy

/* ISR-visible flags/counters must be volatile */
static volatile uint8_t spi_busy = 0; // 1 while a DMA transfer is active
static volatile uint32_t drdy_isr_count = 0; // EXTI interrupts per second (sanity)
static volatile uint32_t dma_done_count = 0; // DMA completions per second (sanity)

static inline void CS_L(void){ HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); }
static inline void CS_H(void){ HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  }

// ADS1299 channels are 24-bit two's complement: [MSB .. LSB] = [b23..b0].
// Sign-extend to 32-bit by propagating bit 23 (0x00800000) into the upper byte.
static inline int32_t ads_s24_to_s32(uint8_t msb, uint8_t mid, uint8_t lsb){
  int32_t v = ((int32_t)msb<<16) | ((int32_t)mid<<8) | lsb;
  if (v & 0x00800000) v |= 0xFF000000;
  return v;
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
  MX_GPIO_Init(); //init GPIOs first for no conflict
  MX_DMA_Init();
  MX_SPI1_Init(); //moved SPI init before USART2 to solve DMA issue
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */


  // Known idle
  CS_H();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // START low, no conversion yet

  // Reset pulse  (PB1), keep HIGH during normal operation, active LOW, pulse when ready
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);   HAL_Delay(10);

  // Put ADS1299 into RDATAC (continuous read) mode: send 0x10 with CS low.
  CS_L(); HAL_SPI_Transmit(&hspi1,(uint8_t[]){0x10},1,10); CS_H(); HAL_Delay(1);

  // START (PB2) HIGH, begin conversions
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

  HAL_UART_Transmit(&huart2,(uint8_t*)"EXTI+DMA ready\r\n",16,100);

  // Boot message (confirms we reached here)
   const char *hello = "ADS1299 ready: DRDY->EXTI, SPI1 TxRx DMA, 1Hz prints\r\n";
   HAL_UART_Transmit(&huart2, (uint8_t*)hello, strlen(hello), 100);


  /* USER CODE END 2 */



  /* Infinite loop */

   /* =========================
      Main loop: no polling for DRDY here.
      ISRs (EXTI/DMA) move data; we just summarize once per second.
      ========================= */

  /* USER CODE BEGIN WHILE */
  uint32_t last_ms = 0;
  while (1)
  {
    uint32_t now = HAL_GetTick();
    if (now - last_ms >= 1000) {
      last_ms = now;

      // Parse latest frame into signed 32-bit samples
      int32_t ch[8];
      for (int i=0; i<8; i++){
        int k = 3 + i*3;
        ch[i] = s24_to_s32(frame[k], frame[k+1], frame[k+2]);
      }
      // Print structured, CSV-like one-line summary: easy to log/plot.
      char buf[256];
      int n = snprintf(buf, sizeof(buf),
        "DRDY/s=%lu DMA/s=%lu  S:%02X %02X %02X  "
        "Ch1:%ld Ch2:%ld Ch3:%ld Ch4:%ld Ch5:%ld Ch6:%ld Ch7:%ld Ch8:%ld\r\n",
        (unsigned long)drdy_isr_count, (unsigned long)dma_done_count,
        frame[0], frame[1], frame[2],
        (long)ch[0], (long)ch[1], (long)ch[2], (long)ch[3],
        (long)ch[4], (long)ch[5], (long)ch[6], (long)ch[7]);
      HAL_UART_Transmit(&huart2,(uint8_t*)buf, n, 200);

      // Reset 1-second counters
      drdy_isr_count = 0;
      dma_done_count = 0;
    }
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

/* USER CODE BEGIN 4 */

/* USER CODE BEGIN 4 */

// DRDY falls, EXT1 callback fires
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0)  // PB0 = DRDY
  {
    drdy_isr_count++;
    if (!spi_busy) { //ensure one transfer at a time
      spi_busy = 1;
      CS_L(); //CS low for the whole frame
      // TransmitReceive DMA: send zeros to clock 27 bytes in on MISO
      HAL_SPI_TransmitReceive_DMA(&hspi1, zeros27, frame, sizeof(frame)); //use zeros27 provides 27 clock bytes so we can capture 27 return bytes into frame
    }
  }
}
// DMA finishes, SPI DMA completes callback
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1) {
    CS_H(); // end transaction
    spi_busy = 0; // allow next EXT1 to start a DMA
    dma_done_count++; // 1 Hz check
  }
}


/* USER CODE END 4 */


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
