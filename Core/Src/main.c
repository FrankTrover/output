/* USER CODE BEGIN Header */
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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "delay.h"
#include "ad7616.h"
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_CHANNEL_COUNT 8U
#define APP_ADC_RANGE_MV 5000L
#define APP_ALERT_THRESHOLD 30000U
#define APP_LCD_REFRESH_MS 100U
#define APP_LED_TOGGLE_MS 250U

#define APP_LED_RED_ON() HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET)
#define APP_LED_RED_OFF() HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET)
#define APP_LED_BLUE_ON() HAL_GPIO_WritePin(LED_BULE_GPIO_Port, LED_BULE_Pin, GPIO_PIN_RESET)
#define APP_LED_BLUE_OFF() HAL_GPIO_WritePin(LED_BULE_GPIO_Port, LED_BULE_Pin, GPIO_PIN_SET)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static int16_t g_ad7616_a[APP_CHANNEL_COUNT];
static int16_t g_ad7616_b[APP_CHANNEL_COUNT];
static uint16_t g_peak_abs_code = 0U;
static uint32_t g_last_lcd_update_ms = 0U;
static uint32_t g_last_led_toggle_ms = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint16_t App_AbsCode(int16_t sample);
static int32_t App_CodeToMilliVolt(int16_t sample);
static void App_FormatMilliVolt(int32_t milli_volt, char *buffer);
static void App_ScanAd7616(void);
static void App_UpdateAlertLed(void);
static void App_InitLcd(void);
static void App_UpdateLcd(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint16_t App_AbsCode(int16_t sample)
{
  if (sample >= 0)
  {
    return (uint16_t)sample;
  }

  if (sample == INT16_MIN)
  {
    return 32768U;
  }

  return (uint16_t)(-sample);
}

static int32_t App_CodeToMilliVolt(int16_t sample)
{
  int32_t scaled = (int32_t)sample * APP_ADC_RANGE_MV;

  if (scaled >= 0)
  {
    scaled += 16384L;
  }
  else
  {
    scaled -= 16384L;
  }

  return scaled / 32768L;
}

static void App_FormatMilliVolt(int32_t milli_volt, char *buffer)
{
  uint32_t abs_mv;

  if (milli_volt < 0)
  {
    *buffer++ = '-';
    abs_mv = (uint32_t)(-milli_volt);
  }
  else
  {
    *buffer++ = '+';
    abs_mv = (uint32_t)milli_volt;
  }

  *buffer++ = (char)('0' + ((abs_mv / 1000U) % 10U));
  *buffer++ = '.';
  *buffer++ = (char)('0' + ((abs_mv / 100U) % 10U));
  *buffer++ = (char)('0' + ((abs_mv / 10U) % 10U));
  *buffer++ = (char)('0' + (abs_mv % 10U));
  *buffer = '\0';
}

static void App_UpdatePeakPair(uint16_t *peak, int16_t sample_a, int16_t sample_b)
{
  uint16_t abs_a = App_AbsCode(sample_a);
  uint16_t abs_b = App_AbsCode(sample_b);

  if (abs_a > *peak)
  {
    *peak = abs_a;
  }

  if (abs_b > *peak)
  {
    *peak = abs_b;
  }
}

static void App_ScanAd7616(void)
{
  uint8_t channel;
  uint16_t peak = 0U;
  int16_t sample_a = 0;
  int16_t sample_b = 0;

  /*
   * AD7616 并行切换通道后，结果会在下一次转换后读出。
   * 先用通道对 0 做一次预充转换，然后每次切到下一对通道并回填当前对的结果。
   */
  AD7616_Parallel_Channel_Select(0U);
  AD7616_Conversion();
  if (AD7616_Read_Data_Timeout(&sample_a, &sample_b, AD7616_BUSY_TIMEOUT_US) != HAL_OK)
  {
    Error_Handler();
  }

  for (channel = 0U; channel < APP_CHANNEL_COUNT; ++channel)
  {
    uint8_t next_channel = (uint8_t)((channel + 1U) & 0x07U);

    AD7616_Parallel_Channel_Select(next_channel);
    AD7616_Conversion();
    if (AD7616_Read_Data_Timeout(&sample_a, &sample_b, AD7616_BUSY_TIMEOUT_US) != HAL_OK)
    {
      Error_Handler();
    }

    g_ad7616_a[channel] = sample_a;
    g_ad7616_b[channel] = sample_b;
    App_UpdatePeakPair(&peak, sample_a, sample_b);
  }

  g_peak_abs_code = peak;
}
//伏笔了这波，有点意思
//大卫拉亚的示例有点东西
//lihai
//固

static void App_UpdateAlertLed(void)
{
  if (g_peak_abs_code >= APP_ALERT_THRESHOLD)
  {
    APP_LED_RED_ON();
  }
  else
  {
    APP_LED_RED_OFF();
  }
}

static void App_InitLcd(void)
{
  LCD_Init();
  LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
  LCD_ShowString(0, 0, (const u8 *)"PAIR  A(V)   B(V)", CYAN, BLACK, 12, 0);
}

static void App_UpdateLcd(void)
{
  uint8_t channel;

  for (channel = 0U; channel < APP_CHANNEL_COUNT; ++channel)
  {
    char a_text[8];
    char b_text[8];
    char line_text[20];
    uint16_t row_y = (uint16_t)(16U + ((uint16_t)channel * 18U));

    App_FormatMilliVolt(App_CodeToMilliVolt(g_ad7616_a[channel]), a_text);
    App_FormatMilliVolt(App_CodeToMilliVolt(g_ad7616_b[channel]), b_text);

    line_text[0] = '0' + (char)channel;
    line_text[1] = ':';
    line_text[2] = ' ';
    line_text[3] = a_text[0];
    line_text[4] = a_text[1];
    line_text[5] = a_text[2];
    line_text[6] = a_text[3];
    line_text[7] = a_text[4];
    line_text[8] = a_text[5];
    line_text[9] = ' ';
    line_text[10] = b_text[0];
    line_text[11] = b_text[1];
    line_text[12] = b_text[2];
    line_text[13] = b_text[3];
    line_text[14] = b_text[4];
    line_text[15] = b_text[5];
    line_text[16] = '\0';

    LCD_ShowString(0, row_y, (const u8 *)line_text, WHITE, BLACK, 12, 0);
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
  delay_init(HAL_RCC_GetSysClockFreq());
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  APP_LED_BLUE_OFF();
  APP_LED_RED_OFF();

  App_InitLcd();

  AD7616_Init(HARDWARE_MODE);
  AD7616_Parallel_Set_voltage(Range_5_V);
  AD7616_Parallel_Channel_Select(0U);
  AD7616_Reset();
  App_ScanAd7616();
  App_UpdateLcd();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now_ms;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    App_ScanAd7616();
    now_ms = HAL_GetTick();

    if ((now_ms - g_last_lcd_update_ms) >= APP_LCD_REFRESH_MS)
    {
      App_UpdateLcd();
      g_last_lcd_update_ms = now_ms;
    }

    if ((now_ms - g_last_led_toggle_ms) >= APP_LED_TOGGLE_MS)
    {
      HAL_GPIO_TogglePin(LED_BULE_GPIO_Port, LED_BULE_Pin);
      g_last_led_toggle_ms = now_ms;
    }

    App_UpdateAlertLed();
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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
    HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
    delay_ms(100);
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
