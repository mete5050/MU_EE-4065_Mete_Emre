/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "baboon100.h"   // #define IMG_W, IMG_H, const uint8_t g_img[]
#include <stdio.h>
#include <string.h>
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

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern UART_HandleTypeDef huart3;   // <<< UART3
#define THR 128
#define IMG_SIZE (IMG_W * IMG_H)

static void uart3_send(const uint8_t *data, uint32_t len) {
    HAL_UART_Transmit(&huart3, (uint8_t *)data, len, HAL_MAX_DELAY);
}

static void send_image_buffer(const uint8_t *buf) {
    char hdr[64];
    int n = snprintf(hdr, sizeof(hdr), "P5\n%d %d\n255\n", IMG_W, IMG_H);
    uart3_send((uint8_t *)hdr, n);

    uint32_t total = IMG_SIZE;
    const uint8_t *p = buf;
    const uint32_t chunk = 256;
    while (total) {
        uint32_t now = (total > chunk) ? chunk : total;
        uart3_send(p, now);
        p += now;
        total -= now;
    }
}

static void img_negative(const uint8_t *src, uint8_t *dst) {
    for (uint32_t i = 0; i < IMG_SIZE; i++)
        dst[i] = 255 - src[i];
}

static void img_threshold(const uint8_t *src, uint8_t *dst, uint8_t thr) {
    for (uint32_t i = 0; i < IMG_SIZE; i++)
        dst[i] = (src[i] >= thr) ? 255 : 0;
}

static uint8_t gamma_lut[256];
static void build_gamma_lut(float gamma_val) {
    for (int i = 0; i < 256; i++) {
        float nr = i / 255.0f;
        float out = powf(nr, gamma_val);
        int v = (int)lroundf(out * 255.0f);
        if (v < 0) v = 0;
        else if (v > 255) v = 255;
        gamma_lut[i] = (uint8_t)v;
    }
}
static void img_gamma(const uint8_t *src, uint8_t *dst, float gamma_val) {
    build_gamma_lut(gamma_val);
    for (uint32_t i = 0; i < IMG_SIZE; i++)
        dst[i] = gamma_lut[src[i]];
}

static inline uint8_t clamp8(int x){ return (x<0)?0:((x>255)?255:(uint8_t)x); }
static void img_piecewise_linear(const uint8_t *src, uint8_t *dst,
                                 uint8_t r1, uint8_t r2,
                                 uint8_t s1, uint8_t s2) {
    for (uint32_t i = 0; i < IMG_SIZE; i++) {
        uint8_t x = src[i];
        int y;
        if (x <= r1) {
            y = (r1 == 0) ? 0 : (x * s1) / r1;
        } else if (x <= r2) {
            y = s1 + (int)(((x - r1) * (s2 - s1)) / (r2 - r1));
        } else {
            y = s2 + (int)(((x - r2) * (255 - s2)) / (255 - r2));
        }
        dst[i] = clamp8(y);
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
  HAL_Delay(3000);
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

      static uint8_t buf[IMG_SIZE];
  /* USER CODE END 2 */
while(1){
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	 //send_image();     // Tüm header’daki resmi gönder
	// a) Negative
	        img_negative(g_img, buf);
	        send_image_buffer(buf);
	        HAL_Delay(5000);  // 5 saniye bekle

	        // b) Threshold
	        img_threshold(g_img, buf, THR);
	        send_image_buffer(buf);
	        HAL_Delay(5000);

	        // c1) Gamma = 3
	        img_gamma(g_img, buf, 3.0f);
	        send_image_buffer(buf);
	        HAL_Delay(5000);

	        // c2) Gamma = 1/3 (yaklaşık 0.333)
	        img_gamma(g_img, buf, 0.333f);
	        send_image_buffer(buf);
	        HAL_Delay(5000);

	        // d) Piecewise linear
	        uint8_t r1 = (THR > 15) ? (THR - 15) : 0;
	        uint8_t r2 = (THR < 240) ? (THR + 15) : 255;
	        uint8_t s1 = 64, s2 = 192;
	        img_piecewise_linear(g_img, buf, r1, r2, s1, s2);
	        send_image_buffer(buf);
	        HAL_Delay(5000);
	    }  // 2 saniye bekle
    /* USER CODE END WHILE */
}
    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
