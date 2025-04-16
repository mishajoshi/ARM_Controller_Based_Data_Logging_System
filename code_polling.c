/* USER CODE BEGIN Header */
/**
 * @file : main.c
 * @brief : Main program body - Polling-based Data Logging System
 * 
 * This program implements a data logging system using an STM32F070RB microcontroller.
 * It samples analog data through ADC, processes it, and transmits it through UART.
 * The sampling rate can be configured by the user through serial commands.
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

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
ADC_HandleTypeDef hadc;
TIM_HandleTypeDef htim16;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t timer_val;                 // Stores the timer value for interval timing
uint8_t rx_buffer[6];               // Buffer to store received UART commands
double value = 0.0;                 // Stores the converted ADC value
double clr = 0.0;                   // Used for clearing the output
uint8_t msg[10] = {'\0'};           // Buffer for formatting output messages
uint8_t tx_data[67] = "Enter the sampling rate: \n\r a:10ms\n\r b:100ms\n\r c:500ms\n\r d:1000ms\n\r"; // Menu options
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM16_Init(void);

/* USER CODE BEGIN PFP */
void adc_output(void);
void end_logging(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief  Transmits the ADC value over UART
 * @param  None
 * @retval None
 */
void adc_output(void) 
{
    // Format the ADC value as a string
    sprintf(msg, "%f\r\n", value);
    
    // Transmit the formatted string over UART
    HAL_UART_Transmit(&huart2, msg, 10, 2);
    
    // Update the timer value for the next interval
    timer_val = __HAL_TIM_GET_COUNTER(&htim16);
}

/**
 * @brief  Stops the data logging process
 * @param  None
 * @retval None
 */
void end_logging(void)
{
    // Clear the command buffer
    rx_buffer[0] = 0;
    rx_buffer[1] = 0;
    rx_buffer[2] = 0;
    
    // Send a clear value to indicate logging has stopped
    sprintf(msg, "%f\r\n", clr);
    HAL_UART_Transmit(&huart2, msg, 10, 2);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point
 * @retval int
 */
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_ADC_Init();
    MX_TIM16_Init();

    /* Start the timer */
    HAL_TIM_Base_Start(&htim16);
    timer_val = __HAL_TIM_GET_COUNTER(&htim16);

    /* Infinite loop */
    while (1) {
        // Start ADC conversion
        HAL_ADC_Start(&hadc);
        
        // Wait for conversion to complete (polling method)
        HAL_ADC_PollForConversion(&hadc, 100);
        
        // Get the ADC value and scale it to voltage (0-3.3V)
        value = (double)HAL_ADC_GetValue(&hadc);
        value = value * (3.3/4096);
        
        // Receive commands from UART
        HAL_UART_Receive_IT(&huart2, rx_buffer, sizeof(rx_buffer));
        
        // Check if data logging command 'R' is received
        if (rx_buffer[0] == 'R') {
            // Check for sampling rate selection
            if (rx_buffer[1] == 'a') {
                // 10ms sampling rate
                while (__HAL_TIM_GET_COUNTER(&htim16) - timer_val >= 10) {
                    adc_output();
                }
                // Check for stop command
                if (rx_buffer[2] == 'S') {
                    end_logging();
                }
            }
            else if (rx_buffer[1] == 'b') {
                // 100ms sampling rate
                while (__HAL_TIM_GET_COUNTER(&htim16) - timer_val >= 100) {
                    adc_output();
                }
                if (rx_buffer[2] == 'S') {
                    end_logging();
                }
            }
            else if (rx_buffer[1] == 'c') {
                // 500ms sampling rate
                while (__HAL_TIM_GET_COUNTER(&htim16) - timer_val >= 500) {
                    adc_output();
                }
                if (rx_buffer[2] == 'S') {
                    end_logging();
                }
            }
            else if (rx_buffer[1] == 'd') {
                // 1000ms sampling rate
                while (__HAL_TIM_GET_COUNTER(&htim16) - timer_val >= 1000) {
                    adc_output();
                }
                if (rx_buffer[2] == 'S') {
                    end_logging();
                }
            }
            else {
                // Default case - 1000ms sampling rate
                while (__HAL_TIM_GET_COUNTER(&htim16) - timer_val >= 1000) {
                    adc_output();
                }
                if (rx_buffer[2] == 'S') {
                    end_logging();
                }
            }
        }
        else {
            // Display menu options if no valid command is received
            HAL_UART_Transmit_IT(&huart2, tx_data, 67);
            HAL_Delay(1000);
        }
    }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    
    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI14;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.HSI14CalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    
    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and
    number of conversion)
    */
    hadc.Instance = ADC1;
    hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc.Init.Resolution = ADC_RESOLUTION_12B;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
    hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc.Init.LowPowerAutoWait = DISABLE;
    hadc.Init.LowPowerAutoPowerOff = DISABLE;
    hadc.Init.ContinuousConvMode = DISABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.DMAContinuousRequests = DISABLE;
    hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    if (HAL_ADC_Init(&hadc) != HAL_OK)
    {
        Error_Handler();
    }
    
    /** Configure for the selected ADC regular channel to be converted.
    */
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief TIM16 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM16_Init(void)
{
    htim16.Instance = TIM16;
    htim16.Init.Prescaler = 48000-1;
    htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim16.Init.Period = 65536-1;
    htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim16.Init.RepetitionCounter = 0;
    htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 38400;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    
    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
    
    /*Configure GPIO pin : LD2_Pin */
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/**
 * @brief This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
