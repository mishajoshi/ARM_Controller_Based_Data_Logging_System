/* USER CODE BEGIN Header */
/**
 * @file : main.c
 * @brief : Main program body - Interrupt-based Data Logging System
 * 
 * This program implements a data logging system using an STM32F070RB microcontroller.
 * It samples analog data through ADC, processes it, and transmits it through UART.
 * The sampling rate can be configured by the user through serial commands.
 * This version uses interrupts instead of polling for more efficient operation.
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
uint8_t rx_buffer[6] = {0};         // Buffer to store received UART commands
double value = 0.0;                 // Stores the converted ADC value
double clr = 0.0;                   // Used for clearing the output
uint8_t msg[10] = {'\0'};           // Buffer for formatting output messages
uint8_t tx_data[67] = "Enter the sampling rate: \n\r a:10ms\n\r b:100ms\n\r c:500ms\n\r d:1000ms\n\r"; // Menu options
uint8_t logging_active = 0;         // Flag to indicate if logging is active
uint8_t command_received = 0;       // Flag to indicate if a command has been received
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
void configure_timer_period(uint8_t rate_option);
void process_command(void);
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
}

/**
 * @brief  Stops the data logging process
 * @param  None
 * @retval None
 */
void end_logging(void)
{
    // Stop the timer interrupt
    HAL_TIM_Base_Stop_IT(&htim16);
    
    // Clear the logging flag
    logging_active = 0;
    
    // Send a clear value to indicate logging has stopped
    sprintf(msg, "%f\r\n", clr);
    HAL_UART_Transmit(&huart2, msg, 10, 2);
    
    // Clear the command buffer
    rx_buffer[0] = 0;
    rx_buffer[1] = 0;
    rx_buffer[2] = 0;
}

/**
 * @brief  Configure timer period based on the selected sampling rate
 * @param  rate_option: The selected rate option (a, b, c, or d)
 * @retval None
 */
void configure_timer_period(uint8_t rate_option)
{
    // Stop the timer first
    HAL_TIM_Base_Stop_IT(&htim16);
    
    // Configure timer period based on sampling rate
    switch (rate_option) {
        case 'a': // 10ms
            htim16.Init.Period = 10 - 1;
            break;
        case 'b': // 100ms
            htim16.Init.Period = 100 - 1;
            break;
        case 'c': // 500ms
            htim16.Init.Period = 500 - 1;
            break;
        case 'd': // 1000ms
        default:
            htim16.Init.Period = 1000 - 1;
            break;
    }
    
    // Initialize the timer with new period
    HAL_TIM_Base_Init(&htim16);
    
    // Start the timer with interrupt
    HAL_TIM_Base_Start_IT(&htim16);
    
    // Set the logging flag
    logging_active = 1;
}

/**
 * @brief  Process received UART command
 * @param  None
 * @retval None
 */
void process_command(void)
{
    // Check if data logging command 'R' is received
    if (rx_buffer[0] == 'R') {
        configure_timer_period(rx_buffer[1]);
    }
    // Check if stop command 'S' is received
    else if (rx_buffer[2] == 'S') {
        end_logging();
    }
    
    // Reset the command received flag
    command_received = 0;
}

/**
 * @brief  UART Rx Transfer completed callback
 * @param  huart: UART handle
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        // Set the command received flag
        command_received = 1;
    }
}

/**
 * @brief  ADC conversion complete callback
 * @param  hadc: ADC handle
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // Get the ADC value and scale it to voltage (0-3.3V)
    value = (double)HAL_ADC_GetValue(hadc);
    value = value * (3.3/4096);
}

/**
 * @brief  Timer period elapsed callback
 * @param  htim: Timer handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM16) {
        if (logging_active) {
            // Start ADC conversion
            HAL_ADC_Start_IT(&hadc);
            
            // Output the ADC value
            adc_output();
        }
    }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point
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
    MX_ADC_Init();
    MX_TIM16_Init();
    
    /* USER CODE BEGIN 2 */
    // Start UART reception in interrupt mode
    HAL_UART_Receive_IT(&huart2, rx_buffer, sizeof(rx_buffer));
    /* USER CODE END 2 */
    
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */
        
        /* USER CODE BEGIN 3 */
        // Check if a command has been received
        if (command_received) {
            process_command();
        }
        
        // If logging is not active, display the menu
        if (!logging_active) {
            HAL_UART_Transmit(&huart2, tx_data, 67, 100);
            HAL_Delay(1000);
        }
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
    /* USER CODE BEGIN ADC_Init 0 */
    /* USER CODE END ADC_Init 0 */
    
    ADC_ChannelConfTypeDef sConfig = {0};
    
    /* USER CODE BEGIN ADC_Init 1 */
    /* USER CODE END ADC_Init 1 */
    
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
    /* USER CODE BEGIN ADC_Init 2 */
    /* USER CODE END ADC_Init 2 */
}

/**
 * @brief TIM16 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM16_Init(void)
{
    /* USER CODE BEGIN TIM16_Init 0 */
    /* USER CODE END TIM16_Init 0 */
    
    /* USER CODE BEGIN TIM16_Init 1 */
    /* USER CODE END TIM16_Init 1 */
    htim16.Instance = TIM16;
    htim16.Init.Prescaler = 48000-1;
    htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim16.Init.Period = 1000-1;  // Default to 1000ms
    htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim16.Init.RepetitionCounter = 0;
    htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM16_Init 2 */
    /* USER CODE END TIM16_Init 2 */
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
    
    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
 * @brief This function is executed in case of error occurrence.
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
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
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
