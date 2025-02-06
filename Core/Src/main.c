
#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

typedef enum {
    WAIT_START,
    RECEIVE_DATA,
    VALIDATE_CHECKSUM,
    HANDLE_PACKET
} UART_State;


UART_State currentState = WAIT_START;
uint8_t packet[64];        // Buffer to store the UART packet
uint8_t packetIndex = 0;   // Index for the packet buffer


UART_HandleTypeDef huart2;  // UART handle



/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

void UART_Handler(uint8_t receivedByte);
int ValidateChecksum(uint8_t *packet, uint8_t len);
void ProcessPacket(uint8_t *packet, uint8_t len);

int main(void)
{


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
//  char message[] = "Hello, World!\r\n";
//   HAL_UART_Transmit(&huart2, (uint8_t *)message, sizeof(message) - 1, HAL_MAX_DELAY);
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  uint8_t receivedByte;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
     {
         // Try receiving 1 byte
         if (HAL_UART_Receive(&huart2, &receivedByte, 1, HAL_MAX_DELAY) == HAL_OK)
         {
        	 char message[20];  // Buffer to store the message

        	 // Create the message with the received byte
        	 sprintf(message, "Received byte: 0x%02X\r\n", receivedByte);

        	 // Send the message via UART
        	 HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
         }
     }
  /* USER CODE END 3 */
}


void UART_Handler(uint8_t receivedByte)
{
    // Print the received byte and the current state
    printf("Received byte: 0x%02X, Current state: %d\n", receivedByte, currentState);  // Debug print

    switch (currentState) {
        case WAIT_START:
            if (receivedByte == 0xAA) {
                currentState = RECEIVE_DATA;
                packetIndex = 0;  // Reset packet index
                char message[50];
                sprintf(message, "Transitioned to WAIT_START -> RECEIVE_DATA state\n");
                HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);  // Send via UART
            }
            break;

        case RECEIVE_DATA:
            packet[packetIndex++] = receivedByte;
            printf("Received data byte: 0x%02X\n", receivedByte);  // Debug print
            if (packetIndex >= 64 || receivedByte == 0x55) {
                currentState = VALIDATE_CHECKSUM;
                char message[50];
                sprintf(message, "Transitioned to RECEIVE_DATA -> VALIDATE_CHECKSUM state\n");
                HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);  // Send via UART
            }
            break;

        case VALIDATE_CHECKSUM:
            if (ValidateChecksum(packet, packetIndex)) {
                currentState = HANDLE_PACKET;
                char message[50];
                sprintf(message, "Transitioned to VALIDATE_CHECKSUM -> HANDLE_PACKET state\n");
                HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);  // Send via UART
            } else {
                currentState = WAIT_START;
                char message[50];
                sprintf(message, "Transitioned to VALIDATE_CHECKSUM -> WAIT_START state\n");
                HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);  // Send via UART
            }
            break;

        case HANDLE_PACKET:
            ProcessPacket(packet, packetIndex);
            currentState = WAIT_START;
            char message[50];
            sprintf(message, "Transitioned to HANDLE_PACKET -> WAIT_START state\n");
            HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);  // Send via UART
            break;

        default:
            currentState = WAIT_START;
            break;
    }
}

int ValidateChecksum(uint8_t *packet, uint8_t len)
{
    uint8_t checksum = 0;
    for (int i = 0; i < len; i++) {
        checksum += packet[i];
    }
    return (checksum % 256 == 0);  // Assume checksum is valid if sum is divisible by 256
}
// Process the packet (e.g., store it, send over network, etc.)
void ProcessPacket(uint8_t *packet, uint8_t len)
{
    // Placeholder for actual packet processing logic
    // This could be where you process the packet for further action
    // For example, store or forward the packet
    // Here, you might add logic to log, store, or transmit the packet
    // Example: Print the packet contents over UART
    for (int i = 0; i < len; i++) {
        printf("Packet byte %d: 0x%02X\n", i, packet[i]);
    }
}

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{


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
