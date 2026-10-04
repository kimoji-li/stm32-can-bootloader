/**
  ******************************************************************************
  * @file    main.c
  * @brief   APP v1.0 - linked at 0x08005000
  ******************************************************************************
  */

#include "main.h"
#include <stdio.h>

UART_HandleTypeDef huart1;
CAN_HandleTypeDef  hcan;        /* ????:? stm32f1xx_it.c ?? */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

int main(void)
{
    uint32_t tick = 0;

    /* ??? ??:???????? APP ????? ??? */
    SCB->VTOR = 0x08005000UL;

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();

    printf("\r\n\r\n");
    printf("================================\r\n");
    printf("   >>>  APP V2.0  RUNNING  <<<\r\n");
    printf("================================\r\n");
    printf("This is VERSION 2.0 at 0x08005000\r\n");
    printf("SYSCLK = %lu Hz\r\n\r\n", (unsigned long)HAL_RCC_GetHCLKFreq());

    while (1)
    {
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        printf("[APP V2.0] tick #%lu\r\n", (unsigned long)tick++);
        HAL_Delay(50);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState        = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue  = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState        = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState    = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL      = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    GPIO_InitStruct.Pin   = GPIO_PIN_13;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}