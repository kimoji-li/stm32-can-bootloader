/**
  ******************************************************************************
  * @file    main.c
  * @brief   CAN Bootloader - Stage 7
  *          - CAN protocol : CMD_START / CMD_ERASE / DATA / CMD_FINISH + ACK
  *          - erase APP area, write firmware to flash (half-word + read back)
  *          - CRC32 (MPEG-2) integrity check on CMD_FINISH
  *          - metadata page holds magic / fw_size / fw_crc
  *          - boot: validate APP via metadata + CRC, 3s upgrade window, jump
  *
  * Flash layout (128KB device):
  *   0x08000000 - 0x08004FFF   bootloader    20 KB
  *   0x08005000 - 0x0801FBFF   application  107 KB
  *   0x0801FC00 - 0x0801FFFF   metadata       1 KB
  ******************************************************************************
  */

#include "main.h"
#include <stdio.h>
#include <string.h>

/* ======================== Protocol ======================== */
#define CAN_ID_CMD        0x100U
#define CAN_ID_DATA       0x101U
#define CAN_ID_ACK        0x102U

#define CMD_START         0x01U
#define CMD_ERASE         0x02U
#define CMD_FINISH        0x03U

#define ST_OK             0x00U
#define ST_ERR_SEQ        0x01U
#define ST_ERR_CMD        0x02U
#define ST_ERR_FLASH      0x03U
#define ST_ERR_CRC        0x04U
#define ST_ERR_SIZE       0x05U

/* ======================== Flash layout ======================== */
#define APP_ADDRESS            0x08005000UL
#define APP_PAGE_START         20U
#define APP_PAGE_END           127U          /* pages 20..126 = 107 KB */
#define BOOT_FLASH_PAGE_SIZE   1024U
#define APP_MAX_SIZE           ((APP_PAGE_END - APP_PAGE_START) * BOOT_FLASH_PAGE_SIZE)

#define META_ADDRESS           0x0801FC00UL  /* last 1 KB page */
#define META_PAGE              127U
#define META_MAGIC             0xA5A5A5A5UL

#define DATA_PER_FRAME         6U
#define DATA_OFFSET(seq)       ((uint32_t)(seq) * DATA_PER_FRAME)

/* ======================== Types ======================== */
typedef struct {
    uint32_t magic;
    uint32_t fw_size;
    uint32_t fw_crc;
    uint32_t reserved;
} AppMeta_t;

typedef void (*pFunction)(void);

/* ======================== Globals ======================== */
CAN_HandleTypeDef  hcan;
UART_HandleTypeDef huart1;

volatile uint16_t g_expected_seq  = 0;
volatile uint16_t g_total_seq     = 0;
volatile uint8_t  g_prog_error    = 0;
volatile uint8_t  g_jump_request  = 0;
volatile uint8_t  g_start_request = 0;

volatile uint32_t g_expected_crc = 0;
volatile uint8_t  g_crc_valid    = 0;

/* ======================== Prototypes ======================== */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN1_Init(void);
static uint32_t crc32_mpeg(const uint8_t *data, uint32_t len);

/* ======================== printf redirect ======================== */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

/* ======================== ACK ======================== */
static void can_send_ack(uint8_t status, uint16_t expect_seq)
{
    CAN_TxHeaderTypeDef tx;
    uint8_t  buf[3];
    uint32_t mailbox;

    tx.StdId              = CAN_ID_ACK;
    tx.ExtId              = 0;
    tx.IDE                = CAN_ID_STD;
    tx.RTR                = CAN_RTR_DATA;
    tx.DLC                = 3;
    tx.TransmitGlobalTime = DISABLE;

    buf[0] = status;
    buf[1] = (uint8_t)(expect_seq & 0xFF);
    buf[2] = (uint8_t)(expect_seq >> 8);

    HAL_CAN_AddTxMessage(&hcan, &tx, buf, &mailbox);
}

/* ======================== Flash: erase APP area ======================== */
static uint8_t flash_erase_app(void)
{
    FLASH_EraseInitTypeDef e;
    uint32_t err = 0;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR);

    e.TypeErase   = FLASH_TYPEERASE_PAGES;
    e.PageAddress = APP_ADDRESS;
    e.NbPages     = (APP_PAGE_END - APP_PAGE_START);

    if (HAL_FLASHEx_Erase(&e, &err) != HAL_OK) {
        HAL_FLASH_Lock();
        return ST_ERR_FLASH;
    }
    HAL_FLASH_Lock();
    return ST_OK;
}

/* ======================== Flash: write half-words + read back ======================== */
static uint8_t flash_write(uint32_t offset, const uint8_t *data, uint32_t len)
{
    uint32_t addr = APP_ADDRESS + offset;
    uint32_t i;
    uint16_t half;

    if ((offset + len) > APP_MAX_SIZE) {
        return ST_ERR_SIZE;
    }

    HAL_FLASH_Unlock();
    for (i = 0; i < len; i += 2) {
        half = data[i];
        if ((i + 1) < len) {
            half |= (uint16_t)(data[i + 1] << 8);
        }
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, half) != HAL_OK) {
            HAL_FLASH_Lock();
            return ST_ERR_FLASH;
        }
        if (*(volatile uint16_t *)addr != half) {
            HAL_FLASH_Lock();
            return ST_ERR_FLASH;
        }
        addr += 2;
    }
    HAL_FLASH_Lock();
    return ST_OK;
}

/* ======================== CRC32 (MPEG-2, same as STM32 hardware CRC) ======================== */
static uint32_t crc32_mpeg(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i;
    uint8_t  b;

    for (i = 0; i < len; i++) {
        crc ^= ((uint32_t)data[i] << 24);
        for (b = 0; b < 8; b++) {
            if (crc & 0x80000000UL) {
                crc = (crc << 1) ^ 0x04C11DB7UL;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

/* ======================== metadata page ======================== */
static uint8_t meta_write(uint32_t fw_size, uint32_t fw_crc)
{
    FLASH_EraseInitTypeDef e;
    uint32_t err  = 0;
    uint32_t addr = META_ADDRESS;
    uint32_t words[4];
    uint32_t i;
    uint32_t w;

    words[0] = META_MAGIC;
    words[1] = fw_size;
    words[2] = fw_crc;
    words[3] = 0;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR);

    e.TypeErase   = FLASH_TYPEERASE_PAGES;
    e.PageAddress = META_ADDRESS;
    e.NbPages     = 1;
    if (HAL_FLASHEx_Erase(&e, &err) != HAL_OK) {
        printf("[META] erase failed\r\n");
        HAL_FLASH_Lock();
        return ST_ERR_FLASH;
    }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR);

    for (i = 0; i < 4; i++) {
        w = words[i];
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr,
                              (uint16_t)(w & 0xFFFFUL)) != HAL_OK) {
            HAL_FLASH_Lock();
            return ST_ERR_FLASH;
        }
        addr += 2;
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr,
                              (uint16_t)((w >> 16) & 0xFFFFUL)) != HAL_OK) {
            HAL_FLASH_Lock();
            return ST_ERR_FLASH;
        }
        addr += 2;
    }

    HAL_FLASH_Lock();
    return ST_OK;
}

static uint8_t meta_erase(void)
{
    FLASH_EraseInitTypeDef e;
    uint32_t err = 0;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR);

    e.TypeErase   = FLASH_TYPEERASE_PAGES;
    e.PageAddress = META_ADDRESS;
    e.NbPages     = 1;
    if (HAL_FLASHEx_Erase(&e, &err) != HAL_OK) {
        HAL_FLASH_Lock();
        return ST_ERR_FLASH;
    }
    HAL_FLASH_Lock();
    return ST_OK;
}

/* ======================== APP validity: metadata + CRC ======================== */
static uint8_t app_is_valid(void)
{
    const AppMeta_t *m = (const AppMeta_t *)META_ADDRESS;
    uint32_t sp;
    uint32_t pc;
    uint32_t crc;

    printf("[BOOT] meta: magic=0x%08lX size=%lu crc=0x%08lX\r\n",
           (unsigned long)m->magic,
           (unsigned long)m->fw_size,
           (unsigned long)m->fw_crc);

    if (m->magic != META_MAGIC) {
        printf("[BOOT] bad magic -> invalid\r\n");
        return 0;
    }
    if (m->fw_size < 8UL || m->fw_size > APP_MAX_SIZE) {
        printf("[BOOT] bad size -> invalid\r\n");
        return 0;
    }

    sp = *(volatile uint32_t *)APP_ADDRESS;
    pc = *(volatile uint32_t *)(APP_ADDRESS + 4);
    printf("[BOOT] APP SP = 0x%08lX  PC = 0x%08lX\r\n",
           (unsigned long)sp, (unsigned long)pc);

    if (sp < 0x20000000UL || sp > 0x20005000UL) {
        printf("[BOOT] bad SP -> invalid\r\n");
        return 0;
    }
    if (pc < APP_ADDRESS || pc >= META_ADDRESS) {
        printf("[BOOT] bad PC -> invalid\r\n");
        return 0;
    }

    printf("[BOOT] verifying flash CRC (%lu bytes) ...\r\n",
           (unsigned long)m->fw_size);
    crc = crc32_mpeg((const uint8_t *)APP_ADDRESS, m->fw_size);
    printf("[BOOT] calc CRC = 0x%08lX (expect 0x%08lX)\r\n",
           (unsigned long)crc, (unsigned long)m->fw_crc);

    if (crc != m->fw_crc) {
        printf("[BOOT] CRC mismatch -> invalid\r\n");
        return 0;
    }

    printf("[BOOT] APP OK\r\n");
    return 1;
}

/* ======================== Jump to APP ======================== */
static void JumpToApp(void)
{
    uint32_t  app_sp = *(volatile uint32_t *)APP_ADDRESS;
    uint32_t  app_pc = *(volatile uint32_t *)(APP_ADDRESS + 4);
    pFunction entry;
    uint8_t   i;

    printf("\r\n[BOOT] ================================\r\n");
    printf("[BOOT] Jumping to APP\r\n");
    printf("[BOOT] SP = 0x%08lX\r\n", (unsigned long)app_sp);
    printf("[BOOT] PC = 0x%08lX\r\n", (unsigned long)app_pc);
    printf("[BOOT] ================================\r\n\r\n");

    HAL_Delay(50);

    __disable_irq();
    HAL_CAN_DeInit(&hcan);
    HAL_UART_DeInit(&huart1);
    HAL_DeInit();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;
    for (i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFFUL;
        NVIC->ICPR[i] = 0xFFFFFFFFUL;
    }

    __set_MSP(app_sp);
    SCB->VTOR = APP_ADDRESS;
    __enable_irq();

    entry = (pFunction)app_pc;
    entry();
}

/* ======================== Command handler ======================== */
static void handle_cmd(uint8_t cmd)
{
    printf("[CMD] 0x%02X\r\n", cmd);

    switch (cmd) {
    case CMD_START:
        g_start_request = 1;
        g_expected_seq  = 0;
        g_total_seq     = 0;
        g_prog_error    = 0;
        g_crc_valid     = 0;
        printf("[CMD] START: enter upgrade mode\r\n");
        can_send_ack(ST_OK, 0);
        break;

    case CMD_ERASE:
        g_start_request = 1;
        g_expected_seq  = 0;
        g_total_seq     = 0;
        g_prog_error    = 0;
        g_crc_valid     = 0;
        printf("[CMD] ERASE: invalidate metadata first\r\n");
        meta_erase();
        printf("[CMD] erasing APP area (%u pages) ...\r\n",
               (unsigned)(APP_PAGE_END - APP_PAGE_START));
        if (flash_erase_app() != ST_OK) {
            printf("[CMD] ERASE FAILED\r\n");
            can_send_ack(ST_ERR_FLASH, 0);
        } else {
            printf("[CMD] erase done\r\n");
            can_send_ack(ST_OK, 0);
        }
        break;

    case CMD_FINISH:
        if (g_prog_error) {
            printf("[CMD] FINISH but prog_error set\r\n");
            can_send_ack(ST_ERR_FLASH, g_expected_seq);
        } else {
            uint32_t len = (uint32_t)g_total_seq * DATA_PER_FRAME;
            uint32_t calc;

            printf("[CMD] FINISH: %u frames (%lu bytes)\r\n",
                   (unsigned)g_total_seq, (unsigned long)len);

            printf("[CMD] computing CRC over flash ...\r\n");
            calc = crc32_mpeg((const uint8_t *)APP_ADDRESS, len);
            printf("[CMD] calc CRC = 0x%08lX\r\n", (unsigned long)calc);

            if (!g_crc_valid) {
                printf("[CMD] ERROR: no CRC received\r\n");
                can_send_ack(ST_ERR_CRC, g_expected_seq);
            } else if (calc != g_expected_crc) {
                printf("[CMD] ERROR: CRC mismatch! expected 0x%08lX\r\n",
                       (unsigned long)g_expected_crc);
                can_send_ack(ST_ERR_CRC, g_expected_seq);
            } else {
                printf("[CMD] CRC OK, writing metadata ...\r\n");
                if (meta_write(len, calc) != ST_OK) {
                    printf("[CMD] metadata write FAILED\r\n");
                    can_send_ack(ST_ERR_FLASH, g_expected_seq);
                } else {
                    printf("[CMD] metadata written, jumping\r\n");
                    can_send_ack(ST_OK, g_expected_seq);
                    g_jump_request = 1;
                }
            }
        }
        break;

    default:
        printf("[CMD] unknown\r\n");
        can_send_ack(ST_ERR_CMD, g_expected_seq);
        break;
    }
}

/* ======================== CAN RX callback ======================== */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx;
    uint8_t  d[8];
    uint16_t seq;
    uint32_t offset;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, d) != HAL_OK) {
        return;
    }

    switch (rx.StdId) {
    case CAN_ID_CMD:
        if (rx.DLC >= 1) {
            if (d[0] == CMD_FINISH && rx.DLC >= 5) {
                g_expected_crc = (uint32_t)d[1]
                               | ((uint32_t)d[2] << 8)
                               | ((uint32_t)d[3] << 16)
                               | ((uint32_t)d[4] << 24);
                g_crc_valid = 1;
            }
            handle_cmd(d[0]);
        }
        break;

    case CAN_ID_DATA:
        if (rx.DLC < 2) {
            can_send_ack(ST_ERR_CMD, g_expected_seq);
            break;
        }
        seq = (uint16_t)d[0] | ((uint16_t)d[1] << 8);

        if (seq != g_expected_seq) {
            can_send_ack(ST_ERR_SEQ, g_expected_seq);
            break;
        }
        offset = DATA_OFFSET(seq);
        if ((offset + (rx.DLC - 2)) > APP_MAX_SIZE) {
            can_send_ack(ST_ERR_SIZE, g_expected_seq);
            break;
        }
        if (flash_write(offset, &d[2], (uint32_t)(rx.DLC - 2)) != ST_OK) {
            g_prog_error = 1;
            can_send_ack(ST_ERR_FLASH, g_expected_seq);
            break;
        }
        g_expected_seq++;
        g_total_seq++;
        can_send_ack(ST_OK, g_expected_seq);
        break;

    default:
        break;
    }
}

/* ======================== main ======================== */
int main(void)
{
    uint8_t  app_ok;
    uint32_t t0;
    uint32_t last_tick = 0;

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_CAN1_Init();

    printf("\r\n\r\n");
    printf("==================================\r\n");
    printf("  CAN Bootloader - Stage 7\r\n");
    printf("==================================\r\n");
    printf("SYSCLK = %lu Hz\r\n", (unsigned long)HAL_RCC_GetHCLKFreq());
    printf("APP    = 0x%08lX  size = %lu KB\r\n",
           (unsigned long)APP_ADDRESS, (unsigned long)(APP_MAX_SIZE / 1024));
    printf("META   = 0x%08lX\r\n", (unsigned long)META_ADDRESS);

    if (HAL_CAN_Start(&hcan) != HAL_OK) {
        printf("[ERR] HAL_CAN_Start failed!\r\n");
        Error_Handler();
    }
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    printf("[OK ] CAN ready: 500 kbps\r\n");
    printf("----------------------------------\r\n\r\n");

    /* ---------- APP check + upgrade window ---------- */
    app_ok = app_is_valid();

    if (app_ok) {
        t0 = HAL_GetTick();
        printf("[BOOT] APP valid, upgrade window = 3s ...\r\n");
        printf("[BOOT] (send CMD_START now to stay in bootloader)\r\n");

        while ((HAL_GetTick() - t0) < 3000U) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            HAL_Delay(100);
            if (g_start_request) {
                break;
            }
        }

        if (!g_start_request) {
            printf("[BOOT] no command, booting APP ...\r\n");
            JumpToApp();
        }
        printf("[BOOT] upgrade mode entered\r\n");
    } else {
        printf("[BOOT] *** NO VALID APP *** stay in bootloader\r\n");
    }

    /* ---------- main loop ---------- */
    while (1)
    {
        uint32_t now = HAL_GetTick();

        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

        if ((now - last_tick) >= 1000U) {
            last_tick = now;
            printf("[STAT] expect_seq=%u  total=%u  err=%u\r\n",
                   (unsigned)g_expected_seq, (unsigned)g_total_seq,
                   (unsigned)g_prog_error);
        }

        if (g_jump_request) {
            HAL_Delay(100);
            JumpToApp();
        }

        HAL_Delay(100);
    }
}

/* ======================== System clock: HSE 8MHz -> 72MHz ======================== */
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

/* ======================== GPIO: PC13 LED ======================== */
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

/* ======================== USART1: 115200-8-N-1 ======================== */
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

/* ======================== CAN1: 500 kbps ======================== */
static void MX_CAN1_Init(void)
{
    CAN_FilterTypeDef filter = {0};

    hcan.Instance                  = CAN1;
    hcan.Init.Prescaler            = 4;
    hcan.Init.Mode                 = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth        = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1             = CAN_BS1_14TQ;
    hcan.Init.TimeSeg2             = CAN_BS2_3TQ;
    hcan.Init.TimeTriggeredMode    = DISABLE;
    hcan.Init.AutoBusOff           = DISABLE;
    hcan.Init.AutoWakeUp           = DISABLE;
    hcan.Init.AutoRetransmission   = ENABLE;
    hcan.Init.ReceiveFifoLocked    = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan) != HAL_OK) {
        Error_Handler();
    }

    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterBank           = 0;
    filter.FilterActivation     = ENABLE;
    if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK) {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
}

/* ======================== Error handler ======================== */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}
