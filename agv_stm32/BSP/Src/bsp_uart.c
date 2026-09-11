#include "bsp_uart.h"
#include "app_config.h"

#if AGV_ENABLE_UART_TTL

#include "ring_buffer.h"
#include <string.h>

#define BSP_UART_DMA_RX_SIZE 256
#define BSP_UART_DMA_RX_FIFO_SIZE 512
#define BSP_UART_DMA_TX_FIFO_SIZE 512

static UART_HandleTypeDef *s_huart = NULL;

static uint8_t s_dma_rx_buffer[BSP_UART_DMA_RX_SIZE];
static size_t s_old_pos = 0;

static uint8_t s_rx_fifo_storage[BSP_UART_DMA_RX_FIFO_SIZE];
static RingBuffer_t s_rx_rb;

static uint8_t s_tx_fifo_storage[BSP_UART_DMA_TX_FIFO_SIZE];
static RingBuffer_t s_tx_rb;

static uint8_t s_dma_tx_chunk[64];
static volatile bool s_dma_tx_busy = false;

static void bsp_uart_trigger_tx_dma(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (s_dma_tx_busy || RingBuffer_Available(&s_tx_rb) == 0) {
        __set_PRIMASK(primask);
        return;
    }

    size_t chunk_len = RingBuffer_PopArray(&s_tx_rb, s_dma_tx_chunk, sizeof(s_dma_tx_chunk));
    s_dma_tx_busy = (chunk_len > 0);
    __set_PRIMASK(primask);

    if (s_dma_tx_busy) {
        if (HAL_UART_Transmit_DMA(s_huart, s_dma_tx_chunk, (uint16_t)chunk_len) != HAL_OK) {
            s_dma_tx_busy = false;
        }
    }
}

void BSP_UART_Init(UART_HandleTypeDef *huart) {
    s_huart = huart;
    s_old_pos = 0;
    s_dma_tx_busy = false;

    RingBuffer_Init(&s_rx_rb, s_rx_fifo_storage, sizeof(s_rx_fifo_storage));
    RingBuffer_Init(&s_tx_rb, s_tx_fifo_storage, sizeof(s_tx_fifo_storage));

    HAL_UART_Receive_DMA(s_huart, s_dma_rx_buffer, sizeof(s_dma_rx_buffer));
    __HAL_UART_ENABLE_IT(s_huart, UART_IT_IDLE);
}

size_t BSP_UART_Write(const uint8_t *data, size_t len) {
    size_t written = RingBuffer_PushArray(&s_tx_rb, data, len);
    bsp_uart_trigger_tx_dma();
    return written;
}

size_t BSP_UART_WriteString(const char *str) {
    return BSP_UART_Write((const uint8_t *)str, strlen(str));
}

bool BSP_UART_ReadByte(uint8_t *byte) {
    return RingBuffer_Pop(&s_rx_rb, byte);
}

size_t BSP_UART_Read(uint8_t *buffer, size_t max_len) {
    return RingBuffer_PopArray(&s_rx_rb, buffer, max_len);
}

size_t BSP_UART_Available(void) {
    return RingBuffer_Available(&s_rx_rb);
}

/* Callbacks & Interrupts Hooks */
void BSP_UART_IRQHandler(void) {
    if (s_huart == NULL) return;

    if (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(s_huart);

        size_t current_pos = sizeof(s_dma_rx_buffer) - __HAL_DMA_GET_COUNTER(s_huart->hdmarx);

        if (current_pos != s_old_pos) {
            if (current_pos > s_old_pos) {
                RingBuffer_PushArray(&s_rx_rb, &s_dma_rx_buffer[s_old_pos], current_pos - s_old_pos);
            } else {
                RingBuffer_PushArray(&s_rx_rb, &s_dma_rx_buffer[s_old_pos], sizeof(s_dma_rx_buffer) - s_old_pos);
                if (current_pos > 0) {
                    RingBuffer_PushArray(&s_rx_rb, &s_dma_rx_buffer[0], current_pos);
                }
            }

            s_old_pos = current_pos;
        }
    }

    if (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_OREFLAG(s_huart);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (s_huart != NULL && huart->Instance == s_huart->Instance) {
        s_dma_tx_busy = false;
        bsp_uart_trigger_tx_dma();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (s_huart != NULL && huart->Instance == s_huart->Instance) {
        __HAL_UART_CLEAR_OREFLAG(s_huart);

        HAL_UART_DMAStop(s_huart);
        s_old_pos = 0;

        HAL_UART_Receive_DMA(s_huart, s_dma_rx_buffer, sizeof(s_dma_rx_buffer));
        __HAL_UART_ENABLE_IT(s_huart, UART_IT_IDLE);
    }
}

#else

/* Stubs không tiêu tốn RAM khi UART TTL được cấu hình tắt */
void   BSP_UART_Init(UART_HandleTypeDef *huart) { (void)huart; }
size_t BSP_UART_Write(const uint8_t *data, size_t len) { (void)data; (void)len; return 0; }
size_t BSP_UART_WriteString(const char *str) { (void)str; return 0; }
bool   BSP_UART_ReadByte(uint8_t *byte) { (void)byte; return false; }
size_t BSP_UART_Read(uint8_t *buffer, size_t max_len) { (void)buffer; (void)max_len; return 0; }
size_t BSP_UART_Available(void) { return 0; }
void   BSP_UART_IRQHandler(void) {}
void   HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) { (void)huart; }
void   HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) { (void)huart; }

#endif /* AGV_ENABLE_UART_TTL */

