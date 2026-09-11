#ifndef BSP_UART_H
#define BSP_UART_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

void     BSP_UART_Init(UART_HandleTypeDef *huart);
size_t   BSP_UART_Write(const uint8_t *data, size_t len);
size_t   BSP_UART_WriteString(const char *str);
bool     BSP_UART_ReadByte(uint8_t *byte);
size_t   BSP_UART_Read(uint8_t *buffer, size_t max_len);
size_t   BSP_UART_Available(void);

/* Callbacks & Interrupts Hooks */
void     BSP_UART_IRQHandler(void);

#endif