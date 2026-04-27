/*
 * uart_rs_485.h
 *
 *  Created on: Dec 20, 2025
 *      Author: GITEX
 */

#ifndef INC_UART_RS_485_H_
#define INC_UART_RS_485_H_

#include "stm32f4xx_hal.h"

#define RS485_TX_ENABLE()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define RS485_RX_ENABLE()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define RX_BUF_SIZE  1024
#define LINE_MAX_LEN 1024

void uart_dma_rx_start(void);
int uart_tx_enqueue(const uint8_t *data, uint16_t len);
void uart_tx_start_next_if_needed(void);
void modbus_handle_frame(uint8_t *frame, uint16_t len);



#endif /* INC_UART_RS_485_H_ */
