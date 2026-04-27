/*
 * uart_rs_485.c
 *
 *  Created on: Dec 20, 2025
 *      Author: GITEX
 */
#include "uart_rs_485.h"
#include <stdio.h>
#include "string.h"
#include "register_map.h"

#define MODBUS_SLAVE_ADDR   0x01
#define MODBUS_FC_READ_HOLDING_REGS  0x03
#define MODBUS_EX_ILLEGAL_FUNCTION   0x01
#define MODBUS_EX_ILLEGAL_ADDRESS    0x02
#define MODBUS_EX_ILLEGAL_VALUE      0x03
#define HOLDING_REG_COUNT  16
#define TX_QUEUE_SLOTS 4
#define TX_SLOT_MAX_LEN LINE_MAX_LEN


uint8_t rx_buf[RX_BUF_SIZE];
volatile uint8_t uart_idle_flag = 0;
extern UART_HandleTypeDef huart2;
volatile uint8_t tx_busy = 0;
static uint8_t tx_queue[TX_QUEUE_SLOTS][TX_SLOT_MAX_LEN];
static uint16_t tx_queue_len[TX_QUEUE_SLOTS];
static volatile uint8_t tx_q_head = 0;
static volatile uint8_t tx_q_tail = 0;
static volatile uint8_t tx_q_count = 0;
static volatile uint8_t tx_dma_slot = 0;
volatile uint32_t uart_tx_total_bytes = 0;
volatile uint32_t uart_tx_errors = 0;
volatile uint32_t uart_rx_errors = 0;
volatile uint32_t rx_overflow_count = 0;
volatile uint8_t rx_overflow_flag = 0;


void uart_dma_rx_start(void)
{
    HAL_UART_Receive_DMA(&huart2, rx_buf, RX_BUF_SIZE);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
}

int uart_tx_enqueue(const uint8_t *data, uint16_t len)
{
    if (len == 0 || len > TX_SLOT_MAX_LEN) return -1;

    HAL_NVIC_DisableIRQ(DMA1_Stream6_IRQn);
    if (tx_q_count >= TX_QUEUE_SLOTS) {
    	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
        return -1;
    }


    memcpy(tx_queue[tx_q_tail], data, len);
    tx_queue_len[tx_q_tail] = len;

    tx_q_tail = (tx_q_tail + 1) % TX_QUEUE_SLOTS;
    tx_q_count++;
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

    return 0;
}


void uart_tx_start_next_if_needed(void)
{
    if (tx_busy) return;
    if (tx_q_count == 0) return;

    HAL_NVIC_DisableIRQ(DMA1_Stream6_IRQn);
    tx_dma_slot = tx_q_head;
    uint16_t len = tx_queue_len[tx_dma_slot];
    tx_q_head = (tx_q_head + 1) % TX_QUEUE_SLOTS;
    tx_q_count--;
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

    RS485_TX_ENABLE();
    __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_TC);
    tx_busy = 1;
    HAL_UART_Transmit_DMA(&huart2, tx_queue[tx_dma_slot], len);

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart2) return;

    uart_rx_errors++;

    HAL_UART_AbortReceive(&huart2);

    HAL_UART_Receive_DMA(&huart2, rx_buf, RX_BUF_SIZE);

    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
}

uint16_t modbus_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t pos = 0; pos < len; pos++) {
        crc ^= buf[pos];
        for (int i = 0; i < 8; i++) {
            if (crc & 1) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void modbus_send_exception(uint8_t slave, uint8_t func, uint8_t code)
{
    uint8_t resp[5];
    resp[0] = slave;
    resp[1] = func | 0x80;
    resp[2] = code;

    uint16_t crc = modbus_crc16(resp, 3);
    resp[3] = crc & 0xFF;
    resp[4] = crc >> 8;

    uart_tx_enqueue(resp, 5);
    uart_tx_start_next_if_needed();
}
static int modbus_reg_writable(uint16_t addr)
{
    return (addr == REG_TARGET_RPM ||
            addr == REG_TARGET_POSITION ||
			addr == REG_MODE);
}
static void modbus_handle_fc06(uint8_t *buf, uint16_t len)
{
    if (len != 8) {
        modbus_send_exception(buf[0], 0x06, 0x03);
        return;
    }

    uint16_t addr = (buf[2] << 8) | buf[3];
    uint16_t val  = (buf[4] << 8) | buf[5];

    if (addr >= HOLDING_REG_COUNT) {
        modbus_send_exception(buf[0], 0x06, 0x02);
        return;
    }

    if (!modbus_reg_writable(addr)) {
        modbus_send_exception(buf[0], 0x06, 0x02);
        return;
    }

//    if (addr == REG_TARGET_RPM) {
//        if (val < 0 || val > 3450) {
//            modbus_send_exception(buf[0], 0x06, 0x03);
//            return;
//        }
//    }
    holding_regs[addr] = val;
    uart_tx_enqueue(buf, 8);
    uart_tx_start_next_if_needed();
}
static void modbus_handle_fc10(uint8_t *buf, uint16_t len)
{
    uint16_t addr = (buf[2] << 8) | buf[3];
    uint16_t qty  = (buf[4] << 8) | buf[5];
    uint8_t  byte_cnt = buf[6];

    if (qty == 0 || qty > 10) {
        modbus_send_exception(buf[0], 0x10, 0x03);
        return;
    }

    if (addr + qty > HOLDING_REG_COUNT) {
        modbus_send_exception(buf[0], 0x10, 0x02);
        return;
    }

    if (byte_cnt != qty * 2) {
        modbus_send_exception(buf[0], 0x10, 0x03);
        return;
    }
    for (uint16_t i = 0; i < qty; i++) {
        if (!modbus_reg_writable(addr + i)) {
            modbus_send_exception(buf[0], 0x10, 0x02);
            return;
        }
    }
    uint8_t *data = &buf[7];
    for (uint16_t i = 0; i < qty; i++) {
        uint16_t val = (data[2*i] << 8) | data[2*i + 1];

//        if ((addr + i) == REG_TARGET_RPM) {
//            if (val < 0 || val > 3450) {
//                modbus_send_exception(buf[0], 0x10, 0x03);
//                return;
//            }
//        }

        holding_regs[addr + i] = val;
    }
    uint8_t resp[8];
    resp[0] = buf[0];
    resp[1] = 0x10;
    resp[2] = buf[2];
    resp[3] = buf[3];
    resp[4] = buf[4];
    resp[5] = buf[5];

    uint16_t crc = modbus_crc16(resp, 6);
    resp[6] = crc & 0xFF;
    resp[7] = crc >> 8;

    uart_tx_enqueue(resp, 8);
    uart_tx_start_next_if_needed();
}

void modbus_handle_frame(uint8_t *frame, uint16_t len)
{
    if (len < 4) return;

    /* CRC check */
    uint16_t rx_crc = frame[len - 2] | (frame[len - 1] << 8);
    uint16_t calc_crc = modbus_crc16(frame, len - 2);
    if (rx_crc != calc_crc) return;

    uint8_t addr = frame[0];
    uint8_t func = frame[1];

    if (addr != MODBUS_SLAVE_ADDR && addr != 0x00) return;

    uint8_t respond = (addr != 0x00);

    uint8_t tx_buf[256];
    uint16_t tx_len = 0;

    switch (func) {

    case MODBUS_FC_READ_HOLDING_REGS:
    {
        if (len != 8) return;

        uint16_t start =
            (frame[2] << 8) | frame[3];
        uint16_t count =
            (frame[4] << 8) | frame[5];

        if (count == 0 || count > HOLDING_REG_COUNT) {
            break;
        }

        if ((start + count) > HOLDING_REG_COUNT) {
            break;
        }

        if (!respond) return;

        tx_buf[0] = MODBUS_SLAVE_ADDR;
        tx_buf[1] = MODBUS_FC_READ_HOLDING_REGS;
        tx_buf[2] = count * 2;

        tx_len = 3;
        for (uint16_t i = 0; i < count; i++) {
            uint16_t val = holding_regs[start + i];
            tx_buf[tx_len++] = val >> 8;
            tx_buf[tx_len++] = val & 0xFF;
        }

        uint16_t crc = modbus_crc16(tx_buf, tx_len);
        tx_buf[tx_len++] = crc & 0xFF;
        tx_buf[tx_len++] = crc >> 8;

        uart_tx_enqueue(tx_buf, tx_len);
        uart_tx_start_next_if_needed();
        return;
    }
    case 0x06:
        modbus_handle_fc06(frame, len);
        return;

    case 0x10:
        modbus_handle_fc10(frame, len);
        return;

    default:
        break;
    }
}




