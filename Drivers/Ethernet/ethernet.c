/*
 * ethernet.c
 *
 *  Created on: Feb 27, 2026
 *      Author: Mohammadreza
 */

#include "wizchip_conf.h"
#include "socket.h"
#include "ethernet_config.h"
#include <stdio.h>
#include <string.h>
#include "DHCP/dhcp.h"
#include "ethernet.h"
#include "register_map.h"

extern SPI_HandleTypeDef hspi1;

uint8_t rx_buf_tcp[2048];


void wizchip_select(void)
{
    W5500_CS_LOW();
}

void wizchip_deselect(void)
{
    W5500_CS_HIGH();
}


uint8_t wizchip_read(void)
{
    uint8_t rb;
    HAL_SPI_Receive(&hspi1, &rb, 1, HAL_MAX_DELAY);
    return rb;
}

void wizchip_write(uint8_t wb)
{
    HAL_SPI_Transmit(&hspi1, &wb, 1, HAL_MAX_DELAY);
}
void W5500_Reset(void)
{
    W5500_RST_LOW();
    HAL_Delay(2);          // >500 µs required
    W5500_RST_HIGH();
    HAL_Delay(2);          // allow internal PHY start
}
void ModbusTCP_SendException(uint8_t *req, uint8_t fc, uint8_t ex)
{
    uint8_t resp[9 + 2];

    /* MBAP */
    resp[0] = req[0];   // TID hi
    resp[1] = req[1];   // TID lo
    resp[2] = 0;
    resp[3] = 0;
    resp[4] = 0;
    resp[5] = 3;        // Length = UnitID + FC + EX
    resp[6] = req[6];   // Unit ID

    /* PDU */
    resp[7] = fc | 0x80;
    resp[8] = ex;

    send(TCP_SOCKET, resp, 9);
}
/*----------------------MODBUS ONLY--------------------------*/
void ModbusTCP_Process(uint8_t *req, uint16_t len)
{
    if(len < 12) return;

    uint8_t  fc         = req[7];
    uint16_t start_addr = (req[8]  << 8) | req[9];
    uint16_t quantity   = (req[10] << 8) | req[11];

    uint8_t resp[260];
    uint16_t i;

    /* ================= FC03 – Read Holding Registers ================= */
    if(fc == 0x03)
    {
        if(quantity == 0 || quantity > 125 ||
           (start_addr + quantity) > HOLDING_REG_COUNT)
        {
            ModbusTCP_SendException(req, fc, 0x02);
            return;
        }

        /* MBAP */
        resp[0] = req[0];
        resp[1] = req[1];
        resp[2] = 0;
        resp[3] = 0;
        resp[4] = 0;
        resp[5] = 3 + quantity * 2;
        resp[6] = req[6];

        /* PDU */
        resp[7] = 0x03;
        resp[8] = quantity * 2;

        for(i = 0; i < quantity; i++)
        {
            resp[9 + i*2]  = holding_regs[start_addr + i] >> 8;
            resp[10 + i*2] = holding_regs[start_addr + i] & 0xFF;
        }

        send(TCP_SOCKET, resp, 9 + quantity * 2);
    }

    /* ================= FC06 – Write Single Register ================= */
    else if(fc == 0x06)
    {
        uint16_t value = (req[10] << 8) | req[11];

        if(start_addr >= HOLDING_REG_COUNT)
        {
            ModbusTCP_SendException(req, fc, 0x02);
            return;
        }

        holding_regs[start_addr] = value;

        /* Echo request as response */
        send(TCP_SOCKET, req, 12);
    }

    /* ================= FC16 – Write Multiple Registers ================= */
    else if(fc == 0x10)
    {
        uint8_t byte_count = req[12];

        if(quantity == 0 ||
           quantity > 123 ||
           byte_count != quantity * 2 ||
           (start_addr + quantity) > HOLDING_REG_COUNT)
        {
            ModbusTCP_SendException(req, fc, 0x02);
            return;
        }

        for(i = 0; i < quantity; i++)
        {
            holding_regs[start_addr + i] =
                (req[13 + i*2] << 8) | req[14 + i*2];
        }

        /* Response = FC + start + quantity */
        resp[0] = req[0];
        resp[1] = req[1];
        resp[2] = 0;
        resp[3] = 0;
        resp[4] = 0;
        resp[5] = 6;
        resp[6] = req[6];

        resp[7]  = 0x10;
        resp[8]  = req[8];
        resp[9]  = req[9];
        resp[10] = req[10];
        resp[11] = req[11];

        send(TCP_SOCKET, resp, 12);
    }

    /* ================= Unsupported Function ================= */
    else
    {
        ModbusTCP_SendException(req, fc, 0x01);
    }
}
/*----------------------MODBUS ONLY--------------------------*/
void TCP_Server(void)
{
    int32_t len;

    if(getSn_SR(TCP_SOCKET) != SOCK_ESTABLISHED)
    {
        // Handle state machine once
        switch(getSn_SR(TCP_SOCKET))
        {
            case SOCK_CLOSED:
                socket(TCP_SOCKET, Sn_MR_TCP, TCP_PORT, 0);
                break;

            case SOCK_INIT:
                listen(TCP_SOCKET);
                break;

            case SOCK_CLOSE_WAIT:
                disconnect(TCP_SOCKET);
                break;
        }
        return;
    }

    uint16_t rx_size = getSn_RX_RSR(TCP_SOCKET);

    if(rx_size > 0)
    {
        len = recv(TCP_SOCKET, rx_buf_tcp, sizeof(rx_buf_tcp));
        if(len > 0)
        {
            ModbusTCP_Process(rx_buf_tcp, len);
        }
    }
}
