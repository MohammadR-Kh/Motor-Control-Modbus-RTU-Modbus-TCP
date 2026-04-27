/*
 * ethernet_config.c
 *
 *  Created on: Jan 2, 2026
 *      Author: GITEX
 */


#include "ethernet_config.h"
#include "socket.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>
/* CHANGE THESE IF YOU WANT */
uint8_t mac[6] = {0x00,0x08,0xDC,0x11,0x22,0x33};
uint8_t ip[4]  = {192,168,1,50};
uint8_t sn[4]  = {255,255,255,0};
uint8_t gw[4]  = {0,0,0,0};

void Ethernet_Init(void)
{
    wiz_NetInfo netinfo = {
        .mac = {0},
        .ip  = {0},
        .sn  = {0},
        .gw  = {0},
        .dns = {8,8,8,8},
        .dhcp = NETINFO_STATIC
    };

    memcpy(netinfo.mac, mac, 6);
    memcpy(netinfo.ip,  ip, 4);
    memcpy(netinfo.sn,  sn, 4);
    memcpy(netinfo.gw,  gw, 4);

    wizchip_setnetinfo(&netinfo);
}
