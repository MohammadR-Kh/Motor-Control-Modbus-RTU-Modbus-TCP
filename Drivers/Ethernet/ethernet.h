/*
 * ethernet.h
 *
 *  Created on: Feb 27, 2026
 *      Author: Mohammadreza
 */

#ifndef INC_ETHERNET_H_
#define INC_ETHERNET_H_

#include "stm32f4xx_hal.h"

#define TCP_SOCKET  0
#define TCP_PORT    502

#define W5500_CS_LOW() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
#define W5500_CS_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
#define W5500_RST_LOW() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
#define W5500_RST_HIGH() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);

void wizchip_select(void);
void wizchip_deselect(void);
uint8_t wizchip_read(void);
void wizchip_write(uint8_t wb);
void W5500_Reset(void);
void ModbusTCP_SendException(uint8_t *req, uint8_t fc, uint8_t ex);
void ModbusTCP_Process(uint8_t *req, uint16_t len);
void TCP_Server(void);



#endif /* INC_ETHERNET_H_ */
