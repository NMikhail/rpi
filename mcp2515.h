#ifndef MCP2515x_H
#define MCP2515x_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include "mcp_can_dfs.h"

#define CNT_CAN_INIT    100
#define SPI_PATH        "/dev/spidev0.1"
// Инициализация CAN
uint8_t can_begin(uint8_t speedset, const uint8_t clockset); 
//Проверка сообщений
uint8_t checkReceive(void); 
//Установка маски и фильтра
uint8_t init_Mask(uint8_t num, uint8_t ext, unsigned long ulData); 
uint8_t init_Filt(uint8_t num, uint8_t ext, unsigned long ulData);
//Отправка сообщений
uint8_t sendMsgBuf(unsigned long id, uint8_t ext, uint8_t len, const uint8_t *buf, uint8_t wait_sent); 
//Чтения сообщения
uint8_t readMsgBuf(uint8_t *len, uint8_t buf[]);
//Получаем CAN ID
unsigned long getCanId(void); 
#endif