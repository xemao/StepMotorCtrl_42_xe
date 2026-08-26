#ifndef UART_CMD_H
#define UART_CMD_H

#include <stdint.h>

/* 处理接收到的串口数据 */
void UartCmd_Process(uint8_t *data, uint16_t len);

#endif
