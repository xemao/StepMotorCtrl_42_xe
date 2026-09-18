/**
 ******************************************************************************
 * @file    uart_cmd.h
 * @brief   USART1 文本命令处理接口
 * @details 仅暴露 UartCmd_Process()：在 USART1 的 IDLE 中断中由接收回调调用，参数是
 *          DMA 接收缓冲区的原始字节与长度。对外的命令单位与内部单位的换算
 *          （电流 A → mA、速度 圈/秒 → 细分步/秒、位置 圈 → 细分步）在 uart_cmd.c 内完成。
 * @note    该接口运行在中断上下文，实现中不可长时间阻塞；缓冲区内容以文本形式给出，
 *          且在回调返回后即被清空。
 *
 * @warning 调用方（USART1 IDLE 中断）不保证缓冲区以 '\0' 结尾，而实现内部用 sscanf 直接
 *          解析，属于依赖当前 DMA 配置与上位机发送习惯的隐式约定。
 ******************************************************************************
 */

#ifndef UART_CMD_H
#define UART_CMD_H

#include <stdint.h>

/**
 * @brief   处理接收到的串口数据，解析并执行一条文本命令
 * @details 支持 "c <电流A>"、"v <速度 圈/秒>"、"p <位置 圈>"、"s"、"z"、"l" 六种命令，
 *          解析失败回复 "[error] Command format error!"，未知命令回复
 *          "[error] Unknown command: x"。
 * @note    在 USART1 IDLE 中断上下文执行，不可长时间阻塞；由 USART1 的 DMA 接收缓冲区
 *          提供数据，使用 sscanf 解析。
 *
 * @warning 在中断上下文执行；调用方不保证缓冲区以 '\0' 结尾，而实现用 sscanf 直接解析，
 *          依赖当前 DMA 配置（256 字节缓冲 + IDLE 中断清零）与上位机发送习惯。
 */
void UartCmd_Process(uint8_t *data, uint16_t len);

#endif
