/**
 ******************************************************************************
 * @file    uart_cmd.c
 * @brief   上位机串口文本命令的解析与执行
 * @details 函数由 USART1 的接收回调调用：IDLE 中断里取走 DMA 已收到的字节并调用
 *          UartCmd_Process()，本文件按首字符分派命令，用 sscanf 解析命令后的数值，
 *          再切换电机模式并下发目标值。命令表（首字符区分，大小写敏感）：
 *          - "c <电流 单位 A>"：切到 MODE_COMMAND_CURRENT，按 ×1000 换算成 mA 后下发；
 *          - "v <速度 单位 圈/秒>"：切到 MODE_COMMAND_VELOCITY，按 ×51200 换算成细分步/秒后下发；
 *          - "p <位置 单位 圈>"：切到 MODE_COMMAND_POSITION，按 ×51200 换算成细分步后下发；
 *          - "s"：停止（切到 MODE_STOP）；
 *          - "z"：位置清零，把当前位置写为新的回零偏移并写 Flash；
 *          - "l"：清除堵转标志。
 *          数值解析失败回 "[error] Command format error!\r\n"，未知命令回
 *          "[error] Unknown command: x\r\n"（x 为收到的首字符）。
 *          换算系数 1000（A → mA）与 51200（圈 → 细分步，即 200 硬步 × 256 软细分）
 *          均来自对外接口与内部单位的约定。
 * @note    本函数在 USART1 IDLE 中断上下文执行（由接收回调直接调用），不可长时间阻塞；
 *          回复字符串用 Uart_SendString() 以 DMA 发送，发送前会等待上一次发送完成。
 *          数据来源为 USART1 的 DMA 接收缓冲区，解析使用 sscanf，依赖缓冲区内容以数字/
 *          空格形式出现，且该缓冲区在本函数返回后即被清空。
 *
 * @warning 本函数在 USART1 IDLE 中断上下文执行，数据来自 USART1 的 DMA 接收缓冲区，
 *          且直接用 sscanf 解析：调用方没有保证缓冲区一定以 '\0' 结尾（当前靠接收缓冲区
 *          为 256 字节、IDLE 中断里 memset 清掉已收字节来间接保证），一旦后续把 DMA 改成
 *          循环模式、或一帧填满整个缓冲区，sscanf 就会越界读。
 * @warning 回复字符串走 Uart_SendString()：该接口虽然使用 DMA 发送，但发送前会忙等上一次
 *          发送完成（tx_complete），在中断上下文里属于潜在阻塞点。
 * @todo    解析直接用 sscanf，建议改为长度受限、显式校验数字格式的解析方式。
 * @todo    "c"/"v"/"p" 支持的方向切换只写目标值，建议补上统一的停机保护后再切换模式。
 ******************************************************************************
 */

#include "uart_cmd.h"
#include "configurations.h"
#include "motor.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

/* 外部引用 */
extern BoardConfig_t boardConfig; /**< 全局板卡配置，位置清零等操作会写回该结构并保存到 Flash */

/**
 * @brief   解析并执行一条上位机文本命令
 * @details 先按数据首字符进入对应分支，再用 sscanf 从缓冲区中提取数值参数；参数解析失败
 *          只回错误提示、不改变电机模式与目标值。数值型命令会先把电机切到对应命令模式，
 *          再通过 Motor_SetCurrent()/Motor_SetVelocity()/Motor_SetPosition() 下发换算后的内部值
 *          （电机内部会按额定值做限幅）。
 * @param[in] data 接收缓冲区指针，指向 USART1 DMA 收到的原始字节，首字节为命令字符
 * @param[in] len  本次收到的字节数；为 0 时直接返回，不产生任何输出
 * @note    在 USART1 的 IDLE 中断上下文（经 Uart_SetRxCallback 注册的回调）执行，
 *          不可长时间阻塞。sscanf 依赖缓冲区内容以数字/空格形式出现：数据来自 DMA
 *          接收缓冲区 rx_buffer（256 字节，IDLE 中断里 memset 掉已收字节后重新启动 DMA），
 *          当前配置下缓冲区内容之后有 NUL 字节，但函数自身并不校验；未知命令的提示用
 *          sprintf 写入 64 字节的局部缓冲 buffer。
 *          函数不修改 data 指向的内容，也不保存任何解析状态。
 *
 * @warning 在 USART1 IDLE 中断上下文执行，数据来自 USART1 的 DMA 接收缓冲区且直接用 sscanf
 *          解析：调用方不保证缓冲区一定以 '\0' 结尾（当前靠 256 字节缓冲区 + IDLE 中断里
 *          memset 清掉已收字节间接保证），若把 DMA 改为循环模式或一帧填满整个缓冲区，
 *          sscanf 会越界读。回复用 Uart_SendString() 发送，其内部会忙等上一次 DMA 发送完成。
 * @todo    建议改为长度受限且显式校验数字格式的解析方式，替代直接 sscanf。
 */
void UartCmd_Process(uint8_t *data, uint16_t len)
{
    float cur, pos, vel;
    int ret = 0;
    char buffer[64]; /* 错误提示字符串的本地缓冲：最大 64 字节，供 Uart_SendString() 发送 */

    if (len == 0)
        return;

    switch (data[0])
    {
    case 'c': /* 电流模式：c <电流 单位 A>，×1000 换算为 mA（内部限幅到 ratedCurrent） */
        ret = sscanf((char *)data, "c %f", &cur);
        if (ret < 1)
        {
            Uart_SendString("[error] Command format error!\r\n");
        }
        else
        {
            if (Motor_GetMode() != MODE_COMMAND_CURRENT)
            {
                Motor_SetMode(MODE_COMMAND_CURRENT);
            }
            Motor_SetCurrent((int32_t)(cur * 1000));
        }
        break;

    case 'v': /* 速度模式：v <速度 单位 圈/秒>，×51200 换算为细分步/秒（内部限幅到 ratedVelocity） */
        ret = sscanf((char *)data, "v %f", &vel);
        if (ret < 1)
        {
            Uart_SendString("[error] Command format error!\r\n");
        }
        else
        {
            if (Motor_GetMode() != MODE_COMMAND_VELOCITY)
            {
                Motor_SetMode(MODE_COMMAND_VELOCITY);
            }
            Motor_SetVelocity((int32_t)(vel * MOTOR_SUBDIVIDE_STEPS));
        }
        break;

    case 'p': /* 位置模式：p <位置 单位 圈>，×51200 换算为细分步后作为目标位置 */
        ret = sscanf((char *)data, "p %f", &pos);
        if (ret < 1)
        {
            Uart_SendString("[error] Command format error!\r\n");
        }
        else
        {
            if (Motor_GetMode() != MODE_COMMAND_POSITION)
            {
                Motor_SetMode(MODE_COMMAND_POSITION);
            }
            Motor_SetPosition((int32_t)(pos * MOTOR_SUBDIVIDE_STEPS));
        }
        break;

    case 's': /* 停止：切到 MODE_STOP，无参数 */
        Motor_SetMode(MODE_STOP);
        break;

    case 'z': /* 位置清零：把当前位置作为新的回零偏移、更新 boardConfig 并写 Flash，无参数 */
        Motor_ZeroPosition();
        break;

    case 'l': /* 清除堵转：清除堵转标志，无参数 */
        Motor_ClearStallFlag();
        break;
    default:
        sprintf(buffer, "[error] Unknown command: %c\r\n", data[0]);
        Uart_SendString(buffer);
        break;
    }
}
