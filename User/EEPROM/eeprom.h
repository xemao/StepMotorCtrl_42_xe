/**
 ******************************************************************************
 * @file    eeprom.h
 * @brief   Flash 模拟 EEPROM 的对外接口声明（介质为片内 Flash 的 APP_DATA 分区）。
 * @details 对外只暴露三个接口，实现见 eeprom.c：addr 是相对分区基地址
 *          （STOCKPILE_APP_DATA_ADDR = 0x0800FC00）的字节偏移，有效范围 0 ~ 1023
 *          （分区大小与 EEPROM_MAX_SIZE 均为 1024 字节）；读/写按字节计数，
 *          写入时内部按半字（16 位）编程。上层以 addr = 0、size = sizeof(BoardConfig_t)
 *          保存整块板卡配置：main.c 上电读取并在无效时回写默认值，motor.c 在位置清零后回写。
 * @note    读写都是对片内 Flash 的直接操作：越界调用会被静默忽略（不报错也不写入），
 *          写入会阻塞等待编程完成且耗时远大于 RAM 访问，不应在中断或实时性敏感处调用。
 ******************************************************************************
 */

#ifndef EEPROM_H
#define EEPROM_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief   从模拟 EEPROM 读取数据（Flash → 内存）
 * @details 以分区基地址加偏移得到内存映射的 Flash 地址，把 addr 起的 size 个字节逐字节拷贝到
 *          data（目标缓冲区需至少能容纳 size 字节）；addr 为相对偏移，取值 0 ~ 1023。
 *          参数与实现的完整说明见 eeprom.c。
 * @note    越界（addr + size > EEPROM_MAX_SIZE = 1024）时直接返回：不写 data、不报错。
 */
void EEPROM_Read(uint32_t addr, void *data, uint32_t size);

/**
 * @brief   向模拟 EEPROM 写入数据（内存 → Flash）
 * @details 借助 stockpile_f103cb.c 的整片擦除与半字编程原语，把 data 指向的 size 个字节写到
 *          addr 偏移处（data 在实现中按 uint16_t * 解释）；addr 为相对偏移，取值 0 ~ 1023。
 *          参数与实现的完整说明见 eeprom.c。
 * @note    越界时直接返回、不写入。只有复位后的首次写入（或 EEPROM_Erase 之后）会自动
 *          整片擦除，之后的写入都是覆盖写。
 * @warning 写入期间 Flash 处于解锁状态并阻塞等待编程完成（取指停顿），不可在中断上下文调用。
 */
void EEPROM_Write(uint32_t addr, void *data, uint32_t size);

/**
 * @brief   判断模拟 EEPROM 中是否已写入过数据
 * @details 读取分区首地址处的第一个 32 位字，与擦除后的空 Flash 值 0xFFFFFFFF 比较。
 * @note    只检查首字，因此仅表示"分区被写过"，不代表内容合法（合法性由上层检查
 *          BoardConfig_t.configStatus 字段）。分区刚被 EEPROM_Erase 擦除时返回 false。
 */
bool EEPROM_IsValid(void);

#endif
