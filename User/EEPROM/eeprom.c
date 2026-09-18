/**
 ******************************************************************************
 * @file    eeprom.c
 * @brief   在片内 Flash 的 APP_DATA 分区上模拟 EEPROM 的读写实现（容量 1024 字节）。
 * @details 地址参数 addr 是相对分区基地址 STOCKPILE_APP_DATA_ADDR(0x0800FC00) 的字节偏移，
 *          有效范围 0 ~ 1023（EEPROM_MAX_SIZE = 1024）。读操作按字节直接访问内存映射的
 *          Flash；写操作借助 stockpile_f103cb.c 的半字编程原语：先整片擦除（仅复位后首次
 *          写入）、再设置写地址、按 (size + 1) / 2 个半字写入、最后重新上锁。上层 main.c 与
 *          motor.c 均以 addr = 0、size = sizeof(BoardConfig_t) 保存整块板卡配置。
 * @note    写入是阻塞操作且期间 Flash 处于解锁状态，不可在中断上下文调用；重复写入不会自动
 *          擦除（见 EEPROM_Write），要覆盖旧数据应先调用 EEPROM_Erase()。
 ******************************************************************************
 */

#include "eeprom.h"
#include "stockpile_f103cb.h"
// #include <string.h>

/**
 * @brief  EEPROM 存储区映射参数：直接以用户配置分区（stockpile_data / APP_DATA）为介质
 * @details 基地址取分区起始地址，容量与分区等长，二者共同界定了 addr 的合法范围。
 */
#define EEPROM_BASE_ADDR STOCKPILE_APP_DATA_ADDR /**< 基地址：APP_DATA 分区起始地址(Flash 绝对地址) */
#define EEPROM_MAX_SIZE  1024                    /**< 存储区容量上限，单位：字节（1024 = 1KB，与 APP_DATA 分区等长） */

static bool s_is_erased = false; /**< 擦除标志：false = 复位后未擦除；true = 已擦除 */

/**
 * @brief   从模拟 EEPROM 读取任意字节数据
 * @details 先做边界检查（addr + size > EEPROM_MAX_SIZE 时直接返回），再以
 *          EEPROM_BASE_ADDR + addr 为源地址，逐字节拷贝到 data 指向的缓冲区。
 * @param[in]  addr  相对分区基地址的字节偏移，范围 0 ~ 1023
 * @param[out] data  目标缓冲区首地址，至少可容纳 size 字节
 * @param[in]  size  待读取的字节数
 * @note    越界调用为静默失败：data 内容保持原样，调用者无法从返回值判断是否读到数据
 *          （main.c 的做法是先读回整个 BoardConfig_t，再检查其 configStatus 字段）。
 *          本函数不检查 data 是否为 NULL；读取无需 Flash 解锁，可在任意上下文调用。
 * @todo    越界时没有错误上报，也未校验 data 指针与 size 的合法性。
 */
void EEPROM_Read(uint32_t addr, void *data, uint32_t size)
{
    if (addr + size > EEPROM_MAX_SIZE)
        return;

    uint8_t *dst = (uint8_t *)data;
    uint32_t flash_addr = EEPROM_BASE_ADDR + addr;

    for (uint32_t i = 0; i < size; i++)
    {
        dst[i] = *((uint8_t *)flash_addr + i);
    }
}

/**
 * @brief   向模拟 EEPROM 写入任意字节数据
 * @details 流程：① 边界检查，addr + size > EEPROM_MAX_SIZE 时直接返回；
 *          ② 若 s_is_erased 为 false（复位后尚未写过），先调用 Stockpile_Flash_Data_Empty()
 *          整片擦除 APP_DATA 分区，并置 s_is_erased = true；③ Stockpile_Flash_Data_Begin()
 *          解锁并复位写地址；④ Stockpile_Flash_Data_Set_Write_Add() 把写地址设为
 *          EEPROM_BASE_ADDR + addr；⑤ 按 (size + 1) / 2 计算半字个数并调用
 *          Stockpile_Flash_Data_Write_Data16() 写入；⑥ Stockpile_Flash_Data_End() 重新上锁。
 * @param[in]  addr  相对分区基地址的字节偏移，范围 0 ~ 1023
 * @param[in]  data  源数据缓冲区首地址（内部按 uint16_t * 解释后交给半字写入）
 * @param[in]  size  待写入的字节数
 * @note    越界调用为静默失败：不擦除、不写入、不报错。
 * @warning s_is_erased 只在复位后第一次写入（或调用 EEPROM_Erase 之后）为 false，
 *          因此第二次及以后的 EEPROM_Write 只写不擦：Flash 只能把 1 改写为 0，
 *          重复写入同一位置的结果相当于新旧数据按位与，新数据比旧数据短时旧内容会残留。
 *          需要改写时应先调用 EEPROM_Erase()（main.c 的 CONFIG_COMMIT 分支也是先
 *          Stockpile_Flash_Data_Empty(&stockpile_data) 再调用本函数）。
 * @warning size 为奇数时按 (size + 1) / 2 个半字写入，会多写 1 个字节：该字节取自 data
 *          缓冲区 size 之后的一个字节，即源缓冲区需多提供 1 字节有效数据。
 * @warning 写入期间 Flash 被解锁且 CPU 阻塞等待编程完成（Flash 取指停顿），不可在中断
 *          上下文中调用；主循环里的 Motor_ZeroPosition() 也会写 Flash。
 * @todo    Flash 擦除/编程的失败（HAL_FLASHEx_Erase、HAL_FLASH_Program 的返回值）被忽略，
 *          本函数没有返回值，写入失败时上层无法感知。
 */
void EEPROM_Write(uint32_t addr, void *data, uint32_t size)
{
    if (addr + size > EEPROM_MAX_SIZE)
        return;

    /* 第一次写入时，先擦除整个分区 */
    if (!s_is_erased)
    {
        Stockpile_Flash_Data_Empty(&stockpile_data);
        s_is_erased = true;
    }

    /* 开始写入 */
    Stockpile_Flash_Data_Begin(&stockpile_data);

    /* 设置写地址 */
    Stockpile_Flash_Data_Set_Write_Add(&stockpile_data, EEPROM_BASE_ADDR + addr);

    /* 写入数据（16位对齐） */
    uint32_t halfword_count = (size + 1) / 2;
    Stockpile_Flash_Data_Write_Data16(&stockpile_data, (uint16_t *)data, halfword_count);

    /* 结束写入 */
    Stockpile_Flash_Data_End(&stockpile_data);
}

/**
 * @brief   检查模拟 EEPROM 中是否已有数据
 * @details 直接读取分区首地址处的第一个 32 位字，判断其是否等于 0xFFFFFFFF
 *          （擦除后的空 Flash 值）。
 * @return  首字不等于 0xFFFFFFFF 时返回 true；等于 0xFFFFFFFF 时返回 false。
 * @note    判据只有首字：若数据从未从偏移 0 开始写入，非空分区也会被判为空。返回值只表示
 *          "分区被写过"，不代表内容合法（合法性由上层检查 BoardConfig_t.configStatus）。
 *          读 Flash 无需解锁，可在任意上下文调用。
 */
bool EEPROM_IsValid(void)
{
    uint32_t *pFirst = (uint32_t *)EEPROM_BASE_ADDR;

    /* 检查第一个字是否全0xFF（空Flash） */
    return (*pFirst != 0xFFFFFFFF);
}

/**
 * @brief   手动擦除整个模拟 EEPROM 分区
 * @details 调用 Stockpile_Flash_Data_Empty() 擦除整个 APP_DATA 分区（1 页 = 1024 字节），
 *          并把 s_is_erased 置 true，使后续 EEPROM_Write 不再重复擦除。
 * @note    擦除后分区内全为 0xFF，EEPROM_IsValid() 会返回 false；本函数未在 eeprom.h 中
 *          声明（仅在本文件内定义），当前工程内没有其它调用者。
 * @warning 阻塞执行且期间 Flash 被解锁，不可在中断上下文中调用。
 * @todo    擦除失败（HAL_FLASHEx_Erase 的返回值）被忽略，本函数没有返回值与错误上报。
 */
void EEPROM_Erase(void)
{
    Stockpile_Flash_Data_Empty(&stockpile_data);
    s_is_erased = true;
}
