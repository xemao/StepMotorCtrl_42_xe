/*****
 ** @file     : stockpile_f103cb.c/h
 ** @brief    : Flash存储库
 ** @versions : newest
 ** @time     : newest
 ** @reviser  : unli (WuHu China)
 ** @explain  : null
 *****/

/**
 ******************************************************************************
 * @file    stockpile_f103cb.c
 * @brief   STM32F103CB 片内 Flash 分区读写的底层实现（整片擦除与按位宽编程）。
 * @details 三个分区实例分别对应 stockpile_config.h 中的固件程序区(APP_FIRMWARE)、编码器
 *          校准表区(APP_CALI)与用户配置区(APP_DATA)，page_num 由容量除以 1KB 页大小得到。
 *          写入流程固定为：Data_Empty() 擦除 → Data_Begin() 解锁并复位写地址 →
 *          Set_Write_Add() 定位（可选）→ Write_Data16/32/64() 编程 → Data_End() 上锁。
 *          所有地址为片内 Flash 绝对地址（单位：字节），半字/字/双字写入的长度分别按
 *          2 / 4 / 8 字节累加 asce_write_add。上层 eeprom.c 与 encoder_calibrator.c 都经由
 *          本模块访问 Flash，不直接调用 HAL。
 * @note    本模块不做互斥与错误上报：HAL 的擦除/编程返回值被忽略，越界写入只会被静默丢弃；
 *          写操作期间 Flash 处于解锁状态且 CPU 取指停顿，不应在中断上下文调用。
 ******************************************************************************
 */

// Oneself
#include "stockpile_f103cb.h"

/*************************************************************** Flash_Start
 * ***************************************************************/
/*************************************************************** Flash_Start
 * ***************************************************************/
/*************************************************************** Flash_Start
 * ***************************************************************/
// Flash分区表实例
/**
 * @brief 固件程序区分区实例（APP_FIRMWARE）
 * @note  起始地址 0x08000000、容量 0xBC00(47KB = 48128 字节)、page_num = 47 页；
 *        asce_write_add 初值 0，只有经 Begin() 后才会被赋值。
 */
Stockpile_FLASH_Typedef stockpile_app_firmware = {STOCKPILE_APP_FIRMWARE_ADDR, STOCKPILE_APP_FIRMWARE_SIZE,
                                                  (STOCKPILE_APP_FIRMWARE_SIZE / Stockpile_Page_Size), 0};
/**
 * @brief 编码器校准表区实例（APP_CALI）
 * @note  起始地址 0x08007C00、容量 0x8000(32KB = 32768 字节)、page_num = 32 页；
 *        由 encoder_calibrator.c 的 GenerateTable() 擦除并写入校准表。
 */
Stockpile_FLASH_Typedef stockpile_quick_cali = {STOCKPILE_APP_CALI_ADDR, STOCKPILE_APP_CALI_SIZE,
                                                (STOCKPILE_APP_CALI_SIZE / Stockpile_Page_Size), 0};
/**
 * @brief 用户配置区实例（APP_DATA）
 * @note  起始地址 0x0800FC00、容量 0x400(1KB = 1024 字节)、page_num = 1 页；
 *        由 eeprom.c 作为模拟 EEPROM 的存储介质使用。
 */
Stockpile_FLASH_Typedef stockpile_data = {STOCKPILE_APP_DATA_ADDR, STOCKPILE_APP_DATA_SIZE,
                                          (STOCKPILE_APP_DATA_SIZE / Stockpile_Page_Size), 0};

/**
 * @brief   擦除整个 Flash 分区（整片清空为 0xFF）
 * @details 按页循环 page_num 次：以 begin_add + count*Stockpile_Page_Size 为页起始地址、
 *          NbPages = 1 调用 HAL_FLASHEx_Erase 擦除一个物理页，随后等待操作结束并清除
 *          FLASH_CR 的 PER 位。
 * @param[in]  stockpile  Flash 分区表实例，用其 begin_add 与 page_num 决定擦除范围
 * @note    函数内部自动 HAL_FLASH_Unlock()/HAL_FLASH_Lock()，返回时 Flash 已重新上锁；
 *          阻塞执行，耗时随页数线性增长（47 页的固件区最慢），不可在中断上下文中调用。
 * @todo    擦除失败（HAL_FLASHEx_Erase 返回非 HAL_OK、FLASH_WaitForLastOperation 超时）时
 *          没有任何上报，调用者无法判断分区是否真的被清空。
 */
void Stockpile_Flash_Data_Empty(Stockpile_FLASH_Typedef *stockpile)
{
    uint32_t count;
    HAL_FLASH_Unlock();
    for (count = 0; count < stockpile->page_num; count++)
    {
        FLASH_EraseInitTypeDef erase_config;
        uint32_t page_error;
        erase_config.TypeErase = FLASH_TYPEERASE_PAGES;                                  // 页擦除
        erase_config.PageAddress = stockpile->begin_add + (count * Stockpile_Page_Size); // 页起始地址
        erase_config.NbPages = 1;                                                        // 擦除页数量
        HAL_FLASHEx_Erase(&erase_config, &page_error);
        FLASH_WaitForLastOperation(HAL_MAX_DELAY);
        CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
    }
    HAL_FLASH_Lock();
}

/**
 * @brief   开始写入：解锁 Flash 并把写地址复位到分区起始地址
 * @param[in]  stockpile  Flash 分区表实例，其 asce_write_add 被置为 begin_add
 * @note    与 Stockpile_Flash_Data_End() 配对使用；调用后 Flash 保持解锁状态，应尽快写完并
 *          调用 End() 上锁；本函数会覆盖此前的写地址，故 Set_Write_Add() 必须在其之后调用。
 */
void Stockpile_Flash_Data_Begin(Stockpile_FLASH_Typedef *stockpile)
{
    HAL_FLASH_Unlock();
    stockpile->asce_write_add = stockpile->begin_add;
}

/**
 * @brief   结束写入：重新锁定 Flash
 * @param[in]  stockpile  Flash 分区表实例（本函数不使用其成员）
 * @note    与 Stockpile_Flash_Data_Begin() 配对；即使没有先调用 Begin() 也会执行
 *          HAL_FLASH_Lock()，因此重复调用是安全的。
 */
void Stockpile_Flash_Data_End(Stockpile_FLASH_Typedef *stockpile) { HAL_FLASH_Lock(); }

/**
 * @brief   设置后续写入的起始地址
 * @details 边界检查采用闭区间 [begin_add, begin_add + area_size]：小于 begin_add 或大于
 *          begin_add + area_size 时直接返回，asce_write_add 保持原值不变。
 * @param[in]  stockpile  Flash 分区表实例
 * @param[in]  write_add  目标写地址（片内 Flash 绝对地址，单位：字节）
 * @note    必须在 Stockpile_Flash_Data_Begin() 之后调用，否则会被 Begin 复位为 begin_add；
 *          本函数不做对齐检查；取值为分区末尾地址（最后一字节之后）时也会被接受，
 *          后续写入仍会被 Write_Data16/32/64 的边界检查拦下。
 */
void Stockpile_Flash_Data_Set_Write_Add(Stockpile_FLASH_Typedef *stockpile, uint32_t write_add)
{
    if (write_add < stockpile->begin_add)
        return;
    if (write_add > stockpile->begin_add + stockpile->area_size)
        return;
    stockpile->asce_write_add = write_add;
}

/**
 * @brief   以半字（16 位）为单位连续写入数据
 * @details 先做两项边界检查：asce_write_add 不得小于 begin_add，写入范围
 *          asce_write_add + num*2 不得超过 begin_add + area_size；任一不满足即整体返回、
 *          一个字节都不写。之后循环调用 HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD)，
 *          只有返回 HAL_OK 才把 asce_write_add 前移 2 字节；失败时地址不前进，
 *          循环继续，后续数据会重复写到同一地址上。
 * @param[in]  stockpile  Flash 分区表实例，提供写地址与分区边界
 * @param[in]  data       待写入的半字缓冲区首地址（按 uint16_t 读取）
 * @param[in]  num        半字个数，1 个半字 = 2 字节
 * @note    写入前对应区域必须已擦除（Flash 只能把 1 写为 0）；需在 Begin() 解锁之后调用，
 *          且不可在中断上下文中使用。
 * @todo    单个半字编程失败（地址未对齐、目标未擦除导致 PGERR 等）时没有错误上报，
 *          函数继续执行且不返回失败信息，上层无法感知数据缺失。
 */
void Stockpile_Flash_Data_Write_Data16(Stockpile_FLASH_Typedef *stockpile, uint16_t *data, uint32_t num)
{
    if (stockpile->asce_write_add < stockpile->begin_add)
        return;
    if ((stockpile->asce_write_add + num * 2) > stockpile->begin_add + stockpile->area_size)
        return;

    for (uint32_t i = 0; i < num; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, stockpile->asce_write_add, (uint64_t)data[i]) == HAL_OK)
            stockpile->asce_write_add += 2;
    }
}

/**
 * @brief   以字（32 位）为单位连续写入数据
 * @details 边界检查同 Write_Data16，按 num*4 字节判断；随后循环调用
 *          HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD)，成功一次则把 asce_write_add 前移
 *          4 字节，失败则不前进（与 Write_Data16 行为一致）。
 * @param[in]  stockpile  Flash 分区表实例，提供写地址与分区边界
 * @param[in]  data       待写入的字缓冲区首地址（按 uint32_t 读取）
 * @param[in]  num        字个数，1 个字 = 4 字节
 * @note    写入前对应区域必须已擦除；需在 Begin() 解锁之后调用，不可在中断上下文中使用。
 * @todo    编程失败时没有错误上报，上层无法感知数据缺失。
 */
void Stockpile_Flash_Data_Write_Data32(Stockpile_FLASH_Typedef *stockpile, uint32_t *data, uint32_t num)
{
    if (stockpile->asce_write_add < stockpile->begin_add)
        return;
    if ((stockpile->asce_write_add + num * 4) > stockpile->begin_add + stockpile->area_size)
        return;

    for (uint32_t i = 0; i < num; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, stockpile->asce_write_add, (uint64_t)data[i]) == HAL_OK)
            stockpile->asce_write_add += 4;
    }
}

/**
 * @brief   以双字（64 位）为单位连续写入数据
 * @details 边界检查同 Write_Data16，按 num*8 字节判断；随后循环调用
 *          HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD)，成功一次则把 asce_write_add
 *          前移 8 字节。注意 F103 的 HAL 实现中 DOUBLEWORD 走 else 分支，实际是把数据拆成
 *          4 个半字依次编程并逐个等待完成（见 stm32f1xx_hal_flash.c），并非硬件 64 位写入。
 * @param[in]  stockpile  Flash 分区表实例，提供写地址与分区边界
 * @param[in]  data       待写入的双字缓冲区首地址（按 uint64_t 读取）
 * @param[in]  num        双字个数，1 个双字 = 8 字节
 * @note    写入前对应区域必须已擦除；需在 Begin() 解锁之后调用，不可在中断上下文中使用。
 * @todo    编程失败时没有错误上报，上层无法感知数据缺失。
 */
void Stockpile_Flash_Data_Write_Data64(Stockpile_FLASH_Typedef *stockpile, uint64_t *data, uint32_t num)
{
    if (stockpile->asce_write_add < stockpile->begin_add)
        return;
    if ((stockpile->asce_write_add + num * 8) > stockpile->begin_add + stockpile->area_size)
        return;

    for (uint32_t i = 0; i < num; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, stockpile->asce_write_add, (uint64_t)data[i]) == HAL_OK)
            stockpile->asce_write_add += 8;
    }
}
/*************************************************************** Flash_End
 * ***************************************************************/
/*************************************************************** Flash_End
 * ***************************************************************/
/*************************************************************** Flash_End
 * ***************************************************************/
