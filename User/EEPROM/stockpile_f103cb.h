/**
 ******************************************************************************
 * @file    stockpile_f103cb.h
 * @brief   STM32F103CB 片内 Flash 分区读写的接口声明与页大小配置。
 * @details 定义分区表结构体 Stockpile_FLASH_Typedef（起始地址 / 区域大小 / 页数量 / 当前写地址）
 *          与三个分区实例，并声明整片擦除、开始写入、设置写地址、结束写入以及按 16/32/64 位
 *          连续写入的底层原语。分区地址与容量取自 stockpile_config.h；eeprom.c 用
 *          stockpile_data 作存储介质，encoder_calibrator.c 用 stockpile_quick_cali 写校准表，
 *          上层因此无需直接接触 HAL 的 Flash 接口。地址单位均为字节（片内 Flash 绝对地址），
 *          page_num 为 1KB 页的页数。
 * @note    本模块不做互斥保护：写操作要求调用者自行配对 Begin()/End()，并保证同一时刻只有
 *          一个使用者；应用侧调用点见 eeprom.c 与 encoder_calibrator.c。
 ******************************************************************************
 */

#ifndef STOCKPILE_F103CB_H
#define STOCKPILE_F103CB_H

#ifdef __cplusplus
extern "C"
{
#endif

// 引用端口定义
#include "gpio.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
// 应用存储配置
#include "stockpile_config.h"

/*************************************************************** FLASH_Start
 * ***************************************************************/
/*************************************************************** FLASH_Start
 * ***************************************************************/
/*************************************************************** FLASH_Start
 * ***************************************************************/
/******************页配置(更换芯片必须修改这个配置)***********************/
/**
 * @brief   Flash 物理页（扇区）大小
 * @details 0x400U = 1024 字节 = 1KB。STM32F103xB 属中容量器件，HAL 的 FLASH_PAGE_SIZE
 *          （stm32f1xx_hal_flash_ex.h）同样为 0x400U；紧随其后的 \#if 在两者不一致时触发
 *          \#error "Stockpile_Page_Size Error !!!"，用于更换芯片/容量系列后强制复核本配置。
 * @note    分区容量必须是本值的整数倍，否则实例初始化时的 area_size / Stockpile_Page_Size
 *          会因整除而丢掉不足一页的尾部空间。
 */
#define Stockpile_Page_Size 0x400U           /**< 页(扇区)大小：1024 字节，需与 HAL FLASH_PAGE_SIZE 一致 */
#if (Stockpile_Page_Size != FLASH_PAGE_SIZE) // 和HAL库获取的Flash页大小比较,检查配置是否有效
#error "Stockpile_Page_Size Error !!!"
#endif

    /**
     * @brief Flash 分区表：描述一个 Flash 分区及其当前写入位置
     * @note  begin_add / area_size 是配置量（定义实例时由 stockpile_config.h 的分区宏给出），
     *        page_num 由 area_size / Stockpile_Page_Size 算出，asce_write_add 是运行期过程量。
     */
    typedef struct
    {
        // 配置
        uint32_t begin_add; /**< 分区起始地址：片内 Flash 绝对地址，单位：字节 */
        uint32_t area_size; /**< 分区容量，单位：字节 */
        uint32_t page_num;  /**< 分区占用的物理页数，单位：页(1 页 = Stockpile_Page_Size) */
        // 过程量
        uint32_t asce_write_add; /**< 当前写地址(过程量)，单位：字节 */
    } Stockpile_FLASH_Typedef;

    /********** Flash分区表实例 **********/
    /**
     * @brief 固件程序区分区实例（APP_FIRMWARE）
     * @note  起始 0x08000000、容量 0xBC00(47KB)、47 页；asce_write_add 初值 0。
     *        本工程未在存储模块中读写该分区。
     */
    extern Stockpile_FLASH_Typedef stockpile_app_firmware;
    /**
     * @brief 编码器校准表区实例（APP_CALI）
     * @note  起始 0x08007C00、容量 0x8000(32KB)、32 页；由 encoder_calibrator.c 的
     *        GenerateTable() 擦除并写入 16384 个 uint16 校准点。
     */
    extern Stockpile_FLASH_Typedef stockpile_quick_cali;
    /**
     * @brief 用户配置区实例（APP_DATA）
     * @note  起始 0x0800FC00、容量 0x400(1KB)、1 页；由 eeprom.c 作为模拟 EEPROM 的介质。
     */
    extern Stockpile_FLASH_Typedef stockpile_data;

    /**
     * @brief   擦除整个分区
     * @details 逐页擦除 page_num 个物理页（stockpile 为分区表实例，用其 begin_add 与 page_num
     *          决定擦除范围），每页固定 Stockpile_Page_Size(1KB)；擦除后分区内全为 0xFF。
     *          实现见 stockpile_f103cb.c。
     * @note    内部自动 HAL_FLASH_Unlock()/HAL_FLASH_Lock()；阻塞执行，页数越多耗时越长。
     */
    void Stockpile_Flash_Data_Empty(Stockpile_FLASH_Typedef *stockpile);
    /**
     * @brief   开始写入：解锁 Flash 并把写地址复位到分区起始地址
     * @details 传入的分区表实例的 asce_write_add 会被置为 begin_add。
     * @note    需与 Stockpile_Flash_Data_End() 配对；调用后 Flash 处于解锁状态，
     *          且会覆盖此前用 Set_Write_Add 设置的写地址（应先 Begin 再 Set_Write_Add）。
     */
    void Stockpile_Flash_Data_Begin(Stockpile_FLASH_Typedef *stockpile);
    /**
     * @brief   结束写入：重新锁定 Flash
     * @details 实现只有一句 HAL_FLASH_Lock()，不使用传入的分区表实例。
     * @note    与 Stockpile_Flash_Data_Begin() 配对调用。
     */
    void Stockpile_Flash_Data_End(Stockpile_FLASH_Typedef *stockpile);
    /**
     * @brief   设置后续写入的起始地址
     * @details 传入的 write_add 为目标写地址（片内 Flash 绝对地址，单位：字节），仅在闭区间
     *          [begin_add, begin_add + area_size] 内被接受，越界时直接返回、不修改 asce_write_add。
     * @note    必须在 Stockpile_Flash_Data_Begin() 之后调用，否则会被 Begin 复位为 begin_add；
     *          本函数不做对齐检查，地址与写入宽度的匹配由调用者保证。
     */
    void Stockpile_Flash_Data_Set_Write_Add(Stockpile_FLASH_Typedef *stockpile, uint32_t write_add);
    /**
     * @brief   以半字（16 位）为单位连续写入数据
     * @details 参数为分区表实例、半字缓冲区 data 与半字个数 num（1 个半字 = 2 字节）；先检查
     *          写地址与写入范围是否在分区内，任一不满足即整体返回、一个字节都不写；随后逐个
     *          调用 HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD)，实现见 stockpile_f103cb.c。
     * @note    写入前对应区域必须已擦除（Flash 只能把 1 写为 0）；需先 Begin() 解锁。
     */
    void Stockpile_Flash_Data_Write_Data16(Stockpile_FLASH_Typedef *stockpile, uint16_t *data,
                                           uint32_t num); // Flash_16位数据写入
    /**
     * @brief   以字（32 位）为单位连续写入数据
     * @details 参数与边界检查同 Write_Data16，长度按 num*4 字节判断；逐字调用
     *          HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD)。
     * @note    写入前对应区域必须已擦除；需先 Begin() 解锁。
     */
    void Stockpile_Flash_Data_Write_Data32(Stockpile_FLASH_Typedef *stockpile, uint32_t *data,
                                           uint32_t num); // Flash_32位数据写入
    /**
     * @brief   以双字（64 位）为单位连续写入数据
     * @details 参数与边界检查同 Write_Data16，长度按 num*8 字节判断；调用
     *          HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD)。
     * @note    F103 的 HAL 实现把该类型当作 4 个半字依次编程（见 stockpile_f103cb.c 说明）；
     *          写入前对应区域必须已擦除，需先 Begin() 解锁。
     */
    void Stockpile_Flash_Data_Write_Data64(Stockpile_FLASH_Typedef *stockpile, uint64_t *data,
                                           uint32_t num); // Flash_64位数据写入

    /*************************************************************** FLASH_End
     * ***************************************************************/
    /*************************************************************** FLASH_End
     * ***************************************************************/
    /*************************************************************** FLASH_End
     * ***************************************************************/

#ifdef __cplusplus
}
#endif

#endif
