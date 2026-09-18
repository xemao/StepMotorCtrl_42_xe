/******
 ************************************************************************
 ******
 ** @versions : 1.1.4
 ** @time     : 2020/09/15
 ******
 ************************************************************************
 ******
 ** @project : XDrive_Step
 ** @brief   : 具有多功能接口和闭环功能的步进电机
 ** @author  : unlir (知不知啊)
 ******
 ** @address : https://github.com/unlir/XDrive
 ******
 ** @issuer  : IVES ( 艾维斯 实验室) (QQ: 557214000)   (master)
 ** @issuer  : REIN (  知驭  实验室) (QQ: 857046846)   (master)
 ******
 ************************************************************************
 ******
 ** {Stepper motor with multi-function interface and closed Main function.}
 ** Copyright (c) {2020}  {unlir(知不知啊)}
 **
 ** This program is free software: you can redistribute it and/or modify
 ** it under the terms of the GNU General Public License as published by
 ** the Free Software Foundation, either version 3 of the License, or
 ** (at your option) any later version.
 **
 ** This program is distributed in the hope that it will be useful,
 ** but WITHOUT ANY WARRANTY; without even the implied warranty of
 ** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 ** GNU General Public License for more details.
 **
 ** You should have received a copy of the GNU General Public License
 ** along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ******
 ************************************************************************
 ******/

/*****
 ** @file     : stockpile_config.c/h
 ** @brief    : 存储配置
 ** @versions : newest
 ** @time     : newest
 ** @reviser  : unli (HeFei China)
 ** @explain  : null
 *****/

/**
 ******************************************************************************
 * @file    stockpile_config.h
 * @brief   XDrive 存储分区配置：定义片内 Flash 各功能分区的起始地址与容量，以及 RAM 分区。
 * @details 本文件是存储子系统的地址来源：stockpile_f103cb.c 用其中的宏初始化三个 Flash 分区
 *          实例（固件程序区 APP_FIRMWARE / 编码器校准表区 APP_CALI / 用户配置区 APP_DATA），
 *          eeprom.c 则以 STOCKPILE_APP_DATA_ADDR 作为上层 EEPROM 的存储基地址。所有地址均为
 *          STM32F103CB 片内 Flash 的绝对地址（0x08000000 起），容量单位统一为字节。
 *          物理页（扇区）大小为 1KB，由 stockpile_f103cb.h 的 Stockpile_Page_Size 定义，并与
 *          HAL 的 FLASH_PAGE_SIZE 做 \#error 一致性校验（分区容量必须是页大小的整数倍）。
 * @note    修改任何分区地址或容量后，必须保证分区页对齐、互不重叠，并复核校准表长度与
 *          APP_CALI 区容量的耦合关系（见 STOCKPILE_APP_CALI_SIZE 上方的说明）。
 * @warning APP_FIRMWARE 与 APP_CALI 的地址区间重叠：前者容量宏仍为 0xBC00(47KB)，
 *          而后者从 0x08007C00 开始，两个区间交叠 16KB。
 * @warning 按 MDK-ARM 最近一次构建的 map，固件映像已排到 0x08007FC8：
 *          sin_form.o 的 .rodata.sin_pi_m2（0x08007676、长 0x802 字节）跨越了 0x08007C00，
 *          即校准表分区的第一页落在固件映像内部，擦写会破坏该页内容。
 * @note    上述重叠需复核分区表或调整链接空间；本组文件只如实记录地址关系，不做修正。
 ******************************************************************************
 */

/*************************************************************** Stockpile_Start
 * ***************************************************************/
/*************************************************************** Stockpile_Start
 * ***************************************************************/
/*************************************************************** Stockpile_Start
 * ***************************************************************/
/*********************STM32F103xx*************************/
// 主储存块容量
// Flash Size(bytes)/RAM size(bytes)
//  大容量   1M / 96K                                     RG               VG           ZG
//  大容量 768K / 96K                                     RF               VF           ZF
//  大容量 512K / 64K                                     RE               VE           ZE
//  大容量 384K / 64K                                     RD               VD           ZD
//  大容量 256K / 48K                                     RC               VC           ZC
//  中容量 128K / 20K      TB           CB                RB               VB
//  中容量  64K / 20K      T8           C8                R8               V8
//  小容量  32K / 10K      T6           C6                R6
//  小容量  16K /  6K      T4           C4                R4
//         						 36pin-QFN	48pin-LQFP/QFN	64pin-BGA/CSP/LQFP  100pin-LQFP  144pin-BGA/LQFP
/*************************************************************** Stockpile_End
 * ***************************************************************/
/*************************************************************** Stockpile_End
 * ***************************************************************/
/*************************************************************** Stockpile_End
 * ***************************************************************/

#ifndef STOCKPILE_CONFIG_H
#define STOCKPILE_CONFIG_H

/* ROM sizes */
/* ROM sizes */
/* ROM sizes */

// DAPLINK_ROM_BL
#define DAPLINK_ROM_BL_START           (0x08000000) /**< DAPLink 引导程序(Bootloader)区起始地址 */
#define DAPLINK_ROM_BL_SIZE            (0x0000BC00) /**< 容量 48128 字节(47KB)，DAPLink_BL(DAPLINK_ROM_BL) */
// DAPLINK_ROM_CONFIG_ADMIN
#define DAPLINK_ROM_CONFIG_ADMIN_START (0x0800BC00) /**< DAPLink 配置管理区起始地址 */
#define DAPLINK_ROM_CONFIG_ADMIN_SIZE  (0x00000400) /**< 容量 1024 字节(1KB)，DAPLink_BL(DAPLINK_ROM_CONFIG_ADMIN) */
// APP_FIRMWARE
#define STOCKPILE_APP_FIRMWARE_ADDR    (0x08000000) /**< 固件程序区起始地址(原注释另记备用值 0x0800C000) */
#define STOCKPILE_APP_FIRMWARE_SIZE    (0x0000BC00) /**< 固件程序区容量 48128 字节(47KB)，XDrive(APP_FIRMWARE) */
// APP_CALI
/**
 * @brief   编码器校准表区（APP_CALI，起始 0x08007C00，容量 32KB）
 * @details 校准表按 uint16 存放：16384 个点 × 2 字节 = 32768 字节 = 整片分区，
 *          即最多支持 14 位编码器（2^14 = 16384 个机械位置）的校准数据。
 * @note    本区容量与校准数据长度强耦合：表长由 ENC_RESOLUTION(16384) 决定，改动其中
 *          之一必须同步修改另一方，否则会越界或被 Stockpile 的边界检查整批丢弃。
 */
#define STOCKPILE_APP_CALI_ADDR        (0x08007C00) /**< 编码器校准表区起始地址 */
#define STOCKPILE_APP_CALI_SIZE \
    (0x00008000) /**< 校准表区容量 32768 字节(32KB)：存放 16384 个 uint16 校准点(对应 14 位编码器) */
// APP_DATA
#define STOCKPILE_APP_DATA_ADDR (0x0800FC00) /**< 用户配置区起始地址 */
#define STOCKPILE_APP_DATA_SIZE (0x00000400) /**< 用户配置区容量 1024 字节(1KB)，即上层模拟 EEPROM 的容量 */

/* RAM sizes */
/* RAM sizes */
/* RAM sizes */

#define STOCKPILE_RAM_APP_START (0x20000000) /**< 应用程序 RAM 区起始地址 */
#define STOCKPILE_RAM_APP_SIZE  (0x00004F00) /**< 应用程序 RAM 区容量 20224 字节(即原注释的 19K768 字节) */

#define STOCKPILE_RAM_SHARED_START (0x20004F00) /**< 共享 RAM 区起始地址 */
#define STOCKPILE_RAM_SHARED_SIZE  (0x00000100) /**< 共享 RAM 区容量 256 字节 */

#endif
