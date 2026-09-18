/**
 ******************************************************************************
 * @file    mt6816.c
 * @brief   MT6816 磁编码器 SPI 读取与校准表线性化的实现
 * @details 通过 SPI1 按 16 位字收发读取 14 位绝对角度，时序为：拉低片选 PA15 ->
 *          HAL_SPI_TransmitReceive 收发 1 个 16 位字 -> 拉高片选。命令字为
 *          0x80 | 命令码，最高位为 1 表示读；读一次角度需依次发送 0x03(读角度)与
 *          0x04(读原始角度)，把两次返回数据的低 8 位拼成 16 位原始数据，再做奇偶
 *          校验(16 位中 1 的个数为偶数则通过)，最多重试 3 次；校验通过后右移 2 位
 *          取出 14 位角度(0~16383)，最后用 Flash 校准表线性化得到校准角度。
 * @note    主要在 20kHz(50us) 控制中断 Motor_Tick20kHz() 中被调用，SPI 收发使用
 *          HAL_MAX_DELAY 阻塞等待，实时性敏感。
 ******************************************************************************
 */

#include "mt6816.h"
#include "main.h"
#include "spi.h"

/* SPI CS引脚 */
#define MT6816_CS_GPIO_Port SPI1_CS_GPIO_Port /**< 片选引脚端口，取自 main.h 的 SPI1_CS_GPIO_Port(GPIOA) */
#define MT6816_CS_Pin       SPI1_CS_Pin       /**< 片选引脚号，取自 main.h 的 SPI1_CS_Pin(PA15) */

/* 命令定义 */
#define MT6816_CMD_ANGLE     0x03 /**< 读取角度寄存器：返回 14 位角度、无磁场标志与校验位 */
#define MT6816_CMD_RAW_ANGLE 0x04 /**< 读取原始角度寄存器：返回未经内部线性化的 14 位角度 */

/**
 * @brief SPI 一帧收发结果的数据结构
 * @details 保存一次读角度流程解析出的 16 位原始数据及由它提取的标志位。
 */
typedef struct
{
    uint16_t raw_data;  /**< SPI 原始 16 位数据：两次读命令返回字节拼接而成 (bit15~bit0) */
    uint16_t raw_angle; /**< 14 位原始角度：raw_data 右移 2 位得到，取值 0~16383，单位：刻度/圈 */
    bool no_mag_flag;   /**< 无磁场标志：raw_data 的 bit1 为 1 表示磁铁未就位/磁场过弱 */
    bool checksum_flag; /**< 校验标志：true 表示本次读到的 raw_data 奇偶校验通过 */
} MT6816_SpiData_t;

/* 静态变量 */
static uint16_t *s_quick_cali_data_ptr = NULL; /**< 校准数据指针：16384 项校准表，索引为原始角度，NULL 未设置 */
static MT6816_SpiData_t s_spi_raw_data;        /**< 最近一次 SPI 读取的原始数据与标志位 */
static uint16_t s_data_tx[2];                  /**< 发送缓冲：两条读命令字(各占高 8 位) */
static uint16_t s_data_rx[2];                  /**< 接收缓冲：两条读命令的 16 位返回数据 */
// static uint8_t s_hcount;
static uint16_t s_rectified_angle; /**< 最近一次校验通过后查表得到的校准角度，量纲由校准表决定 */
static uint16_t s_raw_angle;       /**< 最近一次校验通过后的 14 位原始角度(0~16383) */

/**
 * @brief  通过 SPI 收发一个 16 位字
 * @details 时序：片选(PA15)拉低选中芯片 -> HAL_SPI_TransmitReceive 收发 1 个 16 位字
 *          -> 片选拉高释放，一次调用对应一帧完整读取。
 * @param[in] data_tx  待发送的 16 位数据(读命令字)
 * @warning 使用 HAL_MAX_DELAY 阻塞等待收发完成，SPI 出错或无响应时会一直等待，而本函数
 *          运行在 20kHz(50us) 控制中断里，可能卡死中断。
 * @return 从 MT6816 接收到的 16 位数据
 * @note   仅可在 20kHz 中断上下文中调用(由 MT6816_UpdateAngle() 内部使用)。
 */
static uint16_t MT6816_SpiTransmitAndRead16Bits(uint16_t data_tx)
{
    uint16_t data_rx;

    /* CS低电平选中 */
    HAL_GPIO_WritePin(MT6816_CS_GPIO_Port, MT6816_CS_Pin, GPIO_PIN_RESET);

    /* SPI收发 */
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&data_tx, (uint8_t *)&data_rx, 1, HAL_MAX_DELAY);

    /* CS高电平释放 */
    HAL_GPIO_WritePin(MT6816_CS_GPIO_Port, MT6816_CS_Pin, GPIO_PIN_SET);

    return data_rx;
}

/**
 * @brief  统计 16 位数据中 1 的个数并返回奇偶性
 * @param[in] data  待校验的 16 位数据
 * @return 1 表示奇数个 1，0 表示偶数个 1(MT6816 要求偶数个 1 才算校验通过)
 * @note   逐位判断 bit0~bit15，属于纯粹的组合逻辑，无副作用。
 */
static uint8_t MT6816_CalcParity(uint16_t data)
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < 16; i++)
    {
        if (data & (0x0001 << i))
            count++;
    }
    return count & 0x01; /* 返回1表示奇数个1，0表示偶数个1 */
}

/**
 * @brief  初始化 MT6816：读取一次角度并检查校准数据是否可用
 * @details 先调用 MT6816_UpdateAngle() 触发一次读取，再检查校准数据：指针为空视为未
 *          设置校准表；指针有效时遍历整张表(共 16384 项)，只要出现 0xFFFF(Flash 擦除
 *          态)就判定校准数据无效。判定条件与 MT6816_IsCalibrated() 完全一致。
 * @return true 表示读取流程可用且校准数据有效；false 表示未设置校准表或表内存在 0xFFFF
 * @note   本函数的判定逻辑与 MT6816_IsCalibrated() 完全一致，二者可合并为同一实现。
 * @todo   该函数未在 mt6816.h 中声明，工程内也没有调用者(链接时被丢弃)，建议补上头文件
 *         声明并统一由上层调用，或直接删除。
 */
bool MT6816_Init(void)
{
    MT6816_UpdateAngle();

    /* 检查校准数据是否有效 */
    if (s_quick_cali_data_ptr == NULL)
    {
        return false;
    }

    for (uint32_t i = 0; i < MT6816_RESOLUTION; i++)
    {
        if (s_quick_cali_data_ptr[i] == 0xFFFF)
        {
            return false;
        }
    }

    return true;
}

/**
 * @brief  读取一次编码器角度并刷新原始角度与校准角度缓存
 * @details 构造两条读命令(0x80 | 命令码，最高位为 1 表示读)后，每轮依次读出角度命令
 *          0x03 与原始角度命令 0x04 的返回数据，取两次返回的低 8 位拼成 16 位原始数据
 *          ((rx0 & 0xFF) << 8 | (rx1 & 0xFF))，再做奇偶校验；校验不通过则最多重试 3 次。
 *          校验通过后：raw_angle = raw_data >> 2 (14 位角度，bit2~bit15)，
 *          no_mag_flag = raw_data 的 bit1；随后查校准表线性化：校准表指针非空时取
 *          s_quick_cali_data_ptr[raw_angle]，为空时直接使用原始角度。
 * @warning SPI 收发使用 HAL_MAX_DELAY 阻塞等待，且校验失败时最多重试 3 次(共 6 帧 SPI)；
 *          本函数在 20kHz(50us) 控制中断中被调用，SPI 异常(总线被占用、器件无响应)时会
 *          显著拖长中断时间，可能破坏控制周期。
 * @warning 3 次校验均失败时不更新缓存，s_rectified_angle 保持上一次的值，上层位置积分会
 *          因这个"旧角度"产生误差。
 * @return 校准后的角度；若 3 次校验均失败，则返回上一次的有效值(首次为 0)
 * @note   仅在 20kHz(50us) 控制中断中周期性调用；读取失败时不更新缓存(返回旧值)。
 * @todo   解析出的无磁场标志 s_spi_raw_data.no_mag_flag 只被赋值、没有上层使用，
 *         磁铁丢失/未安装时不会产生告警，建议在控制层补充校验或上报。
 */
uint16_t MT6816_UpdateAngle(void)
{
    /* 构造命令：最高位为1表示读 */
    s_data_tx[0] = (0x80 | MT6816_CMD_ANGLE) << 8;
    s_data_tx[1] = (0x80 | MT6816_CMD_RAW_ANGLE) << 8;

    /* 尝试3次读取，直到校验通过 */
    for (uint8_t i = 0; i < 3; i++)
    {
        s_data_rx[0] = MT6816_SpiTransmitAndRead16Bits(s_data_tx[0]);
        s_data_rx[1] = MT6816_SpiTransmitAndRead16Bits(s_data_tx[1]);

        /* 组合数据：两次返回数据的低8位拼成16位原始数据 */
        s_spi_raw_data.raw_data = ((s_data_rx[0] & 0x00FF) << 8) | (s_data_rx[1] & 0x00FF);

        /* 奇偶校验：16位中1的个数为偶数则通过 */
        if (MT6816_CalcParity(s_spi_raw_data.raw_data) == 0)
        {
            /* 校验通过（偶数个1） */
            s_spi_raw_data.checksum_flag = true;
            break;
        }
        else
        {
            s_spi_raw_data.checksum_flag = false;
        }
    }

    if (s_spi_raw_data.checksum_flag)
    {
        /* 提取14位角度值（bit2-bit15） */
        s_spi_raw_data.raw_angle = s_spi_raw_data.raw_data >> 2;
        /* 提取无磁场标志（bit1），为1表示磁场异常/磁铁未就位 */
        s_spi_raw_data.no_mag_flag = (bool)(s_spi_raw_data.raw_data & (0x0001 << 1));

        s_raw_angle = s_spi_raw_data.raw_angle;

        /* 查表校准：指针为空时直接使用原始角度 */
        if (s_quick_cali_data_ptr != NULL)
        {
            s_rectified_angle = s_quick_cali_data_ptr[s_raw_angle];
        }
        else
        {
            s_rectified_angle = s_raw_angle;
        }
    }

    return s_rectified_angle;
}

/**
 * @brief  获取上一次读取的 14 位原始角度
 * @return 原始角度(0~16383，未做查表校准)
 * @note   不访问硬件，读取的是 MT6816_UpdateAngle() 的缓存；校验失败时保持旧值。
 */
uint16_t MT6816_GetRawAngle(void) { return s_raw_angle; }

/**
 * @brief  获取上一次读取的校准后角度
 * @warning 返回值直接取自校准表，量纲与范围由表内容决定：未设置校准表时为 0~16383 的
 *          原始角度，设置后不再保证仍在 0~16383(若表按细分步写入，一圈应为 51200)，
 *          本模块不做范围检查。
 * @return 校准后的角度；未设置校准表时等于原始角度
 * @note   不访问硬件，读取的是 MT6816_UpdateAngle() 的缓存。
 */
uint16_t MT6816_GetRectifiedAngle(void) { return s_rectified_angle; }

/**
 * @brief  检查校准数据是否有效
 * @details 指针为空返回 false；否则遍历整张表(共 16384 项)，出现 0xFFFF(Flash 擦除态)
 *          即判定为未校准，全部检查通过才返回 true。
 * @return true 表示已设置校准表且表内无 0xFFFF；false 表示未设置校准表或表内存在 0xFFFF
 * @note   判定条件与 MT6816_Init() 的检查部分一致，需遍历全表，耗时随表长增加。
 */
bool MT6816_IsCalibrated(void)
{
    if (s_quick_cali_data_ptr == NULL)
    {
        return false;
    }

    for (uint32_t i = 0; i < MT6816_RESOLUTION; i++)
    {
        if (s_quick_cali_data_ptr[i] == 0xFFFF)
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief  设置校准数据表指针
 * @param[in] cali_data_ptr  校准表首地址(16384 项 uint16_t)，索引为原始角度(0~16383)，
 *                           元素为对应校准后角度；传入 NULL 表示不做校准
 * @note   仅保存指针，不拷贝数据，因此被指向的表必须长期有效(通常为 Flash 映射的数组)。
 */
void MT6816_SetCalibrationData(uint16_t *cali_data_ptr) { s_quick_cali_data_ptr = cali_data_ptr; }

/**
 * @brief  读取一帧角度数据(不解析、不校验)
 * @details 直接发送 0x03(读角度)命令并返回收到的 16 位数据，时序与内部收发函数一致：
 *          片选拉低 -> 收发 1 个 16 位字 -> 片选拉高。
 * @return SPI 收到的 16 位原始数据(未做奇偶校验，也未提取角度)
 * @note   不更新模块内任何缓存，仅用于 SPI 通信自检；当前工程未调用该函数。
 * @todo   该函数未在工程中被调用(链接时被丢弃)，请确认是保留为调试接口还是移除。
 */
uint16_t MT6816_TestRead(void)
{
    uint16_t data_tx = (0x80 | MT6816_CMD_ANGLE) << 8;
    uint16_t data_rx;

    /* CS低电平选中 */
    HAL_GPIO_WritePin(MT6816_CS_GPIO_Port, MT6816_CS_Pin, GPIO_PIN_RESET);

    /* SPI收发 */
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&data_tx, (uint8_t *)&data_rx, 1, HAL_MAX_DELAY);

    /* CS高电平释放 */
    HAL_GPIO_WritePin(MT6816_CS_GPIO_Port, MT6816_CS_Pin, GPIO_PIN_SET);

    return data_rx;
}
