/**
 ******************************************************************************
 * @file    tb67h450.c
 * @brief   TB67H450 双 H 桥步进驱动芯片的 FOC 电流矢量输出实现
 * @details 以 "PWM 当 DAC 设定每相电流幅值 + GPIO 设定每相电流方向" 的方式输出矢量：
 *          由电角度(0~1023)查 sin_pi_m2 正弦表得到 A/B 相正弦值；把电流(mA)换算成
 *          12 位 DAC 值；取正弦绝对值与 DAC 值相乘得到各相占空比；再写入 TIM2
 *          CH3/CH4(PB10/PB11)比较寄存器，并按正弦符号设置 AP/AM、BP/BM 方向脚。
 * @note    由 20kHz(50us) 控制中断经 Motor_Tick20kHz() -> CalcCurrentToOutput() 周期
 *          调用，函数内只做整数运算与寄存器写入，不做阻塞等待。
 ******************************************************************************
 */

#include "tb67h450.h"
#include "gpio.h"
#include "main.h"
#include "sin_form.h"
#include "tim.h"

/**
 * @brief 单相 FOC 输出的中间状态
 * @details 分别保存 A 相与 B 相在本控制周期内查表与换算的中间结果。
 */
typedef struct
{
    uint16_t sinMapPtr;      /**< 正弦表索引：0~1023，一个电周期对应 1024 个刻度 */
    int16_t sinMapData;      /**< 查表值：定标 2^12(±4096 对应 ±1.0)，符号表示电流方向 */
    uint16_t dacValue12Bits; /**< 该相占空比寄存器值：12 位(0~4095)，值越大电流幅值越大 */
} Phase_t;

/* 静态变量 */
static Phase_t s_phaseA; /**< A 相状态：正弦表索引、查表值与占空比寄存器值 */
static Phase_t s_phaseB; /**< B 相状态：正弦表索引、查表值与占空比寄存器值 */

/**
 * @brief  把两相占空比寄存器值写入 TIM2 的 CH3/CH4 比较寄存器
 * @param[in] currentA  A 相 12 位占空比寄存器值(0~4095)
 * @param[in] currentB  B 相 12 位占空比寄存器值(0~4095)
 * @note   TIM2 自动重装载值为 1023，比较寄存器有效范围 0~1023，故先右移 2 位
 *         (>>2)把 12 位值压缩到 10 位再写入；本函数不改变方向引脚。
 */
static void TB67H450_SetTwoCoilsCurrent(uint16_t currentA, uint16_t currentB)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, currentA >> 2);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, currentB >> 2);
}

/**
 * @brief  设置 A 相两个方向控制引脚
 * @param[in] statusAp  true 时 AP(PA2)输出高电平
 * @param[in] statusAm  true 时 AM(PA3)输出高电平
 * @note   AP/AM 同时为低表示该相 H 桥关闭(输出高阻、线圈无电流)，同时为高表示该相
 *         绕组短接制动；仅在两脚电平相反时该相才按占空比输出电流。
 */
static void TB67H450_SetInputA(bool statusAp, bool statusAm)
{
    if (statusAp)
    {
        HAL_GPIO_WritePin(GPIOA, HW_ELEC_AP_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, HW_ELEC_AP_Pin, GPIO_PIN_RESET);
    }

    if (statusAm)
    {
        HAL_GPIO_WritePin(GPIOA, HW_ELEC_AM_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, HW_ELEC_AM_Pin, GPIO_PIN_RESET);
    }
}

/**
 * @brief  设置 B 相两个方向控制引脚
 * @param[in] statusBp  true 时 BP(PA4)输出高电平
 * @param[in] statusBm  true 时 BM(PA5)输出高电平
 * @note   BP/BM 同时为低表示该相 H 桥关闭(输出高阻、线圈无电流)，同时为高表示该相
 *         绕组短接制动；仅在两脚电平相反时该相才按占空比输出电流。
 */
static void TB67H450_SetInputB(bool statusBp, bool statusBm)
{
    if (statusBp)
    {
        HAL_GPIO_WritePin(GPIOA, HW_ELEC_BP_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, HW_ELEC_BP_Pin, GPIO_PIN_RESET);
    }

    if (statusBm)
    {
        HAL_GPIO_WritePin(GPIOA, HW_ELEC_BM_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, HW_ELEC_BM_Pin, GPIO_PIN_RESET);
    }
}

/**
 * @brief  输出 FOC 电流矢量(更新两相 PWM 占空比与方向脚)
 * @details 处理流程：
 *          1) 由电角度算两相正弦表指针：B 相取 _directionInCount & 0x3FF，A 相在 B 相
 *             基础上 +256 后再 & 0x3FF(1024 刻度对应电周期一圈，256 即超前 90 度)；
 *          2) 查 sin_pi_m2 正弦表得到两相正弦值(±4096 对应 ±1.0)；
 *          3) 电流取绝对值后换算成 12 位 DAC 值：dac_reg = (current_abs * 5083) >> 12，
 *             等价于 ×4095/3300，把 3300mA 映射到 4095 满量程，再与 0xFFF 相与；
 *          4) 正弦取绝对值与 dac_reg 相乘并右移 sin_pi_m2_dpiybit(12)位，得到各相占空比
 *             寄存器值(12 位，0~4095)；
 *          5) 写入 TIM2 CH3/CH4 比较寄存器(函数内部还会 >>2 压缩到 TIM2 的 10 位量程)，
 *             并按正弦符号设置 AP/AM、BP/BM 方向脚：正弦为正 -> P 脚高，为负 -> M 脚高，
 *             为零 -> 两脚同时为高(该相绕组短接，不输出电流)。
 * @param[in] _directionInCount  电角度输入(一个电周期 = 1024 个刻度)，内部只使用其低 10 位
 * @param[in] _current_mA        电流幅值，单位 mA(如 1000 = 1A)；只使用其绝对值，
 *                               电流方向由调用方通过电角度偏移(如 ±256 即超前/滞后 90 度)体现
 * @warning "PWM 当 DAC" 的电流换算把 3300mA 固定映射到 4095 满量程(系数 5083 >> 12 ≈ 1.24，
 *          即约 1.24 个 DAC 计数/mA)，该比例与实际采样电阻、母线电压、以及占空比到相电流
 *          的对应关系尚未在代码中校准，需与硬件设计核对后再确定电流定标。
 * @note   由 20kHz 控制中断周期调用；sinMapData 为 0 时对应相两方向脚同时置高。
 * @note   调用方(motor.c)传入的是细分步位置，低 10 位即一个电周期(1024 刻度，一圈 50 个)。
 */
void TB67H450_SetFocCurrentVector(uint32_t _directionInCount, int32_t _current_mA)
{
    uint32_t dac_reg;
    int32_t current_abs;

    /* 1. 计算A/B相的正弦表指针：B相取低10位，A相再+256(超前90度) */
    s_phaseB.sinMapPtr = _directionInCount & 0x000003FF;
    s_phaseA.sinMapPtr = (s_phaseB.sinMapPtr + 256) & 0x000003FF;

    /* 2. 查正弦表：得到定标 2^12 的两相正弦值 */
    s_phaseA.sinMapData = sin_pi_m2[s_phaseA.sinMapPtr];
    s_phaseB.sinMapData = sin_pi_m2[s_phaseB.sinMapPtr];

    /* 3. 计算DAC值：电流(mA)换算成12位占空比寄存器满量程值 */
    current_abs = (_current_mA > 0) ? _current_mA : -_current_mA;
    // 电流(mA) 转 DAC值 (0-4095)
    // 公式: DAC = 电流 × (4095 / 3300) ≈ 电流 × 1.24
    // 5083 >> 12 = 5083 / 4096 ≈ 1.24
    dac_reg = (uint32_t)(current_abs * 5083) >> 12;
    dac_reg = dac_reg & 0x00000FFF;

    /* 取绝对值再乘：正弦幅值(定标2^12) × DAC值，再右移2^12位归一化 */
    int16_t absA = (s_phaseA.sinMapData > 0) ? s_phaseA.sinMapData : -s_phaseA.sinMapData;
    int16_t absB = (s_phaseB.sinMapData > 0) ? s_phaseB.sinMapData : -s_phaseB.sinMapData;

    s_phaseA.dacValue12Bits = (uint32_t)(dac_reg * absA) >> sin_pi_m2_dpiybit;
    s_phaseB.dacValue12Bits = (uint32_t)(dac_reg * absB) >> sin_pi_m2_dpiybit;

    /* 4. 设置PWM占空比：写入TIM2 CH3/CH4比较寄存器 */
    TB67H450_SetTwoCoilsCurrent(s_phaseA.dacValue12Bits, s_phaseB.dacValue12Bits);

    /* 5. 设置方向引脚：正弦为正->P脚高，为负->M脚高，为0->两脚同时为高 */
    if (s_phaseA.sinMapData > 0)
    {
        TB67H450_SetInputA(true, false);
    }
    else if (s_phaseA.sinMapData < 0)
    {
        TB67H450_SetInputA(false, true);
    }
    else
    {
        TB67H450_SetInputA(true, true);
    }

    if (s_phaseB.sinMapData > 0)
    {
        TB67H450_SetInputB(true, false);
    }
    else if (s_phaseB.sinMapData < 0)
    {
        TB67H450_SetInputB(false, true);
    }
    else
    {
        TB67H450_SetInputB(true, true);
    }
}

/**
 * @brief  进入休眠：两相断电，电机不输出转矩
 * @details 两相 PWM 比较值清零(占空比 0)，AP/AM、BP/BM 四脚全部置低，两相 H 桥输出
 *          高阻(休眠状态)，线圈中无电流。
 * @note   由 20kHz 中断在停机/失步/软关断等状态下调用。
 */
void TB67H450_Sleep(void)
{
    TB67H450_SetTwoCoilsCurrent(0, 0);
    TB67H450_SetInputA(false, false);
    TB67H450_SetInputB(false, false);
}

/**
 * @brief  进入刹车：两相绕组短接制动
 * @details 两相 PWM 比较值清零(占空比 0)，AP/AM、BP/BM 四脚全部置高，两相 H 桥进入
 *          制动(Short Brake)状态，绕组被短接，电机转动时被被动制动。
 * @note   由 20kHz 中断在软刹车状态下调用；与 TB67H450_Sleep() 的区别仅在于方向脚电平。
 */
void TB67H450_Brake(void)
{
    TB67H450_SetTwoCoilsCurrent(0, 0);
    TB67H450_SetInputA(true, true);
    TB67H450_SetInputB(true, true);
}
