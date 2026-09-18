/**
 ******************************************************************************
 * @file    motion_planner.c
 * @brief   运动规划实现：4 个 tracker（电流/速度/位置/轨迹）+ 1 个位置插值器
 * @details 每个 tracker 都是"把硬目标平滑成软目标"的梯形/S 形发生器，调用约定统一：
 *          Init() 从 g_motion_config 读取限幅与加速度 → NewTask() 用当前实际值重置轨迹起点 →
 *          CalcSoftGoal() 每个 20kHz 周期推进一步，结果写入 g_go_* / g_traj_* / g_interp_*，
 *          由 motor.c 的控制器当作"软目标"使用。
 *          所有积分器统一采用"累加 → 除以 CONTROL_FREQUENCY(20000) → 余数回填"的整数算法，
 *          把单位是"每秒"的变化率折算成"每周期"的增量，避免整除把小数截断成 0。
 *          内部单位：位置 细分步（51200 细分步/圈）、速度 细分步/秒、电流 mA。
 * @note    全部由 motor.c 在 20kHz（50us）中断中调用，必须保持纯整数运算、无阻塞、无打印；
 *          调用任何 tracker 之前 g_motion_config 必须已由 Motor_Init() 赋值，否则解引用空指针。
 ******************************************************************************
 */

#include "motion_planner.h"
#include "usart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* 配置指针 */
/** @brief 全局运动规划配置指针，由 motor.c 的 Motor_Init() 指向 Motor_Config_t.motionParams */
MotionPlanner_Config_t *g_motion_config = NULL;

/* ==================== CurrentTracker 全局变量 ==================== */
/** @brief 电流变化率（梯形斜率），单位 mA/s，由 CurrentTracker_SetCurrentAcc() 写入 */
static int32_t s_current_acc = 0;
/** @brief 电流积分余数：每周期先累加变化率，再除以 20000 得到增量，除不尽的余数留在这里下周期继续用 */
static int32_t s_current_integral = 0;
/** @brief 电流规划内部跟踪值（当前软目标电流），单位 mA */
static int32_t s_track_current = 0;
/** @brief 电流规划输出：软目标指令电流，单位 mA，每个 20kHz 周期刷新（无实测电流反馈） */
int32_t g_go_current = 0;

/* ==================== VelocityTracker 全局变量 ==================== */
/** @brief 速度规划加速度（梯形斜率），单位 细分步/秒²，由 VelocityTracker_SetVelocityAcc() 写入 */
static int32_t s_velocity_acc = 0;
/** @brief 速度积分余数：每周期累加加速度后除以 20000 得到速度增量，余数留待下周期继续累加 */
static int32_t s_velocity_integral = 0;
/** @brief 速度规划内部跟踪值（当前软目标速度），单位 细分步/秒 */
static int32_t s_track_velocity = 0;
/** @brief 速度规划输出：软目标速度，单位 细分步/秒，每个 20kHz 周期刷新 */
int32_t g_go_velocity = 0;

/* ==================== PositionTracker 全局变量 ==================== */
/** @brief 位置规划的加速斜率，单位 细分步/秒²（与减速斜率取同一数值） */
static int32_t s_velocity_up_acc = 0;
/** @brief 位置规划的减速斜率，单位 细分步/秒² */
static int32_t s_velocity_down_acc = 0;
/** @brief 减速距离系数 = 1/(2a)，单位 s²/细分步；need_down = v² × 本系数，用浮点保存以防整数溢出 */
static float s_quick_velocity_down_acc = 0;
/** @brief 低速锁定刹车阈值，单位 细分步/秒（= ratedVelocityAcc/1000，默认 0.1 圈/秒） */
static int32_t s_speed_locking_brake = 0;
/** @brief 位置规划中速度环的积分余数（除以 20000 后的余数回填） */
static int32_t s_velocity_integral_pos = 0;
/** @brief 位置规划内部跟踪速度（软目标速度），单位 细分步/秒 */
static int32_t s_track_velocity_pos = 0;
/** @brief 位置积分余数：每周期累加跟踪速度后除以 20000 得到位置增量，余数留待下周期 */
static int32_t s_position_integral = 0;
/** @brief 位置规划内部跟踪位置（软目标位置），单位 细分步 */
static int32_t s_track_position = 0;
/** @brief 位置规划输出：软目标位置，单位 细分步，每个 20kHz 周期刷新 */
int32_t g_go_location = 0;
/** @brief 位置规划输出：软目标速度，单位 细分步/秒（同时作为 DCE 的速度前馈目标） */
int32_t g_go_location_velocity = 0;

/* ==================== PositionInterpolator 全局变量 ==================== */
/** @brief 本次收到的目标位置，单位 细分步 */
static int32_t s_record_position = 0;
/** @brief 上一次收到的目标位置，单位 细分步（与本次差分后用于估计速度） */
static int32_t s_record_position_last = 0;
/** @brief 插值器跟踪位置，单位 细分步（直接等于目标位置，不做位置平滑） */
static int32_t s_est_position = 0;
/** @brief 速度估计的泄漏积分累加器：保存 63/64 一阶低通未进位的余数 */
static int32_t s_est_position_integral = 0;
/** @brief 低通滤波后的速度估计，单位 细分步/秒（时间常数 64 个控制周期 ≈ 3.2ms） */
static int32_t s_est_velocity_interp = 0;
/** @brief 插值器输出位置，单位 细分步（当前工程内无调用者读取） */
int32_t g_interp_go_position = 0;
/** @brief 插值器输出速度，单位 细分步/秒（当前工程内无调用者读取） */
int32_t g_interp_go_velocity = 0;

/* ==================== TrajectoryTracker 全局变量 ==================== */
/** @brief 轨迹规划与超时停车使用的减速度，单位 细分步/秒²（来自 ratedVelocityAcc） */
static int32_t s_velocity_down_acc_traj = 0;
/** @brief 目标变化时按 v²-v₀²=2as 反算出的加速度，单位 细分步/秒²（正负表示加速/减速） */
static int32_t s_dynamic_velocity_acc = 0;
/** @brief 距上一次目标变化的累计时间，单位 微秒（每周期加 CONTROL_PERIOD_US） */
static int32_t s_update_time = 0;
/** @brief 指令更新超时阈值，单位 ms（motor.c 传入 200，比较时乘 1000 换算成微秒） */
static int32_t s_update_timeout = 200;
/** @brief 超时标志：为 true 表示上位机长时间没给新轨迹指令，按减速度安全停车 */
static bool s_overtime_flag = false;
/** @brief 上一次记录的目标速度，单位 细分步/秒（与本次比较以判断目标是否变化） */
static int32_t s_record_velocity = 0;
/** @brief 上一次记录的目标位置，单位 细分步（与本次比较以判断目标是否变化） */
static int32_t s_record_position_traj = 0;
/** @brief 加速度积分余数：累加动态加速度后除以 20000 得到速度增量，余数留待下周期 */
static int32_t s_dynamic_vel_acc_remainder = 0;
/** @brief 轨迹规划内部跟踪速度（软目标速度），单位 细分步/秒 */
static int32_t s_velocity_now = 0;
/** @brief 速度积分余数：累加跟踪速度后除以 20000 得到位置增量，余数留待下周期 */
static int32_t s_velocity_now_remainder = 0;
/** @brief 轨迹规划内部跟踪位置（软目标位置），单位 细分步 */
static int32_t s_position_now = 0;
/** @brief 轨迹规划输出：软目标位置，单位 细分步，每个 20kHz 周期刷新 */
int32_t g_traj_go_position = 0;
/** @brief 轨迹规划输出：软目标速度，单位 细分步/秒，每个 20kHz 周期刷新 */
int32_t g_traj_go_velocity = 0;

/* ==================== 辅助函数 ==================== */
/**
 * @brief  电流积分器：把电流变化率（mA/s）折算成每周期增量并累加到软目标电流上
 * @details 定点/单位约定：本函数不含移位，采用"余数回填"的整数积分：
 *          1) s_current_integral += current，累加器单位是 mA/s；
 *          2) s_current_integral / CONTROL_FREQUENCY(20000) 得到本周期的整数增量（mA），
 *             累加到 s_track_current —— 20000 就是控制频率，等于 1s / 50us；
 *          3) s_current_integral %= 20000，把除不尽的余数留回累加器，下周期继续参与整除，
 *             否则每周期 mA/s 量级的斜率会被整除截断成 0 而完全不动。
 *          等效效果：s_track_current 平均每个周期增加 current/20000（mA），即按 current mA/s 的速率爬升。
 * @param[in] current 电流变化率，单位 mA/s，符号决定加或减（调用方按方向传入 ±s_current_acc）
 * @note   只在 20kHz 中断上下文（CurrentTracker_CalcSoftGoal()）中调用，纯整数运算无阻塞。
 * @warning 累加出来的 s_track_current 最终会作为"指令电流"送给 TB67H450，本工程没有实测电流反馈。
 */
static void CalcCurrentIntegral(int32_t current)
{
    s_current_integral += current;
    s_track_current += s_current_integral / CONTROL_FREQUENCY;
    s_current_integral = s_current_integral % CONTROL_FREQUENCY;
}

/**
 * @brief  速度积分器：把加速度（细分步/秒²）折算成每周期增量并累加到软目标速度上
 * @details 与 CalcCurrentIntegral() 完全相同的"余数回填"整数积分：s_velocity_integral 累加加速度，
 *          除以 CONTROL_FREQUENCY(20000) 的商（单位 细分步/秒）加到 s_track_velocity，
 *          余数取模后留回累加器。等效效果：速度平均每周期增加 velocity/20000，
 *          即按 velocity（细分步/秒²）的加速度变化。
 * @param[in] velocity 加速度，单位 细分步/秒²，符号决定加或减（调用方传入 ±s_velocity_acc）
 * @note   只在 20kHz 中断上下文（VelocityTracker_CalcSoftGoal()）中调用。
 */
static void CalcVelocityIntegral(int32_t velocity)
{
    s_velocity_integral += velocity;
    s_track_velocity += s_velocity_integral / CONTROL_FREQUENCY;
    s_velocity_integral = s_velocity_integral % CONTROL_FREQUENCY;
}

/**
 * @brief  位置规划的速度积分器：把加速度折算成每周期增量并累加到位置规划的内部速度上
 * @details 与 CalcVelocityIntegral() 相同的整数积分，但作用于位置模式自己的状态
 *          s_velocity_integral_pos / s_track_velocity_pos，避免与速度模式共用累加器。
 *          数量级说明：加速度 100×51200 细分步/秒² 时，每周期速度增量约为 256 细分步/秒。
 * @param[in] value 加速度，单位 细分步/秒²，符号决定加或减（调用方传入 ±s_velocity_up_acc 或 ±s_velocity_down_acc）
 * @note   只在 20kHz 中断上下文（PositionTracker_CalcSoftGoal()）中调用。
 */
static void CalcPositionVelocityIntegral(int32_t value)
{
    s_velocity_integral_pos += value;
    s_track_velocity_pos += s_velocity_integral_pos / CONTROL_FREQUENCY;
    s_velocity_integral_pos = s_velocity_integral_pos % CONTROL_FREQUENCY;
}

/**
 * @brief  位置积分器：把速度（细分步/秒）折算成每周期位置增量并累加到软目标位置上
 * @details 同样的"余数回填"整数积分，只是被积量换成了速度：s_position_integral 累加速度，
 *          除以 CONTROL_FREQUENCY(20000) 的商（单位 细分步）加到 s_track_position，
 *          余数取模后留回累加器。等效效果：位置平均每周期增加 速度/20000，
 *          也就是位置按速度值推进（单位 细分步）。
 * @param[in] value 速度，单位 细分步/秒（调用方固定传入当前软目标速度 s_track_velocity_pos）
 * @note   只在 20kHz 中断上下文（PositionTracker_CalcSoftGoal() 末尾）中调用，每周期必须调用一次，
 *         否则软目标位置不会前进。
 */
static void CalcPositionIntegral(int32_t value)
{
    s_position_integral += value;
    s_track_position += s_position_integral / CONTROL_FREQUENCY;
    s_position_integral = s_position_integral % CONTROL_FREQUENCY;
}

/**
 * @brief  轨迹规划的速度积分器：把动态加速度折算成每周期增量并累加到轨迹速度上
 * @details 同样的"余数回填"整数积分，累加器是 s_dynamic_vel_acc_remainder，被积量是
 *          TrajectoryTracker_CalcSoftGoal() 按 v²-v₀²=2as 算出的 s_dynamic_velocity_acc
 *          （超时停车时改用 -s_velocity_down_acc_traj）。商（单位 细分步/秒）加到 s_velocity_now，
 *          余数取模后留回累加器。
 * @param[in] value 加速度，单位 细分步/秒²，符号决定加或减
 * @note   只在 20kHz 中断上下文（TrajectoryTracker_CalcSoftGoal()）中调用。
 */
static void CalcTrajVelocityIntegral(int32_t value)
{
    s_dynamic_vel_acc_remainder += value;
    s_velocity_now += s_dynamic_vel_acc_remainder / CONTROL_FREQUENCY;
    s_dynamic_vel_acc_remainder = s_dynamic_vel_acc_remainder % CONTROL_FREQUENCY;
}

/**
 * @brief  轨迹规划的位置积分器：把轨迹速度折算成每周期位置增量并累加到轨迹位置上
 * @details 同样的"余数回填"整数积分，累加器是 s_velocity_now_remainder，被积量是当前轨迹速度
 *          s_velocity_now。商（单位 细分步）加到 s_position_now，余数取模后留回累加器，
 *          等效于位置按 速度 × 50us 前进。
 * @param[in] value 速度，单位 细分步/秒（调用方固定传入 s_velocity_now）
 * @note   只在 20kHz 中断上下文（TrajectoryTracker_CalcSoftGoal() 第 4 步）中调用。
 */
static void CalcTrajPositionIntegral(int32_t value)
{
    s_velocity_now_remainder += value;
    s_position_now += s_velocity_now_remainder / CONTROL_FREQUENCY;
    s_velocity_now_remainder = s_velocity_now_remainder % CONTROL_FREQUENCY;
}

/* ==================== CurrentTracker 实现 ==================== */
/**
 * @brief  初始化电流规划器：用配置里的电流变化率设置梯形斜率
 * @note   Motor_Init() 中调用（主循环上下文），要求 g_motion_config 已赋值。
 */
void CurrentTracker_Init(void) { CurrentTracker_SetCurrentAcc(g_motion_config->ratedCurrentAcc); }

/**
 * @brief  设置电流变化率（梯形斜率）
 * @param[in] currentAcc 电流变化率，单位 mA/s；默认 2000，即 1A 约需 0.5s 爬升到位
 * @note   只写 s_current_acc，不做范围校验；任意上下文可调用。
 */
void CurrentTracker_SetCurrentAcc(int32_t currentAcc) { s_current_acc = currentAcc; }

/**
 * @brief  以当前实际指令电流重置电流规划起点
 * @details 清空积分余数 s_current_integral，并把内部跟踪电流 s_track_current 设为 realCurrent，
 *          使新的梯形轨迹从当前电流无缝接续（避免从 0 重新爬升造成电流突跳）。
 * @param[in] realCurrent 当前实际指令电流，单位 mA（motor.c 传入上一周期的 s_foc_current）
 * @note   由 motor.c 在软目标重建时调用（模式切换、休眠/刹车状态变化），20kHz 中断上下文。
 */
void CurrentTracker_NewTask(int32_t realCurrent)
{
    s_current_integral = 0;
    s_track_current = realCurrent;
}
// 电流梯形平滑规划控制
/**
 * @brief  推进一步电流梯形规划，输出软目标指令电流
 * @details 按"当前跟踪值到目标值的差 delta"分四种情况做匀速斜坡（梯形）规划：
 *          - delta == 0：直接把软目标对齐到目标值；
 *          - delta > 0 且当前电流 ≥ 0：按 +s_current_acc 加速，越过目标就对齐并清零积分余数；
 *          - delta > 0 但当前电流 < 0：先按 +s_current_acc 反向减速，减到 0 时对齐到 0 并清零积分余数
 *            （过零时清零余数是为了避免上一方向的余数把新方向的起点顶偏）；
 *          - delta < 0 的两种情况与上面完全对称，改用 -s_current_acc。
 *          由于斜坡由 CalcCurrentIntegral() 的余数回填实现，实际斜率约为 s_current_acc mA/s。
 * @param[in] goalCurrent 目标电流，单位 mA（已由 motor.c 限幅到 ±ratedCurrent）
 * @note   每个 20kHz 周期调用一次，结果写入 g_go_current；
 *         在电流模式下该值直接送 TB67H450，在其它模式下只当作目标值不参与控制。
 * @warning 本工程没有实测电流反馈，g_go_current 是"指令电流"而非实际相电流。
 */
void CurrentTracker_CalcSoftGoal(int32_t goalCurrent)
{
    int32_t delta = goalCurrent - s_track_current;

    if (delta == 0)
    {
        s_track_current = goalCurrent;
    }
    else if (delta > 0)
    {
        if (s_track_current >= 0)
        {
            CalcCurrentIntegral(s_current_acc);
            if (s_track_current >= goalCurrent)
            {
                s_current_integral = 0;
                s_track_current = goalCurrent;
            }
        }
        else
        {
            CalcCurrentIntegral(s_current_acc);
            if (s_track_current >= 0)
            {
                s_current_integral = 0;
                s_track_current = 0;
            }
        }
    }
    else
    {
        if (s_track_current <= 0)
        {
            CalcCurrentIntegral(-s_current_acc);
            if (s_track_current <= goalCurrent)
            {
                s_current_integral = 0;
                s_track_current = goalCurrent;
            }
        }
        else
        {
            CalcCurrentIntegral(-s_current_acc);
            if (s_track_current <= 0)
            {
                s_current_integral = 0;
                s_track_current = 0;
            }
        }
    }

    g_go_current = s_track_current;
}

/* ==================== VelocityTracker 实现 ==================== */
/**
 * @brief  初始化速度规划器：用配置里的加速度设置梯形斜率
 * @note   Motor_Init() 中调用（主循环上下文），要求 g_motion_config 已赋值。
 */
void VelocityTracker_Init(void) { VelocityTracker_SetVelocityAcc(g_motion_config->ratedVelocityAcc); }

/**
 * @brief  设置速度规划加速度（梯形斜率）
 * @param[in] velocityAcc 加速度，单位 细分步/秒²；默认 100×51200，即 100 圈/秒²
 * @note   只写 s_velocity_acc，不做范围校验；任意上下文可调用。
 */
void VelocityTracker_SetVelocityAcc(int32_t velocityAcc) { s_velocity_acc = velocityAcc; }

/**
 * @brief  以当前估计速度重置速度规划起点
 * @details 清空积分余数 s_velocity_integral，并把内部跟踪速度 s_track_velocity 设为 realVelocity，
 *          使速度梯形从当前实际速度接续爬升（避免从 0 重新加速造成转速突跳）。
 * @param[in] realVelocity 当前估计速度，单位 细分步/秒（motor.c 传入 s_est_velocity）
 * @note   由 motor.c 在软目标重建时调用，20kHz 中断上下文。
 */
void VelocityTracker_NewTask(int32_t realVelocity)
{
    s_velocity_integral = 0;
    s_track_velocity = realVelocity;
}

// 速度梯形平滑规划控制
/**
 * @brief  推进一步速度梯形规划，输出软目标速度
 * @details 分四种情况做匀速斜坡（梯形）规划，逻辑与 CurrentTracker_CalcSoftGoal() 完全对称：
 *          - delta == 0：软目标直接对齐目标速度；
 *          - delta > 0 且当前速度 ≥ 0：按 +s_velocity_acc 加速，越界即对齐并清余数；
 *          - delta > 0 但当前速度 < 0：先反向减速到 0，清余数后再由下一个周期决定是否反向加速；
 *          - delta < 0 的两种情况对称处理，改用 -s_velocity_acc。
 *          这里只做"限加速度"的规划，不检查剩余距离；与目标距离相关的减速由位置/轨迹规划负责。
 * @param[in] goalVelocity 目标速度，单位 细分步/秒（已由 motor.c 限幅到 ±ratedVelocity）
 * @note   每个 20kHz 周期调用一次，结果写入 g_go_velocity；该值在速度模式下作为 PID 的目标速度。
 */
void VelocityTracker_CalcSoftGoal(int32_t goalVelocity)
{
    int32_t delta = goalVelocity - s_track_velocity;

    if (delta == 0)
    {
        s_track_velocity = goalVelocity;
    }
    else if (delta > 0)
    {
        if (s_track_velocity >= 0)
        {
            CalcVelocityIntegral(s_velocity_acc);
            if (s_track_velocity >= goalVelocity)
            {
                s_velocity_integral = 0;
                s_track_velocity = goalVelocity;
            }
        }
        else
        {
            CalcVelocityIntegral(s_velocity_acc);
            if (s_track_velocity >= 0)
            {
                s_velocity_integral = 0;
                s_track_velocity = 0;
            }
        }
    }
    else
    {
        if (s_track_velocity <= 0)
        {
            CalcVelocityIntegral(-s_velocity_acc);
            if (s_track_velocity <= goalVelocity)
            {
                s_velocity_integral = 0;
                s_track_velocity = goalVelocity;
            }
        }
        else
        {
            CalcVelocityIntegral(-s_velocity_acc);
            if (s_track_velocity <= 0)
            {
                s_velocity_integral = 0;
                s_track_velocity = 0;
            }
        }
    }

    g_go_velocity = s_track_velocity;
}

/* ==================== PositionTracker 实现 ==================== */
/**
 * @brief  初始化位置规划器：设置加/减速斜率与低速锁定刹车阈值
 * @details 先用 ratedVelocityAcc 同时设置加速与减速斜率，再由 ratedVelocityAcc/1000 得到低速锁定阈值
 *          s_speed_locking_brake（默认 5120 细分步/秒 = 0.1 圈/秒）：到达目标且速度低于该阈值时
 *          直接把软速度夹到 0，避免在目标点附近长期爬行。
 * @note   Motor_Init() 中调用（主循环上下文），要求 g_motion_config 已赋值。
 */
void PositionTracker_Init(void)
{
    PositionTracker_SetVelocityAcc(g_motion_config->ratedVelocityAcc);
    s_speed_locking_brake = g_motion_config->ratedVelocityAcc / 1000;
}

/**
 * @brief  设置位置规划的加/减速斜率，并重算减速距离系数
 * @details s_quick_velocity_down_acc = 0.5f / a，即 1/(2a)：算减速距离时只需 v² × 该系数，
 *          就能得到 need_down = v²/(2a)（单位 细分步），把除法变成一次乘法并避免大数平方溢出。
 * @param[in] value 加/减速斜率，单位 细分步/秒²（加速与减速取同一数值）
 * @note   value 为 0 时该系数为 inf，位置模式将失去减速距离判断，调用方应保证其为正数。
 */
void PositionTracker_SetVelocityAcc(int32_t value)
{
    s_velocity_up_acc = value;
    s_velocity_down_acc = value;
    s_quick_velocity_down_acc = 0.5f / (float)s_velocity_down_acc;
}

/**
 * @brief  以当前位置与速度重置位置规划起点
 * @details 清零速度积分余数与位置积分余数，并把内部跟踪位置/速度设为当前估计值，
 *          使新的位置轨迹从当前状态无缝接续。
 * @param[in] realLocation 当前估计位置，单位 细分步（编码器绝对坐标）
 * @param[in] realSpeed    当前估计速度，单位 细分步/秒
 * @note   由 motor.c 在软目标重建时调用（进入位置模式、或休眠/刹车状态变化），20kHz 中断上下文。
 */
void PositionTracker_NewTask(int32_t realLocation, int32_t realSpeed)
{
    s_velocity_integral_pos = 0;
    s_track_velocity_pos = realSpeed;
    s_position_integral = 0;
    s_track_position = realLocation;
}
// 位置S形平滑规划控制
/**
 * @brief  推进一步位置规划（梯形/S 形加减速），输出软目标位置与软目标速度
 * @details 整体是"加速—匀速—减速"的梯形规划（原注释称 S 形），每个 20kHz 周期执行一次：
 *          1) 剩余距离 delta = goalPosition - s_track_position；
 *          2) delta == 0（已到目标）：速度落在低速锁定带内（|v| ≤ s_speed_locking_brake）就直接锁 0，
 *             否则按 -s_velocity_down_acc（或反号）减速，减到过零即夹到 0；
 *          3) delta != 0：速度为 0 时按 delta 的符号加速起步；速度与目标同向时用减速距离判据决定
 *             继续加速还是开始减速；速度与目标反向时先减速到 0，下一个周期再换向；
 *          4) 最后调用 CalcPositionIntegral(s_track_velocity_pos) 把速度积成位置，
 *             输出 g_go_location 与 g_go_location_velocity。
 *          减速距离判据（代码中的局部变量 need_down）：由 v² = 2as 得 s = v²/(2a)，代码写成
 *          need_down = v × v × s_quick_velocity_down_acc（系数 1/(2a) 在 SetVelocityAcc 里算好）。
 *          只要 |delta| > need_down，说明剩余距离还够减速，就允许继续加速到 ratedVelocity；
 *          一旦 |delta| ≤ need_down，说明再加速就刹不住，立刻转入减速段。速度超过 ratedVelocity 时
 *          无条件减速回限速值。用浮点乘一次再转 int32，是为了避免 v² 用整数计算时溢出。
 * @param[in] goalPosition 目标位置，单位 细分步（编码器绝对坐标，已含 encoderHomeOffset）
 * @note   每个 20kHz 周期调用一次；输出速度同时作为 DCE 控制器的速度目标（前馈），
 *         因此位置模式的速度平滑与位置误差一起决定指令电流。
 */
void PositionTracker_CalcSoftGoal(int32_t goalPosition)
{
    int32_t delta = goalPosition - s_track_position; // 剩余距离

    /* ==================== 情况1：已到达目标位置 ==================== */
    if (delta == 0)
    {
        // 速度很小时（在刹车阈值内），直接锁定停止
        if ((s_track_velocity_pos >= -s_speed_locking_brake) && (s_track_velocity_pos <= s_speed_locking_brake))
        {
            s_velocity_integral_pos = 0;
            s_track_velocity_pos = 0;
            s_position_integral = 0;
        }
        // 速度为正，需要减速到0
        else if (s_track_velocity_pos > 0)
        {
            CalcPositionVelocityIntegral(-s_velocity_down_acc); // 减速
            if (s_track_velocity_pos <= 0)                      // 已经减到0或以下
            {
                s_velocity_integral_pos = 0;
                s_track_velocity_pos = 0;
            }
        }
        // 速度为负，需要减速到0
        else if (s_track_velocity_pos < 0)
        {
            CalcPositionVelocityIntegral(s_velocity_down_acc); // 减速（反向）
            if (s_track_velocity_pos >= 0)
            {
                s_velocity_integral_pos = 0;
                s_track_velocity_pos = 0;
            }
        }
    }

    /* ==================== 情况2：还需要移动 ==================== */
    else
    {
        /* ---------- 子情况2.1：当前速度为0（从静止开始加速）---------- */
        if (s_track_velocity_pos == 0)
        {
            if (delta > 0)
            {
                CalcPositionVelocityIntegral(s_velocity_up_acc); // 正向加速
            }
            else
            {
                CalcPositionVelocityIntegral(-s_velocity_up_acc); // 反向加速
            }
        }

        /* ---------- 子情况2.2：正向移动中（方向和目标一致）---------- */
        else if ((delta > 0) && (s_track_velocity_pos > 0))
        {
            // 检查当前速度是否在限速范围内
            if (s_track_velocity_pos <= g_motion_config->ratedVelocity)
            {
                // 核心公式：计算从当前速度减到0需要的距离
                // need_down = v2 / (2a)
                int32_t need_down =
                    (int32_t)((float)s_track_velocity_pos * (float)s_track_velocity_pos * s_quick_velocity_down_acc);

                // 判断：剩余距离是否足够减速？
                if (abs(delta) > need_down)
                {
                    // 距离足够，可以继续加速或保持匀速
                    if (s_track_velocity_pos < g_motion_config->ratedVelocity)
                    {
                        CalcPositionVelocityIntegral(s_velocity_up_acc); // 继续加速
                        // 限幅：不超过最大速度
                        if (s_track_velocity_pos >= g_motion_config->ratedVelocity)
                        {
                            s_velocity_integral_pos = 0;
                            s_track_velocity_pos = g_motion_config->ratedVelocity;
                        }
                    }
                    else if (s_track_velocity_pos > g_motion_config->ratedVelocity)
                    {
                        CalcPositionVelocityIntegral(-s_velocity_down_acc); // 减速到限速
                    }
                }
                else
                {
                    // 距离不够了，必须开始减速！
                    CalcPositionVelocityIntegral(-s_velocity_down_acc);
                    if (s_track_velocity_pos <= 0)
                    {
                        s_velocity_integral_pos = 0;
                        s_track_velocity_pos = 0;
                    }
                }
            }
            else
            {
                // 速度超限，强制减速
                CalcPositionVelocityIntegral(-s_velocity_down_acc);
                if (s_track_velocity_pos <= 0)
                {
                    s_velocity_integral_pos = 0;
                    s_track_velocity_pos = 0;
                }
            }
        }

        /* ---------- 子情况2.3：反向移动中（方向和目标一致）---------- */
        else if ((delta < 0) && (s_track_velocity_pos < 0))
        {
            // 逻辑与正向对称，方向相反
            if (s_track_velocity_pos >= -g_motion_config->ratedVelocity)
            {
                int32_t need_down =
                    (int32_t)((float)s_track_velocity_pos * (float)s_track_velocity_pos * s_quick_velocity_down_acc);
                if (abs(delta) > need_down)
                {
                    if (s_track_velocity_pos > -g_motion_config->ratedVelocity)
                    {
                        CalcPositionVelocityIntegral(-s_velocity_up_acc);
                        if (s_track_velocity_pos <= -g_motion_config->ratedVelocity)
                        {
                            s_velocity_integral_pos = 0;
                            s_track_velocity_pos = -g_motion_config->ratedVelocity;
                        }
                    }
                    else if (s_track_velocity_pos < -g_motion_config->ratedVelocity)
                    {
                        CalcPositionVelocityIntegral(s_velocity_down_acc);
                    }
                }
                else
                {
                    CalcPositionVelocityIntegral(s_velocity_down_acc);
                    if (s_track_velocity_pos >= 0)
                    {
                        s_velocity_integral_pos = 0;
                        s_track_velocity_pos = 0;
                    }
                }
            }
            else
            {
                CalcPositionVelocityIntegral(s_velocity_down_acc);
                if (s_track_velocity_pos >= 0)
                {
                    s_velocity_integral_pos = 0;
                    s_track_velocity_pos = 0;
                }
            }
        }

        /* ---------- 子情况2.4：速度方向与目标方向相反 ---------- */
        else if ((delta < 0) && (s_track_velocity_pos > 0))
        {
            // 需要反向，但当前正在正向运动 → 先减速到0
            CalcPositionVelocityIntegral(-s_velocity_down_acc);
            if (s_track_velocity_pos <= 0)
            {
                s_velocity_integral_pos = 0;
                s_track_velocity_pos = 0;
            }
        }

        /* ---------- 子情况2.5：速度方向与目标方向相反 ---------- */
        else if ((delta > 0) && (s_track_velocity_pos < 0))
        {
            // 需要正向，但当前正在反向运动 → 先减速到0
            CalcPositionVelocityIntegral(s_velocity_down_acc);
            if (s_track_velocity_pos >= 0)
            {
                s_velocity_integral_pos = 0;
                s_track_velocity_pos = 0;
            }
        }
    }

    /* 根据当前速度，更新位置 */
    CalcPositionIntegral(s_track_velocity_pos);

    /* 输出规划后的位置和速度 */
    g_go_location = s_track_position;
    g_go_location_velocity = s_track_velocity_pos;
}

/* ==================== PositionInterpolator 实现 ==================== */
/**
 * @brief  初始化位置插值器（空实现）
 * @details 函数体内只有 "Nothing to init" 注释，没有任何初始化动作：所有状态都靠 NewTask() 重置，
 *          静态变量以 0 作为上电初值。
 * @note   Motor_Init() 中调用；由于其它接口当前无人调用，本函数实际上也没有实际作用。
 * @todo   位置插值器整体尚未接入控制流程（见 PositionInterpolator_CalcSoftGoal()）。
 */
void PositionInterpolator_Init(void) { /* Nothing to init */ }

/**
 * @brief  以当前位置与速度重置插值器状态
 * @details 记录位置同时写入"当前"和"上一次"两个变量（保证第一次差分结果为 0），
 *          并把滤波速度初值设为传入的实际速度，避免上电瞬间产生虚假速度尖峰。
 * @param[in] realPosition 当前实际位置，单位 细分步
 * @param[in] realVelocity 当前实际速度，单位 细分步/秒
 * @note   当前工程内无调用者。
 * @todo   为 MODE_STEP_DIR（Step/Dir 输入）预留的接口，motor.c 未调用，实际未生效。
 */
void PositionInterpolator_NewTask(int32_t realPosition, int32_t realVelocity)
{
    s_record_position = realPosition;
    s_record_position_last = realPosition;
    s_est_position = realPosition;
    s_est_velocity_interp = realVelocity;
}
// Step/Dir模式
/**
 * @brief  按新到的目标位置推进一步插值：位置直接跟随，速度做一阶低通滤波
 * @details 定点/单位约定（<<6、>>6 即 64 倍定标）：
 *          1) s_est_position_integral += Δp × CONTROL_FREQUENCY + ((v << 6) - v)，
 *             其中 Δp = 本次目标位置 - 上次目标位置（细分步），乘以 20000 换算成 细分步/秒，
 *             (v<<6)-v 就是 63v（等价于 64v 但不额外做乘法），即泄漏积分项；
 *          2) s_est_velocity_interp = 累加器 >> 6；
 *          3) 累加器 -= (v << 6)，把已进位部分取走、只留低 6 位余数（余数回填）。
 *          合并后等效于 v ← v + (20000×Δp − v)/64，是一个时间常数 64 个控制周期（≈3.2ms）的
 *          一阶低通，稳态时 v 等于 20000×Δp 即目标位置的变化率（细分步/秒）。
 *          位置不做平滑：s_est_position 直接等于本次目标位置。
 * @param[in] goalPosition 本次收到的目标位置，单位 细分步
 * @note   每个 20kHz 周期调用一次时才能得到上述时间常数；输出写入 g_interp_go_position 与
 *         g_interp_go_velocity。若 Δp 恒为 0，滤波速度会按 63/64 衰减到 0。
 * @todo   当前工程内无调用者（motor.c 只调用了 PositionInterpolator_Init()），
 *         MODE_STEP_DIR 模式在 motor.c 中也没有处理分支，本函数属于半成品。
 */
void PositionInterpolator_CalcSoftGoal(int32_t goalPosition)
{
    s_record_position_last = s_record_position;
    s_record_position = goalPosition;

    s_est_position_integral += ((s_record_position - s_record_position_last) * CONTROL_FREQUENCY) +
                               ((s_est_velocity_interp << 6) - s_est_velocity_interp);
    s_est_velocity_interp = s_est_position_integral >> 6;
    s_est_position_integral -= (s_est_velocity_interp << 6);

    s_est_position = s_record_position;

    g_interp_go_position = s_est_position;
    g_interp_go_velocity = s_est_velocity_interp;
}

/* ==================== TrajectoryTracker 实现 ==================== */
/**
 * @brief  初始化轨迹规划器：设置超时减速斜率与指令更新超时
 * @param[in] updateTimeout 指令更新超时阈值，单位 ms（motor.c 传入 200，即 200ms 没有新指令就安全停车）
 * @note   Motor_Init() 中调用（主循环上下文），要求 g_motion_config 已赋值。
 */
void TrajectoryTracker_Init(int32_t updateTimeout)
{
    TrajectoryTracker_SetSlowDownVelocityAcc(g_motion_config->ratedVelocityAcc);
    s_update_timeout = updateTimeout;
}

/**
 * @brief  设置轨迹规划的减速斜率（同时用于超时安全停车）
 * @param[in] value 减速度，单位 细分步/秒²
 * @note   只写 s_velocity_down_acc_traj，不做范围校验。
 */
void TrajectoryTracker_SetSlowDownVelocityAcc(int32_t value) { s_velocity_down_acc_traj = value; }

/**
 * @brief  以当前位置与速度重置轨迹规划起点并复位超时计时
 * @details 清零累计时间 s_update_time、超时标志、加速度与速度两个积分余数，并把内部跟踪位置/速度
 *          设为当前估计值；同时把速度余数显式置 0，保证新的轨迹从静止余数状态开始积分。
 * @param[in] realLocation 当前估计位置，单位 细分步（编码器绝对坐标）
 * @param[in] realSpeed    当前估计速度，单位 细分步/秒
 * @note   由 motor.c 在软目标重建时调用（进入轨迹模式、或休眠/刹车状态变化），20kHz 中断上下文。
 */
void TrajectoryTracker_NewTask(int32_t realLocation, int32_t realSpeed)
{
    s_update_time = 0;
    s_overtime_flag = false;
    s_dynamic_vel_acc_remainder = 0;
    s_velocity_now = realSpeed;
    s_velocity_now_remainder = 0;
    s_position_now = realLocation;
}

/**
 * @brief  推进一步轨迹规划：按目标位置与目标速度反算加速度，输出软目标位置与速度
 * @details 每个 20kHz 周期按 5 步执行：
 *          1) 目标（位置或速度）发生变化时，用 v₂²-v₁²=2as 反算所需加速度
 *             a = (v₂²-v₁²)/(2s)，代码写成 (v₂+v₁)(v₂-v₁)/(2×(goalPosition-s_position_now))，
 *             用因式分解代替平方以避免大数溢出；同时把超时计时清零；
 *          2) 目标未变化时累加 s_update_time（单位 微秒），超过 s_update_timeout×1000 就置超时标志；
 *          3) 未超时则用第 1 步算出的加速度积分速度；超时（通信中断）则改用 ±s_velocity_down_acc_traj
 *             减速到 0，实现安全停车；
 *          4) 把速度积成位置；
 *          5) 输出 g_traj_go_position 与 g_traj_go_velocity。
 * @warning 第 1 步的除法存在除零风险：当 goalPosition == s_position_now（位移为 0）时分母为 0，
 *          浮点除法会得到 inf/NaN，再强制转换成 int32_t 属于未定义行为，实际表现为加速度/速度突变。
 * @param[in] goalPosition 目标位置，单位 细分步（编码器绝对坐标）
 * @param[in] goalVelocity 目标速度，单位 细分步/秒
 * @note   每个 20kHz 周期调用一次；上位机需要持续下发新目标，否则 200ms 后自动减速停车。
 * @todo   当前工程内只有 motor.c 的 MODE_COMMAND_TRAJECTORY 分支会调用，而 main.c/uart_cmd.c 都没有
 *         设置该模式的入口，因此轨迹模式目前没有实际触发路径。
 */
void TrajectoryTracker_CalcSoftGoal(int32_t goalPosition, int32_t goalVelocity)
{
    /* ==================== 第1步：检查目标是否变化 ==================== */
    if (goalVelocity != s_record_velocity || goalPosition != s_record_position_traj)
    {
        // 目标有变化（收到了新的轨迹指令）
        s_update_time = 0;                     // 重置超时计时器
        s_record_velocity = goalVelocity;      // 记录新目标速度
        s_record_position_traj = goalPosition; // 记录新目标位置

        /**
         * 核心公式：计算需要的加速度
         *
         * 由运动学公式：v22 - v12 = 2 × a × s
         * 推导出：a = (v22 - v12) / (2 × s)
         *
         * 代码中用 (v2 + v1)(v2 - v1) 代替 v22 - v12，
         * 避免大数平方导致溢出。
         *
         * 参数说明：
         *   goalVelocity   = v2（目标速度）
         *   s_velocity_now = v1（当前速度）
         *   goalPosition - s_position_now = s（位移）
         */
        s_dynamic_velocity_acc =
            (int32_t)((float)(goalVelocity + s_velocity_now) * (float)(goalVelocity - s_velocity_now) /
                      (float)(2 * (goalPosition - s_position_now)));
        s_overtime_flag = false; // 清除超时标志
    }
    /* ==================== 第2步：目标未变化，检查超时 ==================== */
    else
    {
        // 长时间没收到新指令，累积超时时间
        if (s_update_time >= (s_update_timeout * 1000))
        {
            s_overtime_flag = true; // 超时！触发安全停车
        }
        else
        {
            s_update_time += CONTROL_PERIOD_US; // 累加时间（单位：微秒）
        }
    }

    /* ==================== 第3步：根据模式执行运动 ==================== */
    if (s_overtime_flag)
    {
        /**
         * 超时模式：通信中断，安全停车
         *
         * 作用：如果上位机长时间没有发送新的轨迹指令，
         *       认为通信可能中断，主动减速到 0。
         */
        if (s_velocity_now == 0)
        {
            // 已经停止，无事可做
            s_dynamic_vel_acc_remainder = 0;
        }
        else if (s_velocity_now > 0)
        {
            // 正向运动 → 减速（负加速度）
            CalcTrajVelocityIntegral(-s_velocity_down_acc_traj);
            if (s_velocity_now <= 0)
            {
                // 已经减到 0 或以下
                s_dynamic_vel_acc_remainder = 0;
                s_velocity_now = 0;
            }
        }
        else
        {
            // 反向运动 → 减速（正加速度，因为速度是负的）
            CalcTrajVelocityIntegral(s_velocity_down_acc_traj);
            if (s_velocity_now >= 0)
            {
                s_dynamic_vel_acc_remainder = 0;
                s_velocity_now = 0;
            }
        }
    }
    else
    {
        /**
         * 正常模式：按计算出的加速度运动
         *
         * 加速度可能是正（加速）、负（减速）或 0（匀速）
         */
        CalcTrajVelocityIntegral(s_dynamic_velocity_acc);
    }

    /* ==================== 第4步：根据速度更新位置 ==================== */
    CalcTrajPositionIntegral(s_velocity_now);

    /* ==================== 第5步：输出结果 ==================== */
    g_traj_go_position = s_position_now; // 规划后的位置
    g_traj_go_velocity = s_velocity_now; // 规划后的速度
}
