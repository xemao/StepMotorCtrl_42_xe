/**
 ******************************************************************************
 * @file    motor.c
 * @brief   42 步进电机闭环 FOC 驱动的核心实现：20kHz 闭环控制 + 堵转/过载保护
 * @details 每个 20kHz（50us）周期按固定顺序执行：读 MT6816 磁编码器角度并累加多圈位置 →
 *          一阶低通估计速度 → 查 CompensateAdvancedAngle() 得超前角并合成估计位置（同时当作 FOC
 *          换相角）→ 按运行模式选控制器（位置模式用 DCE、速度模式用 PID、电流模式直接输出）算出
 *          指令电流 → 运动规划把"目标值"平滑成"软目标值" → 堵转/过载检测 → 状态机刷新。
 *          内部单位：位置 细分步（51200 细分步/圈，校准表把机械角映射到该刻度，1 细分步同时等于
 *          FOC 正弦表的 1 个索引，1024 个索引为一个电周期）、速度 细分步/秒、电流 mA。
 * @note    实时性：Motor_Tick20kHz() 只在 TIM4 的 20kHz 中断中调用，函数内不能有阻塞操作；
 *          Motor_SetMode()/SetPosition()/SetVelocity()/SetCurrent() 等接口由 100Hz 任务或串口命令
 *          调用，它们只改"目标值"，与中断之间通过"目标值 → 软目标值"解耦。
 * @warning 本工程没有实测电流反馈：s_foc_current 是控制器算出的"指令电流"，ADC 只被 CubeMX
 *          初始化过、没有任何采样接入，因此这里的电流环节是开环电流矢量控制而非电流闭环。
 ******************************************************************************
 */

#include "motor.h"
#include "configurations.h"
#include "eeprom.h"
#include "encoder_calibrator.h"
#include "mt6816.h"
#include "tb67h450.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* ==================== 电机状态变量 ==================== */
/** @brief 整机配置指针：由 Motor_SetConfig() 写入，全模块的增益/限幅/Home 偏移都从这里读 */
static Motor_Config_t *s_config = NULL;
/** @brief 请求模式：由 Motor_SetMode() 写入（任意上下文），20kHz 中断把它同步成 s_mode_running */
static Motor_Mode_t s_request_mode = MODE_STOP;
/** @brief 运行模式：20kHz 中断实际使用的模式，切换瞬间会触发软目标重建 */
static Motor_Mode_t s_mode_running = MODE_STOP;
/** @brief 电机状态：由中断末尾的状态机刷新，供 LED 与上位机查询 */
static Motor_State_t s_state = STATE_STOP;
/** @brief 堵转标志：为 true 时驱动强制休眠，需 Motor_ClearStallFlag() 才能恢复 */
static bool s_is_stalled = false;

/* 实际值 */
/** @brief 本周期读到的单圈位置，单位 细分步（0~51199，来自校准后的编码器角度） */
static int32_t s_real_lap_position = 0;
/** @brief 上一周期的单圈位置，用于做差得到位移增量 */
static int32_t s_real_lap_position_last = 0;
/** @brief 累计多圈位置，单位 细分步（按增量累加，越过一圈时回绕修正） */
static int32_t s_real_position = 0;
/** @brief 上一周期的累计位置，用于估计速度 */
static int32_t s_real_position_last = 0;

/* 估计值 */
/** @brief 估计速度，单位 细分步/秒（对位置差分做一阶低通，时间常数约 32 个控制周期） */
static int32_t s_est_velocity = 0;
/** @brief 速度估计的泄漏积分累加器：保存 31/32 低通未进位的余数（配合 <<5/>>5 定标使用） */
static int32_t s_est_velocity_integral = 0;
/** @brief 超前角补偿量，单位 细分步（即 FOC 正弦表索引数），由 CompensateAdvancedAngle() 按速度给出 */
static int32_t s_est_lead_position = 0;
/** @brief 估计位置 = 实际位置 + 超前角补偿，单位 细分步；同时被当作 FOC 电角度索引送 TB67H450 */
static int32_t s_est_position = 0;

/* 目标值 */
/** @brief 目标位置，单位 细分步（编码器绝对坐标 = 用户坐标 + encoderHomeOffset） */
static int32_t s_goal_position = 0;
/** @brief 目标速度，单位 细分步/秒（已由 Motor_SetVelocity() 限幅） */
static int32_t s_goal_velocity = 0;
/** @brief 目标电流，单位 mA（已由 Motor_SetCurrent() 限幅） */
static int32_t s_goal_current = 0;
/** @brief 休眠请求：true 表示要求驱动断使能（由 Motor_SetDisable() 写入） */
static bool s_goal_disable = false;
/** @brief 刹车请求：true 表示要求驱动短接制动（由 Motor_SetBrake() 写入） */
static bool s_goal_brake = false;

/* 软目标值（平滑后的）*/
/** @brief 软目标位置，单位 细分步：位置/轨迹模式由 PositionTracker、TrajectoryTracker 规划输出 */
static int32_t s_soft_position = 0;
/** @brief 软目标速度，单位 细分步/秒：速度模式由 VelocityTracker 规划，位置模式由 PositionTracker 规划 */
static int32_t s_soft_velocity = 0;
/** @brief 软目标电流，单位 mA：电流模式由 CurrentTracker 规划 */
static int32_t s_soft_current = 0;
/** @brief 软目标休眠状态：在中断末尾由 s_goal_disable 同步而来 */
static bool s_soft_disable = false;
/** @brief 软目标刹车状态：在中断末尾由 s_goal_brake 同步而来 */
static bool s_soft_brake = false;
/** @brief 软目标重建请求：置位后下一周期重新 NewTask 各 tracker 并清积分（模式切换/休眠刹车变化时置位） */
static bool s_soft_new_curve = false;

/* FOC 输出 */
/** @brief 输出给 TB67H450 的换相角，单位 正弦表索引（其低 10 位 0~1023 即电角度） */
static int32_t s_foc_position = 0;
/** @brief 输出给 TB67H450 的指令电流，单位 mA（没有实测电流反馈，见文件头 @warning） */
static int32_t s_foc_current = 0;

/* 故障检测 */
/** @brief 堵转累计时间，单位 微秒（每周期加 50，累计到 1000000 即 1s 判定堵转） */
static uint32_t s_stalled_time = 0;
/** @brief 过载累计时间，单位 微秒（每周期加 50，累计到 1000000 即 1s 置过载标志） */
static uint32_t s_overload_time = 0;
/** @brief 过载标志：指令电流顶到 ratedCurrent 超时置位，电流降下来立刻清除（不锁存） */
static bool s_overload_flag = false;

/* 首次调用标志 */
/** @brief 首次调用标志：上电第一次进入 20kHz 中断时只初始化位置并直接返回，不进入控制流程 */
static bool s_first_called = true;

/* 外部引用 */
/** @brief 板级配置（EEPROM 镜像）：本文件只在 Motor_ZeroPosition() 里回写 encoderHomeOffset */
extern BoardConfig_t boardConfig;

/* ==================== 辅助函数 ==================== */
/**
 * @brief  按估计速度计算超前角补偿量（分段折线，用于补偿高速时的控制滞后）
 * @details 输入速度单位 细分步/秒、输出补偿量单位 细分步（也就是 FOC 正弦表索引数，1024 索引 = 1 个
 *          电周期 = 360° 电角度），补偿量直接加到实际位置上得到换相用的 s_est_position。
 *          正速度与负速度各是一条四段折线（负速度分支是正速度的镜像，偏移取负号），阈值与斜率如下：
 *          - |vel| < 100000（≈1.95 圈/秒）：不补偿，compensate = 0；
 *          - 100000 ≤ |vel| < 1300000（≈1.95~25.4 圈/秒）：斜率 262 >> 20（≈250us 的时间超前），
 *            从 0 线性升到约 300 细分步；
 *          - 1300000 ≤ |vel| < 2200000（≈25.4~43.0 圈/秒）：斜率 105 >> 20（≈100us），
 *            偏移 +300，继续升到约 390 细分步；
 *          - |vel| ≥ 2200000：斜率 52 >> 20（≈50us），偏移 +390。
 *          各段用 ((速度 - 段起点) × 斜率) >> 20 的定点乘法实现，段间偏移 0/300/390 保证折线在阈值处连续；
 *          最后用 ±430 的上限把补偿角夹住（430 细分步 ≈ 3.0° 机械角 ≈ 0.42 个电周期），
 *          避免高速段补偿随速度无限增长导致换相角超前过多而失步。
 *          斜率除以 2^20 后量纲是"秒"，即该段相当于按固定时间提前（250us/100us/50us）补偿。
 * @param[in] vel 估计速度，单位 细分步/秒（一般传入 s_est_velocity）
 * @return 超前角补偿量，单位 细分步，范围 [-430, 430]；正速度返回正值（超前），负速度返回负值
 * @note   只在 20kHz 中断里由 Motor_Tick20kHz() 调用；纯整数运算，无副作用。
 * @note   负速度分支里 (vel + 阈值) 为负、>>20 对负数向下取整，因此正负分支在阈值附近可能相差 1 个
 *         细分步，属于整数取整误差，不影响功能。
 */
static int32_t CompensateAdvancedAngle(int32_t vel)
{
    int32_t compensate;

    if (vel < 0)
    {
        if (vel > -100000)
            compensate = 0;
        else if (vel > -1300000)
            compensate = (((vel + 100000) * 262) >> 20) - 0;
        else if (vel > -2200000)
            compensate = (((vel + 1300000) * 105) >> 20) - 300;
        else
            compensate = (((vel + 2200000) * 52) >> 20) - 390;

        if (compensate < -430)
            compensate = -430;
    }
    else
    {
        if (vel < 100000)
            compensate = 0;
        else if (vel < 1300000)
            compensate = (((vel - 100000) * 262) >> 20) + 0;
        else if (vel < 2200000)
            compensate = (((vel - 1300000) * 105) >> 20) + 300;
        else
            compensate = (((vel - 2200000) * 52) >> 20) + 390;

        if (compensate > 430)
            compensate = 430;
    }

    return compensate;
}

/**
 * @brief  把指令电流合成为 FOC 电流矢量并输出到 TB67H450
 * @details 先按电流方向给换相角加/减 90° 电角度：s_foc_position = s_est_position ± SOFT_DIVIDE_NUM，
 *          因为 256 细分步 = 1024/4 = 1/4 个电周期（一个电周期 1024 个索引）；
 *          正电流超前 90°、负电流滞后 90°、零电流不加偏移，随后交给 TB67H450_SetFocCurrentVector()，
 *          由它取低 10 位查正弦表并按 mA 折算两相 PWM 占空比。
 * @param[in] current 指令电流，单位 mA；正负号决定电机转向，幅值决定输出电流大小
 * @note   只在 20kHz（50us）中断中由 CalcPidToOutput()/CalcDceToOutput() 或电流模式直接调用。
 * @warning 这里送出去的是"指令电流"，TB67H450 只按电压/PWM 开环输出，工程内没有任何电流采样，
 *          所以无法确认实际相电流是否等于该指令值。
 */
static void CalcCurrentToOutput(int32_t current)
{
    s_foc_current = current;

    if (s_foc_current > 0)
    {
        s_foc_position = s_est_position + SOFT_DIVIDE_NUM; /* 超前90° */
    }
    else if (s_foc_current < 0)
    {
        s_foc_position = s_est_position - SOFT_DIVIDE_NUM; /* 滞后90° */
    }
    else
    {
        s_foc_position = s_est_position;
    }

    TB67H450_SetFocCurrentVector(s_foc_position, s_foc_current);
}

/**
 * @brief  PID 速度环：由目标软速度与估计速度算出指令电流（速度模式使用）
 * @details 定点定标与限幅（详见 motor.h 的 PID_t 说明）：
 *          1) vError = speed - s_est_velocity（单位 细分步/秒），限幅 ±1024×1024（≈±20.5 圈/秒）；
 *             该限幅既把误差控制在同一数量级，也保证 kp×vError 不会在 int32 下溢出；
 *          2) outputKp = kp × vError，与后两项一样是 <<10 定标（相当于 mA×1024）；
 *          3) 积分项：integralRound += ki × vError；integralRemainder = integralRound >> 10；
 *             integralRound -= integralRemainder << 10（把余数回填，低 10 位不足 1 的部分下周期继续累积，
 *             避免小误差被整除截断成 0）；outputKi += integralRemainder；
 *          4) 积分限幅：outputKi 夹在 ±(ratedCurrent << 10)（即 ±ratedCurrent 对应的 <<10 定标值），
 *             注意只夹紧 outputKi，integralRound 里的余数不会清零，需要靠 ClearIntegral() 复位；
 *          5) 微分项：outputKd = kd × (vError - vErrorLast)，是误差的真实差分；
 *          6) 总输出 output = (outputKp + outputKi + outputKd) >> 10，得到单位 mA 的指令电流，
 *             再限幅到 ±ratedCurrent，最后交给 CalcCurrentToOutput()。
 * @param[in] speed 目标软速度，单位 细分步/秒（motor.c 传入 VelocityTracker 规划出的 s_soft_velocity）
 * @note   只在 20kHz 中断中按 MODE_COMMAND_VELOCITY / MODE_PWM_VELOCITY 调用；会写回 PID_t 的中间量。
 * @warning 输出只是指令电流，没有实测电流反馈参与闭环。
 */
static void CalcPidToOutput(int32_t speed)
{
    /* PID 速度环 */
    s_config->ctrlParams.pid.vErrorLast = s_config->ctrlParams.pid.vError;
    s_config->ctrlParams.pid.vError = speed - s_est_velocity;

    /* 限幅 */
    if (s_config->ctrlParams.pid.vError > (1024 * 1024))
        s_config->ctrlParams.pid.vError = (1024 * 1024);
    if (s_config->ctrlParams.pid.vError < (-1024 * 1024))
        s_config->ctrlParams.pid.vError = (-1024 * 1024);

    s_config->ctrlParams.pid.outputKp = s_config->ctrlParams.pid.kp * s_config->ctrlParams.pid.vError;

    /* 积分项 */
    s_config->ctrlParams.pid.integralRound += (s_config->ctrlParams.pid.ki * s_config->ctrlParams.pid.vError);
    s_config->ctrlParams.pid.integralRemainder = s_config->ctrlParams.pid.integralRound >> 10;
    s_config->ctrlParams.pid.integralRound -= (s_config->ctrlParams.pid.integralRemainder << 10);
    s_config->ctrlParams.pid.outputKi += s_config->ctrlParams.pid.integralRemainder;

    /* 积分限幅 */
    if (s_config->ctrlParams.pid.outputKi > (s_config->motionParams.ratedCurrent << 10))
        s_config->ctrlParams.pid.outputKi = (s_config->motionParams.ratedCurrent << 10);
    else if (s_config->ctrlParams.pid.outputKi < -(s_config->motionParams.ratedCurrent << 10))
        s_config->ctrlParams.pid.outputKi = -(s_config->motionParams.ratedCurrent << 10);

    /* 微分项 */
    s_config->ctrlParams.pid.outputKd =
        s_config->ctrlParams.pid.kd * (s_config->ctrlParams.pid.vError - s_config->ctrlParams.pid.vErrorLast);

    /* 总输出 */
    s_config->ctrlParams.pid.output =
        (s_config->ctrlParams.pid.outputKp + s_config->ctrlParams.pid.outputKi + s_config->ctrlParams.pid.outputKd) >>
        10;

    /* 输出限幅 */
    if (s_config->ctrlParams.pid.output > s_config->motionParams.ratedCurrent)
        s_config->ctrlParams.pid.output = s_config->motionParams.ratedCurrent;
    else if (s_config->ctrlParams.pid.output < -s_config->motionParams.ratedCurrent)
        s_config->ctrlParams.pid.output = -s_config->motionParams.ratedCurrent;

    CalcCurrentToOutput(s_config->ctrlParams.pid.output);

    //      printf("target=%ld, est=%ld, err=%ld, out=%ld\r\n",
    //       speed, s_est_velocity,
    //       s_config->ctrlParams.pid.vError,
    //       s_config->ctrlParams.pid.output);
}

/**
 * @brief  DCE 双闭环（位置环 + 速度环）：由目标软位置/软速度算出指令电流（位置与轨迹模式使用）
 * @details 两环合并成一次定点运算，定标与限幅（详见 motor.h 的 DCE_t 说明）：
 *          1) pError = location - s_est_position（单位 细分步），限幅 ±3200（3200/51200 = 1/16 圈 ≈ 22.5°
 *             机械角）；该限幅避免目标突变时比例项过大，也保证 kp×pError 不溢出；
 *          2) vError = (speed - s_est_velocity) >> 7，先把速度误差缩小 128 倍再限幅 ±4000
 *             （对应约 ±512000 细分步/秒 ≈ ±10 圈/秒），防止高速段巨大的速度误差主导输出；
 *          3) 比例项 outputKp = kp × pError（<<10 定标，相当于 mA×1024）；
 *          4) 积分项：integralRound += ki × pError + kv × vError；
 *             integralRemainder = integralRound >> 7（注意这里是 >>7 而不是 >>10）；
 *             integralRound -= integralRemainder << 7（余数回填，低 7 位继续累积）；
 *             outputKi += integralRemainder —— 即每周期把 (ki×pError + kv×vError) 的 1/128 累加进
 *             outputKi，因此 DCE 的 ki/kv 与 PID 的 ki 在数值口径上并不相同（PID 是 1/1024）；
 *          5) 积分限幅：outputKi 夹在 ±(ratedCurrent << 10)，integralRound 里的余数不清零；
 *          6) 微分项 outputKd = kd × vError —— 用的是本周期的速度误差本身，不是误差差分，
 *             所以实际起"速度阻尼"作用而不是微分预测；
 *          7) 总输出 output = (outputKp + outputKi + outputKd) >> 10，得到单位 mA 的指令电流，
 *             再限幅到 ±ratedCurrent，最后交给 CalcCurrentToOutput()。
 * @param[in] location 目标软位置，单位 细分步（motor.c 传入 PositionTracker/TrajectoryTracker 的软目标）
 * @param[in] speed    目标软速度，单位 细分步/秒（同上，作为速度环目标/阻尼参考）
 * @note   只在 20kHz 中断中按 MODE_COMMAND_POSITION / MODE_PWM_POSITION / MODE_COMMAND_TRAJECTORY 调用。
 * @warning 输出只是指令电流，没有实测电流反馈参与闭环；pError 用的是"估计位置"而不是实测负载位置。
 */
static void CalcDceToOutput(int32_t location, int32_t speed)
{
    /* DCE 双闭环控制器 */
    s_config->ctrlParams.dce.pError = location - s_est_position;
    s_config->ctrlParams.dce.vError = (speed - s_est_velocity) >> 7;

    /* 限幅 */
    if (s_config->ctrlParams.dce.pError > 3200)
        s_config->ctrlParams.dce.pError = 3200;
    if (s_config->ctrlParams.dce.pError < -3200)
        s_config->ctrlParams.dce.pError = -3200;
    if (s_config->ctrlParams.dce.vError > 4000)
        s_config->ctrlParams.dce.vError = 4000;
    if (s_config->ctrlParams.dce.vError < -4000)
        s_config->ctrlParams.dce.vError = -4000;

    /* 比例项 */
    s_config->ctrlParams.dce.outputKp = s_config->ctrlParams.dce.kp * s_config->ctrlParams.dce.pError;

    /* 积分项 */
    s_config->ctrlParams.dce.integralRound += (s_config->ctrlParams.dce.ki * s_config->ctrlParams.dce.pError +
                                               s_config->ctrlParams.dce.kv * s_config->ctrlParams.dce.vError);
    s_config->ctrlParams.dce.integralRemainder = s_config->ctrlParams.dce.integralRound >> 7;
    s_config->ctrlParams.dce.integralRound -= (s_config->ctrlParams.dce.integralRemainder << 7);
    s_config->ctrlParams.dce.outputKi += s_config->ctrlParams.dce.integralRemainder;

    /* 积分限幅 */
    if (s_config->ctrlParams.dce.outputKi > (s_config->motionParams.ratedCurrent << 10))
        s_config->ctrlParams.dce.outputKi = (s_config->motionParams.ratedCurrent << 10);
    else if (s_config->ctrlParams.dce.outputKi < -(s_config->motionParams.ratedCurrent << 10))
        s_config->ctrlParams.dce.outputKi = -(s_config->motionParams.ratedCurrent << 10);

    /* 微分项 */
    s_config->ctrlParams.dce.outputKd = s_config->ctrlParams.dce.kd * s_config->ctrlParams.dce.vError;

    /* 总输出 */
    s_config->ctrlParams.dce.output =
        (s_config->ctrlParams.dce.outputKp + s_config->ctrlParams.dce.outputKi + s_config->ctrlParams.dce.outputKd) >>
        10;

    /* 输出限幅 */
    if (s_config->ctrlParams.dce.output > s_config->motionParams.ratedCurrent)
        s_config->ctrlParams.dce.output = s_config->motionParams.ratedCurrent;
    else if (s_config->ctrlParams.dce.output < -s_config->motionParams.ratedCurrent)
        s_config->ctrlParams.dce.output = -s_config->motionParams.ratedCurrent;

    CalcCurrentToOutput(s_config->ctrlParams.dce.output);
}

/**
 * @brief  清零 PID 与 DCE 的积分累加器（抗积分饱和的手动复位）
 * @details 把 pid/dce 各自的 integralRound、integralRemainder、outputKi 全部清 0；
 *          比例项与微分项本来就每周期重算，不需要清。
 *          因为 CalcPidToOutput()/CalcDceToOutput() 的积分限幅只夹紧 outputKi、不会清累加器余数，
 *          所以休眠、刹车、未校准以及软目标重建（模式切换）时都必须调用本函数，
 *          否则重新使能瞬间会把停机期间累积的积分一次性放出去（积分饱和冲击）。
 * @note   只在 20kHz 中断上下文调用；无参数、无返回值，直接改静态状态。
 *         要求 s_config 非空（休眠/刹车分支进入前已保证配置已设置）。
 */
static void ClearIntegral(void)
{
    s_config->ctrlParams.pid.integralRound = 0;
    s_config->ctrlParams.pid.integralRemainder = 0;
    s_config->ctrlParams.pid.outputKi = 0;

    s_config->ctrlParams.dce.integralRound = 0;
    s_config->ctrlParams.dce.integralRemainder = 0;
    s_config->ctrlParams.dce.outputKi = 0;
}

/* ==================== 公共函数 ==================== */
/**
 * @brief  初始化电机系统：复位故障状态并初始化 5 个运动规划器
 * @details 复位首次调用标志、堵转/过载标志与两个故障计时；若已通过 Motor_SetConfig() 设置配置，
 *          则把运动规划模块的全局配置指针 g_motion_config 指向本模块配置里的 motionParams，
 *          并依次初始化 CurrentTracker、VelocityTracker、PositionTracker、PositionInterpolator，
 *          最后用固定 200ms 超时初始化 TrajectoryTracker。
 * @note   在 main() 初始化阶段调用一次（主循环上下文，开启定时器之前）；
 *         必须先调用 Motor_SetConfig()，否则 s_config 为 NULL 时整段初始化被跳过，跑起来会解引用空指针。
 * @todo   BoardConfig_t 里的 defaultMode、enableMotorOnBoot 会从 EEPROM 读出并保存，但本模块与 main.c
 *         都没有读取它们：上电模式恒为 MODE_STOP、也不会自动使能电机，这两项配置目前是无效的半成品。
 */
void Motor_Init(void)
{
    s_first_called = true;
    s_is_stalled = false;
    s_overload_flag = false;
    s_stalled_time = 0;
    s_overload_time = 0;

    /* 初始化运动规划器 */
    if (s_config)
    {
        g_motion_config = &s_config->motionParams;
        CurrentTracker_Init();
        VelocityTracker_Init();
        PositionTracker_Init();
        PositionInterpolator_Init();
        TrajectoryTracker_Init(200);
    }
}

/**
 * @brief  设置整机配置指针
 * @param[in] config 配置结构体地址；本模块只保存指针不拷贝内容，该结构体必须在整个运行期有效
 *                   （main.c 中是静态变量 motor_config），其成员可在运行中被上位机改写
 * @note   必须在 Motor_Init() 之前调用；主循环上下文调用，20kHz 中断随后会读取同一份配置。
 */
void Motor_SetConfig(Motor_Config_t *config) { s_config = config; }

/**
 * @brief  20kHz 闭环控制主流程（一次控制周期，50us）
 * @details 执行顺序与各步骤要点：
 *          1) MT6816_UpdateAngle() 刷新编码器，再取校准后的单圈角度 rectified_angle（校准表已把机械角
 *             线性化成 0~51199 细分步，1 细分步 = 1 个 FOC 正弦表索引 = 1/1024 电周期）；
 *          2) 首次调用：把单圈角度平移一个整圈，使它与 encoderHomeOffset 的差落在半圈以内，
 *             用作多圈位置零点，然后直接 return（这一周期不做控制输出）；
 *          3) 之后每周期按增量累加多圈位置，增量超过半圈就回绕修正（跨 0 点处理）；
 *          4) 速度估计：位置差分 ×20000 作为瞬时速度，再与上周期速度做 31/32 泄漏积分的一阶低通
 *             （((v<<5)-v) 再 >>5，等效 v += (20000×Δp − v)/32，时间常数约 32 个控制周期 ≈ 1.6ms），
 *             得到 s_est_velocity（细分步/秒）；累加器 s_est_velocity_integral 只保留低 5 位余数；
 *          5) 估计位置：s_est_position = s_real_position + CompensateAdvancedAngle(s_est_velocity)，
 *             该值同时就是送 TB67H450 的 FOC 换相角；
 *          6) 控制分支：堵转/休眠请求/未校准 → 清积分并休眠；刹车 → 清积分并短接制动；
 *             否则按运行模式调用 CalcDceToOutput()（位置）、CalcPidToOutput()（速度）或
 *             CalcCurrentToOutput()（电流）；MODE_STOP 只让驱动休眠；
 *          7) 模式切换检测：请求模式与运行模式不同则同步并置软目标重建标志；
 *          8) 目标限幅后做运动规划，把目标平滑成软目标（软目标值在本周期末尾才更新，
 *             所以本周期的控制器用的是上一周期的软目标）；
 *          9) 故障检测：堵转判据为（电流模式且指令电流非零，或指令电流顶到 ratedCurrent）且估计速度
 *             低于 0.2 圈/秒，累计满 1000000us（1s）置 s_is_stalled；过载判据为"非电流模式下指令电流
 *             顶到 ratedCurrent"累计 1s，电流一降下来就清除（不锁存）；
 *          10) 状态机刷新 s_state（优先级：未校准 > 停止 > 堵转 > 过载 > 完成/运行中）。
 * @note   只在 TIM4 的 20kHz（50us）中断中调用（经 Tim4Callback20kHz()），且编码器校准触发期间该中断
 *         改走校准状态机、本函数不被调用；函数内不允许阻塞（printf 已被注释掉）。
 * @note   关闭堵转保护（stallProtectSwitch 为 false）时，s_stalled_time 既不累加也不清零，会保持原值，
 *         重新打开后接着上次的计时继续累计。过载检测不区分该开关，始终生效但只反映到状态上。
 * @warning 整个电流环节是开环的：s_foc_current 只是控制器算出的指令电流，工程内没有电流采样。
 * @todo   MODE_STOP 分支只调用 TB67H450_Sleep()，没有把 s_foc_current/s_foc_position 清零，
 *         因此停止后 Motor_GetCurrent() 仍会返回上一次的指令电流，过载检测也会看到这个旧值。
 */
void Motor_Tick20kHz(void)
{
    MT6816_UpdateAngle();
    /* 读取编码器角度*/
    uint16_t rectified_angle = 0;
    rectified_angle = MT6816_GetRectifiedAngle();

    /* 首次调用：初始化位置 */
    if (s_first_called)
    {
        int32_t angle;
        if (s_config->motionParams.encoderHomeOffset < MOTOR_SUBDIVIDE_STEPS / 2)
        {
            angle = (rectified_angle > s_config->motionParams.encoderHomeOffset + MOTOR_SUBDIVIDE_STEPS / 2)
                        ? rectified_angle - MOTOR_SUBDIVIDE_STEPS
                        : rectified_angle;
        }
        else
        {
            angle = (rectified_angle < s_config->motionParams.encoderHomeOffset - MOTOR_SUBDIVIDE_STEPS / 2)
                        ? rectified_angle + MOTOR_SUBDIVIDE_STEPS
                        : rectified_angle;
        }

        s_real_lap_position = angle;
        s_real_lap_position_last = angle;
        s_real_position = angle;
        s_real_position_last = angle;
        s_first_called = false;
        return;
    }

    /* 更新位置 */
    s_real_lap_position_last = s_real_lap_position;
    s_real_lap_position = rectified_angle;

    int32_t delta = s_real_lap_position - s_real_lap_position_last;
    if (delta > (MOTOR_SUBDIVIDE_STEPS >> 1))
        delta -= MOTOR_SUBDIVIDE_STEPS;
    else if (delta < -(MOTOR_SUBDIVIDE_STEPS >> 1))
        delta += MOTOR_SUBDIVIDE_STEPS;

    s_real_position_last = s_real_position;
    s_real_position += delta;

    /* 估计速度 */
    s_est_velocity_integral +=
        ((s_real_position - s_real_position_last) * CONTROL_FREQUENCY + ((s_est_velocity << 5) - s_est_velocity));
    s_est_velocity = s_est_velocity_integral >> 5;
    s_est_velocity_integral -= (s_est_velocity << 5);

    /* 估计位置（带超前角补偿）*/
    s_est_lead_position = CompensateAdvancedAngle(s_est_velocity);
    s_est_position = s_real_position + s_est_lead_position;

    /* 控制循环 */
    if (s_is_stalled || s_soft_disable || !EncoderCalibrator_IsCalibrated()) /* 休眠 */
    {
        ClearIntegral();
        s_foc_position = 0;
        s_foc_current = 0;
        TB67H450_Sleep();
    }
    else if (s_soft_brake) /* 刹车 */
    {
        ClearIntegral();
        s_foc_position = 0;
        s_foc_current = 0;
        TB67H450_Brake();
    }
    else
    {
        switch (s_mode_running)
        {
        case MODE_STOP:
            TB67H450_Sleep();
            break;
        case MODE_COMMAND_POSITION:
        case MODE_COMMAND_TRAJECTORY:
        case MODE_PWM_POSITION:
            CalcDceToOutput(s_soft_position, s_soft_velocity);
            break;
        case MODE_COMMAND_VELOCITY:
        case MODE_PWM_VELOCITY:
            CalcPidToOutput(s_soft_velocity);
            break;
        case MODE_COMMAND_CURRENT:
        case MODE_PWM_CURRENT:
            CalcCurrentToOutput(s_soft_current);
            break;
        default:
            break;
        }
    }

    /* 模式切换 */
    if (s_mode_running != s_request_mode)
    {
        s_mode_running = s_request_mode;
        s_soft_new_curve = true;
    }

    /* 限幅 */
    if (s_goal_velocity > s_config->motionParams.ratedVelocity)
        s_goal_velocity = s_config->motionParams.ratedVelocity;
    else if (s_goal_velocity < -s_config->motionParams.ratedVelocity)
        s_goal_velocity = -s_config->motionParams.ratedVelocity;
    if (s_goal_current > s_config->motionParams.ratedCurrent)
        s_goal_current = s_config->motionParams.ratedCurrent;
    else if (s_goal_current < -s_config->motionParams.ratedCurrent)
        s_goal_current = -s_config->motionParams.ratedCurrent;

    /* 运动规划 */
    if ((s_soft_disable && !s_goal_disable) || (s_soft_brake && !s_goal_brake))
    {
        s_soft_new_curve = true;
    }

    if (s_soft_new_curve)
    {
        s_soft_new_curve = false;
        ClearIntegral();
        Motor_ClearStallFlag();

        switch (s_mode_running)
        {
        case MODE_COMMAND_POSITION:
        case MODE_PWM_POSITION:
            PositionTracker_NewTask(s_est_position, s_est_velocity);
            break;
        case MODE_COMMAND_VELOCITY:
        case MODE_PWM_VELOCITY:
            VelocityTracker_NewTask(s_est_velocity);
            break;
        case MODE_COMMAND_CURRENT:
        case MODE_PWM_CURRENT:
            CurrentTracker_NewTask(s_foc_current);
            break;
        case MODE_COMMAND_TRAJECTORY:
            TrajectoryTracker_NewTask(s_est_position, s_est_velocity);
            break;
        default:
            break;
        }
    }

    /* 计算软目标 */
    switch (s_mode_running)
    {
    case MODE_COMMAND_POSITION:
    case MODE_PWM_POSITION:
        PositionTracker_CalcSoftGoal(s_goal_position);
        s_soft_position = g_go_location;
        s_soft_velocity = g_go_location_velocity;
        break;
    case MODE_COMMAND_VELOCITY:
    case MODE_PWM_VELOCITY:
        VelocityTracker_CalcSoftGoal(s_goal_velocity);
        s_soft_velocity = g_go_velocity;
        break;
    case MODE_COMMAND_CURRENT:
    case MODE_PWM_CURRENT:
        CurrentTracker_CalcSoftGoal(s_goal_current);
        s_soft_current = g_go_current;
        break;
    case MODE_COMMAND_TRAJECTORY:
        TrajectoryTracker_CalcSoftGoal(s_goal_position, s_goal_velocity);
        s_soft_position = g_traj_go_position;
        s_soft_velocity = g_traj_go_velocity;
        break;
    default:
        break;
    }

    s_soft_disable = s_goal_disable;
    s_soft_brake = s_goal_brake;

    /* 故障检测 */
    int32_t current_abs = abs(s_foc_current);

    if (s_config->ctrlParams.stallProtectSwitch)
    {
        if (((s_mode_running == MODE_COMMAND_CURRENT || s_mode_running == MODE_PWM_CURRENT) && current_abs != 0) ||
            current_abs == s_config->motionParams.ratedCurrent)
        {
            if (abs(s_est_velocity) < MOTOR_SUBDIVIDE_STEPS / 5)
            {
                if (s_stalled_time >= 1000 * 1000)
                {
                    s_is_stalled = true;
                }
                else
                {
                    s_stalled_time += CONTROL_PERIOD_US;
                }
            }
        }
        else
        {
            s_stalled_time = 0;
        }
    }

    /* 过载检测 */
    if ((s_mode_running != MODE_COMMAND_CURRENT) && (s_mode_running != MODE_PWM_CURRENT) &&
        current_abs == s_config->motionParams.ratedCurrent)
    {
        if (s_overload_time >= 1000 * 1000)
        {
            s_overload_flag = true;
        }
        else
        {
            s_overload_time += CONTROL_PERIOD_US;
        }
    }
    else
    {
        s_overload_time = 0;
        s_overload_flag = false;
    }

    /* 状态机 */
    if (!EncoderCalibrator_IsCalibrated())
    {
        s_state = STATE_NO_CALIB;
    }
    else if (s_mode_running == MODE_STOP)
    {
        s_state = STATE_STOP;
    }
    else if (s_is_stalled)
    {
        s_state = STATE_STALL;
    }
    else if (s_overload_flag)
    {
        s_state = STATE_OVERLOAD;
    }
    else
    {
        // 加上模式判断
        if (s_mode_running == MODE_COMMAND_POSITION)
        {
            if ((s_soft_position == s_goal_position) && (s_soft_velocity == 0))
                s_state = STATE_FINISH;
            else
                s_state = STATE_RUNNING;
        }
        else if (s_mode_running == MODE_COMMAND_VELOCITY)
        {
            if (s_soft_velocity == s_goal_velocity)
                s_state = STATE_FINISH;
            else
                s_state = STATE_RUNNING;
        }
        else if (s_mode_running == MODE_COMMAND_CURRENT)
        {
            if (s_soft_current == s_goal_current)
                s_state = STATE_FINISH;
            else
                s_state = STATE_RUNNING;
        }
        else
        {
            s_state = STATE_FINISH;
        }
    }
}

/* ==================== 控制接口 ==================== */
/**
 * @brief  设置请求模式
 * @param[in] mode 目标运行模式，取值见 Motor_Mode_t
 * @note   只写 s_request_mode，不立即生效：20kHz 中断在下一周期检测到差异后才同步成 s_mode_running，
 *         并置 s_soft_new_curve 触发一次软目标重建（清积分 + NewTask）。
 *         本函数不做合法性检查，传入超出枚举范围的值会被 switch 的 default 分支忽略。
 */
void Motor_SetMode(Motor_Mode_t mode) { s_request_mode = mode; }

/**
 * @brief  设置目标位置
 * @details 把入参加上 encoderHomeOffset 换算成编码器绝对坐标后写入 s_goal_position：
 *          用户坐标以 Home 为零点，而位置规划与编码器都以绝对坐标运算。
 * @param[in] pos 目标位置，单位 细分步（1 圈 = 51200 细分步），可为负、可超出多圈，不做范围检查
 * @note   只改目标值，真正的位置规划由 20kHz 中断里的 PositionTracker 完成；
 *         串口位置命令会把用户输入的圈数乘 51200 后传入，并先切换模式。
 */
void Motor_SetPosition(int32_t pos) { s_goal_position = pos + s_config->motionParams.encoderHomeOffset; }

/**
 * @brief  设置目标速度
 * @param[in] vel 目标速度，单位 细分步/秒（正负号决定方向）
 * @note   仅当 |vel| ≤ ratedVelocity 时才写入 s_goal_velocity，条件不满足时不做任何处理；
 *         写入后由 20kHz 中断里的 VelocityTracker 按 ratedVelocityAcc 平滑逼近。
 * @warning 超范围时静默忽略：不报错、不返回状态、也不改变原有目标值，
 *          调用方只能靠 Motor_GetVelocity() 间接确认是否生效。
 */
void Motor_SetVelocity(int32_t vel)
{
    if (vel >= -s_config->motionParams.ratedVelocity && vel <= s_config->motionParams.ratedVelocity)
    {
        s_goal_velocity = vel;
    }
}

/**
 * @brief  设置目标电流
 * @param[in] cur 目标电流，单位 mA（正负号决定方向）
 * @note   超出 ±ratedCurrent 时钳位到边界后写入 s_goal_current，因此实际目标可能与入参不同；
 *         只在电流模式（MODE_COMMAND_CURRENT / MODE_PWM_CURRENT）下才有控制作用。
 * @warning 该值是"指令电流"，没有实测电流反馈校验；电流模式下的软目标由 CurrentTracker 按
 *          ratedCurrentAcc 的变化率逼近它。
 */
void Motor_SetCurrent(int32_t cur)
{
    if (cur > s_config->motionParams.ratedCurrent)
        s_goal_current = s_config->motionParams.ratedCurrent;
    else if (cur < -s_config->motionParams.ratedCurrent)
        s_goal_current = -s_config->motionParams.ratedCurrent;
    else
        s_goal_current = cur;
}

/**
 * @brief  设置休眠请求
 * @param[in] disable true 请求休眠（驱动断使能），false 解除休眠
 * @note   写 s_goal_disable，中断末尾同步到 s_soft_disable；状态翻转时会触发软目标重建（清积分）。
 * @todo   当前工程内没有调用者，休眠/使能暂无外部入口。
 */
void Motor_SetDisable(bool disable) { s_goal_disable = disable; }

/**
 * @brief  设置刹车请求
 * @param[in] brake true 请求刹车（TB67H450 两相同时导通，绕组短接制动），false 解除刹车
 * @note   写 s_goal_brake，中断末尾同步到 s_soft_brake；状态翻转时会触发软目标重建（清积分）。
 *         刹车分支优先级低于休眠/堵转，高于正常控制。
 * @todo   当前工程内没有调用者。
 */
void Motor_SetBrake(bool brake) { s_goal_brake = brake; }

/**
 * @brief  清除堵转标志：清零堵转累计时间并复位 s_is_stalled
 * @details 堵转判定后驱动会被强制休眠，只有调用本函数才能退出（状态机回到 STATE_STOP/STATE_RUNNING）。
 *          软目标重建时 motor.c 内部也会调用它，使每次换模式/重新使能都重新开始计时。
 * @note   由 100Hz 任务的按键事件或串口命令 'l' 调用（主循环上下文）；
 *         清零与中断里的累加存在竞争窗口，最坏情况下丢失一个周期的计时，无实质影响。
 */
void Motor_ClearStallFlag(void)
{
    s_stalled_time = 0;
    s_is_stalled = false;
}

/* ==================== 状态读取 ==================== */
/**
 * @brief  读取电机状态
 * @return 最近一次 20kHz 中断刷新的状态，取值见 Motor_State_t
 * @note   直接返回 s_state，无副作用，主循环/中断均可调用。
 */
Motor_State_t Motor_GetState(void) { return s_state; }

/**
 * @brief  读取当前位置
 * @param[in] isLap true 取单圈内的角度（0~51199 细分步），false 取累计多圈位置
 * @return 位置，单位：圈；返回值为 (内部细分步位置 - encoderHomeOffset) / 51200，
 *         因此 Home 点读数为 0，isLap 为 true 时结果落在 ±1 圈内
 * @note   读取 20kHz 中断更新的快照 s_real_lap_position / s_real_position；
 *         isLap 分支只减 Home 偏移、不做 0~51200 归一化，所以结果可能为负值。
 */
float Motor_GetPosition(bool isLap)
{
    if (isLap)
    {
        return (float)(s_real_lap_position - s_config->motionParams.encoderHomeOffset) / (float)MOTOR_SUBDIVIDE_STEPS;
    }
    else
    {
        return (float)(s_real_position - s_config->motionParams.encoderHomeOffset) / (float)MOTOR_SUBDIVIDE_STEPS;
    }
}

/**
 * @brief  读取估计速度
 * @return 估计速度，单位：圈/秒；等于内部 s_est_velocity（细分步/秒）除以 51200
 * @note   内部值来自 20kHz 位置差分的一阶低通（时间常数约 32 个控制周期），不是硬件测速。
 */
float Motor_GetVelocity(void) { return (float)s_est_velocity / (float)MOTOR_SUBDIVIDE_STEPS; }

/**
 * @brief  读取当前指令电流
 * @return 指令电流，单位：A；等于内部 s_foc_current（mA）除以 1000
 * @warning 返回的是控制器算出的指令电流，不是实测相电流：本工程没有电流采样，
 *          且 MODE_STOP 分支不会清零 s_foc_current，停止后读到的可能是停机前的旧值。
 */
float Motor_GetCurrent(void) { return (float)s_foc_current / 1000.0f; }

/**
 * @brief  查询编码器是否已校准
 * @return true 已校准（Flash 中有有效校准表或本次标定建表成功）；false 未校准
 * @note   转发编码器校准模块的状态，本身不访问硬件；未校准时 20kHz 中断让驱动休眠、不进入闭环。
 */
bool Motor_IsCalibrated(void)
{
    // 直接从编码器校准模块获取状态
    return EncoderCalibrator_IsCalibrated();
}

/**
 * @brief  请求触发一次编码器校准
 * @details 转发给 EncoderCalibrator_Trigger()；该模块只在"尚未校准且未在校准中"时才真正置位触发标志。
 *          触发后 Tim4Callback20kHz() 会改走校准状态机，本模块的闭环控制暂停，电机由校准状态机
 *          以固定电流矢量开环拖动。
 * @note   主循环上下文调用（main.c 检测到两个按钮同时按下，或上位机命令）；
 *         调用前应保证电机处于可安全开环转动的状态。
 */
void Motor_TriggerCalibration(void)
{
    // 触发编码器校准
    EncoderCalibrator_Trigger();
}

/**
 * @brief  读取当前运行模式
 * @return 运行模式（s_mode_running 的数值，对应 Motor_Mode_t），不是最近一次请求的模式
 * @note   返回类型是 uint8_t，与 Motor_Mode_t 比较时由调用方隐式转换；
 *         100Hz 任务用它与 MODE_STOP 比较来判断当前是否停机。
 */
uint8_t Motor_GetMode(void) { return s_mode_running; }

/**
 * @brief  一次性读取全部遥测数据
 * @param[out] pos   位置，单位：圈（等价于 Motor_GetPosition(false)）
 * @param[out] vel   估计速度，单位：圈/秒（等价于 Motor_GetVelocity()）
 * @param[out] cur   指令电流，单位：A（等价于 Motor_GetCurrent()）
 * @param[out] mode  运行模式数值（Motor_Mode_t）
 * @param[out] state 电机状态数值（Motor_State_t）
 * @note   五个指针都必须由调用方保证非空（本函数不判空）；各值来自同一次调用时刻的静态变量快照，
 *         但中断可能在任何一条语句之间更新它们，因此五个值未必来自同一个控制周期。
 * @todo   当前工程内没有调用者（main.c 分别调用各个单独接口），是给上位机遥测预留的便捷接口。
 */
void Motor_GetTelemetry(float *pos, float *vel, float *cur, uint8_t *mode, uint8_t *state)
{
    *pos = Motor_GetPosition(false);
    *vel = Motor_GetVelocity();
    *cur = Motor_GetCurrent();
    *mode = s_mode_running;
    *state = s_state;
}

/**
 * @brief  把当前位置记为零点：更新 encoderHomeOffset 并整块写回 EEPROM
 * @details 取当前累计位置对一圈取模作为新的单圈零点，写入配置（立即影响位置/速度的坐标原点与
 *          后续 Motor_SetPosition() 的换算），再把全局 boardConfig 整块写入 EEPROM 起始地址持久化。
 * @note   由串口命令 'z' 调用（主循环上下文）；EEPROM 写入是阻塞操作，不应在中断中调用。
 * @note   取模结果保留符号：s_real_position 为负时 % 得到负值，encoderHomeOffset 也会是负偏移，
 *         代码没有把它归一化到 0~51199，属实现细节，本次不改动行为。
 */
void Motor_ZeroPosition(void)
{
    // 把当前位置设为新的 HomeOffset
    s_config->motionParams.encoderHomeOffset = s_real_position % MOTOR_SUBDIVIDE_STEPS;

    // 保存到 EEPROM
    boardConfig.encoderHomeOffset = s_config->motionParams.encoderHomeOffset;
    EEPROM_Write(0, &boardConfig, sizeof(BoardConfig_t));
}
