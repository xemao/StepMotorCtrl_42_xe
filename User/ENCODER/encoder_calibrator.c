/**
 ******************************************************************************
 * @file    encoder_calibrator.c
 * @brief   磁编码器安装误差标定模块：采集一圈角度数据、校验后生成线性化查表并写入 Flash 分区。
 * @details 标定流程由两级上下文协作完成：
 *          1) 20kHz 中断里的实时采样状态机 EncoderCalibrator_Tick20kHz()：以开环电流矢量驱动电机，
 *             先正向走一圈（每个机械位置连续采样 SAMPLE_PER_STEP 次取平均），再反向走一圈消除回差；
 *          2) 主循环里的后处理 EncoderCalibrator_TickMainLoop()：数据校验（方向/连续性/相位跳变个数）
 *             → 线性插值生成 16384 点查表数据 → 写入 Flash 分区后系统复位。
 *          数据单位：位置为细分步（SUBDIVIDE_STEPS = 51200 细分步/圈）；原始角度均为 14 位编码器刻度
 *          （0~16383 对应 0~360°，1 刻度 ≈ 0.02197°）；sample_fwd/sample_rev 中每个
 *          元素是"一个机械步位置"上的平均原始角度，共 HARD_STEPS + 1 = 201 个点（多采 1 点用于首尾闭合）。
 *          注意校准表存的是"校正位置"（细分步，0~51199）而非角度刻度，MT6816 查表后直接把它当作位置量输出。
 *          本模块与 MT6816 驱动（取原始角度、设置校准表指针）、TB67H450 驱动（输出 FOC 电流矢量）、
 *          Stockpile Flash 分区（存放校准表）耦合，Motor 模块不参与标定过程。
 * @note    校准完成前闭环会被禁止：标定触发期间 Tim4Callback20kHz() 只调用本模块的状态机而不再调用
 *          Motor_Tick20kHz()，且 Motor 侧以 EncoderCalibrator_IsCalibrated() 作为门控（未校准时休眠/不使能），
 *          因此校准期间电机仅按本模块给定的电流矢量开环转动。
 *          状态机运行在 20kHz（50us）中断中，查表生成与 Flash 擦写运行在主循环中（耗时较长，会阻塞主循环）。
 *          校准表写入 STOCKPILE_APP_CALI_ADDR 分区；生成成功后调用 HAL_NVIC_SystemReset() 复位。
 * @warning 校准表与 Flash 分区容量强耦合：分区 STOCKPILE_APP_CALI_ADDR 共 32KB，恰好等于
 *          16384 × sizeof(uint16_t)，即 14 位编码器的满量程表。改动 ENC_RESOLUTION、编码器位数或分区大小
 *          都会导致越界擦写或数据截断，必须同步修改（MT6816 侧同样按 16384 点查表）。
 * @warning 校准状态只由 cali_error 与 cali_is_calibrated 反映：未校准时 Motor 侧会禁止闭环并点 NO_CALIB 灯；
 *          校准失败（cali_error != 0）时不会写入有效表、也不会复位，此时不应继续运行闭环控制。
 * @todo  错误码链路尚未闭环：cali_error 的 1/2/3/4 只被赋值、无人消费，状态机失败后也没有重试或降级路径；
 *        状态 6 的分支只负责停机清零，真正决定成败的 CheckData()/GenerateTable() 全在主循环里，
 *        这些"只设置不处理"的分支建议后续统一梳理并上报。
 ******************************************************************************
 */

#include "encoder_calibrator.h"
#include "mt6816.h"
#include "stockpile_f103cb.h"
#include "tb67h450.h"
#include <stdio.h>
#include <stdlib.h>

/* ==================== 常量定义 ==================== */

/**
 * @brief 电机整步步数：一圈 200 个整步（1.8° 步距角）。
 */
#define HARD_STEPS      200
/**
 * @brief 单个机械步位置上的采样次数：连续取 16 次原始角度再求平均，用于抑制单次读数噪声。
 */
#define SAMPLE_PER_STEP 16
/**
 * @brief 状态 1（正向准备）时每个 20kHz 周期的位置增量，单位：细分步/周期。
 * @note  2 细分步/周期 × 20kHz = 40000 细分步/s。
 */
#define AUTO_SPEED      2
/**
 * @brief 状态 2/3/4/5 时每个 20kHz 周期的位置增量，单位：细分步/周期。
 * @note  1 细分步/周期 × 20kHz = 20000 细分步/s。
 */
#define FINE_SPEED      1
/**
 * @brief 编码器 14 位分辨率：2^14 = 16384 刻度/圈。
 * @note  原始角度与校正角度的取值范围均为 0~16383。
 */
#define ENC_RESOLUTION  16384
/**
 * @brief 软细分数：把一个整步再细分为 256 个细分步，也是采样点的位置间距（细分步/采样点）。
 */
#define SOFT_DIVIDE     256
/**
 * @brief 一圈细分步数 = HARD_STEPS × SOFT_DIVIDE = 200 × 256 = 51200 细分步/圈。
 * @note  用于换算为 FOC 电流矢量的位置输入。
 */
#define SUBDIVIDE_STEPS 51200
/* ==================== 全局变量 ==================== */

/**
 * @brief 校准触发标志：true 表示校准流程已启动且尚未结束。
 * @note  由 EncoderCalibrator_Trigger() 置位，EncoderCalibrator_TickMainLoop() 收尾时清零；
 *        20kHz 中断据此决定是否走校准分支。
 */
static bool cali_triggered = false;

/**
 * @brief 是否已有可用校准表。
 * @note  EncoderCalibrator_Init() 检测到 Flash 内有效数据时置位，或 GenerateTable()
 *        生成满 16384 点时置位。
 */
static bool cali_is_calibrated = false;

/**
 * @brief 标定错误码，由 CheckData() / GenerateTable() 写入，0 表示无错误。
 * @note  取值含义：
 *        - 0：无错误，数据校验通过且校准表点数正确；
 *        - 1：方向错误（首尾角度差为 0，或相邻点递增/递减方向与首尾判断不一致）；
 *        - 2：数据不连续（相邻点角度差超出 [step_res/2, step_res*3/2]，step_res = 16384/200）；
 *        - 3：相位错误（一圈内检测到的 3/4→1/4 跳变点个数不等于 1）；
 *        - 4：数量错误（写入 Flash 的校准表点数不等于 ENC_RESOLUTION = 16384）。
 */
static int cali_error = 0;

/**
 * @brief 实时采样状态机状态（只在 20kHz 中断中推进，主循环把 6 当作"可计算"标志）。
 * @note  取值含义：
 *        - 0：空闲，等待 cali_triggered 置位；
 *        - 1：正向准备，从当前位置加速移动到一圈起点 2 × SUBDIVIDE_STEPS；
 *        - 2：正向测量，正转一圈并在每个 SOFT_DIVIDE 位置采样；
 *        - 3：反向返回，继续正转越过起点 20 × SOFT_DIVIDE 细分步（为消隙预留过冲量）；
 *        - 4：消除间隙，反转回到 2 × SUBDIVIDE_STEPS（消除齿轮/传动回差）；
 *        - 5：反向测量，反转一圈并在每个 SOFT_DIVIDE 位置采样；
 *        - 6：计算，停止电流输出，交由主循环做校验与建表。
 */
static int cali_state = 0;

/**
 * @brief 当前给定位置，单位：细分步，一圈 = SUBDIVIDE_STEPS = 51200。
 * @note  状态 1/2/3 递增、状态 4/5 递减；其低 10 位被 TB67H450_SetFocCurrentVector()
 *        直接当作 FOC 电角度查表下标，因此位置本身即开环换相角。
 */
static uint32_t go_pos = 0;

/**
 * @brief 测量方向标志：由 CheckData() 依据 sample_fwd 首尾角度差的符号确定。
 * @note  CycleSub 结果 > 0 为 true，< 0 为 false；GenerateTable() 据此选择插值分支。初值 true。
 */
static bool go_dir = true;

/**
 * @brief 当前机械步位置已累计的采样次数，取值 0 ~ SAMPLE_PER_STEP。
 * @note  攒满 16 次后求平均并清零。
 */
static uint16_t sample_cnt = 0;

/**
 * @brief 单步采样缓冲区：暂存同一机械位置连续 16 次 MT6816_GetRawAngle() 的原始角度（0~16383）。
 */
static uint16_t sample_raw[SAMPLE_PER_STEP];

/**
 * @brief 正向测量结果：下标为机械步序号（0 ~ HARD_STEPS），值为该位置 16 次采样的平均原始角度（0~16383）。
 */
static uint16_t sample_fwd[HARD_STEPS + 1];

/**
 * @brief 反向测量结果：下标含义同上，值为反向回程在同一位置的平均原始角度（0~16383），与正向数据取平均用。
 */
static uint16_t sample_rev[HARD_STEPS + 1];

/**
 * @brief 记录相位跳变（原始角度由 3/4 圈以上跌落到 1/4 圈以下）所在的位置。
 * @note  rcd_x = 跳变前的机械步序号（0 ~ HARD_STEPS-1）；
 *        rcd_y = (ENC_RESOLUTION-1) - 跳变前的原始角度，即跳变点在整步内剩余的刻度数。
 */
static int32_t rcd_x = 0, rcd_y = 0;

/**
 * @brief GenerateTable() 实际写入 Flash 的校准表点数（单位：个 uint16_t 条目）。
 * @note  等于 ENC_RESOLUTION 才算成功。
 */
static uint32_t result_num = 0;

/**
 * @brief 校准表基址指针（uint16_t 寻址）。
 * @note  指向 Flash 分区 STOCKPILE_APP_CALI_ADDR = 0x08007C00，容量 32KB，共
 *        ENC_RESOLUTION = 16384 个 uint16_t 条目；下标为原始角度（0~16383），
 *        内容为校正位置（单位：细分步）。同一块内存既被 MT6816_SetCalibrationData()
 *        引用作查表依据，也被 GenerateTable() 写入流程使用。
 */
static uint16_t *cali_table = (uint16_t *)STOCKPILE_APP_CALI_ADDR;

/* ==================== 辅助函数 ==================== */

/**
 * @brief  对位置/角度下标做循环取模，把任意下标折回 [0, b) 区间。
 * @details 环绕处理原则：编码器角度与机械位置都是首尾相接的环形量，越过一圈边界后必须回到区间起点。
 *          实现上先加 b 再取模（(a + b) % b），因此即使传入的 a 略大于一圈（例如建表时
 *          SOFT_DIVIDE * x + SOFT_DIVIDE * y / data 可能超过 SUBDIVIDE_STEPS 一个步长），结果仍落在 [0, b) 内。
 * @param[in] a 被除数/待归一化的下标（无符号，允许大于 b）。
 * @param[in] b 除数，即一圈的总刻度数或数组长度（如 SUBDIVIDE_STEPS = 51200）。
 * @return (a + b) % b，取值范围 [0, b)。
 * @note 仅在主循环的建表流程（GenerateTable）中调用，不参与 20kHz 中断。
 */
static uint32_t CycleMod(uint32_t a, uint32_t b) { return (a + b) % b; }

/**
 * @brief  环形（跨 0°）角度减法：求 a 相对 b 的最短有符号角度差。
 * @details 环绕处理原则：两个角度直接相减会在一圈边界处产生伪大跳变，例如原始角度从 16300 跨到 100，
 *          直接相减得到 -16200，而真实角度差只有 +184。本函数把差值折到半圈以内：
 *          差值大于 cyc/2 就减去一圈，小于 -cyc/2 就加上一圈，从而得到"物理上最近"的角度差，
 *          可直接用于判断转向（符号）与步进幅度（绝对值）。
 * @param[in] a 角度 A，单位：编码器刻度（0 ~ cyc-1）。
 * @param[in] b 角度 B，单位：编码器刻度（0 ~ cyc-1）。
 * @param[in] cyc 一圈的总刻度数，标定场景固定传入 ENC_RESOLUTION = 16384。
 * @return a 与 b 的最短有符号差值，范围 (-cyc/2, cyc/2]，单位：编码器刻度。
 * @code
 * // 例：角度 10° 与 350°，本函数返回 +20°（即 10 - 350 + 360），而不是 -340°。
 * @endcode
 * @note 用于 CheckData() 的方向/连续性判断与 GenerateTable() 的相邻采样点角度差计算；主循环上下文调用。
 */
static int32_t CycleSub(int32_t a, int32_t b, int32_t cyc)
{
    int32_t sub = a - b;
    if (sub > (cyc >> 1))
        sub -= cyc; /* 超过半圈，减一圈 */
    if (sub < (-(cyc >> 1)))
        sub += cyc; /* 低于负半圈，加一圈 */
    return sub;
}

/**
 * @brief  环形（跨 0°）角度平均：求两个角度的物理中点。
 * @details 环绕处理原则：算术平均 (a + b) / 2 在两个角度分处一圈两侧时会得到相反方向的错误结果，
 *          例如 10° 与 350° 的算术平均是 180°，但两者实际相距 20°，中点应为 0°。
 *          做法是先取算术平均 ave，再用 |a - b| 判断两点是否跨过 0°（差值超过半圈即认为跨零），
 *          若是则把 ave 平移半圈，从而把结果折回跨零一侧的中点。
 * @param[in] a 角度 A，单位：编码器刻度（0 ~ cyc-1）。
 * @param[in] b 角度 B，单位：编码器刻度（0 ~ cyc-1）。
 * @param[in] cyc 一圈的总刻度数，标定场景固定传入 ENC_RESOLUTION = 16384。
 * @return 两个角度的环形平均值，单位：编码器刻度。
 * @code
 * // 例：角度 10° 和 350° 的平均值是 0° 而不是 180°。
 * @endcode
 * @note 在 CheckData() 中用于把同一机械位置的正向、反向平均角度合并（消回差）；主循环上下文调用。
 */
static int32_t CycleAvg(int32_t a, int32_t b, int32_t cyc)
{
    int32_t sub = a - b;
    int32_t ave = (a + b) >> 1;
    if (abs(sub) > (cyc >> 1))
    {
        if (ave >= (cyc >> 1))
            ave -= (cyc >> 1);
        else
            ave += (cyc >> 1);
    }
    return ave;
}

/**
 * @brief  多数据环形（跨 0°）角度平均：对同一位置的一串采样值求平均并处理跨 0° 环绕。
 * @details 环绕处理原则：无符号原始角度在 0° 附近会由 16383 跳回 0，若直接累加求和会被这一跳变严重拉偏。
 *          做法是以 data[0] 为基准，对每个元素先算它与基准的差值 sub：sub 超过半圈就减一圈、
 *          低于负半圈就加一圈，把整串数据"展开"到基准附近的连续区间后再求和平均；
 *          最后把结果归一化回 [0, cyc)（小于 0 加一圈，大于一圈减一圈）。
 *          注意求平均用的是整数除法，负数时按 C 语言规则向 0 截断，结果可能与真实均值相差 1 个刻度以内。
 * @param[in] data 采样数据数组，元素为原始角度，单位：编码器刻度（0 ~ cyc-1）。
 * @param[in] len 数组有效长度，标定场景传入 SAMPLE_PER_STEP = 16；要求 len >= 1。
 * @param[in] cyc 一圈的总刻度数，标定场景固定传入 ENC_RESOLUTION = 16384。
 * @return 展开后求得的平均值，已归一化到 [0, cyc]，单位：编码器刻度。
 * @warning 求平均用的是整数除法（sum / len），当 sum 为负时按 C 语言规则向 0 截断，结果与真实均值最多相差
 *          1 个刻度；这也使本函数返回值与返回值类型 int32_t 之间没有小数补偿，标定精度受此限制（约 0.022°）。
 * @note 在 20kHz 中断的状态 2/5 中，每攒满 SAMPLE_PER_STEP 次采样调用一次，把平均值写入 sample_fwd / sample_rev。
 */
static int32_t CycleDataAvg(const uint16_t *data, uint16_t len, int32_t cyc)
{
    int32_t sum = data[0];
    for (uint16_t i = 1; i < len; i++)
    {
        int32_t diff = data[i];
        int32_t sub = data[i] - data[0];
        /* 以第一个数据为基准，处理其他数据的环绕 */
        if (sub > (cyc >> 1))
            diff = data[i] - cyc;
        if (sub < (-(cyc >> 1)))
            diff = data[i] + cyc;
        sum += diff;
    }
    sum = sum / len;
    /* 将结果归一化到[0, cyc)范围 */
    if (sum < 0)
        sum += cyc;
    if (sum > cyc)
        sum -= cyc;
    return sum;
}

/* ==================== 数据检查 ==================== */

/**
 * @brief  校验正向/反向采样数据的有效性，并确定方向标志与相位跳变点。
 * @details 处理步骤（与代码中的注释序号一一对应）：
 *          1. 把同一机械步位置的正向与反向平均角度用 CycleAvg() 环形平均合并（抵消回差），结果写回 sample_fwd；
 *          2. 方向检查：以 sample_fwd[0] 与 sample_fwd[HARD_STEPS-1] 的环形差值判断整体增/减方向，
 *             结果存入 go_dir；差值为 0 视为方向错误；
 *          3. 连续性检查：相邻点环形差 step_res = ENC_RESOLUTION / HARD_STEPS（16384/200 = 81 刻度，约 1.8°）
 *             必须落在 [step_res/2, step_res*3/2] 内，越界记为数据不连续；
 *             差值为 0 或增/减方向与 go_dir 不一致记为方向错误；
 *          4. 相位跳变检查：扫描相邻点，统计原始角度由 3/4 圈以上（> 12288）跌落到 1/4 圈以下（< 4096）的次数，
 *             记录跳变位置 rcd_x 与 rcd_y = (ENC_RESOLUTION - 1) - 跳变前角度；跳变点个数必须为 1，否则记为相位错误。
 *          任一步失败立即返回，不再写 go_dir/后续结果（步 3 中途失败会保留已求得的方向、跳变点保持默认值 0）。
 * @note 仅在主循环调用（由 EncoderCalibrator_TickMainLoop() 在 cali_state == 6 时调用）。
 *       输出：cali_error（0 成功，1/2/3 分别对应方向/连续性/相位错误）、go_dir、rcd_x、rcd_y，
 *       并通过 printf 打印诊断信息。
 *       注意：步骤 2 用首尾两点定方向，因此若采样整体发生整圈错位（首尾落在同一机械位置），会被判为方向错误。
 *       本函数只判定这 3 类错误；错误码 4（数量错误）由 GenerateTable() 在写入点数不符时设置。
 * @warning 本函数一旦在中途判定失败就立即返回，此时 rcd_x / rcd_y 可能仍是上一轮的旧值（或默认 0），
 *          cali_error != 0 时不要使用这两个值；同时 sample_fwd 已被正向/反向平均覆盖，原始正反向数据不可再分离。
 */
static void CheckData(void)
{
    int32_t sub;
    int32_t step_res = ENC_RESOLUTION / HARD_STEPS;
    uint32_t step_num = 0;

    /* 1. 正向和反向数据平均 */
    for (int i = 0; i < HARD_STEPS + 1; i++)
    {
        sample_fwd[i] = CycleAvg(sample_fwd[i], sample_rev[i], ENC_RESOLUTION);
    }

    /* 2. 检查方向 */
    sub = CycleSub(sample_fwd[0], sample_fwd[HARD_STEPS - 1], ENC_RESOLUTION);
    if (sub == 0)
    {
        cali_error = 1;
        printf("Error: Direction zero\r\n");
        return;
    }
    go_dir = (sub > 0);

    /* 3. 检查连续性 */
    for (int i = 1; i < HARD_STEPS; i++)
    {
        sub = CycleSub(sample_fwd[i], sample_fwd[i - 1], ENC_RESOLUTION);
        if (abs(sub) > (step_res * 3 / 2))
        {
            cali_error = 2;
            printf("Error: Continuity large, i=%d\r\n", i);
            return;
        }
        if (abs(sub) < (step_res * 1 / 2))
        {
            cali_error = 2;
            printf("Error: Continuity small, i=%d\r\n", i);
            return;
        }
        if (sub == 0)
        {
            cali_error = 1;
            printf("Error: Zero delta, i=%d\r\n", i);
            return;
        }
        if ((sub > 0) != go_dir)
        {
            cali_error = 1;
            printf("Error: Direction mismatch, i=%d\r\n", i);
            return;
        }
    }

    /* 4. 找跳跃点：直接用原始值判断过零 */
    for (int i = 0; i < HARD_STEPS; i++)
    {
        int32_t curr = sample_fwd[i];
        int32_t next = sample_fwd[i + 1];
        if (curr > (ENC_RESOLUTION * 3 / 4) && next < (ENC_RESOLUTION / 4))
        {
            step_num++;
            rcd_x = i;
            rcd_y = (ENC_RESOLUTION - 1) - curr;
        }
    }

    if (step_num != 1)
    {
        cali_error = 3;
        printf("Error: Phase step, num=%u\r\n", step_num);
    }
    else
    {
        cali_error = 0;
        printf("CheckData PASS, rcd_x=%d, rcd_y=%d\r\n", rcd_x, rcd_y);
    }
}
// static void CheckData(void)
//{
//     int32_t sub;
//     int32_t step_res = ENC_RESOLUTION / HARD_STEPS;
//     char buf[128];
//
//     // 1. 平均
//     for (int i = 0; i < HARD_STEPS + 1; i++) {
//         sample_fwd[i] = CycleAvg(sample_fwd[i], sample_rev[i], ENC_RESOLUTION);
//     }
//
//     // 2. 方向检查
//     sub = CycleSub(sample_fwd[0], sample_fwd[HARD_STEPS - 1], ENC_RESOLUTION);
//     if (sub == 0) {
//         cali_error = 1;
//         Uart_SendString("Error: Direction zero\r\n");
//         return;
//     }
//     go_dir = (sub > 0);
//
//     // 3. 连续性检查
//     for (int i = 1; i < HARD_STEPS; i++) {
//         sub = CycleSub(sample_fwd[i], sample_fwd[i-1], ENC_RESOLUTION);
//         if (abs(sub) > (step_res * 3 / 2)) {
//             cali_error = 2;
//             Uart_SendString("Error: Continuity too large\r\n");
//             return;
//         }
//         if (abs(sub) < (step_res * 1 / 2)) {
//             cali_error = 2;
//             Uart_SendString("Error: Continuity too small\r\n");
//             return;
//         }
//         if (sub == 0) {
//             cali_error = 1;
//             Uart_SendString("Error: Zero delta\r\n");
//             return;
//         }
//         if ((sub > 0) && (!go_dir)) {
//             cali_error = 1;
//             Uart_SendString("Error: Direction mismatch\r\n");
//             return;
//         }
//         if ((sub < 0) && (go_dir)) {
//             cali_error = 1;
//             Uart_SendString("Error: Direction mismatch\r\n");
//             return;
//         }
//     }
//
//     // 4. 跳跃点检测（宽松版）
//     uint32_t step_num = 0;
//     for (int i = 0; i < HARD_STEPS; i++) {
//         int32_t curr = sample_fwd[i];
//         int32_t next = sample_fwd[i+1];
//         if (curr > 12000 && next < 4000) {
//             step_num++;
//             rcd_x = i;
//             rcd_y = (ENC_RESOLUTION - 1) - curr;
//         }
//     }
//
//     if (step_num == 0) {
//         // 如果没找到跳跃点，强制设一个
//         step_num = 1;
//         rcd_x = 100;
//         rcd_y = 0;
//         Uart_SendString("Warning: No jump point found, forced\r\n");
//     } else if (step_num > 1) {
//         cali_error = 3;
//         Uart_SendString("Error: Multiple jump points\r\n");
//         return;
//     }
//
//     cali_error = 0;
//     Uart_SendString("CheckData PASS\r\n");
// }
/* ==================== 生成校准表 ==================== */

/**
 * @brief  由合并后的采样数据线性插值生成 16384 点校准映射表，并写入 Flash 校准分区。
 * @details 表结构：以原始角度为下标（0 ~ ENC_RESOLUTION-1，共 ENC_RESOLUTION = 16384 个 uint16_t 条目，
 *          即 16384 × 2 = 32768 字节 = 32KB），表项内容是该原始角度对应的校正位置，单位与位置量相同
 *          （细分步，0 ~ SUBDIVIDE_STEPS-1，由 SOFT_DIVIDE * x + SOFT_DIVIDE * y / data 归一化得到）；
 *          写入分区 stockpile_quick_cali（起始地址 STOCKPILE_APP_CALI_ADDR = 0x08007C00，分区容量 32KB）。
 *          建表过程：先调用 Stockpile_Flash_Data_Empty() 擦除分区、Stockpile_Flash_Data_Begin() 开始写入，
 *          然后对每个机械步区间 [x, x+1] 求出两端采样点的环形角度差 data，在该区间内逐点线性插值，
 *          用 CycleMod() 把 SOFT_DIVIDE * x + SOFT_DIVIDE * y / data 折算成位置量并归一化到
 *          [0, SUBDIVIDE_STEPS)（单位：细分步），得到该原始角度对应的校正位置，最后
 *          Stockpile_Flash_Data_End() 结束写入。
 *          go_dir 为 true 时按 x 递增、表项递增的次序写入（正转分支）；go_dir 为 false 时按 x 递减、
 *          表项递减的次序写入（反转分支，插值公式相应变为 SOFT_DIVIDE * (x + 1) - SOFT_DIVIDE * y / data），
 *          两个分支的首尾区间使用 rcd_y 作为半区间长度，使一圈闭合。
 * @note 仅在主循环调用（CheckData() 校验通过后由 EncoderCalibrator_TickMainLoop() 调用）。
 *       副作用：擦写 Flash 分区（耗时较长，会阻塞主循环）、更新 result_num；
 *       当 result_num 恰为 ENC_RESOLUTION 时置位 cali_is_calibrated，否则置 cali_error = 4（数量错误）。
 *       插值公式含除以 data 的运算，依赖 CheckData() 已保证相邻点差值非 0 且方向一致（data 不为 0）。
 * @warning 写入量与 Flash 分区容量强耦合：必须恰好写入 ENC_RESOLUTION = 16384 个 uint16_t（32KB）。
 *          若 HARD_STEPS、SOFT_DIVIDE、ENC_RESOLUTION 与分区容量（STOCKPILE_APP_CALI_SIZE）不同步修改，
 *          会出现只写了一半（点数不符 → cali_error = 4）或越界写坏相邻分区的风险。
 * @warning 表项内容按位置量（细分步，0 ~ 51199）写入，而查表下标范围只有 0 ~ 16383（14 位编码器刻度），
 *          入参与存储量纲不同：每 1 个编码器刻度对应约 3.125 个细分步，因此位置反馈的分辨率仍受
 *          编码器 14 位分辨率限制；若上层把该表项直接与细分步目标位置比较，需注意量纲是否匹配。
 */
static void GenerateTable(void)
{
    int32_t data;
    uint16_t val;

    result_num = 0;

    /* 擦除并开始写入Flash */
    Stockpile_Flash_Data_Empty(&stockpile_quick_cali);
    Stockpile_Flash_Data_Begin(&stockpile_quick_cali);

    if (go_dir)
    {
        /* 正转方向：线性插值生成校准表 */
        for (int x = rcd_x; x < rcd_x + HARD_STEPS + 1; x++)
        {
            /* 计算相邻两个采样点之间的角度差 */
            data =
                CycleSub(sample_fwd[CycleMod(x + 1, HARD_STEPS)], sample_fwd[CycleMod(x, HARD_STEPS)], ENC_RESOLUTION);

            /* 确定插值范围 */
            int start_y = (x == rcd_x) ? rcd_y : 0;
            int end_y = (x == rcd_x + HARD_STEPS) ? rcd_y : data;

            /* 在两点之间线性插值 */
            for (int y = start_y; y < end_y; y++)
            {
                val = CycleMod(SOFT_DIVIDE * x + SOFT_DIVIDE * y / data, SUBDIVIDE_STEPS);
                Stockpile_Flash_Data_Write_Data16(&stockpile_quick_cali, &val, 1);
                result_num++;
            }
        }
    }
    else
    {
        /* 反转方向：线性插值生成校准表 */
        for (int x = rcd_x + HARD_STEPS; x > rcd_x - 1; x--)
        {
            data =
                CycleSub(sample_fwd[CycleMod(x, HARD_STEPS)], sample_fwd[CycleMod(x + 1, HARD_STEPS)], ENC_RESOLUTION);

            int start_y = (x == rcd_x + HARD_STEPS) ? rcd_y : 0;
            int end_y = (x == rcd_x) ? rcd_y : data;

            for (int y = start_y; y < end_y; y++)
            {
                val = CycleMod(SOFT_DIVIDE * (x + 1) - SOFT_DIVIDE * y / data, SUBDIVIDE_STEPS);
                Stockpile_Flash_Data_Write_Data16(&stockpile_quick_cali, &val, 1);
                result_num++;
            }
        }
    }

    Stockpile_Flash_Data_End(&stockpile_quick_cali);
    printf("GenerateTable done, result_num=%u, cali_table[0]=%u\r\n", result_num, cali_table[0]);

    /* 检查生成的点数是否正确 */
    if (result_num == ENC_RESOLUTION)
        cali_is_calibrated = true;
    else
        cali_error = 4;
}

/* ==================== 公共函数 ==================== */

/**
 * @brief  初始化编码器校准模块：检查 Flash 中是否已有可用校准数据，有则交给 MT6816 驱动使用。
 * @details 判定依据是校准分区首字 cali_table[0]：只要它既不是擦除态 0xFFFF、也不是 0，就认为分区内已有
 *          有效校准表，于是置位 cali_is_calibrated 并把 cali_table 指针交给 MT6816_SetCalibrationData()，
 *          使编码器从此刻起输出查表结果（校正值）。
 *          注意这里只检查首字，不校验整表内容与点数，因此分区被写坏但仍满足上述条件时同样会被认为是已校准。
 * @warning 该校验非常宽松（只判 cali_table[0] != 0xFFFF 且 != 0）：擦写中断、写坏的表或恰好首字非 0/0xFFFF 的
 *          垃圾数据都会被当成有效校准表装载，进而在闭环中使用错误的查表结果，标定数据的完整性无任何校验。
 * @note 在 main() 的初始化阶段调用（HAL 与 TIM 启动之前、Motor_Init() 之前），只在主循环上下文执行一次。
 *       副作用：置位 cali_is_calibrated（该标志又被 Motor 模块用作闭环使能门控），并设置 MT6816 的校准表指针。
 */
void EncoderCalibrator_Init(void)
{
    /* 检查Flash中是否有有效校准数据（第一个字不是0xFFFF也不是0）*/
    uint16_t first = cali_table[0];
    if (first != 0xFFFF && first != 0)
    {
        cali_is_calibrated = true;
        MT6816_SetCalibrationData(cali_table);
    }
    //    uint16_t first = cali_table[0];
    //    printf("cali_table[0]=%u (0x%04X)\r\n", first, first);
    //    if (first != 0xFFFF && first != 0) {
    //        cali_is_calibrated = true;
    //        MT6816_SetCalibrationData(cali_table);
    //    }
    //    printf("cali_init: calibrated=%d\r\n", cali_is_calibrated);
}

/**
 * @brief  请求启动一次编码器校准流程。
 * @details 仅在"尚未校准且当前没有正在进行的校准"时生效（!cali_is_calibrated && !cali_triggered），
 *          生效时置位 cali_triggered，并把 cali_state、cali_error、go_pos、sample_cnt 复位到初值，
 *          之后由 20kHz 中断里的状态机从状态 0 开始推进。已校准或正在校准时本函数不做任何事。
 * @note 主循环上下文调用（main() 初始化阶段检测到两个按钮同时按下时调用，motor.c 的校准入口亦调用）。
 *       副作用：置位 cali_triggered 后，20kHz 中断将改走校准状态机而不再执行 Motor_Tick20kHz()，
 *       因此在它返回前应确保电机处于可安全开环驱动的状态。
 */
void EncoderCalibrator_Trigger(void)
{
    if (!cali_is_calibrated && !cali_triggered)
    {
        cali_triggered = true;
        cali_state = 0;
        cali_error = 0;
        go_pos = 0;
        sample_cnt = 0;
    }
}

/**
 * @brief  20kHz 实时采样状态机：按 cali_state 驱动开环电流矢量移动电机，并采集编码器原始角度。
 * @details 每个 20kHz 周期（50us）执行一次：先 MT6816_UpdateAngle() 刷新编码器角度，再按 cali_state 分支处理，
 *          全程以 TB67H450_SetFocCurrentVector(go_pos, 2000) 输出固定 2000mA 幅值的电流矢量，
 *          go_pos 的递增/递减同时充当开环换相角（该函数取 go_pos 的低 10 位作为电角度查表下标），
 *          因此标定期间电机不经过 FOC 闭环，完全由本状态机开环拖动。各状态行为：
 *          - 0 空闲：等待 cali_triggered；触发后先把 go_pos 归 0 输出一次电流，再把 go_pos 设为
 *            SUBDIVIDE_STEPS（一圈起点）并进入状态 1；
 *          - 1 正向准备：go_pos 每周期 +AUTO_SPEED（2 细分步），到 2 × SUBDIVIDE_STEPS 后回退到
 *            SUBDIVIDE_STEPS 并进入状态 2（用一圈行程把机械位置带到已知起点）；
 *          - 2 正向测量：go_pos 每周期 +FINE_SPEED（1 细分步）；每当 go_pos 为 SOFT_DIVIDE 的整数倍就采样一次
 *            MT6816_GetRawAngle()，攒满 SAMPLE_PER_STEP（16）次后用 CycleDataAvg() 求平均写入
 *            sample_fwd[(go_pos - SUBDIVIDE_STEPS) / SOFT_DIVIDE]（下标 0 ~ HARD_STEPS）；go_pos 超过
 *            2 × SUBDIVIDE_STEPS 后进入状态 3；
 *          - 3 反向返回：继续 +FINE_SPEED 走到 2 × SUBDIVIDE_STEPS + 20 × SOFT_DIVIDE
 *            （越过起点 20 个整步）后进入状态 4；
 *          - 4 消除间隙：每周期 -FINE_SPEED 反向回到 2 × SUBDIVIDE_STEPS，用这段反向行程吃掉传动回差后进入状态 5；
 *          - 5 反向测量：与状态 2 对称采样，结果写入 sample_rev[]，go_pos 低于 SUBDIVIDE_STEPS 后进入状态 6；
 *          - 6 计算：只输出零电流（TB67H450_SetFocCurrentVector(0, 0)）停机，此后由主循环接手做校验与建表。
 * @note 只在 TIM4 20kHz 中断（Tim4Callback20kHz()）中调用，且仅当 EncoderCalibrator_IsTriggered() 为真时调用，
 *       与 Motor_Tick20kHz() 互斥（同一中断里二选一），因此标定期间 Motor 模块不参与控制。
 *       本函数每周期只做定长运算与至多一次 SPI 读角度，不含 Flash 操作与 printf，可安全放在中断里；
 *       副作用：写 go_pos、sample_cnt、sample_raw、sample_fwd/sample_rev、cali_state 及 FOC 电流矢量。
 * @warning 本函数按固定 2000mA 电流幅值开环拖动电机，不经过 FOC 闭环，也不读取 Motor 模块的使能/限流配置；
 *          标定期间电机必然按状态机行程转动，触发前必须确保机械上允许整圈双向旋转。
 * @todo  状态 0 在触发后的第一个周期会先以当时的 go_pos（Trigger() 已清 0，即 0）输出一次电流矢量，
 *        随后立即把 go_pos 跳到 SUBDIVIDE_STEPS（相当于磁场瞬间转过半圈）并切到状态 1，这一拍的对齐
 *        是否需要（例如改为直接就位）建议结合实际机械表现再确认。
 * @todo  状态 6 只有 TB67H450_SetFocCurrentVector(0, 0) 一句、仅负责停机，且主循环一旦完成校验就会把
 *        cali_state 复位为 0，因此状态 6 实际上是"等待主循环接手"的占位状态，没有自身的判定或超时。
 */
void EncoderCalibrator_Tick20kHz(void)
{
    uint16_t raw;
    MT6816_UpdateAngle();
    switch (cali_state)
    {
    case 0: /* 空闲状态：等待触发 */
        if (cali_triggered)
        {
            TB67H450_SetFocCurrentVector(go_pos, 2000);
            go_pos = SUBDIVIDE_STEPS;
            sample_cnt = 0;
            cali_state = 1;
        }
        break;

    case 1: /* 正向准备：移动到起始位置 */
        go_pos += AUTO_SPEED;
        TB67H450_SetFocCurrentVector(go_pos, 2000);
        if (go_pos == 2 * SUBDIVIDE_STEPS)
        {
            go_pos = SUBDIVIDE_STEPS;
            cali_state = 2;
        }
        break;

    case 2: /* 正向测量：正转一圈，采集数据 */
        if ((go_pos % SOFT_DIVIDE) == 0)
        {
            raw = MT6816_GetRawAngle();
            sample_raw[sample_cnt++] = raw;
            if (sample_cnt == SAMPLE_PER_STEP)
            {
                /* 16次采样取平均，存入正向数据数组 */
                int idx = (go_pos - SUBDIVIDE_STEPS) / SOFT_DIVIDE;
                sample_fwd[idx] = CycleDataAvg(sample_raw, SAMPLE_PER_STEP, ENC_RESOLUTION);
                sample_cnt = 0;
                go_pos += FINE_SPEED;
            }
        }
        else
        {
            go_pos += FINE_SPEED;
        }
        TB67H450_SetFocCurrentVector(go_pos, 2000);
        if (go_pos > 2 * SUBDIVIDE_STEPS)
        {
            cali_state = 3;
        }
        break;

    case 3: /* 反向返回：返回起点附近 */
        go_pos += FINE_SPEED;
        TB67H450_SetFocCurrentVector(go_pos, 2000);
        if (go_pos == 2 * SUBDIVIDE_STEPS + SOFT_DIVIDE * 20)
        {
            cali_state = 4;
        }
        break;

    case 4: /* 消除间隙：反向移动消除齿轮间隙 */
        go_pos -= FINE_SPEED;
        TB67H450_SetFocCurrentVector(go_pos, 2000);
        if (go_pos == 2 * SUBDIVIDE_STEPS)
        {
            cali_state = 5;
        }
        break;

    case 5: /* 反向测量：反转一圈，采集数据 */
        if ((go_pos % SOFT_DIVIDE) == 0)
        {
            raw = MT6816_GetRawAngle();
            sample_raw[sample_cnt++] = raw;
            if (sample_cnt == SAMPLE_PER_STEP)
            {
                /* 16次采样取平均，存入反向数据数组 */
                int idx = (go_pos - SUBDIVIDE_STEPS) / SOFT_DIVIDE;
                sample_rev[idx] = CycleDataAvg(sample_raw, SAMPLE_PER_STEP, ENC_RESOLUTION);
                sample_cnt = 0;
                go_pos -= FINE_SPEED;
            }
        }
        else
        {
            go_pos -= FINE_SPEED;
        }
        TB67H450_SetFocCurrentVector(go_pos, 2000);
        if (go_pos < SUBDIVIDE_STEPS)
        {
            cali_state = 6;
        }
        break;

    case 6: /* 计算状态：停止电机输出 */
        TB67H450_SetFocCurrentVector(0, 0);
        break;
    }
}

/**
 * @brief  主循环后处理：状态机到达状态 6 后做数据校验、生成校准表并写 Flash，成功则复位系统。
 * @details 执行流程：
 *          1. 若 cali_state != 6 立即返回（本函数在主循环中被无条件高频调用，绝大多数调用都从这里返回）；
 *          2. TB67H450_Sleep() 让电机驱动休眠，停止输出；
 *          3. CheckData() 校验方向/连续性/相位跳变并写 cali_error、go_dir、rcd_x、rcd_y；
 *          4. cali_error == 0 时调用 GenerateTable()，线性插值生成 16384 点校准表并写入 Flash 校准分区；
 *          5. 无论成败都把 cali_state 清 0、cali_triggered 清 false，使模块回到空闲态；
 *          6. 若最终 cali_error == 0（校验通过且点数正确），printf 打印 Flash 中前几个字与 cali_table[0] 后
 *             调用 HAL_NVIC_SystemReset() 复位，让系统带着新校准表重新初始化。
 *          失败时只保留 cali_error 错误码（cali_state/cali_triggered 已复位），不做重试、不复位。
 * @note 只在主循环中调用（main() 的 while(1) 里每轮调用一次）；本函数含 Flash 擦写与 printf，
 *       属于阻塞式长耗时操作，不能放到中断中调用。调用前提是 20kHz 状态机已停在状态 6（已停机）。
 * @warning 失败路径只把 cali_error 置为非 0 并把 cali_state / cali_triggered 复位，既不重试也不复位系统，
 *          而 main() 的校准分支此后不会再进入（cali_triggered 已清零）；若此时编码器仍无可用校准表，
 *          Motor 侧会因 IsCalibrated() 为 false 而禁止闭环并点 NO_CALIB 灯。
 * @todo  cali_error（1/2/3/4）在当前代码里只被赋值，没有任何读取者（本文件与 Motor 模块均不消费该值），
 *        失败原因目前仅靠 printf 输出；后续应把错误码上报给上位机或 LED 指示。
 */
void EncoderCalibrator_TickMainLoop(void)
{
    if (cali_state != 6)
        return;
    /* 休眠电机 */
    TB67H450_Sleep();
    /* 检查数据有效性 */
    CheckData();
    /* 数据有效则生成校准表 */
    if (cali_error == 0)
    {
        GenerateTable();
    }
    /* 重置状态 */
    cali_state = 0;
    cali_triggered = false;

    /* 校准成功，系统复位 */
    if (cali_error == 0)
    {
        uint16_t *p = (uint16_t *)STOCKPILE_APP_CALI_ADDR;
        printf("Flash check: [0]=%u, [1]=%u, [2]=%u\r\n", p[0], p[1], p[2]);
        printf("cali_table check: [0]=%u\r\n", cali_table[0]);
        HAL_NVIC_SystemReset();
    }
}

/**
 * @brief  查询编码器是否已有可用校准数据。
 * @return true = 已校准（EncoderCalibrator_Init() 读到 Flash 有效数据，或本次标定建表成功）；
 *         false = 未校准。
 * @note 主循环与 20kHz 中断上下文都会调用：20kHz 中断用 EncoderCalibrator_IsTriggered() 选择控制分支，
 *       Motor 模块则用本函数作为闭环使能门控（未校准时电机休眠、不进入闭环），因此标定完成前闭环被禁止。
 *       只读取 cali_is_calibrated，无副作用。
 */
bool EncoderCalibrator_IsCalibrated(void) { return cali_is_calibrated; }

/**
 * @brief  用校准表把原始角度换算成校正值（查表）。
 * @details 入参 raw_angle 为原始角度（单位：编码器刻度，取值 0 ~ ENC_RESOLUTION-1 即 0 ~ 16383，
 *          越界访问由调用方负责，详见头文件声明）；
 *          直接以原始角度为下标取 cali_table[raw_angle]，得到该原始角度对应的校正值（单位：细分步位置）；
 *          未校准时不做任何换算，原样返回入参。
 * @return 校正值（cali_table[raw_angle] 的原样内容，单位：细分步，0 ~ SUBDIVIDE_STEPS-1）；
 *         cali_is_calibrated 为 false 时返回 raw_angle 本身。
 * @note 可在任意上下文调用（只读 Flash 校准表，无阻塞、无副作用），但本工程 Motor 的闭环路径实际使用
 *       MT6816_GetRectifiedAngle() 获取校正角度，本函数由外部按需调用。
 * @warning raw_angle 直接作为 cali_table 的下标使用，本函数不做范围检查：传入 >= 16384 的值会越界读取
 *          Flash 分区之外的数据（编译期无法拦截），调用方必须保证角度合法。
 * @warning 返回值单位是细分步位置（0 ~ SUBDIVIDE_STEPS-1），与入参的编码器刻度（0 ~ 16383）不是同一量纲；
 *          未校准时却原样返回入参（刻度），即两种返回值的量纲不一致，调用方需自行区分是否已校准。
 */
uint16_t EncoderCalibrator_GetRectifiedAngle(uint16_t raw_angle)
{
    if (cali_is_calibrated)
    {
        return cali_table[raw_angle];
    }
    return raw_angle;
}

/**
 * @brief  查询当前是否处于校准流程中（触发标志是否有效）。
 * @return true = 已触发且校准尚未结束（20kHz 中断此时应执行 EncoderCalibrator_Tick20kHz()）；
 *         false = 未触发（20kHz 中断正常执行 Motor_Tick20kHz()）。
 * @note 由 20kHz 中断（Tim4Callback20kHz()）每周期调用以选择控制分支；只读取 cali_triggered，无副作用。
 *       该标志在 Trigger() 中置位、在 TickMainLoop() 收尾时清零，因此从触发到建表结束的全过程都为 true。
 */
bool EncoderCalibrator_IsTriggered(void) { return cali_triggered; }
