/**
 ******************************************************************************
 * @file    motion_planner.h
 * @brief   运动规划模块接口：把"硬目标"平滑成每周期逼近的"软目标"
 * @details 对外提供 5 个规划器（Current/Velocity/Position/Trajectory 四个 tracker 加一个位置插值器），
 *          每个规划器都遵循三件套调用约定：Init() 设定加速度等参数 → NewTask() 用当前实际值重置轨迹
 *          起点 → CalcSoftGoal() 每个控制周期推进一步，并把结果写进对应的 g_go_* / g_traj_* / g_interp_*
 *          全局变量，这个每周期输出就是"软目标"，供 motor.c 的控制器当输入使用。
 *          内部单位：位置为细分步（51200 细分步/圈）、速度为细分步/秒、电流为 mA。
 * @note    所有 CalcSoftGoal() 都按 20kHz（50us）周期由 motor.c 调用，全部是整数定点运算，无阻塞操作；
 *          全局配置指针 g_motion_config 必须在调用任何 tracker 之前完成赋值（Motor_Init() 负责）。
 * @warning 本模块的电流通道是"指令值"通道：CurrentTracker 与 g_go_current 给出的都是控制器算出的
 *          指令电流，本工程没有电流采样，因此不存在实测电流反馈，不构成电流闭环。
 ******************************************************************************
 */
#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <stdbool.h>
#include <stdint.h>

/* 控制频率 */
#define CONTROL_FREQUENCY 20000 /**< 控制频率，单位：Hz；积分器用它把"每秒增量"折算成每周期增量（/20000） */
#define CONTROL_PERIOD_US 50    /**< 控制周期，单位：微秒；超时计时按该步长累加 */

/* ==================== 配置 ==================== */
/**
 * @brief 运动规划配置参数（限幅、加速度与零点偏移）
 * @details 由 main.c 从 BoardConfig_t 填充，整机配置结构体 Motor_Config_t 的成员之一；
 *          各 tracker 在 Init() 时读取其中的加速度与限幅值。
 */
typedef struct
{
    int32_t encoderHomeOffset; /**< 单圈零点偏移（细分步），换算绝对坐标时由上层加上/减去 */
    int32_t caliCurrent;       /**< 编码器校准时的开环电流，单位 mA（main.c 用 calibrationCurrent 填充） */
    int32_t ratedCurrent;      /**< 电流限幅，单位 mA（默认 1000，即 1A），各控制器输出钳位到 ±该值 */
    int32_t ratedVelocity;     /**< 速度限幅，单位 细分步/秒（默认 30 圈/秒 = 30×51200） */
    int32_t ratedVelocityAcc;  /**< 速度规划加速度，单位 细分步/秒²（默认 100 圈/秒² = 100×51200） */
    int32_t ratedCurrentAcc;   /**< 电流规划变化率，单位 mA/s（main.c 固定填 2000，即 1A 约需 0.5s） */
} MotionPlanner_Config_t;
/**
 * @todo caliCurrent 只被 main.c 赋值、从未被任何模块读取：编码器校准电流实际硬编码在
 *       encoder_calibrator.c 中（TB67H450_SetFocCurrentVector(pos, 2000)），该配置项目前无效。
 */

/* 全局配置指针（需要在c文件中赋值）*/
/** @brief 全局运动规划配置指针，由 motor.c 的 Motor_Init() 指向 Motor_Config_t.motionParams */
extern MotionPlanner_Config_t *g_motion_config;

/* ==================== CurrentTracker ==================== */
/** @brief CurrentTracker 输出的软目标指令电流，单位 mA；由 CurrentTracker_CalcSoftGoal() 每周期更新 */
extern int32_t g_go_current;

/**
 * @brief  初始化电流规划器
 * @note   Motor_Init() 中调用（主循环上下文）；读取 g_motion_config->ratedCurrentAcc 作为电流变化率。
 */
void CurrentTracker_Init(void);
/**
 * @brief  设置电流变化率（梯形规划的斜率）
 * @note   只写内部静态变量 s_current_acc，不校验范围；任意上下文可调用。
 */
void CurrentTracker_SetCurrentAcc(int32_t currentAcc);
/**
 * @brief  以当前实际指令电流重置电流规划轨迹的起点
 * @note   在软目标重建时由 20kHz 中断调用（模式切换、休眠/刹车状态变化）；会清零积分余数。
 */
void CurrentTracker_NewTask(int32_t realCurrent);
/**
 * @brief  推进一步电流梯形规划，输出软目标电流
 * @note   每个 20kHz 周期调用一次，结果写入 g_go_current；过零时先按斜率减速到 0 再反向加速。
 */
void CurrentTracker_CalcSoftGoal(int32_t goalCurrent);

/* ==================== VelocityTracker ==================== */
/** @brief VelocityTracker 输出的软目标速度，单位 细分步/秒；由 VelocityTracker_CalcSoftGoal() 每周期更新 */
extern int32_t g_go_velocity;

/**
 * @brief  初始化速度规划器
 * @note   Motor_Init() 中调用；读取 g_motion_config->ratedVelocityAcc 作为加速度。
 */
void VelocityTracker_Init(void);
/**
 * @brief  设置速度规划加速度（梯形规划的斜率）
 * @note   只写内部静态变量 s_velocity_acc，不校验范围。
 */
void VelocityTracker_SetVelocityAcc(int32_t velocityAcc);
/**
 * @brief  以当前实际速度重置速度规划轨迹的起点
 * @note   在软目标重建时由 20kHz 中断调用；会清零积分余数。
 */
void VelocityTracker_NewTask(int32_t realVelocity);
/**
 * @brief  推进一步速度梯形规划，输出软目标速度
 * @note   每个 20kHz 周期调用一次，结果写入 g_go_velocity。
 */
void VelocityTracker_CalcSoftGoal(int32_t goalVelocity);

/* ==================== PositionTracker ==================== */
/** @brief PositionTracker 输出的软目标位置，单位 细分步；由 PositionTracker_CalcSoftGoal() 每周期更新 */
extern int32_t g_go_location;
/** @brief PositionTracker 输出的软目标速度，单位 细分步/秒（同时用作 DCE 的速度前馈目标） */
extern int32_t g_go_location_velocity;

/**
 * @brief  初始化位置规划器
 * @details 读取 ratedVelocityAcc 同时设置加速/减速斜率，并用 ratedVelocityAcc/1000 作为"低速锁定刹车"
 *          阈值（默认 5120 细分步/秒 = 0.1 圈/秒），速度低于该阈值且已到目标时直接把速度锁 0。
 * @note   Motor_Init() 中调用。
 */
void PositionTracker_Init(void);
/**
 * @brief  设置位置规划的加/减速斜率
 * @note   同时更新 s_velocity_up_acc、s_velocity_down_acc 与 s_quick_velocity_down_acc。
 */
void PositionTracker_SetVelocityAcc(int32_t value);
/**
 * @brief  以当前位置与当前速度重置位置规划轨迹的起点
 * @note   在软目标重建时由 20kHz 中断调用（进入位置模式或休眠/刹车状态变化）；会清零两个积分余数。
 */
void PositionTracker_NewTask(int32_t realLocation, int32_t realSpeed);
/**
 * @brief  推进一步位置 S 曲线/梯形规划，输出软目标位置与软目标速度
 * @note   每个 20kHz 周期调用一次，结果写入 g_go_location 与 g_go_location_velocity；
 *         规划内部带限速与 1/(2a) 减速距离判断，不依赖 g_motion_config 之外的模块状态。
 */
void PositionTracker_CalcSoftGoal(int32_t goalPosition);

/* ==================== PositionInterpolator ==================== */
/** @brief PositionInterpolator 输出的位置（细分步）；NewTask/CalcSoftGoal 当前未被调用 */
extern int32_t g_interp_go_position;
/** @brief PositionInterpolator 输出的滤波速度（细分步/秒）；NewTask/CalcSoftGoal 当前未被调用 */
extern int32_t g_interp_go_velocity;

/**
 * @brief  初始化位置插值器（当前为空实现）
 * @note   由 Motor_Init() 调用，但函数体内只有 "Nothing to init" 注释，不做任何初始化。
 */
void PositionInterpolator_Init(void);
/**
 * @brief  以当前位置与速度重置插值器状态
 * @todo   本函数与 PositionInterpolator_CalcSoftGoal()、以及两个 g_interp_* 输出全局变量在
 *         当前工程中都没有任何调用者（motor.c 只调用了 Init()），是为 MODE_STEP_DIR 预留的
 *         半成品，实际未生效。
 */
void PositionInterpolator_NewTask(int32_t realPosition, int32_t realVelocity);
/**
 * @brief  按新到的目标位置推进一步插值，输出位置与低通滤波后的速度
 * @note   位置直接跟随目标，速度用目标位置的差分做一阶低通（63/64 泄漏积分，时间常数 64 个控制周期）；
 *         结果写入 g_interp_go_position 与 g_interp_go_velocity。
 * @todo   当前无调用者（同 PositionInterpolator_NewTask()）。
 */
void PositionInterpolator_CalcSoftGoal(int32_t goalPosition);

/* ==================== TrajectoryTracker ==================== */
/** @brief TrajectoryTracker 输出的软目标位置，单位 细分步；由 TrajectoryTracker_CalcSoftGoal() 每周期更新 */
extern int32_t g_traj_go_position;
/** @brief TrajectoryTracker 输出的软目标速度，单位 细分步/秒；由 TrajectoryTracker_CalcSoftGoal() 每周期更新 */
extern int32_t g_traj_go_velocity;

/**
 * @brief  初始化轨迹规划器
 * @note   Motor_Init() 中调用；同时用 ratedVelocityAcc 作超时减速斜率。
 */
void TrajectoryTracker_Init(int32_t updateTimeout);
/**
 * @brief  设置超时安全停车与轨迹规划使用的减速斜率
 * @note   只写内部静态变量 s_velocity_down_acc_traj。
 */
void TrajectoryTracker_SetSlowDownVelocityAcc(int32_t value);
/**
 * @brief  以当前位置与速度重置轨迹规划起点，并复位超时计时
 * @note   在软目标重建时由 20kHz 中断调用；清零 s_update_time、超时标志与全部积分余数。
 */
void TrajectoryTracker_NewTask(int32_t realLocation, int32_t realSpeed);
/**
 * @brief  推进一步轨迹规划，输出软目标位置与速度
 * @note   每个 20kHz 周期调用一次；目标变化时按 v²-v₀²=2as 重算加速度，目标不变且超过 updateTimeout
 *         未更新时减速到 0（通信中断保护）；结果写入 g_traj_go_position 与 g_traj_go_velocity。
 */
void TrajectoryTracker_CalcSoftGoal(int32_t goalPosition, int32_t goalVelocity);

#endif
