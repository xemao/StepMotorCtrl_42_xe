/**
 ******************************************************************************
 * @file    motor.h
 * @brief   42 步进电机闭环 FOC 驱动的对外接口与配置数据结构声明
 * @details 本文件声明 20kHz 闭环控制入口 Motor_Tick20kHz()、目标/模式设定接口与状态读取接口，
 *          并定义控制器（PID/DCE）以及整机（运动规划 + 控制器）的配置结构体。
 *          内部单位约定：位置为细分步（1 圈 = MOTOR_SUBDIVIDE_STEPS = 51200 细分步）、速度为细分步/秒、
 *          电流为 mA；对外查询接口分别换算成 圈、圈/秒、A。
 * @note    Motor_SetConfig() 必须在 Motor_Init() 之前调用；Motor_Tick20kHz() 只在 TIM4 的
 *          20kHz（50us）中断中调用，其余接口供 100Hz 任务与串口命令使用，两类上下文通过
 *          "目标值 → 软目标值" 交换数据。
 * @warning 本工程没有实测电流反馈：ADC 只被 CubeMX 初始化，没有任何采样接入，Motor_GetCurrent()
 *          返回的 s_foc_current 是控制器算出的"指令电流"，因此电流环节实为开环电流矢量控制。
 ******************************************************************************
 */
#ifndef MOTOR_H
#define MOTOR_H

#include "motion_planner.h"
#include <stdbool.h>
#include <stdint.h>

/* 电机常量 */
#define MOTOR_HARD_STEPS      200 /**< 每圈硬步数：1.8° 两相步进电机的整步数（200 整步/圈） */
#define SOFT_DIVIDE_NUM       256 /**< 每硬步细分数，256 = 1024/4 即 90° 电角度对应的细分数 */
#define MOTOR_SUBDIVIDE_STEPS (MOTOR_HARD_STEPS * SOFT_DIVIDE_NUM) /**< 每圈细分步数 = 51200 */

/* 电机模式 */
/**
 * @brief 电机运行模式
 * @details 请求模式 s_request_mode 与运行模式 s_mode_running 共用该类型：上层用 Motor_SetMode() 写请求模式，
 *          20kHz 中断在下一个周期把它同步成运行模式，并触发一次软目标重建（NewTask）。
 * @todo MODE_PWM_POSITION/MODE_PWM_VELOCITY/MODE_PWM_CURRENT 与对应的 MODE_COMMAND_* 走完全相同的
 *       处理分支，且 main.c 与 uart_cmd.c 都没有设置这三种模式的入口；MODE_STEP_DIR 在 motor.c 中没有任何
 *       处理分支（配套的 PositionInterpolator 也未被调用）。这些模式目前属于预留/未实现。
 */
typedef enum
{
    MODE_STOP,               /**< 停止：驱动休眠，不输出电流矢量 */
    MODE_COMMAND_POSITION,   /**< 命令位置模式：PositionTracker 规划软目标，DCE 控制器输出 */
    MODE_COMMAND_VELOCITY,   /**< 命令速度模式：VelocityTracker 规划软目标，PID 速度环输出 */
    MODE_COMMAND_CURRENT,    /**< 命令电流模式：CurrentTracker 规划软目标，直接输出指令电流 */
    MODE_COMMAND_TRAJECTORY, /**< 命令轨迹模式：TrajectoryTracker 按位置+速度轨迹规划软目标 */
    MODE_PWM_POSITION,       /**< PWM 位置模式：处理流程与 MODE_COMMAND_POSITION 完全相同 */
    MODE_PWM_VELOCITY,       /**< PWM 速度模式：处理流程与 MODE_COMMAND_VELOCITY 完全相同 */
    MODE_PWM_CURRENT,        /**< PWM 电流模式：处理流程与 MODE_COMMAND_CURRENT 完全相同 */
    MODE_STEP_DIR            /**< Step/Dir 模式：仅枚举占位，motor.c 中无处理分支 */
} Motor_Mode_t;

/* 电机状态 */
/**
 * @brief 电机状态（由 20kHz 中断里的状态机更新，供 LED/上位机显示）
 * @details 优先级从高到低：未校准 > 停止 > 堵转 > 过载 > 完成/运行中。
 *          完成判据按模式区分：位置模式看软位置与软速度、速度模式看软速度、电流模式看软电流，
 *          其余模式（含轨迹/PWM/StepDir）一律判为 STATE_FINISH。
 * @note  STATE_OVERLOAD 不会锁存：一旦指令电流不再顶到 ratedCurrent，下一周期就恢复为运行/完成。
 */
typedef enum
{
    STATE_STOP,     /**< 停止：运行模式为 MODE_STOP */
    STATE_FINISH,   /**< 完成：软目标已到达目标（或该模式没有完成判据） */
    STATE_RUNNING,  /**< 运行中：软目标尚未到达目标 */
    STATE_OVERLOAD, /**< 过载：非电流模式下指令电流顶到 ratedCurrent 累计约 1s */
    STATE_STALL,    /**< 堵转：使能堵转保护时，指令电流非零/饱和而估计速度长期接近 0，累计约 1s 后置位 */
    STATE_NO_CALIB  /**< 未校准：编码器校准未完成，闭环使能被禁止（驱动休眠） */
} Motor_State_t;

/* PID 结构体 */
/**
 * @brief PID 速度环控制器状态（MODE_COMMAND_VELOCITY / MODE_PWM_VELOCITY 使用）
 * @details 定点约定（见 motor.c 的 CalcPidToOutput()）：
 *          - 增益：kp 比例增益、ki 积分增益、kd 微分增益，都是整数点位增益、无独立单位，配合 >>10 定标；
 *          - 速度误差 vError = 软目标速度 - 估计速度，限幅 ±1024×1024 细分步/秒；
 *          - 三项输出 outputKp/outputKi/outputKd 都是 <<10 定点（相当于 mA×1024），最后整体 >>10 得到 mA；
 *          - 积分项先把 ki×vError 累加到 integralRound（比 outputKi 再低 10 位），每次只把 >>10 的整数
 *            部分取到 outputKi，不足 1 的余数留在 integralRound 里下周期继续累积，避免小误差被整数
 *            除法截断成 0；积分限幅只夹紧 outputKi，integralRound 不清零（模式切换/故障时才 ClearIntegral()）。
 * @warning 这里的"输出"只是送进 TB67H450 的指令电流，没有实测电流反馈做闭环。
 */
typedef struct
{
    int32_t kp, ki, kd;                   /**< PID 增益 kp/ki/kd（整数定点，配合 >>10 定标） */
    int32_t vError, vErrorLast;           /**< 本周期速度误差与上周期速度误差，单位：细分步/秒 */
    int32_t outputKp, outputKi, outputKd; /**< Kp/Ki/Kd 三项输出，均为 <<10 定点（mA×1024） */
    int32_t integralRound;                /**< 积分累加器（比 outputKi 再低 10 位，保存未进位的余数） */
    int32_t integralRemainder;            /**< 本周期从 integralRound 取出的整数部分（>>10 的结果） */
    int32_t output;                       /**< 本周期总输出：指令电流，单位 mA（限幅 ±ratedCurrent） */
} PID_t;

/* DCE 结构体 */
/**
 * @brief DCE 双闭环控制器状态（MODE_COMMAND_POSITION / MODE_PWM_POSITION 使用）
 * @details 位置环与速度环合并成一次定点运算（见 motor.c 的 CalcDceToOutput()）：
 *          - 位置误差 pError = 软目标位置 - 估计位置（细分步），限幅 ±3200（= 1/16 圈 = 22.5° 机械角）；
 *          - 速度误差 vError 先右移 7 位再限幅 ±4000（对应约 ±512000 细分步/秒 ≈ ±10 圈/秒）；
 *          - outputKp/outputKi/outputKd 同样是 <<10 定点（mA×1024），最后整体 >>10 得到 mA；
 *          - 积分项把 ki×pError + kv×vError 累加到 integralRound（比 outputKi 再低 7 位），
 *            只把 >>7 的整数部分取到 outputKi，余数留在 integralRound 下周期继续累积；
 *          - outputKd = kd × vError，用的是本周期速度误差本身而不是误差差分，实际起速度阻尼作用。
 * @warning 这里的"输出"只是送进 TB67H450 的指令电流，没有实测电流反馈做闭环。
 */
typedef struct
{
    int32_t kp, kv, ki, kd;               /**< DCE 增益：kp 位置比例、kv 速度比例、ki/kd 积分/微分 */
    int32_t pError, vError;               /**< 位置误差（细分步）与速度误差（>>7 后，细分步/秒 ÷ 128） */
    int32_t outputKp, outputKi, outputKd; /**< Kp/Ki/Kd 三项输出，均为 <<10 定点（mA×1024） */
    int32_t integralRound;                /**< 积分累加器（比 outputKi 再低 7 位，保存未进位的余数） */
    int32_t integralRemainder;            /**< 本周期从 integralRound 取出的整数部分（>>7 的结果） */
    int32_t output;                       /**< 本周期总输出：指令电流，单位 mA（限幅 ±ratedCurrent） */
} DCE_t;

/* 控制器配置 */
/** @brief 控制器配置：一套 PID 参数 + 一套 DCE 参数 + 堵转保护开关，由 main.c 从 BoardConfig_t 填充 */
typedef struct
{
    PID_t pid;               /**< PID 速度环参数（速度模式使用） */
    DCE_t dce;               /**< DCE 双闭环参数（位置模式使用：位置环 + 速度环） */
    bool stallProtectSwitch; /**< 堵转保护开关（取自 BoardConfig_t.enableStallProtect，main.c 默认 false） */
} Controller_Config_t;

/* 电机配置 */
/** @brief 电机整机配置：运动规划参数 + 控制器参数，Motor_SetConfig() 只保存指向本结构体的指针 */
typedef struct
{
    MotionPlanner_Config_t motionParams; /**< 运动规划参数（限幅、加速度、Home 偏移），初始化时传给各 tracker */
    Controller_Config_t ctrlParams;      /**< 控制器参数（PID/DCE 增益与堵转保护开关） */
} Motor_Config_t;

/* 初始化电机系统 */
/**
 * @brief  初始化电机系统：复位故障状态并初始化 5 个运动规划 tracker
 * @details 清零首次调用标志、堵转/过载标志与计时，然后在已设置配置指针时把全局 g_motion_config 指向
 *          本模块的 motionParams，并依次调用 CurrentTracker_Init()、VelocityTracker_Init()、
 *          PositionTracker_Init()、PositionInterpolator_Init()，最后以 200ms 超时初始化
 *          TrajectoryTracker_Init(200)。
 * @note   在 main() 初始化阶段调用一次（主循环上下文）；必须先调用 Motor_SetConfig()，
 *         否则 s_config 为 NULL，整段初始化被跳过，之后任何使用配置的接口都会解引用空指针。
 */
void Motor_Init(void);

/* 设置配置指针 */
/**
 * @brief  设置电机配置指针
 * @details 保存的是指针而非副本：其成员（增益、限幅、Home 偏移）可在运行中被上位机改写，
 *          20kHz 中断每周期都读取同一份配置。
 * @note   必须在 Motor_Init() 之前调用；主循环上下文调用。
 */
void Motor_SetConfig(Motor_Config_t *config);

/* 20kHz 中断中调用 */
/**
 * @brief  20kHz 闭环控制主流程：读编码器 → 估计速度与超前角 → 按模式选控制器输出 FOC 电流矢量 → 故障检测
 * @details 每个 50us 周期依次执行：MT6816 读角度并累加多圈位置 → 一阶低通估计速度 → 用估计速度查
 *          CompensateAdvancedAngle() 得到超前角并合成 s_est_position → 休眠/刹车判断 → 按运行模式调用
 *          DCE（位置）、PID（速度）或直接输出（电流）→ 模式切换检测与软目标重建 → 目标限幅 →
 *          运动规划算软目标 → 堵转/过载检测 → 状态机刷新。
 *          注意本周期参与运算的软目标是上一周期算出的值：控制器分支在 CalcSoftGoal 之前执行。
 * @note   只在 TIM4 的 20kHz（50us）中断中调用（经 Tim4Callback20kHz()）；校准流程触发时该中断改走
 *         校准状态机，本函数不被调用。
 * @warning 电流环节没有实测反馈：s_foc_current 只是控制器算出的指令电流，经 TB67H450 开环输出。
 * @todo   MODE_STOP 分支只调用 TB67H450_Sleep()，没有把 s_foc_current 清零，因此停止后
 *         Motor_GetCurrent() 仍可能返回上一次的指令电流，过载检测也会看到该旧值。
 */
void Motor_Tick20kHz(void);

/* 控制接口 */
/**
 * @brief  设置请求模式（下一个 20kHz 周期生效）
 * @note   只写 s_request_mode；运行模式 s_mode_running 由 20kHz 中断同步，同步瞬间会重建软目标轨迹
 *         并清积分。任意上下文可调用。
 */
void Motor_SetMode(Motor_Mode_t mode);

/**
 * @brief  设置目标位置
 * @details 本函数内部会加上 encoderHomeOffset，换算到编码器绝对坐标后写入 s_goal_position。
 * @note   仅位置模式（MODE_COMMAND_POSITION/MODE_PWM_POSITION）使用；任意上下文可调用。
 */
void Motor_SetPosition(int32_t pos);

/**
 * @brief  设置目标速度
 * @note   仅当 |vel| ≤ ratedVelocity 时才写入 s_goal_velocity。
 * @warning 超出范围时静默忽略：不报错、也不改变原有目标值，调用方无法从返回值判断是否生效。
 */
void Motor_SetVelocity(int32_t vel);

/**
 * @brief  设置目标指令电流
 * @note   超出 ±ratedCurrent 时按边界钳位后写入 s_goal_current，故与本函数的入参可能不同。
 * @warning 该电流只是指令值，没有实测电流反馈校验。
 */
void Motor_SetCurrent(int32_t cur);

/**
 * @brief  设置休眠请求
 * @note   写入 s_goal_disable，在 20kHz 中断末尾同步到 s_soft_disable。
 * @todo   当前工程内没有调用者，休眠/使能暂无外部入口。
 */
void Motor_SetDisable(bool disable);

/**
 * @brief  设置刹车请求
 * @note   写入 s_goal_brake，在 20kHz 中断末尾同步到 s_soft_brake。
 * @todo   当前工程内没有调用者。
 */
void Motor_SetBrake(bool brake);

/**
 * @brief  清除堵转标志并复位堵转累计时间
 * @details 把 s_stalled_time 清零、s_is_stalled 置 false，使状态机可以从 STATE_STALL 退出，
 *          驱动器不再保持休眠。
 * @note   在 100Hz 任务（按键）或串口命令 'l' 中调用；也可由本模块在软目标重建时调用。
 */
void Motor_ClearStallFlag(void);

/* 状态读取 */
/**
 * @brief  读取电机状态
 * @note   直接返回 s_state，无副作用，可在任意上下文调用。
 */
Motor_State_t Motor_GetState(void);

/**
 * @brief  读取当前位置
 * @note   读取的是 20kHz 中断更新的 s_real_lap_position / s_real_position 快照。
 */
float Motor_GetPosition(bool isLap);

/**
 * @brief  读取估计速度
 * @note   内部值是 20kHz 对编码器位置差分做一阶低通后的结果（时间常数约 32 个控制周期）。
 */
float Motor_GetVelocity(void);

/**
 * @brief  读取当前指令电流
 * @warning 返回的是控制器算出的指令电流，不是实测相电流；本工程没有电流采样。
 */
float Motor_GetCurrent(void);

/**
 * @brief  查询编码器是否已校准
 * @note   实际由 EncoderCalibrator_IsCalibrated() 提供，未校准时 20kHz 中断让驱动休眠、不进入闭环。
 */
bool Motor_IsCalibrated(void);

/**
 * @brief  读取当前运行模式
 * @note   返回类型为 uint8_t，需要与 Motor_Mode_t 比较时由调用方隐式转换。
 */
uint8_t Motor_GetMode(void);

/* 编码器校准接口 */
/**
 * @brief  查询编码器是否已校准（本文件中重复声明的第二个原型，与上面那个内容一致）
 * @note   该原型在本文件里重复声明了两次，本次只补注释、不改动代码结构，因此两处声明都保留。
 */
bool Motor_IsCalibrated(void);

/**
 * @brief  请求触发一次编码器校准
 * @note   转调 EncoderCalibrator_Trigger()；触发后 20kHz 中断改走校准状态机，本模块暂停控制。
 */
void Motor_TriggerCalibration(void);

/**
 * @brief  一次性读取全部遥测数据
 * @note   五个指针都必须非空，本函数不做判空检查；读取的是各变量快照，5 个值可能取自不同周期。
 * @todo   当前工程内没有调用者（main.c 分别调用各单独接口），预留给上位机遥测使用。
 */
void Motor_GetTelemetry(float *pos, float *vel, float *cur, uint8_t *mode, uint8_t *state);

/**
 * @brief  把当前位置记为零点（写入 encoderHomeOffset 并保存到 EEPROM）
 * @details 取 s_real_position % 51200 作为新的单圈零点，同时更新全局 boardConfig.encoderHomeOffset，
 *          再整块写回 EEPROM 起始地址。
 * @note   在串口命令 'z' 中调用（主循环上下文）；写 Flash 是阻塞操作。
 */
void Motor_ZeroPosition(void);

#endif
