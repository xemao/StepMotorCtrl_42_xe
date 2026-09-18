/**
 ******************************************************************************
 * @file    configurations.h
 * @brief   板卡可持久化参数的类型定义（配置状态枚举与总配置结构体）
 * @details 定义 configStatus_t（配置状态）与 BoardConfig_t（全部可持久化参数），并声明全局
 *          实例 boardConfig。上电时 main() 从 EEPROM 读出整个结构体：状态不等于 CONFIG_OK
 *          就写入一组默认值并保存；主循环再根据 configStatus 决定"写 Flash"还是"写 Flash
 *          后复位"。main() 随后把这些参数拷入 motor_config（额定电流/额定速度/加速度、
 *          校准电流、电流环与位置环增益、堵转保护开关）再交给电机模块使用。
 * @note    单位约定：位置 = 细分步（1/51200 圈 = 200 硬步 × 256 软细分），速度 = 细分步/秒，
 *          电流 = mA。以下成员当前只被写入、没有代码读取使用（写了 Flash 但不生效）：
 *          canNodeId、defaultMode、enableMotorOnBoot；其中 CAN 通信逻辑尚未实现。
 *
 * @todo    CAN 通信逻辑未实现：can.c 只有初始化，canNodeId 仅被存储（上电默认填 1），
 *          无任何代码读取使用。
 * @todo    defaultMode、enableMotorOnBoot 从未被任何代码读取，只会随结构体写入 Flash，
 *          属于"写了但不生效"的配置项。
 ******************************************************************************
 */

#ifndef CONFIGURATIONS_H
#define CONFIGURATIONS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

    /* ==================== 配置状态枚举 ==================== */
    /**
     * @brief 配置状态：指示 main() 是否需要保存或恢复配置
     * @note  该字段随整个结构体一起保存在 Flash 中；上电读取后只要不是 CONFIG_OK，
     *        main() 就会填入默认配置并置回 CONFIG_OK 后写回。
     */
    typedef enum
    {
        CONFIG_RESTORE = 0, /**< 请求保存配置并复位：main() 主循环写 Flash 后调用 HAL_NVIC_SystemReset() */
        CONFIG_OK,          /**< 配置有效、与 Flash 内容一致；main() 不做任何处理（上电时也用于标记已使用默认值） */
        CONFIG_COMMIT       /**< 请求把当前配置提交到 Flash：main() 主循环擦除配置分区并写入，不复位 */
    } configStatus_t;

    /* ==================== 板卡配置结构体 ==================== */
    /**
     * @brief 板卡全部可持久化参数
     * @note  整个结构体以 EEPROM_Write(0, &boardConfig, sizeof(BoardConfig_t)) 原样写入 Flash，
     *        上电由 EEPROM_Read() 原样读回，因此字段类型与顺序即 Flash 中的存储布局，
     *        增删成员会改变已有设备的配置解析结果。位置类参数内部单位为细分步
     *        （1/51200 圈 = 200 硬步 × 256 软细分），速度类为细分步/秒，电流类为 mA；
     *        电流环（dce_*）与位置环（pid_*）增益为整数定标值，在电机模块中按定点数参与运算：
     *        电流环积分累加器与位置环积分/输出分别做 >>7、>>10 的移位定标。
     *        其中 canNodeId、defaultMode、enableMotorOnBoot 目前没有任何代码读取使用，
     *        虽会被写入 Flash 但不会生效；CAN 通信逻辑也尚未实现。
     *
     * @todo   CAN 通信逻辑未实现：can.c 只有初始化，canNodeId 仅被存储（上电默认填 1），
     *         无任何代码读取使用。
     * @todo   defaultMode、enableMotorOnBoot 从未被任何代码读取，只会随结构体写入 Flash，
     *         属于"写了但不生效"的配置项。
     */
    typedef struct
    {
        configStatus_t configStatus; /**< 配置状态：由 main() 读写，决定写 Flash 或写 Flash 后复位 */
        uint32_t canNodeId;          /**< CAN 节点编号；仅被存储，CAN 通信逻辑未实现 */
        int32_t encoderHomeOffset;   /**< 编码器电角度零点偏移，单位：细分步；把编码器角度对齐到机械零位 */
        uint32_t defaultMode;  /**< 上电默认工作模式（如 MODE_COMMAND_POSITION）；写了 Flash 但当前无代码读取，不生效 */
        int32_t currentLimit;  /**< 电流环额定电流上限，单位：mA；同时作为 Motor_SetCurrent() 的限幅值 */
        int32_t velocityLimit; /**< 速度上限（额定速度），单位：细分步/秒；Motor_SetVelocity() 按该值限幅 */
        int32_t velocityAcc;   /**< 速度给定加速度，单位：细分步/秒；运动规划按每控制周期增量使用 */
        int32_t calibrationCurrent; /**< 编码器校准阶段的电流，单位：mA */
        int32_t dce_kp;             /**< 电流环比例增益（整数定标） */
        int32_t dce_kv;             /**< 电流环速度反馈增益（整数定标） */
        int32_t dce_ki;             /**< 电流环积分增益（整数定标） */
        int32_t dce_kd;             /**< 电流环微分增益（整数定标） */
        int32_t pid_kp;             /**< 位置环比例增益（整数定标） */
        int32_t pid_ki;             /**< 位置环积分增益（整数定标） */
        int32_t pid_kd;             /**< 位置环微分增益（整数定标） */
        bool enableMotorOnBoot;     /**< 是否上电自动使能电机；写了 Flash 但当前无代码读取，不生效 */
        bool enableStallProtect;    /**< 堵转保护开关，拷贝到 motor_config.ctrlParams.stallProtectSwitch */
    } BoardConfig_t;

    /* ==================== 全局变量声明 ==================== */
    /**
     * @brief 全局板卡配置实例
     * @note  定义在 main.c；上电从 Flash 读出，运行期间由 main() 及位置清零命令更新，
     *        置 configStatus 为 CONFIG_COMMIT 或 CONFIG_RESTORE 即可请求保存到 Flash。
     */
    extern BoardConfig_t boardConfig;

#ifdef __cplusplus
}
#endif

#endif /* CONFIGURATIONS_H */
