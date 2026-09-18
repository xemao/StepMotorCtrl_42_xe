/**
 ******************************************************************************
 * @file    encoder_calibrator.h
 * @brief   磁编码器标定模块的对外接口声明（触发标定、20kHz 采样、主循环建表、查表取校正角度）。
 * @details 本模块为 MT6816 磁编码器（14 位，16384 刻度/圈）标定一张以原始角度为下标的线性化查表
 *          （16384 个 uint16_t 条目），并写入 Flash 校准分区（STOCKPILE_APP_CALI_ADDR），
 *          用于补偿磁体安装偏心等误差。表项内容是"校正位置"，单位与位置量相同（细分步，0 ~ 51199）。
 *          典型调用次序：EncoderCalibrator_Init() → EncoderCalibrator_Trigger()（请求标定）→
 *          20kHz 中断中在 IsTriggered() 为真时调用 Tick20kHz() 完成一圈正/反向采样 →
 *          主循环中 TickMainLoop() 负责校验、建表、写 Flash 并复位系统。
 *          标定成功后 MT6816 查表输出校正位置（细分步），Motor 可获得线性度更好的位置反馈。
 * @note    调用上下文与实时性：Tick20kHz() 必须只在 TIM4 20kHz（50us）中断中调用，且与 Motor_Tick20kHz()
 *          互斥（中断里按 IsTriggered() 二选一）；TickMainLoop() 只在主循环中调用，内部含 Flash 擦写（阻塞）；
 *          Trigger()/Init() 在主循环上下文调用；IsCalibrated()/IsTriggered() 可被中断与主循环共同调用。
 *          校准完成前闭环会被禁止：Motor 模块以 IsCalibrated() 作为门控，未校准时电机保持休眠、不进入闭环。
 * @warning 校准表与 Flash 分区容量强耦合：分区 STOCKPILE_APP_CALI_ADDR 共 32KB，恰好等于
 *          16384 × sizeof(uint16_t)，改动编码器分辨率或分区大小都会导致越界擦写，必须同步修改。
 * @warning 校准失败（cali_error != 0）时不会产生可用校准表，IsCalibrated() 保持 false，Motor 侧继续禁止闭环，
 *          此时不应忽略失败状态继续运行闭环控制。
 ******************************************************************************
 */

#ifndef ENCODER_CALIBRATOR_H
#define ENCODER_CALIBRATOR_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief  初始化校准器：检查 Flash 中是否已有校准数据，有则装载给 MT6816。
 * @note 在 main() 初始化阶段（定时器与 Motor 初始化之前）调用一次，主循环上下文。
 */
void EncoderCalibrator_Init(void);

/**
 * @brief  20kHz 实时采样状态机：开环驱动电机并采集编码器原始角度。
 * @note 仅在 TIM4 20kHz 中断中调用，且仅当 EncoderCalibrator_IsTriggered() 为真时调用（每 50us 一次）。
 * @warning 该函数会以固定电流幅值开环整圈双向拖动电机，且与 Motor_Tick20kHz() 互斥；调用它期间不能同时
 *          运行闭环控制，触发前必须确认机械允许整圈旋转。实现细节见 .c 中同名函数的说明。
 */
void EncoderCalibrator_Tick20kHz(void);

/**
 * @brief  主循环后处理：数据校验、线性插值建表、写入 Flash 校准分区，成功后复位系统。
 * @note 在 main() 的 while(1) 中调用；内部含 Flash 擦写与 printf，属阻塞操作，不可在中断中调用。
 * @warning 校准失败时本函数不清除失败信息、不回滚 Flash，也不会阻止后续使用；失败后编码器仍无可用校准表，
 *          Motor 侧会因 IsCalibrated() 为 false 继续禁止闭环。
 * @todo  失败路径缺少上报通道：标定错误码只在本模块内部记录（且当前无读取者），调用方无法区分
 *        "未标定"与"标定失败"，建议后续增加错误查询接口或状态上报。
 */
void EncoderCalibrator_TickMainLoop(void);

/**
 * @brief  触发一次编码器校准（外部调用）。
 * @note 仅在未校准且当前未在标定时生效；主循环上下文调用（按钮事件或 Motor 模块的标定入口）。
 * @warning 触发后电机将由校准状态机开环拖动整圈，重复调用在已触发时会被忽略，不会重启流程。
 */
void EncoderCalibrator_Trigger(void);

/**
 * @brief  检查是否已校准。
 * @note 兼作 Motor 侧的闭环使能门控；中断与主循环均可调用，只读标志、无副作用。
 * @warning 该校验只依赖 Flash 校准分区首字是否为 0xFFFF/0，不做整表校验，因此"已校准"不代表表内容一定正确。
 */
bool EncoderCalibrator_IsCalibrated(void);

/**
 * @brief  获取校准后的角度（查表）。
 * @note 只读 Flash 校准表，无阻塞，可在任意上下文调用。
 * @warning raw_angle 被直接用作校准表下标，本函数不做范围检查，传入越界值会读到分区之外的数据。
 * @warning 返回值与入参量纲不同（细分步 vs 刻度），仅"未校准"分支会原样返回入参，使用方需注意区分。
 */
uint16_t EncoderCalibrator_GetRectifiedAngle(uint16_t raw_angle);

/**
 * @brief  查询当前是否处于校准流程中。
 * @note 由 20kHz 中断每周期调用以在 Tick20kHz() 与 Motor_Tick20kHz() 之间选择。
 */
bool EncoderCalibrator_IsTriggered(void);

#endif
