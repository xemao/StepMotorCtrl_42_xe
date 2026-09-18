/**
 ******************************************************************************
 * @file    led.h
 * @brief   双 LED 状态指示模块对外接口
 * @details 对外只暴露初始化与周期刷新两个接口。LED1（编号 0，PA12）为状态/心跳灯，
 *          LED2（编号 1，PA11）为错误码闪烁灯，两灯的编号约定与 led.c 内部实现一致。
 *          灯效由传入的电机状态决定，具体编码见 LED_Tick() 的说明。
 * @note    LED_Tick() 必须在 100Hz 中断中以 10ms 增量调用，且应在电机状态更新之后调用，
 *          以保证本周期显示的是最新状态。
 *
 * @todo    刷新节奏依赖调用方传入的毫秒增量，未在模块内部做周期校验。
 ******************************************************************************
 */

#ifndef LED_H
#define LED_H

#include "motor.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief   初始化 LED 模块：清零内部计时与相位状态，并熄灭两路 LED
 * @note    在 main() 初始化阶段调用一次，需在 GPIO 初始化之后调用
 */
void LED_Init(void);

/**
 * @brief   刷新两路 LED，实现状态/心跳指示与错误码闪烁
 * @details 内部按传入的时间增量累加计时并推进相位状态机，故障状态会在错误灯上以
 *          固定次数的闪烁表示：STATE_NO_CALIB 闪 1 次、STATE_STALL 闪 2 次、STATE_OVERLOAD 闪 3 次；
 *          STATE_RUNNING 心跳、STATE_FINISH 常亮、STATE_STOP 熄灭。
 * @note    在 TIM1 100Hz 中断回调中每 10ms 调用一次
 */
void LED_Tick(uint32_t time_elapse_millis, Motor_State_t state);

#endif /* LED_H */
