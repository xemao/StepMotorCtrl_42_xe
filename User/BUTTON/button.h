/**
 ******************************************************************************
 * @file    button.h
 * @brief   按键模块对外接口
 * @details 提供初始化、周期扫描以及单击/长按/按下状态查询四类接口。按键低电平有效
 *          （GPIO 上拉输入）：BUTTON_1 = PB12、BUTTON_2 = PB2（引脚宏定义见 main.h）。
 *          单击与长按为一次性事件标志，读取后自动清除，便于在中断或主循环中消费。
 *          本模块只产生按键事件，不涉及任何物理单位换算。
 * @note    Button_Tick() 必须由 100Hz 中断以 10ms 周期调用，单击/长按才具备正确的时长
 *          判定；Button_IsPressed() 直接读引脚，不依赖扫描状态机。
 *
 * @todo    模块未做按键消抖，抖动电平可能被识别为多次按下/释放。
 ******************************************************************************
 */

#ifndef BUTTON_H
#define BUTTON_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief   初始化按键模块内部状态；需在 GPIO 初始化之后调用一次
 */
void Button_Init(void);
/**
 * @brief   按键扫描；由 100Hz 中断每 10ms 调用一次
 */
void Button_Tick(void);
/**
 * @brief   读取并清除单击事件标志（读后自动清除，一次性标志）
 */
bool Button_GetClick(uint8_t id);
/**
 * @brief   读取并清除长按事件标志（读后自动清除，一次性标志）
 */
bool Button_GetLong(uint8_t id);
/**
 * @brief   查询按键当前是否按下（直接读引脚，不复用扫描状态机）
 */
bool Button_IsPressed(uint8_t id);

#endif
