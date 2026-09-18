/**
 ******************************************************************************
 * @file    led.c
 * @brief   双 LED 状态指示：运行心跳灯与错误码闪烁灯
 * @details LED1（编号 0，PA12）为状态灯：电机使能且运行中时输出心跳闪烁，使能但未运行时
 *          常亮，未使能时熄灭；LED2（编号 1，PA11）为错误灯：把当前电机状态编码为
 *          连续的闪烁次数（无校准 1 次、堵转 2 次、过载 3 次），无错误时熄灭。
 *          两路输出均由内部软件定时器累加时间推进相位状态机，不使用硬件 PWM。
 *          时间基准为 LED_Tick() 传入的毫秒增量，不涉及位置/速度/电流单位。
 * @note    LED_Tick() 由 TIM1 的 100Hz 中断每 10ms 调用一次，参数固定为 10；
 *          相位切换依赖传入的毫秒增量而非 HAL_GetTick()，改变调用周期会同时改变
 *          心跳节奏与闪烁节奏。LED 为高电平点亮的推挽输出。
 *
 * @todo    心跳与闪烁均由软件相位机 + 10ms 调用推进，精度受调用周期限制；若需要更稳定的
 *          闪烁节奏，可改由硬件定时器 PWM 驱动。
 ******************************************************************************
 */

#include "led.h"
#include "main.h"

/* 内部状态变量 */
static uint32_t s_timer = 0;            /**< 内部累计时间，单位 ms，每次 Tick 累加传入增量 */
static uint32_t s_timer_heartbeat = 0;  /**< 心跳相位上次切换的时间戳，单位 ms */
static uint32_t s_timer_blink = 0;      /**< 闪烁相位上次切换的时间戳，单位 ms */
static bool s_motor_enable = false;     /**< 状态灯使能：true = 常亮/心跳，false = 熄灭 */
static bool s_heartbeat_enable = false; /**< 心跳使能：true = 状态灯闪烁，false = 常亮 */
static uint8_t s_target_blink_num = 0;  /**< 错误码目标闪烁次数：0 不闪，1 无校准，2 堵转，3 过载 */
static uint8_t s_blink_num = 0;         /**< 本轮已闪烁次数，相位 2 累加、相位 3 清零 */
static uint8_t s_heartbeat_phase = 1;   /**< 心跳相位机当前相位，取值 1 ~ 4，非法值复位 1 */
static uint8_t s_blink_phase = 1;       /**< 闪烁相位机当前相位，取值 1 ~ 3，非法值复位 1 */

/**
 * @brief   设置指定 LED 的输出电平
 * @details 直接把引脚写为高/低电平（LED 高电平点亮），不做任何状态机判断。
 * @param[in] id    LED 编号，0 = LED1 (PA12，状态/心跳灯)，非 0 = LED2 (PA11，错误码闪烁灯)
 * @param[in] state true = 点亮（输出高电平），false = 熄灭（输出低电平）
 * @note    内部使用，在 TIM1 100Hz 中断上下文中被调用。
 */
static void LED_SetState(uint8_t id, bool state)
{
    if (state)
    {
        if (id == 0)
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    }
    else
    {
        if (id == 0)
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
        else
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    }
}

/**
 * @brief   初始化 LED 模块的内部状态与输出
 * @details 清零内部时间累加器与两个相位机（心跳相位 = 1、闪烁相位 = 1），清除使能标志与
 *          闪烁计数，并把两路 LED 都置为熄灭。
 * @note    在 main() 初始化阶段调用一次，需在 GPIO 初始化之后调用；此时定时器尚未启动。
 */
void LED_Init(void)
{
    s_timer = 0;
    s_timer_heartbeat = 0;
    s_timer_blink = 0;
    s_motor_enable = false;
    s_heartbeat_enable = false;
    s_target_blink_num = 0;
    s_blink_num = 0;
    s_heartbeat_phase = 1;
    s_blink_phase = 1;

    /* 初始状态：两灯都灭 */
    LED_SetState(0, false);
    LED_SetState(1, false);
}

/**
 * @brief   按固定节奏刷新两路 LED，实现状态指示与错误码闪烁
 * @details 每次调用先把 time_elapse_millis 累加到内部时间 s_timer，再按当前电机状态刷新灯效，
 *          最后推进两个相位状态机：
 *          1) 依据 state 设置状态灯使能与心跳使能、以及错误灯需要闪烁的次数；
 *          2) 状态灯：心跳使能时按相位 1 ~ 4 循环（相位 1 灭 100ms、相位 2 亮 100ms、
 *             相位 3 灭 100ms、相位 4 亮 700ms，构成周期约 1s 的心跳），
 *             心跳关闭时按 s_motor_enable 常亮或熄灭；
 *          3) 错误灯：相位 1 保持熄灭 100ms，相位 2 点亮并累加闪烁次数——未达到目标次数则回到
 *             相位 1 继续闪，达到后进入相位 3；相位 3 熄灭 1000ms 作为组间间隔，随后把闪烁计数
 *             清零、相位回到 1 并再次点亮。因此一轮内累计出现"目标次数"次点亮，但亮的时间
 *             明显长于灭的时间（点亮跨越相位 3 的 1000ms 与相位 1 的 100ms，熄灭仅相位 2 与
 *             相位 3 之间）。目标次数为 0 时，相位 3 直接置灭并保持熄灭。
 * @param[in] time_elapse_millis 距离上一次调用的时间增量，单位 ms；决定相位切换节奏
 * @param[in] state 当前电机状态（Motor_State_t），决定两灯的灯效
 * @note    在 TIM1 的 100Hz 中断回调里以 LED_Tick(10, Motor_GetState()) 每 10ms 调用一次；
 *          不同状态对应的灯效为：STATE_NO_CALIB 闪 1 次、STATE_RUNNING 心跳、STATE_FINISH 常亮、
 *          STATE_STOP 熄灭、STATE_OVERLOAD 闪 3 次、STATE_STALL 闪 2 次（闪烁均指错误灯）。
 *          错误灯闪烁计数只在本函数内维护，组间间隔内电机状态变化会在下一轮生效。
 * @todo    灯效完全依赖调用周期参数（当前固定 10ms），若改用其它周期调用，心跳与闪烁的
 *          实际时长会同步改变，后续可把时间常量改成显式的时间比较。
 */
void LED_Tick(uint32_t time_elapse_millis, Motor_State_t state)
{
    s_timer += time_elapse_millis;

    /* 根据电机状态设置LED模式 */
    switch (state)
    {
    case STATE_NO_CALIB:
        s_motor_enable = false;
        s_heartbeat_enable = false;
        s_target_blink_num = 1;
        break;
    case STATE_RUNNING:
        s_motor_enable = true;
        s_heartbeat_enable = true;
        s_target_blink_num = 0;
        break;
    case STATE_FINISH:
        s_motor_enable = true;
        s_heartbeat_enable = false;
        s_target_blink_num = 0;
        break;
    case STATE_STOP:
        s_motor_enable = false;
        s_heartbeat_enable = false;
        s_target_blink_num = 0;
        break;
    case STATE_OVERLOAD:
        s_motor_enable = true;
        s_heartbeat_enable = false;
        s_target_blink_num = 3;
        break;
    case STATE_STALL:
        s_motor_enable = false;
        s_heartbeat_enable = false;
        s_target_blink_num = 2;
        break;
    }

    /* LED0 (LED1): 心跳或常亮/常灭控制 */
    if (s_motor_enable)
    {
        if (s_heartbeat_enable)
        {
            switch (s_heartbeat_phase)
            {
            case 1:
                if (s_timer - s_timer_heartbeat > 100)
                {
                    LED_SetState(0, false);
                    s_timer_heartbeat = s_timer;
                    s_heartbeat_phase = 2;
                }
                break;
            case 2:
                if (s_timer - s_timer_heartbeat > 100)
                {
                    LED_SetState(0, true);
                    s_timer_heartbeat = s_timer;
                    s_heartbeat_phase = 3;
                }
                break;
            case 3:
                if (s_timer - s_timer_heartbeat > 100)
                {
                    LED_SetState(0, false);
                    s_timer_heartbeat = s_timer;
                    s_heartbeat_phase = 4;
                }
                break;
            case 4:
                if (s_timer - s_timer_heartbeat > 700)
                {
                    LED_SetState(0, true);
                    s_timer_heartbeat = s_timer;
                    s_heartbeat_phase = 1;
                }
                break;
            default:
                s_heartbeat_phase = 1;
                break;
            }
        }
        else
        {
            LED_SetState(0, true);
            s_heartbeat_phase = 1;
        }
    }
    else
    {
        LED_SetState(0, false);
    }

    /* LED1 (LED2): 错误码闪烁控制 */
    switch (s_blink_phase)
    {
    case 1:
        if (s_timer - s_timer_blink > 100)
        {
            LED_SetState(1, false);
            s_timer_blink = s_timer;
            s_blink_phase = 2;
        }
        break;
    case 2:
        if (s_timer - s_timer_blink > 100)
        {
            s_blink_num++;
            if (s_target_blink_num > s_blink_num)
            {
                LED_SetState(1, true);
                s_blink_phase = 1;
                s_timer_blink = s_timer;
            }
            else
            {
                LED_SetState(1, false);
                s_blink_phase = 3;
                s_timer_blink = s_timer;
            }
        }
        break;
    case 3:
        if (s_timer - s_timer_blink > 1000)
        {
            s_blink_num = 0;
            LED_SetState(1, (s_target_blink_num > 0));
            s_timer_blink = s_timer;
            s_blink_phase = 1;
        }
        break;
    default:
        s_blink_phase = 1;
        break;
    }
}
