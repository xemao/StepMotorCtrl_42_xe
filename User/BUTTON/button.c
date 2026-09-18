/**
 ******************************************************************************
 * @file    button.c
 * @brief   按键扫描与单击/长按事件识别
 * @details 上电由 Button_Init() 记录两路按键的初始引脚电平；之后由 TIM1 的 100Hz 中断每 10ms
 *          调用 Button_Tick() 扫描一次，用相邻两次采样的电平差检测下降沿（按下）与上升沿
 *          （释放），并在按住期间累计时长判定长按。单击与长按以一次性标志对外提供，
 *          按下状态另可直接查询。按键低电平有效（GPIO 上拉输入），无物理单位换算。
 * @note    各函数均运行在中断上下文（Button_Tick 位于 TIM1 中断回调中），需保持短小、
 *          不可阻塞；扫描周期固定 10ms，当前实现未做软件消抖。
 *
 * @todo    未做按键软件消抖，机械抖动的多次电平跳变可能被当作多次按下/释放，后续可加
 *          去抖计数或时间窗。
 * @todo    长按事件的语义与 main.c 中的动作不完全一致（详见 Button_GetLong 的说明），
 *          建议明确"长按 = 复位/停机"的约定后统一注释与代码。
 ******************************************************************************
 */

#include "button.h"
#include "main.h"

/* 按键数量与长按判定门限 */
#define BUTTON_NUM    2    /**< 可用按键个数，编号范围 1 ~ BUTTON_NUM */
#define LONG_PRESS_MS 3000 /**< 长按判定门限，单位 ms（按住累计达到即判为长按） */

/* 按键内部状态：下标即按键编号（0 号元素不使用），共 BUTTON_NUM + 1 个元素 */
static bool s_pressed[BUTTON_NUM + 1];        /**< 当前是否处于按下（按住）状态 */
static uint32_t s_press_time[BUTTON_NUM + 1]; /**< 本次按下时刻，单位 ms（HAL_GetTick() 时间戳） */
static bool s_click_flag[BUTTON_NUM + 1];     /**< 单击事件标志，置位后由读取函数清除 */
static bool s_long_flag[BUTTON_NUM + 1];      /**< 长按事件标志，置位后由读取函数清除 */
static bool s_last_state[BUTTON_NUM + 1];     /**< 上一次扫描的电平，true = 按下（低电平） */

/**
 * @brief   读取指定按键的按下状态
 * @details 直接读 GPIO 引脚电平，低电平视为按下；不读写任何状态机变量。
 * @param[in] id 按键编号，取值 1 = BUTTON_1 (PB12)，2 = BUTTON_2 (PB2)
 * @return  true = 当前按下（引脚为低电平），false = 未按下或 id 非法
 * @note    内部使用，不做消抖；供 Button_Init() 与 Button_Tick() 采样。
 */
static bool ReadPin(uint8_t id)
{
    switch (id)
    {
    case 1:
        return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET;
    case 2:
        return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET;
    default:
        return false;
    }
}

/**
 * @brief   初始化按键模块的内部状态
 * @details 以当前引脚电平作为上一次采样值，并清零按住状态、按下时刻及单击/长按标志。
 * @note    在 main() 初始化阶段（定时器中断启动之前）调用一次，需在 GPIO 初始化之后调用。
 */
void Button_Init(void)
{
    for (int i = 1; i <= BUTTON_NUM; i++)
    {
        s_last_state[i] = ReadPin(i);
        s_pressed[i] = false;
        s_press_time[i] = 0;
        s_click_flag[i] = false;
        s_long_flag[i] = false;
    }
}

/**
 * @brief   按键扫描一次，更新单击/长按状态机
 * @details 对每个按键依次执行三步：
 *          1) 检测下降沿（本次按下、上次未按下）：置按下标志、记录按下时刻，并清零单击/长按标志；
 *          2) 检测上升沿（本次未按下、上次按下）：清除按下标志，若期间未触发长按则置单击标志；
 *          3) 按住期间累计时长，达到 LONG_PRESS_MS（3000ms）时置长按标志并清除单击标志。
 *          按键低电平有效，长按在按住过程中即可判定。
 * @note    在 TIM1 的 100Hz 中断回调里每 10ms 调用一次，依赖 HAL_GetTick() 的毫秒计数；
 *          本函数只做采样与标志维护，事件由 Button_GetClick()/Button_GetLong() 消费。
 */
void Button_Tick(void)
{
    uint32_t now = HAL_GetTick();

    for (int i = 1; i <= BUTTON_NUM; i++)
    {
        bool cur = ReadPin(i);

        // 检测下降沿（按下）
        if (cur == true && s_last_state[i] == false)
        {
            s_pressed[i] = true;
            s_press_time[i] = now;
            s_click_flag[i] = false;
            s_long_flag[i] = false;
        }

        // 检测上升沿（释放）
        if (cur == false && s_last_state[i] == true)
        {
            s_pressed[i] = false;
            // 释放时判断是单击还是长按
            if (s_long_flag[i] == false)
            {
                s_click_flag[i] = true;
            }
        }

        // 按住中，检测长按
        if (s_pressed[i] == true)
        {
            if (s_long_flag[i] == false && (now - s_press_time[i]) >= LONG_PRESS_MS)
            {
                s_long_flag[i] = true;
                s_click_flag[i] = false; // 触发了长按就不算单击
            }
        }

        s_last_state[i] = cur;
    }
}

/**
 * @brief   读取并清除按键的单击事件标志
 * @details 读取即清除（一次性标志），同一次单击只会被返回一次。
 * @param[in] id 按键编号，取值 1 = BUTTON_1 (PB12)，2 = BUTTON_2 (PB2)
 * @return  true = 自上次读取后发生过单击，false = 无单击或 id 超出 1 ~ BUTTON_NUM
 * @note    由 Button_Tick() 在按键释放时置位；实际在 TIM1 100Hz 中断回调中被调用。
 */
bool Button_GetClick(uint8_t id)
{
    if (id < 1 || id > BUTTON_NUM)
        return false;
    if (s_click_flag[id])
    {
        s_click_flag[id] = false;
        return true;
    }
    return false;
}

/**
 * @brief   读取并清除按键的长按事件标志
 * @details 读取即清除（一次性标志），同一次长按只会被返回一次。
 * @param[in] id 按键编号，取值 1 = BUTTON_1 (PB12)，2 = BUTTON_2 (PB2)
 * @return  true = 自上次读取后发生过长按，false = 无长按或 id 超出 1 ~ BUTTON_NUM
 * @note    由 Button_Tick() 在按住达到 LONG_PRESS_MS 时置位；实际在 TIM1 100Hz 中断回调中被调用。
 * @warning main.c 中按键 2 的长按分支虽然打印 "Motor_Stop"，实际只把目标位置、目标速度、
 *          目标电流清零（Motor_SetPosition/SetVelocity/SetCurrent），并未切换到 MODE_STOP，
 *          因此电机仍停留在原命令模式下；注释与提示字符串并不描述真实行为。
 * @todo    建议把按键 2 长按真正切换为 MODE_STOP（或修改提示字符串），使行为与命名一致。
 */
bool Button_GetLong(uint8_t id)
{
    if (id < 1 || id > BUTTON_NUM)
        return false;
    if (s_long_flag[id])
    {
        s_long_flag[id] = false;
        return true;
    }
    return false;
}

/**
 * @brief   查询按键当前是否被按住
 * @details 直接读取 GPIO 引脚电平，不复用 Button_Tick() 的状态机，因此不受扫描周期与
 *          单击/长按标志的影响。
 * @param[in] id 按键编号，取值 1 = BUTTON_1 (PB12)，2 = BUTTON_2 (PB2)
 * @return  true = 当前按下（引脚为低电平），false = 未按下或 id 非法
 * @note    在 main() 启动流程中用于判断"两个按键同时按下"以触发编码器校准。
 */
bool Button_IsPressed(uint8_t id)
{
    if (id == 1)
        return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET;
    if (id == 2)
        return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET;

    return false;
}
