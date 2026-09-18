# StepMotorCtrl_42 项目框架地图与模块职责

> 本文档是《嵌入式项目程序框架分析方法论》在本项目上的落地,产出**框架地图 + 每个模块的职责说明**。
> 分析依据:`main.c`、`stm32f1xx_it.c` 以及 `User/` 下各模块的头文件接口。

---

## 一、项目定位

这是一个**基于 FOC 电流矢量控制的闭环步进电机驱动板**,基于开源项目 **XDrive**(作者 unlir/知不知啊,GPL v3)改造。

- 电机:42 步进电机,硬件 200 步,256 细分(共 51200 微步/圈)。
- 反馈:MT6816 磁编码器(14 位,16384 分辨率)。
- 驱动:TB67H450 芯片,支持 FOC 电流矢量 + 休眠/刹车。
- 通信:USART(上位机命令 + 遥测)、CAN(多机通信)。
- 存储:片上 Flash 模拟 EEPROM,持久化配置与编码器校准表。

---

## 二、目录结构总览

```
StepMotorCtrl_42/
├── Core/                    # CubeMX 生成的芯片级代码(不手改,重新生成会覆盖)
│   ├── Inc/  *.h            # 外设头文件、main.h、中断头文件
│   └── Src/  main.c         # 入口 + 主循环 + 定时器回调
│             stm32f1xx_it.c # 中断服务函数(ISR)
│             *_hal_msp.c    # 外设引脚/DMA 接线
│             system_*.c     # 时钟初始化
├── Drivers/                 # ST HAL 库 + CMSIS(只读,禁止格式化)
├── User/                    # ★ 业务代码(手写,核心在这里)
│   ├── Motor/               # 电机控制 + 运动规划(核心闭环)
│   ├── ENCODER/             # 编码器校准
│   ├── MT6816/              # 磁编码器驱动
│   ├── DRIVER/              # 步进驱动芯片 + 正弦表
│   ├── EEPROM/              # Flash 模拟 EEPROM + 分区表
│   ├── Communication/       # 串口命令解析
│   ├── BUTTON/ LED/         # 按键、指示灯
│   └── Common/              # 板卡配置结构体
└── MDK-ARM/                 # Keil 工程文件
```

**核心结论:分析本项目,只需要读 `Core/Src/main.c` + `Core/Src/stm32f1xx_it.c` + `User/` 全部。`Drivers/` 是库,不用读。**

---

## 三、运行框架:三条执行线

### 3.1 初始化流程(`main()`)

按 [main.c](Core/Src/main.c) 的注释序号,业务初始化分 9 步:

| 步骤 | 动作 | 说明 |
| --- | --- | --- |
| 硬件 | `HAL_Init` + `SystemClock_Config` + `MX_XXX_Init` | CubeMX 生成,初始化时钟和外设 |
| 1 | `LED_Init` `Button_Init` | 外设驱动 |
| 2 | `EncoderCalibrator_Init` | 校准模块 |
| 3 | `EEPROM_Read` 读 `boardConfig` | 读配置,无效则写默认值 |
| 4 | 配置 `motor_config` | 把 boardConfig 映射到电机参数 |
| 5 | `Motor_SetConfig` + `Motor_Init` | 电机系统就绪 |
| 6 | 启动 TIM1(100Hz)、TIM4(20kHz)中断 | 开启「心跳」 |
| 7 | 双键同按 → 触发校准 | 上电校准检测 |

### 3.2 中断表(实时心脏)

| 中断源 | 频率 | 调用 | 职责 |
| --- | --- | --- | --- |
| TIM4 更新 | **20 kHz** | `Tim4Callback20kHz` | 校准状态机 或 电机闭环 |
| TIM1 更新 | **100 Hz** | `Tim1Callback100Hz` | 按键、LED、遥测计数 |
| USART1 IDLE | 帧触发 | `OnRecvEnd` → `UartCmd_Process` | 串口命令 |
| SysTick | 1 kHz | `HAL_IncTick` | 延时基准 |

### 3.3 主循环 `while(1)`

非实时「闲时任务」:校准计算、配置保存/恢复、上位机遥测(约每 10 次 100Hz 打印一次位置/速度/电流)。

---

## 四、数据流图

```
                ┌───────────── 输入 ─────────────┐
 MT6816 磁编码器 ─SPI──► 位置/角度反馈            │
 ADC 相电流采样  ─DMA──► 电流反馈                │
 USART 上位机    ─DMA──► 命令解析(uart_cmd)      │
                └───────────────┬───────────────┘
                                ▼
        ┌────────── 核心闭环(Motor / MotionPlanner)──────────┐
        │  20kHz 中断:位置/速度/电流 三环(PID + DCE)          │
        └──────────────────────────┬────────────────────────┘
                                   ▼
                    ┌────────────── 输出 ──────────────┐
   TIM2 PWM ──────► TB67H450(FOC) ──► 电机绕组          │
   CAN ───────────► 多机通信(节点ID、配置)             │
   Flash ─────────► 配置与校准表持久化                  │
                    └──────────────────────────────────┘
```

**这是一条完整的反馈控制回路:** 编码器 + 电流 → 20kHz 闭环 → PWM → 驱动芯片 → 电机,再回到编码器。

---

## 五、模块职责详解

### 5.1 Motor/ —— 电机控制(核心)

| 文件 | 职责 |
| --- | --- |
| `motor.h` | 电机模式/状态枚举、PID/DCE 结构体、对外控制接口 |
| `motor.c` | 电机系统核心:模式切换、三环闭环控制 |
| `motion_planner.h` | 运动规划:Current/Velocity/Position/Trajectory 四个 tracker + 插值器 |
| `motion_planner.c` | 规划实现:软目标平滑、加减速 |

**关键接口([motor.h](User/Motor/motor.h)):**

```c
void Motor_Tick20kHz(void);        // 20kHz 中断中调用,闭环主入口
void Motor_SetMode(Motor_Mode_t);  // 切换模式(停止/位置/速度/电流/轨迹...)
void Motor_SetPosition(int32_t);   // 设置目标位置
void Motor_SetVelocity(int32_t);   // 设置目标速度
float Motor_GetPosition(bool isLap); // 读位置
```

**电机模式枚举** `Motor_Mode_t`:STOP / COMMAND_POSITION / COMMAND_VELOCITY / COMMAND_CURRENT / COMMAND_TRAJECTORY / PWM_POSITION / PWM_VELOCITY / PWM_CURRENT / STEP_DIR。

**电机状态枚举** `Motor_State_t`:STOP / FINISH / RUNNING / OVERLOAD / STALL / NO_CALIB。

**控制结构**:`PID_t`(位置/速度环)+ `DCE_t`(电流环,含 DCE 前馈)——见 `Controller_Config_t`。

### 5.2 MotionPlanner/ —— 运动规划

`motion_planner.h` 里定义了**四个 tracker**(软目标平滑器)+ 一个插值器:

| Tracker | 作用 |
| --- | --- |
| CurrentTracker | 电流目标平滑(限电流变化率) |
| VelocityTracker | 速度目标平滑(限加速度) |
| PositionTracker | 位置目标平滑(梯形/ S 曲线) |
| PositionInterpolator | 位置插值 |
| TrajectoryTracker | 轨迹规划(含超时、减速) |

每个 tracker 提供 `Init` / `NewTask`(设起点)/ `CalcSoftGoal`(算软目标)三件套,在 20kHz 闭环里逐周期逼近目标。

### 5.3 MT6816/ —— 磁编码器驱动

```c
#define MT6816_RESOLUTION 16384   // 14 位
uint16_t MT6816_UpdateAngle(void);        // 读 SPI,更新角度
uint16_t MT6816_GetRawAngle(void);        // 原始角度(未校准)
uint16_t MT6816_GetRectifiedAngle(void);  // 校准后角度
```

通过 SPI 读取 14 位绝对角度,配合校准表做线性化。

### 5.4 ENCODER/ —— 编码器校准

```c
void EncoderCalibrator_Tick20kHz(void);  // 20kHz 中断调用,校准状态机
void EncoderCalibrator_TickMainLoop(void);// 主循环调用,校准计算
void EncoderCalibrator_Trigger(void);    // 触发校准
uint16_t EncoderCalibrator_GetRectifiedAngle(uint16_t raw); // 查表校正
```

职责:在 20kHz 里跑校准状态机,采集编码器原始角度,在主循环里计算校准表(线性化),供 `MT6816` 使用。校准表持久化在 Flash 的 `stockpile_quick_cali` 分区。

### 5.5 DRIVER/ —— 步进驱动芯片

| 文件 | 职责 |
| --- | --- |
| `tb67h450.c/h` | TB67H450 芯片:FOC 电流矢量输出、休眠、刹车 |
| `sin_form.c/h` | 1024 点正弦表(`sin_pi_m2[1025]`) |

```c
void TB67H450_SetFocCurrentVector(uint32_t _directionInCount, int32_t _current_mA);
// _directionInCount: 电角度(0-1023); _current_mA: 电流(mA)
void TB67H450_Sleep(void);   // 休眠
void TB67H450_Brake(void);   // 刹车
```

### 5.6 EEPROM/ —— Flash 模拟 EEPROM

三层封装:

| 层 | 文件 | 职责 |
| --- | --- | --- |
| 分区表 | `stockpile_config.h` | 定义 Flash 各分区地址/大小 |
| 底层驱动 | `stockpile_f103cb.c/h` | 分区读写原语(16/32/64 位) |
| EEPROM 接口 | `eeprom.c/h` | 上层读写 `Read`/`Write`/`IsValid` |

**Flash 分区表([stockpile_config.h](User/EEPROM/stockpile_config.h)):**

| 分区 | 起始地址 | 大小 | 用途 |
| --- | --- | --- | --- |
| APP_FIRMWARE | `0x08000000` | 47K | 固件程序 |
| APP_CALI | `0x08007C00` | 32K | 编码器校准表(16K×2byte) |
| APP_DATA | `0x0800FC00` | 1K | 用户配置 `boardConfig` |

### 5.7 Communication/ —— 串口命令

```c
void UartCmd_Process(uint8_t *data, uint16_t len);
```

解析上位机发来的单字符命令,映射到电机控制:

| 命令 | 含义 |
| --- | --- |
| `c <mA>` | 电流模式 |
| `v <rps>` | 速度模式 |
| `p <圈>` | 位置模式 |
| `s` | 停止 |
| `z` | 位置清零 |
| `l` | 清除堵转标志 |

### 5.8 BUTTON/ 与 LED/

| 模块 | 接口 | 说明 |
| --- | --- | --- |
| Button | `Button_Tick`(10ms)、`GetClick`、`GetLong`、`IsPressed` | 单击/长按/按下检测 |
| LED | `LED_Init`、`LED_Tick(ms, state)` | 按电机状态闪烁指示 |

---

## 六、关键数据结构

### 板卡配置 BoardConfig_t

定义见 [configurations.h](User/Common/configurations.h)。所有可持久化参数集中于此,通过 `EEPROM_Write(0, &boardConfig, sizeof(BoardConfig_t))` 存到 Flash:

- 通信:`canNodeId`
- 运动:`currentLimit`、`velocityLimit`、`velocityAcc`
- 校准:`encoderHomeOffset`、`calibrationCurrent`
- 控制参数:`dce_kp/kv/ki/kd`、`pid_kp/ki/kd`
- 开关:`enableMotorOnBoot`、`enableStallProtect`
- 状态:`configStatus`(RESTORE / OK / COMMIT)

### 电机配置 Motor_Config_t

定义见 [motor.h](User/Motor/motor.h):

```c
typedef struct {
    MotionPlanner_Config_t motionParams;   // 运动参数
    Controller_Config_t    ctrlParams;     // PID + DCE 参数
} Motor_Config_t;
```

`main()` 把 `boardConfig` 里的值映射进 `motor_config`,再 `Motor_SetConfig(&motor_config)`。

---

## 七、一句话串起整个项目

> 上电后 `main()` 读 Flash 配置 → 初始化电机系统 → 启动 20kHz 定时器;之后每个 50µs,`TIM4` 中断里跑**编码器校准状态机**或**电机三环闭环**(位置/速度/电流),算出的 FOC 电流矢量经 `TB67H450` 输出到电机绕组;与此同时,磁编码器通过 SPI 反馈角度、上位机通过串口发命令改目标、按键和 LED 在 100Hz 中断里处理人机交互。

---

## 附:快速定位索引

| 想知道什么 | 看哪里 |
| --- | --- |
| 程序从哪开始、初始化了什么 | `Core/Src/main.c` 的 `main()` |
| 有哪些中断、各干什么 | `Core/Src/stm32f1xx_it.c` |
| 电机怎么闭环控制 | `User/Motor/motor.c` |
| 运动怎么规划、加减速 | `User/Motor/motion_planner.c` |
| 编码器怎么读、怎么校准 | `User/MT6816/` + `User/ENCODER/` |
| 参数存在哪、分区怎么划分 | `User/EEPROM/stockpile_config.h` |
| 上位机命令有哪些 | `User/Communication/uart_cmd.c` |
