# StepMotorCtrl_42 固件代码文档 {#mainpage}

42 步进电机闭环 FOC 驱动器固件（STM32F103CB），由开源项目 XDrive 改造而来。
本文档由源码注释生成，覆盖 `User/`（手写业务代码）与 `Core/`（CubeMX 生成的外设初始化与中断入口）；
`Drivers/` 为 ST HAL 与 CMSIS 库，不纳入文档。

## 一、硬件与关键器件

| 项目 | 说明 |
| --- | --- |
| MCU | STM32F103CB，中容量（128KB Flash / 20KB RAM），HSE×6 PLL = 72MHz |
| 电机 | 42 步进电机：200 硬步/圈，软细分 256 → 51200 细分步/圈 |
| 位置反馈 | MT6816 磁编码器（14 位，16384 刻度/圈），SPI1 @9Mbit/s，片选 PA15 |
| 功率驱动 | TB67H450 双 H 桥：TIM2_CH3/CH4（PB10/PB11，70kHz）当 10 位 DAC 设电流幅值，PA2~PA5 设两相电流方向 |
| 通信 | USART1（PB6/PB7，115200，DMA + IDLE 判帧）；CAN1 已初始化但未实现收发 |
| 存储 | 片上 Flash 模拟 EEPROM：校准表 32KB @0x08007C00，用户配置 1KB @0x0800FC00 |
| 人机交互 | 按键 PB12/PB2（上拉，低有效）；LED PA12（状态/心跳）、PA11（错误码闪烁） |

## 二、三条执行线（程序骨架）

| 触发源 | 频率 | 入口 | 职责 |
| --- | --- | --- | --- |
| TIM4 更新中断 | **20 kHz** | `Tim4Callback20kHz()` | 编码器校准状态机 **或** 电机闭环（二选一） |
| TIM1 更新中断 | 100 Hz | `Tim1Callback100Hz()` | 按键扫描（10ms 一次）、LED、遥测计数 |
| USART1 IDLE 中断 | 按帧触发 | `UartCmd_Process()` | 上位机命令解析 |
| 主循环 | 空闲 | `main()` 的 `while(1)` | 校准建表、配置保存/复位、遥测打印 |

上电流程：读 Flash 配置（无效则写默认值）→ 映射到 `motor_config` → `Motor_Init()` → 启动 TIM1/TIM4 →
双键同按则触发编码器校准。

## 三、模块地图

| 模块 | 关键职责 |
| --- | --- |
| `Core/Src/main.c` | 初始化 9 步、主循环、TIM1/TIM4 回调分派 |
| `User/Motor/motor.c` | 20kHz 闭环核心：读编码器 → 一阶低通估计速度 → 超前角补偿 → 按模式选 PID/DCE/直接电流 → FOC 电流矢量输出 → 堵转/过载检测 → 状态机 |
| `User/Motor/motion_planner.c` | 运动规划：Current/Velocity/Position/Trajectory 四个 tracker + 位置插值器，把"硬目标"逐周期平滑成"软目标" |
| `User/ENCODER/encoder_calibrator.c` | 编码器校准：20kHz 状态机采样（正转、反转、消隙）、主循环校验并生成 16384 点线性化表写入 Flash |
| `User/MT6816/` | 14 位磁编码器 SPI 驱动、偶校验与重试、查表线性化 |
| `User/DRIVER/tb67h450.c` | FOC 电流矢量：查 1024 点正弦表 → 电流转 DAC 值 → 写 PWM 比较寄存器 + 设方向脚；含休眠/刹车 |
| `User/EEPROM/` | Flash 分区表与读写原语、模拟 EEPROM 上层接口 |
| `User/Communication/uart_cmd.c` | 串口命令 `c/v/p/s/z/l` 与单位换算 |
| `User/BUTTON/`、`User/LED/`、`User/Common/` | 按键状态机、指示灯语义、板卡配置结构体 |

## 四、数据单位约定（读代码时务必注意）

- **位置**：内部单位是"细分步"（1 圈 = 51200 细分步 = 200 硬步 × 256 软细分），对外接口按"圈"
- **速度**：内部 细分步/秒，对外接口"圈/秒"
- **电流**：内部 mA（1000 = 1A），对外接口 A
- **角度**：编码器原始角度 0~16383（14 位）；校准表中的"校正位置"是 0~51199（细分步），两者量纲不同
- **时间**：20kHz 控制周期 = 50us；100Hz 交互任务周期 = 10ms
- **电角度**：正弦表 1024 点/电周期，256 细分步 = 90° 电角度

## 五、已知问题与待办

- 所有 `@todo` 汇总在 @ref todo 页面；`@warning` 不单独成页，显示在对应文件/函数的说明段落里
- **Flash 分区重叠风险**：`STOCKPILE_APP_FIRMWARE_SIZE`(47KB) 与 `APP_CALI`(@0x08007C00) 区间重叠 16KB，
  按 MDK map，正弦表 `sin_pi_m2`（0x08007676，长 0x802）跨越 0x08007C00 —— 详见 `User/EEPROM/stockpile_config.h` 文件头
- **电流环没有实测反馈**：ADC 只被初始化，`s_foc_current` 是控制器算出的指令电流，电流通道实为开环电流矢量控制
- `BoardConfig_t` 中 `defaultMode`、`enableMotorOnBoot`、`caliCurrent` 只写不读；CAN 收发未实现

## 六、如何重新生成文档

```bash
# 必须在工程根目录执行：Doxyfile 中的 INPUT / OUTPUT_DIRECTORY 为相对路径
doxygen doxygen/Doxyfile
# 输出：doxygen/html/index.html（本页）、doxygen/latex/（如需 PDF 再跑 latex/make.bat）
```

CI 配置见 `.github/workflows/docs.yml`：推送到 master/main 时自动构建并发布到 GitHub Pages。
文档的生成与发布细节（CI 固定 Doxygen 版本、PDF 与离线包的取舍、其他单文件形态）见 @ref doc_publish "文档生成与发布说明"。

## 七、许可

本项目是 [unlir/XDrive](https://github.com/unlir/XDrive) 的衍生作品，整体以 **GNU General Public License v3.0** 授权，
全文见工程根目录的 `LICENSE`。分发（含以固件形式提供二进制）时须一并提供对应源码，衍生作品亦须以 GPL-3.0 授权。
`Core/` 中 CubeMX 生成部分为 ST BSD 3-Clause，`Drivers/` 为 ST HAL 与 ARM CMSIS（Apache-2.0 等），均与 GPL-3.0 兼容。
