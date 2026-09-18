# StepMotorCtrl_42

42 步进电机闭环 FOC 驱动器固件，基于 STM32F103CB，由开源项目 **XDrive** 改造而来。

- **位置**：内部 51200 细分步/圈（200 硬步 × 256 软细分），对外接口按「圈」
- **速度**：内部 细分步/秒，对外接口「圈/秒」
- **电流**：内部 mA（1000 = 1A），对外接口「A」

## 一、硬件与关键器件

| 项目 | 说明 |
| --- | --- |
| MCU | STM32F103CB（128KB Flash / 20KB RAM），HSE × 6 PLL = 72MHz |
| 电机 | 42 步进电机，200 硬步/圈，软细分 256 → 51200 细分步/圈 |
| 位置反馈 | MT6816 磁编码器（14 位，16384 刻度/圈），SPI1 @9Mbit/s，片选 PA15 |
| 功率驱动 | TB67H450 双 H 桥：TIM2_CH3/CH4（PB10/PB11，70kHz）当 10 位 DAC 设电流幅值，PA2~PA5 设两相电流方向 |
| 通信 | USART1（PB6/PB7，115200，DMA + IDLE 判帧）；CAN1 已初始化但未实现收发 |
| 存储 | 片上 Flash 模拟 EEPROM：校准表 32KB @0x08007C00，用户配置 1KB @0x0800FC00 |
| 人机交互 | 按键 PB12/PB2（上拉，低有效）；LED PA12（状态/心跳）、PA11（错误码闪烁） |

接线参考 `接线图.JPG`，电机资料见 `42steper.pdf`。

## 二、程序骨架（三条执行线）

| 触发源 | 频率 | 入口 | 职责 |
| --- | --- | --- | --- |
| TIM4 更新中断 | **20 kHz** | `Tim4Callback20kHz()` | 编码器校准状态机 **或** 电机闭环（二选一） |
| TIM1 更新中断 | 100 Hz | `Tim1Callback100Hz()` | 按键扫描（10ms 一次）、LED、遥测计数 |
| USART1 IDLE 中断 | 按帧触发 | `UartCmd_Process()` | 上位机命令解析 |
| 主循环 | 空闲 | `main()` 的 `while(1)` | 校准建表、配置保存/复位、遥测打印 |

上电流程：读 Flash 配置（无效则写默认值）→ 映射到 `motor_config` → `Motor_Init()` → 启动 TIM1/TIM4 → 双键同按则触发编码器校准。

## 三、目录结构

```
Core/           CubeMX 生成的外设初始化与中断入口（手写代码尽量不放这里）
User/           手写业务代码，按模块划分
  Motor/        20kHz 闭环核心 + 运动规划
  MT6816/       14 位磁编码器 SPI 驱动
  DRIVER/       tb67h450 FOC 电流矢量、正弦表
  ENCODER/      编码器校准状态机与线性化建表
  EEPROM/       Flash 分区表、模拟 EEPROM
  Communication/uart_cmd.c 串口命令解析
  BUTTON/ LED/ Common/
Drivers/        ST HAL 与 CMSIS 库（厂商代码，不纳入文档）
MDK-ARM/        Keil MDK 工程（StepMotorCtrl_42.uvprojx）
doxygen/        Doxyfile、mainpage.md 与文档源 markdown
.github/        GitHub Actions：自动构建并发布 API 文档到 Pages
```

## 四、构建

1. 用 **Keil MDK**（ARM Compiler 5/6）打开 `MDK-ARM/StepMotorCtrl_42.uvprojx`
2. 目标器件 `STM32F103CB`，工程输出名 `StepMotorCtrl_42`
3. 编译产物 `MDK-ARM/StepMotorCtrl_42/StepMotorCtrl_42.hex`，用 J-Link / ST-Link 下载

> `MDK-ARM/` 下的 `*.o`、`*.crf`、`*.axf`、`*.map` 等编译中间产物不入库，见 `.gitignore`。

## 五、串口命令

| 命令 | 含义 |
| --- | --- |
| `c` | 电流相关设置 |
| `v` | 速度相关设置 |
| `p` | 位置相关设置 |
| `s` | 保存配置 |
| `z` | 复位/零位相关 |
| `l` | 读取/遥测 |

单位换算细节见 `User/Communication/uart_cmd.c`。

## 六、API 文档

```bash
# 必须在工程根目录执行：Doxyfile 中的 INPUT / OUTPUT_DIRECTORY 是相对路径
doxygen doxygen/Doxyfile
# 输出：doxygen/html/index.html、doxygen/latex/
```

CI 见 `.github/workflows/docs.yml`：推送到 `main` 且改动涉及 `User/`、`Core/`、`doxygen/Doxyfile` 时自动构建并发布到 GitHub Pages。
首次使用需在仓库 **Settings → Pages** 把 Source 设为 **GitHub Actions**。

> 生成的 HTML 站点（`doxygen/html/`、`StepMotorCtrl_42_API_Doc_html/`）不入库，由 CI 构建。

## 七、分支与版本维护约定

本仓库用**分支**区分「主线开发」与「需要单独维护的版本」，两者互不干扰：

| 分支 | 职责 |
| --- | --- |
| `main` | 主线，持续开发新功能 |
| `release/v1` | **单独维护的 v1 版本**：只接受 bugfix，不再合入新功能，按需发布 `v1.x.y` |
| `feature/*` | 短生命周期功能分支，开发完合并回 `main` |

### 日常开发（不影响 v1）

```bash
git switch main
git switch -c feature/新功能名
# ... 开发、提交 ...
git switch main
git merge --no-ff feature/新功能名
git push origin main
```

### 只维护 v1 版本

```bash
git switch release/v1
# ... 修改、提交 ...
git commit -m "fix: 修复 xxx"

git tag -a v1.0.1 -m "v1.0.1"
git push origin release/v1 --tags

# 如果这个修复 main 也需要：
git switch main
git cherry-pick <刚才的 commit sha>
git push origin main
```

### 发布新版本（v2 起的独立维护线）

```bash
git switch main
git switch -c release/v2      # 从主线切出，开始单独维护 v2
git tag -a v2.0.0 -m "v2.0.0"
git push origin release/v2 --tags
```

要点：
- 长期维护分支一旦切出就**只收 bugfix**，新功能一律进 `main`，避免两条线越走越远
- 每个发布点都打**附注 tag**（`git tag -a`），GitHub Releases 从这里生成
- 跨分支的同一个修复用 `cherry-pick` 同步，不要用 `merge` 把新功能带进维护分支

## 八、已知问题与待办

- **Flash 分区重叠风险**：`STOCKPILE_APP_FIRMWARE_SIZE`(47KB) 与 `APP_CALI`(@0x08007C00) 区间重叠 16KB；按 MDK map，正弦表 `sin_pi_m2`（0x08007676，长 0x802）跨越 0x08007C00 —— 详见 `User/EEPROM/stockpile_config.h` 文件头
- **电流环没有实测反馈**：ADC 只被初始化，`s_foc_current` 是控制器算出的指令电流，电流通道实为开环电流矢量控制
- `BoardConfig_t` 中 `defaultMode`、`enableMotorOnBoot`、`caliCurrent` 只写不读
- CAN 收发未实现
- 所有 `@todo` 汇总见 Doxygen 的 `todo` 页面

## 九、来源与许可

- 由开源项目 **XDrive** 改造而来
- `Core/` 中 CubeMX 生成的部分版权归 STMicroelectronics，采用 BSD 3-Clause License
- `Drivers/` 为 ST HAL 与 ARM CMSIS 厂商代码，遵循各自原始许可（见 `Drivers/CMSIS/LICENSE.txt`）
- 本仓库尚未添加顶层 `LICENSE` 文件，如需开源请自行补充
