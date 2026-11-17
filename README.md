# 省赛循迹

基于 TI MSPM0G3507 的双电机循迹小车工程，面向省级竞赛的循迹任务。工程使用
CCS + SysConfig 开发，包含八路灰度循迹、双路直流电机 PWM 驱动、编码器测速、
PI 速度闭环，以及 MPU6050 姿态和 OLED 调试驱动。

## 主要功能

- 八路循迹传感器输入，将传感器组合解码为横向偏差并计算左右轮差速。
- 两路直流电机 PWM 输出，支持方向控制和编码器反馈测速。
- 基于定时器中断的电机 PI 速度闭环，可配置目标速度、PID 参数和 PWM 限幅。
- 清理版采用闭环速度转向，通过编码器测速和 PD 偏差控制分配左右轮目标速度。
- 集成 MPU6050 DMP 姿态解算、Mahony 姿态更新和硬件 I2C OLED 调试驱动。
- 使用 SysConfig 管理引脚复用、PWM、定时器、I2C 和 GPIO 中断。

## 硬件与接口

| 模块 | 接口 |
| --- | --- |
| 主控 | MSPM0G3507，LQFP-64（PM） |
| 开发环境 | TI Code Composer Studio、MSPM0 SDK 2.11.00.07、SysConfig 1.26.2 |
| 电机 PWM | TIMG0，PA12 / PA13 |
| 电机方向 | PB17、PB19、PA16、PB24 |
| 编码器 | PA26 / PA27、PA25 / PA14 |
| 八路循迹 | PB14、PB11、PB10、PB1、PB0、PB22、PB18、PB25 |
| OLED | 硬件 I2C0，PA10 / PA11 |
| 调试器 | XDS110，SWD |

## 工程结构

| 路径 | 说明 |
| --- | --- |
| `empty.c` | 主循环、电机 PWM、编码器计数、PI 速度控制和定时器中断 |
| `Drivers/detector.c` | 八路循迹采样、偏差解码、循迹转向控制 |
| `Drivers/detector.h` | 循迹状态、方向和接口声明 |
| `Drivers/MPU6050/` | MPU6050、DMP 和 Mahony 姿态解算驱动 |
| `Drivers/OLED_Hardware_I2C/` | OLED 硬件 I2C 驱动及字库 |
| `empty.syscfg` | 外设、引脚、时钟和中断配置 |
| `targetConfigs/` | XDS110 与 MSPM0G3507 调试配置 |
| `conflict_snapshots/` | 源快照中四个带冲突文件的原始副本 |

## 构建与运行

1. 安装 TI Code Composer Studio、MSPM0 SDK 2.11.00.07 和对应 SysConfig 组件。
2. 在 CCS 中从仓库根目录导入现有工程。
3. 打开 `empty.syscfg`，确认或重新生成 SysConfig 配置。
4. 编译 `empty` 工程并通过 XDS110 下载到 MSPM0G3507。
5. 根据实车电机方向、编码器方向、轮径、减速比和赛道材质调整参数。

## 两版内容

本仓库的内容完整同步自
[`eedino/mspm0_motor`](https://github.com/eedino/mspm0_motor) 的提交
`d86d5cf69db860113d2595f9b6d3924f4bd272ef`。

### 清理版

根目录工程已清理全部冲突标记。`empty.c`、`Drivers/detector.c`、
`empty.syscfg` 和 `.vscode/settings.json` 统一采用提交说明中的新实现
`fec2fb861346cc9f8ffafd1890805f55bed6d7fa`，对应编码器四倍频、10 ms 速度闭环
和闭环循迹方案。

### 冲突保留版

源提交中四个带冲突标记的文件已原样保存到 `conflict_snapshots/`，目录层级与
原工程一致：

- `conflict_snapshots/empty.c`
- `conflict_snapshots/Drivers/detector.c`
- `conflict_snapshots/empty.syscfg`
- `conflict_snapshots/.vscode/settings.json`

该目录仅用于对照和回溯，不参与根目录 CCS 工程编译。根目录版本已经消除冲突，
但尚未在安装 TI 工具链的环境中完成编译和实车验证；仓库不包含编译产物和烧录
文件。
