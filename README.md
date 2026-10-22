# STM32F407CtrE — 双电机 PID 触摸屏调参系统

基于 STM32F407 的双直流电机 PID 速度闭环控制系统，通过 LCD 触摸屏实时调节 PID 参数、切换电机，并以示波器风格显示目标/实际转速波形。

## 功能特性

- **双电机独立 PID 闭环**：每台电机独立保存目标转速、Kp / Ki / Kd 参数，支持运行中实时调参
- **编码器测速**：霍尔编码器读取转速，配合一阶低通滤波（0.5/0.5）抑制抖动
- **抗积分饱和（Anti-Windup）**：输出饱和且误差持续同向时冻结积分，防止启动过冲、积分堆积
- **启动死区补偿**：M2 静止时施加 PWM 死区，消除低速"启不动"与启停震荡
- **LVGL 深色调参 UI**：简约高端深色主题，参数行、电机切换标签、启动/停止/阶跃/复位按钮
- **示波器风格波形**：细密网格 + 辉光曲线 + 通道标签，实时绘制目标转速与实际转速
- **FreeRTOS 多任务**：控制采样、屏幕刷新、触摸输入分任务运行

## 硬件平台

| 项目 | 说明 |
|---|---|
| 主控 | STM32F407 |
| 电机 | 双直流减速电机（M1 / M2） |
| 编码器 | 霍尔正交编码器 |
| 显示 | LCD 触摸屏（FSMC 并口） |
| 电机驱动 | PWM 调速（H 桥驱动模块） |
| 调试/烧录 | ST-Link |

## 软件架构

```
Core/
├── Src/
│   ├── main.c          # 主任务：采样、PID 运算、PWM 输出
│   ├── pid_screen.c    # LVGL 调参界面与事件回调
│   ├── wave_grid.c     # 示波器波形网格绘制
│   ├── encoder.c       # 编码器读取与转速计算
│   ├── motor.c         # 电机 PWM 驱动
│   ├── lcd.c           # LCD 底层驱动（FSMC）
│   └── lv_port_*.c     # LVGL 显示/输入移植
└── Inc/                # 对应头文件
Middlewares/            # FreeRTOS、LVGL
MDK-ARM/                # Keil MDK 工程
```

## 快速开始

### 编译（Keil MDK）

使用 Keil 打开 `MDK-ARM/LCD.uvprojx`，编译生成 `LCD.hex`。

### 一键编译（PowerShell）

```powershell
.\build.ps1            # 增量编译
.\build.ps1 -Rebuild   # 全量重新编译
.\build.ps1 -Flash     # 编译并通过 ST-Link 烧录
.\build.ps1 -Clean     # 清理编译输出
```

### 烧录

```powershell
.\flash_stlink.ps1
```

## 操作说明

1. 点击 **M1 / M2** 标签切换当前控制的电机
2. 点击参数行右侧 **+ / −** 按钮调节目标转速、Kp、Ki、Kd
3. 点击 **启动 / 停止** 控制电机运行
4. 点击 **阶跃** 在 100 / 200 RPM 之间切换目标，观察响应与超调
5. 点击 **复位** 恢复默认 PID 参数

## 技术栈

- STM32 HAL 库
- FreeRTOS
- LVGL
- Keil MDK-ARM
- C
