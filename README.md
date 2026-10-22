# STM32F407 双电机 PID 触摸屏调参系统

基于 STM32F407 的双直流减速电机 PID 速度闭环控制系统，通过 320×240 触摸屏实时调节 PID 参数、切换电机，并以示波器风格显示目标转速与实际转速波形。核心目标：用双电机学习/演示 PID 调参。

## 功能特性

- **双电机独立 PID 闭环**：M1/M2 各自保存目标转速、Kp/Ki/Kd 参数，运行中可实时调参，互不干扰
- **编码器测速 + 一阶低通滤波**：0.95/0.05 强滤波抑制抖动（5ms 控制周期）
- **抗积分饱和（Anti-Windup）**：输出饱和且误差同向时冻结积分，防止启动过冲后积分堆积
- **LVGL 深色调参 UI**：左列参数 ± 按钮、右列双踪示波器波形、底部 START/STOP/STEP/RESET
- **FreeRTOS 多任务**：PID 控制（5ms）、LCD 刷新（10ms）、触摸输入分任务运行，PID 优先级高于 UI

## 硬件平台

| 项目 | 说明 |
|---|---|
| 主控 | STM32F407VET6（GenBotter Motor-1 开发板） |
| 电机 | 双直流减速电机（M1/M2），L298N 类 H 桥驱动 |
| 编码器 | 霍尔正交编码器，手测 **1594 脉冲/转**（减速后输出轴） |
| 屏幕 | 320×240 TFT 触摸屏，FSMC 16 位并口 |
| 触摸 | XPT2046，软件 SPI |
| 调试/烧录 | ST-Link（SWD） |

### 引脚分配

| 功能 | 引脚 |
|---|---|
| M1 方向 IN1/IN2 | PA2 / PA3 |
| M1 PWM (ENA) | PB0 / TIM3_CH3 |
| M1 编码器 | PA0/PA1 / TIM2 |
| M2 方向 IN1/IN2 | PA6 / PA7 |
| M2 PWM (ENB) | PC6 / TIM8_CH1 |
| M2 编码器 | PB6/PB7 / TIM4（内部上拉 + 数字滤波） |
| 触摸 CS/CLK/MISO/MOSI/PEN | PB12/PB13/PB14/PB15/PC5 |

## 软件架构

```
Core/
├── Src/
│   ├── main.c          # PID 主循环：5ms 采样、PID 运算、PWM 输出
│   ├── pid_screen.c    # LVGL 调参界面、波形绘制、触摸事件
│   ├── encoder.c       # TIM2/TIM4 编码器模式读取
│   ├── motor.c         # L298N 电机方向 + PWM 驱动
│   ├── lcd.c / fsmc.c  # LCD 底层 FSMC 驱动
│   └── lv_port_*.c     # LVGL 显示/输入移植
Middlewares/            # FreeRTOS、LVGL
MDK-ARM/                # Keil MDK 工程
```

## 关键参数

| 参数 | 值 | 说明 |
|---|---|---|
| 控制周期 | 5 ms | PID 采样 + PWM 更新 |
| 波形采样 | 50 ms | 100 点 × 50ms = 5 秒窗口 |
| PWM 频率 | 100 Hz | TIM3（M1）、TIM8（M2） |
| 编码器脉冲/转 | 1594 | 手测标定（原 1320 有误） |
| 低通滤波系数 | 0.95 / 0.05 | 新值占 5%，时间常数 ~100ms |
| 积分限幅 | ±200 | Anti-Windup 阈值 |
| PWM 输出限幅 | ±100% | |

## 快速开始

### 编译（Keil MDK）

用 Keil 打开 `MDK-ARM/LCD.uvprojx`，编译生成 `LCD.hex`。

命令行编译：
```powershell
$uv4 = "D:\app\keilsomething\UV4\UV4.exe"
$mdk = "MDK-ARM"
Start-Process $uv4 -ArgumentList '-b LCD.uvprojx -o build.log' -Wait -WorkingDirectory $mdk
```

### 烧录（OpenOCD / ST-Link）

```powershell
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg `
  -c "transport select swd" `
  -c "program MDK-ARM/LCD/LCD.hex verify reset exit"
```

## 操作说明

1. 点击 **M1 / M2** 标签切换当前控制的电机
2. 点击参数行右侧 **+ / −** 按钮调节目标转速、Kp、Ki、Kd
3. **START**：启动闭环控制；**STOP**：停止
4. **STEP**：目标在 100 / 200 RPM 之间切换，同时重置超调统计
5. **RESET**：恢复默认参数（Kp=2.0、Ki=0.50、Kd=0.20）

## 调参指南

- **先 P 后 I 再 D**：从 Kp=0.3、Ki=0、Kd=0 开始，逐步加大 Kp 直到转速有小幅振荡，再退一点
- **加 Ki 消稳态误差**：Kp 稳定后逐步加 Ki，消除静差
- **加 Kd 压超调**：过冲大时加 Kd 抑制
- 典型可用参数（M2，target=200）：Kp≈1.0~1.75、Ki≈0.2~0.35、Kd≈0.05

## 开发笔记 / 踩坑记录

- **数值显示**：必须用标准 C 库 `snprintf`，`lv_snprintf` 默认不支持 `%f`，会显示成字母 "f"
- **波形数据点**：用 `LV_PART_INDICATOR` size=0 关闭圆点 marker；线宽在 `LV_PART_ITEMS`
- **编码器方向**：M2 电机正转时 TIM4 读负数，闭环代码里用 `actual = -actual` 取反补偿
- **PWM 引脚冲突**：M2 PWM 曾用 PB1，与 LCD 背光冲突导致屏幕全黑，已改到 PC6/TIM8_CH1
- **DMA 刷屏**：历史上尝试过 DMA 块填充，但 FSMC 时序 + 栈缓冲问题未跑通，当前回退为 CPU 填充
- **控制周期**：原 100ms 周期导致明显正弦振荡，改为 5ms 后稳定性大幅提升

## 仓库

- GitHub: https://github.com/CorePowerL/STM32F407CtrE.git
