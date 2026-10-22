/**
 ******************************************************************************
 * @file    pid_screen.h
 * @brief   PID 调试屏幕模块 - 320x240 TFT LCD + XPT2046 触摸
 *          适配 GenBotter Motor-1 开发板 (STM32F407 + SSD1963)
 *
 *          功能:
 *           - M1/M2 双电机独立 PID 参数
 *           - 触摸屏 +/- 按钮调 Kp/Ki/Kd/Target
 *           - 目标/实际转速实时波形 (10秒窗口)
 *           - 超调量/稳态误差显示
 *
 *          用法:
 *           1. main() 里初始化 LCD/FSMC/触摸 后调用 PID_Screen_Init()
 *           2. 创建 FreeRTOS 任务 PID_Screen_Task
 *           3. 电机定时中断里读编码器后调用 PID_Screen_SetActualRPM()
 *           4. 主循环里用 PID_Screen_GetKp/Ki/Kd/TargetRPM 跑 PID
 ******************************************************************************
 */

#ifndef __PID_SCREEN_H
#define __PID_SCREEN_H

#include "main.h"

/* 电机编号 */
#define PID_MOTOR_1     0
#define PID_MOTOR_2     1
#define PID_MOTOR_NUM   2

/* 初始化屏幕 UI (在 lcd_init() 之后调用) */
void PID_Screen_Init(void);

/* FreeRTOS 任务入口, 周期 10ms */
void PID_Screen_Task(void * argument);

/* ---- 主程序数据接口 ---- */
/* 喂入某电机实际转速 (RPM), 由编码器测速后调用 */
void PID_Screen_SetActualRPM(uint8_t motor, float rpm);

/* 喂入某电机当前 PWM 输出 (-100 ~ 100 %), 由主程序 PID 计算后调用 */
void PID_Screen_SetPWMOUT(uint8_t motor, float pct);

/* 读取当前 PID 参数 (供主程序 PID 计算用) */
float PID_Screen_GetKp(uint8_t motor);
float PID_Screen_GetKi(uint8_t motor);
float PID_Screen_GetKd(uint8_t motor);
float PID_Screen_GetTargetRPM(uint8_t motor);

/* 屏幕是否处于 RUN 状态 (当前选中的电机) */
uint8_t PID_Screen_IsRunning(void);

/* 获取指定电机的运行状态 */
uint8_t PID_Screen_IsMotorRunning(uint8_t motor);

/* 获取当前选中的电机 (0=M1, 1=M2) */
uint8_t PID_Screen_GetCurrentMotor(void);

/* 触摸读取 (XPT2046, 软件 SPI), 成功返回 1 并输出映射到 320x240 的坐标 */
uint8_t PID_Screen_TouchGetPoint(uint16_t *x, uint16_t *y);

#endif /* __PID_SCREEN_H */
