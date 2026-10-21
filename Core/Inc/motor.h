/**
 ******************************************************************************
 * @file    motor.h
 * @brief   L298N 双路电机驱动 - M1/M2 方向控制 + PWM 速度控制
 *          引脚分配:
 *            M1: IN1=PA2, IN2=PA3, ENA=PB0 (TIM3_CH3)
 *            M2: IN1=PA6, IN2=PA7, ENA=PC6 (TIM8_CH1)
 *          注意: PB1 是 LCD 背光, 不能用!
 ******************************************************************************
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

/* 电机编号 */
#define MOTOR_1     0
#define MOTOR_2     1

/* 初始化电机 GPIO 和 PWM */
void Motor_Init(void);

/* 电机控制
 * motor: MOTOR_1 / MOTOR_2
 * speed: -100 ~ 100, 正数正转, 负数反转, 0 停止
 */
void Motor_SetSpeed(uint8_t motor, int8_t speed);

#endif /* __MOTOR_H */
