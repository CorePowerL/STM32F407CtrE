/**
 ******************************************************************************
 * @file    encoder.h
 * @brief   编码器驱动 - TIM2/TIM4 编码器模式
 *          引脚分配:
 *            M1: A=PA0 (TIM2_CH1), B=PA1 (TIM2_CH2)
 *            M2: A=PB6 (TIM4_CH1), B=PB7 (TIM4_CH2)
 ******************************************************************************
 */

#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

/* 编码器编号 */
#define ENCODER_1   0
#define ENCODER_2   1

/* 初始化编码器 */
void Encoder_Init(void);

/* 读取编码器计数 (累积脉冲数) */
int16_t Encoder_Read(uint8_t encoder);

/* 读取并清零, 返回一个采样周期内的原始计数 (开环诊断用) */
int16_t Encoder_GetCount(uint8_t encoder);

/* 清零编码器计数 */
void Encoder_Reset(uint8_t encoder);

/* 计算转速 (RPM)
 * pulse_per_rev: 编码器线数 (比如 11 线减速后是 11*30=330)
 * dt_ms: 采样间隔 (毫秒)
 */
float Encoder_GetRPM(uint8_t encoder, uint16_t pulse_per_rev, uint16_t dt_ms);

#endif /* __ENCODER_H */
