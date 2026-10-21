/**
 * @file    delay.h
 * @brief   延时函数头文件
 * @author  Dr. GAO
 * @date    2025-10-06
 * @version V1.0
 * @website https://genbotter.taobao.com
 * @email   mailto:yanzenggao@163.com
 * @note    该文件适用于GenBotter Motor-1电机开发板, 且使用DWT实现微秒级延时, STM32F407IGT6
 */

#ifndef __DELAY_H
#define __DELAY_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"

void DWT_Init(void);          /* 初始化DWT */
void delay_us(uint32_t us);   /* 微秒级延时函数 */

#ifdef __cplusplus
}
#endif

#endif /* __DELAY_H */
