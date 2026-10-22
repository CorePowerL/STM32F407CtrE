/**
 ******************************************************************************
 * @file    encoder.c
 * @brief   编码器驱动实现 - TIM2/TIM4 编码器模式
 ******************************************************************************
 */

#include "encoder.h"

/* ====================== 编码器 GPIO 初始化 ====================== */
static void Encoder_GPIO_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    /* PA0/PA1 - TIM2_CH1/CH2, AF1 复用功能 */
    GPIOA->MODER &= ~((3<<0) | (3<<2));   /* 先清位 */
    GPIOA->MODER |=  ((2<<0) | (2<<2));   /* AF 模式 */
    GPIOA->PUPDR &= ~((3<<0) | (3<<2));   /* 无上下拉 */
    GPIOA->AFR[0] &= ~((0xF<<0) | (0xF<<4));  /* 先清 AF */
    GPIOA->AFR[0] |=  ((1<<0) | (1<<4));      /* AF1 = TIM2 */

    /* PB6/PB7 - TIM4_CH1/CH2, AF2 复用功能
     * 加内部上拉, 防止编码器输出悬空导致读数乱跳 (M2 震荡修复) */
    GPIOB->MODER &= ~((3<<12) | (3<<14));  /* 先清位 */
    GPIOB->MODER |=  ((2<<12) | (2<<14));  /* AF 模式 */
    GPIOB->PUPDR &= ~((3<<12) | (3<<14));
    GPIOB->PUPDR |=  ((1<<12) | (1<<14));  /* 内部上拉 */
    GPIOB->AFR[0] &= ~((0xF<<24) | (0xF<<28));  /* 先清 AF */
    GPIOB->AFR[0] |=  ((2<<24) | (2<<28));      /* AF2 = TIM4 */
}

/* ====================== TIM2 编码器模式 ====================== */
static void Encoder_TIM2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 0;
    TIM2->ARR = 65535;

    /* 编码器模式 3: 同时计数 CH1 和 CH2 */
    TIM2->SMCR = 0x0003;  /* SMS=011 */

    /* CH1 和 CH2 输入, 不滤波 */
    TIM2->CCMR1 = 0x0101;  /* CC1S=01, CC2S=10 */

    /* 使能 CH1 和 CH2 捕获 */
    TIM2->CCER = 0;  /* 不反转极性 */

    TIM2->CR1 |= TIM_CR1_CEN;
}

/* ====================== TIM4 编码器模式 ====================== */
static void Encoder_TIM4_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    TIM4->PSC = 0;
    TIM4->ARR = 65535;

    /* 编码器模式 3 */
    TIM4->SMCR = 0x0003;

    /* CH1/CH2 输入, 加硬件数字滤波:
     * IC1F=0011, IC2F=0011 (3 个时钟周期滤波) 消除毛刺
     * CC1S=01, CC2S=10 → CCMR1 = 0x3231 */
    TIM4->CCMR1 = 0x3231;

    TIM4->CCER = 0;

    TIM4->CR1 |= TIM_CR1_CEN;
}

/* ====================== 对外接口 ====================== */
void Encoder_Init(void)
{
    Encoder_GPIO_Init();
    Encoder_TIM2_Init();
    Encoder_TIM4_Init();
}

int16_t Encoder_Read(uint8_t encoder)
{
    if (encoder == ENCODER_1) {
        return (int16_t)TIM2->CNT;
    } else {
        return (int16_t)TIM4->CNT;
    }
}

/* 读取并清零, 返回一个采样周期内的原始计数 (用于开环诊断) */
int16_t Encoder_GetCount(uint8_t encoder)
{
    int16_t c;
    if (encoder == ENCODER_1) {
        c = (int16_t)TIM2->CNT;
        TIM2->CNT = 0;
    } else {
        c = (int16_t)TIM4->CNT;
        TIM4->CNT = 0;
    }
    return c;
}

void Encoder_Reset(uint8_t encoder)
{
    if (encoder == ENCODER_1) {
        TIM2->CNT = 0;
    } else {
        TIM4->CNT = 0;
    }
}

float Encoder_GetRPM(uint8_t encoder, uint16_t pulse_per_rev, uint16_t dt_ms)
{
    int16_t count = Encoder_Read(encoder);
    Encoder_Reset(encoder);

    /* RPM = count * 60000 / (pulse_per_rev * dt_ms) */
    return (float)count * 60000.0f / (pulse_per_rev * dt_ms);
}
