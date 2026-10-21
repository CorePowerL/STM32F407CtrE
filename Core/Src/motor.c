/**
 ******************************************************************************
 * @file    motor.c
 * @brief   L298N 双路电机驱动实现
 ******************************************************************************
 */

#include "motor.h"
#include "main.h"

/* PWM 时基: 84MHz / (84 * 10000) = 100kHz */
#define TIM_PSC   84
#define TIM_ARR   10000

/* 方向控制引脚定义 */
#define M1_IN1_PORT   GPIOA
#define M1_IN1_PIN    GPIO_PIN_2
#define M1_IN2_PORT   GPIOA
#define M1_IN2_PIN    GPIO_PIN_3

#define M2_IN1_PORT   GPIOA
#define M2_IN1_PIN    GPIO_PIN_6
#define M2_IN2_PORT   GPIOA
#define M2_IN2_PIN    GPIO_PIN_7

/* ====================== GPIO 初始化 ====================== */
static void Motor_GPIO_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    /* PA2, PA3, PA6, PA7 - 推挽输出 */
    GPIOA->MODER &= ~((3<<4) | (3<<6) | (3<<12) | (3<<14));
    GPIOA->MODER |=  ((1<<4) | (1<<6) | (1<<12) | (1<<14));
    GPIOA->OSPEEDR |= ((3<<4) | (3<<6) | (3<<12) | (3<<14));

    /* PB0 - AF2 = TIM3_CH3 (M1 ENA) */
    GPIOB->MODER &= ~(3<<0);
    GPIOB->MODER |=  (2<<0);
    GPIOB->OSPEEDR |= (3<<0);
    GPIOB->AFR[0] &= ~(0xF<<0);
    GPIOB->AFR[0] |=  (2<<0);

    /* PC6 - AF3 = TIM8_CH1 (M2 ENA) */
    GPIOC->MODER &= ~(3<<12);
    GPIOC->MODER |=  (2<<12);
    GPIOC->OSPEEDR |= (3<<12);
    GPIOC->AFR[0] &= ~(0xF<<24);
    GPIOC->AFR[0] |=  (3<<24);
}

/* ====================== PWM 初始化 ====================== */
static void Motor_PWM_Init(void)
{
    /* TIM3: M1 ENA */
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = TIM_PSC - 1;
    TIM3->ARR = TIM_ARR - 1;
    TIM3->CR1 = TIM_CR1_ARPE;
    TIM3->CCMR2 = (6<<4) | (1<<3);  /* CH3 PWM mode1 */
    TIM3->CCER = TIM_CCER_CC3E;
    TIM3->CCR3 = 0;
    TIM3->CR1 |= TIM_CR1_CEN;

    /* TIM8: M2 ENA */
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    TIM8->PSC = TIM_PSC - 1;
    TIM8->ARR = TIM_ARR - 1;
    TIM8->CR1 = TIM_CR1_ARPE;
    TIM8->CCMR1 = (6<<4) | (1<<3);  /* CH1 PWM mode1 */
    TIM8->CCER = TIM_CCER_CC1E;
    TIM8->CCR1 = 0;
    TIM8->CR1 |= TIM_CR1_CEN;
}

/* ====================== 对外接口 ====================== */
void Motor_Init(void)
{
    Motor_GPIO_Init();
    Motor_PWM_Init();
}

void Motor_SetSpeed(uint8_t motor, int8_t speed)
{
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;

    uint16_t pwm = (uint16_t)(abs(speed) * TIM_ARR / 100);

    if (motor == MOTOR_1) {
        /* M1: PA2=IN1, PA3=IN2, PB0=ENA */
        if (speed > 0) {
            GPIOA->BSRR = M1_IN1_PIN;        /* IN1=1 */
            GPIOA->BSRR = M1_IN2_PIN << 16;  /* IN2=0 */
        } else if (speed < 0) {
            GPIOA->BSRR = M1_IN1_PIN << 16;  /* IN1=0 */
            GPIOA->BSRR = M1_IN2_PIN;        /* IN2=1 */
        } else {
            GPIOA->BSRR = M1_IN1_PIN << 16;  /* IN1=0 */
            GPIOA->BSRR = M1_IN2_PIN << 16;  /* IN2=0 */
        }
        TIM3->CCR3 = pwm;
    } else {
        /* M2: PA6=IN1, PA7=IN2, PC6=ENB (TIM8_CH1) */
        if (speed > 0) {
            GPIOA->BSRR = M2_IN1_PIN;        /* IN1=1 */
            GPIOA->BSRR = M2_IN2_PIN << 16;  /* IN2=0 */
        } else if (speed < 0) {
            GPIOA->BSRR = M2_IN1_PIN << 16;  /* IN1=0 */
            GPIOA->BSRR = M2_IN2_PIN;        /* IN2=1 */
        } else {
            GPIOA->BSRR = M2_IN1_PIN << 16;  /* IN1=0 */
            GPIOA->BSRR = M2_IN2_PIN << 16;  /* IN2=0 */
        }
        TIM8->CCR1 = pwm;
    }
}
