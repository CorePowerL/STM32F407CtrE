/**
 ******************************************************************************
 * @file    main.c
 * @brief   PID 调参屏幕主程序 - 真实电机驱动版
 *          L298N 电机 + 编码器测速
 ******************************************************************************
 */

#include "main.h"
#include "gpio.h"
#include "fsmc.h"
#include "lcd.h"
#include "delay.h"
#include "pid_screen.h"
#include "motor.h"
#include "encoder.h"
#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"

/* ====================== PID 变量 ====================== */
typedef struct {
    float integral;
    float last_err;
    float rpm;
    float rpm_filtered;   /* 滤波后的转速 */
} PID_Data_t;

static PID_Data_t pid[2] = {
    { 0, 0, 0, 0 },
    { 0, 0, 0, 0 }
};

/* 编码器线数 (减速后每转脉冲数)
 * 手转输出轴一圈实测约 1594 脉冲 (原 1320 标定错误, 2026-10-22 修正) */
#define PULSE_PER_REV   1594.0f
/* 采样周期 ms (PID 控制周期)
 * 原 100ms 太慢导致正弦振荡, 改成 5ms 快速控制 */
#define SAMPLE_DT_MS    5

/* 诊断模式:
 *   0 = 正常闭环 PID
 *   1 = 开环 35% 正转, 看 100ms 脉冲数 (已验证硬件正常)
 *   2 = 手动计数模式: 电机不转, 用手转 M2 输出轴, 屏幕显示累计脉冲数
 *       → 用来标定 PULSE_PER_REV (转一整圈看数字涨了多少) */
#define M2_OPENLOOP_TEST   0

/* 手动计数模式下累计的脉冲数 (防 16 位溢出) */
static int32_t m2_total_pulses = 0;

/* ====================== PID 任务 ====================== */
static void PID_Task(void *pv)
{
    const float dt = SAMPLE_DT_MS / 1000.0f;
    int i;
    (void)pv;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_DT_MS));

        /* 两个电机独立判断运行状态 */
        for (i = 0; i < 2; i++) {
            if (!PID_Screen_IsMotorRunning(i)) {
                Motor_SetSpeed(i, 0);
                pid[i].integral = 0;
                pid[i].last_err = 0;
                pid[i].rpm = 0;
                PID_Screen_SetActualRPM(i, 0);
                PID_Screen_SetPWMOUT(i, 0);
                continue;
            }

#if M2_OPENLOOP_TEST == 2
            /* ===== 手动计数模式: 电机不转, 累计编码器脉冲数 ===== */
            if (i == 1) {
                Motor_SetSpeed(1, 0);                 /* 电机断电, 纯手动转 */
                int16_t raw = Encoder_GetCount(1);     /* 100ms 内增量 */
                m2_total_pulses += raw;                /* 累计, 防溢出 */
                /* RPM 位显示累计脉冲数 (转一圈看涨了多少)
                 * PWM% 位显示本次增量 (用来观察手转时是否稳定) */
                PID_Screen_SetActualRPM(1, (float)m2_total_pulses);
                PID_Screen_SetPWMOUT(1, (float)raw);
                continue;
            }
#elif M2_OPENLOOP_TEST == 1
            /* ===== 开环诊断: M2 固定 35% 正转, 不做闭环 ===== */
            if (i == 1) {
                Motor_SetSpeed(1, 35);                /* 固定正转 35% (启动临界) */
                int16_t raw = Encoder_GetCount(1);    /* 100ms 内原始计数, 不取反 */
                float raw_rpm = (float)raw * 60000.0f / (PULSE_PER_REV * SAMPLE_DT_MS);
                PID_Screen_SetActualRPM(1, raw_rpm);  /* RPM 位: 实际转速 */
                PID_Screen_SetPWMOUT(1, (float)raw);  /* PWM% 位: 显示原始计数 */
                continue;
            }
#endif

            /* 读编码器 */
            float actual = Encoder_GetRPM(i, PULSE_PER_REV, SAMPLE_DT_MS);
            actual = -actual;
            pid[i].rpm = actual;

            /* 低通滤波: 控制周期 5ms, 用 0.95/0.05 强滤波滤掉编码器噪声
             * 时间常数 ~100ms, 平滑波形抖动 */
            pid[i].rpm_filtered = pid[i].rpm_filtered * 0.95f + actual * 0.05f;
            actual = pid[i].rpm_filtered;

            /* 读屏幕参数 (每个电机有自己独立的参数) */
            float target = PID_Screen_GetTargetRPM(i);
            float kp = PID_Screen_GetKp(i);
            float ki = PID_Screen_GetKi(i);
            float kd = PID_Screen_GetKd(i);

            /* PID 计算 */
            float err = target - actual;
            float deriv = (err - pid[i].last_err) / dt;
            pid[i].last_err = err;

            /* 抗积分饱和 (anti-windup):
             * 先算新积分下的输出, 若输出已饱和且误差还在往饱和方向推,
             * 则冻结积分 (防止启动前积分囤太多, 过冲后迟迟卸不掉) */
            float new_integral = pid[i].integral + err * dt;
            if (new_integral > 200) new_integral = 200;
            if (new_integral < -200) new_integral = -200;
            float pwm_new = kp * err + ki * new_integral + kd * deriv;
            if ((pwm_new > 100 && err > 0) || (pwm_new < -100 && err < 0)) {
                /* 冻结积分, 用旧值 */
                pwm_new = kp * err + ki * pid[i].integral + kd * deriv;
            } else {
                pid[i].integral = new_integral;
            }
            float pwm = pwm_new;

            /* M2 死区补偿已删除 (用户要求)
             * if (i == 1 && actual < 0.5f) {
             *     if (pwm > 0 && pwm < 35) pwm = 0;
             *     if (pwm < 0 && pwm > -35) pwm = 0;
             * } */

            if (pwm > 100)  pwm = 100;
            if (pwm < -100) pwm = -100;

            /* 输出到电机 */
            Motor_SetSpeed(i, (int8_t)pwm);

            /* 喂给屏幕 */
            PID_Screen_SetActualRPM(i, actual);
            PID_Screen_SetPWMOUT(i, pwm);
        }
    }
}

/* ====================== main ====================== */
void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_FSMC_Init();

    DWT_Init();
    lcd_init();
    lcd_clear(BLACK);

    Motor_Init();
    Encoder_Init();
    PID_Screen_Init();

    xTaskCreate(PID_Task,        "PID", 256, NULL, 3, NULL);
    xTaskCreate(PID_Screen_Task, "UI",  1024, NULL, 2, NULL);

    vTaskStartScheduler();
    while (1);
}

/* ====================== 系统时钟 ====================== */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef o = {0};
    RCC_ClkInitTypeDef c = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    o.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    o.HSIState = RCC_HSI_ON;
    o.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    o.PLL.PLLState = RCC_PLL_ON;
    o.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    o.PLL.PLLM = 8; o.PLL.PLLN = 168; o.PLL.PLLP = RCC_PLLP_DIV2; o.PLL.PLLQ = 4;
    HAL_RCC_OscConfig(&o);
    c.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    c.AHBCLKDivider = RCC_SYSCLK_DIV1;
    c.APB1CLKDivider = RCC_HCLK_DIV4;
    c.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&c, FLASH_LATENCY_5);
}

void vApplicationStackOverflowHook(TaskHandle_t t, char *n) { (void)t; (void)n; while (1); }
void Error_Handler(void) { while (1); }
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *f, uint32_t l) { (void)f; (void)l; }
#endif
