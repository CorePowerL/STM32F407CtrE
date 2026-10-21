/**
 ******************************************************************************
 * @file    pid_screen.c
 * @brief   PID 调试屏幕模块实现
 *          320x240, 黑底绿/黄/青/红, 沿用 GenBotter Motor-1 LCD 风格
 ******************************************************************************
 */

#include "main.h"
#include "lcd.h"
#include "delay.h"
#include "pid_screen.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

/* ====================== 屏幕参数 ====================== */
#define SCR_W           320
#define SCR_H           240

/* 布局 */
#define TOP_BAR_H       24
#define BOTTOM_BAR_H    54          /* 底栏按钮高 */
#define LEFT_PANEL_W    120
#define WAVE_X          122
#define WAVE_Y          27
#define WAVE_W          195
#define WAVE_H          113
#define RO_Y            145         /* 读数行 y */
#define RO_H            38

/* 颜色 (RGB565, 用 lcd.h 已定义的) */
#define COL_BG          BLACK
#define COL_PANEL       0x2104
#define COL_BORDER      0x12A6
#define COL_GREEN       GREEN
#define COL_YELLOW      YELLOW
#define COL_RED         RED
#define COL_CYAN        CYAN
#define COL_GRAY        GRAY
#define COL_MAGENTA     MAGENTA

/* ====================== PID 参数存储 ====================== */
typedef struct {
    float kp;
    float ki;
    float kd;
    float target_rpm;
    float actual_rpm;
    float pwm_out;
    float overshoot;        /* 超调量 % */
    float peak_rpm;         /* 阶跃后的峰值 */
    uint8_t step_armed;     /* 阶跃后是否已统计过峰值 */
} PID_Motor_t;

static PID_Motor_t mtr[PID_MOTOR_NUM] = {
    { 0.5f, 0.10f, 0.05f, 50.0f, 0, 0, 0, 0, 0 },
    { 0.5f, 0.10f, 0.05f, 50.0f, 0, 0, 0, 0, 0 }
};

static uint8_t cur_motor = PID_MOTOR_1;
static uint8_t is_running[2] = {0, 0};  /* 每个电机独立的运行状态 */

/* 波形缓冲: 100 点 × 100ms = 10 秒 */
#define WAVE_POINTS     100
static float wave_target[WAVE_POINTS];
static float wave_actual[WAVE_POINTS];
static uint16_t wave_idx = 0;

/* ====================== 按钮定义 ====================== */
typedef struct {
    uint16_t x1, y1, x2, y2;
    uint8_t  id;
} Button_t;

/* 左列参数按钮: [-] 和 [+] */
/* Target: 标签 y=30, 按钮框 y=44~62 */
/* Kp:     标签 y=68, 按钮框 y=82~100 */
/* Ki:     标签 y=106, 按钮框 y=120~138 */
/* Kd:     标签 y=144, 按钮框 y=158~176 */
static Button_t btn_dec[4] = {
    {  4,  44,  26,  62, 10 },   /* Target - */
    {  4,  82,  26, 100, 11 },   /* Kp - */
    {  4, 120,  26, 138, 12 },   /* Ki - */
    {  4, 158,  26, 176, 13 },   /* Kd - */
};
static Button_t btn_inc[4] = {
    {  94,  44, 116,  62, 20 },
    {  94,  82, 116, 100, 21 },
    {  94, 120, 116, 138, 22 },
    {  94, 158, 116, 176, 23 },
};

/* 顶栏 M1/M2 切换 */
static Button_t btn_m1 = { 110, 3, 140, 21, 30 };
static Button_t btn_m2 = { 145, 3, 175, 21, 31 };

/* 底栏按钮: START / STOP / STEP / RESET */
/* y = 186 ~ 239, 4 等分 80px */
static Button_t btn_start = {  0, 186,  79, 239, 40 };
static Button_t btn_stop  = { 80, 186, 159, 239, 41 };
static Button_t btn_step  = {160, 186, 239, 239, 42 };
static Button_t btn_reset = {240, 186, 319, 239, 43 };

/* ====================== XPT2046 触摸 (软件 SPI) ====================== */
/* 沿用之前 main.c 的引脚定义 */
#define TP_PEN()      (GPIOC->IDR & GPIO_PIN_5)
#define TP_CS_H()     (GPIOB->BSRR = GPIO_PIN_12)
#define TP_CS_L()     (GPIOB->BSRR = GPIO_PIN_12<<16)
#define TP_CLK_H()    (GPIOB->BSRR = GPIO_PIN_13)
#define TP_CLK_L()    (GPIOB->BSRR = GPIO_PIN_13<<16)
#define TP_MOSI_H()   (GPIOB->BSRR = GPIO_PIN_15)
#define TP_MOSI_L()   (GPIOB->BSRR = GPIO_PIN_15<<16)
#define TP_MISO()     (GPIOB->IDR & GPIO_PIN_14)

static void tp_udelay(void) { for (volatile int i=0;i<8;i++); }

static void tp_init_hw(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    /* PC5=PEN 上拉输入 */
    GPIOC->MODER &= ~(3<<10);
    GPIOC->PUPDR &= ~(3<<10);
    GPIOC->PUPDR |=  (1<<10);
    /* PB14=MISO 上拉输入 */
    GPIOB->MODER &= ~(3<<28);
    GPIOB->PUPDR &= ~(3<<28);
    GPIOB->PUPDR |=  (1<<28);
    /* PB12=CS PB13=CLK PB15=MOSI 推挽输出 */
    GPIOB->MODER &= ~((3<<24)|(3<<26)|(3<<30));
    GPIOB->MODER |=  ((1<<24)|(1<<26)|(1<<30));
    GPIOB->OSPEEDR |= ((3<<24)|(3<<26)|(3<<30));
    TP_CS_H(); TP_CLK_H();
}

static uint16_t tp_adc(uint8_t cmd)
{
    uint16_t v=0;
    TP_CS_L(); tp_udelay();
    for (int i=0;i<8;i++) {
        if (cmd&0x80) TP_MOSI_H(); else TP_MOSI_L();
        cmd<<=1;
        TP_CLK_L(); tp_udelay(); TP_CLK_H(); tp_udelay();
    }
    tp_udelay();
    TP_CLK_L(); tp_udelay(); TP_CLK_H(); tp_udelay(); TP_CLK_L();
    for (int i=0;i<12;i++) {
        v<<=1;
        TP_CLK_L(); tp_udelay();
        if (TP_MISO()) v++;
        TP_CLK_H(); tp_udelay();
    }
    TP_CS_H();
    return v;
}

static uint8_t tp_get_point(uint16_t *x, uint16_t *y)
{
    uint16_t ax, ay;
    if (TP_PEN()) return 0;
    ax = tp_adc(0x90);
    ay = tp_adc(0xD0);
    /* 五点校准映射 (320x240) */
    if (ax > 3700) ax = 3700;
    if (ay > 3725) ay = 3725;
    *x = (uint32_t)(ax - 450) * 319 / 3250;
    *y = (uint32_t)(3725 - ay) * 239 / 3235;
    return 1;
}

/* ====================== UI 绘制 ====================== */

/* 立体按钮绘制: 圆角 + 顶部亮边 + 底部暗边 = 凸起效果 (无填充) */
static void draw_button_3d(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                           uint16_t base_color, uint16_t border_color,
                           const char *text, uint16_t text_color, uint8_t font_size)
{
    uint16_t i;
    /* 圆角半径 */
    uint16_t r = 4;

    /* 1. 顶部亮边 (模拟高光) */
    lcd_draw_line(x1+r, y1, x2-r, y1, 0xFFFF);  /* 白色顶边 */
    lcd_draw_line(x1, y1+r, x1, y2-r, 0xFFFF);  /* 左边亮线 */

    /* 2. 底部暗边 (模拟阴影) */
    lcd_draw_line(x1+r, y2, x2-r, y2, 0x0000);  /* 黑色底边 */
    lcd_draw_line(x2, y1+r, x2, y2-r, 0x0000);  /* 右边暗线 */

    /* 3. 圆角切角线 */
    /* 左上角 */
    for (i=0;i<r;i++) {
        lcd_draw_line(x1+r-i, y1+i, x1+i, y1+r-i, border_color);
    }
    /* 右上角 */
    for (i=0;i<r;i++) {
        lcd_draw_line(x2-r+i, y1+i, x2-i, y1+r-i, border_color);
    }
    /* 左下角 */
    for (i=0;i<r;i++) {
        lcd_draw_line(x1+r-i, y2-i, x1+i, y2-r+i, border_color);
    }
    /* 右下角 */
    for (i=0;i<r;i++) {
        lcd_draw_line(x2-r+i, y2-i, x2-i, y2-r+i, border_color);
    }

    /* 4. 文字居中 */
    uint16_t w = x2 - x1 + 1;
    uint16_t h = y2 - y1 + 1;
    uint16_t text_w = strlen(text) * font_size * 0.8;
    uint16_t text_x = x1 + (w - text_w) / 2;
    uint16_t text_y = y1 + (h - font_size) / 2;
    lcd_show_string(text_x, text_y, w, h, font_size, (char*)text, text_color);
}

/* 普通按钮 (小按钮用) */
static void draw_button(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                        uint16_t bg_color, uint16_t border_color,
                        const char *text, uint16_t text_color, uint8_t font_size)
{
    draw_button_3d(x1, y1, x2, y2, bg_color, border_color, text, text_color, font_size);
}

static void draw_static_ui(void)
{
    char s[16];
    int i;

    /* 顶栏 */
    lcd_fill(0, 0, SCR_W-1, TOP_BAR_H-1, COL_PANEL);
    lcd_show_string(4, 4, 70, 16, 12, "PID Tuner", COL_CYAN);

    /* M1/M2 按钮 */
    draw_button(btn_m1.x1, btn_m1.y1, btn_m1.x2, btn_m1.y2,
                0x0d2a, COL_GREEN, "M1", COL_GREEN, 12);
    draw_button(btn_m2.x1, btn_m2.y1, btn_m2.x2, btn_m2.y2,
                COL_PANEL, COL_GRAY, "M2", COL_GRAY, 12);

    /* RUN 状态灯 */
    lcd_fill_circle(260, 12, 4, COL_GRAY);
    lcd_show_string(268, 5, 40, 14, 12, "STOP", COL_GRAY);

    /* 左列参数面板背景 */
    lcd_fill(0, TOP_BAR_H, LEFT_PANEL_W-1, SCR_H-BOTTOM_BAR_H-1, COL_BG);
    lcd_draw_rectangle(0, TOP_BAR_H, LEFT_PANEL_W-1, SCR_H-BOTTOM_BAR_H-1, COL_BORDER);

    /* 4 组参数标签和按钮框 */
    const char *labels[4] = {"Target", "Kp", "Ki", "Kd"};
    uint16_t ys[4] = {30, 68, 106, 144};
    for (i=0;i<4;i++) {
        lcd_show_string(6, ys[i]-2, 50, 12, 12, (char*)labels[i], COL_GRAY);
        /* - 按钮 (红色) */
        draw_button(btn_dec[i].x1, btn_dec[i].y1, btn_dec[i].x2, btn_dec[i].y2,
                    0x4a12, COL_RED, "-", COL_RED, 12);
        /* + 按钮 (绿色) */
        draw_button(btn_inc[i].x1, btn_inc[i].y1, btn_inc[i].x2, btn_inc[i].y2,
                    0x0a2a, COL_GREEN, "+", COL_GREEN, 12);
    }

    /* 波形区 */
    lcd_draw_rectangle(WAVE_X-2, WAVE_Y-2, WAVE_X+WAVE_W+1, WAVE_Y+WAVE_H+1, COL_BORDER);
    lcd_show_string(WAVE_X+4, WAVE_Y-1, 80, 10, 12, "RPM Wave", COL_GRAY);

    /* 波形网格横线: 0/100/200 rpm */
    /* Y 轴: -50 ~ 250 rpm, 高 WAVE_H */
    /* y(rpm) = WAVE_Y + WAVE_H - (rpm+50)/300*WAVE_H */

    /* 读数行 */
    lcd_draw_rectangle(WAVE_X-2, RO_Y, WAVE_X+WAVE_W+1, RO_Y+RO_H, COL_BORDER);
    /* 4 列分隔线: 宽 195, 每列 ~48 */
    lcd_draw_line(WAVE_X+48, RO_Y,     WAVE_X+48, RO_Y+RO_H, COL_BORDER);
    lcd_draw_line(WAVE_X+97, RO_Y,     WAVE_X+97, RO_Y+RO_H, COL_BORDER);
    lcd_draw_line(WAVE_X+146, RO_Y,    WAVE_X+146, RO_Y+RO_H, COL_BORDER);
    lcd_show_string(WAVE_X+4,  RO_Y+2,  40, 10, 12, "RPM",   COL_GRAY);
    lcd_show_string(WAVE_X+52, RO_Y+2,  40, 10, 12, "PWM%",  COL_GRAY);
    lcd_show_string(WAVE_X+101,RO_Y+2,  40, 10, 12, "Overshoot", COL_GRAY);
    lcd_show_string(WAVE_X+150,RO_Y+2,  40, 10, 12, "Err",   COL_GRAY);

    /* 底栏 */
    lcd_fill(0, SCR_H-BOTTOM_BAR_H, SCR_W-1, SCR_H-1, COL_PANEL);
    lcd_draw_hline(0, SCR_H-BOTTOM_BAR_H, SCR_W, COL_BORDER);

    /* 底栏 4 个大按钮 */
    draw_button(0,   SCR_H-BOTTOM_BAR_H+5, 79,  SCR_H-5,
                0x0a2a, COL_GREEN, "START", COL_GREEN, 16);
    draw_button(80,  SCR_H-BOTTOM_BAR_H+5, 159, SCR_H-5,
                0x4a12, COL_RED, "STOP", COL_RED, 16);
    draw_button(160, SCR_H-BOTTOM_BAR_H+5, 239, SCR_H-5,
                0x0d2a, COL_CYAN, "STEP", COL_CYAN, 16);
    draw_button(240, SCR_H-BOTTOM_BAR_H+5, 319, SCR_H-5,
                0x2a2a, COL_YELLOW, "RESET", COL_YELLOW, 16);
}

/* 数值刷新 (擦旧值再写新值) */
static void draw_param_values(void)
{
    char s[16];
    PID_Motor_t *m = &mtr[cur_motor];

    /* 擦旧值区域 */
    lcd_fill(32, 46,  90, 60, COL_BG);
    lcd_fill(32, 84,  90, 98, COL_BG);
    lcd_fill(32, 122, 90, 136, COL_BG);
    lcd_fill(32, 160, 90, 174, COL_BG);

    sprintf(s, "%.0f", m->target_rpm);
    lcd_show_string(34, 48,  55, 16, 16, s, COL_YELLOW);
    sprintf(s, "%.1f", m->kp);
    lcd_show_string(34, 86,  55, 16, 16, s, COL_GREEN);
    sprintf(s, "%.2f", m->ki);
    lcd_show_string(34, 124, 55, 16, 16, s, COL_CYAN);
    sprintf(s, "%.2f", m->kd);
    lcd_show_string(34, 162, 55, 16, 16, s, COL_MAGENTA);
}

static void draw_motor_tabs(void)
{
    /* 高亮当前电机 */
    draw_button(btn_m1.x1, btn_m1.y1, btn_m1.x2, btn_m1.y2,
                cur_motor==0 ? 0x0d2a : COL_PANEL,
                cur_motor==0 ? COL_GREEN : COL_GRAY,
                "M1", cur_motor==0 ? COL_GREEN : COL_GRAY, 12);
    draw_button(btn_m2.x1, btn_m2.y1, btn_m2.x2, btn_m2.y2,
                cur_motor==1 ? 0x0d2a : COL_PANEL,
                cur_motor==1 ? COL_GREEN : COL_GRAY,
                "M2", cur_motor==1 ? COL_GREEN : COL_GRAY, 12);
}

static void draw_run_status(void)
{
    uint8_t running = is_running[cur_motor];
    lcd_fill_circle(260, 12, 4, running ? COL_GREEN : COL_GRAY);
    lcd_fill(268, 5, 310, 19, COL_PANEL);
    lcd_show_string(268, 5, 40, 14, 12,
                    running ? "RUN" : "STOP",
                    running ? COL_GREEN : COL_GRAY);
}

static void draw_readout(void)
{
    char s[16];
    PID_Motor_t *m = &mtr[cur_motor];

    lcd_fill(WAVE_X+4,  RO_Y+15, WAVE_X+44,  RO_Y+33, COL_BG);
    lcd_fill(WAVE_X+52, RO_Y+15, WAVE_X+93,  RO_Y+33, COL_BG);
    lcd_fill(WAVE_X+101,RO_Y+15, WAVE_X+143, RO_Y+33, COL_BG);
    lcd_fill(WAVE_X+150,RO_Y+15, WAVE_X+190, RO_Y+33, COL_BG);

    sprintf(s, "%.1f", m->actual_rpm);
    lcd_show_string(WAVE_X+4,  RO_Y+16, 42, 14, 12, s, COL_GREEN);

    sprintf(s, "%d", (int)m->pwm_out);
    lcd_show_string(WAVE_X+52, RO_Y+16, 40, 14, 12, s, COL_YELLOW);

    if (m->target_rpm > 1) {
        sprintf(s, "%.1f%%", m->overshoot);
    } else {
        sprintf(s, "--");
    }
    lcd_show_string(WAVE_X+101,RO_Y+16, 42, 14, 12, s, COL_RED);

    sprintf(s, "%.1f", m->target_rpm - m->actual_rpm);
    lcd_show_string(WAVE_X+150,RO_Y+16, 42, 14, 12, s, COL_YELLOW);
}

/* RPM -> 像素 y */
static uint16_t rpm_to_y(float rpm)
{
    float v = rpm;
    if (v < -50) v = -50;
    if (v > 250) v = 250;
    return WAVE_Y + WAVE_H - 1 - (uint16_t)((v + 50.0f) / 300.0f * (WAVE_H - 2));
}

static void draw_waveform(void)
{
    uint16_t i;
    /* 清波形区 */
    lcd_fill(WAVE_X, WAVE_Y, WAVE_X+WAVE_W-1, WAVE_Y+WAVE_H-1, COL_BG);

    /* 网格 */
    for (i=0;i<3;i++) {
        uint16_t y = rpm_to_y(i*100.0f);
        lcd_draw_hline(WAVE_X, y, WAVE_W, 0x12A6);
    }

    /* 画旧波形 (先擦掉重画整屏, 简单可靠) */
    /* 目标线 (黄色) */
    for (i=0;i<WAVE_POINTS-1;i++) {
        uint16_t i1 = (wave_idx + i) % WAVE_POINTS;
        uint16_t i2 = (wave_idx + i + 1) % WAVE_POINTS;
        uint16_t x1 = WAVE_X + (uint16_t)((float)i/(WAVE_POINTS-1)*WAVE_W);
        uint16_t x2 = WAVE_X + (uint16_t)((float)(i+1)/(WAVE_POINTS-1)*WAVE_W);
        lcd_draw_line(x1, rpm_to_y(wave_target[i1]), x2, rpm_to_y(wave_target[i2]), COL_YELLOW);
    }
    /* 实际线 (绿色) */
    for (i=0;i<WAVE_POINTS-1;i++) {
        uint16_t i1 = (wave_idx + i) % WAVE_POINTS;
        uint16_t i2 = (wave_idx + i + 1) % WAVE_POINTS;
        uint16_t x1 = WAVE_X + (uint16_t)((float)i/(WAVE_POINTS-1)*WAVE_W);
        uint16_t x2 = WAVE_X + (uint16_t)((float)(i+1)/(WAVE_POINTS-1)*WAVE_W);
        lcd_draw_line(x1, rpm_to_y(wave_actual[i1]), x2, rpm_to_y(wave_actual[i2]), COL_GREEN);
    }
}

/* ====================== 触摸处理 ====================== */
static uint8_t in_btn(uint16_t x, uint16_t y, Button_t *b)
{
    return (x>=b->x1 && x<=b->x2 && y>=b->y1 && y<=b->y2);
}

static void tp_handle(void)
{
    uint16_t x, y;
    if (!tp_get_point(&x, &y)) return;

    /* 左列 +/- 按钮 */
    int i;
    for (i=0;i<4;i++) {
        if (in_btn(x, y, &btn_dec[i])) {
            switch (i) {
                case 0: mtr[cur_motor].target_rpm -= 10; if (mtr[cur_motor].target_rpm<0) mtr[cur_motor].target_rpm=0; break;
                case 1: mtr[cur_motor].kp -= 0.01f; if (mtr[cur_motor].kp<0) mtr[cur_motor].kp=0; break;
                case 2: mtr[cur_motor].ki -= 0.05f; if (mtr[cur_motor].ki<0) mtr[cur_motor].ki=0; break;
                case 3: mtr[cur_motor].kd -= 0.05f; if (mtr[cur_motor].kd<0) mtr[cur_motor].kd=0; break;
            }
            draw_param_values();
            vTaskDelay(pdMS_TO_TICKS(150));  /* 简单去抖 */
            return;
        }
        if (in_btn(x, y, &btn_inc[i])) {
            switch (i) {
                case 0: mtr[cur_motor].target_rpm += 10; if (mtr[cur_motor].target_rpm>250) mtr[cur_motor].target_rpm=250; break;
                case 1: mtr[cur_motor].kp += 0.01f; break;
                case 2: mtr[cur_motor].ki += 0.05f; break;
                case 3: mtr[cur_motor].kd += 0.05f; break;
            }
            draw_param_values();
            vTaskDelay(pdMS_TO_TICKS(150));
            return;
        }
    }

    /* M1/M2 */
    if (in_btn(x, y, &btn_m1)) {
        cur_motor = PID_MOTOR_1;
        draw_motor_tabs(); draw_param_values();
        /* 清空波形并重置索引 */
        wave_idx = 0;
        for (i=0;i<WAVE_POINTS;i++) {
            wave_target[i] = mtr[cur_motor].target_rpm;
            wave_actual[i] = mtr[cur_motor].actual_rpm;
        }
        draw_waveform();
        vTaskDelay(pdMS_TO_TICKS(150));
        return;
    }
    if (in_btn(x, y, &btn_m2)) {
        cur_motor = PID_MOTOR_2;
        draw_motor_tabs(); draw_param_values();
        wave_idx = 0;
        for (i=0;i<WAVE_POINTS;i++) {
            wave_target[i] = mtr[cur_motor].target_rpm;
            wave_actual[i] = mtr[cur_motor].actual_rpm;
        }
        draw_waveform();
        vTaskDelay(pdMS_TO_TICKS(150));
        return;
    }

    /* 底栏按钮 - 只控制当前选中的电机 */
    if (in_btn(x, y, &btn_start)) {
        is_running[cur_motor] = 1;
        draw_run_status();
        vTaskDelay(pdMS_TO_TICKS(150));
        return;
    }
    if (in_btn(x, y, &btn_stop)) {
        is_running[cur_motor] = 0;
        mtr[cur_motor].pwm_out = 0;
        draw_run_status();
        vTaskDelay(pdMS_TO_TICKS(150));
        return;
    }
    if (in_btn(x, y, &btn_step)) {
        /* 在 100 和 200 之间切换目标 */
        if (mtr[cur_motor].target_rpm < 150) mtr[cur_motor].target_rpm = 200;
        else mtr[cur_motor].target_rpm = 100;
        mtr[cur_motor].peak_rpm = mtr[cur_motor].actual_rpm;
        mtr[cur_motor].overshoot = 0;
        mtr[cur_motor].step_armed = 1;
        draw_param_values();
        vTaskDelay(pdMS_TO_TICKS(150));
        return;
    }
    if (in_btn(x, y, &btn_reset)) {
        mtr[cur_motor].kp = 2.0f;
        mtr[cur_motor].ki = 0.50f;
        mtr[cur_motor].kd = 0.20f;
        draw_param_values();
        vTaskDelay(pdMS_TO_TICKS(150));
        return;
    }
}

/* ====================== 对外接口 ====================== */
void PID_Screen_Init(void)
{
    int i;
    tp_init_hw();
    for (i=0;i<WAVE_POINTS;i++) {
        wave_target[i] = 0;
        wave_actual[i] = 0;
    }
    wave_idx = 0;
    draw_static_ui();
    draw_motor_tabs();
    draw_param_values();
    draw_run_status();
    draw_waveform();
}

void PID_Screen_SetActualRPM(uint8_t motor, float rpm)
{
    if (motor >= PID_MOTOR_NUM) return;
    mtr[motor].actual_rpm = rpm;

    /* 统计超调量 */
    if (mtr[motor].step_armed) {
        if (rpm > mtr[motor].peak_rpm) mtr[motor].peak_rpm = rpm;
        if (mtr[motor].target_rpm > 1) {
            mtr[motor].overshoot = (mtr[motor].peak_rpm - mtr[motor].target_rpm)
                                   / mtr[motor].target_rpm * 100.0f;
            if (mtr[motor].overshoot < 0) mtr[motor].overshoot = 0;
        }
    }
}

void PID_Screen_SetPWMOUT(uint8_t motor, float pct)
{
    if (motor >= PID_MOTOR_NUM) return;
    mtr[motor].pwm_out = pct;
}

float PID_Screen_GetKp(uint8_t motor)    { return mtr[motor].kp; }
float PID_Screen_GetKi(uint8_t motor)    { return mtr[motor].ki; }
float PID_Screen_GetKd(uint8_t motor)    { return mtr[motor].kd; }
float PID_Screen_GetTargetRPM(uint8_t motor) { return mtr[motor].target_rpm; }
uint8_t PID_Screen_IsRunning(void)       { return is_running[cur_motor]; }
uint8_t PID_Screen_IsMotorRunning(uint8_t motor) { return is_running[motor]; }
uint8_t PID_Screen_GetCurrentMotor(void) { return cur_motor; }

/* ---- FreeRTOS 任务 ---- */
void PID_Screen_Task(void * argument)
{
    TickType_t last_wave = 0;
    TickType_t last_ro   = 0;
    TickType_t now;
    int i;

    (void)argument;

    /* 把 pwm_out 也暴露一下: 主程序算完 PWM 后写回 mtr[i].pwm_out */
    /* 这里通过外部函数间接更新, 简化处理: 主程序在 PID 计算后调用
       PID_Screen_SetActualRPM() 同时也应把 PWM 写入, 这里另开一个函数
       见 pid_screen.h 末尾注释 */

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10));
        now = xTaskGetTickCount();

        /* 触摸扫描 */
        tp_handle();

        /* 波形: 每 100ms 推一个点 */
        if ((now - last_wave) >= 100) {
            last_wave = now;
            wave_target[wave_idx] = mtr[cur_motor].target_rpm;
            wave_actual[wave_idx] = mtr[cur_motor].actual_rpm;
            wave_idx++;
            if (wave_idx >= WAVE_POINTS) wave_idx = 0;
            draw_waveform();
        }

        /* 读数: 每 200ms 刷新一次 (降低闪烁) */
        if ((now - last_ro) >= 200) {
            last_ro = now;
            draw_readout();
        }
    }
}
