/**
 ******************************************************************************
 * @file    pid_screen.c
 * @brief   PID 调试屏幕模块 - LVGL 实现
 *          320x240, 黑底绿/黄/青/红, 布局与 PID_Tuner_UI.html 完全一致
 *
 *          功能:
 *           - M1/M2 双电机独立 PID 参数
 *           - 触摸屏 +/- 按钮调 Kp/Ki/Kd/Target
 *           - 目标/实际转速实时波形 (10秒窗口, 100点)
 *           - 超调量/稳态误差显示
 *
 *          对外数据接口与旧版保持一致, 主程序 PID_Task 无需改动
 ******************************************************************************
 */

#include "main.h"
#include "lcd.h"
#include "delay.h"
#include "pid_screen.h"
#include "lvgl.h"
#include "lv_port.h"
#include "wave_grid.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

/* ====================== 屏幕参数 (与 html 一致) ====================== */
#define SCR_W           320
#define SCR_H           240

/* 颜色 (RGB565, 与 pid_screen 旧版 / HTML 一致) */
#define COL_BLACK       lv_color_hex(0x0000)
#define COL_PANEL       lv_color_hex(0x2104)
#define COL_BORDER      lv_color_hex(0x12A6)
#define COL_GREEN       lv_color_hex(0x07E0)
#define COL_YELLOW      lv_color_hex(0xFFE0)
#define COL_RED         lv_color_hex(0xF800)
#define COL_CYAN        lv_color_hex(0x07FF)
#define COL_GRAY        lv_color_hex(0x8430)
#define COL_MAGENTA     lv_color_hex(0xF81F)

/* 按钮底色 (按 HTML) */
#define FILL_M1_ACT     lv_color_hex(0x0d2a)
#define FILL_PLUS       lv_color_hex(0x0a2a)
#define FILL_MINUS      lv_color_hex(0x4a12)
#define FILL_RESET      lv_color_hex(0x2a2a)

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
static uint8_t is_running[2] = {0, 0};

/* 波形缓冲: 100 点 × 100ms = 10 秒 */
#define WAVE_POINTS     100
static float wave_target[2][WAVE_POINTS];
static float wave_actual[2][WAVE_POINTS];
static uint16_t wave_idx[2] = {0, 0};

/* ====================== LVGL 控件句柄 ====================== */
static lv_obj_t *tab_m1, *tab_m2;
static lv_obj_t *led_obj, *led_txt;
static lv_obj_t *val[4];            /* 左栏参数值 */
static lv_obj_t *ro[4];             /* 读数行数值 */
static lv_obj_t *chart;
static lv_chart_series_t *ser_target, *ser_actual;

/* ====================== XPT2046 触摸 (软件 SPI, 沿用旧版) ====================== */
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
    GPIOC->MODER &= ~(3<<10);              /* PC5=PEN 上拉输入 */
    GPIOC->PUPDR &= ~(3<<10);
    GPIOC->PUPDR |=  (1<<10);
    GPIOB->MODER &= ~(3<<28);              /* PB14=MISO 上拉输入 */
    GPIOB->PUPDR &= ~(3<<28);
    GPIOB->PUPDR |=  (1<<28);
    GPIOB->MODER &= ~((3<<24)|(3<<26)|(3<<30));  /* PB12=CS PB13=CLK PB15=MOSI 推挽输出 */
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

uint8_t PID_Screen_TouchGetPoint(uint16_t *x, uint16_t *y)
{
    return tp_get_point(x, y);
}

/* ====================== UI 构建 (与 html 一致) ====================== */

static lv_obj_t *mk_label(lv_obj_t *parent, int x, int y, const char *txt,
                          lv_color_t color, const lv_font_t *font)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_obj_set_pos(l, x, y);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_color(l, color, 0);
    lv_obj_set_style_text_font(l, font, 0);
    return l;
}

/* 示波器通道标签: 半透明底 + 色块 + 文字 (叠加在网格顶部) */
static lv_obj_t *mk_ch_tag(lv_obj_t *parent, int x, lv_color_t c, const char *txt)
{
    lv_obj_t *bg = lv_obj_create(parent);
    lv_obj_set_pos(bg, x, 2);
    lv_obj_set_size(bg, 50, 12);
    lv_obj_set_style_bg_color(bg, lv_color_hex(0x04090a), 0);
    lv_obj_set_style_bg_opa(bg, LV_OPA_60, 0);
    lv_obj_set_style_border_width(bg, 0, 0);
    lv_obj_set_style_radius(bg, 3, 0);
    lv_obj_set_style_pad_all(bg, 0, 0);

    lv_obj_t *sw = lv_obj_create(bg);
    lv_obj_set_pos(sw, 3, 2);
    lv_obj_set_size(sw, 9, 9);
    lv_obj_set_style_bg_color(sw, c, 0);
    lv_obj_set_style_border_width(sw, 0, 0);
    lv_obj_set_style_radius(sw, 1, 0);
    lv_obj_set_style_pad_all(sw, 0, 0);

    lv_obj_t *l = lv_label_create(bg);
    lv_obj_set_pos(l, 14, 1);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_color(l, c, 0);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_12, 0);
    return bg;
}

/* 扁平 3D 风格按钮: 深底 + 亮色描边 + 亮色文字 */
static lv_obj_t *mk_btn(lv_obj_t *parent, int x, int y, int w, int h,
                        const char *txt, lv_color_t bg, lv_color_t border,
                        lv_color_t fg, const lv_font_t *font)
{
    lv_obj_t *b = lv_btn_create(parent);
    lv_obj_set_pos(b, x, y);
    lv_obj_set_size(b, w, h);
    lv_obj_set_style_bg_color(b, bg, 0);
    lv_obj_set_style_border_color(b, border, 0);
    lv_obj_set_style_border_width(b, 1, 0);
    lv_obj_set_style_radius(b, 4, 0);
    lv_obj_set_style_pad_all(b, 0, 0);
    lv_obj_set_style_text_color(b, fg, 0);
    lv_obj_set_style_text_font(b, font, 0);

    lv_obj_t *l = lv_label_create(b);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_color(l, fg, 0);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_center(l);
    return b;
}

static void draw_param_values(void)
{
    char s[16];
    PID_Motor_t *m = &mtr[cur_motor];
    lv_snprintf(s, sizeof(s), "%.0f", m->target_rpm);  lv_label_set_text(val[0], s);
    lv_snprintf(s, sizeof(s), "%.1f", m->kp);          lv_label_set_text(val[1], s);
    lv_snprintf(s, sizeof(s), "%.2f", m->ki);          lv_label_set_text(val[2], s);
    lv_snprintf(s, sizeof(s), "%.2f", m->kd);          lv_label_set_text(val[3], s);
}

static void draw_motor_tabs(void)
{
    if (cur_motor == 0) {
        lv_obj_set_style_bg_color(tab_m1, FILL_M1_ACT, 0);
        lv_obj_set_style_border_color(tab_m1, COL_GREEN, 0);
        lv_obj_set_style_text_color(tab_m1, COL_GREEN, 0);
        lv_obj_set_style_bg_color(tab_m2, COL_PANEL, 0);
        lv_obj_set_style_border_color(tab_m2, COL_GRAY, 0);
        lv_obj_set_style_text_color(tab_m2, COL_GRAY, 0);
    } else {
        lv_obj_set_style_bg_color(tab_m1, COL_PANEL, 0);
        lv_obj_set_style_border_color(tab_m1, COL_GRAY, 0);
        lv_obj_set_style_text_color(tab_m1, COL_GRAY, 0);
        lv_obj_set_style_bg_color(tab_m2, FILL_M1_ACT, 0);
        lv_obj_set_style_border_color(tab_m2, COL_CYAN, 0);
        lv_obj_set_style_text_color(tab_m2, COL_CYAN, 0);
    }
}

static void draw_run_status(void)
{
    uint8_t running = is_running[cur_motor];
    lv_obj_set_style_bg_color(led_obj, running ? COL_GREEN : COL_GRAY, 0);
    lv_label_set_text(led_txt, running ? "RUN" : "STOP");
    lv_obj_set_style_text_color(led_txt, running ? COL_GREEN : COL_GRAY, 0);
}

/* 把当前电机的环形波形缓冲按时间序写入图表 */
static void draw_waveform(void)
{
    int m = cur_motor;
    lv_coord_t *yt = lv_chart_get_y_array(chart, ser_target);
    lv_coord_t *ya = lv_chart_get_y_array(chart, ser_actual);
    for (int i = 0; i < WAVE_POINTS; i++) {
        int idx = (wave_idx[m] + i) % WAVE_POINTS;
        yt[i] = (lv_coord_t)wave_target[m][idx];
        ya[i] = (lv_coord_t)wave_actual[m][idx];
    }
    lv_chart_refresh(chart);
}

static void draw_readout(void)
{
    char s[16];
    PID_Motor_t *m = &mtr[cur_motor];

    lv_snprintf(s, sizeof(s), "%.1f", m->actual_rpm);  lv_label_set_text(ro[0], s);
    lv_snprintf(s, sizeof(s), "%d", (int)m->pwm_out);  lv_label_set_text(ro[1], s);
    if (m->target_rpm > 1) {
        lv_snprintf(s, sizeof(s), "%.1f%%", m->overshoot);
    } else {
        lv_snprintf(s, sizeof(s), "--");
    }
    lv_label_set_text(ro[2], s);
    lv_snprintf(s, sizeof(s), "%.1f", m->target_rpm - m->actual_rpm);  lv_label_set_text(ro[3], s);
}

/* 电机切换: 重置该电机波形缓冲 */
static void switch_motor(uint8_t m)
{
    cur_motor = m;
    draw_motor_tabs();
    draw_param_values();
    draw_run_status();
    wave_idx[m] = 0;
    for (int i = 0; i < WAVE_POINTS; i++) {
        wave_target[m][i] = mtr[m].target_rpm;
        wave_actual[m][i] = mtr[m].actual_rpm;
    }
    draw_waveform();
}

/* ====================== 触摸事件回调 ====================== */
static void ev_dec(lv_event_t *e)
{
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    switch (idx) {
        case 0: mtr[cur_motor].target_rpm -= 10; if (mtr[cur_motor].target_rpm < 0) mtr[cur_motor].target_rpm = 0; break;
        case 1: mtr[cur_motor].kp -= 0.1f; if (mtr[cur_motor].kp < 0) mtr[cur_motor].kp = 0; break;
        case 2: mtr[cur_motor].ki -= 0.05f; if (mtr[cur_motor].ki < 0) mtr[cur_motor].ki = 0; break;
        case 3: mtr[cur_motor].kd -= 0.05f; if (mtr[cur_motor].kd < 0) mtr[cur_motor].kd = 0; break;
    }
    draw_param_values();
}

static void ev_inc(lv_event_t *e)
{
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    switch (idx) {
        case 0: mtr[cur_motor].target_rpm += 10; if (mtr[cur_motor].target_rpm > 250) mtr[cur_motor].target_rpm = 250; break;
        case 1: mtr[cur_motor].kp += 0.1f; break;
        case 2: mtr[cur_motor].ki += 0.05f; break;
        case 3: mtr[cur_motor].kd += 0.05f; break;
    }
    draw_param_values();
}

static void ev_tab(lv_event_t *e)
{
    int m = (int)(intptr_t)lv_event_get_user_data(e);
    switch_motor((uint8_t)m);
}

static void ev_start(lv_event_t *e) { (void)e; is_running[cur_motor] = 1; draw_run_status(); }
static void ev_stop(lv_event_t *e)  { (void)e; is_running[cur_motor] = 0; mtr[cur_motor].pwm_out = 0; draw_run_status(); }

static void ev_step(lv_event_t *e)
{
    (void)e;
    if (mtr[cur_motor].target_rpm < 150) mtr[cur_motor].target_rpm = 200;
    else                                 mtr[cur_motor].target_rpm = 100;
    mtr[cur_motor].peak_rpm   = mtr[cur_motor].actual_rpm;
    mtr[cur_motor].overshoot  = 0;
    mtr[cur_motor].step_armed = 1;
    draw_param_values();
}

static void ev_reset(lv_event_t *e)
{
    (void)e;
    mtr[cur_motor].kp = 2.0f;
    mtr[cur_motor].ki = 0.50f;
    mtr[cur_motor].kd = 0.20f;
    draw_param_values();
}

/* ====================== UI 搭建 ====================== */
static void build_ui(void)
{
    const lv_font_t *f12 = &lv_font_montserrat_12;
    const lv_font_t *f16 = &lv_font_montserrat_16;

    /* 主屏黑底 */
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, COL_BLACK, 0);

    /* ===== 顶栏 ===== */
    lv_obj_t *top = lv_obj_create(scr);
    lv_obj_set_pos(top, 0, 0);
    lv_obj_set_size(top, SCR_W, 24);
    lv_obj_set_style_bg_color(top, COL_PANEL, 0);
    lv_obj_set_style_border_width(top, 0, 0);
    lv_obj_set_style_radius(top, 0, 0);
    lv_obj_set_style_pad_all(top, 0, 0);

    mk_label(top, 4, 4, "PID Tuner", COL_CYAN, f12);

    tab_m1 = mk_btn(top, 110, 3, 31, 18, "M1", FILL_M1_ACT, COL_GREEN, COL_GREEN, f12);
    tab_m2 = mk_btn(top, 145, 3, 31, 18, "M2", COL_PANEL, COL_GRAY, COL_GRAY, f12);

    led_obj = lv_obj_create(top);
    lv_obj_set_pos(led_obj, 256, 8);
    lv_obj_set_size(led_obj, 9, 9);
    lv_obj_set_style_radius(led_obj, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(led_obj, COL_GRAY, 0);
    lv_obj_set_style_border_width(led_obj, 0, 0);
    lv_obj_set_style_pad_all(led_obj, 0, 0);
    led_txt = mk_label(top, 269, 4, "STOP", COL_GRAY, f12);

    /* ===== 左栏参数面板 ===== */
    lv_obj_t *left = lv_obj_create(scr);
    lv_obj_set_pos(left, 0, 24);
    lv_obj_set_size(left, 120, 162);
    lv_obj_set_style_bg_color(left, COL_BLACK, 0);
    lv_obj_set_style_border_color(left, COL_BORDER, 0);
    lv_obj_set_style_border_width(left, 1, 0);
    lv_obj_set_style_radius(left, 0, 0);
    lv_obj_set_style_pad_all(left, 0, 0);

    const char *labels[4] = {"Target", "Kp", "Ki", "Kd"};
    int ly[4] = {4, 42, 80, 118};
    int vy[4] = {24, 62, 100, 138};
    lv_color_t vcol[4] = {COL_YELLOW, COL_GREEN, COL_CYAN, COL_MAGENTA};

    for (int i = 0; i < 4; i++) {
        mk_label(left, 6, ly[i], labels[i], COL_GRAY, f12);
        val[i] = mk_label(left, 34, vy[i], "", vcol[i], f16);

        lv_obj_t *dec = mk_btn(left, 4,  ly[i]+18, 23, 19, "-", FILL_MINUS, COL_RED, COL_RED, f12);
        lv_obj_t *inc = mk_btn(left, 94, ly[i]+18, 23, 19, "+", FILL_PLUS, COL_GREEN, COL_GREEN, f12);
        lv_obj_add_event_cb(dec, ev_dec, LV_EVENT_CLICKED, (void *)(intptr_t)i);
        lv_obj_add_event_cb(inc, ev_inc, LV_EVENT_CLICKED, (void *)(intptr_t)i);
    }

    /* ===== 波形区 (示波器风格) ===== */
    lv_obj_t *wavebox = lv_obj_create(scr);
    lv_obj_set_pos(wavebox, 120, 25);
    lv_obj_set_size(wavebox, 199, 117);
    lv_obj_set_style_bg_color(wavebox, COL_BLACK, 0);
    lv_obj_set_style_border_color(wavebox, COL_BORDER, 0);
    lv_obj_set_style_border_width(wavebox, 1, 0);
    lv_obj_set_style_radius(wavebox, 0, 0);
    lv_obj_set_style_pad_all(wavebox, 0, 0);

    /* 示波器网格背景 (静态图, 与 HTML 一致) */
    lv_obj_t *grid = lv_img_create(wavebox);
    lv_img_set_src(grid, &wave_grid);
    lv_obj_set_pos(grid, 1, 1);

    chart = lv_chart_create(wavebox);
    lv_obj_set_pos(chart, 1, 1);
    lv_obj_set_size(chart, 195, 113);
    lv_obj_set_style_bg_opa(chart, LV_OPA_TRANSP, 0);    /* 透明, 露出网格 */
    lv_obj_set_style_border_width(chart, 0, 0);
    lv_obj_set_style_pad_all(chart, 0, 0);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(chart, WAVE_POINTS);
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, -50, 250);
    lv_chart_set_div_line_count(chart, 0, 0);
    lv_chart_set_axis_tick(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 0, 0, 0, false, 0);
    lv_chart_set_axis_tick(chart, LV_CHART_AXIS_PRIMARY_X, 0, 0, 0, 0, false, 0);
    lv_obj_set_style_line_width(chart, 2, LV_PART_ITEMS);
    lv_obj_set_style_line_rounded(chart, true, LV_PART_ITEMS);

    ser_target = lv_chart_add_series(chart, lv_color_hex(0xFFD34D), LV_CHART_AXIS_PRIMARY_Y);
    ser_actual = lv_chart_add_series(chart, lv_color_hex(0x39FF8B), LV_CHART_AXIS_PRIMARY_Y);

    /* 通道标签 + 时基 (叠加在网格顶部) */
    mk_ch_tag(wavebox, 4, lv_color_hex(0xFFD34D), "Target");
    mk_ch_tag(wavebox, 54, lv_color_hex(0x39FF8B), "Actual");
    mk_label(wavebox, 150, 3, "1 s/div", COL_GRAY, f12);

    /* ===== 读数行 ===== */
    lv_obj_t *robox = lv_obj_create(scr);
    lv_obj_set_pos(robox, 120, 145);
    lv_obj_set_size(robox, 199, 40);
    lv_obj_set_style_bg_color(robox, COL_BLACK, 0);
    lv_obj_set_style_border_color(robox, COL_BORDER, 0);
    lv_obj_set_style_border_width(robox, 1, 0);
    lv_obj_set_style_radius(robox, 0, 0);
    lv_obj_set_style_pad_all(robox, 0, 0);

    const char *roh[4] = {"RPM", "PWM%", "OVS%", "ERR"};
    int rox[4] = {0, 50, 100, 150};
    lv_color_t rocol[4] = {COL_GREEN, COL_YELLOW, COL_RED, COL_YELLOW};
    for (int i = 0; i < 4; i++) {
        /* 列标签: 宽49, 居中 */
        lv_obj_t *rh = lv_label_create(robox);
        lv_obj_set_pos(rh, rox[i], 3);
        lv_obj_set_width(rh, 49);
        lv_label_set_text(rh, roh[i]);
        lv_obj_set_style_text_color(rh, COL_GRAY, 0);
        lv_obj_set_style_text_font(rh, f12, 0);
        lv_obj_set_style_text_align(rh, LV_TEXT_ALIGN_CENTER, 0);
        /* 数值: 宽49, 居中 */
        lv_obj_t *rv = lv_label_create(robox);
        lv_obj_set_pos(rv, rox[i], 19);
        lv_obj_set_width(rv, 49);
        lv_label_set_text(rv, "");
        lv_obj_set_style_text_color(rv, rocol[i], 0);
        lv_obj_set_style_text_font(rv, f12, 0);
        lv_obj_set_style_text_align(rv, LV_TEXT_ALIGN_CENTER, 0);
        ro[i] = rv;
    }
    /* 水平分隔线 (标签/数值之间) */
    lv_obj_t *hsep = lv_obj_create(robox);
    lv_obj_set_pos(hsep, 0, 15);
    lv_obj_set_size(hsep, 199, 1);
    lv_obj_set_style_bg_color(hsep, COL_BORDER, 0);
    lv_obj_set_style_border_width(hsep, 0, 0);
    lv_obj_set_style_pad_all(hsep, 0, 0);
    /* 列分隔线 */
    int sepX[3] = {50, 99, 149};
    for (int i = 0; i < 3; i++) {
        lv_obj_t *sep = lv_obj_create(robox);
        lv_obj_set_pos(sep, sepX[i], 0);
        lv_obj_set_size(sep, 1, 40);
        lv_obj_set_style_bg_color(sep, COL_BORDER, 0);
        lv_obj_set_style_border_width(sep, 0, 0);
        lv_obj_set_style_pad_all(sep, 0, 0);
    }

    /* ===== 底栏 ===== */
    lv_obj_t *bar = lv_obj_create(scr);
    lv_obj_set_pos(bar, 0, 186);
    lv_obj_set_size(bar, SCR_W, 54);
    lv_obj_set_style_bg_color(bar, COL_PANEL, 0);
    lv_obj_set_style_border_width(bar, 0, 0);
    lv_obj_set_style_radius(bar, 0, 0);
    lv_obj_set_style_pad_all(bar, 0, 0);

    lv_obj_t *b_start = mk_btn(bar, 0,  5, 80, 44, "START", FILL_PLUS,  COL_GREEN,  COL_GREEN,  f16);
    lv_obj_t *b_stop  = mk_btn(bar, 80, 5, 80, 44, "STOP",  FILL_MINUS, COL_RED,    COL_RED,    f16);
    lv_obj_t *b_step  = mk_btn(bar, 160,5, 80, 44, "STEP",  FILL_M1_ACT,COL_CYAN,   COL_CYAN,   f16);
    lv_obj_t *b_reset = mk_btn(bar, 240,5, 80, 44, "RESET", FILL_RESET, COL_YELLOW, COL_YELLOW, f16);
    lv_obj_add_event_cb(b_start, ev_start, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(b_stop,  ev_stop,  LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(b_step,  ev_step,  LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(b_reset, ev_reset, LV_EVENT_CLICKED, NULL);

    lv_obj_add_event_cb(tab_m1, ev_tab, LV_EVENT_CLICKED, (void *)(intptr_t)PID_MOTOR_1);
    lv_obj_add_event_cb(tab_m2, ev_tab, LV_EVENT_CLICKED, (void *)(intptr_t)PID_MOTOR_2);

    /* 初始状态 */
    draw_motor_tabs();
    draw_param_values();
    draw_run_status();
    draw_waveform();
}

/* ====================== 对外接口 ====================== */
void PID_Screen_Init(void)
{
    int m, i;
    tp_init_hw();

    lv_init();
    lv_port_disp_init();
    lv_port_indev_init();

    for (m = 0; m < PID_MOTOR_NUM; m++) {
        for (i = 0; i < WAVE_POINTS; i++) {
            wave_target[m][i] = 0;
            wave_actual[m][i] = 0;
        }
        wave_idx[m] = 0;
    }

    build_ui();
}

/* ---- FreeRTOS 任务: 10ms 周期, 驱动 LVGL ---- */
void PID_Screen_Task(void * argument)
{
    TickType_t last_wave = 0;
    TickType_t last_ro   = 0;
    TickType_t last_tick;
    TickType_t now;

    (void)argument;
    last_tick = xTaskGetTickCount();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10));
        now = xTaskGetTickCount();

        /* LVGL tick 递增 (FreeRTOS 1ms tick 作为时钟源) */
        lv_tick_inc(now - last_tick);
        last_tick = now;

        /* 驱动 LVGL: 处理事件/触摸/刷新 */
        lv_timer_handler();

        /* 波形: 每 100ms 推一个点 */
        if ((now - last_wave) >= 100) {
            last_wave = now;
            int m = cur_motor;
            wave_target[m][wave_idx[m]] = mtr[m].target_rpm;
            wave_actual[m][wave_idx[m]] = mtr[m].actual_rpm;
            wave_idx[m]++;
            if (wave_idx[m] >= WAVE_POINTS) wave_idx[m] = 0;
            draw_waveform();
        }

        /* 读数: 每 200ms 刷新 */
        if ((now - last_ro) >= 200) {
            last_ro = now;
            draw_readout();
        }
    }
}

/* ---- 主程序数据接口 ---- */
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

float PID_Screen_GetKp(uint8_t motor)         { return mtr[motor].kp; }
float PID_Screen_GetKi(uint8_t motor)         { return mtr[motor].ki; }
float PID_Screen_GetKd(uint8_t motor)         { return mtr[motor].kd; }
float PID_Screen_GetTargetRPM(uint8_t motor)  { return mtr[motor].target_rpm; }
uint8_t PID_Screen_IsRunning(void)            { return is_running[cur_motor]; }
uint8_t PID_Screen_IsMotorRunning(uint8_t motor) { return is_running[motor]; }
uint8_t PID_Screen_GetCurrentMotor(void)      { return cur_motor; }
