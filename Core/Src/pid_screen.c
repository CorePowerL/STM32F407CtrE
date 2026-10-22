/**
 ******************************************************************************
 * @file    pid_screen.c
 * @brief   PID 调试屏幕模块 - LVGL 实现 (简约高端深色仪器风)
 *          320x240, 显示与 PID_Tuner_UI.html 当前设计完全一致
 *
 *          设计语言:
 *           - 分层深灰黑底, 细分隔线 (白色 6%~7%), 无粗边框
 *           - 单一强调色蓝 #4a9eff, 红色仅用于 STOP/超调
 *           - 参数数值统一浅色, 无荧光多彩, 无辉光
 *
 *          功能 (按键与 HTML 同步):
 *           - M1/M2 双电机独立 PID 参数, Tab 切换
 *           - 触摸 +/- 调 Target/Kp/Ki/Kd
 *           - 目标/实际转速实时波形 (10s 窗口, 100 点)
 *           - RPM/PWM/OVS/ERR 读数, START/STOP/STEP/RESET
 *
 *          对外数据接口与旧版一致, 主程序 PID_Task 无需改动
 ******************************************************************************
 */

#include "main.h"
#include "lcd.h"
#include "delay.h"
#include "pid_screen.h"
#include "lvgl.h"
#include "lv_port.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

/* ====================== 屏幕参数 (与 html 一致) ====================== */
#define SCR_W           320
#define SCR_H           240

/* ---- 简约配色 (取自当前 HTML) ---- */
#define COL_SCR         lv_color_hex(0x0C0F14)   /* 屏幕/面板底 */
#define COL_TOPBAR      lv_color_hex(0x0F1318)   /* 顶栏/底栏 */
#define COL_WAVE        lv_color_hex(0x0A0D11)   /* 波形区 */
#define COL_BTN         lv_color_hex(0x1A2029)   /* 加减按钮 */
#define COL_INK         lv_color_hex(0xE6EAF0)   /* 主文字/数值 */
#define COL_MUTED       lv_color_hex(0x5A6472)   /* 标签/未选中 */
#define COL_SUB         lv_color_hex(0x8B95A3)   /* 次级文字 */
#define COL_ACCENT      lv_color_hex(0x4A9EFF)   /* 唯一强调蓝 */
#define COL_ACCENT_DK   lv_color_hex(0x1A5FB4)   /* 主按钮实底 */
#define COL_DANGER      lv_color_hex(0xC97074)   /* 克制红 */
#define COL_TARGET      lv_color_hex(0x6B7684)   /* 目标波形灰蓝 */

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
static lv_obj_t *tab_m1_lbl, *tab_m2_lbl;
static lv_obj_t *tab_m1_line, *tab_m2_line;
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

/* ====================== UI 辅助构建 ====================== */
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

/* 幽灵/实色按钮: 颜色由调用方通过样式给定 */
static lv_obj_t *mk_btn(lv_obj_t *parent, int x, int y, int w, int h,
                        const char *txt, lv_color_t bg, lv_opa_t bg_opa,
                        lv_color_t border, lv_color_t fg,
                        const lv_font_t *font, lv_coord_t radius)
{
    lv_obj_t *b = lv_btn_create(parent);
    lv_obj_set_pos(b, x, y);
    lv_obj_set_size(b, w, h);
    lv_obj_set_style_bg_color(b, bg, 0);
    lv_obj_set_style_bg_opa(b, bg_opa, 0);
    lv_obj_set_style_border_color(b, border, 0);
    lv_obj_set_style_border_width(b, 1, 0);
    lv_obj_set_style_radius(b, radius, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
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

/* 透明容器: 无圆角无描边无内边距 */
static lv_obj_t *mk_panel(lv_obj_t *parent, int x, int y, int w, int h,
                          lv_color_t bg)
{
    lv_obj_t *p = lv_obj_create(parent);
    lv_obj_set_pos(p, x, y);
    lv_obj_set_size(p, w, h);
    lv_obj_set_style_bg_color(p, bg, 0);
    lv_obj_set_style_border_width(p, 0, 0);
    lv_obj_set_style_radius(p, 0, 0);
    lv_obj_set_style_pad_all(p, 0, 0);
    lv_obj_clear_flag(p, LV_OBJ_FLAG_SCROLLABLE);
    return p;
}

/* 1px 分隔线 */
static void mk_sep(lv_obj_t *parent, int x, int y, int w, int h)
{
    lv_obj_t *s = lv_obj_create(parent);
    lv_obj_set_pos(s, x, y);
    lv_obj_set_size(s, w, h);
    lv_obj_set_style_bg_color(s, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(s, LV_OPA_10, 0);
    lv_obj_set_style_border_width(s, 0, 0);
    lv_obj_set_style_radius(s, 0, 0);
    lv_obj_set_style_pad_all(s, 0, 0);
    lv_obj_clear_flag(s, LV_OBJ_FLAG_CLICKABLE);
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

/* Tab 选中态: 文字亮度 + 底部 2px 蓝线 */
static void draw_motor_tabs(void)
{
    lv_obj_set_style_text_color(tab_m1_lbl, cur_motor == 0 ? COL_INK : COL_MUTED, 0);
    lv_obj_set_style_text_color(tab_m2_lbl, cur_motor == 1 ? COL_INK : COL_MUTED, 0);
    if (cur_motor == 0) { lv_obj_clear_flag(tab_m1_line, LV_OBJ_FLAG_HIDDEN);
                          lv_obj_add_flag(tab_m2_line, LV_OBJ_FLAG_HIDDEN); }
    else                { lv_obj_add_flag(tab_m1_line, LV_OBJ_FLAG_HIDDEN);
                          lv_obj_clear_flag(tab_m2_line, LV_OBJ_FLAG_HIDDEN); }
}

static void draw_run_status(void)
{
    uint8_t running = is_running[cur_motor];
    lv_obj_set_style_bg_color(led_obj, running ? COL_ACCENT : lv_color_hex(0x3A414C), 0);
    lv_label_set_text(led_txt, running ? "RUN" : "STOP");
    lv_obj_set_style_text_color(led_txt, running ? lv_color_hex(0x8FB8EC) : COL_MUTED, 0);
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

/* ====================== 触摸事件回调 (与 HTML 同步) ====================== */
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

/* ====================== UI 搭建 (与 html 一致) ====================== */
static void build_ui(void)
{
    const lv_font_t *f12 = &lv_font_montserrat_12;
    const lv_font_t *f14 = &lv_font_montserrat_14;
    const lv_font_t *f16 = &lv_font_montserrat_16;

    /* 主屏底色 */
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, COL_SCR, 0);

    /* ===== 顶栏 (0,0 320x24) ===== */
    mk_panel(scr, 0, 0, SCR_W, 24, COL_TOPBAR);
    mk_sep(scr, 0, 23, SCR_W, 1);

    mk_label(scr, 10, 5, "PID TUNER", COL_MUTED, f12);

    /* Tab: 透明按钮, 内部 label + 底部线 */
    tab_m1 = mk_panel(scr, 88, 0, 42, 24, COL_TOPBAR);
    tab_m1_lbl = mk_label(tab_m1, 0, 5, "M1", COL_INK, f12);
    lv_obj_set_width(tab_m1_lbl, 42);
    lv_obj_set_style_text_align(tab_m1_lbl, LV_TEXT_ALIGN_CENTER, 0);
    tab_m1_line = lv_obj_create(tab_m1);
    lv_obj_set_pos(tab_m1_line, 8, 22);
    lv_obj_set_size(tab_m1_line, 26, 2);
    lv_obj_set_style_bg_color(tab_m1_line, COL_ACCENT, 0);
    lv_obj_set_style_border_width(tab_m1_line, 0, 0);
    lv_obj_set_style_radius(tab_m1_line, 1, 0);
    lv_obj_set_style_pad_all(tab_m1_line, 0, 0);
    lv_obj_clear_flag(tab_m1_line, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(tab_m1, LV_OBJ_FLAG_CLICKABLE);

    tab_m2 = mk_panel(scr, 134, 0, 42, 24, COL_TOPBAR);
    tab_m2_lbl = mk_label(tab_m2, 0, 5, "M2", COL_MUTED, f12);
    lv_obj_set_width(tab_m2_lbl, 42);
    lv_obj_set_style_text_align(tab_m2_lbl, LV_TEXT_ALIGN_CENTER, 0);
    tab_m2_line = lv_obj_create(tab_m2);
    lv_obj_set_pos(tab_m2_line, 8, 22);
    lv_obj_set_size(tab_m2_line, 26, 2);
    lv_obj_set_style_bg_color(tab_m2_line, COL_ACCENT, 0);
    lv_obj_set_style_border_width(tab_m2_line, 0, 0);
    lv_obj_set_style_radius(tab_m2_line, 1, 0);
    lv_obj_set_style_pad_all(tab_m2_line, 0, 0);
    lv_obj_clear_flag(tab_m2_line, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(tab_m2_line, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(tab_m2, LV_OBJ_FLAG_CLICKABLE);

    /* 状态点 + 状态文字 (右侧) */
    led_obj = lv_obj_create(scr);
    lv_obj_set_pos(led_obj, 308, 9);
    lv_obj_set_size(led_obj, 6, 6);
    lv_obj_set_style_radius(led_obj, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(led_obj, lv_color_hex(0x3A414C), 0);
    lv_obj_set_style_border_width(led_obj, 0, 0);
    lv_obj_set_style_pad_all(led_obj, 0, 0);
    led_txt = mk_label(scr, 270, 5, "STOP", COL_MUTED, f12);

    /* ===== 左栏参数面板 (0,24 90x162) ===== */
    lv_obj_t *left = mk_panel(scr, 0, 24, 90, 162, COL_SCR);
    mk_sep(left, 89, 0, 1, 162);

    const char *labels[4] = {"TARGET", "KP", "KI", "KD"};
    int ly[4] = {3, 43, 83, 123};      /* 标签行 */
    int by[4] = {17, 57, 97, 137};     /* 数值/按钮行 */

    for (int i = 0; i < 4; i++) {
        mk_label(left, 10, ly[i], labels[i], COL_MUTED, f12);

        lv_obj_t *dec = mk_btn(left, 2,  by[i], 22, 16, "-",
                               COL_BTN, LV_OPA_COVER, COL_BTN, COL_SUB, f12, 3);
        val[i] = mk_label(left, 26, by[i] - 1, "", COL_INK, f16);
        lv_obj_set_width(val[i], 38);
        lv_obj_set_style_text_align(val[i], LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_t *inc = mk_btn(left, 66, by[i], 22, 16, "+",
                               COL_BTN, LV_OPA_COVER, COL_BTN, COL_SUB, f12, 3);
        lv_obj_add_event_cb(dec, ev_dec, LV_EVENT_CLICKED, (void *)(intptr_t)i);
        lv_obj_add_event_cb(inc, ev_inc, LV_EVENT_CLICKED, (void *)(intptr_t)i);
    }

    /* ===== 波形区 (90,25 229x126) ===== */
    mk_panel(scr, 90, 25, 229, 126, COL_WAVE);

    chart = lv_chart_create(lv_scr_act());
    lv_obj_set_pos(chart, 90, 25);
    lv_obj_set_size(chart, 229, 126);
    lv_obj_set_style_bg_opa(chart, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(chart, 0, 0);
    lv_obj_set_style_pad_all(chart, 0, 0);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(chart, WAVE_POINTS);
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, -50, 250);
    /* 大格网格: 水平 5 条 (每 50rpm), 垂直 9 条 (每 1s) */
    lv_chart_set_div_line_count(chart, 5, 9);
    lv_obj_set_style_line_color(chart, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_line_opa(chart, LV_OPA_10, 0);
    lv_obj_set_style_line_width(chart, 1, 0);
    lv_chart_set_axis_tick(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 0, 0, 0, false, 0);
    lv_chart_set_axis_tick(chart, LV_CHART_AXIS_PRIMARY_X, 0, 0, 0, 0, false, 0);
    /* 系列线条: 目标细灰蓝, 实际蓝 */
    ser_target = lv_chart_add_series(chart, COL_TARGET, LV_CHART_AXIS_PRIMARY_Y);
    ser_actual = lv_chart_add_series(chart, COL_ACCENT, LV_CHART_AXIS_PRIMARY_Y);
    lv_obj_set_style_line_width(chart, 1, LV_PART_ITEMS);
    lv_obj_set_style_size(chart, 0, LV_PART_ITEMS);

    /* 图例: 小色块 + 文字 */
    lv_obj_t *sw1 = lv_obj_create(lv_scr_act());
    lv_obj_set_pos(sw1, 100, 32);
    lv_obj_set_size(sw1, 7, 7);
    lv_obj_set_style_bg_color(sw1, COL_TARGET, 0);
    lv_obj_set_style_border_width(sw1, 0, 0);
    lv_obj_set_style_radius(sw1, 2, 0);
    lv_obj_set_style_pad_all(sw1, 0, 0);
    mk_label(lv_scr_act(), 111, 30, "Target", COL_SUB, f12);

    lv_obj_t *sw2 = lv_obj_create(lv_scr_act());
    lv_obj_set_pos(sw2, 160, 32);
    lv_obj_set_size(sw2, 7, 7);
    lv_obj_set_style_bg_color(sw2, COL_ACCENT, 0);
    lv_obj_set_style_border_width(sw2, 0, 0);
    lv_obj_set_style_radius(sw2, 2, 0);
    lv_obj_set_style_pad_all(sw2, 0, 0);
    mk_label(lv_scr_act(), 171, 30, "Actual", COL_SUB, f12);

    mk_label(lv_scr_act(), 268, 31, "1 s/div", lv_color_hex(0x4A535F), f12);

    /* ===== 读数行 (90,152 229x34) ===== */
    lv_obj_t *robox = mk_panel(scr, 90, 152, 229, 34, COL_SCR);
    mk_sep(robox, 0, 0, 229, 1);

    const char *roh[4] = {"RPM", "PWM", "OVS", "ERR"};
    int rox[4] = {0, 57, 114, 171};
    for (int i = 0; i < 4; i++) {
        lv_obj_t *rh = mk_label(robox, rox[i], 4, roh[i], COL_MUTED, f12);
        lv_obj_set_width(rh, 57);
        lv_obj_set_style_text_align(rh, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_t *rv = mk_label(robox, rox[i], 18, "",
                                i == 2 ? COL_DANGER : COL_INK, f12);
        lv_obj_set_width(rv, 57);
        lv_obj_set_style_text_align(rv, LV_TEXT_ALIGN_CENTER, 0);
        ro[i] = rv;
    }
    /* 列分隔线 */
    for (int i = 0; i < 3; i++) {
        mk_sep(robox, rox[i + 1], 0, 1, 34);
    }

    /* ===== 底栏 (0,186 320x54) ===== */
    mk_panel(scr, 0, 186, SCR_W, 54, COL_TOPBAR);
    mk_sep(scr, 0, 186, SCR_W, 1);

    lv_obj_t *b_start = mk_btn(lv_scr_act(), 8, 193, 72, 40, "START",
                               COL_ACCENT_DK, LV_OPA_COVER, COL_ACCENT_DK,
                               lv_color_hex(0xFFFFFF), f14, 3);
    lv_obj_t *b_stop  = mk_btn(lv_scr_act(), 88, 193, 72, 40, "STOP",
                               COL_TOPBAR, LV_OPA_TRANSP, COL_DANGER,
                               COL_DANGER, f14, 3);
    lv_obj_t *b_step  = mk_btn(lv_scr_act(), 168, 193, 72, 40, "STEP",
                               COL_TOPBAR, LV_OPA_TRANSP, COL_SUB,
                               COL_SUB, f14, 3);
    lv_obj_t *b_reset = mk_btn(lv_scr_act(), 248, 193, 64, 40, "RESET",
                               COL_TOPBAR, LV_OPA_TRANSP, COL_SUB,
                               COL_SUB, f14, 3);
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
