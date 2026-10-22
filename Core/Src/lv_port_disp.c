/**
 * @file    lv_port_disp.c
 * @brief   LVGL 显示端口 - SSD1963 (320x240, RGB565, FSMC 接口)
 *          flush 回调: 通过 lcd_set_window + 写 GRAM 把 LVGL 缓冲区的
 *          局部区域像素写到屏上。
 */
#include "lvgl.h"
#include "lcd.h"
#include "lv_port.h"

/* 部分绘制缓冲: 320 * 20 行 = 12.8KB (屏为 320x240, RGB565) */
#define DISP_HOR_RES    320
#define DISP_VER_RES    240
#define DISP_BUF_ROWS   20
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf_1[DISP_HOR_RES * DISP_BUF_ROWS];
static lv_disp_drv_t  disp_drv;

static void disp_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = area->x2 - area->x1 + 1;
    uint32_t h = area->y2 - area->y1 + 1;

    /* 设置写入窗口并准备写 GRAM */
    lcd_set_window((uint16_t)area->x1, (uint16_t)area->y1, (uint16_t)w, (uint16_t)h);
    lcd_write_ram_prepare();

    /* 逐像素写入 RGB565 (LVGL 原生 16bit 颜色与 LCD 一致) */
    uint32_t cnt = w * h;
    for (uint32_t i = 0; i < cnt; i++) {
        LCD->LCD_RAM = color_p[i].full;
    }

    lv_disp_flush_ready(drv);
}

void lv_port_disp_init(void)
{
    lv_disp_draw_buf_init(&draw_buf, buf_1, NULL, DISP_HOR_RES * DISP_BUF_ROWS);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res   = DISP_HOR_RES;
    disp_drv.ver_res   = DISP_VER_RES;
    disp_drv.flush_cb  = disp_flush;
    disp_drv.draw_buf  = &draw_buf;
    lv_disp_drv_register(&disp_drv);
}
