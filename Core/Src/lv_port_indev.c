/**
 * @file    lv_port_indev.c
 * @brief   LVGL 输入端口 - XPT2046 触摸 (软件 SPI, 坐标已映射到 320x240)
 *          触摸硬件读取由 pid_screen.c 提供 PID_Screen_TouchGetPoint()
 */
#include "lvgl.h"
#include "lv_port.h"
#include "pid_screen.h"

static lv_indev_drv_t indev_drv;

static void indev_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    uint16_t x = 0, y = 0;
    (void)drv;

    if (PID_Screen_TouchGetPoint(&x, &y)) {
        data->point.x = x;
        data->point.y = y;
        data->state   = LV_INDEV_STATE_PRESSED;
    } else {
        data->state   = LV_INDEV_STATE_RELEASED;
    }
}

void lv_port_indev_init(void)
{
    lv_indev_drv_init(&indev_drv);
    indev_drv.type    = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = indev_read_cb;
    lv_indev_drv_register(&indev_drv);
}
