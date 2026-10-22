/**
 * @file    lv_port.h
 * @brief   LVGL 显示与输入端口对外接口
 */
#ifndef __LV_PORT_H
#define __LV_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

void lv_port_disp_init(void);    /* 注册 LCD 显示驱动 (SSD1963 via FSMC) */
void lv_port_indev_init(void);   /* 注册 XPT2046 触摸输入驱动 */

#ifdef __cplusplus
}
#endif

#endif /* __LV_PORT_H */
