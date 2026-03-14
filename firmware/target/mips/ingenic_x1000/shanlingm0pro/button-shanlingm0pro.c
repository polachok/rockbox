/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2021 by Aidan MacDonald
 * Copyright (C) 2021 by Dana Conrad
 * Copyright (C) 2026 by Alexander Polakov
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/

#include "button.h"
#include "touchscreen.h"
#include "hynitron-cst.h"
#include "kernel.h"
#include "backlight.h"
#include "powermgmt.h"
#include "gpio-x1000.h"
#include "irq-x1000.h"
#include "i2c-x1000.h"
#include <stdbool.h>

#ifndef BOOTLOADER
# include "lcd.h"
# include "font.h"
#endif

/* Volume wheel rotation */
static volatile int wheel_pos = 0;

void button_init_device(void)
{
    /* Setup interrupts for the volume wheel */
    gpio_set_function(GPIO_WHEEL1, GPIOF_IRQ_EDGE(0));
    gpio_set_function(GPIO_WHEEL2, GPIOF_IRQ_EDGE(0));
    gpio_flip_edge_irq(GPIO_WHEEL1);
    gpio_flip_edge_irq(GPIO_WHEEL2);
    gpio_enable_irq(GPIO_WHEEL1);
    gpio_enable_irq(GPIO_WHEEL2);

    /* Init Hynitron touchscreen driver */
    i2c_x1000_set_freq(HYNITRON_BUS, I2C_FREQ_400K);
    hynitron_init();

    /* Reset touch controller */
    gpio_set_level(GPIO_HYNITRON_RESET, 0);
    mdelay(5);
    gpio_set_level(GPIO_HYNITRON_RESET, 1);
    mdelay(50);

    /* Jack detect GPIOs — PB6 = single-ended, PB7 = balanced (unused) */
    gpio_set_function(GPIO_PB(6), GPIOF_INPUT);
    gpio_set_function(GPIO_PB(7), GPIOF_INPUT);
    gpio_set_pull(GPIO_PB(6), 1);
    gpio_set_pull(GPIO_PB(7), 1);

    /* Enable Hynitron interrupt */
    system_set_irq_handler(GPIO_TO_IRQ(GPIO_HYNITRON_INTERRUPT),
                           hynitron_irq_handler);
    gpio_set_function(GPIO_HYNITRON_INTERRUPT, GPIOF_IRQ_EDGE(0));
    gpio_enable_irq(GPIO_HYNITRON_INTERRUPT);
}

int button_read_device(int* data)
{
    const struct hynitron_point* point;
    int r = 0;

    /* Read power button GPIO — active low */
    uint32_t b = REG_GPIO_PIN(GPIO_B);
    if((b & (1 << 31)) == 0) r |= BUTTON_POWER;

    /* Check the wheel */
    int wheel_btn = 0;
    int whpos = wheel_pos;
    if(whpos > 3)
        wheel_btn = BUTTON_VOL_DOWN;
    else if(whpos < -3)
        wheel_btn = BUTTON_VOL_UP;

    if(wheel_btn) {
        wheel_pos = 0;

        /* Post the event (rapid motion is more reliable this way) */
        button_queue_post(wheel_btn, 0);
        button_queue_post(wheel_btn|BUTTON_REL, 0);

        /* Poke the backlight */
        backlight_on();
        reset_poweroff_timer();
    }

    /* Map gestures to buttons (L/R swapped for X inversion) */
    int gesture = hynitron_state.gesture;
    static int last_gesture = HYNITRON_GESTURE_NONE;
    if(gesture != HYNITRON_GESTURE_NONE && gesture != last_gesture) {
        int gbtn = 0;
        switch(gesture) {
            case HYNITRON_GESTURE_SWIPE_LEFT:  gbtn = BUTTON_MIDRIGHT; break;
            case HYNITRON_GESTURE_SWIPE_RIGHT: gbtn = BUTTON_MIDLEFT;  break;
            case HYNITRON_GESTURE_SWIPE_UP:    gbtn = BUTTON_TOPMIDDLE; break;
            case HYNITRON_GESTURE_SWIPE_DOWN:
                gbtn = BUTTON_BOTTOMMIDDLE; break;
        }
        if(gbtn) {
            button_queue_post(gbtn, 0);
            button_queue_post(gbtn|BUTTON_REL, 0);
            backlight_on();
            reset_poweroff_timer();
        }
    }
    last_gesture = gesture;

    /* Touch point handling — skip if a swipe gesture is active */
    if(gesture == HYNITRON_GESTURE_NONE) {
        point = &hynitron_state.points[0];
        int tx = (LCD_WIDTH - 1) - point->pos_x;
        int ty = (LCD_HEIGHT - 1) - point->pos_y;
        int t = touchscreen_to_pixels(tx, ty, data);
        if(point->event == HYNITRON_EVT_PRESS ||
           point->event == HYNITRON_EVT_CONTACT)
            r |= t;
    } else {
        for(int i = 0; i < hynitron_state.nr_points; ++i) {
            point = &hynitron_state.points[i];
            int tx = (LCD_WIDTH - 1) - point->pos_x;
            int ty = (LCD_HEIGHT - 1) - point->pos_y;
            if(point->event == HYNITRON_EVT_PRESS ||
               point->event == HYNITRON_EVT_CONTACT)
                r |= touchscreen_to_pixels(tx, ty, data);
        }
    }

    return r;
}

void touchscreen_enable_device(bool en)
{
    hynitron_enable(en);
}

bool headphones_inserted(void)
{
    /* PB6 active low = jack inserted. Requires ALDO4 enabled. */
    return !gpio_get_level(GPIO_PB(6));
}

static void handle_wheel_irq(void)
{
    /* Wheel quadrature decoding — same as Q1/Eros Q */
    static const int delta[16] = { 0, -1,  1,  0,
                                   1,  0,  0, -1,
                                  -1,  0,  0,  1,
                                   0,  1, -1,  0 };
    static uint32_t state = 0;
    state <<= 2;
    state |= (REG_GPIO_PIN(GPIO_D) >> 2) & 3;
    state &= 0xf;

    wheel_pos += delta[state];
}

void GPIOD02(void)
{
    handle_wheel_irq();
    gpio_flip_edge_irq(GPIO_WHEEL1);
}

void GPIOD03(void)
{
    handle_wheel_irq();
    gpio_flip_edge_irq(GPIO_WHEEL2);
}

#ifndef BOOTLOADER
static int getbtn(void)
{
    int btn;
    do {
        btn = button_get_w_tmo(1);
    } while(btn & (BUTTON_REL|BUTTON_REPEAT));
    return btn;
}

bool dbg_shanlingm0pro_touchscreen(void)
{
    const int pad_w = LCD_WIDTH;
    const int pad_h = LCD_HEIGHT;
    const int box_h = pad_h - SYSFONT_HEIGHT*5;
    const int box_w = pad_w * box_h / pad_h;
    const int box_x = (LCD_WIDTH - box_w) / 2;
    const int box_y = SYSFONT_HEIGHT * 9 / 2;

    bool draw_border = true;

    do {
        int line = 0;
        lcd_clear_display();
        lcd_putsf(0, line++, "nr_points: %d  gesture: %d",
                  hynitron_state.nr_points, hynitron_state.gesture);

        if(draw_border)
            lcd_drawrect(box_x, box_y, box_w, box_h);

        for(int i = 0; i < hynitron_state.nr_points; ++i) {
            const struct hynitron_point* point = &hynitron_state.points[i];
            int cx = (LCD_WIDTH - 1) - point->pos_x;
            int cy = (LCD_HEIGHT - 1) - point->pos_y;
            lcd_putsf(0, line++, "pt%d  id:%d  pos: %d,%d",
                      i, point->touch_id, cx, cy);

            int tx = box_x + cx * box_w / pad_w;
            int ty = box_y + cy * box_h / pad_h;
            lcd_hline(tx-2, tx+2, ty);
            lcd_vline(tx, ty-2, ty+2);
        }

        lcd_update();
    } while(getbtn() != BUTTON_POWER);
    return false;
}
#endif
