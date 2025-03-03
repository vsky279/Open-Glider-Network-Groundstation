/*
 * OLEDHelper.h
 * Copyright (C) 2019-2021 Linar Yusupov
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef OLEDHELPER_H
#define OLEDHELPER_H

#include "SSD1306Wire.h"

#define SSD1306_OLED_I2C_ADDR   0x3C

void OLED_setup(void);

void OLED_write(const char *, short, short, bool);

void OLED_clear(void);

void OLED_bar(uint8_t, uint8_t);
void OLED_info(bool);

void OLED_update(void);

void OLED_disable(void);
void OLED_enable(void);

void OLED_draw_Bitmap(int16_t, int16_t, uint8_t, bool);

extern bool OLED_blank;

#endif /* OLEDHELPER_H */
