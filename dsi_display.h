/*
 * This file is part of AtomGL.
 *
 * Copyright 2024 Paul Guyot <pguyot@kallisys.net>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _DSI_DISPLAY_H_
#define _DSI_DISPLAY_H_

#include <esp_lcd_mipi_dsi.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_ldo_regulator.h>

#include <stdbool.h>

#include <globalcontext.h>

struct DSIDisplay
{
    esp_lcd_dsi_bus_handle_t dsi_bus;
    esp_lcd_panel_io_handle_t io_handle;
    esp_lcd_panel_handle_t panel_handle;
    esp_ldo_channel_handle_t ldo_mipi_phy;
};

struct DSIDisplayConfig
{
    int num_lanes;
    int lane_bit_rate_mbps;
    int panel_width;
    int panel_height;
    int hporch_front;
    int hporch_back;
    int hsync;
    int vporch_front;
    int vporch_back;
    int vsync;
};

bool dsi_display_init(struct DSIDisplay *dsi_disp, struct DSIDisplayConfig *dsi_config);
bool dsi_display_write_command(struct DSIDisplay *dsi_disp, uint8_t command, const uint8_t *data, size_t data_len);
bool dsi_display_draw_bitmap(struct DSIDisplay *dsi_disp, int x_start, int y_start, int x_end, int y_end, const void *color_data);
void dsi_display_init_config(struct DSIDisplayConfig *dsi_config);
bool dsi_display_parse_config(struct DSIDisplayConfig *dsi_config, term opts, GlobalContext *global);

#endif
